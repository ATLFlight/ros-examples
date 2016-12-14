/****************************************************************************
 *   Copyright (c) 2016 Ramakrishna Kintada. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "SnapdragonCameraManager.hpp"
#include "SnapdragonCameraUtil.hpp"
#include "SnapdragonDebugPrint.h"

#include <fstream>
#include <iostream>
#include <string>
#include <chrono>

Snapdragon::CameraManager::CameraManager( Snapdragon::CameraParameters* params_ptr) {
  initialized_ = false;
  running_     = false;

  snap_camera_param_ptr_ = params_ptr;
  camera_config_ptr_     = &params_ptr->camera_config;

  camera_ptr_           = NULL;
  preview_size_.width   = camera_config_ptr_->pixel_width;
  preview_size_.height  = camera_config_ptr_->pixel_height;
  image_size_bytes_     = camera_config_ptr_->pixel_height * camera_config_ptr_->memory_stride *1.5;
  exposure_setting_     = camera_config_ptr_->min_exposure
                          + camera_config_ptr_->exposure
                          * (camera_config_ptr_->max_exposure - camera_config_ptr_->min_exposure);

  gain_setting_         = camera_config_ptr_->min_gain + camera_config_ptr_->gain
                          * (camera_config_ptr_->max_gain - camera_config_ptr_->min_gain);

  exposure_target_      = exposure_setting_;
  gain_target_          = gain_setting_;
  fps_avg_              = 0;
  timestamp_last_nsecs_ = 0;

  next_frame_id_ = 0;
  frame_q_write_index_ = 0;
  frame_q_read_index_ = 0;

  for (unsigned int ii = 0; ii < camera_config_ptr_->num_image_buffers; ++ii)
  {
    frame_queue_.push_back( std::make_pair( -1, nullptr ) );
  }

  mvCPA_ptr_ = NULL;
}

int32_t Snapdragon::CameraManager::Initialize(){
  if (!initialized_) {
    int32_t cam_id;
    if( Snapdragon::FindCamera( camera_config_ptr_->cam_type, &cam_id ) != 0 ) {
      ERROR_PRINT( "Cannot Find Camera Id for Type: %d", camera_config_ptr_->cam_type );
      return -1;
    }
    int ret = camera::ICameraDevice::createInstance(cam_id, &camera_ptr_);
    if (ret != 0) {
      ERROR_PRINT("Could not open camera %d", cam_id);
      return ret;
    }
    else {
      INFO_PRINT("Opened camera %d Type: %d", cam_id, camera_config_ptr_->cam_type );
    }

    camera_ptr_->addListener(this);

    ret = params_.init(camera_ptr_);
    if (ret != 0) {
      ERROR_PRINT("Failed to initialize camera parameters.");
      camera::ICameraDevice::deleteInstance(&camera_ptr_);
      return ret;
    }

    // Check desired FPS against supported FPS values
    std::vector<camera::Range> preview_fps_ranges = params_.getSupportedPreviewFpsRanges();
    int fps_index = -1;
    for (unsigned int ii = 0; ii < preview_fps_ranges.size(); ++ii) {
      //std::cout << ii << ": [ " << preview_fps_ranges[ii].min << ", "
      //  << preview_fps_ranges[ii].max << " ] " << std::endl;
      if (preview_fps_ranges[ii].max / 1000 == camera_config_ptr_->fps) {
        fps_index = static_cast<int>(ii);
      }
    }

    {
      std::lock_guard<std::mutex> lock( frame_mutex_ );
      if (fps_index != -1)
      {
        INFO_PRINT("Setting FPS to %d", camera_config_ptr_->fps);
        params_.setPreviewFpsRange(preview_fps_ranges[fps_index]);
      }
      else
      {
        ERROR_PRINT("Invalid FPS value of %d. Using camera default.", camera_config_ptr_->fps);
      }

      params_.setPreviewSize(preview_size_);

      char exposure_string[6];
      sprintf(exposure_string,"%d", exposure_setting_);
      char gain_string[6];
      sprintf(gain_string,"%d", gain_setting_);

      params_.set("qc-exposure-manual",exposure_string);
      params_.set("qc-gain-manual",gain_string);

      // commit the camera parameters.
      if (params_.commit() != 0)
      {
        ERROR_PRINT("Error setting initial camera params.");
        return -1;
      }
    }

    snap_camera_param_ptr_->mv_cpa_config.startExposure = camera_config_ptr_->exposure;
    snap_camera_param_ptr_->mv_cpa_config.startGain = camera_config_ptr_->gain;

    if (snap_camera_param_ptr_->enable_cpa) {
      mvCPA_ptr_ = mvCPA_Initialize(&snap_camera_param_ptr_->mv_cpa_config);
    }
    initialized_ = true;
  }
  else {
    WARN_PRINT("CameraManager is already initialized.");
  }

  return 0;
}

int32_t Snapdragon::CameraManager::Terminate() {
  if( camera_ptr_ != nullptr ) {
    // remove this as a listener.
    camera_ptr_->removeListener( this );
    //stop the camera.
    Stop();
    //delete the camera ptr
    camera::ICameraDevice::deleteInstance(&camera_ptr_);
    camera_ptr_ = nullptr;
  }
  if( mvCPA_ptr_ != nullptr ) {
    mvCPA_Deinitialize(mvCPA_ptr_);
    mvCPA_ptr_ = nullptr;
  }
  return 0;
}

int32_t Snapdragon::CameraManager::Start() {
  if (initialized_) {
    if (!running_) {
      int ret = camera_ptr_->startPreview();
      if (ret == 0) {
        running_ = true;
      }
      else {
        return ret;
      }
    }
    else {
      WARN_PRINT( "CameraManager is already started." );
    }
  }
  else {
    ERROR_PRINT( "CameraManager has not yet been initialized."  ); 
    return -1;
  }

  return 0;
}

int32_t Snapdragon::CameraManager::Stop() {
  if( running_ ) {
    camera_ptr_->stopPreview();
  }
  running_ = false;

  std::lock_guard<std::mutex> lock( frame_mutex_ );
  //increment the frame id so that the condition variable is woken up.
  next_frame_id_++;
  frame_cv_.notify_all();

  //if there are any frames that are not read. free the frames
  while( frame_q_read_index_ != frame_q_write_index_ ) {
    if( frame_queue_[frame_q_read_index_].second != nullptr ) {
      frame_queue_[frame_q_read_index_].second->releaseRef();
      frame_queue_[frame_q_read_index_] = std::make_pair( -1, nullptr );
      frame_q_read_index_++;
      frame_q_read_index_ = ( frame_q_read_index_ >= camera_config_ptr_->num_image_buffers)?0:frame_q_read_index_;
    }
  }
  return 0;
}

void Snapdragon::CameraManager::UpdateGainAndExposure()
{
  mvCPA_AddFrame(mvCPA_ptr_, frame_queue_[frame_q_write_index_].second->data,
      camera_config_ptr_->pixel_width,
      camera_config_ptr_->pixel_height,
      camera_config_ptr_->memory_stride);

  float cpa_exposure, cpa_gain;
  mvCPA_GetValues(mvCPA_ptr_, &cpa_exposure, &cpa_gain);

  exposure_target_ = static_cast<int>(camera_config_ptr_->min_exposure + cpa_exposure
      * (camera_config_ptr_->max_exposure - camera_config_ptr_->min_exposure));
  gain_target_ = static_cast<int>(camera_config_ptr_->min_gain + cpa_gain
      * (camera_config_ptr_->max_gain - camera_config_ptr_->min_gain));

  static std::chrono::system_clock::time_point time_last;
  auto time_now = std::chrono::system_clock::now();
  auto diff = time_now - time_last;

  if( diff.count() > 0  
      ||
      abs(exposure_target_ - exposure_setting_) > camera_config_ptr_->cpa_exposure_change_threshold 
      ||
      abs(gain_target_ - gain_setting_) > camera_config_ptr_->cpa_gain_change_threshold
      &&
      (exposure_target_ != exposure_setting_ || gain_target_ != gain_setting_)
    ) {

        time_last = time_now;
        char exposure_string[6];
        sprintf(exposure_string,"%d",exposure_target_);
        char gain_string[6];
        sprintf(gain_string,"%d",gain_target_);
        
        params_.set("qc-exposure-manual",exposure_string);
        params_.set("qc-gain-manual",gain_string);
        params_.commit();

        // The exposure and gain take effect in the second frame
        // after the params are committed i.e. there is one frame in between
        // committing params and receiving a frame with the params applied
        static int exposure_temp = exposure_setting_;
        static int gain_temp = gain_setting_;
        exposure_setting_ = exposure_temp;
        gain_setting_ = gain_temp;
        exposure_temp = exposure_target_;
        gain_temp = gain_target_;
  }
}

void Snapdragon::CameraManager::onError()
{
  ERROR_PRINT("Camera error.");
  if( running_ ) {
    camera_ptr_->stopPreview();
  }
  running_ = false;
}

int32_t Snapdragon::CameraManager::GetNextImageData
(
  int64_t*  frame_id, 
  uint64_t* timestamp_ns,
  uint8_t* image_data, 
  uint32_t size, 
  uint32_t* used
) {
  
  std::unique_lock<std::mutex> lock( frame_mutex_ );
  int32_t ret_code = 0;

  // wait for new frame if queue is empty. 
  frame_cv_.wait( lock, [&]{ return (frame_q_read_index_ != frame_q_write_index_); } );

  if( !running_ ) {
    // the camera has stopped.  so return an error code.
    return  -1; 
  }

  if( frame_queue_[ frame_q_read_index_].first == -1 || frame_queue_[frame_q_read_index_].second == nullptr ) {
    //this should not happen incorrect frame id;
    ERROR_PRINT( "FrameId at read_index(%d) is incorrect: ", frame_q_read_index_ );
    return -2;
  }

  if( size < image_size_bytes_ ) {
    ERROR_PRINT( "Insuffient image buffer size: given: %d expected: %d",
      size, image_size_bytes_ );
    return -3;
  }

  *frame_id = frame_queue_[ frame_q_read_index_].first;
  *timestamp_ns = frame_queue_[ frame_q_read_index_].second->timeStamp;
  memcpy( image_data, reinterpret_cast<uint8_t*>( frame_queue_[frame_q_read_index_].second->data ), image_size_bytes_ );
  *used = image_size_bytes_;

  //invalidate the entry in the queue;
  frame_queue_[frame_q_read_index_].second->releaseRef();
  frame_queue_[frame_q_read_index_] = std::make_pair( -1, nullptr );

  // increment the read counter.
  frame_q_read_index_++;
  frame_q_read_index_ = ( frame_q_read_index_ >= camera_config_ptr_->num_image_buffers)?0:frame_q_read_index_;

  return 0;
}

void Snapdragon::CameraManager::onPreviewFrame(camera::ICameraFrame* frame)
{
  if (frame->timeStamp != timestamp_last_nsecs_)
  {
    //increment the frameid;
    int64_t old_frame_id = next_frame_id_;

    //set the image_size bytes information.
    image_size_bytes_ = frame->size;
    next_frame_id_++;

    //now add the frame to the queue.
    {
      std::lock_guard<std::mutex> lock( frame_mutex_ );

      //check if the queue already has an valid frame.  If yes, release the frame to be used 
      // by the camera pipeline.
      if( frame_queue_[ frame_q_write_index_ ].first != -1 && 
        frame_queue_[ frame_q_write_index_ ].second != nullptr ) {
        frame_queue_[ frame_q_write_index_ ].second->releaseRef();
      }

      // add the frame to the queue.
      frame->acquireRef();
      frame_queue_[ frame_q_write_index_ ] = std::make_pair( next_frame_id_, frame );

      //update the gain and exposure only one every 4 frames, as there a delay in taking the new
      // exposure parameters to take into effect.
      if( snap_camera_param_ptr_->enable_cpa 
          &&
          (next_frame_id_ > 0 && (next_frame_id_ % 4 ) == 0 ) ) {
        UpdateGainAndExposure();
      }

      frame_q_write_index_++;
      frame_q_write_index_ = (frame_q_write_index_ >= camera_config_ptr_->num_image_buffers )?0:frame_q_write_index_;

      if( frame_q_write_index_ == frame_q_read_index_ ) { 
        //queue is full, increment the read index;
        frame_q_read_index_++;
        frame_q_read_index_ = ( frame_q_read_index_ >= camera_config_ptr_->num_image_buffers)?0:frame_q_read_index_;
      }

      // notify the caller who is waiting on a frame.
      frame_cv_.notify_all();
    }

    // update the frame stats.
    uint64_t timestamp_nsecs = frame->timeStamp;
    uint32_t dt_nsecs = timestamp_nsecs - timestamp_last_nsecs_;
    timestamp_last_nsecs_ = timestamp_nsecs;
    fps_avg_ = ((fps_avg_ * old_frame_id) + (1e9 / dt_nsecs)) / (next_frame_id_);
  }
  else{
    WARN_PRINT( "Got duplicate image frame at timestamp: %lld frame_id: %llu", frame->timeStamp, next_frame_id_ );
  }
}

void Snapdragon::CameraManager::onVideoFrame(camera::ICameraFrame* frame)
{
  // Should never get this
  INFO_PRINT("Got video frame!");
}

