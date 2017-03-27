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
#include "SnapdragonVislamManager.hpp"
#include "SnapdragonDebugPrint.h"

Snapdragon::VislamManager::VislamManager() {
  cam_man_ptr_ = nullptr;
  imu_man_ptr_ = nullptr;
  vislam_ptr_ = nullptr;
  initialized_ = false;
  image_buffer_size_bytes_ = 0;
  image_buffer_ = nullptr;
}

Snapdragon::VislamManager::~VislamManager() {
  CleanUp();
}

int32_t Snapdragon::VislamManager::CleanUp() {
  // stop the imu api first.
  if( imu_man_ptr_ != nullptr ) {
    imu_man_ptr_->RemoveHandler( this );
    imu_man_ptr_->Terminate();
    delete imu_man_ptr_;
    imu_man_ptr_ = nullptr;
  }

  //stop the camera.
  if( cam_man_ptr_ != nullptr ) {
    WARN_PRINT( "Stopping Camera...." );
    cam_man_ptr_->Terminate();
    WARN_PRINT( "Deleting Camera Pointer" );
    delete cam_man_ptr_;
    cam_man_ptr_ = nullptr;
  }

  //stop the vislam engine.
  if( vislam_ptr_ != nullptr ) {
    mvVISLAM_Deinitialize( vislam_ptr_ );
    vislam_ptr_ = nullptr;
  } 

  if( image_buffer_ != nullptr ) {
    delete[] image_buffer_;
    image_buffer_ = nullptr;
    image_buffer_size_bytes_ = 0;
  }
  return 0;  
}

int32_t Snapdragon::VislamManager::Initialize
( 
  const Snapdragon::CameraParameters& cam_params,
  const Snapdragon::VislamManager::InitParams& vislam_params
) {
  cam_params_ = cam_params;
  vislam_params_ = vislam_params;
  int32_t rc = 0;
  if( rc == 0 ) { //initialize the camera module.
    cam_man_ptr_ = new Snapdragon::CameraManager( &cam_params_ ) ;
    if( cam_man_ptr_ != nullptr ) {
      rc = cam_man_ptr_->Initialize();
    }
    else {
      rc = -1;
    }
  }

  if( rc == 0 ) { //initialize the Imu Manager.
    imu_man_ptr_ = new Snapdragon::ImuManager();
    if( imu_man_ptr_ != nullptr ) {
      rc = imu_man_ptr_->Initialize();
      imu_man_ptr_->AddHandler( this );
    }
    else {
      rc = -1;
    }
  }

  //now intialize the VISLAM module.
  if( rc == 0 ) {
    vislam_ptr_ = mvVISLAM_Initialize
    (
      &(cam_params_.mv_camera_config),
      vislam_params_.tbc, vislam_params_.ombc, vislam_params_.delta,
      vislam_params_.std0Tbc, vislam_params_.std0Ombc, vislam_params_.std0Delta,
      vislam_params_.accelMeasRange, vislam_params_.gyroMeasRange,
      vislam_params_.stdAccelMeasNoise, vislam_params_.stdGyroMeasNoise,
      vislam_params_.stdCamNoise, vislam_params_.minStdPixelNoise, vislam_params_.failHighPixelNoisePoints,
      vislam_params_.logDepthBootstrap, vislam_params_.useLogCameraHeight, vislam_params_.logCameraHeightBootstrap,
      vislam_params_.noInitWhenMoving 
    );
    if( vislam_ptr_ == nullptr ) {
      rc = -1;
    }
  }

  if( rc != 0 ) {
    ERROR_PRINT( "Error initializing the Vislam Manager." );
    CleanUp();
  }
  else {
    initialized_ = true;
  }
  return 0;
}

int32_t Snapdragon::VislamManager::Start() {
  int32_t rc = 0;
  if( initialized_ ) {
    //start the camera
    rc =  imu_man_ptr_->Start();
    rc |= cam_man_ptr_->Start();

    //wait till we get the first frame.
    while( cam_man_ptr_->GetLatestFrameId()  < 10 ) {
      std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
    }
    // allocate the image buffer here.
    image_buffer_size_bytes_ = cam_man_ptr_->GetImageSize();
    INFO_PRINT( "image Size: %d frameId: %lld", cam_man_ptr_->GetImageSize(), cam_man_ptr_->GetLatestFrameId() );
    image_buffer_ = new uint8_t[ image_buffer_size_bytes_ ];
  }
  else {
    ERROR_PRINT( "Calling Start without calling intialize" );
    rc = -1;
  }
  return rc;
}

int32_t Snapdragon::VislamManager::Stop() {
  CleanUp();
  return 0;
}

int32_t Snapdragon::VislamManager::GetPointCloud( mvVISLAMMapPoint* points, uint32_t max_points ) {
  std::lock_guard<std::mutex> lock( sync_mutex_ );
  return mvVISLAM_GetPointCloud( vislam_ptr_, points, max_points );
}

bool Snapdragon::VislamManager::HasUpdatedPointCloud() {
  std::lock_guard<std::mutex> lock( sync_mutex_);
  int32_t mv_ret = mvVISLAM_HasUpdatedPointCloud( vislam_ptr_ );
  return (mv_ret != 0 );
}

int32_t Snapdragon::VislamManager::Imu_IEventListener_ProcessSamples( sensor_imu* imu_samples, uint32_t sample_count ) {
  int32_t rc = 0;
  for (int ii = 0; ii < sample_count; ++ii)
  {
    int64_t current_timestamp_ns = (int64_t)imu_samples[ii].timestamp_in_us * 1000;

    float delta = 0.f;
    static int64_t last_timestamp = 0;
    if (last_timestamp != 0)
    {
      delta = (current_timestamp_ns - last_timestamp)*1e-6;
      const float imu_sample_dt_reasonable_threshold_ms = 2.5;
      if (delta > imu_sample_dt_reasonable_threshold_ms)
      {
        if (cam_params_.verbose)
        {
          WARN_PRINT("IMU sample dt > %f ms -- %f ms",
              imu_sample_dt_reasonable_threshold_ms,
              delta);
        }
      }
    }
    last_timestamp = current_timestamp_ns;

    float lin_acc[3], ang_vel[3];
    const float kNormG = 9.80665f;
    lin_acc[0] = imu_samples[ii].linear_acceleration[0] * kNormG;
    lin_acc[1] = imu_samples[ii].linear_acceleration[1] * kNormG;
    lin_acc[2] = imu_samples[ii].linear_acceleration[2] * kNormG;
    ang_vel[0] = imu_samples[ii].angular_velocity[0];
    ang_vel[1] = imu_samples[ii].angular_velocity[1];
    ang_vel[2] = imu_samples[ii].angular_velocity[2];

    static uint32_t sequence_number_last = 0;
    int num_dropped_samples = 0;
    if (sequence_number_last != 0)
    {
      // The diff should be 1, anything greater means we dropped samples
      num_dropped_samples = imu_samples[ii].sequence_number
        - sequence_number_last - 1;
      if (num_dropped_samples > 0)
      {
        if (cam_params_.verbose)
        {
          WARN_PRINT("Current IMU sample = %u, last IMU sample = %u",
              imu_samples[ii].sequence_number, sequence_number_last);
        }
      }
    }
    sequence_number_last = imu_samples[ii].sequence_number;

    {
      std::lock_guard<std::mutex> lock( sync_mutex_ );
      mvVISLAM_AddAccel(vislam_ptr_, current_timestamp_ns,
          lin_acc[0], lin_acc[1], lin_acc[2]);
      mvVISLAM_AddGyro(vislam_ptr_, current_timestamp_ns,
          ang_vel[0], ang_vel[1], ang_vel[2]);
    }
  }  
  return rc;
}

int32_t Snapdragon::VislamManager::GetPose( mvVISLAMPose& pose, int64_t& pose_frame_id, uint64_t& timestamp_ns ) {
  int32_t rc = 0;
  if( !initialized_ ) {
    WARN_PRINT( "VislamManager not initialize" );
    return -1;
  }

  // first set the Image from the camera.
  // Next add it to the mvVISLAM 
  // Then call the API to get the Pose.
  int64_t frame_id;
  uint32_t used = 0;
  uint64_t frame_ts_ns;
  static int64_t prev_frame_id = 0;
  rc = cam_man_ptr_->GetNextImageData( &frame_id, &frame_ts_ns, image_buffer_, image_buffer_size_bytes_ , &used );

  if( rc != 0 ) {
    WARN_PRINT( "Error Getting the image from camera" );
  }
  else {
    if( prev_frame_id != 0 && (prev_frame_id + 1 != frame_id ) ) {
      WARN_PRINT( "Warning: Missed/Dropped Camera Frames.  recvd(%lld) expected(%lld)", frame_id, (prev_frame_id+1) );
    }

    // adjust the frame-timestamp for VISLAM at it needs the time at the center of the exposure and not the sof.
    // Correction from exposure time
    float correction = 1e3 * (cam_man_ptr_->GetExposureTimeUs()/2.f);
    float modified_timestamp = frame_ts_ns - static_cast<int64_t>(correction);
    {
      std::lock_guard<std::mutex> lock( sync_mutex_ );
      mvVISLAM_AddImage(vislam_ptr_, modified_timestamp, image_buffer_, false);
      pose = mvVISLAM_GetPose(vislam_ptr_);
      pose_frame_id = frame_id;
      timestamp_ns = static_cast<uint64_t>(modified_timestamp);
    }
  }
  return rc;
}
