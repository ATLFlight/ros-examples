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
#include "SnapdragonCameraTypes.hpp"
#include "SnapdragonCameraUtil.hpp"
#include <iostream>
#include <chrono>
#include <thread>
int main( int argc, char** argv ) {
  Snapdragon::CameraParameters param;
  param.enable_cpa = 1;
  param.camera_config.fps = 30;

  param.camera_config.cam_type = Snapdragon::CameraType::OPTIC_FLOW;

  Snapdragon::CameraManager cam_man( &param );

  //Initialize Test.
  if( cam_man.Initialize() == 0 ) {
    std::cout << "PASS: CameraManager::Initialize()" << std::endl;
  }
  else {
    std::cout << "FAIL: CameraManager::Initialize()" << std::endl;
  }

  //start the camera test.
  if( cam_man.Start() == 0 ) {
    std::cout << "PASS: CameraManager::Start()" << std::endl;
  }
  else {
    std::cout << "FAIL: CameraManager::Start()" << std::endl;
  }

  //Test to see if we are getting the frames.
  int32_t max_iter = 100;
  int32_t count = 0;
  uint32_t sleep_interval_ms = (uint32_t)(1000.0/(float)param.camera_config.fps);
  int64_t last_frame_id = -1;
  int64_t current_frame_id;
  bool pass = true;
  uint32_t image_size;
  while( count < max_iter ) {
    current_frame_id = cam_man.GetLatestFrameId();
    image_size = cam_man.GetImageSize();
    std::cout << "Latest FrameId: " << current_frame_id
              << " Image Size: " << image_size
              << " Fps: " << cam_man.GetAvgFps()
              << " Expsure: " << cam_man.GetExposureTimeUs()
              << " Oldest FrameId: " << cam_man.GetOldestFrameId()
              << std::endl;
    if( last_frame_id != -1 ) {
      if( last_frame_id + 1 != current_frame_id ) {
        std::cout << "FAIL: Frame Id not matching: last_frame: " << last_frame_id 
                  << " recieved Frame: " 
                  << current_frame_id << std::endl;
        pass = false;
      }
    }
    last_frame_id = current_frame_id;
    std::this_thread::sleep_for( std::chrono::milliseconds( sleep_interval_ms ) );
    count++;
  }

  if( pass ) {
    std::cout << "PASS: Getting Frames From OPTIC_FLOW Camera" << std::endl;
  }

  pass = true;
  count = 0;
  uint8_t* buffer = new uint8_t[ image_size ];
  int32_t rc;
  int64_t frame_id;
  int64_t prev_frame_id = -1;
  uint64_t prev_ts_ns = -1;
  uint64_t timestamp_ns;
  uint32_t used = 0;
  while( count < max_iter ) {
    rc = cam_man.GetNextImageData( &frame_id, &timestamp_ns, buffer, image_size, &used );
    if( rc != 0 ) {
      std::cout << "FAIL: GetImageData() Unable to get Frame date for id: " 
                << " rc: " << rc
                << std::endl;
      pass = false;
    }
    else {
      std::cout << "PASS: GetImageData() " 
                << " frameId: " << frame_id
                << " ts: " << timestamp_ns
                << " tsdelta: " << timestamp_ns - prev_ts_ns 
                << " used: " << used 
                << std::endl;
      if( prev_frame_id != -1 && prev_frame_id + 1 != frame_id ) {
        std::cout << "WARN: Missed Frames: "
                  << " Expected: " << prev_frame_id + 1 
                  << " got: " << frame_id
                  << std::endl; 
      }
      prev_frame_id = frame_id;
      prev_ts_ns = timestamp_ns;
    }

    if( used != image_size ) {
      std::cout << "FAIL: GetImageData() : Buffer size returned do not match: size: " 
                << image_size << " got: " 
                << used 
                << std::endl;
      pass = false;
     }
     count++;
  }

  if( !pass ) {
    std::cout << "FAIL: GetImageData() : Got Error reading Image data" << std::endl;
  }


  //stop the camera
  if( cam_man.Terminate() != 0 ) {
    std::cout << "FAIL: Error calling Stop to the manager" << std::endl;
  }
  else {
    std::cout << "PASS: Camera::Stop API Success" << std::endl;
  }

  //test to make sure the condition variable is working fine.
  // count = 0;
  // while( count < max_iter ) {
  //   std::cout << "Case:(When Camera Is not running) GetImageData: Iter Count: " << count << std::endl;
  //   rc = cam_man.GetNextImageData( &frame_id, &timestamp_ns, buffer, image_size, &used );
  //   if( rc == 0 ) {
  //     std::cout << "FAIL: Expected to not return with success" 
  //               << " FrameId: " << frame_id 
  //               << " TimeStamp: " << timestamp_ns 
  //               << std::endl;
  //   }
  //   else {
  //     std::cout << "PASS: Expected to not return but fail code is ok." << std::endl;
  //   }
  //   count++;
  // }

  return 0;
}
