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
#pragma once
#include "SnapdragonCameraTypes.hpp"
#include "mvVISLAM.h"
#include "SnapdragonCameraManager.hpp"
#include "SnapdragonImuManager.hpp"
#include <mutex>

namespace Snapdragon {
  class VislamManager;
}

/**
 * Class to wrap the mvVISLAM SDK with Camera and IMU Samples.
 */
class Snapdragon::VislamManager : public Snapdragon::Imu_IEventListener {
public:

  /**
   * This structure defines all the parameters needed to initialize
   * the mvVISLAM_Initialize() method. Refer the API definition in 
   * mvVISLAM.h file to get the description for each field.
   **/
  typedef struct {
    float64_t tbc[3];
    float64_t ombc[3];
    float64_t delta;
    float64_t std0Tbc[3];
    float64_t std0Ombc[3];
    float64_t std0Delta;
    float64_t accelMeasRange;
    float64_t gyroMeasRange;
    float64_t stdAccelMeasNoise;
    float64_t stdGyroMeasNoise;
    float64_t stdCamNoise;
    float64_t minStdPixelNoise;
    bool      failHighPixelNoisePoints;
    float64_t logDepthBootstrap;
    bool      useLogCameraHeight;
    float64_t logCameraHeightBootstrap;
    bool      noInitWhenMoving;
  } InitParams;

  /**
   * Constructor
   **/
  VislamManager();

  /**
   * Initalizes the VISLAM Manager with Camera and VISLAM Parameters
   * @param params
   *  The structure that holds the VISLAM parameters.
   * @return 
   *  0 = success
   * otherwise = failure.
   **/
  int32_t Initialize
  ( 
    const Snapdragon::CameraParameters& cam_params, 
    const Snapdragon::VislamManager::InitParams& params
  );

  /**
   * Start the Camera and Imu modules for the VISLAM functionality.
   * @return 
   *   0 = success
   *  otherwise = failure;
   **/
  int32_t Start();

  /**
   * Stops the VISLAM engine by stoping the Camera and IMU modules.
   * @return 
   *   0 = success;
   * otherwise = failure.
   **/
  int32_t Stop();

  /**
   * This is wrapper for the MVSDK's vislam API.  Provides information
   * if there is an updated PointCloud.
   * @return bool
   *  true  = if it has updated point cloud
   *  false = otherwise
   **/
  bool HasUpdatedPointCloud();

  /**
   * MV SDK's wrapper to get the Pose Information.
   * @param pose
   *  The Pose as determined by VISLAM engine.
   * @frame_id
   *  The frame Id of the image that was processed.
   * @timestamp_ns
   *  The timestamp of the image frame that was used. The units are in nano-seconds;
   * @return int32_t
   *   0 = success
   *  otherwise = failure.
   **/
  int32_t GetPose( mvVISLAMPose& pose, int64_t& frame_id, uint64_t& timestamp_ns );

  /** 
   * MV SDK's wrapper to get the PointCloud data.
   * @param points
   *  The Point Cloud points
   * @param max_points
   *  The max points that can be returned.
   * @return int32_t
   *  The number of points filled into the array.
   **/
  int32_t GetPointCloud( mvVISLAMMapPoint* points, uint32_t max_points );

  /**
   * MV SDK's wrapper to reset the EKF filters.
   * @return int32_t
   *  0 = success
   * otherwise = false;
   **/
  int32_t Reset();

  /**
   * The IMU callback handler to add the accel/gyro data into VISLAM.
   * @param samples
   *  The imu Samples to be added to VISLAM.
   * @param count
   *  The number of samples in the buffer.
   * @return int32_t
   *  0 = success;
   * otherwise = false;
   **/
  int32_t Imu_IEventListener_ProcessSamples( sensor_imu* samples, uint32_t count );

  /**
   * Destructor
   */
  virtual ~VislamManager();

private:
  // utility methods 
  int32_t CleanUp();
  std::atomic<bool> initialized_;
  Snapdragon::CameraParameters          cam_params_;
  Snapdragon::VislamManager::InitParams vislam_params_;
  bool                          verbose_;
  Snapdragon::CameraManager*    cam_man_ptr_;
  Snapdragon::ImuManager*       imu_man_ptr_;
  mvVISLAM*                     vislam_ptr_;
  std::mutex                    sync_mutex_;
  uint8_t*                      image_buffer_;
  size_t                        image_buffer_size_bytes_;
};
