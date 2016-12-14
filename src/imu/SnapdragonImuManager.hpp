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
#include <stdint.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

#include <sensor-imu/sensor_imu_api.h>
#include <sensor-imu/sensor_datatypes.h>

namespace Snapdragon {
  class ImuManager;
  class Imu_IEventListener;
}

/**
 * class to invoke call backs for received IMU samples.
 **/
class Snapdragon::Imu_IEventListener {
public:
  /**
   * Callback function with ImuSmaples received.
   * @param samples
   *  The Imu Sample buffer;
   * @param sample_count
   *  The number of samples returned.
   * @return int32_t
   *  0 = success
   *  otherwise = failure.
   */
  virtual int32_t Imu_IEventListener_ProcessSamples( sensor_imu* samples, uint32_t sample_count ) = 0;

  /**
   * Virtual Destructor
   **/
  virtual ~Imu_IEventListener() {};
};

/**
 * Class to manage the IMU samples using the IMU API.
 **/
class Snapdragon::ImuManager {
public:

  /**
   * Constructor
   **/
  ImuManager();

  /**
   * Initializes the manager. This should be called first.
   * @return int32_t
   *   0 = success
   *  otherwise = failure.
   **/  
  int32_t Initialize();

  /**
   * Start a thread to read the IMU samples from the API.
   * @return int32_t
   *   0 = success
   *  otherwise = failure.
   **/
  int32_t Start();

  /**
   * stops the thread to read IMU samples and waits till thread is exited.
   * @return int32_t
   *   0 = success
   *  otherwise = failure.
   **/
  int32_t Stop();

  /**
   * Call this to perform any clean-up of resources created as part of Initilize()
   * @return int32_t
   *   0 = success
   *  otherwise = failure.
   **/  
  int32_t Terminate();

  /**
   * Add a Callback handler.
   * @param* handler
   *  The handler to add.
   * @return int32_t
   *   0 = success
   *  otherwise = failure.
   *  If the handler is already present, an error code is returned.
   **/
  int32_t AddHandler( Snapdragon::Imu_IEventListener* handler );

  /**
   * Remove a Callback handler.
   * @param* handler
   *  The handler to remove.
   * @return int32_t
   *   0 = success
   *  otherwise = failure.
   *  If the handler is not present, an error code is returned.
   **/
  int32_t RemoveHandler( Snapdragon::Imu_IEventListener* handler );
private:
  // thread function to read the imu samples.
  void ImuThreadMain();

  // utility function to handle initialize the imu apis.
  int32_t SetupImuApi();

  // class member variables.
  std::atomic<bool> thread_stop_;
  std::atomic<bool> initialized_;
  std::atomic<bool> thread_started_;
  std::thread imu_reader_thread_;
  std::mutex sync_mutex_;
  std::vector<Snapdragon::Imu_IEventListener*> imu_listeners_;

  //internal sensor_handle for imu_api's
  sensor_handle* imu_api_handle_ptr_;
};