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
#include "SnapdragonImuManager.hpp"
#include <thread>
#include <chrono>
#include <iostream>

namespace SnapdragonTest {
  class TestImuHandler;
};

class SnapdragonTest::TestImuHandler : public Snapdragon::Imu_IEventListener {
public:

  struct ImuStats {
    ImuStats() {
      total_sample_count = 0;
      latest_timestamp_us = 0;
      latest_sequence_num = 0;
      missed_samples = 0;
    }
    uint64_t total_sample_count;
    uint64_t latest_timestamp_us;
    uint32_t latest_sequence_num;
    uint64_t missed_samples;   
  };

  TestImuHandler() {
  }

  int32_t Imu_IEventListener_ProcessSamples( sensor_imu* samples, uint32_t count ) {
    stats_.total_sample_count += count;
    stats_.latest_timestamp_us = samples[count-1].timestamp_in_us;
    if( stats_.latest_sequence_num != 0 &&  (stats_.latest_sequence_num + 1 != samples[0].sequence_number ) ) {
      stats_.missed_samples = samples[0].sequence_number - stats_.latest_sequence_num;
    }
    stats_.latest_sequence_num = samples[count-1].sequence_number;
    return 0;
  }

  ImuStats GetStats()  { return stats_; };
private:
  ImuStats stats_;

};

int main( int argc, char** argv ) {

  SnapdragonTest::TestImuHandler imu_handler;
  Snapdragon::ImuManager imu_man;

  if( imu_man.AddHandler( &imu_handler ) != 0 ) {
    std::cout << "FAIL: Error Adding Handler for IMU" << std::endl;
  }

  // call the initialize;
  if( imu_man.Initialize() != 0 ) {
    std::cout << "FAIL: Error Calling Initialize() " << std::endl;
    return -1;
  }

  // call to start the imu processing;
  // First Get the Metrics/stats/
  SnapdragonTest::TestImuHandler::ImuStats begin_stats = imu_handler.GetStats();

  if( imu_man.Start() != 0 ) {
    std::cout << "FAIL: Error calling the start of the imu_manager" << std::endl;
    imu_man.Terminate();
    return -1;
  }

  std::this_thread::sleep_for( std::chrono::seconds(1) );

  if( imu_man.Stop() != 0 ) {
    std::cout << "FAIL: Error calling the stop imu manager" << std::endl;
  }

  //check to make sure the stats are ok.
  SnapdragonTest::TestImuHandler::ImuStats end_stats = imu_handler.GetStats();

  if( end_stats.missed_samples != begin_stats.missed_samples ) {
    std::cout << "FAIL: Missing imu samples from Imu: " 
              << (end_stats.missed_samples - begin_stats.missed_samples) 
              << std::endl;
  }
  std::cout << "Received Samples: " << (end_stats.total_sample_count - begin_stats.total_sample_count ) << std::endl;

  if( imu_man.Terminate() != 0 ) {
    std::cout << "Fail: Error calling terminate for the ImuHandler" << std::endl;
    return -1;
  }

  std::cout << "PASS() Passed all ImuManager Api Tests()" << std::endl;
  return 0;
}
