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

int main( int argc, char** argv ) {
  mvCameraConfiguration config;
  // Set up camera configuraiton (snapdragon down facing camera)
  memset(&config, 0, sizeof(config));
  config.pixelWidth = 640;
  config.pixelHeight = 480;
  config.memoryStride = 640;

  config.principalPoint[0] = 320;
  config.principalPoint[1] = 240;

  config.focalLength[0] = 275;
  config.focalLength[1] = 275;

  config.distortion[0] = 0.003908;
  config.distortion[1] = -0.009574;
  config.distortion[2] = 0.010173;
  config.distortion[3] = -0.003329;
  config.distortion[4] = 0;
  config.distortion[5] = 0;
  config.distortion[6] = 0;
  config.distortion[7] = 0;
  config.distortionModel = 10;

  Snapdragon::VislamManager::InitParams vislamParams;

  vislamParams.tbc[0] = 0.005;
  vislamParams.tbc[1] = 0.0150;
  vislamParams.tbc[2] = 0.0;

  vislamParams.ombc[0] = 0.0;
  vislamParams.ombc[1] = 0.0;
  vislamParams.ombc[2] = 1.57;

  vislamParams.delta = -0.008;

  vislamParams.std0Tbc[0] = 0.002;
  vislamParams.std0Tbc[1] = 0.002;
  vislamParams.std0Tbc[2] = 0.001;

  vislamParams.std0Ombc[0] = 0.0174532925199433;
  vislamParams.std0Ombc[1] = 0.0174532925199433;
  vislamParams.std0Ombc[2] = 0.0174532925199433;

  vislamParams.std0Delta = 0.001;
  vislamParams.accelMeasRange = 156;
  vislamParams.gyroMeasRange = 34;

  vislamParams.stdAccelMeasNoise = 0.316227766016838; // sqrt(1e-1);
  vislamParams.stdGyroMeasNoise = 1e-2; // sqrt(1e-4);

  vislamParams.stdCamNoise = 100;
  vislamParams.minStdPixelNoise = 0.5;
  vislamParams.failHighPixelNoisePoints = false;

  vislamParams.logDepthBootstrap = 0;
  vislamParams.useLogCameraHeight = false;
  vislamParams.logCameraHeightBootstrap = -3.22;
  vislamParams.noInitWhenMoving = true;

  Snapdragon::CameraParameters param;
  param.enable_cpa = 1;
  param.camera_config.fps = 30;
  param.camera_config.cam_type = Snapdragon::CameraType::OPTIC_FLOW;
  param.mv_camera_config = config;

  //set the cpa configuration.
  mvCPA_Configuration cpaConfig;
  cpaConfig.cpaType = 1;
  cpaConfig.startExposure = param.camera_config.exposure;
  cpaConfig.startGain = param.camera_config.gain;
  cpaConfig.filterSize = 1;
  cpaConfig.exposureCost = 1.0f;
  cpaConfig.gainCost = 0.3333f;

  param.mv_cpa_config = cpaConfig;  

  Snapdragon::VislamManager vislam_man;
  int32_t vislam_ret;
  vislam_ret = vislam_man.Initialize( param, vislamParams );
  if( vislam_ret != 0 ) {
    std::cout << "FAIL() Error Initializing the VISLAM Manager rc: " << vislam_ret << std::endl;
    return -1;
  }

  vislam_ret = vislam_man.Start();
  if( vislam_ret != 0 ) {
    std::cout << "FAIL() Error Starting the VISLAM Manager rc: " << vislam_ret << std::endl;
    return -1;
  }

  // now try to get the pose and print it to the screen.
  int32_t max_poses = 100;
  int32_t count = 0;
  mvVISLAMPose vislamPose;
  int64_t vislamFrameId;
  uint64_t timestamp_ns;
  while( count < max_poses ) {
    vislam_ret = vislam_man.GetPose( vislamPose, vislamFrameId, timestamp_ns );
    if( vislam_ret == 0 ) {
            printf( "%lld, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f\n",
                    vislamFrameId,
                    vislamPose.bodyPose.matrix[0][0], vislamPose.bodyPose.matrix[0][1], vislamPose.bodyPose.matrix[0][2], vislamPose.bodyPose.matrix[0][3],
                    vislamPose.bodyPose.matrix[1][0], vislamPose.bodyPose.matrix[1][1], vislamPose.bodyPose.matrix[1][2], vislamPose.bodyPose.matrix[1][3],
                    vislamPose.bodyPose.matrix[2][0], vislamPose.bodyPose.matrix[2][1], vislamPose.bodyPose.matrix[2][2], vislamPose.bodyPose.matrix[2][3], 
                    vislamPose.timeAlignment, vislamPose.velocity[0], vislamPose.velocity[1], vislamPose.velocity[2] );      
    }
    else {
      std::cout << "Fail(): Error Getting pose from VISLAM" << std::endl;
    }
    count++;
  }
  vislam_man.Stop();
  return 0;
}
