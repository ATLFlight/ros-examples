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
#include <cmath>

int16_t TestVislam( Snapdragon::VislamManager& vislam_man ) {
  int32_t vislam_ret;
  int32_t max_init_tries = 100;
  int32_t max_poses = 100;
  int32_t count = 0;
  int32_t init_count = 0;
  bool vislam_init_successful = false;
  mvVISLAMPose vislamPose;
  int64_t vislamFrameId;
  uint64_t timestamp_ns;
  std::vector<mvVISLAMPose> pose_history;

  //loop to make sure that the initialzation is completed in < max_init_tries;
  while( !vislam_init_successful && init_count < max_init_tries ) {
    vislam_ret = vislam_man.GetPose( vislamPose, vislamFrameId, timestamp_ns );
    if( vislam_ret == 0 ) {
      if( vislamPose.poseQuality != MV_TRACKING_STATE_FAILED && vislamPose.poseQuality != MV_TRACKING_STATE_INITIALIZING) {
        // this indicates that initialization is successful.
        vislam_init_successful = true;
      }
      else {
        init_count++;
      }
    }  
  }

  if( !vislam_init_successful ) {
    std::cerr << "FAIL(): Error: Could not initialize VISLAM in: " << max_init_tries << " attempts" << std::endl;
    std::cerr << "   Possible causes: " << std::endl;
    std::cerr << "     1. Not enough Featues   ; Remedy: Place the board in a position where the optic flow can see the features" << std::endl;
    std::cerr << "     2. Board is moving      ; Remedy: Place the board stationary" << std::endl;
    std::cerr << "     3. Bad IMU or Bad Images; Remedy: Test Camera/IMU independently to validate that the input data is correct" << std::endl;
    std::cerr << "     4. None of the above    ; Remedy: VISLAM configuration is incorrect or bug in algorithm" << std::endl;
    return -1;
  }

  count = 0;
  while( count < max_poses ) {
    vislam_ret = vislam_man.GetPose( vislamPose, vislamFrameId, timestamp_ns );
    if( vislam_ret == 0 ) {
            pose_history.push_back( vislamPose );
            /*
            printf( "%lld, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f\n",
                    vislamFrameId,
                    vislamPose.bodyPose.matrix[0][0], vislamPose.bodyPose.matrix[0][1], vislamPose.bodyPose.matrix[0][2], vislamPose.bodyPose.matrix[0][3],
                    vislamPose.bodyPose.matrix[1][0], vislamPose.bodyPose.matrix[1][1], vislamPose.bodyPose.matrix[1][2], vislamPose.bodyPose.matrix[1][3],
                    vislamPose.bodyPose.matrix[2][0], vislamPose.bodyPose.matrix[2][1], vislamPose.bodyPose.matrix[2][2], vislamPose.bodyPose.matrix[2][3], 
                    vislamPose.timeAlignment, vislamPose.velocity[0], vislamPose.velocity[1], vislamPose.velocity[2] ); 
            */     
    }
    else {
      //std::cout << "Fail(): Error Getting pose from VISLAM" << std::endl;
    }
    count++;
  }

  // now run the check to make sure the stationary bench test is passing.
  // if there are less than max_poses; then it is an error.
  if( pose_history.size() < max_poses ) {
    std::cerr << "FAIL(): Error getting " << max_poses << ".  Got only: " << pose_history.size() << " poses. " << std::endl;
    return -1;
  }

  // check to see if there is atleast one good pose. and since this is a stationary test, 
  // the returned pose should be the [0,0,0] for translation and the Rotation matrix should be identity.
  int16_t valid_pose_count = 0;
  int16_t good_translation_pose_count = 0;
  int16_t good_rotation_pose_count = 0;
  int16_t high_quality_pose_count = 0;
  int16_t low_quality_pose_count = 0;
  int32_t loop_count = 0;
  for( mvVISLAMPose p : pose_history ) {
    if( p.poseQuality != MV_TRACKING_STATE_FAILED && p.poseQuality != MV_TRACKING_STATE_INITIALIZING ) {
      valid_pose_count++;
      if( p.poseQuality == MV_TRACKING_STATE_LOW_QUALITY ) {
        low_quality_pose_count ++;
      }
      if( p.poseQuality == MV_TRACKING_STATE_HIGH_QUALITY ) {
        high_quality_pose_count++;

        if( ( std::abs( p.bodyPose.matrix[0][3] ) < 0.01 )  &&
            ( std::abs( p.bodyPose.matrix[1][3] ) < 0.01 )  && 
            ( std::abs( p.bodyPose.matrix[2][3] ) < 0.01 ) ) {
          good_translation_pose_count ++;
        }

        //brute force method to check that the rotation matrix is an indentity matrix.
        if( ( std::abs( std::abs( p.bodyPose.matrix[0][0] ) - 1.0 ) < 0.001 ) && 
            ( std::abs( std::abs( p.bodyPose.matrix[1][1] ) - 1.0 ) < 0.001 ) && 
            ( std::abs( std::abs( p.bodyPose.matrix[2][2] ) - 1.0 ) < 0.001 ) && 
            ( std::abs( p.bodyPose.matrix[0][1])  < 0.001 ) && ( std::abs( p.bodyPose.matrix[0][2])  < 0.001 ) && 
            ( std::abs( p.bodyPose.matrix[1][0])  < 0.001 ) && ( std::abs( p.bodyPose.matrix[1][2])  < 0.001 ) && 
            ( std::abs( p.bodyPose.matrix[2][0])  < 0.001 ) && ( std::abs( p.bodyPose.matrix[2][1])  < 0.001 )  ) {
          good_rotation_pose_count ++;
        }
        else {
          // // this is debug only.  Remove it one done.
          // std::cerr << "[" << loop_count << "]: " << std::endl << "{" << std::endl
          //           << " " << p.bodyPose.matrix[0][0] << "," << p.bodyPose.matrix[0][1] << "," << p.bodyPose.matrix[0][1] << std::endl
          //           << " " << p.bodyPose.matrix[1][0] << "," << p.bodyPose.matrix[1][1] << "," << p.bodyPose.matrix[1][1] << std::endl
          //           << " " << p.bodyPose.matrix[2][0] << "," << p.bodyPose.matrix[2][1] << "," << p.bodyPose.matrix[2][1] << std::endl
          //           << "}" << std::endl;
        }
      }
    }
    loop_count++;
  }

  // print the stats here.
  std::cerr << " Initization Tries Count : " << init_count << std::endl;
  std::cerr << " Stats After Initilization: " << std::endl;
  std::cerr << "  Valid Pose       : " << valid_pose_count << std::endl;
  std::cerr << "  Low Pose Quality : " << low_quality_pose_count << std::endl;
  std::cerr << "  High Pose Quality: " << high_quality_pose_count << std::endl;
  std::cerr << "     Good Translation : " << good_translation_pose_count << std::endl;
  std::cerr << "     Good Rotation    : " << good_rotation_pose_count << std::endl;

  if( good_translation_pose_count < (int16_t)(max_poses*0.9) ) {
    std::cerr << "FAIL(): Error got too many bad translation information. expected greater than: " << (int16_t)(max_poses*0.9) << std::endl;
    return -1;
  }
  return 0;
}

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
  vislamParams.limitedIMUbWtrigger = 35.0;

  Snapdragon::CameraParameters param;
  param.enable_cpa = 1;
  param.camera_config.fps = 30;
  param.camera_config.cam_type = Snapdragon::CameraType::OPTIC_FLOW;
  param.mv_camera_config = config;

  //set the cpa configuration.
  mvCPA_Configuration cpaConfig;
  cpaConfig.cpaType = MVCPA_MODE_COST;
  cpaConfig.legacyCost.startExposure = param.camera_config.exposure;
  cpaConfig.legacyCost.startGain = param.camera_config.gain;
  cpaConfig.legacyCost.filterSize = 1;
  cpaConfig.legacyCost.exposureCost = 1.0f;
  cpaConfig.legacyCost.gainCost = 0.3333f;

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
  
  int16_t rc = 0;

  rc = TestVislam( vislam_man );
  if( rc == 0 ) {
    std::cerr << "SUCCESS(): Passed VISLAM Stationary Initlization Tests" << std::endl;
  }  
 
  vislam_man.Stop();

  return rc;
}
