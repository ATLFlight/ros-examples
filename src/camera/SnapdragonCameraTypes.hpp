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
#include <cstring>
#include "mv.h"
#include "mvCPA.h"

namespace Snapdragon {

  static const uint16_t DEFAULT_WIDTH  = 640;
  static const uint16_t DEFAULT_HEIGHT = 480;
  static const uint16_t DEFAULT_MEM_STRIDE = 640;
  static const int16_t  DEFAULT_FPS = 60;
  static const float    DEFAULT_EXPOSURE = 0.36; // This is a normalized value between 0 and 1.0
  static const float    DEFAULT_GAIN = 0.35;     // This is a normalized value between 0 and 1.0.
  static const int16_t  MIN_EXPOSURE = 0;        // MIN/MAX exposure value to normalize Exposure values.
  static const int16_t  MAX_EXPOSURE = 500;      // MIN/MAX exposure values to normalize Exposure values.
  static const int16_t  MIN_GAIN = 0;            // MIN/MAX gain values to normalize Exposure values.
  static const int16_t  MAX_GAIN = 200;          // MIN/MAX gain values to normalize Exposure values.
  static const float    DEFAULT_ROW_PERIOD_US = 19.3333; // This is used to compute the total exposure time.
  static const int16_t  DEFAULT_CPA_EXPOSURE_CHANGE_THRESHOLD = 0;
  static const int16_t  DEFAULT_CPA_GAIN_CHANGE_THRESHOLD = 0;
  static const uint16_t DEFAULT_IMAGE_BUFFER_SIZE = 5; // if the image buffers are > max, then the camera pipeline can be stalled.
  static const uint16_t MAX_IMAGE_BUFFER_SIZE = 6; 
  /**
   * Enum for the camera types. The numbers are listed a documented in the 
   * camera.h api file.
   **/
  enum class CameraType
  {
    UNKNOWN = -1,
    HIRES = 0,
    OPTIC_FLOW = 1,
    RIGHT_STEREO = 2,
    STEREO = 3,
  };

  /**
   * Enum to determine the mode of the camera capture. 
   */
  enum CameraCaptureMode {
    PREVIEW,
    VIDEO
  };

  /**
   * Structure to hold the camera configration parameters.
   */
  struct CameraConfig
  {
    CameraConfig()
    {
      verbose                       = false;
      cam_type                      = CameraType::UNKNOWN;
      pixel_width                   = DEFAULT_WIDTH;
      pixel_height                  = DEFAULT_HEIGHT;
      memory_stride                 = DEFAULT_MEM_STRIDE;
      fps                           = DEFAULT_FPS;
      exposure                      = DEFAULT_EXPOSURE;
      gain                          = DEFAULT_GAIN;
      min_exposure                  = MIN_EXPOSURE;
      max_exposure                  = MAX_EXPOSURE;
      min_gain                      = MIN_GAIN;
      max_gain                      = MAX_GAIN;
      row_period_us                 = DEFAULT_ROW_PERIOD_US;
      cpa_exposure_change_threshold = DEFAULT_CPA_EXPOSURE_CHANGE_THRESHOLD;
      cpa_gain_change_threshold     = DEFAULT_CPA_GAIN_CHANGE_THRESHOLD;
      num_image_buffers             = DEFAULT_IMAGE_BUFFER_SIZE;
    }

    bool       verbose;
    CameraType cam_type;
    uint16_t   pixel_width;
    uint16_t   pixel_height;
    uint16_t   memory_stride;
    int16_t    fps;
    float      exposure;
    float      gain;
    int16_t    min_exposure;
    int16_t    max_exposure;
    int16_t    min_gain;
    int16_t    max_gain;
    float      row_period_us;
    int16_t    cpa_exposure_change_threshold;
    int16_t    cpa_gain_change_threshold;
    uint16_t   num_image_buffers;
  };

  /**
   * structure to encapsulate the mvSDK parameters and provide an initializer.
   **/
  struct CameraParameters {
    CameraParameters() {
      verbose = false;
      enable_cpa = 0;
      // not initializing the camera_config variable as it has the constructor.
      std::memset(&mv_camera_config, 0, sizeof(mvCameraConfiguration));
      std::memset(&mv_cpa_config, 0, sizeof(mvCPA_Configuration));
    }

    bool enable_cpa;
    bool verbose;

    CameraConfig          camera_config;
    mvCameraConfiguration mv_camera_config;
    mvCPA_Configuration   mv_cpa_config;
  };
}
