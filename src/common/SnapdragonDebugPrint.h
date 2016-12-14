#pragma once
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

#ifdef __cplusplus
extern "C"{
#endif

#define HEADER_SIZE 48
#define DEBUG_PRINT_BUFFER_SIZE (1024 - HEADER_SIZE)

typedef enum
{
  SNAP_DEBUG_PRINT_TYPE_INFO,
  SNAP_DEBUG_PRINT_TYPE_WARN,
  SNAP_DEBUG_PRINT_TYPE_ERROR,
} SnapDebugPrintType;

#define INFO_PRINT(fmt, ...) { snapdragon_debug_print(SNAP_DEBUG_PRINT_TYPE_INFO,   "%s %d: " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__); }
#define WARN_PRINT(fmt, ...) { snapdragon_debug_print(SNAP_DEBUG_PRINT_TYPE_WARN,   "%s %d: " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__); }
#define ERROR_PRINT(fmt, ...) { snapdragon_debug_print(SNAP_DEBUG_PRINT_TYPE_ERROR, "%s %d: " fmt, __FUNCTION__, __LINE__, ##__VA_ARGS__); }

int snapdragon_debug_print(SnapDebugPrintType print_type, const char * fmt, ...);

#ifdef __cplusplus
}
#endif

