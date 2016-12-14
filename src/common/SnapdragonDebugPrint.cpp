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

#include "SnapdragonDebugPrint.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <chrono>

int snapdragon_debug_print(SnapDebugPrintType print_type, const char * fmt, ...)
{
  char debug_print_buffer[DEBUG_PRINT_BUFFER_SIZE];
  memset(&debug_print_buffer,0, DEBUG_PRINT_BUFFER_SIZE);

  va_list args;
  va_start(args,fmt);
  int msg_len = vsnprintf(debug_print_buffer, DEBUG_PRINT_BUFFER_SIZE, fmt, args);
  va_end(args);

  char header[HEADER_SIZE];
  FILE* stream;
  if (print_type == SNAP_DEBUG_PRINT_TYPE_INFO)
  {
    snprintf(header, HEADER_SIZE, "[%lld] VISLAM INFO: ", 
      std::chrono::duration_cast<std::chrono::microseconds>
      ( std::chrono::steady_clock::now().time_since_epoch() ).count() );
    stream = stdout;
  }
  else if (print_type == SNAP_DEBUG_PRINT_TYPE_ERROR)
  {
    snprintf(header, HEADER_SIZE, "[%lld] VISLAM ERROR: ", 
            std::chrono::duration_cast<std::chrono::microseconds>
            ( std::chrono::steady_clock::now().time_since_epoch() ).count() );
    stream = stderr;
  }
  else if (print_type == SNAP_DEBUG_PRINT_TYPE_WARN)
  {
    snprintf(header, HEADER_SIZE, "[%lld] VISLAM WARNING: ", 
            std::chrono::duration_cast<std::chrono::microseconds>
            ( std::chrono::steady_clock::now().time_since_epoch() ).count() );
    stream = stderr;
  }
  else
  {
    snprintf(header, HEADER_SIZE, "[%lld] VISLAM: ", 
            std::chrono::duration_cast<std::chrono::microseconds>
            ( std::chrono::steady_clock::now().time_since_epoch() ).count() );
    stream = stdout;
  }

  char* combined_string = strcat(header, debug_print_buffer);
  fprintf(stream, strcat(combined_string, "\n"));
  fflush(stream);

  return 0;
}

