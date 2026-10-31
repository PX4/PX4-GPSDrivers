/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file definitions.h
 * Default `definitions.h` used when building this library standalone
 * (i.e. not vendored into PX4-Autopilot or QGroundControl, which each
 * provide their own `GPS_DEFINITIONS_HEADER`). The structs it pulls in
 * are px4-gpsdrivers' own canonical, packed layout - see sensor_gnss.h.
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "sensor_gnss.h"
#include "satellite_info.h"
#include "sensor_gnss_relative.h"
#include "sensor_gnss_rf.h"
#include "sensor_gnss_spectrum.h"

#ifndef M_DEG_TO_RAD
#define M_DEG_TO_RAD (M_PI / 180.0)
#endif
#ifndef M_RAD_TO_DEG
#define M_RAD_TO_DEG (180.0 / M_PI)
#endif
#ifndef M_DEG_TO_RAD_F
#define M_DEG_TO_RAD_F 0.0174532925f
#endif
#ifndef M_RAD_TO_DEG_F
#define M_RAD_TO_DEG_F 57.2957795f
#endif
#ifndef M_PI_2_F
#define M_PI_2_F 1.57079632679489661923f
#endif

#ifndef GPS_INFO
#define GPS_INFO(...) { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); }
#endif
#ifndef GPS_WARN
#define GPS_WARN(...) { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); }
#endif
#ifndef GPS_ERR
#define GPS_ERR(...) { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); }
#endif

static inline void gps_usleep(unsigned long usecs)
{
	std::this_thread::sleep_for(std::chrono::microseconds(usecs));
}

typedef uint64_t gps_abstime;
static inline gps_abstime gps_absolute_time()
{
	return static_cast<gps_abstime>(std::chrono::duration_cast<std::chrono::microseconds>(
						std::chrono::steady_clock::now().time_since_epoch()).count());
}
