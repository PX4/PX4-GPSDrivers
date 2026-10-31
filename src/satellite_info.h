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
 * @file satellite_info.h
 * Fields match PX4-Autopilot's generated uORB `satellite_info_s`
 * (msg/SatelliteInfo.msg) one-for-one, but this struct's own byte layout
 * belongs to px4-gpsdrivers, not to uORB - see sensor_gnss.h for why.
 */

#pragma once

#include <cstdint>

// Pinned explicitly (rather than relying on the platform's default
// alignment) so this layout is identical on every architecture/compiler -
// this library's consumers all link against the same packed layout,
// regardless of what any other project's in-memory struct looks like.
#pragma pack(push, 1)

struct satellite_info_s {
	uint64_t timestamp;
	static constexpr uint8_t SAT_INFO_MAX_SATELLITES = 40;

	uint8_t count;
	uint8_t svid[SAT_INFO_MAX_SATELLITES];
	uint8_t used[SAT_INFO_MAX_SATELLITES];
	uint8_t elevation[SAT_INFO_MAX_SATELLITES];
	uint8_t azimuth[SAT_INFO_MAX_SATELLITES];
	uint8_t snr[SAT_INFO_MAX_SATELLITES];
	uint8_t prn[SAT_INFO_MAX_SATELLITES];
};

#pragma pack(pop)
