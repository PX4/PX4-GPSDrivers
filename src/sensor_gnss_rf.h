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
 * @file sensor_gnss_rf.h
 * Fields match PX4-Autopilot's generated uORB `sensor_gnss_rf_s`
 * (msg/SensorGnssRf.msg) one-for-one, but this struct's own byte layout
 * belongs to px4-gpsdrivers, not to uORB - see sensor_gnss.h for why.
 * Reported unconditionally by GPSHelper::gotRFMessage(), independent of
 * CONFIG_GPS_UBX_SPAN.
 */

#pragma once

#include <cstdint>

// Pinned explicitly (rather than relying on the platform's default
// alignment) so this layout is identical on every architecture/compiler -
// this library's consumers all link against the same packed layout,
// regardless of what any other project's in-memory struct looks like.
#pragma pack(push, 1)

// pack(1) only pins layout/padding, not byte order - see sensor_gnss.h.
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "big-endian targets are not supported by this packed layout");
#endif

struct sensor_gnss_rf_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint32_t device_id;
	uint32_t post_status;
	uint32_t center_frequency;
	uint16_t noise_per_ms;
	uint16_t automatic_gain_control;
	uint8_t block_id;
	uint8_t antenna_status;
	uint8_t antenna_power;
	uint8_t jamming_indicator;
	uint8_t jamming_state;
	int8_t i_offset;
	uint8_t i_magnitude;
	int8_t q_offset;
	uint8_t q_magnitude;
	uint8_t _padding0[7];
};

#pragma pack(pop)
