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
 * @file sensor_gnss.h
 * The GNSS position report. Fields match PX4-Autopilot's generated uORB
 * `sensor_gnss_s` (msg/SensorGnss.msg, formerly msg/SensorGps.msg)
 * one-for-one, but this struct's own byte layout belongs to
 * px4-gpsdrivers, not to uORB: it is packed (see below) so it is identical
 * across architectures/compilers, which PX4-Autopilot's naturally-aligned
 * struct (or this project's own tests/definitions.h) does not guarantee.
 * Consumers link against this layout as-is rather than reinterpreting it
 * as some other project's in-memory struct.
 */

#pragma once

#include <cstdint>

// Pinned explicitly (rather than relying on the platform's default
// alignment) so this layout is identical on every architecture/compiler -
// this library's consumers all link against the same packed layout,
// regardless of what any other project's in-memory struct looks like.
#pragma pack(push, 1)

// pack(1) above only pins layout/padding, not byte order - every real target
// (PX4-Autopilot, QGroundControl) is little-endian, so fail loudly at compile
// time rather than silently producing corrupt GPS data on a hypothetical
// big-endian build.
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "big-endian targets are not supported by this packed layout");
#endif

struct sensor_gnss_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;

	uint32_t device_id;

	double latitude;
	double longitude;
	double altitude_msl;
	double altitude_ellipsoid;

	float speed_accuracy;
	float course_accuracy;

	static constexpr uint8_t FIX_TYPE_NONE = 1;
	static constexpr uint8_t FIX_TYPE_2D = 2;
	static constexpr uint8_t FIX_TYPE_3D = 3;
	static constexpr uint8_t FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4;
	static constexpr uint8_t FIX_TYPE_RTK_FLOAT = 5;
	static constexpr uint8_t FIX_TYPE_RTK_FIXED = 6;
	static constexpr uint8_t FIX_TYPE_EXTRAPOLATED = 8;
	uint8_t fix_type;

	float eph;
	float epv;

	float hdop;
	float vdop;

	int32_t noise;
	uint16_t automatic_gain_control;

	static constexpr uint8_t JAMMING_STATE_UNKNOWN = 0;
	static constexpr uint8_t JAMMING_STATE_OK = 1;
	static constexpr uint8_t JAMMING_STATE_MITIGATED = 2;
	static constexpr uint8_t JAMMING_STATE_DETECTED = 3;
	uint8_t jamming_state;
	int32_t jamming_indicator;

	static constexpr uint8_t SPOOFING_STATE_UNKNOWN = 0;
	static constexpr uint8_t SPOOFING_STATE_OK = 1;
	static constexpr uint8_t SPOOFING_STATE_MITIGATED = 2;
	static constexpr uint8_t SPOOFING_STATE_DETECTED = 3;
	uint8_t spoofing_state;

	static constexpr uint8_t AUTHENTICATION_STATE_UNKNOWN = 0;
	static constexpr uint8_t AUTHENTICATION_STATE_INITIALIZING = 1;
	static constexpr uint8_t AUTHENTICATION_STATE_ERROR = 2;
	static constexpr uint8_t AUTHENTICATION_STATE_OK = 3;
	static constexpr uint8_t AUTHENTICATION_STATE_DISABLED = 4;
	uint8_t authentication_state;

	float ground_speed;
	float vel_north;
	float vel_east;
	float vel_down;
	float course;
	bool vel_ned_valid;

	int32_t timestamp_time_relative;
	uint64_t time_utc_usec;

	uint8_t satellites_used;

	static constexpr uint32_t SYSTEM_ERROR_OK = 0;
	static constexpr uint32_t SYSTEM_ERROR_INCOMING_CORRECTIONS = 1;
	static constexpr uint32_t SYSTEM_ERROR_CONFIGURATION = 2;
	static constexpr uint32_t SYSTEM_ERROR_SOFTWARE = 4;
	static constexpr uint32_t SYSTEM_ERROR_ANTENNA = 8;
	static constexpr uint32_t SYSTEM_ERROR_EVENT_CONGESTION = 16;
	static constexpr uint32_t SYSTEM_ERROR_CPU_OVERLOAD = 32;
	static constexpr uint32_t SYSTEM_ERROR_OUTPUT_CONGESTION = 64;
	uint32_t system_error;

	float rtcm_injection_rate;
	uint8_t selected_rtcm_instance;

	static constexpr uint8_t CORRECTIONS_PROTOCOL_UNKNOWN = 0;
	static constexpr uint8_t CORRECTIONS_PROTOCOL_RTCM3 = 1;
	static constexpr uint8_t CORRECTIONS_PROTOCOL_SPARTN = 2;
	static constexpr uint8_t CORRECTIONS_PROTOCOL_HAS = 3;
	static constexpr uint8_t CORRECTIONS_PROTOCOL_PMP = 4;
	static constexpr uint8_t CORRECTIONS_PROTOCOL_QZSS_L6 = 5;
	uint8_t corrections_protocol;

	bool corrections_crc_failed;

	static constexpr uint8_t CORRECTIONS_MSG_USED_UNKNOWN = 0;
	static constexpr uint8_t CORRECTIONS_MSG_USED_NOT_USED = 1;
	static constexpr uint8_t CORRECTIONS_MSG_USED_USED = 2;
	uint8_t corrections_msg_used;
};

#pragma pack(pop)
