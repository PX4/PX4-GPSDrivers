/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file nmea.h
 *
 * NMEA protocol definitions
 *
 * @author WeiPeng Guo <guoweipeng1990@sina.com>
 * @author Stone White <stone@thone.io>
 *
 */

#pragma once

#include "gps_helper.h"
#include "../../definitions.h"

#define NMEA_RECV_BUFFER_SIZE 1024
#define NMEA_DEFAULT_BAUDRATE 9600

class GPSDriverNMEA : public GPSHelper
{
public:
	/**
	 * @param heading_offset heading offset in radians [-pi, pi]. It is substracted from the measurement.
	 */
	GPSDriverNMEA(GPSCallbackPtr callback, void *callback_user,
		      sensor_gps_s *gps_position,
		      satellite_info_s *satellite_info,
		      float heading_offset = 0.f);
	virtual ~GPSDriverNMEA() = default;

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, OutputMode output_mode) override;

private:
	enum class NMEADecodeFlags : int {
		got_zda = (1 << 0),
		got_gga = (1 << 1),
		got_hdt = (1 << 2),
		got_gns = (1 << 3),
		got_rmc = (1 << 4),
		got_gst = (1 << 5),
		got_gsa = (1 << 6),
		got_gsv = (1 << 7),
		got_vtg = (1 << 8)
	};

	enum class NMEADecodeState {
		uninit,
		got_sync1,
		got_asteriks,
		got_first_cs_byte,
	};

	void decodeInit(void);
	int handleMessage(int len);
	int parseChar(uint8_t b);

	int32_t read_int();
	double read_float();
	char read_char();

	sensor_gps_s *_gps_position {nullptr};
	satellite_info_s *_satellite_info {nullptr};
	uint32_t _last_POS_timeUTC{0};
	uint32_t _last_VEL_timeUTC{0};
	uint32_t _last_FIX_timeUTC{0};

	uint8_t sat_num_gga{0};
	uint8_t sat_num_gns{0};
	uint8_t sat_num_gsv{0};
	uint8_t sat_num_gpgsv{0};
	uint8_t sat_num_glgsv{0};
	uint8_t sat_num_gagsv{0};
	uint8_t sat_num_gbgsv{0};
	uint8_t sat_num_bdgsv{0};

	//nmea protocol decode flags NMEADecodeFlags
	uint32_t _decode_flags{0};

	bool clock_set {false};

//  check if we got all basic essential packages we need
	bool _TIME_received{false};
	bool _POS_received{false};
	bool _ALT_received{false};
	bool _SVNUM_received{false};
	bool _SVINFO_received{false};
	bool _FIX_received{false};
	bool _DOP_received{false};
	bool _VEL_received{false};
	bool _EPH_received{false};
	bool _HEAD_received{false};

	NMEADecodeState _decode_state{NMEADecodeState::uninit};
	uint8_t _rx_buffer[NMEA_RECV_BUFFER_SIZE] {};
	uint16_t _rx_buffer_bytes{0};

	uint8_t _read_buf[GPS_READ_BUFFER_SIZE] {};
	ssize_t _bytes_parsed{0};
	ssize_t _bytes_readed{0};

	float _heading_offset;
};
