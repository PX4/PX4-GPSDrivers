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
	GPSDriverNMEA(GPSCallbackPtr callback, void *callback_user, 
				  sensor_gps_s *gps_position,
				  struct satellite_info_s *satellite_info,
				  float heading_offset = 0.f);
	
	virtual ~GPSDriverNMEA() = default;

	int receive(unsigned timeout);
    int configure(unsigned &baudrate, OutputMode output_mode);

private:
	
	void decodeInit(void);
	int handleMessage(int len);
	int parseChar(uint8_t b);

	int32_t read_int();
	double read_float();
	char read_char();
	enum nmea_decode_state_t {
			NMEA_DECODE_UNINIT,
			NMEA_DECODE_GOT_SYNC1,
			NMEA_DECODE_GOT_NMEA,
			NMEA_DECODE_GOT_FIRST_CS_BYTE
	};

	sensor_gps_s *_gps_position {nullptr};
	struct satellite_info_s *_satellite_info {nullptr};
    uint32_t _last_POS_timeUTC{0};
	uint32_t _last_VEL_timeUTC{0};
	uint32_t _last_FIX_timeUTC{0};
    int _nmea_fd;

    uint8_t sat_num_gga{0};
	uint8_t sat_num_gns{0};
	uint8_t sat_num_gsv{0};
	uint8_t sat_num_gpgsv{0};
	uint8_t sat_num_glgsv{0};
	uint8_t sat_num_gagsv{0};
	uint8_t sat_num_gbgsv{0};
	uint8_t sat_num_bdgsv{0};

    bool clock_set {false};

//  check if we got all basic essential packages we need
    bool _POS_received{false};
    bool _ALT_received{false};
    bool _SVNUM_received{false};
    bool _SVINFO_received{false};
    bool _FIX_received{false};
    bool _DOP_received{false};
    bool _VEL_received{false};
    bool _EPH_received{false};
    bool _TIME_received{false};
    bool _HEAD_received{false};

    nmea_decode_state_t _decode_state{NMEA_DECODE_UNINIT};
    uint8_t _rx_buffer[NMEA_RECV_BUFFER_SIZE] {};
    uint16_t _rx_buffer_bytes{};
	
    float _heading_offset;
};
