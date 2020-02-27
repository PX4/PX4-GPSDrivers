/****************************************************************************
 *
 *   Copyright (C) 2019. All rights reserved.
 *   Author: Rui Zheng <ruizheng@femtomes.com>
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

/** @file Femtomes protocol definitions */

#include "gps_helper.h"
#include "base_station.h"
#include "../../definitions.h"
#include <cmath>


class RTCMParsing;

#if defined(__GNUC__)
#define PACKED __attribute__((packed))
#else
#define PACKED 
#endif

/* ms, timeout for waiting for a response*/
#define FEMO_RESPONSE_TIMEOUT	200		

/* Femtomes ID for UAV output message */
#define FEMTO_MSG_ID_UAVGPS 		8001   

/* Femto uavgps message frame premble 3 bytes*/
#define FEMTO_PREAMBLE1				0xaa
#define	FEMTO_PREAMBLE2				0x44
#define FEMTO_PREAMBLE3				0x12


class GPSDriverFemto : public GPSBaseStationSupport
{
public:
	/**
	 * @param heading_offset heading offset in radians [-pi, pi]. It is substracted from the measurement.
	 */
	GPSDriverFemto(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position,
			 struct satellite_info_s *satellite_info, float heading_offset = 0.f);
	virtual ~GPSDriverFemto();

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, OutputMode output_mode) override;

private:
	enum class FemtoCommand {
		Acked,			/**< Command that returns a (N)Ack */
		PRT,			/**< port config*/ 
		RID,			/**< board identification*/ 
		RECEIPT			/**< board identification*/ 
	};

	enum class FemtoDecodeState {
		pream_ble1,			/**< Frame header preamble first byte 0xaa */
		pream_ble2,			/**< Frame header preamble second byte 0x44 */
		pream_ble3,			/**< Frame header preamble third byte 0x12 */
		head_length,		/**< Frame header length */
		head_data,			/**< Frame header data */
		data,				/**< Frame data */
		crc1,				/**< Frame crc1 */
		crc2,				/**< Frame crc2 */
		crc3,				/**< Frame crc3 */
		crc4				/**< Frame crc4 */
	};

	/**
	 * Femto board number
	 */
	enum class FemtoBoardType {
		BT_672, 
		BT_682,	
		BT_680,
		BT_681,
		BT_6A0,
		other
	};
	
	/**
	 * femto_uav_gps_t struct need to be packed
	 */
	struct PACKED femto_uav_gps_t {
		uint64_t timestamp;
		uint64_t time_utc_usec;
		int32_t lat;
		int32_t lon;
		int32_t alt;
		int32_t alt_ellipsoid;
		float s_variance_m_s;
		float c_variance_rad;
		float eph;
		float epv;
		float hdop;
		float vdop;
		int32_t noise_per_ms;
		int32_t jamming_indicator;
		float vel_m_s;
		float vel_n_m_s;
		float vel_e_m_s;
		float vel_d_m_s;
		float cog_rad;
		int32_t timestamp_time_relative;
		float heading;
		uint8_t fix_type;
		bool vel_ned_valid;
		uint8_t satellites_used;
	};

	/**
	 * femto_msg_header_t is femto data header
	 */
	struct PACKED femto_msg_header_t
	{
		uint8_t preamble[3];	/**< Frame header preamble 0xaa 0x44 0x12 */
		uint8_t headerlength;	/**< Frame header length ,from the beginning 0xaa */
		uint16_t messageid;     /**< Frame message id ,example the FEMTO_MSG_ID_UAVGPS 8001*/
		uint8_t messagetype;	/**< Frame message id type */
		uint8_t portaddr;		/**< Frame message port address */
		uint16_t messagelength; /**< Frame message data length,from the beginning headerlength+1,end headerlength + messagelength*/
		uint16_t sequence;		
		uint8_t idletime;		/**< Frame message idle module time */
		uint8_t timestatus;		
		uint16_t week;			
		uint32_t tow;			
		uint32_t recvstatus;	
		uint16_t resv;			
		uint16_t recvswver;		
	};

	/**
	 * Analysis Femto uavgps frame content 
	 */
	union PACKED msg_buf_t {
		vehicle_gps_position_s femto_uav_gps;
		uint8_t bytes[256];
	};

	/**
	 * Analysis Femto uavgps frame header
	 */
	union PACKED msg_header_t {
		struct femto_msg_header_t femto_header;
		uint8_t data[28];
	};

	/**
	 * receive Femto complete uavgps frame  
	 */
	struct PACKED femto_msg_t
	{
		msg_buf_t data;			/**< receive Frame message content */
		uint32_t crc;			/**< receive Frame message crc 4 bytes */
		msg_header_t header;	/**< receive Frame message header */
		uint16_t read;			/**< receive Frame message read bytes count */
	};

	/**
	 * caculate the frame crc value  
	 */
    uint32_t CRC32Value(uint32_t icrc);
    uint32_t CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc);
	
	/**
	 * when Constructor is work, initialize parameters
	 */
	void decodeInit(void);

	/**
	 * check the message if whether is 8001,memcpy data to _gps_position
	 */
	int handleMessage(int len);

	/**
	 * analysis frame data from buf[] to _femto_msg and check the frame is suceess or not
	 */
	int parseChar(uint8_t b);

	/**
	 * Write a command and wait for a (N)Ack
	 * @return 0 on success, <0 otherwise
	 */
	int writeAckedCommandFemto(const char* command, const char* reply, const unsigned timeout);

	/**
	 * receive data for at least the specified amount of time
	 */
	void receiveWait(unsigned timeout_min);

	/**
	 * enable output of correction output
	 */
	void activateCorrectionOutput();

	void sendSurveyInStatusUpdate(bool active, bool valid, double latitude = (double)NAN, double longitude = (double)NAN,
				      float altitude = NAN);

	void activateRTCMOutput();

	struct satellite_info_s *_satellite_info {nullptr};
	struct vehicle_gps_position_s *_gps_position {nullptr};
	struct femto_uav_gps_t _femto_uav_gps;
	struct femto_msg_t _femto_msg;
	FemtoDecodeState _decode_state{FemtoDecodeState::pream_ble1};

	bool _got_pashr_pos_message{false}; /**< If we got a PASHR,POS message, we will ignore GGA messages */
	
	uint64_t _last_timestamp_time{0};

	FemtoCommand _waiting_for_command;
	char _port{' '}; /**< port we are connected to (e.g. 'A') */
	FemtoBoardType _board{FemtoBoardType::other}; /**< board we are connected to */

	RTCMParsing *_rtcm_parsing{nullptr};

	gps_abstime _survey_in_start{ 0 };

	OutputMode _output_mode{ OutputMode::GPS };
	bool _correction_output_activated{false};
	bool _configure_done{false};
	float _heading_offset;
};
