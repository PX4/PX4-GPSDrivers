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
 * @file skytraq.h
 *
 * SkyTraq Phoenix binary protocol driver, as documented in SkyTraq
 * Application Note AN0037 ("UAV Messages Of SkyTraq Phoenix GNSS
 * Receiver", Ver 1.4.72). Only the subset of the protocol needed
 * to configure the receiver and stream a position/velocity solution is
 * implemented:
 *  - Query Receiver Info and Version (0x75 / Sub-ID 0x20) / ACK (0x83):
 *    used for baudrate auto-detection.
 *  - Configure Serial Port (0x05): after auto-detection, move the receiver
 *    to SKYTRAQ_TARGET_BAUDRATE. Written to SRAM, which on a board that fits
 *    the RTC/backup supply survives a power cycle.
 *  - Configure Message Type (0x09): switch periodic output from NMEA to
 *    the binary protocol.
 *  - Configure Position Update Rate (0x0E): set the fix rate.
 *  - Configure Receiver Message Type (0x75 / Sub-ID 0x21): enable the
 *    Navigation messages group (Bit 1), which covers both the Navigation PVT
 *    Message and the Navigation Satellite SV Status Message.
 *  - Navigation PVT Message (0x75 / Sub-ID 0xA0):
 *    position/velocity/accuracy/UTC-time solution. Latitude and Longitude
 *    are IEEE-754 doubles in degrees, so an RTK solution survives the
 *    message intact.
 *  - Navigation Satellite SV Status Message (0x75 / Sub-ID 0xA1):
 *    per-satellite elevation/azimuth/SNR/used-in-fix status.
 *
 * Baudrate handling: a rate pinned via SER_GPSn_BAUD is the only one probed
 * and is left alone, while auto-detection sweeps a fallback list and then
 * moves the receiver onto the target rate.
 *
 * Verified against real hardware (Pixhawk 6C + SkyTraq Phoenix GNSS
 * receiver).
 */

#pragma once

#include "gps_helper.h"

// Sized to hold a Navigation Satellite SV Status Message (0x75/0xA1): 8 header
// bytes + NSVS*13 bytes/SV. 900 bytes comfortably covers 68 SVs, well above any
// realistic total SV-in-view count across all supported GNSS constellations.
#define SKYTRAQ_RECV_BUFFER_SIZE 900

// Baudrate the receiver is moved to once auto-detection has located it. Only applied
// when PX4 asked for auto-detection (SER_GPSn_BAUD = 0); a pinned baudrate is used
// as-is.
#define SKYTRAQ_TARGET_BAUDRATE 115200

// Receiver Info and Version (0x75/0x90) reports an ODM version as X1.Y1.Z1; AN0037
// defines X1 >= 240 as the UAV firmware line, which is the one carrying the message set
// this driver speaks. Anything below that cannot drive PX4.
#define SKYTRAQ_MIN_ODM_MAJOR 240

class GPSDriverSkyTraq : public GPSHelper
{
public:
	GPSDriverSkyTraq(GPSCallbackPtr callback, void *callback_user,
			 sensor_gnss_s *gps_position,
			 satellite_info_s *satellite_info);

	virtual ~GPSDriverSkyTraq() = default;

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, const GPSConfig &config) override;

private:

	enum class SkytraqDecodeState {
		Sync1,
		Sync2,
		Length1,
		Length2,
		Payload,
		Checksum,
		End1,
		End2,
	};

	void decodeInit();

	/**
	 * Parse a single incoming byte.
	 * @return the payload length (message ID byte + body, > 0) once a full,
	 *         checksum-valid frame has been received, 0 otherwise.
	 */
	int parseChar(uint8_t b);

	/** Decode the payload currently held in _rx_buffer[0.._rx_payload_length-1]. */
	int handleMessage();

	/**
	 * Navigation PVT Message (ID 0x75, Sub-ID 0xA0).
	 * @return true once _gps_position holds this message; false if it was discarded.
	 */
	bool handleNavigationPvtMessage();

	/**
	 * Navigation Satellite SV Status Message (ID 0x75, Sub-ID 0xA1).
	 * @return true once _satellite_info holds this message; false if it was discarded.
	 */
	bool handleNavigationSvStatusMessage();

	/** Receiver Info and Version (ID 0x75, Sub-ID 0x90), the reply to the 0x20 probe. */
	void handleReceiverInfoMessage();

	/** Map a SkyTraq (GNSS type, SVID) pair to the PX4 PRN numbering scheme. */
	static uint8_t skytraqSvidToPx4Prn(uint8_t gnss_type, uint8_t svid);

	/** Map a baudrate to AN0037's Configure Serial Port (0x05) rate index, -1 if unsupported. */
	static int baudrateToIndex(unsigned baudrate);

	/**
	 * Switch the receiver and this end to @a baudrate via Configure Serial Port (0x05).
	 * @return true if the receiver ACKed and both ends moved; false leaves them untouched.
	 */
	bool setDeviceBaudrate(unsigned baudrate);

	/** Build and send a request/config frame. @return num written bytes, <0 on error. */
	int sendMessage(uint8_t msg_id, const uint8_t *payload, uint16_t payload_length);

	/**
	 * Send a message and block (with retries) until the matching ACK is received.
	 * @param sub_id expected Sub-ID echoed back in the ACK/NACK, or 0xff if msg_id has no Sub-ID.
	 */
	bool sendMessageAndWaitForAck(uint8_t msg_id, uint8_t sub_id, const uint8_t *payload,
				      uint16_t payload_length, unsigned timeout_per_try_ms, int num_tries);

	sensor_gnss_s *_gps_position {nullptr};
	satellite_info_s *_satellite_info {nullptr};

	SkytraqDecodeState _decode_state{SkytraqDecodeState::Sync1};
	uint8_t _rx_buffer[SKYTRAQ_RECV_BUFFER_SIZE] {};
	uint16_t _rx_payload_length{0};
	uint16_t _rx_payload_index{0};
	uint8_t _rx_checksum{0};

	bool _warned_short_pvt{false};   ///< latches the "firmware too old" warning to one print
	bool _warned_bad_position{false}; ///< likewise for the position sanity check

	bool _got_receiver_info{false}; ///< a 0x75/0x90 reply arrived during the probe
	uint32_t _kernel_version{0};
	uint32_t _odm_version{0};

	uint8_t _expected_ack_msg_id{0};
	uint8_t _expected_ack_sub_id{0};
	bool _ack_pending{false};
	bool _last_ack_ok{false};

	unsigned _current_baudrate{0};
};
