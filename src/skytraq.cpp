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
 * @file skytraq.cpp
 *
 * SkyTraq Phoenix binary protocol implementation. See skytraq.h and
 * SkyTraq Application Note AN0037 for the protocol reference.
 */

#include <cmath>
#include <cstring>
#include <ctime>

#include "skytraq.h"

#define SKYTRAQ_WARN(...)  {GPS_WARN(__VA_ARGS__);}
#define SKYTRAQ_DEBUG(...) {/*GPS_WARN(__VA_ARGS__);*/}

namespace
{

uint16_t readU16BE(const uint8_t *buf)
{
	return (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

uint32_t readU32BE(const uint8_t *buf)
{
	return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
}

double readF64BE(const uint8_t *buf)
{
	// IEEE-754 binary64, most significant byte first, as the Navigation PVT Message
	// carries Latitude/Longitude in its 84-byte layout.
	uint64_t bits = 0;

	for (unsigned n = 0; n < 8; ++n) {
		bits = (bits << 8) | buf[n];
	}

	double value;
	memcpy(&value, &bits, sizeof(value));
	return value;
}

} // namespace

GPSDriverSkyTraq::GPSDriverSkyTraq(GPSCallbackPtr callback, void *callback_user,
				   sensor_gnss_s *gps_position,
				   satellite_info_s *satellite_info) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position),
	_satellite_info(satellite_info)
{
	decodeInit();
}

void GPSDriverSkyTraq::decodeInit()
{
	_decode_state = SkytraqDecodeState::Sync1;
	_rx_payload_length = 0;
	_rx_payload_index = 0;
	_rx_checksum = 0;
}

int GPSDriverSkyTraq::parseChar(uint8_t b)
{
	int ret = 0;

	switch (_decode_state) {
	case SkytraqDecodeState::Sync1:
		if (b == 0xA0) {
			_decode_state = SkytraqDecodeState::Sync2;
		}

		break;

	case SkytraqDecodeState::Sync2:
		if (b == 0xA1) {
			_decode_state = SkytraqDecodeState::Length1;

		} else {
			decodeInit();
		}

		break;

	case SkytraqDecodeState::Length1:
		_rx_payload_length = (uint16_t)b << 8;
		_decode_state = SkytraqDecodeState::Length2;
		break;

	case SkytraqDecodeState::Length2:
		_rx_payload_length |= b;
		_rx_payload_index = 0;
		_rx_checksum = 0;

		if (_rx_payload_length == 0 || _rx_payload_length > sizeof(_rx_buffer)) {
			decodeInit();

		} else {
			_decode_state = SkytraqDecodeState::Payload;
		}

		break;

	case SkytraqDecodeState::Payload:
		_rx_buffer[_rx_payload_index++] = b;
		_rx_checksum ^= b;

		if (_rx_payload_index >= _rx_payload_length) {
			_decode_state = SkytraqDecodeState::Checksum;
		}

		break;

	case SkytraqDecodeState::Checksum:
		if (b == _rx_checksum) {
			_decode_state = SkytraqDecodeState::End1;

		} else {
			decodeInit();
		}

		break;

	case SkytraqDecodeState::End1:
		if (b == 0x0D) {
			_decode_state = SkytraqDecodeState::End2;

		} else {
			decodeInit();
		}

		break;

	case SkytraqDecodeState::End2:
		if (b == 0x0A) {
			ret = _rx_payload_length;

		} else {
			decodeInit();
		}

		break;
	}

	return ret;
}

int GPSDriverSkyTraq::handleMessage()
{
	int ret = 0;

	if (_rx_payload_length == 0) {
		return ret;
	}

	const uint8_t msg_id = _rx_buffer[0];

	switch (msg_id) {
	case 0x83: // ACK
	case 0x84: { // NACK
			if (_rx_payload_length < 2) {
				break;
			}

			const uint8_t ack_msg_id = _rx_buffer[1];
			const uint8_t ack_sub_id = (_rx_payload_length >= 3) ? _rx_buffer[2] : 0xff;

			SKYTRAQ_DEBUG("SkyTraq ack/nack: msg_id=0x%02x ack_msg_id=0x%02x/0x%02x expected=0x%02x/0x%02x",
				      msg_id, ack_msg_id, ack_sub_id, _expected_ack_msg_id, _expected_ack_sub_id);

			if (_ack_pending && ack_msg_id == _expected_ack_msg_id && ack_sub_id == _expected_ack_sub_id) {
				_last_ack_ok = (msg_id == 0x83);
				_ack_pending = false;
				ret = 1;
			}
		}

		break;

	case 0x75: // messages with Sub-ID
		if (_rx_payload_length >= 2 && _rx_buffer[1] == 0x90) {
			// Receiver Info and Version (Sub-ID 0x90), sent unprompted right after the
			// ACK to the 0x20 probe.
			handleReceiverInfoMessage();

		} else if (_rx_payload_length >= 2 && _rx_buffer[1] == 0xA0) {
			// Navigation PVT Message (Sub-ID 0xA0)
			if (handleNavigationPvtMessage() && !_ack_pending) {
				ret = 1;
			}

		} else if (_rx_payload_length >= 2 && _rx_buffer[1] == 0xA1) {
			// Navigation Satellite SV Status Message (Sub-ID 0xA1)
			if (handleNavigationSvStatusMessage() && !_ack_pending) {
				ret = 2;
			}
		}

		break;

	default:
		SKYTRAQ_DEBUG("unhandled SkyTraq message 0x%02x", msg_id);
		break;
	}

	return ret;
}

bool GPSDriverSkyTraq::handleNavigationPvtMessage()
{
	// AN0037 "Navigation PVT Message" (ID 0x75, Sub-ID 0xA0): 86 bytes total, including
	// the ID/Sub-ID bytes.
	if (_rx_payload_length < 86) {
		// Warn once: the message repeats several times a second, and dropping it silently
		// leaves no clue why the receiver never produces a fix.
		if (!_warned_short_pvt) {
			_warned_short_pvt = true;
			SKYTRAQ_WARN("Navigation PVT Message is %u bytes, this driver needs 86 -- "
				     "the receiver firmware predates the PX4 message set",
				     _rx_payload_length);
		}

		return false;
	}

	unsigned i = 2; // _rx_buffer[0..1] are ID/Sub-ID, already consumed by handleMessage()

	i += 1; // Version: unused, only one revision of this message is defined
	i += 4; // Receiver TOW: unused, UTC calendar fields below are used instead

	const uint16_t year  = readU16BE(&_rx_buffer[i]); i += 2;
	const uint8_t month  = _rx_buffer[i++];
	const uint8_t day    = _rx_buffer[i++];
	const uint8_t hour   = _rx_buffer[i++];
	const uint8_t minute = _rx_buffer[i++];
	const uint8_t second = _rx_buffer[i++];
	const uint8_t valid  = _rx_buffer[i++];
	const uint32_t fractional_second_ns = readU32BE(&_rx_buffer[i]); i += 4;

	const uint8_t fix_mode = _rx_buffer[i++];
	const uint8_t num_sv   = _rx_buffer[i++];

	const double latitude_deg  = readF64BE(&_rx_buffer[i]); i += 8; // deg
	const double longitude_deg = readF64BE(&_rx_buffer[i]); i += 8; // deg

	const int32_t alt_ellipsoid_raw = (int32_t)readU32BE(&_rx_buffer[i]); i += 4; // mm
	const int32_t alt_msl_raw       = (int32_t)readU32BE(&_rx_buffer[i]); i += 4; // mm
	const uint32_t horizontal_acc_mm = readU32BE(&_rx_buffer[i]); i += 4;
	const uint32_t vertical_acc_mm   = readU32BE(&_rx_buffer[i]); i += 4;
	const int32_t vel_n_raw = (int32_t)readU32BE(&_rx_buffer[i]); i += 4; // mm/s
	const int32_t vel_e_raw = (int32_t)readU32BE(&_rx_buffer[i]); i += 4; // mm/s
	const int32_t vel_d_raw = (int32_t)readU32BE(&_rx_buffer[i]); i += 4; // mm/s

	i += 4; // Ground Speed: unused, derived from vel_n/vel_e below instead
	i += 2; // Ground Course: unused, course is derived from vel_n/vel_e at full resolution
	const uint32_t speed_acc_mm_s = readU32BE(&_rx_buffer[i]); i += 4;
	const uint16_t course_acc_raw = readU16BE(&_rx_buffer[i]); i += 2; // 0.01 deg
	const uint16_t hdop = readU16BE(&_rx_buffer[i]); i += 2;
	const uint16_t vdop = readU16BE(&_rx_buffer[i]); i += 2;
	const uint16_t heading_raw     = readU16BE(&_rx_buffer[i]); i += 2; // 0.01 deg
	const uint16_t heading_acc_raw = readU16BE(&_rx_buffer[i]); i += 2; // 0.01 deg
	const uint8_t flags = _rx_buffer[i++];

	// The 8-bit XOR checksum is weak enough that a corrupt frame can still reach here, and
	// an IEEE-754 field can decode to NaN. Never hand that to sensor_gnss_s.
	if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg)
	    || latitude_deg < -90.0 || latitude_deg > 90.0
	    || longitude_deg < -180.0 || longitude_deg > 180.0) {
		if (!_warned_bad_position) {
			_warned_bad_position = true;
			SKYTRAQ_WARN("discarding PVT message with out-of-range position");
		}

		return false;
	}

	_gps_position->timestamp = gps_absolute_time();
	_gps_position->latitude = latitude_deg;
	_gps_position->longitude = longitude_deg;
	_gps_position->altitude_msl = alt_msl_raw * 0.001;
	_gps_position->altitude_ellipsoid = alt_ellipsoid_raw * 0.001;

	// fix_mode: 0 = no fix, 1 = dead reckoning, 2 = 2D, 3 = 3D, 4 = 3D+DGNSS,
	// 5 = float RTK, 6 = fixed RTK.
	switch (fix_mode) {
	case 2:
		_gps_position->fix_type = 2;
		break;

	case 3:
		_gps_position->fix_type = 3;
		break;

	case 4:
		_gps_position->fix_type = 4; // sensor_gnss_s::FIX_TYPE_RTCM_CODE_DIFFERENTIAL
		break;

	case 5:
		_gps_position->fix_type = 5; // sensor_gnss_s::FIX_TYPE_RTK_FLOAT
		break;

	case 6:
		_gps_position->fix_type = 6; // sensor_gnss_s::FIX_TYPE_RTK_FIXED
		break;

	default:
		_gps_position->fix_type = 0;
		break;
	}

	_gps_position->satellites_used = num_sv;

	_gps_position->hdop = hdop * 0.01f;
	_gps_position->vdop = vdop * 0.01f;

	// Direct 1-sigma horizontal/vertical accuracy estimates, no DOP-based approximation needed.
	_gps_position->eph = horizontal_acc_mm * 0.001f;
	_gps_position->epv = vertical_acc_mm * 0.001f;

	_gps_position->vel_north = vel_n_raw * 0.001f;
	_gps_position->vel_east = vel_e_raw * 0.001f;
	_gps_position->vel_down = vel_d_raw * 0.001f;
	_gps_position->ground_speed = sqrtf(_gps_position->vel_north * _gps_position->vel_north
					    + _gps_position->vel_east * _gps_position->vel_east);
	_gps_position->course = atan2f(_gps_position->vel_east, _gps_position->vel_north);
	_gps_position->vel_ned_valid = (fix_mode >= 2); // same threshold the fix_type map uses
	_gps_position->speed_accuracy = speed_acc_mm_s * 0.001f;
	_gps_position->course_accuracy = course_acc_raw * 0.01f * M_DEG_TO_RAD_F;

	// Flags bits 1-2 and 3-4 use the same four levels sensor_gnss_s defines, so both pass
	// through unchanged.
	_gps_position->spoofing_state = (flags >> 1) & 0x03;
	_gps_position->jamming_state  = (flags >> 3) & 0x03;

	// BIT0: GPS time of week valid, BIT1: GPS week number valid.
	if ((valid & 0x1) && (valid & 0x2)) {
		struct tm timeinfo {};
		timeinfo.tm_year = year - 1900;
		timeinfo.tm_mon  = month - 1;
		timeinfo.tm_mday = day;
		timeinfo.tm_hour = hour;
		timeinfo.tm_min  = minute;
		timeinfo.tm_sec  = second;

		_gps_position->time_utc_usec = timeFromUtc(timeinfo, (int32_t)fractional_second_ns);

	} else {
		_gps_position->time_utc_usec = 0;
	}

	_gps_position->timestamp_time_relative = 0;

	// Vehicle Heading is the dual-antenna baseline heading, valid only when Flags bit 0 is
	// set. Navigation PVT carries no baseline length, so the length is left unreported
	// rather than zero, which a consumer checking it against a configured baseline rejects.
	const bool heading_valid = (flags & 0x01) != 0;

	sensor_gnss_relative_s gnss_relative{};

	// timestamp_sample is left unset: the receiver latency is not known here and is applied by the consumer
	gnss_relative.time_utc_usec = _gps_position->time_utc_usec;
	gnss_relative.position_length = NAN;
	gnss_relative.heading_valid = heading_valid;
	gnss_relative.heading = NAN;
	gnss_relative.heading_accuracy = NAN;

	if (heading_valid) {
		float heading_rad = heading_raw * 0.01f * M_DEG_TO_RAD_F; // reported over [0, 2pi)

		if (heading_rad > M_PI_F) {
			heading_rad -= 2.f * M_PI_F; // rad in range [-pi, pi]
		}

		gnss_relative.heading = heading_rad;
		gnss_relative.heading_accuracy = heading_acc_raw * 0.01f * M_DEG_TO_RAD_F;
	}

	gotRelativePositionMessage(gnss_relative);

	SKYTRAQ_DEBUG("SkyTraq PVT: fix_mode=%d fix_type=%d sats=%d hdop=%.2f eph=%.2f time_utc_usec=%llu",
		      fix_mode, _gps_position->fix_type, num_sv, (double)_gps_position->hdop,
		      (double)_gps_position->eph, _gps_position->time_utc_usec);

	_rate_count_lat_lon++;
	_rate_count_vel++;

	return true;
}

void GPSDriverSkyTraq::handleReceiverInfoMessage()
{
	// AN0037 "Receiver Info and Version" (ID 0x75, Sub-ID 0x90): 26 bytes total, including
	// the ID/Sub-ID bytes. Only the two version words are of interest here.
	if (_rx_payload_length < 26) {
		return;
	}

	_kernel_version = readU32BE(&_rx_buffer[2]);
	_odm_version    = readU32BE(&_rx_buffer[6]);
	_got_receiver_info = true;

	SKYTRAQ_DEBUG("SkyTraq kernel %u.%u.%u, ODM %u.%u.%u",
		      (unsigned)((_kernel_version >> 16) & 0xFF), (unsigned)((_kernel_version >> 8) & 0xFF),
		      (unsigned)(_kernel_version & 0xFF), (unsigned)((_odm_version >> 16) & 0xFF),
		      (unsigned)((_odm_version >> 8) & 0xFF), (unsigned)(_odm_version & 0xFF));
}

uint8_t GPSDriverSkyTraq::skytraqSvidToPx4Prn(uint8_t gnss_type, uint8_t svid)
{
	// AN0037's Remark below the Navigation SAT Message field table gives an allocation
	// for a host that needs one identifier across constellations: GPS 1-32, SBAS 33-58,
	// GLONASS 65-96, Galileo 120-159, BeiDou 160-192 and 205-239, QZSS 193-204, IRNSS
	// 240-255. The ranges do not overlap, so a PRN identifies its constellation on its
	// own. Other drivers here use schemes of their own, so the numbering is not shared.
	// The message reports each SV's native number within its constellation (field 10
	// "SVID"); offset it into the ranges above.
	switch (gnss_type & 0x0F) {
	case 0: // GPS: SkyTraq PRN 1-32 -> PX4 PRN 1-32
		if (svid >= 1 && svid <= 32) {
			return svid;
		}

		break;

	case 1: { // SBAS: SkyTraq PRN 120-145 -> PX4 PRN 33-58
			const int mapped = (int)svid - 120 + 33;

			if (mapped >= 33 && mapped <= 58) {
				return (uint8_t)mapped;
			}
		}
		break;

	case 2: // GLONASS: SkyTraq slot 1-24 -> PX4 PRN 65-96
		if (svid >= 1 && svid <= 24) {
			return (uint8_t)(svid + 64);
		}

		break;

	case 3: { // Galileo: SkyTraq PRN 1-36 -> PX4 PRN 120-159
			const int mapped = (int)svid + 119;

			if (mapped >= 120 && mapped <= 159) {
				return (uint8_t)mapped;
			}
		}
		break;

	case 4: // QZSS: SkyTraq PRN 193-202 -> PX4 PRN 193-204
		if (svid >= 193 && svid <= 204) {
			return svid;
		}

		break;

	case 5: { // BeiDou: SkyTraq PRN 1-63 -> PX4 PRN 160-192 and 205-239
			// The run steps over 193-204, which belongs to QZSS.
			const int mapped = (svid <= 33) ? (int)svid + 159 : (int)svid + 171;

			if ((mapped >= 160 && mapped <= 192) || (mapped >= 205 && mapped <= 239)) {
				return (uint8_t)mapped;
			}
		}
		break;

	case 6: { // IRNSS: SkyTraq PRN 1-14 -> PX4 PRN 240-255
			const int mapped = (int)svid + 239;

			if (mapped >= 240 && mapped <= 255) {
				return (uint8_t)mapped;
			}
		}
		break;
	}

	// Unmapped. 0 is the only value no constellation claims here; 255 is a real IRNSS PRN,
	// so it cannot serve as the sentinel other drivers use it for.
	return 0;
}

bool GPSDriverSkyTraq::handleNavigationSvStatusMessage()
{
	// AN0037 "Navigation Satellite SV Status Message" (ID 0x75, Sub-ID 0xA1):
	// 8 + NSVS*13 bytes total, including the ID/Sub-ID bytes.
	if (_satellite_info == nullptr) {
		return false;
	}

	if (_rx_payload_length < 8) {
		return false;
	}

	unsigned i = 2; // _rx_buffer[0..1] are ID/Sub-ID, already consumed by handleMessage()

	i += 1; // Version: unused
	i += 4; // Receiver TOW: unused

	const uint8_t num_svs = _rx_buffer[i++];

	if (_rx_payload_length < (uint16_t)(8 + num_svs * 13)) {
		// truncated/malformed message: don't index past the SV blocks actually present
		return false;
	}

	unsigned count = num_svs;

	if (count > satellite_info_s::SAT_INFO_MAX_SATELLITES) {
		count = satellite_info_s::SAT_INFO_MAX_SATELLITES;
	}

	memset(_satellite_info, 0, sizeof(*_satellite_info));
	_satellite_info->count = count;

	for (unsigned sat_index = 0; sat_index < num_svs; ++sat_index) {
		const uint8_t gnss_type = _rx_buffer[i++];
		const uint8_t svid      = _rx_buffer[i++];
		i += 2; // SV Status indicator (almanac/ephemeris/health flags): unused
		const uint8_t cn0             = _rx_buffer[i++];
		const int16_t elevation_raw   = (int16_t)readU16BE(&_rx_buffer[i]); i += 2;
		const int16_t azimuth_raw     = (int16_t)readU16BE(&_rx_buffer[i]); i += 2;
		i += 2; // Pseudorange residual: unused
		const uint16_t signal_status  = readU16BE(&_rx_buffer[i]); i += 2;

		if (sat_index >= count) {
			continue; // still advance i above so later SVs stay aligned
		}

		uint8_t elevation_deg = 0;

		if (elevation_raw > 0) {
			elevation_deg = (elevation_raw < 90) ? (uint8_t)elevation_raw : 90;
		}

		uint8_t azimuth_scaled = 0;

		if (azimuth_raw >= 0) {
			azimuth_scaled = (uint8_t)((float)azimuth_raw * 255.0f / 360.0f);
		}

		// Bit4: used in normal fix mode, Bit5: used in differential fix mode.
		_satellite_info->svid[sat_index]      = skytraqSvidToPx4Prn(gnss_type, svid);
		_satellite_info->prn[sat_index]       = _satellite_info->svid[sat_index];
		_satellite_info->used[sat_index]      = ((signal_status & 0x30) != 0) ? 1 : 0;
		_satellite_info->elevation[sat_index] = elevation_deg;
		_satellite_info->azimuth[sat_index]   = azimuth_scaled;
		_satellite_info->snr[sat_index]       = cn0;
	}

	_satellite_info->timestamp = gps_absolute_time();

	return true;
}

int GPSDriverSkyTraq::sendMessage(uint8_t msg_id, const uint8_t *payload, uint16_t payload_length)
{
	static constexpr size_t kMaxTxPayload = 16;

	if (payload_length > kMaxTxPayload) {
		return -1;
	}

	uint8_t frame[8 + kMaxTxPayload];

	const uint16_t pl = (uint16_t)(payload_length + 1); // + message ID byte

	frame[0] = 0xA0;
	frame[1] = 0xA1;
	frame[2] = (uint8_t)(pl >> 8);
	frame[3] = (uint8_t)(pl & 0xFF);
	frame[4] = msg_id;

	uint8_t checksum = msg_id;

	for (uint16_t i = 0; i < payload_length; ++i) {
		frame[5 + i] = payload[i];
		checksum ^= payload[i];
	}

	frame[5 + payload_length] = checksum;
	frame[6 + payload_length] = 0x0D;
	frame[7 + payload_length] = 0x0A;

	SKYTRAQ_DEBUG("SkyTraq tx: msg_id=0x%02x payload_len=%d", msg_id, payload_length);

	return write(frame, 8 + payload_length);
}

bool GPSDriverSkyTraq::sendMessageAndWaitForAck(uint8_t msg_id, uint8_t sub_id, const uint8_t *payload,
		uint16_t payload_length, unsigned timeout_per_try_ms, int num_tries)
{
	for (int try_index = 0; try_index < num_tries; ++try_index) {
		_expected_ack_msg_id = msg_id;
		_expected_ack_sub_id = sub_id;
		_ack_pending = true;
		_last_ack_ok = false;

		if (sendMessage(msg_id, payload, payload_length) < 0) {
			continue;
		}

		const gps_abstime time_started = gps_absolute_time();

		while (_ack_pending
		       && (gps_absolute_time() - time_started) < (gps_abstime)timeout_per_try_ms * 1000) {
			receive(20);
		}

		if (!_ack_pending && _last_ack_ok) {
			return true;
		}
	}

	return false;
}

int GPSDriverSkyTraq::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE];

	const gps_abstime time_started = gps_absolute_time();

	int handled = 0;

	while (true) {
		const int ret = read(buf, sizeof(buf), timeout);

		if (ret < 0) {
			SKYTRAQ_WARN("poll_or_read err");
			return -1;

		} else if (ret > 0) {
			for (int i = 0; i < ret; ++i) {
				const int len = parseChar(buf[i]);

				if (len > 0) {
					handled |= handleMessage();
					decodeInit();
				}
			}

			if (handled > 0) {
				return handled;
			}
		}

		if (time_started + timeout * 1000 < gps_absolute_time()) {
			return -1;
		}
	}
}

int GPSDriverSkyTraq::baudrateToIndex(unsigned baudrate)
{
	// AN0037 "Configure Serial Port" (ID 0x05), field 3 "Baud Rate".
	switch (baudrate) {
	case 4800:   return 0;

	case 9600:   return 1;

	case 19200:  return 2;

	case 38400:  return 3;

	case 57600:  return 4;

	case 115200: return 5;

	case 230400: return 6;

	case 460800: return 7;

	case 921600: return 8;
	}

	return -1;
}

bool GPSDriverSkyTraq::setDeviceBaudrate(unsigned baudrate)
{
	const int baud_index = baudrateToIndex(baudrate);

	if (baud_index < 0) {
		SKYTRAQ_WARN("baudrate %u is not one of AN0037's Configure Serial Port rates", baudrate);
		return false;
	}

	// Attributes = 0 ("update to SRAM"). AN0037 also lists 2 ("temporarily") for this
	// message, but that attribute is not usable for the baudrate. Note that SkyTraq's
	// SRAM is held up by the RTC/backup supply, so on a board that fits one the receiver
	// keeps this baudrate across power cycles and for whatever host it is attached to
	// next. Benign for PX4 -- configure() re-detects on every boot, and
	// SKYTRAQ_TARGET_BAUDRATE is the first rate probed, so the module is found
	// immediately -- but it is a visible side effect elsewhere.
	const uint8_t payload[3] = {0x00, (uint8_t)baud_index, 0x00}; // COM 1, baud index, update to SRAM

	// The receiver ACKs at the current baudrate and only then switches, so the ACK has to
	// arrive before this end moves. Exactly one try: a retry would go out at the old rate
	// after the receiver had already changed.
	if (!sendMessageAndWaitForAck(0x05, 0xff, payload, sizeof(payload), 300, 1)) {
		SKYTRAQ_WARN("SkyTraq refused the switch to %u baud, staying at %u", baudrate, _current_baudrate);
		return false;
	}

	setBaudrate(baudrate);
	_current_baudrate = baudrate;

	// Drop whatever was still in flight at the old rate.
	decodeInit();
	receive(20);
	decodeInit();

	SKYTRAQ_DEBUG("SkyTraq baudrate switched to %d", _current_baudrate);

	return true;
}

int GPSDriverSkyTraq::configure(unsigned &baudrate, const GPSConfig &config)
{
	if (config.output_mode != OutputMode::GPS) {
		SKYTRAQ_WARN("SkyTraq driver cannot act as an RTCM base; use plain GPS output");
		return -1;
	}

	// 4800 is deliberately absent: the Navigation PVT message alone is 88 bytes at 5 Hz =
	// 4400 bps, which already saturates a 4800 baud link before the Navigation SAT
	// message is added.
	static constexpr unsigned kFallbackBauds[] = {115200, 38400, 9600, 19200, 57600, 230400};

	// A non-zero requested baudrate (SER_GPSn_BAUD) pins the link to exactly that rate and
	// nothing else is probed; only auto-detection sweeps the fallback list and then moves
	// the receiver to a fixed target rate.
	const bool auto_baudrate = (baudrate == 0);
	const unsigned desired_baudrate = auto_baudrate ? SKYTRAQ_TARGET_BAUDRATE : baudrate;

	unsigned bauds_to_try[sizeof(kFallbackBauds) / sizeof(kFallbackBauds[0])];
	int num_bauds = 0;

	if (auto_baudrate) {
		for (unsigned fallback : kFallbackBauds) {
			bauds_to_try[num_bauds++] = fallback;
		}

	} else {
		bauds_to_try[num_bauds++] = baudrate;
	}

	_current_baudrate = 0;
	_got_receiver_info = false;

	// Probe for a receiver using the Query Receiver Info and Version message
	// (ID 0x75, Sub-ID 0x20), at every candidate baudrate.
	{
		const uint8_t version_query_payload[1] = {0x20};

		for (int i = 0; i < num_bauds && _current_baudrate == 0; ++i) {
			setBaudrate(bauds_to_try[i]);
			decodeInit();

			SKYTRAQ_DEBUG("SkyTraq receiver try baud rate idx = %d, baudrate = %d", i, bauds_to_try[i]);

			if (sendMessageAndWaitForAck(0x75, 0x20, version_query_payload, 1, 300, 2)) {
				_current_baudrate = bauds_to_try[i];

				// The receiver ACKs first and sends Receiver Info and Version (0x75/0x90)
				// immediately after, so keep reading briefly to pick that reply up.
				const gps_abstime info_started = gps_absolute_time();

				while (!_got_receiver_info
				       && (gps_absolute_time() - info_started) < 300 * 1000) {
					receive(20);
				}
			}
		}
	}

	if (_current_baudrate == 0) {
		SKYTRAQ_WARN("no SkyTraq receiver found");
		return -1;
	}

	SKYTRAQ_DEBUG("SkyTraq receiver found, baudrate = %d", _current_baudrate);

	// AN0037 marks the UAV firmware line by an ODM major version of at least 240; only
	// that line carries the message set below. Refusing here, rather than letting the
	// configuration run and leaving the receiver silent, puts the actual version in front
	// of whoever has to pick the right firmware.
	if (_got_receiver_info) {
		const unsigned odm_major = (_odm_version >> 16) & 0xFF;

		if (odm_major < SKYTRAQ_MIN_ODM_MAJOR) {
			SKYTRAQ_WARN("receiver firmware ODM %u.%u.%u is not a UAV build (major < %u)",
				     odm_major, (unsigned)((_odm_version >> 8) & 0xFF),
				     (unsigned)(_odm_version & 0xFF), (unsigned)SKYTRAQ_MIN_ODM_MAJOR);
			return -1;
		}

	} else {
		// Firmware old enough to predate the version reply: let configuration continue, so
		// a receiver that does speak the message set is not locked out by a missing reply.
		SKYTRAQ_WARN("no Receiver Info and Version reply; firmware may predate the UAV message set");
	}

	// Move the receiver onto the target baudrate. Non-fatal: setDeviceBaudrate() leaves
	// both ends on the detected rate if the receiver won't take the change, and
	// configuration continues from there.
	if (desired_baudrate != _current_baudrate) {
		setDeviceBaudrate(desired_baudrate);
	}

	baudrate = _current_baudrate;

	// Switch the output protocol to binary. Some RTK-only variants always output binary
	// and will NACK this command; that's not fatal.
	{
		const uint8_t payload[2] = {0x02, 0x00}; // binary messages, update to SRAM only
		sendMessageAndWaitForAck(0x09, 0xff, payload, sizeof(payload), 300, 2);
	}

	// Explicitly enable the SkyTraq Binary Navigation messages group via Configure
	// Receiver Message Type (0x75/0x21, Bit 1). This single bit covers both the
	// Navigation PVT Message (0x75/0xA0) and the Navigation Satellite SV Status Message
	// (0x75/0xA1) as one group -- there is no separate bit for SAT.
	{
		const uint8_t payload[4] = {0x21, 0x00, 0x02, 0x00}; // sub id, binary msg bit1 (nav msg group: PVT + SAT), SRAM only

		if (!sendMessageAndWaitForAck(0x75, 0x21, payload, sizeof(payload), 300, 3)) {
			SKYTRAQ_WARN("failed to enable SkyTraq navigation message group");
			return -1;
		}
	}

	// Request 5 Hz navigation solutions (see AN0037, msg 0x0E). Combined with the
	// Configure Navigation Message Rate call below, this yields PVT at 5 Hz (divisor 1)
	// and SAT every 2 s (divisor 10).
	// AN0037's note that rates of 4-10 Hz need >= 38400 baud was measured against NMEA
	// output and does not constrain the binary 0x75/0xA0 + 0x75/0xA1 stream, so the rate
	// is not gated on the detected baudrate.
	{
		const uint8_t payload[2] = {0x05, 0x00}; // 5 Hz, update to SRAM only
		sendMessageAndWaitForAck(0x0E, 0xff, payload, sizeof(payload), 300, 2);
	}

	// Configure the Navigation PVT (0x75/0xA0) and Navigation Satellite SV Status
	// (0x75/0xA1) message output rates via Configure Navigation Message Rate
	// (0x75/0x23). PVT is set to "every navigation solution", which at the 5 Hz fix
	// rate requested above outputs PVT at 5 Hz. SAT is set to "once every 10
	// navigation solutions", a 2-second output interval at 5 Hz (left at 0/disabled if
	// satellite info isn't requested). Not fatal if NACKed/unsupported: the PVT message
	// keeps flowing at whatever rate Bit 1 of Configure Receiver Message Type left it
	// at, and satellite_info simply stays unpopulated.
	{
		const uint16_t pvt_rate = 1;
		const uint16_t sat_rate = (_satellite_info != nullptr) ? 10 : 0;
		const uint8_t payload[7] = {
			0x23, // sub id
			0x01, // version
			(uint8_t)(pvt_rate >> 8), (uint8_t)(pvt_rate & 0xFF),
			(uint8_t)(sat_rate >> 8), (uint8_t)(sat_rate & 0xFF),
			0x00, // attributes: update to SRAM only
		};

		if (!sendMessageAndWaitForAck(0x75, 0x23, payload, sizeof(payload), 300, 3)) {
			SKYTRAQ_WARN("failed to configure SkyTraq navigation PVT/SAT message rate");
		}
	}

	return 0;
}
