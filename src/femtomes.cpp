/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
#include <cmath>

#include "femtomes.h"
#include "rtcm.h"


/* ms, timeout for waiting for a response*/
#define FEMTO_RESPONSE_TIMEOUT		200

#define FEMTO_MSG_MAX_LENGTH		256
/* Femtomes ID for UAV output message */
#define FEMTO_MSG_ID_UAVGPS 		8001
#define FEMTO_MSG_ID_RTCM3          784
#define FEMTO_MSG_ID_GPGGA          218
#define FEMTO_MSG_ID_UAVSTATUS		8017

/* Femto uavgps message frame premble 3 bytes*/
#define FEMTO_PREAMBLE1			0xaa
#define FEMTO_PREAMBLE2			0x44
#define FEMTO_PREAMBLE3			0x12

#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
#define FEMTO_UNUSED(x) (void)x;

#if defined _FMTOMES_DEBUG
#define FEMTO_INFO(...)			{GPS_INFO(__VA_ARGS__);}
#define FEMTO_DEBUG(...)		{GPS_WARN(__VA_ARGS__);}
#define FEMTO_ERR(...)			{GPS_ERR(__VA_ARGS__);}
#else
#define FEMTO_INFO(...)			{(void)0;}
#define FEMTO_DEBUG(...)		{(void)0;}
#define FEMTO_ERR(...)			{GPS_WARN(__VA_ARGS__);}
#endif


GPSDriverFemto::GPSDriverFemto(GPSCallbackPtr callback, void *callback_user,
			       struct sensor_gps_s *gps_position,
			       satellite_info_s *satellite_info,
			       float heading_offset) :
	GPSBaseStationSupport(callback, callback_user),
	_gps_position(gps_position),
	_satellite_info(satellite_info),
	_heading_offset(heading_offset)
{
	decodeInit();
}

GPSDriverFemto::~GPSDriverFemto()
{
	delete _rtcm_parsing;
}

int GPSDriverFemto::handleMessage(int len)
{
	int ret = 0;
	uint16_t messageid = _femto_msg.header.femto_header.messageid;

	if (messageid == FEMTO_MSG_ID_UAVGPS) { /**< uavgpsB*/
		memcpy(&_femto_uav_gps, _femto_msg.data, sizeof(femto_uav_gps_t));

		_gps_position->time_utc_usec = _femto_uav_gps.time_utc_usec;
		_gps_position->lat = _femto_uav_gps.lat;
		_gps_position->lon = _femto_uav_gps.lon;
		_gps_position->alt = _femto_uav_gps.alt;
		_gps_position->alt_ellipsoid = _femto_uav_gps.alt_ellipsoid;
		_gps_position->s_variance_m_s = _femto_uav_gps.s_variance_m_s;
		_gps_position->c_variance_rad = _femto_uav_gps.c_variance_rad;
		_gps_position->eph = _femto_uav_gps.eph;
		_gps_position->epv = _femto_uav_gps.epv;
		_gps_position->hdop = _femto_uav_gps.hdop;
		_gps_position->vdop = _femto_uav_gps.vdop;
		_gps_position->noise_per_ms = _femto_uav_gps.noise_per_ms;
		_gps_position->jamming_indicator = _femto_uav_gps.jamming_indicator;
		_gps_position->vel_m_s = _femto_uav_gps.vel_m_s;
		_gps_position->vel_n_m_s = _femto_uav_gps.vel_n_m_s;
		_gps_position->vel_e_m_s = _femto_uav_gps.vel_e_m_s;
		_gps_position->vel_d_m_s = _femto_uav_gps.vel_d_m_s;
		_gps_position->cog_rad = _femto_uav_gps.cog_rad;
		_gps_position->timestamp_time_relative = _femto_uav_gps.timestamp_time_relative;
		_gps_position->fix_type = _femto_uav_gps.fix_type;
		_gps_position->vel_ned_valid = _femto_uav_gps.vel_ned_valid;
		_gps_position->satellites_used = _femto_uav_gps.satellites_used;

		if (_femto_uav_gps.heading_type == 6) {
			float heading = _femto_uav_gps.heading;
			heading *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]
			heading -= _heading_offset; // range: [-pi, 3pi]

			if (heading > M_PI_F) {
				heading -= 2.f * M_PI_F; // final range is [-pi, pi]
			}

			_gps_position->heading = heading;

		} else {
			_gps_position->heading = NAN;
		}

		_gps_position->timestamp = gps_absolute_time();

		ret = 1;

	} else if (_satellite_info && messageid == FEMTO_MSG_ID_UAVSTATUS) {	/**< set satellite info */
		const femto_uav_status_t *uav_status = (const femto_uav_status_t *)_femto_msg.data;

		_satellite_info->count = MIN(uav_status->sat_number, satellite_info_s::SAT_INFO_MAX_SATELLITES);

		for (int8_t i = 0; i < _satellite_info->count; i++) {
			_satellite_info->svid[i] = uav_status->sat_status[i].svid;
			_satellite_info->used[i] = 1;
			_satellite_info->elevation[i] = uav_status->sat_status[i].ele;
			_satellite_info->azimuth[i] = uav_status->sat_status[i].azi;
			_satellite_info->snr[i] = uav_status->sat_status[i].cn0;
			_satellite_info->prn[i] = uav_status->sat_status[i].system_id;
		}

		ret = 2;

	} else if (OutputMode::RTCM == _output_mode
		   && messageid == FEMTO_MSG_ID_GPGGA
		   && (memcmp(_femto_msg.data + 3, "GGA,", 3) == 0)) { /**< GPGGA only used in base station, for survey-in */
		int uiCalcComma = 0;

		for (int i = 0 ; i < len; i++) {
			if (_femto_msg.data[i] == ',') { uiCalcComma++; }
		}

		if (uiCalcComma == 14) {
			char *bufptr = (char *)(_femto_msg.data + 6);
			char *endp = nullptr;
			double ashtech_time = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
			int num_of_sv = 0, fix_quality = 0;
			double hdop = 99.9;
			char ns = '?', ew = '?';

			if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

			if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

			if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }

			if (bufptr && *(++bufptr) != ',') { alt = strtod(bufptr, &endp); bufptr = endp; }

			if (ns == 'S') {
				lat = -lat;
			}

			if (ew == 'W') {
				lon = -lon;
			}

			FEMTO_UNUSED(ashtech_time)
			FEMTO_UNUSED(hdop)

			if (!_correction_output_activated && 7 == fix_quality) {
				_survey_in_start = 0;	/**< finished survey-in */

				lat = (int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000;
				lon = (int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000;
				alt = alt * 1000;

				sendSurveyInStatusUpdate(false, true, lat, lon, (float)alt);
				activateRTCMOutput();
			}

			if (_satellite_info) {
				_satellite_info->count = (uint8_t)num_of_sv;    /**< base station satellite count */
			}

			ret = 2;
		}
	}

	// handle survey-in status update
	if (_survey_in_start != 0) {
		const gps_abstime now = gps_absolute_time();
		uint32_t survey_in_duration = (now - _survey_in_start) / 1000000;

		if (survey_in_duration != _base_settings.settings.survey_in.min_dur) {
			_base_settings.settings.survey_in.min_dur = survey_in_duration;
			sendSurveyInStatusUpdate(true, false);
		}
	}

	return ret;
}

void GPSDriverFemto::receiveWait(unsigned timeout_min)
{
	gps_abstime time_started = gps_absolute_time();

	while (gps_absolute_time() < time_started + timeout_min * 1000) {
		receive(timeout_min);
	}

}

int GPSDriverFemto::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE];

	/* timeout additional to poll */
	uint64_t time_started = gps_absolute_time();
	int j = 0;
	int bytes_count = 0;

	while (true) {

		/* pass received bytes to the packet decoder */
		while (j < bytes_count) {
			int l = 0;

			if ((l = parseChar(buf[j])) > 0) {
				/* return to configure during configuration or to the gps driver during normal work
				 * if a packet has arrived */
				int ret = handleMessage(l);

				if (ret > 0) {
					_decode_state = FemtoDecodeState::pream_ble1;
					return ret;
				}
			}

			j++;
		}

		/* everything is read */
		j = bytes_count = 0;

		/* then poll or read for new data */
		int ret = read(buf, sizeof(buf), timeout * 2);

		if (ret < 0) {
			/* something went wrong when polling */
			return -1;

		} else if (ret == 0) {
			/* Timeout while polling or just nothing read if reading, let's
			 * stay here, and use timeout below. */

		} else {
			/* if we have new data from GPS, go handle it */
			bytes_count = ret;
		}

		/* in case we get crap from GPS or time out */
		if (time_started + timeout * 1000 * 4 < gps_absolute_time()) {
			FEMTO_DEBUG("Femto: timeout\n");
			return -1;
		}
	}
}

#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

int GPSDriverFemto::parseChar(uint8_t temp)
{
	int iRet = 0;

	if (_output_mode == OutputMode::GPS) {

		switch (_decode_state) {
		case FemtoDecodeState::pream_ble1:
			if (temp == FEMTO_PREAMBLE1) {
				_decode_state = FemtoDecodeState::pream_ble2;
				_femto_msg.read = 0;
			}

			break;

		case FemtoDecodeState::pream_ble2:
			if (temp == FEMTO_PREAMBLE2) {
				_decode_state = FemtoDecodeState::pream_ble3;

			} else {
				_decode_state = FemtoDecodeState::pream_ble1;
			}

			break;

		case FemtoDecodeState::pream_ble3:
			if (temp == FEMTO_PREAMBLE3) {
				_decode_state = FemtoDecodeState::head_length;

			} else {
				_decode_state = FemtoDecodeState::pream_ble1;
			}

			break;

		case FemtoDecodeState::head_length:
			_femto_msg.header.data[0] = FEMTO_PREAMBLE1;
			_femto_msg.header.data[1] = FEMTO_PREAMBLE2;
			_femto_msg.header.data[2] = FEMTO_PREAMBLE3;
			_femto_msg.header.data[3] = temp;
			_femto_msg.header.femto_header.headerlength = temp;
			_decode_state = FemtoDecodeState::head_data;
			_femto_msg.read = 4;
			break;

		case FemtoDecodeState::head_data:
			if (_femto_msg.read >= sizeof(_femto_msg.header.data)) {
				_decode_state = FemtoDecodeState::pream_ble1;
				break;
			}

			_femto_msg.header.data[_femto_msg.read] = temp;
			_femto_msg.read++;

			if (_femto_msg.read >= _femto_msg.header.femto_header.headerlength) {
				_decode_state = FemtoDecodeState::data;
			}

			break;

		case FemtoDecodeState::data:
			if (_femto_msg.read >= FEMTO_MSG_MAX_LENGTH) {
				_decode_state = FemtoDecodeState::pream_ble1;
				break;
			}

			_femto_msg.data[_femto_msg.read - _femto_msg.header.femto_header.headerlength] = temp;
			_femto_msg.read++;

			if (_femto_msg.read >= (_femto_msg.header.femto_header.messagelength + _femto_msg.header.femto_header.headerlength)) {
				_decode_state = FemtoDecodeState::crc1;
			}

			break;

		case FemtoDecodeState::crc1:
			_femto_msg.crc = (uint32_t)(temp << 0);
			_decode_state = FemtoDecodeState::crc2;
			break;

		case FemtoDecodeState::crc2:
			_femto_msg.crc += (uint32_t)(temp << 8);
			_decode_state = FemtoDecodeState::crc3;
			break;

		case FemtoDecodeState::crc3:
			_femto_msg.crc += (uint32_t)(temp << 16);
			_decode_state = FemtoDecodeState::crc4;
			break;

		case FemtoDecodeState::crc4: {
				_femto_msg.crc += (uint32_t)(temp << 24);
				_decode_state = FemtoDecodeState::pream_ble1;

				uint32_t crc = calculateBlockCRC32((uint32_t)_femto_msg.header.femto_header.headerlength,
								   (uint8_t *)&_femto_msg.header.data, (uint32_t)0);
				crc = calculateBlockCRC32((uint32_t)_femto_msg.header.femto_header.messagelength, (uint8_t *)&_femto_msg.data[0], crc);

				if (_femto_msg.crc == crc) {
					iRet = _femto_msg.read;

				} else {
					FEMTO_DEBUG("Femto: data packet is bad");
				}
			}
			break;

		default:
			break;
		}

	} else {	/**< RTCM mode */

		switch (_decode_state) {
		case FemtoDecodeState::pream_ble1:
			if (temp == '$') {
				_decode_state = FemtoDecodeState::pream_nmea_got_sync1;
				_femto_msg.read = 0;
				_femto_msg.data[_femto_msg.read++] = temp;

			} else if (temp == RTCM3_PREAMBLE && _rtcm_parsing) {
				_decode_state = FemtoDecodeState::decode_rtcm3;
				_rtcm_parsing->addByte(temp);
			}

			break;

		case FemtoDecodeState::pream_nmea_got_sync1:
			if (temp == '$') {
				_decode_state = FemtoDecodeState::pream_nmea_got_sync1;
				_femto_msg.read = 0;

			} else if (temp == '*') {
				_decode_state = FemtoDecodeState::pream_nmea_got_asteriks;

			} else if (temp == RTCM3_PREAMBLE && _rtcm_parsing) {
				_decode_state = FemtoDecodeState::decode_rtcm3;
				_rtcm_parsing->addByte(temp);
			}

			if (_femto_msg.read >= (sizeof(_femto_msg.data) - 5)) {
				FEMTO_DEBUG("buffer overflow")
				_decode_state = FemtoDecodeState::pream_ble1;
				_femto_msg.read = 0;

			} else {
				_femto_msg.data[_femto_msg.read++] = temp;
			}

			break;

		case FemtoDecodeState::pream_nmea_got_asteriks:
			_femto_msg.data[_femto_msg.read++] = temp;
			_decode_state = FemtoDecodeState::pream_nmea_got_first_cs_byte;
			break;

		case FemtoDecodeState::pream_nmea_got_first_cs_byte: {
				_femto_msg.data[_femto_msg.read++] = temp;
				uint8_t checksum = 0;
				uint8_t *buffer = _femto_msg.data + 1;
				uint8_t *bufend = _femto_msg.data + _femto_msg.read - 3;

				for (; buffer < bufend; buffer++) {
					checksum ^= *buffer;
				}

				if ((HEXDIGIT_CHAR(checksum >> 4) == *(_femto_msg.data + _femto_msg.read - 2)) &&
				    (HEXDIGIT_CHAR(checksum & 0x0F) == *(_femto_msg.data + _femto_msg.read - 1))) {
					iRet = _femto_msg.read;
					_femto_msg.header.femto_header.messageid = FEMTO_MSG_ID_GPGGA;
					FEMTO_DEBUG("Femto: got NMEA message with length %i", _femto_msg.read)
				}

				decodeInit();
				break;
			}

		case FemtoDecodeState::decode_rtcm3:
			if (_rtcm_parsing->addByte(temp)) {
				FEMTO_DEBUG("Femto: got RTCM message with length %i", (int)_rtcm_parsing->messageLength())
				gotRTCMMessage(_rtcm_parsing->message(), _rtcm_parsing->messageLength());
				decodeInit();
			}

			break;

		default:
			break;
		}

	}


	return iRet;
}

void GPSDriverFemto::decodeInit()
{
	_decode_state = FemtoDecodeState::pream_ble1;

	/** init or reset rtcm parsing */
	if (_output_mode == OutputMode::RTCM) {
		if (!_rtcm_parsing) {
			_rtcm_parsing = new RTCMParsing();
		}

		if (_rtcm_parsing) {
			_rtcm_parsing->reset();
		}
	}
}

int GPSDriverFemto::writeAckedCommandFemto(const char *command, const char *reply, const unsigned int timeout)
{
	/**< write command*/
	write(command, strlen(command));
	/**< wait for reply*/
	uint8_t buf[GPS_READ_BUFFER_SIZE];
	gps_abstime time_started = gps_absolute_time();

	while (time_started + timeout * 2000 > gps_absolute_time()) {
		int ret = read(buf, sizeof(buf), timeout);
		buf[sizeof(buf) - 1] = 0;

		if (ret > 0 && strstr((char *)buf, reply) != nullptr) {
			FEMTO_DEBUG("Femto: command reply success: %s", command);
			return 0;
		}
	}

	return -1;
}

int GPSDriverFemto::configure(unsigned &baudrate, const GPSConfig &config)
{
	FEMTO_DEBUG("Femto: configure gps driver")

	if (config.output_mode != OutputMode::GPS && config.output_mode != OutputMode::RTCM) {
		FEMTO_DEBUG("Femto: Unsupported Output Mode %i", (int)config.output_mode);
		return -1;
	}

	_output_mode = config.output_mode;
	_configure_done = false;
	_correction_output_activated = false;
	/** Try different baudrates (115200 is the default for Femtomes) and request the baudrate that we want.	 */
	const unsigned baudrates_to_try[] = {115200};
	bool success = false;

	unsigned test_baudrate = 0;

	for (unsigned int baud_i = 0; !success && baud_i < sizeof(baudrates_to_try) / sizeof(baudrates_to_try[0]); baud_i++) {
		test_baudrate = baudrates_to_try[baud_i];

		if (baudrate > 0 && baudrate != test_baudrate) {
			continue; /**< skip to next baudrate*/
		}

		setBaudrate(test_baudrate);

		FEMTO_DEBUG("Femto: baudrate set to %i", test_baudrate);

		for (int run = 0; run < 2; ++run) { /** try several times*/
			if (writeAckedCommandFemto("UNLOGALL THISPORT\r\n", "<UNLOGALL OK", FEMTO_RESPONSE_TIMEOUT) == 0 &&
			    writeAckedCommandFemto("VERSION\r\n", "<VERSION OK", FEMTO_RESPONSE_TIMEOUT) == 0) {
				FEMTO_DEBUG("Femto: got port for baudrate %i", test_baudrate);
				success = true;
				break;
			}
		}
	}

	if (!success) {
		FEMTO_DEBUG("Femto: gps driver configure failed,no port for baudrate %i", test_baudrate);
		return -1;
	}

	/**
	* We successfully got a response and know to which port we are connected. Now set the desired baudrate
	* if it's different from the current one.
	*/
	const unsigned desired_baudrate = 115200; /**< changing this requires also changing the SPD command*/

	baudrate = test_baudrate;

	if (baudrate != desired_baudrate) {
		baudrate = desired_baudrate;
		const char baud_config[] = "com 115200\r\n"; // configure baudrate to 115200
		write(baud_config, sizeof(baud_config));
		decodeInit();
		receiveWait(200);
		decodeInit();
		setBaudrate(baudrate);

		success = false;

		for (int run = 0; run < 10; ++run) {
			/** We ask for the port config again. If we get a reply, we know that the changed settings work.*/
			if (writeAckedCommandFemto("UNLOGALL THISPORT\r\n", "<UNLOGALL OK", FEMTO_RESPONSE_TIMEOUT) == 0 &&
			    writeAckedCommandFemto("VERSION\r\n", "<VERSION OK", FEMTO_RESPONSE_TIMEOUT) == 0) {
				success = true;
				break;
			}
		}

		if (!success) {
			return -1;
		}

	} else {
		decodeInit();
	}

	if (_output_mode == OutputMode::GPS) {
		if (writeAckedCommandFemto("LOG UAVGPSB 0.1\r\n", "<LOG OK", FEMTO_RESPONSE_TIMEOUT) == 0) {
			FEMTO_DEBUG("Femto: command LOG UAVGPSB 0.1 success");

			/** 20Hz need authorization in femtomes device */
			if (writeAckedCommandFemto("LOG UAVGPSB 0.05\r\n", "<LOG OK", FEMTO_RESPONSE_TIMEOUT) == 0) {
				FEMTO_DEBUG("Femto: command LOG UAVGPSB 0.05 success");

			} else {
				FEMTO_ERR("Femto: command LOG UAVGPSB 0.05 failed,maybe no authorization");
			}

		} else {
			FEMTO_ERR("Femto: command LOG UAVGPSB 0.1 failed");
		}

		if (_satellite_info) {
			if (writeAckedCommandFemto("LOG UAVSTATUSB 1\r\n", "<LOG OK", FEMTO_RESPONSE_TIMEOUT) == 0) {
				FEMTO_DEBUG("Femto: command LOG UAVSTATUSB 1 success");

			} else {
				FEMTO_ERR("Femto: command LOG UAVSTATUSB 1 failed");
			}
		}


	} else {	/**< RTCM mode for base station */
		activateCorrectionOutput();
	}

	_configure_done = true;
	FEMTO_DEBUG("Femto: gps driver configure done")

	return 0;
}

#define CRC32_POLYNOMIAL 0xEDB88320L
uint32_t
GPSDriverFemto::crc32Value(uint32_t icrc)
{
	int i;
	uint32_t crc = icrc;

	for (i = 8 ; i > 0; i--) {
		if (crc & 1) {
			crc = (crc >> 1) ^ CRC32_POLYNOMIAL;

		} else {
			crc >>= 1;
		}
	}

	return crc;
}

uint32_t
GPSDriverFemto::calculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc)
{
	while (length-- != 0) {
		crc = ((crc >> 8) & 0x00FFFFFFL) ^ (crc32Value(((uint32_t) crc ^ *buffer++) & 0xff));
	}

	return (crc);
}

void GPSDriverFemto::activateCorrectionOutput()
{
	if (_output_mode != OutputMode::RTCM) {
		return;	/**< only for base station */
	}

	char buffer[100];

	if (_base_settings.type == BaseSettingsType::survey_in) {
		FEMTO_DEBUG("Femto: enabling survey-in")

		if (writeAckedCommandFemto("POSAVE ON \r\n", "<POSAVE OK", FEMTO_RESPONSE_TIMEOUT) == 0) {
			FEMTO_DEBUG("Femto: command POSAVE ON success")

			if (writeAckedCommandFemto("LOG GPGGA 1 \r\n", "<LOG OK",
						   FEMTO_RESPONSE_TIMEOUT) == 0) { /**< for updating GPS satellite count of RTK */
				FEMTO_DEBUG("Femto: command LOG GPGGA 1 success")

			} else {
				FEMTO_ERR("Femto: LOG GPGGA command failed")
			}

		} else {
			FEMTO_ERR("Femto: command POSAVE ON failed")
		}

		_base_settings.settings.survey_in.min_dur = 0; // use it as counter how long survey-in has been active
		_survey_in_start = gps_absolute_time();
		sendSurveyInStatusUpdate(true, false);

	} else {
		FEMTO_DEBUG("Femto: setting base station position")

		const FixedPositionSettings &settings = _base_settings.settings.fixed_position;
		int len = snprintf(buffer, sizeof(buffer), "FIX POSITION %.8lf %.8lf %.5f\r\n",
				   settings.latitude, settings.longitude, (double)settings.altitude);

		if (len >= 0 && len < (int)(sizeof(buffer))) {
			if (writeAckedCommandFemto(buffer, "FIX OK", FEMTO_RESPONSE_TIMEOUT) == 0) {
				FEMTO_DEBUG("Femto: command %s success", buffer)
				activateRTCMOutput();
				sendSurveyInStatusUpdate(false, true, settings.latitude, settings.longitude, settings.altitude);

				if (writeAckedCommandFemto("LOG GPGGA 1 \r\n", "<LOG OK",
							   FEMTO_RESPONSE_TIMEOUT) == 0) { /**< for updating GPS satellite count of RTK */
					FEMTO_DEBUG("Femto: command LOG GPGGA 1 success")

				} else {
					FEMTO_ERR("Femto: LOG GPGGA command failed")
				}

			} else {
				FEMTO_ERR("Femto: fix base station position failed.")
			}

		} else {
			FEMTO_DEBUG("Femto: snprintf failed (buffer too short)")
		}


	}
}

void GPSDriverFemto::activateRTCMOutput()
{
	if (writeAckedCommandFemto("LOG RTCM 1\r\n", "<LOG OK", FEMTO_RESPONSE_TIMEOUT) != 0) {
		FEMTO_ERR("Femto: command LOG RTCM failed")

	} else {
		FEMTO_DEBUG("Femto: command LOG RTCM 1 success")
	}

	_correction_output_activated = true;
}

void GPSDriverFemto::sendSurveyInStatusUpdate(bool active, bool valid, double latitude, double longitude,
		float altitude)
{
	SurveyInStatus status;
	status.latitude = latitude;
	status.longitude = longitude;
	status.altitude = altitude;
	status.duration = _base_settings.settings.survey_in.min_dur;
	status.mean_accuracy = 0; // unknown
	status.flags = (int)valid | ((int)active << 1);
	surveyInStatus(status);
}
