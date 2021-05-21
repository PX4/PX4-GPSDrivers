/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file ubx.cpp
 *
 * U-Blox protocol implementation. Following u-blox 6/7/8/9 Receiver Description
 * including Prototol Specification.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Beat Kueng <beat-kueng@gmx.net>
 *
 * @author Hannes Delago
 *   (rework, add ubx7+ compatibility)
 *
 * @see https://www.u-blox.com/sites/default/files/products/documents/u-blox6-GPS-GLONASS-QZSS-V14_ReceiverDescrProtSpec_%28GPS.G6-SW-12013%29_Public.pdf
 * @see https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf
 * @see https://www.u-blox.com/sites/default/files/ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf
 */

#include <string.h>

#include "rtcm.h"
#include "ubx.h"

#define MIN(X,Y)              ((X) < (Y) ? (X) : (Y))
#define SWAP16(X)             ((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))

/**** Trace macros, disable for production builds */
#define UBX_TRACE_PARSER(...) {/*GPS_INFO(__VA_ARGS__);*/}    // decoding progress in parse_char()
#define UBX_TRACE_RXMSG(...)  {/*GPS_INFO(__VA_ARGS__);*/}    // Rx msgs in payload_rx_done()
#define UBX_TRACE_SVINFO(...) {/*GPS_INFO(__VA_ARGS__);*/}    // NAV-SVINFO processing (debug use only, will cause rx buffer overflows)

/**** Warning macros, disable to save memory */
#define UBX_WARN(...)         {GPS_WARN(__VA_ARGS__);}
#define UBX_DEBUG(...)        {/*GPS_WARN(__VA_ARGS__);*/}

GPSDriverUBX::GPSDriverUBX(Interface gpsInterface, GPSCallbackPtr callback, void *callback_user,
			   sensor_gps_s *gps_position, satellite_info_s *satellite_info, uint8_t dynamic_model,
			   float heading_offset, UBXMode mode) :
	GPSBaseStationSupport(callback, callback_user),
	_interface(gpsInterface),
	_gps_position(gps_position),
	_satellite_info(satellite_info),
	_dyn_model(dynamic_model),
	_mode(mode),
	_heading_offset(heading_offset)
{
	decodeInit();
}

GPSDriverUBX::~GPSDriverUBX()
{
	delete _rtcm_parsing;
}

int
GPSDriverUBX::configure(unsigned &baudrate, const GPSConfig &config)
{
	_configured = false;
	_output_mode = config.output_mode;

	ubx_payload_tx_cfg_prt_t cfg_prt[2];

	uint16_t out_proto_mask = _output_mode == OutputMode::GPS ?
				  UBX_TX_CFG_PRT_PROTO_UBX :
				  (UBX_TX_CFG_PRT_PROTO_UBX | UBX_TX_CFG_PRT_PROTO_RTCM);

	uint16_t in_proto_mask = (_output_mode == OutputMode::GPS || _output_mode == OutputMode::GPSAndRTCM) ?
				 (UBX_TX_CFG_PRT_PROTO_UBX | UBX_TX_CFG_PRT_PROTO_RTCM) :
				 UBX_TX_CFG_PRT_PROTO_UBX;

	const bool auto_baudrate = baudrate == 0;

	if (_interface == Interface::UART) {

		/* try different baudrates */
		const unsigned baudrates[] = {38400, 57600, 9600, 115200, 230400};

		unsigned baud_i;
		unsigned desired_baudrate = auto_baudrate ? UBX_BAUDRATE_M8_AND_NEWER : baudrate;

		for (baud_i = 0; baud_i < sizeof(baudrates) / sizeof(baudrates[0]); baud_i++) {
			unsigned test_baudrate = baudrates[baud_i];

			if (!auto_baudrate && baudrate != test_baudrate) {
				continue; // skip to next baudrate
			}

			UBX_DEBUG("baudrate set to %i", test_baudrate);

			setBaudrate(test_baudrate);

			/* flush input and wait for at least 20 ms silence */
			decodeInit();
			receive(20);
			decodeInit();

			// try CFG-VALSET: if we get an ACK we know we can use protocol version 27+
			int cfg_valset_msg_size = initCfgValset();
			// UART1
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1_STOPBITS, 1, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1_DATABITS, 0, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1_PARITY, 0, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_UBX, 1, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_RTCM3X, _output_mode == OutputMode::RTCM ? 0 : 1,
					   cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_NMEA, 0, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_UBX, 1, cfg_valset_msg_size);

			if (_output_mode != OutputMode::GPS) {
				cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X, 1, cfg_valset_msg_size);
			}

			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_NMEA, 0, cfg_valset_msg_size);
			// TODO: are we ever connected to UART2?

			// USB
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBINPROT_UBX, 1, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBINPROT_RTCM3X, _output_mode == OutputMode::RTCM ? 0 : 1,
					   cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBINPROT_NMEA, 0, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBOUTPROT_UBX, 1, cfg_valset_msg_size);

			if (_output_mode != OutputMode::GPS) {
				cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBOUTPROT_RTCM3X, 1, cfg_valset_msg_size);
			}

			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBOUTPROT_NMEA, 0, cfg_valset_msg_size);

			bool cfg_valset_success = false;

			if (sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {

				if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) == 0) {
					cfg_valset_success = true;
				}
			}

			if (cfg_valset_success) {
				_proto_ver_27_or_higher = true;
				// Now we only have to change the baudrate
				cfg_valset_msg_size = initCfgValset();
				cfgValset<uint32_t>(UBX_CFG_KEY_CFG_UART1_BAUDRATE, desired_baudrate, cfg_valset_msg_size);

				if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
					continue;
				}

				/* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
				waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false);

			} else {
				_proto_ver_27_or_higher = false;

				UBX_DEBUG("trying old protocol");

				/* Send a CFG-PRT message to set the UBX protocol for in and out
				 * and leave the baudrate as it is, we just want an ACK-ACK for this */
				memset(cfg_prt, 0, 2 * sizeof(ubx_payload_tx_cfg_prt_t));
				cfg_prt[0].portID		= UBX_TX_CFG_PRT_PORTID;
				cfg_prt[0].mode		= UBX_TX_CFG_PRT_MODE;
				cfg_prt[0].baudRate	= test_baudrate;
				cfg_prt[0].inProtoMask	= in_proto_mask;
				cfg_prt[0].outProtoMask	= out_proto_mask;
				cfg_prt[1].portID		= UBX_TX_CFG_PRT_PORTID_USB;
				cfg_prt[1].mode		= UBX_TX_CFG_PRT_MODE;
				cfg_prt[1].baudRate	= test_baudrate;
				cfg_prt[1].inProtoMask	= in_proto_mask;
				cfg_prt[1].outProtoMask	= out_proto_mask;

				if (!sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt, 2 * sizeof(ubx_payload_tx_cfg_prt_t))) {
					continue;
				}

				if (waitForAck(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false) < 0) {
					/* try next baudrate */
					continue;
				}

				if (auto_baudrate) {
					desired_baudrate = UBX_TX_CFG_PRT_BAUDRATE;
				}

				/* Send a CFG-PRT message again, this time change the baudrate */
				cfg_prt[0].baudRate	= desired_baudrate;
				cfg_prt[1].baudRate	= desired_baudrate;

				if (!sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt, 2 * sizeof(ubx_payload_tx_cfg_prt_t))) {
					continue;
				}

				/* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
				waitForAck(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false);
			}

			if (desired_baudrate != test_baudrate) {
				setBaudrate(desired_baudrate);

				decodeInit();
				receive(20);
				decodeInit();
			}

			/* at this point we have correct baudrate on both ends */
			baudrate = desired_baudrate;
			break;
		}

		if (baud_i >= sizeof(baudrates) / sizeof(baudrates[0])) {
			return -1;	// connection and/or baudrate detection failed
		}

	} else if (_interface == Interface::SPI) {

		// try CFG-VALSET: if we get an ACK we know we can use protocol version 27+
		int cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_SPI_ENABLED, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_SPI_MAXFF, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIINPROT_UBX, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIINPROT_RTCM3X, _output_mode == OutputMode::RTCM ? 0 : 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIINPROT_NMEA, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIOUTPROT_UBX, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIOUTPROT_RTCM3X, _output_mode == OutputMode::GPS ? 0 : 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIOUTPROT_NMEA, 0, cfg_valset_msg_size);

		bool cfg_valset_success = false;

		if (sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {

			if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) == 0) {
				cfg_valset_success = true;
			}
		}

		if (cfg_valset_success) {
			_proto_ver_27_or_higher = true;

		} else {
			_proto_ver_27_or_higher = false;
			memset(cfg_prt, 0, sizeof(ubx_payload_tx_cfg_prt_t));
			cfg_prt[0].portID		= UBX_TX_CFG_PRT_PORTID_SPI;
			cfg_prt[0].mode			= UBX_TX_CFG_PRT_MODE_SPI;
			cfg_prt[0].inProtoMask	= in_proto_mask;
			cfg_prt[0].outProtoMask	= out_proto_mask;

			if (!sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt, sizeof(ubx_payload_tx_cfg_prt_t))) {
				return -1;
			}

			waitForAck(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false);
		}

	} else {
		return -1;
	}

	UBX_DEBUG("Protocol version 27+: %i", static_cast<int>(_proto_ver_27_or_higher));

	/* Request module version information by sending an empty MON-VER message */
	if (!sendMessage(UBX_MSG_MON_VER, nullptr, 0)) {
		return -1;
	}

	/* Wait for the reply so that we know to which device we're connected (_board will be set).
	 * Note: we won't actually get an ACK-ACK, but UBX_MSG_MON_VER will also set the ack state.
	 */
	if (waitForAck(UBX_MSG_MON_VER, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}


	/* Now that we know the board, update the baudrate on M8 boards (on F9+ we already used the
	 * higher baudrate with CFG-VALSET) */
	if (_interface == Interface::UART && auto_baudrate && _board == Board::u_blox8) {

		cfg_prt[0].baudRate	= UBX_BAUDRATE_M8_AND_NEWER;
		cfg_prt[1].baudRate	= UBX_BAUDRATE_M8_AND_NEWER;

		if (sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt, 2 * sizeof(ubx_payload_tx_cfg_prt_t))) {
			/* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
			waitForAck(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false);

			setBaudrate(UBX_BAUDRATE_M8_AND_NEWER);
			baudrate = UBX_BAUDRATE_M8_AND_NEWER;
		}
	}


	if (_output_mode == OutputMode::RTCM) {
		// RTCM mode force stationary dynamic model
		_dyn_model = 2;
	}

	int ret;

	if (_proto_ver_27_or_higher) {
		ret = configureDevice(config.gnss_systems);

	} else {
		ret = configureDevicePreV27(config.gnss_systems);
	}

	if (ret != 0) {
		return ret;
	}

	if (_output_mode == OutputMode::RTCM) {
		if (restartSurveyIn() < 0) {
			return -1;
		}

	} else if (_output_mode == OutputMode::GPSAndRTCM) {
		if (activateRTCMOutput(false) < 0) {
			return -1;
		}
	}

	_configured = true;
	return 0;
}


int GPSDriverUBX::configureDevicePreV27(const GNSSSystemsMask &gnssSystems)
{
	/* Send a CFG-RATE message to define update rate */
	memset(&_buf.payload_tx_cfg_rate, 0, sizeof(_buf.payload_tx_cfg_rate));
	_buf.payload_tx_cfg_rate.measRate	= UBX_TX_CFG_RATE_MEASINTERVAL;
	_buf.payload_tx_cfg_rate.navRate	= UBX_TX_CFG_RATE_NAVRATE;
	_buf.payload_tx_cfg_rate.timeRef	= UBX_TX_CFG_RATE_TIMEREF;

	if (!sendMessage(UBX_MSG_CFG_RATE, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_rate))) {
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_RATE, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}

	/* send a NAV5 message to set the options for the internal filter */
	memset(&_buf.payload_tx_cfg_nav5, 0, sizeof(_buf.payload_tx_cfg_nav5));
	_buf.payload_tx_cfg_nav5.mask		= UBX_TX_CFG_NAV5_MASK;
	_buf.payload_tx_cfg_nav5.dynModel	= _dyn_model;
	_buf.payload_tx_cfg_nav5.fixMode	= UBX_TX_CFG_NAV5_FIXMODE;

	if (!sendMessage(UBX_MSG_CFG_NAV5, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_nav5))) {
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_NAV5, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}

	/* configure active GNSS systems (number of channels and used signals taken from U-Center default) */
	if (static_cast<int32_t>(gnssSystems) != 0) {
		memset(&_buf.payload_tx_cfg_gnss, 0, sizeof(_buf.payload_tx_cfg_gnss));
		_buf.payload_tx_cfg_gnss.msgVer = 0x00;
		_buf.payload_tx_cfg_gnss.numTrkChHw = 0x00;  // read only
		_buf.payload_tx_cfg_gnss.numTrkChUse = 0xFF;  // use max number of HW channels
		_buf.payload_tx_cfg_gnss.numConfigBlocks = 7;  // always configure all systems

		// GPS and QZSS should always be enabled and disabled together, according to uBlox
		_buf.payload_tx_cfg_gnss.block[0].gnssId = UBX_TX_CFG_GNSS_GNSSID_GPS;
		_buf.payload_tx_cfg_gnss.block[1].gnssId = UBX_TX_CFG_GNSS_GNSSID_QZSS;

		if (gnssSystems & GNSSSystemsMask::ENABLE_GPS) {
			UBX_DEBUG("GNSS Systems: Use GPS + QZSS");
			_buf.payload_tx_cfg_gnss.block[0].resTrkCh = 8;
			_buf.payload_tx_cfg_gnss.block[0].maxTrkCh = 16;
			_buf.payload_tx_cfg_gnss.block[0].flags = UBX_TX_CFG_GNSS_FLAGS_GPS_L1CA | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
			_buf.payload_tx_cfg_gnss.block[1].resTrkCh = 0;
			_buf.payload_tx_cfg_gnss.block[1].maxTrkCh = 3;
			_buf.payload_tx_cfg_gnss.block[1].flags = UBX_TX_CFG_GNSS_FLAGS_QZSS_L1CA | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
		}

		_buf.payload_tx_cfg_gnss.block[2].gnssId = UBX_TX_CFG_GNSS_GNSSID_SBAS;

		if (gnssSystems & GNSSSystemsMask::ENABLE_SBAS) {
			UBX_DEBUG("GNSS Systems: Use SBAS");
			_buf.payload_tx_cfg_gnss.block[2].resTrkCh = 1;
			_buf.payload_tx_cfg_gnss.block[2].maxTrkCh = 3;
			_buf.payload_tx_cfg_gnss.block[2].flags = UBX_TX_CFG_GNSS_FLAGS_SBAS_L1CA | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
		}

		_buf.payload_tx_cfg_gnss.block[3].gnssId = UBX_TX_CFG_GNSS_GNSSID_GALILEO;

		if (gnssSystems & GNSSSystemsMask::ENABLE_GALILEO) {
			UBX_DEBUG("GNSS Systems: Use Galileo");
			_buf.payload_tx_cfg_gnss.block[3].resTrkCh = 4;
			_buf.payload_tx_cfg_gnss.block[3].maxTrkCh = 8;
			_buf.payload_tx_cfg_gnss.block[3].flags = UBX_TX_CFG_GNSS_FLAGS_GALILEO_E1 | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
		}

		_buf.payload_tx_cfg_gnss.block[4].gnssId = UBX_TX_CFG_GNSS_GNSSID_BEIDOU;

		if (gnssSystems & GNSSSystemsMask::ENABLE_BEIDOU) {
			UBX_DEBUG("GNSS Systems: Use BeiDou");
			_buf.payload_tx_cfg_gnss.block[4].resTrkCh = 8;
			_buf.payload_tx_cfg_gnss.block[4].maxTrkCh = 16;
			_buf.payload_tx_cfg_gnss.block[4].flags = UBX_TX_CFG_GNSS_FLAGS_BEIDOU_B1I | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
		}

		_buf.payload_tx_cfg_gnss.block[5].gnssId = UBX_TX_CFG_GNSS_GNSSID_GLONASS;

		if (gnssSystems & GNSSSystemsMask::ENABLE_GLONASS) {
			UBX_DEBUG("GNSS Systems: Use GLONASS");
			_buf.payload_tx_cfg_gnss.block[5].resTrkCh = 8;
			_buf.payload_tx_cfg_gnss.block[5].maxTrkCh = 14;
			_buf.payload_tx_cfg_gnss.block[5].flags = UBX_TX_CFG_GNSS_FLAGS_GLONASS_L1 | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
		}

		// IMES always disabled
		_buf.payload_tx_cfg_gnss.block[6].gnssId = UBX_TX_CFG_GNSS_GNSSID_IMES;
		_buf.payload_tx_cfg_gnss.block[6].flags = 0;

		// send message
		if (!sendMessage(UBX_MSG_CFG_GNSS, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_gnss))) {
			GPS_ERR("UBX CFG-GNSS message send failed");
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_GNSS, UBX_CONFIG_TIMEOUT, true) < 0) {
			GPS_ERR("UBX CFG-GNSS message ACK failed");
			return -1;
		}
	}

	/* configure message rates */
	/* the last argument is divisor for measurement rate (set by CFG RATE), i.e. 1 means 5Hz */

	/* try to set rate for NAV-PVT */
	/* (implemented for ubx7+ modules only, use NAV-SOL, NAV-POSLLH, NAV-VELNED and NAV-TIMEUTC for ubx6) */
	if (!configureMessageRate(UBX_MSG_NAV_PVT, 1)) {
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
		_use_nav_pvt = false;

	} else {
		_use_nav_pvt = true;
	}

	UBX_DEBUG("%susing NAV-PVT", _use_nav_pvt ? "" : "not ");

	if (!_use_nav_pvt) {
		if (!configureMessageRateAndAck(UBX_MSG_NAV_TIMEUTC, 5, true)) {
			return -1;
		}

		if (!configureMessageRateAndAck(UBX_MSG_NAV_POSLLH, 1, true)) {
			return -1;
		}

		if (!configureMessageRateAndAck(UBX_MSG_NAV_SOL, 1, true)) {
			return -1;
		}

		if (!configureMessageRateAndAck(UBX_MSG_NAV_VELNED, 1, true)) {
			return -1;
		}
	}

	if (!configureMessageRateAndAck(UBX_MSG_NAV_DOP, 1, true)) {
		return -1;
	}

	if (!configureMessageRateAndAck(UBX_MSG_NAV_SVINFO, (_satellite_info != nullptr) ? 5 : 0, true)) {
		return -1;
	}

	if (!configureMessageRateAndAck(UBX_MSG_MON_HW, 1, true)) {
		return -1;
	}

	return 0;
}

int GPSDriverUBX::configureDevice(const GNSSSystemsMask &gnssSystems)
{
	/* set configuration parameters */
	int cfg_valset_msg_size = initCfgValset();
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVSPG_FIXMODE, 3 /* Auto 2d/3d */, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVSPG_UTCSTANDARD, 3 /* USNO (U.S. Naval Observatory derived from GPS) */,
			   cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVSPG_DYNMODEL, _dyn_model, cfg_valset_msg_size);

	// disable odometer & filtering
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_USE_ODO, 0, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_USE_COG, 0, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_OUTLPVEL, 0, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_OUTLPCOG, 0, cfg_valset_msg_size);

	// enable jamming monitor
	cfgValset<uint8_t>(UBX_CFG_KEY_ITFM_ENABLE, 1, cfg_valset_msg_size);

	// measurement rate
	// In case of F9P we use 10Hz, otherwise 8Hz (receivers such as M9N can go higher as well, but
	// the number of used satellites will be restricted to 16. Not mentioned in datasheet)
	const int rate_meas = (_board == Board::u_blox9_F9P) ? 100 : 125;
	cfgValset<uint16_t>(UBX_CFG_KEY_RATE_MEAS, rate_meas, cfg_valset_msg_size);
	cfgValset<uint16_t>(UBX_CFG_KEY_RATE_NAV, 1, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_RATE_TIMEREF, 0, cfg_valset_msg_size);

	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}

	// RTK (optional, as only RTK devices like F9P support it)
	cfg_valset_msg_size = initCfgValset();
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVHPG_DGNSSMODE, 3 /* RTK Fixed */, cfg_valset_msg_size);

	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
		return -1;
	}

	waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false);

	// configure active GNSS systems (leave signal bands as is)
	if (static_cast<int32_t>(gnssSystems) != 0) {
		cfg_valset_msg_size = initCfgValset();

		// GPS and QZSS should always be enabled and disabled together, according to uBlox
		if (gnssSystems & GNSSSystemsMask::ENABLE_GPS) {
			UBX_DEBUG("GNSS Systems: Use GPS + QZSS");
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GPS_ENA, 1, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_QZSS_ENA, 1, cfg_valset_msg_size);

		} else {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GPS_ENA, 0, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_QZSS_ENA, 0, cfg_valset_msg_size);
		}

		if (gnssSystems & GNSSSystemsMask::ENABLE_GALILEO) {
			UBX_DEBUG("GNSS Systems: Use Galileo");
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GAL_ENA, 1, cfg_valset_msg_size);

		} else {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GAL_ENA, 0, cfg_valset_msg_size);
		}


		if (gnssSystems & GNSSSystemsMask::ENABLE_BEIDOU) {
			UBX_DEBUG("GNSS Systems: Use BeiDou");
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_BDS_ENA, 1, cfg_valset_msg_size);

		} else {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_BDS_ENA, 0, cfg_valset_msg_size);
		}

		if (gnssSystems & GNSSSystemsMask::ENABLE_GLONASS) {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GLO_ENA, 1, cfg_valset_msg_size);

		} else {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GLO_ENA, 0, cfg_valset_msg_size);
		}

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			GPS_ERR("UBX GNSS config send failed");
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

		// send SBAS config separately, because it seems to be buggy (with u-center, too)
		cfg_valset_msg_size = initCfgValset();

		if (gnssSystems & GNSSSystemsMask::ENABLE_SBAS) {
			UBX_DEBUG("GNSS Systems: Use SBAS");
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_SBAS_ENA, 1, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_SBAS_L1CA_ENA, 1, cfg_valset_msg_size);

		} else {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_SBAS_ENA, 0, cfg_valset_msg_size);
		}

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true);
	}

	// Configure message rates
	// Send a new CFG-VALSET message to make sure it does not get too large
	cfg_valset_msg_size = initCfgValset();
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_PVT_I2C, 1, cfg_valset_msg_size);
	_use_nav_pvt = true;
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_DOP_I2C, 1, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_SAT_I2C, (_satellite_info != nullptr) ? 10 : 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_MON_RF_I2C, 1, cfg_valset_msg_size);

	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}

	if (_interface == Interface::UART) {
		// Disable GPS protocols at I2C
		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2CINPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2CINPROT_NMEA, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2CINPROT_RTCM3X, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2COUTPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2COUTPROT_NMEA, 0, cfg_valset_msg_size);

		if (_board == Board::u_blox9_F9P) {
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2COUTPROT_RTCM3X, 0, cfg_valset_msg_size);
		}

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}
	}

	int uart2_baudrate = 230400;

	if (_mode == UBXMode::RoverWithMovingBase) {
		UBX_DEBUG("Configuring UART2 for rover");
		cfg_valset_msg_size = initCfgValset();
		// heading output @3Hz
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_RELPOSNED_I2C, 3, cfg_valset_msg_size);
		// enable RTCM input on uart2 + set baudrate
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_STOPBITS, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_DATABITS, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_PARITY, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_RTCM3X, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_NMEA, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_RTCM3X, 0, cfg_valset_msg_size);
		cfgValset<uint32_t>(UBX_CFG_KEY_CFG_UART2_BAUDRATE, uart2_baudrate, cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

	} else if (_mode == UBXMode::MovingBase) {
		UBX_DEBUG("Configuring UART2 for moving base");
		// enable RTCM output on uart2 + set baudrate
		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_STOPBITS, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_DATABITS, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_PARITY, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_RTCM3X, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_NMEA, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_RTCM3X, 1, cfg_valset_msg_size);
		cfgValset<uint32_t>(UBX_CFG_KEY_CFG_UART2_BAUDRATE, uart2_baudrate, cfg_valset_msg_size);

		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE4072_0_UART2, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE4072_1_UART2, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1077_UART2, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1087_UART2, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1097_UART2, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1127_UART2, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_UART2, 1, cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}
	}

	return 0;
}

int GPSDriverUBX::initCfgValset()
{
	memset(&_buf.payload_tx_cfg_valset, 0, sizeof(_buf.payload_tx_cfg_valset));
	_buf.payload_tx_cfg_valset.layers = UBX_CFG_LAYER_RAM;
	return sizeof(_buf.payload_tx_cfg_valset) - sizeof(_buf.payload_tx_cfg_valset.cfgData);
}

template<typename T>
bool GPSDriverUBX::cfgValset(uint32_t key_id, T value, int &msg_size)
{
	if (msg_size + sizeof(key_id) + sizeof(value) > sizeof(_buf)) {
		// If this happens use several CFG-VALSET messages instead of one
		UBX_WARN("buf for CFG_VALSET too small");
		return false;
	}

	uint8_t *buffer = (uint8_t *)&_buf.payload_tx_cfg_valset;
	memcpy(buffer + msg_size, &key_id, sizeof(key_id));
	msg_size += sizeof(key_id);
	memcpy(buffer + msg_size, &value, sizeof(value));
	msg_size += sizeof(value);
	return true;
}

bool GPSDriverUBX::cfgValsetPort(uint32_t key_id, uint8_t value, int &msg_size)
{
	if (_interface == Interface::SPI) {
		if (!cfgValset<uint8_t>(key_id + 4, value, msg_size)) {
			return false;
		}

	} else {
		// enable on UART1 & USB (TODO: should we enable UART2 too? -> better would be to detect the port)
		if (!cfgValset<uint8_t>(key_id + 1, value, msg_size)) {
			return false;
		}

		if (!cfgValset<uint8_t>(key_id + 3, value, msg_size)) {
			return false;
		}
	}

	return true;
}

int GPSDriverUBX::restartSurveyInPreV27()
{
	//disable RTCM output
	configureMessageRate(UBX_MSG_RTCM3_1005, 0);
	configureMessageRate(UBX_MSG_RTCM3_1077, 0);
	configureMessageRate(UBX_MSG_RTCM3_1087, 0);
	configureMessageRate(UBX_MSG_RTCM3_1230, 0);
	configureMessageRate(UBX_MSG_RTCM3_1097, 0);
	configureMessageRate(UBX_MSG_RTCM3_1127, 0);

	//stop it first
	//FIXME: stopping the survey-in process does not seem to work
	memset(&_buf.payload_tx_cfg_tmode3, 0, sizeof(_buf.payload_tx_cfg_tmode3));
	_buf.payload_tx_cfg_tmode3.flags        = 0; /* disable time mode */

	if (!sendMessage(UBX_MSG_CFG_TMODE3, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_tmode3))) {
		UBX_WARN("TMODE3 failed. Device w/o base station support?");
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_TMODE3, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}

	if (_base_settings.type == BaseSettingsType::survey_in) {
		UBX_DEBUG("Starting Survey-in");

		memset(&_buf.payload_tx_cfg_tmode3, 0, sizeof(_buf.payload_tx_cfg_tmode3));
		_buf.payload_tx_cfg_tmode3.flags        = 1; /* start survey-in */
		_buf.payload_tx_cfg_tmode3.svinMinDur   = _base_settings.settings.survey_in.min_dur;
		_buf.payload_tx_cfg_tmode3.svinAccLimit = _base_settings.settings.survey_in.acc_limit;

		if (!sendMessage(UBX_MSG_CFG_TMODE3, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_tmode3))) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_TMODE3, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

		/* enable status output of survey-in */
		if (!configureMessageRateAndAck(UBX_MSG_NAV_SVIN, 5, true)) {
			return -1;
		}

	} else {
		UBX_DEBUG("Setting fixed base position");

		const FixedPositionSettings &settings = _base_settings.settings.fixed_position;

		memset(&_buf.payload_tx_cfg_tmode3, 0, sizeof(_buf.payload_tx_cfg_tmode3));
		_buf.payload_tx_cfg_tmode3.flags = 2 /* fixed mode */ | (1 << 8) /* lat/lon mode */;
		int64_t lat64 = (int64_t)(settings.latitude * 1e9);
		_buf.payload_tx_cfg_tmode3.ecefXOrLat = (int32_t)(lat64 / 100);
		_buf.payload_tx_cfg_tmode3.ecefXOrLatHP = lat64 % 100; // range [-99, 99]
		int64_t lon64 = (int64_t)(settings.longitude * 1e9);
		_buf.payload_tx_cfg_tmode3.ecefYOrLon = (int32_t)(lon64 / 100);
		_buf.payload_tx_cfg_tmode3.ecefYOrLonHP = lon64 % 100;
		int64_t alt64 = (int64_t)((double)settings.altitude * 1e4);
		_buf.payload_tx_cfg_tmode3.ecefZOrAlt = (int32_t)(alt64 / 100); // cm
		_buf.payload_tx_cfg_tmode3.ecefZOrAltHP = alt64 % 100; // 0.1mm

		_buf.payload_tx_cfg_tmode3.fixedPosAcc = (uint32_t)(settings.position_accuracy * 10.f);

		if (!sendMessage(UBX_MSG_CFG_TMODE3, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_tmode3))) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_TMODE3, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

		// directly enable RTCM3 output
		return activateRTCMOutput(true);
	}

	return 0;
}

int GPSDriverUBX::restartSurveyIn()
{
	if (_output_mode != OutputMode::RTCM) {
		return -1;
	}

	if (!_proto_ver_27_or_higher) {
		return restartSurveyInPreV27();
	}

	//disable RTCM output
	int cfg_valset_msg_size = initCfgValset();
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1005_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1077_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1087_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1097_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1127_I2C, 0, cfg_valset_msg_size);
	sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size);
	waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false);

	if (_base_settings.type == BaseSettingsType::survey_in) {
		UBX_DEBUG("Starting Survey-in");

		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_TMODE_MODE, 1 /* Survey-in */, cfg_valset_msg_size);
		cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_SVIN_MIN_DUR, _base_settings.settings.survey_in.min_dur, cfg_valset_msg_size);
		cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_SVIN_ACC_LIMIT, _base_settings.settings.survey_in.acc_limit, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_SVIN_I2C, 5, cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

	} else {
		UBX_DEBUG("Setting fixed base position");

		const FixedPositionSettings &settings = _base_settings.settings.fixed_position;
		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_TMODE_MODE, 2 /* Fixed Mode */, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_TMODE_POS_TYPE, 1 /* Lat/Lon/Height */, cfg_valset_msg_size);
		int64_t lat64 = (int64_t)(settings.latitude * 1e9);
		cfgValset<int32_t>(UBX_CFG_KEY_TMODE_LAT, (int32_t)(lat64 / 100), cfg_valset_msg_size);
		cfgValset<int8_t>(UBX_CFG_KEY_TMODE_LAT_HP, lat64 % 100 /* range [-99, 99] */, cfg_valset_msg_size);
		int64_t lon64 = (int64_t)(settings.longitude * 1e9);
		cfgValset<int32_t>(UBX_CFG_KEY_TMODE_LON, (int32_t)(lon64 / 100), cfg_valset_msg_size);
		cfgValset<int8_t>(UBX_CFG_KEY_TMODE_LON_HP, lon64 % 100 /* range [-99, 99] */, cfg_valset_msg_size);
		int64_t alt64 = (int64_t)((double)settings.altitude * 1e4);
		cfgValset<int32_t>(UBX_CFG_KEY_TMODE_HEIGHT, (int32_t)(alt64 / 100) /* cm */, cfg_valset_msg_size);
		cfgValset<int8_t>(UBX_CFG_KEY_TMODE_HEIGHT_HP, alt64 % 100 /* 0.1mm */, cfg_valset_msg_size);
		cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_FIXED_POS_ACC, (uint32_t)(settings.position_accuracy * 10.f),
				    cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

		// directly enable RTCM3 output
		return activateRTCMOutput(true);

	}

	return 0;
}

int	// -1 = NAK, error or timeout, 0 = ACK
GPSDriverUBX::waitForAck(const uint16_t msg, const unsigned timeout, const bool report)
{
	int ret = -1;

	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = msg;	// memorize sent msg class&ID for ACK check

	gps_abstime time_started = gps_absolute_time();

	while ((_ack_state == UBX_ACK_WAITING) && (gps_absolute_time() < time_started + timeout * 1000)) {
		receive(timeout);
	}

	if (_ack_state == UBX_ACK_GOT_ACK) {
		ret = 0;	// ACK received ok

	} else if (report) {
		if (_ack_state == UBX_ACK_GOT_NAK) {
			UBX_DEBUG("ubx msg 0x%04x NAK", SWAP16((unsigned)msg));

		} else {
			UBX_DEBUG("ubx msg 0x%04x ACK timeout", SWAP16((unsigned)msg));
		}
	}

	_ack_state = UBX_ACK_IDLE;
	return ret;
}

int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverUBX::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE];

	/* timeout additional to poll */
	gps_abstime time_started = gps_absolute_time();

	int handled = 0;

	while (true) {
		bool ready_to_return = _configured ? (_got_posllh && _got_velned) : handled;

		/* Wait for only UBX_PACKET_TIMEOUT if something already received. */
		int ret = read(buf, sizeof(buf), ready_to_return ? UBX_PACKET_TIMEOUT : timeout);

		if (ret < 0) {
			/* something went wrong when polling or reading */
			UBX_WARN("ubx poll_or_read err");
			return -1;

		} else if (ret == 0) {
			/* return success if ready */
			if (ready_to_return) {
				_got_posllh = false;
				_got_velned = false;
				return handled;
			}

		} else {
			//UBX_DEBUG("read %d bytes", ret);

			/* pass received bytes to the packet decoder */
			for (int i = 0; i < ret; i++) {
				handled |= parseChar(buf[i]);
				//UBX_DEBUG("parsed %d: 0x%x", i, buf[i]);
			}

			if (_interface == Interface::SPI) {
				if (buf[ret - 1] == 0xff) {
					if (ready_to_return) {
						_got_posllh = false;
						_got_velned = false;
						return handled;
					}
				}
			}
		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			UBX_DEBUG("timed out, returning");
			return -1;
		}
	}
}

int	// 0 = decoding, 1 = message handled, 2 = sat info message handled
GPSDriverUBX::parseChar(const uint8_t b)
{
	int ret = 0;

	switch (_decode_state) {

	/* Expecting Sync1 */
	case UBX_DECODE_SYNC1:
		if (b == UBX_SYNC1) {	// Sync1 found --> expecting Sync2
			UBX_TRACE_PARSER("A");
			_decode_state = UBX_DECODE_SYNC2;

		} else if (b == RTCM3_PREAMBLE && _rtcm_parsing) {
			UBX_TRACE_PARSER("RTCM");
			_decode_state = UBX_DECODE_RTCM3;
			_rtcm_parsing->addByte(b);
		}

		break;

	/* Expecting Sync2 */
	case UBX_DECODE_SYNC2:
		if (b == UBX_SYNC2) {	// Sync2 found --> expecting Class
			UBX_TRACE_PARSER("B");
			_decode_state = UBX_DECODE_CLASS;

		} else {		// Sync1 not followed by Sync2: reset parser
			decodeInit();
		}

		break;

	/* Expecting Class */
	case UBX_DECODE_CLASS:
		UBX_TRACE_PARSER("C");
		addByteToChecksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
		_rx_msg = b;
		_decode_state = UBX_DECODE_ID;
		break;

	/* Expecting ID */
	case UBX_DECODE_ID:
		UBX_TRACE_PARSER("D");
		addByteToChecksum(b);
		_rx_msg |= b << 8;
		_decode_state = UBX_DECODE_LENGTH1;
		break;

	/* Expecting first length byte */
	case UBX_DECODE_LENGTH1:
		UBX_TRACE_PARSER("E");
		addByteToChecksum(b);
		_rx_payload_length = b;
		_decode_state = UBX_DECODE_LENGTH2;
		break;

	/* Expecting second length byte */
	case UBX_DECODE_LENGTH2:
		UBX_TRACE_PARSER("F");
		addByteToChecksum(b);
		_rx_payload_length |= b << 8;	// calculate payload size

		if (payloadRxInit() != 0) {	// start payload reception
			// payload will not be handled, discard message
			decodeInit();

		} else {
			_decode_state = (_rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
		}

		break;

	/* Expecting payload */
	case UBX_DECODE_PAYLOAD:
		UBX_TRACE_PARSER(".");
		addByteToChecksum(b);

		switch (_rx_msg) {
		case UBX_MSG_NAV_SAT:
			ret = payloadRxAddNavSat(b);	// add a NAV-SAT payload byte
			break;

		case UBX_MSG_NAV_SVINFO:
			ret = payloadRxAddNavSvinfo(b);	// add a NAV-SVINFO payload byte
			break;

		case UBX_MSG_MON_VER:
			ret = payloadRxAddMonVer(b);	// add a MON-VER payload byte
			break;

		default:
			ret = payloadRxAdd(b);		// add a payload byte
			break;
		}

		if (ret < 0) {
			// payload not handled, discard message
			decodeInit();

		} else if (ret > 0) {
			// payload complete, expecting checksum
			_decode_state = UBX_DECODE_CHKSUM1;

		} else {
			// expecting more payload, stay in state UBX_DECODE_PAYLOAD
		}

		ret = 0;
		break;

	/* Expecting first checksum byte */
	case UBX_DECODE_CHKSUM1:
		if (_rx_ck_a != b) {
			UBX_DEBUG("ubx checksum err");
			decodeInit();

		} else {
			_decode_state = UBX_DECODE_CHKSUM2;
		}

		break;

	/* Expecting second checksum byte */
	case UBX_DECODE_CHKSUM2:
		if (_rx_ck_b != b) {
			UBX_DEBUG("ubx checksum err");

		} else {
			ret = payloadRxDone();	// finish payload processing
		}

		decodeInit();
		break;

	case UBX_DECODE_RTCM3:
		if (_rtcm_parsing->addByte(b)) {
			UBX_DEBUG("got RTCM message with length %i", static_cast<int>(_rtcm_parsing->messageLength()));
			gotRTCMMessage(_rtcm_parsing->message(), _rtcm_parsing->messageLength());
			decodeInit();
		}

		break;

	default:
		break;
	}

	return ret;
}

/**
 * Start payload rx
 */
int	// -1 = abort, 0 = continue
GPSDriverUBX::payloadRxInit()
{
	int ret = 0;

	_rx_state = UBX_RXMSG_HANDLE;	// handle by default

	switch (_rx_msg) {
	case UBX_MSG_NAV_PVT:
		if ((_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX7)		/* u-blox 7 msg format */
		    && (_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8)) {	/* u-blox 8+ msg format */
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else if (!_use_nav_pvt) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if not using NAV-PVT
		}

		break;

	case UBX_MSG_INF_DEBUG:
	case UBX_MSG_INF_ERROR:
	case UBX_MSG_INF_NOTICE:
	case UBX_MSG_INF_WARNING:
		if (_rx_payload_length >= sizeof(ubx_buf_t)) {
			_rx_payload_length = sizeof(ubx_buf_t) - 1; //avoid buffer overflow
		}

		break;

	case UBX_MSG_NAV_POSLLH:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_posllh_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else if (_use_nav_pvt) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_NAV_SOL:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_sol_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else if (_use_nav_pvt) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_NAV_DOP:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_dop_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}

		break;

	case UBX_MSG_NAV_RELPOSNED:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_relposned_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}

		break;

	case UBX_MSG_NAV_TIMEUTC:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_timeutc_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else if (_use_nav_pvt) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_NAV_SAT:
	case UBX_MSG_NAV_SVINFO:
		if (_satellite_info == nullptr) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if sat info not requested

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else {
			memset(_satellite_info, 0, sizeof(*_satellite_info));        // initialize sat info
		}

		break;

	case UBX_MSG_NAV_SVIN:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_svin_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}

		break;

	case UBX_MSG_NAV_VELNED:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_velned_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else if (_use_nav_pvt) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_MON_VER:
		break;		// unconditionally handle this message

	case UBX_MSG_MON_HW:
		if ((_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx6_t))	/* u-blox 6 msg format */
		    && (_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx7_t))) {	/* u-blox 7+ msg format */
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured
		}

		break;

	case UBX_MSG_MON_RF:
		if (_rx_payload_length < sizeof(ubx_payload_rx_mon_rf_t) ||
		    (_rx_payload_length - 4) % sizeof(ubx_payload_rx_mon_rf_t::ubx_payload_rx_mon_rf_block_t) != 0) {

			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured
		}

		break;

	case UBX_MSG_ACK_ACK:
		if (_rx_payload_length != sizeof(ubx_payload_rx_ack_ack_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if _configured
		}

		break;

	case UBX_MSG_ACK_NAK:
		if (_rx_payload_length != sizeof(ubx_payload_rx_ack_nak_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if _configured
		}

		break;

	default:
		_rx_state = UBX_RXMSG_DISABLE;	// disable all other messages
		break;
	}

	switch (_rx_state) {
	case UBX_RXMSG_HANDLE:	// handle message
	case UBX_RXMSG_IGNORE:	// ignore message but don't report error
		ret = 0;
		break;

	case UBX_RXMSG_DISABLE:	// disable unexpected messages
		UBX_DEBUG("ubx msg 0x%04x len %u unexpected", SWAP16((unsigned)_rx_msg), (unsigned)_rx_payload_length);

		if (_proto_ver_27_or_higher) {
			uint32_t key_id = 0;

			switch (_rx_msg) { // we cannot infer the config Key ID from _rx_msg for protocol version 27+
			case UBX_MSG_RXM_RAWX:
				key_id = UBX_CFG_KEY_MSGOUT_UBX_RXM_RAWX_I2C;
				break;

			case UBX_MSG_RXM_SFRBX:
				key_id = UBX_CFG_KEY_MSGOUT_UBX_RXM_SFRBX_I2C;
				break;
			}

			if (key_id != 0) {
				gps_abstime t = gps_absolute_time();

				if (t > _disable_cmd_last + DISABLE_MSG_INTERVAL && _configured) {
					/* don't attempt for every message to disable, some might not be disabled */
					_disable_cmd_last = t;
					UBX_DEBUG("ubx disabling msg 0x%04x (0x%04x)", SWAP16((unsigned)_rx_msg), key_id);

					// this will overwrite _buf, which is fine, as we'll return -1 and abort further parsing
					int cfg_valset_msg_size = initCfgValset();
					cfgValsetPort(key_id, 0, cfg_valset_msg_size);
					sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size);
				}
			}

		} else {
			gps_abstime t = gps_absolute_time();

			if (t > _disable_cmd_last + DISABLE_MSG_INTERVAL) {
				/* don't attempt for every message to disable, some might not be disabled */
				_disable_cmd_last = t;
				UBX_DEBUG("ubx disabling msg 0x%04x", SWAP16((unsigned)_rx_msg));

				configureMessageRate(_rx_msg, 0);
			}
		}

		ret = -1;	// return error, abort handling this message
		break;

	case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
		UBX_WARN("ubx msg 0x%04x invalid len %u", SWAP16((unsigned)_rx_msg), (unsigned)_rx_payload_length);
		ret = -1;	// return error, abort handling this message
		break;

	default:	// invalid message state
		UBX_WARN("ubx internal err1");
		ret = -1;	// return error, abort handling this message
		break;
	}

	return ret;
}

/**
 * Add payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverUBX::payloadRxAdd(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	p_buf[_rx_payload_index] = b;

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverUBX::payloadRxAddNavSat(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	if (_rx_payload_index < sizeof(ubx_payload_rx_nav_sat_part1_t)) {
		// Fill Part 1 buffer
		p_buf[_rx_payload_index] = b;

	} else {
		if (_rx_payload_index == sizeof(ubx_payload_rx_nav_sat_part1_t)) {
			// Part 1 complete: decode Part 1 buffer
			_satellite_info->count = MIN(_buf.payload_rx_nav_sat_part1.numSvs, satellite_info_s::SAT_INFO_MAX_SATELLITES);
			UBX_TRACE_SVINFO("SAT len %u  numCh %u", (unsigned)_rx_payload_length,
					 (unsigned)_buf.payload_rx_nav_sat_part1.numSvs);
		}

		if (_rx_payload_index < sizeof(ubx_payload_rx_nav_sat_part1_t) + _satellite_info->count * sizeof(
			    ubx_payload_rx_nav_sat_part2_t)) {
			// Still room in _satellite_info: fill Part 2 buffer
			unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_sat_part1_t)) % sizeof(
						     ubx_payload_rx_nav_sat_part2_t);
			p_buf[buf_index] = b;

			if (buf_index == sizeof(ubx_payload_rx_nav_sat_part2_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned sat_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_sat_part1_t)) /
						     sizeof(ubx_payload_rx_nav_sat_part2_t);

				// convert gnssId:svId to a 8 bit number (use svId numbering from NAV-SVINFO)
				uint8_t ubx_sat_gnssId = static_cast<uint8_t>(_buf.payload_rx_nav_sat_part2.gnssId);
				uint8_t ubx_sat_svId = static_cast<uint8_t>(_buf.payload_rx_nav_sat_part2.svId);

				uint8_t svinfo_svid = 255;

				switch (ubx_sat_gnssId) {
				case 0:  // GPS: G1-G23 -> 1-32
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 32) {
						svinfo_svid = ubx_sat_svId;
					}

					break;

				case 1:  // SBAS: S120-S158 -> 120-158
					if (ubx_sat_svId >= 120 && ubx_sat_svId <= 158) {
						svinfo_svid = ubx_sat_svId;
					}

					break;

				case 2:  // Galileo: E1-E36 -> 211-246
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 36) {
						svinfo_svid = ubx_sat_svId + 210;
					}

					break;

				case 3:  // BeiDou: B1-B37 -> 159-163,33-64
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 4) {
						svinfo_svid = ubx_sat_svId + 158;

					} else if (ubx_sat_svId >= 5 && ubx_sat_svId <= 37) {
						svinfo_svid = ubx_sat_svId + 28;
					}

					break;

				case 4:  // IMES: I1-I10 -> 173-182
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 10) {
						svinfo_svid = ubx_sat_svId + 172;
					}

					break;

				case 5:  // QZSS: Q1-A10 -> 193-202
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 10) {
						svinfo_svid = ubx_sat_svId + 192;
					}

					break;

				case 6:  // GLONASS: R1-R32 -> 65-96, R? -> 255
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 32) {
						svinfo_svid = ubx_sat_svId + 64;
					}

					break;
				}

				_satellite_info->svid[sat_index]	  = svinfo_svid;
				_satellite_info->used[sat_index]	  = static_cast<uint8_t>(_buf.payload_rx_nav_sat_part2.flags & 0x01);
				_satellite_info->elevation[sat_index] = static_cast<uint8_t>(_buf.payload_rx_nav_sat_part2.elev);
				_satellite_info->azimuth[sat_index]	  = static_cast<uint8_t>(static_cast<float>(_buf.payload_rx_nav_sat_part2.azim) *
						255.0f / 360.0f);
				_satellite_info->snr[sat_index]		  = static_cast<uint8_t>(_buf.payload_rx_nav_sat_part2.cno);
				_satellite_info->prn[sat_index]		  = svinfo_svid;
				UBX_TRACE_SVINFO("SAT #%02u  svid %3u  used %u  elevation %3u  azimuth %3u  snr %3u  prn %3u",
						 static_cast<unsigned>(sat_index + 1),
						 static_cast<unsigned>(_satellite_info->svid[sat_index]),
						 static_cast<unsigned>(_satellite_info->used[sat_index]),
						 static_cast<unsigned>(_satellite_info->elevation[sat_index]),
						 static_cast<unsigned>(_satellite_info->azimuth[sat_index]),
						 static_cast<unsigned>(_satellite_info->snr[sat_index]),
						 static_cast<unsigned>(_satellite_info->prn[sat_index])
						);
			}
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Add NAV-SVINFO payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverUBX::payloadRxAddNavSvinfo(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	if (_rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
		// Fill Part 1 buffer
		p_buf[_rx_payload_index] = b;

	} else {
		if (_rx_payload_index == sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
			// Part 1 complete: decode Part 1 buffer
			_satellite_info->count = MIN(_buf.payload_rx_nav_svinfo_part1.numCh, satellite_info_s::SAT_INFO_MAX_SATELLITES);
			UBX_TRACE_SVINFO("SVINFO len %u  numCh %u", (unsigned)_rx_payload_length,
					 (unsigned)_buf.payload_rx_nav_svinfo_part1.numCh);
		}

		if (_rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t) + _satellite_info->count * sizeof(
			    ubx_payload_rx_nav_svinfo_part2_t)) {
			// Still room in _satellite_info: fill Part 2 buffer
			unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) % sizeof(
						     ubx_payload_rx_nav_svinfo_part2_t);
			p_buf[buf_index] = b;

			if (buf_index == sizeof(ubx_payload_rx_nav_svinfo_part2_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned sat_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) /
						     sizeof(ubx_payload_rx_nav_svinfo_part2_t);
				_satellite_info->svid[sat_index]      = static_cast<uint8_t>(_buf.payload_rx_nav_svinfo_part2.svid);
				_satellite_info->used[sat_index]      = static_cast<uint8_t>(_buf.payload_rx_nav_svinfo_part2.flags & 0x01);
				_satellite_info->elevation[sat_index] = static_cast<uint8_t>(_buf.payload_rx_nav_svinfo_part2.elev);
				_satellite_info->azimuth[sat_index]   = static_cast<uint8_t>(static_cast<float>(_buf.payload_rx_nav_svinfo_part2.azim) *
									255.0f / 360.0f);
				_satellite_info->snr[sat_index]       = static_cast<uint8_t>(_buf.payload_rx_nav_svinfo_part2.cno);
				_satellite_info->prn[sat_index]       = static_cast<uint8_t>(_buf.payload_rx_nav_svinfo_part2.svid);

				UBX_TRACE_SVINFO("SVINFO #%02u  svid %3u  used %u  elevation %3u  azimuth %3u  snr %3u  prn %3u",
						 static_cast<unsigned>(sat_index + 1),
						 static_cast<unsigned>(_satellite_info->svid[sat_index]),
						 static_cast<unsigned>(_satellite_info->used[sat_index]),
						 static_cast<unsigned>(_satellite_info->elevation[sat_index]),
						 static_cast<unsigned>(_satellite_info->azimuth[sat_index]),
						 static_cast<unsigned>(_satellite_info->snr[sat_index]),
						 static_cast<unsigned>(_satellite_info->prn[sat_index])
						);
			}
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Add MON-VER payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverUBX::payloadRxAddMonVer(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	if (_rx_payload_index < sizeof(ubx_payload_rx_mon_ver_part1_t)) {
		// Fill Part 1 buffer
		p_buf[_rx_payload_index] = b;

	} else {
		if (_rx_payload_index == sizeof(ubx_payload_rx_mon_ver_part1_t)) {
			// Part 1 complete: decode Part 1 buffer and calculate hash for SW&HW version strings
			_ubx_version = fnv1_32_str(_buf.payload_rx_mon_ver_part1.swVersion, FNV1_32_INIT);
			_ubx_version = fnv1_32_str(_buf.payload_rx_mon_ver_part1.hwVersion, _ubx_version);
			UBX_DEBUG("VER hash 0x%08x", _ubx_version);
			UBX_DEBUG("VER hw  \"%10s\"", _buf.payload_rx_mon_ver_part1.hwVersion);
			UBX_DEBUG("VER sw  \"%30s\"", _buf.payload_rx_mon_ver_part1.swVersion);

			// Device detection (See https://forum.u-blox.com/index.php/9432/need-help-decoding-ubx-mon-ver-hardware-string)
			if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "00040005",
				    sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox5;

			} else if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "00040007",
					   sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox6;

			} else if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "00070000",
					   sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox7;

			} else if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "00080000",
					   sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox8;

			} else if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "00190000",
					   sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox9;

			} else {
				UBX_WARN("unknown board hw: %s", _buf.payload_rx_mon_ver_part1.hwVersion);
			}

			UBX_DEBUG("detected board: %i", static_cast<int>(_board));
		}

		// fill Part 2 buffer
		unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_mon_ver_part1_t)) % sizeof(
					     ubx_payload_rx_mon_ver_part2_t);
		p_buf[buf_index] = b;

		if (buf_index == sizeof(ubx_payload_rx_mon_ver_part2_t) - 1) {
			// Part 2 complete: decode Part 2 buffer
			UBX_DEBUG("VER ext \" %30s\"", _buf.payload_rx_mon_ver_part2.extension);

			// in case of u-blox9 family, check if it's an F9P
			if (_board == Board::u_blox9) {
				if (strstr((const char *)_buf.payload_rx_mon_ver_part2.extension, "MOD=") &&
				    strstr((const char *)_buf.payload_rx_mon_ver_part2.extension, "F9P")) {
					_board = Board::u_blox9_F9P;
					UBX_DEBUG("F9P detected");
				}
			}
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Finish payload rx
 */
int	// 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverUBX::payloadRxDone()
{
	int ret = 0;

	// return if no message handled
	if (_rx_state != UBX_RXMSG_HANDLE) {
		return ret;
	}

	// handle message
	switch (_rx_msg) {

	case UBX_MSG_NAV_PVT:
		UBX_TRACE_RXMSG("Rx NAV-PVT");

		//Check if position fix flag is good
		if ((_buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1) {
			_gps_position->fix_type		 = _buf.payload_rx_nav_pvt.fixType;

			if (_buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_DIFFSOLN) {
				_gps_position->fix_type = 4; //DGPS
			}

			uint8_t carr_soln = _buf.payload_rx_nav_pvt.flags >> 6;

			if (carr_soln == 1) {
				_gps_position->fix_type = 5; //Float RTK

			} else if (carr_soln == 2) {
				_gps_position->fix_type = 6; //Fixed RTK
			}

			_gps_position->vel_ned_valid = true;

		} else {
			_gps_position->fix_type		 = 0;
			_gps_position->vel_ned_valid = false;
		}

		_gps_position->satellites_used	= _buf.payload_rx_nav_pvt.numSV;

		_gps_position->lat		= _buf.payload_rx_nav_pvt.lat;
		_gps_position->lon		= _buf.payload_rx_nav_pvt.lon;
		_gps_position->alt		= _buf.payload_rx_nav_pvt.hMSL;
		_gps_position->alt_ellipsoid	= _buf.payload_rx_nav_pvt.height;

		_gps_position->eph		= static_cast<float>(_buf.payload_rx_nav_pvt.hAcc) * 1e-3f;
		_gps_position->epv		= static_cast<float>(_buf.payload_rx_nav_pvt.vAcc) * 1e-3f;
		_gps_position->s_variance_m_s	= static_cast<float>(_buf.payload_rx_nav_pvt.sAcc) * 1e-3f;

		_gps_position->vel_m_s		= static_cast<float>(_buf.payload_rx_nav_pvt.gSpeed) * 1e-3f;

		_gps_position->vel_n_m_s	= static_cast<float>(_buf.payload_rx_nav_pvt.velN) * 1e-3f;
		_gps_position->vel_e_m_s	= static_cast<float>(_buf.payload_rx_nav_pvt.velE) * 1e-3f;
		_gps_position->vel_d_m_s	= static_cast<float>(_buf.payload_rx_nav_pvt.velD) * 1e-3f;

		_gps_position->cog_rad		= static_cast<float>(_buf.payload_rx_nav_pvt.headMot) * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->c_variance_rad	= static_cast<float>(_buf.payload_rx_nav_pvt.headAcc) * M_DEG_TO_RAD_F * 1e-5f;

		//Check if time and date fix flags are good
		if ((_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDDATE)
		    && (_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDTIME)
		    && (_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_FULLYRESOLVED)) {
			/* convert to unix timestamp */
			tm timeinfo{};
			timeinfo.tm_year	= _buf.payload_rx_nav_pvt.year - 1900;
			timeinfo.tm_mon		= _buf.payload_rx_nav_pvt.month - 1;
			timeinfo.tm_mday	= _buf.payload_rx_nav_pvt.day;
			timeinfo.tm_hour	= _buf.payload_rx_nav_pvt.hour;
			timeinfo.tm_min		= _buf.payload_rx_nav_pvt.min;
			timeinfo.tm_sec		= _buf.payload_rx_nav_pvt.sec;

#ifndef NO_MKTIME
			time_t epoch = mktime(&timeinfo);

			if (epoch > GPS_EPOCH_SECS) {
				// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
				// and control its drift. Since we rely on the HRT for our monotonic
				// clock, updating it from time to time is safe.

				timespec ts{};
				ts.tv_sec = epoch;
				ts.tv_nsec = _buf.payload_rx_nav_pvt.nano;

				setClock(ts);

				_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
				_gps_position->time_utc_usec += _buf.payload_rx_nav_pvt.nano / 1000;

			} else {
				_gps_position->time_utc_usec = 0;
			}

#else
			_gps_position->time_utc_usec = 0;
#endif
		}

		_gps_position->timestamp = gps_absolute_time();
		_last_timestamp_time = _gps_position->timestamp;

		_rate_count_vel++;
		_rate_count_lat_lon++;

		_got_posllh = true;
		_got_velned = true;

		ret = 1;
		break;

	case UBX_MSG_INF_DEBUG:
	case UBX_MSG_INF_NOTICE: {
			uint8_t *p_buf = (uint8_t *)&_buf;
			p_buf[_rx_payload_length] = 0;
			UBX_DEBUG("ubx msg: %s", p_buf);
		}
		break;

	case UBX_MSG_INF_ERROR:
	case UBX_MSG_INF_WARNING: {
			uint8_t *p_buf = (uint8_t *)&_buf;
			p_buf[_rx_payload_length] = 0;
			UBX_WARN("ubx msg: %s", p_buf);
		}
		break;

	case UBX_MSG_NAV_POSLLH:
		UBX_TRACE_RXMSG("Rx NAV-POSLLH");

		_gps_position->lat	= _buf.payload_rx_nav_posllh.lat;
		_gps_position->lon	= _buf.payload_rx_nav_posllh.lon;
		_gps_position->alt	= _buf.payload_rx_nav_posllh.hMSL;
		_gps_position->eph	= static_cast<float>(_buf.payload_rx_nav_posllh.hAcc) * 1e-3f; // from mm to m
		_gps_position->epv	= static_cast<float>(_buf.payload_rx_nav_posllh.vAcc) * 1e-3f; // from mm to m
		_gps_position->alt_ellipsoid = _buf.payload_rx_nav_posllh.height;

		_gps_position->timestamp = gps_absolute_time();

		_rate_count_lat_lon++;
		_got_posllh = true;

		ret = 1;
		break;

	case UBX_MSG_NAV_SOL:
		UBX_TRACE_RXMSG("Rx NAV-SOL");

		_gps_position->fix_type		= _buf.payload_rx_nav_sol.gpsFix;
		_gps_position->s_variance_m_s	= static_cast<float>(_buf.payload_rx_nav_sol.sAcc) * 1e-2f;	// from cm to m
		_gps_position->satellites_used	= _buf.payload_rx_nav_sol.numSV;

		ret = 1;
		break;

	case UBX_MSG_NAV_DOP:
		UBX_TRACE_RXMSG("Rx NAV-DOP");

		_gps_position->hdop		= _buf.payload_rx_nav_dop.hDOP * 0.01f;	// from cm to m
		_gps_position->vdop		= _buf.payload_rx_nav_dop.vDOP * 0.01f;	// from cm to m

		ret = 1;
		break;

	case UBX_MSG_NAV_TIMEUTC:
		UBX_TRACE_RXMSG("Rx NAV-TIMEUTC");

		if (_buf.payload_rx_nav_timeutc.valid & UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC) {
			// convert to unix timestamp
			tm timeinfo {};
			timeinfo.tm_year	= _buf.payload_rx_nav_timeutc.year - 1900;
			timeinfo.tm_mon		= _buf.payload_rx_nav_timeutc.month - 1;
			timeinfo.tm_mday	= _buf.payload_rx_nav_timeutc.day;
			timeinfo.tm_hour	= _buf.payload_rx_nav_timeutc.hour;
			timeinfo.tm_min		= _buf.payload_rx_nav_timeutc.min;
			timeinfo.tm_sec		= _buf.payload_rx_nav_timeutc.sec;
			timeinfo.tm_isdst	= 0;
#ifndef NO_MKTIME
			time_t epoch = mktime(&timeinfo);

			// only set the time if it makes sense

			if (epoch > GPS_EPOCH_SECS) {
				// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
				// and control its drift. Since we rely on the HRT for our monotonic
				// clock, updating it from time to time is safe.

				timespec ts{};
				ts.tv_sec = epoch;
				ts.tv_nsec = _buf.payload_rx_nav_timeutc.nano;

				setClock(ts);

				_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
				_gps_position->time_utc_usec += _buf.payload_rx_nav_timeutc.nano / 1000;

			} else {
				_gps_position->time_utc_usec = 0;
			}

#else
			_gps_position->time_utc_usec = 0;
#endif
		}

		_last_timestamp_time = gps_absolute_time();

		ret = 1;
		break;

	case UBX_MSG_NAV_SAT:
	case UBX_MSG_NAV_SVINFO:
		UBX_TRACE_RXMSG("Rx NAV-SVINFO");

		// _satellite_info already populated by payload_rx_add_svinfo(), just add a timestamp
		_satellite_info->timestamp = gps_absolute_time();

		ret = 2;
		break;

	case UBX_MSG_NAV_SVIN:
		UBX_TRACE_RXMSG("Rx NAV-SVIN");
		{
			ubx_payload_rx_nav_svin_t &svin = _buf.payload_rx_nav_svin;

			UBX_DEBUG("Survey-in status: %is cur accuracy: %imm nr obs: %i valid: %i active: %i",
				  svin.dur, svin.meanAcc / 10, svin.obs, static_cast<int>(svin.valid), static_cast<int>(svin.active));

			SurveyInStatus status{};
			double ecef_x = (static_cast<double>(svin.meanX) + static_cast<double>(svin.meanXHP) * 0.01) * 0.01;
			double ecef_y = (static_cast<double>(svin.meanY) + static_cast<double>(svin.meanYHP) * 0.01) * 0.01;
			double ecef_z = (static_cast<double>(svin.meanZ) + static_cast<double>(svin.meanZHP) * 0.01) * 0.01;
			ECEF2lla(ecef_x, ecef_y, ecef_z, status.latitude, status.longitude, status.altitude);
			status.duration = svin.dur;
			status.mean_accuracy = svin.meanAcc / 10;
			status.flags = (svin.valid & 1) | ((svin.active & 1) << 1);
			surveyInStatus(status);

			if (svin.valid == 1 && svin.active == 0) {
				if (activateRTCMOutput(true) != 0) {
					return -1;
				}
			}
		}

		ret = 1;
		break;

	case UBX_MSG_NAV_VELNED:
		UBX_TRACE_RXMSG("Rx NAV-VELNED");

		_gps_position->vel_m_s        = static_cast<float>(_buf.payload_rx_nav_velned.speed) * 1e-2f;
		_gps_position->vel_n_m_s      = static_cast<float>(_buf.payload_rx_nav_velned.velN)  * 1e-2f; // NED NORTH velocity
		_gps_position->vel_e_m_s      = static_cast<float>(_buf.payload_rx_nav_velned.velE)  * 1e-2f; // NED EAST velocity
		_gps_position->vel_d_m_s      = static_cast<float>(_buf.payload_rx_nav_velned.velD)  * 1e-2f; // NED DOWN velocity
		_gps_position->cog_rad        = static_cast<float>(_buf.payload_rx_nav_velned.heading) * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->c_variance_rad = static_cast<float>(_buf.payload_rx_nav_velned.cAcc)    * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->vel_ned_valid  = true;

		_rate_count_vel++;
		_got_velned = true;

		ret = 1;
		break;

	case UBX_MSG_NAV_RELPOSNED:
		UBX_TRACE_RXMSG("Rx NAV-RELPOSNED");

		if (_mode == UBXMode::RoverWithMovingBase) {
			float heading = _buf.payload_rx_nav_relposned.relPosHeading * 1e-5f;
			float heading_acc = _buf.payload_rx_nav_relposned.accHeading * 1e-5f;
			float rel_length = _buf.payload_rx_nav_relposned.relPosLength + _buf.payload_rx_nav_relposned.relPosHPLength * 1e-2f;
			float rel_length_acc = _buf.payload_rx_nav_relposned.accLength * 1e-2f;
			bool heading_valid = _buf.payload_rx_nav_relposned.flags & (1 << 8);
			bool rel_pos_valid = _buf.payload_rx_nav_relposned.flags & (1 << 2);
			(void)heading_acc;
			(void)rel_length_acc;
			UBX_DEBUG("Heading: %.1f deg, acc: %.1f deg, relLen: %.1f cm, relAcc: %.1f cm, valid: %i %i", (double)heading,
				  (double)heading_acc, (double)rel_length, (double)rel_length_acc, heading_valid, rel_pos_valid);

			if (heading_valid && rel_pos_valid && rel_length < 1000.f) { // validity & sanity checks
				heading *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]
				heading -= _heading_offset; // range: [-pi, 3pi]

				if (heading > M_PI_F) {
					heading -= 2.f * M_PI_F; // final range is [-pi, pi]
				}

				_gps_position->heading = heading;
			}

			ret = 1;
		}

		break;

	case UBX_MSG_MON_VER:
		UBX_TRACE_RXMSG("Rx MON-VER");

		// This is polled only on startup, and the startup code waits for an ack
		if (_ack_state == UBX_ACK_WAITING && _ack_waiting_msg == UBX_MSG_MON_VER) {
			_ack_state = UBX_ACK_GOT_ACK;
		}

		ret = 1;
		break;

	case UBX_MSG_MON_HW:
		UBX_TRACE_RXMSG("Rx MON-HW");

		switch (_rx_payload_length) {

		case sizeof(ubx_payload_rx_mon_hw_ubx6_t):	/* u-blox 6 msg format */
			_gps_position->noise_per_ms		= _buf.payload_rx_mon_hw_ubx6.noisePerMS;
			_gps_position->automatic_gain_control   = _buf.payload_rx_mon_hw_ubx6.agcCnt;
			_gps_position->jamming_indicator	= _buf.payload_rx_mon_hw_ubx6.jamInd;

			ret = 1;
			break;

		case sizeof(ubx_payload_rx_mon_hw_ubx7_t):	/* u-blox 7+ msg format */
			_gps_position->noise_per_ms		= _buf.payload_rx_mon_hw_ubx7.noisePerMS;
			_gps_position->automatic_gain_control   = _buf.payload_rx_mon_hw_ubx7.agcCnt;
			_gps_position->jamming_indicator	= _buf.payload_rx_mon_hw_ubx7.jamInd;

			ret = 1;
			break;

		default:		// unexpected payload size:
			ret = 0;	// don't handle message
			break;
		}

		break;

	case UBX_MSG_MON_RF:
		UBX_TRACE_RXMSG("Rx MON-RF");

		_gps_position->noise_per_ms		= _buf.payload_rx_mon_rf.block[0].noisePerMS;
		_gps_position->jamming_indicator	= _buf.payload_rx_mon_rf.block[0].jamInd;
		_gps_position->jamming_state		= _buf.payload_rx_mon_rf.block[0].flags;

		ret = 1;
		break;

	case UBX_MSG_ACK_ACK:
		UBX_TRACE_RXMSG("Rx ACK-ACK");

		if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
			_ack_state = UBX_ACK_GOT_ACK;
		}

		ret = 1;
		break;

	case UBX_MSG_ACK_NAK:
		UBX_TRACE_RXMSG("Rx ACK-NAK");

		if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
			_ack_state = UBX_ACK_GOT_NAK;
		}

		ret = 1;
		break;

	default:
		break;
	}

	if (ret > 0) {
		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
	}

	return ret;
}

int
GPSDriverUBX::activateRTCMOutput(bool reduce_update_rate)
{
	/* For base stations we switch to 1 Hz update rate, which is enough for RTCM output.
	 * For the survey-in, we still want 5/10 Hz, because this speeds up the process */

	if (_proto_ver_27_or_higher) {
		int cfg_valset_msg_size = initCfgValset();

		if (reduce_update_rate) {
			cfgValset<uint16_t>(UBX_CFG_KEY_RATE_MEAS, 1000, cfg_valset_msg_size);
		}

		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1005_I2C, 5, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1077_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1087_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1097_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1127_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_SVIN_I2C, 0, cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false) < 0) {
			return -1;
		}

	} else {

		if (reduce_update_rate) {
			memset(&_buf.payload_tx_cfg_rate, 0, sizeof(_buf.payload_tx_cfg_rate));
			_buf.payload_tx_cfg_rate.measRate	= 1000;
			_buf.payload_tx_cfg_rate.navRate	= UBX_TX_CFG_RATE_NAVRATE;
			_buf.payload_tx_cfg_rate.timeRef	= UBX_TX_CFG_RATE_TIMEREF;

			if (!sendMessage(UBX_MSG_CFG_RATE, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_rate))) { return -1; }

			// according to the spec we should receive an (N)ACK here, but we don't
		}

		configureMessageRate(UBX_MSG_NAV_SVIN, 0);

		// stationary RTK reference station ARP (can be sent at lower rate)
		if (!configureMessageRate(UBX_MSG_RTCM3_1005, 5)) { return -1; }

		// GPS
		if (!configureMessageRate(UBX_MSG_RTCM3_1077, 1)) { return -1; }

		// GLONASS
		if (!configureMessageRate(UBX_MSG_RTCM3_1087, 1)) { return -1; }

		// GLONASS code-phase biases
		if (!configureMessageRate(UBX_MSG_RTCM3_1230, 1)) { return -1; }

		// Galileo
		if (!configureMessageRate(UBX_MSG_RTCM3_1097, 1)) { return -1; }

		// BeiDou
		if (!configureMessageRate(UBX_MSG_RTCM3_1127, 1)) { return -1; }
	}

	return 0;
}

void
GPSDriverUBX::decodeInit()
{
	_decode_state = UBX_DECODE_SYNC1;
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_payload_length = 0;
	_rx_payload_index = 0;

	if (_output_mode == OutputMode::GPSAndRTCM || _output_mode == OutputMode::RTCM) {
		if (!_rtcm_parsing) {
			_rtcm_parsing = new RTCMParsing();
		}

		if (_rtcm_parsing) {
			_rtcm_parsing->reset();
		}
	}
}

void
GPSDriverUBX::addByteToChecksum(const uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

void
GPSDriverUBX::calcChecksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum)
{
	for (uint16_t i = 0; i < length; i++) {
		checksum->ck_a = checksum->ck_a + buffer[i];
		checksum->ck_b = checksum->ck_b + checksum->ck_a;
	}
}

bool
GPSDriverUBX::configureMessageRate(const uint16_t msg, const uint8_t rate)
{
	if (_proto_ver_27_or_higher) {
		// configureMessageRate() should not be called if _proto_ver_27_or_higher is true.
		// If you see this message the calling code needs to be fixed.
		UBX_WARN("FIXME: use of deprecated msg CFG_MSG (%i %i)", msg, rate);
	}

	ubx_payload_tx_cfg_msg_t cfg_msg;	// don't use _buf (allow interleaved operation)
	memset(&cfg_msg, 0, sizeof(cfg_msg));

	cfg_msg.msg	= msg;
	cfg_msg.rate	= rate;

	return sendMessage(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
}

bool
GPSDriverUBX::configureMessageRateAndAck(uint16_t msg, uint8_t rate, bool report_ack_error)
{
	if (!configureMessageRate(msg, rate)) {
		return false;
	}

	return waitForAck(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, report_ack_error) >= 0;
}

bool
GPSDriverUBX::sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
	ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2, 0, 0};
	ubx_checksum_t checksum = {0, 0};

	// Populate header
	header.msg	= msg;
	header.length	= length;

	// Calculate checksum
	calcChecksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

	if (payload != nullptr) {
		calcChecksum(payload, length, &checksum);
	}

	// Send message
	if (write((void *)&header, sizeof(header)) != sizeof(header)) {
		return false;
	}

	if (payload && write((void *)payload, length) != length) {
		return false;
	}

	if (write((void *)&checksum, sizeof(checksum)) != sizeof(checksum)) {
		return false;
	}

	return true;
}

uint32_t
GPSDriverUBX::fnv1_32_str(uint8_t *str, uint32_t hval)
{
	uint8_t *s = str;

	/*
	 * FNV-1 hash each octet in the buffer
	 */
	while (*s) {

		/* multiply by the 32 bit FNV magic prime mod 2^32 */
#if defined(NO_FNV_GCC_OPTIMIZATION)
		hval *= FNV1_32_PRIME;
#else
		hval += (hval << 1) + (hval << 4) + (hval << 7) + (hval << 8) + (hval << 24);
#endif

		/* xor the bottom with the current octet */
		hval ^= (uint32_t) * s++;
	}

	/* return our new hash value */
	return hval;
}

int
GPSDriverUBX::reset(GPSRestartType restart_type)
{
	memset(&_buf.payload_tx_cfg_rst, 0, sizeof(_buf.payload_tx_cfg_rst));
	_buf.payload_tx_cfg_rst.resetMode = UBX_TX_CFG_RST_MODE_SOFTWARE;

	switch (restart_type) {
	case GPSRestartType::Hot:
		_buf.payload_tx_cfg_rst.navBbrMask = UBX_TX_CFG_RST_BBR_MODE_HOT_START;
		break;

	case GPSRestartType::Warm:
		_buf.payload_tx_cfg_rst.navBbrMask = UBX_TX_CFG_RST_BBR_MODE_WARM_START;
		break;

	case GPSRestartType::Cold:
		_buf.payload_tx_cfg_rst.navBbrMask = UBX_TX_CFG_RST_BBR_MODE_COLD_START;
		break;

	default:
		return -2;
	}

	if (sendMessage(UBX_MSG_CFG_RST, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_rst))) {
		return 0;
	}

	return -2;
}
