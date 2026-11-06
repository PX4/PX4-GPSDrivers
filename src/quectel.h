/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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

#pragma once

/**
 * @file quectel.h
 *
 * Quectel GNSS driver
 *
 * @author Hongbin Chu <hongbin.chu@qq.com>
 *
 * version 1.0, 2025-06-01
 *
 * Introduction to the receiver working mode:
 * *** Rover mode ***
 * Configuration:
 *   set_work_mode(QL_WORK_MODE_ROVER); // Configure the receiver working mode
 *   set_save_config_to_nvm();          // Save the configuration to NVM
 *   set_reset_module();                // Restart the module to take effect
 * 
 * *** Base station mode ***
 * Configuration:
 *   set_work_mode(QL_WORK_MODE_BASE_STATION); // Configure the receiver working mode
 *   // Set the base position or survey-in specs
 * #if 0
 *   setBasePosition(latitude, longitude, altitude, position_accuracy); // Set the base position
 * #else
 *   setSurveyInSpecs(survey_in_acc_limit, survey_in_min_dur);          // Set the survey-in specs
 * #endif
 *   setOutputSurveyInStatus(true);         // Enable output survey-in status
 *   set_save_config_to_nvm();              // Save the configuration to NVM
 *   set_reset_module();                    // Restart the module to take effect
 */

#include "../../definitions.h"
#include "base_station.h"
#include "rtcm.h"
#include "satellite_info.h"

#define UNUSED(x) (void)(x)

#define QL_LOG_LEVEL_DEBUG 4
#define QL_LOG_LEVEL_INFO  3
#define QL_LOG_LEVEL_WARN  2
#define QL_LOG_LEVEL_ERR   1
#define QL_LOG_LEVEL_NONE  0

#define QL_LOG_LEVEL QL_LOG_LEVEL_WARN  // default log level

#if QL_LOG_LEVEL >= QL_LOG_LEVEL_DEBUG
#define QL_DEBUG(fmt, ...) GPS_INFO("[D] " fmt "\n", ##__VA_ARGS__)
#else
#define QL_DEBUG(fmt, ...) do {} while (0)
#endif

#if QL_LOG_LEVEL >= QL_LOG_LEVEL_INFO
#define QL_INFO(fmt, ...) GPS_INFO("[I] " fmt "\n", ##__VA_ARGS__)
#else
#define QL_INFO(fmt, ...) do {} while (0)
#endif

#if QL_LOG_LEVEL >= QL_LOG_LEVEL_WARN
#define QL_WARN(fmt, ...) GPS_WARN("[W] " fmt "\n", ##__VA_ARGS__)
#else
#define QL_WARN(fmt, ...) do {} while (0)
#endif

#if QL_LOG_LEVEL >= QL_LOG_LEVEL_ERR
#define QL_ERR(fmt, ...)  GPS_ERR("[E] " fmt "\n", ##__VA_ARGS__)
#else
#define QL_ERR(fmt, ...)  do {} while (0)
#endif

typedef enum  {
	QL_DECODE_NONE = 0,        ///< No decode state
	QL_DECODE_HEADER,          ///< Decode header
	QL_DECODE_CHECKSUM1,       ///< Decode checksum first byte
	QL_DECODE_CHECKSUM2        ///< Decode checksum second byte
} ql_decode_state_t;

typedef struct
{
    uint64_t timestamp;  ///< Timestamp
    double epe_north;    ///< Estimated north error (m)
    double epe_east;     ///< Estimated east error (m)
    double epe_down;     ///< Estimated down error (m)
    double epe_2d;       ///< Estimated 2D position error (m)
    double epe_3d;       ///< Estimated 3D position error (m)
} ql_msg_pqtmepe_t;

typedef struct
{
	uint64_t timestamp;  ///< Timestamp
	double time_format;  ///< hhmmss.sss
	double vel_n;        ///< Velocity north (m/s)
	double vel_e;        ///< Velocity east (m/s)
	double vel_d;        ///< Velocity down (m/s)
	double grd_spd;      ///< 2D speed (m/s)
	double spd;          ///< 3D Speed (m/s)
	double heading;      ///< Heading (degrees)
	double grd_spd_acc;  ///< Estimated 2D speed accuracy (m/s)
	double spd_acc;      ///< Estimated 3D Speed accuracy (m/s)
	double heading_acc;  ///< Heading accuracy (degrees)
} ql_msg_pqtmvel_t;

typedef enum
{
	QL_WORK_MODE_NONE = 0,          ///< No work mode
	QL_WORK_MODE_ROVER = 1,         ///< Rover work mode
	QL_WORK_MODE_BASE_STATION = 2   ///< Base station work mode
} ql_work_mode_t;

class GPSDriverQuectel : public GPSBaseStationSupport
{
public:

    GPSDriverQuectel(GPSCallbackPtr callback, void* callback_user, struct sensor_gps_s* gps_position);
	
	virtual~GPSDriverQuectel();

	int receive(unsigned timeout) override;

	int configure(unsigned& baudrate, const GPSConfig& config) override;

	/**
	 * Get the estimated position error
	 */
	ql_msg_pqtmepe_t get_msg_epe() const
	{
		return _msg_pqtmepe;
	}
	/**
	 * Get the velocity information
	 */
	ql_msg_pqtmvel_t get_msg_pqtmvel() const
	{
		return _msg_pqtmvel;
	}
	/**
	 * Get the work mode flag
	 */
	ql_work_mode_t get_work_mode() const
	{
		return _work_mode;
	}
	/**
	 * Get the module version
	 * @param ver Pointer to a buffer to store the version string
	 * @return true if the version was successfully retrieved, false otherwise
	 */
	bool query_module_verion(uint8_t* ver);
	/**
	 *  Set the receiver working mode
	 * @param work_mode The desired working mode (QL_WORK_MODE_ROVER or QL_WORK_MODE_BASE_STATION)
	 * @return true if the working mode was successfully set, false otherwise
	 * @note save and reset take effect.
	 */
	bool set_work_mode(ql_work_mode_t work_mode);
	/**
	 *  Get the current working mode
	 * @param work_mode Pointer to store the current working mode
	 * @return true if the working mode was successfully retrieved, false otherwise
	 */
	bool get_work_mode(ql_work_mode_t* work_mode);
	/**
	 *  Set the survey-in specs
	 * @param survey_in_acc_limit Minimum accuracy in 0.1mm
	 * @param survey_in_min_dur Minimum duration in seconds
	 * @return true if the specs were successfully set, false otherwise
	 * @note save and reset take effect.
	 */
	bool setSurveyInSpecs(uint32_t survey_in_acc_limit, uint32_t survey_in_min_dur);
	/**
	 *  Set the base position
	 * @param latitude Latitude in degrees
	 * @param longitude Longitude in degrees
	 * @param altitude Altitude in meters
	 * @param position_accuracy Position accuracy in meters
	 * @return true if the base position was successfully set, false otherwise
	 * @note save and reset take effect.
	 */
	bool setBasePosition(double latitude, double longitude, float altitude, float position_accuracy);
	/**
	 * Save the current configuration to non-volatile memory
	 * @return true if the configuration was successfully saved, false otherwise
	 */
	bool setOutputSurveyInStatus(bool enable);
	/**
	 * Save the current configuration to non-volatile memory
	 * @return true if the configuration was successfully saved, false otherwise
	 */
	bool set_save_config_to_nvm();
	/**
	 * Restore the configuration to non-volatile memory
	 * @return true if the configuration was successfully restored, false otherwise
	 */
	bool set_restore_config_to_nvm();
	 /**
	 * Reset the module
	 * @return true if the reset command was successfully sent, false otherwise
	 */
	bool set_reset_module();
		/**
	 * Set the message rate for a specific message
	 * @param msg_name Name of the message to set the rate
	 * @param msg_rate Rate in Hz (0 to disable)
	 * @param msg_ver Version of the message
	 * @param timeout_ms Timeout in milliseconds
	 * @return true if the command was successfully sent, false otherwise
	 */
	bool set_cmd_msgrate(uint8_t* msg_name, uint8_t msg_rate, uint8_t msg_ver, uint16_t timeout_ms);

private:

#define QL_RX_BUFF_LENGTH  (1024)
#define QL_NMEA_MSG_LENGTH  (512)

	/**
	 * end of frame, used to check if we have a complete frame
	 */
	bool _is_frame_end{ false };

	/**
	 * Check the checksum of the data
	 * @param pData Pointer to the data buffer
	 * @param lentgh Length of the data buffer
	 * @return The checksum value
	 */
	unsigned char checkXOR(const unsigned char *pData, const unsigned int lentgh);
	/**
	 * Parse the QL packet
	 */
	int parseChar(uint8_t b);

	/**
	 * Handle the package once it has arrived
	 */
	int handleMessage(int packet_len);

	/**
	 * Initialize the message flags
	 */
	void msgFlagInit();
	/**
	 * Check if a specific SVID is used in positioning
	 * @param svid Satellite ID to check
	 * @param used_svid Pointer to an array to store the used SVIDs
	 * @return true if the SVID is used, false otherwise
	 */
	bool is_used_svid(uint8_t svid, uint8_t* used_svid);

	// Decode the NMEA0183 messages
	int decode_msg_gga(char* bufptr);
	int decode_msg_rmc(char* bufptr);
	int decode_msg_gsa(char* bufptr);
	int decode_msg_gsv(char* bufptr);
	int decode_msg_vtg(char* bufptr);

	// Decode the Quectel specific messages
	int decode_msg_pqtmvel(char* bufptr);
	int decode_msg_pqtmepe(char* bufptr);
	int decode_msg_pqtmsvinstatus(char* bufptr);

	/**
	 * Encode a command into a buffer
	 * @param buf Pointer to the buffer to store the encoded command
	 * @param maxBufLength Maximum length of the buffer
	 * @param fmt Format string for the command
	 * @return The length of the encoded command, or -1 on error
	 */
	int encode_cmd(uint8_t* buf, uint16_t maxBufLength, const char*fmt, ...);
	/**
	 * Wait for a specific message to arrive
	 * @param msg_name Name of the message to wait for
	 * @param timeout Timeout in seconds
	 * @return The length of the received message, or 0 if the timeout was reached
	 */
	int wait_message(const uint8_t *msg_name, unsigned timeout);
	/**
	 * Convert WGS84 to ECEF
	 * @param lat Latitude in degrees
	 * @param lon Longitude in degrees
	 * @param alt Altitude in meters
	 * @param x ECEF X-coordinate [m] (output)
	 * @param y ECEF Y-coordinate [m] (output)
	 * @param z ECEF Z-coordinate [m] (output)
	 */
	void wgs84_to_ecef(double lat, double lon, double alt, double& x, double& y, double& z);
	/**
	 * Convert ECEF to WGS84
	 * @param x ECEF X-coordinate [m]
	 * @param y ECEF Y-coordinate [m]
	 * @param z ECEF Z-coordinate [m]
	 * @param lat Latitude in degrees
	 * @param lon Longitude in degrees
	 * @param alt Altitude in meters
	 */
	void ecef_to_wgs84(double x, double y, double z, double& lat, double& lon, double& alt);

	ql_msg_pqtmepe_t _msg_pqtmepe{ 0, 0, 0, 0, 0, 0 };  // PQTMEPE message
	ql_msg_pqtmvel_t _msg_pqtmvel{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // PQTMVEL message

	// 
	sensor_gps_s* _gps_position{ nullptr };
	satellite_info_s _gps_satellite_info;  // GPS
	satellite_info_s _gln_satellite_info;	// GLONASS
	satellite_info_s _bds_satellite_info;  // BDS
	satellite_info_s _gal_satellite_info;  // Galileo
	satellite_info_s _qzss_satellite_info;  // QZSS

	uint8_t _gps_used_svid[SAT_INFO_SATELLITES_SIZE];  // GPS
	uint8_t _gln_used_svid[SAT_INFO_SATELLITES_SIZE];  // GLONASS
	uint8_t _bds_used_svid[SAT_INFO_SATELLITES_SIZE];  // BDS
	uint8_t _gal_used_svid[SAT_INFO_SATELLITES_SIZE];  // Galileo
	uint8_t _qzss_used_svid[SAT_INFO_SATELLITES_SIZE];  // QZSS

	double _last_POS_timeUTC{ 0 };
	double _last_VEL_timeUTC{ 0 };
	double _last_FIX_timeUTC{ 0 };
	uint64_t _last_timestamp_time{ 0 };

	uint8_t _sat_num_gga{ 0 };
	uint8_t _sat_num_gns{ 0 };
	uint8_t _sat_num_gsv{ 0 };
	uint8_t _sat_num_gpgsv{ 0 };
	uint8_t _sat_num_glgsv{ 0 };
	uint8_t _sat_num_gagsv{ 0 };
	uint8_t _sat_num_gbgsv{ 0 };
	uint8_t _sat_num_gqgsv{ 0 };

	// Check if we got all basic essential packages we need
	bool _TIME_received{ false };
	bool _POS_received{ false };
	bool _ALT_received{ false };
	bool _SVNUM_received{ false };
	bool _SVINFO_received{ false };
	bool _FIX_received{ false };
	bool _DOP_received{ false };
	bool _VEL_received{ false };
	bool _EPH_received{ false };
	bool _HEAD_received{ false };

	ql_work_mode_t _work_mode{ QL_WORK_MODE_ROVER };  // Work mode: rover or base station
	RTCMParsing* _rtcm_parsing{ nullptr };
	ql_decode_state_t _decode_state{ QL_DECODE_NONE };
	unsigned _rx_count{};
	uint8_t _rx_buffer[QL_RX_BUFF_LENGTH];
	uint8_t _rx_ck_a{};
	uint8_t _rx_ck_b{};
};