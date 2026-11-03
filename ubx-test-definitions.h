#pragma once

// Minimal stand-ins for the PX4 platform types that the driver code depends
// on, so the parsers can be unit-tested outside of PX4. Selected through the
// GPS_DEFINITIONS_HEADER macro (see CMakeLists.txt).

#include <cstdint>
#include <ctime>

#define M_PI_F          3.14159265358979323846f
#define M_DEG_TO_RAD_F  0.017453292519943295f

void test_gps_log(const char *fmt, ...);
#define GPS_INFO(...)   test_gps_log(__VA_ARGS__)
#define GPS_WARN(...)   test_gps_log(__VA_ARGS__)
#define GPS_ERR(...)    test_gps_log(__VA_ARGS__)

extern "C" uint64_t hrt_absolute_time();
typedef uint64_t gps_abstime;
#define gps_absolute_time hrt_absolute_time

struct sensor_gps_s
{
	uint64_t timestamp;
	double latitude_deg;
	double longitude_deg;
	double altitude_msl_m;
	double altitude_ellipsoid_m;
	uint64_t time_utc_usec;
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
	float heading_accuracy;
	uint16_t automatic_gain_control;
	uint8_t fix_type;
	uint8_t jamming_state;
	uint8_t spoofing_state;
	bool vel_ned_valid;
	uint8_t satellites_used;
	bool rtcm_crc_failed;
	uint8_t rtcm_msg_used;
};

struct satellite_info_s
{
	static constexpr uint8_t SAT_INFO_MAX_SATELLITES = 40;

	uint64_t timestamp;
	uint8_t count;
	uint8_t svid[SAT_INFO_MAX_SATELLITES];
	uint8_t used[SAT_INFO_MAX_SATELLITES];
	uint8_t elevation[SAT_INFO_MAX_SATELLITES];
	uint8_t azimuth[SAT_INFO_MAX_SATELLITES];
	uint8_t snr[SAT_INFO_MAX_SATELLITES];
	uint8_t prn[SAT_INFO_MAX_SATELLITES];
};

struct sensor_gnss_relative_s
{
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint64_t time_utc_usec;
	uint32_t device_id;
	float position[3];
	float position_accuracy[3];
	float heading;
	float heading_accuracy;
	float position_length;
	float accuracy_length;
	uint16_t reference_station_id;
	bool gnss_fix_ok;
	bool differential_solution;
	bool relative_position_valid;
	bool carrier_solution_floating;
	bool carrier_solution_fixed;
	bool moving_base_mode;
	bool reference_position_miss;
	bool reference_observations_miss;
	bool heading_valid;
	bool relative_position_normalized;
};
