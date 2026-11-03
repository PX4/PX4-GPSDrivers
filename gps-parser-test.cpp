#include "unicore.h"
#include "ubx.h"
#include <cassert>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <vector>

void test_empty()
{
	const char str[] = " ";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);
		assert(result == UnicoreParser::Result::None);
	}
}

void test_garbage()
{
	const char str[] = "#GARBAGE,BLA";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);
		assert(result == UnicoreParser::Result::None);
	}
}

void test_too_long()
{
	const char str[] =
		"#UNIHEADINGA,89,GPS,FINE,2251,168052600,0,0,18,11;SOL_COMPUTED,"
		"NARROW_INT,0.3718,67.0255,-0.7974,0.0000,0.8065,3.3818,\"999\","
		"31,21,21,18,3,01,3,f3,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,1234567890,1234567890,1234567890,1234567890,1234567890,"
		"1234567890,ffffffff";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);
		assert(result == UnicoreParser::Result::None);
	}
}

void test_uniheadinga_wrong_crc()
{
	const char str[] =
		"#UNIHEADINGA,89,GPS,FINE,2251,168052600,0,0,18,11;SOL_COMPUTED,NARROW_INT,"
		"0.3718,67.0255,-0.7974,0.0000,0.8065,3.3818,\"999\",31,21,21,18,3,01,3,f3*"
		"ffffffff";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);

		if (result == UnicoreParser::Result::WrongCrc) {
			return;
		}
	}

	assert(false);
}


void test_uniheadinga()
{
	const char str[] =
		"#UNIHEADINGA,89,GPS,FINE,2251,168052600,0,0,18,11;SOL_COMPUTED,NARROW_INT,0.3718,67.0255,-0.7974,0.0000,0.8065,3.3818,\"999\",31,21,21,18,3,01,3,f3*ece5bb07";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);

		if (result == UnicoreParser::Result::GotHeading) {
			assert(unicore_parser.heading().heading_deg == 67.0255f);
			assert(unicore_parser.heading().baseline_m == 0.3718f);
			return;
		}
	}

	assert(false);
}

void test_uniheadinga_twice()
{
	const char str[] =
		"#UNIHEADINGA,89,GPS,FINE,2251,168052600,0,0,18,11;SOL_COMPUTED,NARROW_INT,0.3718,67.0255,-0.7974,0.0000,0.8065,3.3818,\"999\",31,21,21,18,3,01,3,f3*ece5bb07"
		"#UNIHEADINGA,88,GPS,FINE,2251,175391000,0,0,18,13;SOL_COMPUTED,NARROW_INT,0.3500,88.4714,-1.5969,0.0000,1.0175,2.8484,\"999\",28,18,18,12,3,01,3,f3*2888f737";

	UnicoreParser unicore_parser;

	unsigned num_parsed = 0;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);

		if (result == UnicoreParser::Result::GotHeading) {
			++num_parsed;
		}
	}

	assert(num_parsed == 2);
}

void test_agrica()
{
	const char str[] =
		"#AGRICA,68,GPS,FINE,2063,454587000,0,0,18,38;GNSS,236,19,7,26,6,16,9,4,4,12,10,"
		"9,306.7191,10724.0176,-16.4796,0.0089,0.0070,0.0181,67.9651,29.3584,0.0000,0.003,0.003,0.001,-0.002,0.021,"
		"0.039,0.025,40.07896719907,116.23652055432,67.3108,-2160482.7849,4383625.2350,4084735.7632,0.0140,0.0125,0.0296,0.0107,"
		"0.0198,0.0128,40.07627310896,116.11079363322,65.3740,0.00000000000,0.00000000000,0.0000,454587000,38.000,16.723207,-9.406086,0.000000,0.000000,8,0,0,0*e9402e02";

	UnicoreParser unicore_parser;

	for (unsigned i = 0; i < sizeof(str); ++i) {
		auto result = unicore_parser.parseChar(str[i]);

		if (result == UnicoreParser::Result::GotAgrica) {
			assert(unicore_parser.agrica().velocity_m_s == 0.003f);
			assert(unicore_parser.agrica().velocity_north_m_s == 0.003f);
			assert(unicore_parser.agrica().velocity_east_m_s == 0.001f);
			assert(unicore_parser.agrica().velocity_up_m_s == -0.002f);
			assert(unicore_parser.agrica().stddev_velocity_north_m_s == 0.021f);
			assert(unicore_parser.agrica().stddev_velocity_east_m_s == 0.039f);
			assert(unicore_parser.agrica().stddev_velocity_up_m_s == 0.025f);
			return;
		}
	}

	assert(false);
}

void test_unicore()
{
	test_empty();
	test_garbage();
	test_too_long();
	test_uniheadinga_wrong_crc();
	test_uniheadinga();
	test_uniheadinga_twice();
	test_agrica();
}

/******************************************************************************
 * GPSDriverUBX tests
 ******************************************************************************/

extern "C" uint64_t hrt_absolute_time()
{
	using namespace std::chrono;
	return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

void test_gps_log(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);
}

static std::vector<uint8_t> g_ubx_stream;
static size_t g_ubx_pos = 0;
static unsigned g_ubx_write_count = 0;

static int ubxTestCallback(GPSCallbackType type, void *data1, int data2, void *user)
{
	(void)user;

	switch (type) {
	case GPSCallbackType::readDeviceData: {
		uint8_t *buf = (uint8_t *)data1;
		size_t remaining = g_ubx_stream.size() - g_ubx_pos;
		size_t n = (data2 < (int)remaining) ? (size_t)data2 : remaining;
		memcpy(buf, g_ubx_stream.data() + g_ubx_pos, n);
		g_ubx_pos += n;
		return (int)n;
	}

	case GPSCallbackType::writeDeviceData:
		++g_ubx_write_count;
		return data2;

	default:
		return 0;
	}
}

static void appendUbxFrame(std::vector<uint8_t> &stream, uint8_t msg_class, uint8_t msg_id,
			   const std::vector<uint8_t> &payload)
{
	stream.push_back(0xB5);
	stream.push_back(0x62);
	stream.push_back(msg_class);
	stream.push_back(msg_id);
	stream.push_back((uint8_t)(payload.size() & 0xFF));
	stream.push_back((uint8_t)((payload.size() >> 8) & 0xFF));

	uint8_t ck_a = 0;
	uint8_t ck_b = 0;

	for (size_t i = 2; i < stream.size(); i++) {
		ck_a = (uint8_t)(ck_a + stream[i]);
		ck_b = (uint8_t)(ck_b + ck_a);
	}

	for (uint8_t b : payload) {
		stream.push_back(b);
		ck_a = (uint8_t)(ck_a + b);
		ck_b = (uint8_t)(ck_b + ck_a);
	}

	stream.push_back(ck_a);
	stream.push_back(ck_b);
}

// Unknown class/id: payloadRxInit() routes this to UBX_RXMSG_DISABLE and the
// driver answers with a CFG-MSG rate command, which is observable through
// writeDeviceData. Fails the rate limit only once per driver instance.
static void appendDisableProbe(std::vector<uint8_t> &stream)
{
	appendUbxFrame(stream, 0xFF, 0xFF, std::vector<uint8_t> {});
}

static unsigned runUbxDriver(const std::vector<uint8_t> &stream)
{
	g_ubx_stream = stream;
	g_ubx_pos = 0;
	g_ubx_write_count = 0;

	GPSDriverUBX::Settings settings {};
	settings.dynamic_model = 7;
	settings.dgnss_timeout = 0;
	settings.min_cno = 0;
	settings.min_elev = 0;
	settings.output_rate = 0;
	settings.heading_offset = 0.f;
	settings.uart2_baudrate = 0;
	settings.ppk_output = false;
	settings.jam_det_sensitivity_hi = false;
	settings.mode = GPSDriverUBX::UBXMode::Normal;

	sensor_gps_s gps_position {};
	satellite_info_s satellite_info {};

	GPSDriverUBX *driver = new GPSDriverUBX(GPSDriverUBX::Interface::UART, ubxTestCallback,
						nullptr, &gps_position, &satellite_info, settings);
	driver->receive(100);
	delete driver;

	return g_ubx_write_count;
}

void test_ubx_probe_reaches_driver()
{
	std::vector<uint8_t> stream;
	appendDisableProbe(stream);

	assert(runUbxDriver(stream) > 0);
}

// payloadRxInit() accepts any MON-RF payload of 4 + 24*n blocks, and
// payloadRxAdd() used to store every byte into the 92-byte ubx_buf_t without
// a bounds check. The marching write hits the parser's own state fields and
// wedges the driver: it swallows all further bytes without parsing anything,
// so the probe frame after the oversized one gets no response. With the fix,
// excess bytes are skipped and the driver keeps working.
void test_ubx_oversized_mon_rf()
{
	std::vector<uint8_t> stream;
	appendUbxFrame(stream, 0x0A, 0x38, std::vector<uint8_t>(988, 'A')); // 4 + 24*41 blocks
	appendDisableProbe(stream);

	assert(runUbxDriver(stream) > 0);
}

// A receiver with two RF blocks legitimately sends 52-byte MON-RF frames.
// The excess bytes of oversized payloads need to be skipped while still
// consuming the announced length, so multi-block frames stay
// checksum-delimited and the parser keeps alignment.
void test_ubx_mon_rf_multiblock()
{
	std::vector<uint8_t> stream;
	appendUbxFrame(stream, 0x0A, 0x38, std::vector<uint8_t>(52, 0));
	appendDisableProbe(stream);

	assert(runUbxDriver(stream) > 0);
}

void test_ubx()
{
	test_ubx_probe_reaches_driver();
	test_ubx_oversized_mon_rf();
	test_ubx_mon_rf_multiblock();
}

int main(int, char **)
{
	test_unicore();
	test_ubx();

	return 0;
}