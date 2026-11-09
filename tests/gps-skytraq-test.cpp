#include "skytraq.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

// Keep checks active in Release, too.
#define CHECK(condition) do { if (!(condition)) { \
			throw std::runtime_error(std::string("line ") + std::to_string(__LINE__) + ": " #condition); \
		} } while (0)

using Bytes = std::vector<uint8_t>;

static void appendBE(Bytes &bytes, uint64_t value, size_t width)
{
	for (size_t i = 0; i < width; ++i) {
		bytes.push_back(uint8_t(value >> (8 * (width - 1 - i))));
	}
}

static void appendF64BE(Bytes &bytes, double value)
{
	uint64_t bits = 0;
	std::memcpy(&bits, &value, sizeof(bits));
	appendBE(bytes, bits, 8);
}

/** Wrap a payload (message ID first) in the <0xA0 0xA1> length payload checksum <0x0D 0x0A> frame. */
static Bytes frame(const Bytes &payload)
{
	Bytes bytes{0xA0, 0xA1, uint8_t(payload.size() >> 8), uint8_t(payload.size())};
	bytes.insert(bytes.end(), payload.begin(), payload.end());
	uint8_t checksum = 0;

	for (uint8_t b : payload) {
		checksum ^= b;
	}

	bytes.push_back(checksum);
	bytes.push_back(0x0D);
	bytes.push_back(0x0A);
	return bytes;
}

/** Fields of a Navigation PVT Message (0x75/0xA0) a test wants to vary; the rest are fixed. */
struct Pvt {
	uint8_t fix_mode = 3;
	uint8_t num_sv = 12;
	uint8_t valid = 0x03;   // GPS time of week and week number valid
	double latitude = 24.784879;
	double longitude = 121.008696;
	int32_t altitude_msl_mm = 97260;
	int32_t altitude_ellipsoid_mm = 116860;
	uint32_t horizontal_acc_mm = 351;
	uint32_t vertical_acc_mm = 499;
	int32_t vel_n_mm_s = 100;
	int32_t vel_e_mm_s = -200;
	int32_t vel_d_mm_s = 300;
	uint32_t speed_acc_mm_s = 26;
	uint16_t hdop_cent = 37;
	uint16_t vdop_cent = 58;
	uint16_t heading_cent_deg = 1026;
	uint16_t heading_acc_cent_deg = 7;
	uint8_t flags = 0x00;
	size_t truncate_to = 0; // drop the payload to this many bytes when non-zero
};

static Bytes pvtPayload(const Pvt &pvt)
{
	Bytes p{0x75, 0xA0};
	p.push_back(0x01);          // version
	appendBE(p, 123456789, 4);  // receiver TOW, unused
	appendBE(p, 2026, 2);       // year
	p.push_back(11);            // month
	p.push_back(4);             // day
	p.push_back(7);             // hour
	p.push_back(8);             // minute
	p.push_back(9);             // second
	p.push_back(pvt.valid);
	appendBE(p, 200000000, 4);  // fractional second [ns]
	p.push_back(pvt.fix_mode);
	p.push_back(pvt.num_sv);
	appendF64BE(p, pvt.latitude);
	appendF64BE(p, pvt.longitude);
	appendBE(p, uint32_t(pvt.altitude_ellipsoid_mm), 4);
	appendBE(p, uint32_t(pvt.altitude_msl_mm), 4);
	appendBE(p, pvt.horizontal_acc_mm, 4);
	appendBE(p, pvt.vertical_acc_mm, 4);
	appendBE(p, uint32_t(pvt.vel_n_mm_s), 4);
	appendBE(p, uint32_t(pvt.vel_e_mm_s), 4);
	appendBE(p, uint32_t(pvt.vel_d_mm_s), 4);
	appendBE(p, 0, 4);          // ground speed, unused
	appendBE(p, 0, 2);          // ground course, unused
	appendBE(p, pvt.speed_acc_mm_s, 4);
	appendBE(p, 0, 2);          // course accuracy
	appendBE(p, pvt.hdop_cent, 2);
	appendBE(p, pvt.vdop_cent, 2);
	appendBE(p, pvt.heading_cent_deg, 2);
	appendBE(p, pvt.heading_acc_cent_deg, 2);
	p.push_back(pvt.flags);
	CHECK(p.size() == 86);

	if (pvt.truncate_to != 0) {
		p.resize(pvt.truncate_to);
	}

	return p;
}

/** One satellite block of a Navigation Satellite SV Status Message (0x75/0xA1). */
struct Sv {
	uint8_t gnss_type;
	uint8_t svid;
	uint8_t cn0 = 42;
	int16_t elevation = 45;
	int16_t azimuth = 180;
	uint16_t signal_status = 0x0010; // bit 4: used in normal fix mode
};

static Bytes satPayload(const std::vector<Sv> &svs, int num_svs_override = -1)
{
	Bytes p{0x75, 0xA1};
	p.push_back(0x01);         // version
	appendBE(p, 123456789, 4); // receiver TOW, unused
	p.push_back(uint8_t(num_svs_override >= 0 ? num_svs_override : int(svs.size())));

	for (const Sv &sv : svs) {
		p.push_back(sv.gnss_type);
		p.push_back(sv.svid);
		appendBE(p, 0, 2); // SV status indicator, unused
		p.push_back(sv.cn0);
		appendBE(p, uint16_t(sv.elevation), 2);
		appendBE(p, uint16_t(sv.azimuth), 2);
		appendBE(p, 0, 2); // pseudorange residual, unused
		appendBE(p, sv.signal_status, 2);
	}

	return p;
}

/** Receiver Info and Version (0x75/0x90), padded to the 26 bytes the driver requires. */
/** More satellites than satellite_info_s can hold. */
static std::vector<Sv> moreSatellitesThanFit()
{
	std::vector<Sv> svs;

	for (uint8_t i = 0; i < 50; ++i) {
		svs.push_back({0, uint8_t(1 + i % 32)});
	}

	return svs;
}

static Bytes receiverInfoPayload(unsigned odm_major, unsigned odm_minor, unsigned odm_patch)
{
	Bytes p{0x75, 0x90};
	appendBE(p, (3u << 16) | (9u << 8) | 0u, 4); // kernel 3.9.0
	appendBE(p, (odm_major << 16) | (odm_minor << 8) | odm_patch, 4);
	p.resize(26, 0);
	return p;
}

class Receiver
{
public:
	// Configuration knobs for the failure-path cases.
	bool answer_version_query = true;
	bool send_receiver_info = true;
	bool nack_message_group = false;
	unsigned odm_major = 240;
	size_t read_chunk = 7; // Exercise frame fragmentation through the real parser.

	unsigned version_queries = 0;
	unsigned message_group_configs = 0;
	unsigned rate_configs = 0;
	std::vector<sensor_gnss_relative_s> relative_messages;

	static int callback(GPSCallbackType type, void *data, int size, void *user)
	{
		return static_cast<Receiver *>(user)->handle(type, data, size);
	}

	void queue(const Bytes &payload) { send(frame(payload)); }

	/** Queue a frame whose checksum byte is wrong, so the parser must drop it. */
	void queueCorrupted(const Bytes &payload)
	{
		Bytes bytes = frame(payload);
		bytes[bytes.size() - 3] ^= 0xFF;
		send(bytes);
	}

private:
	Bytes outgoing;
	std::deque<uint8_t> incoming;

	void send(const Bytes &bytes) { incoming.insert(incoming.end(), bytes.begin(), bytes.end()); }

	void ack(const Bytes &ids, bool positive = true)
	{
		Bytes payload{uint8_t(positive ? 0x83 : 0x84)};
		payload.insert(payload.end(), ids.begin(), ids.end());
		queue(payload);
	}

	void process(const Bytes &request)
	{
		// Validate the driver's own framing on the way through.
		const Bytes payload(request.begin() + 4, request.end() - 3);
		CHECK(request == frame(payload));
		CHECK(!payload.empty());

		const uint8_t msg_id = payload[0];

		if (msg_id == 0x75) {
			CHECK(payload.size() >= 2);
			const uint8_t sub_id = payload[1];

			if (sub_id == 0x20) { // Query Receiver Info and Version
				++version_queries;

				if (!answer_version_query) {
					return;
				}

				ack({0x75, 0x20});

				if (send_receiver_info) {
					queue(receiverInfoPayload(odm_major, 7, 32));
				}

				return;
			}

			if (sub_id == 0x21) { // Configure Receiver Message Type
				++message_group_configs;
				ack({0x75, 0x21}, !nack_message_group);
				return;
			}

			if (sub_id == 0x23) { // Configure Navigation Message Rate
				++rate_configs;
				ack({0x75, 0x23});
				return;
			}

			ack({0x75, sub_id}, false);
			return;
		}

		// 0x05 Configure Serial Port, 0x09 Configure Message Type, 0x0E Configure Position Update Rate
		ack({msg_id});
	}

	int handle(GPSCallbackType type, void *data, int size)
	{
		if (type == GPSCallbackType::readDeviceData) {
			int timeout;
			std::memcpy(&timeout, data, sizeof(timeout));
			CHECK(timeout >= 0);

			if (incoming.empty()) {
				// The driver's deadlines use strict comparisons; move past the timeout.
				gps_test_time += uint64_t(timeout) * 1000 + 1;
				return 0;
			}

			gps_test_time += 1000;
			const size_t count = std::min({incoming.size(), size_t(size), read_chunk});

			for (size_t i = 0; i < count; ++i) {
				static_cast<uint8_t *>(data)[i] = incoming.front();
				incoming.pop_front();
			}

			return int(count);
		}

		if (type == GPSCallbackType::writeDeviceData) {
			const auto *bytes = static_cast<const uint8_t *>(data);
			outgoing.insert(outgoing.end(), bytes, bytes + size);

			// <0xA0 0xA1> <length> <payload> <checksum> <0x0D 0x0A>
			if (outgoing.size() >= 4) {
				const size_t expected = 7 + (size_t(outgoing[2]) << 8 | outgoing[3]);

				if (outgoing.size() == expected) {
					const Bytes complete = std::move(outgoing);
					outgoing.clear();
					process(complete);
				}
			}

			return size;
		}

		if (type == GPSCallbackType::gotRelativePositionMessage) {
			CHECK(size == int(sizeof(sensor_gnss_relative_s)));
			relative_messages.push_back(*static_cast<sensor_gnss_relative_s *>(data));
		}

		return 0;
	}
};

struct Fixture {
	Receiver receiver;
	sensor_gnss_s position{};
	satellite_info_s satellite_info{};
	GPSDriverSkyTraq driver{Receiver::callback, &receiver, &position, &satellite_info};

	Fixture()
	{
		gps_test_time = 0;
		gps_test_warnings.clear();
	}

	/** Configure at a pinned baudrate, so no rate is probed or changed. */
	int configure()
	{
		unsigned baudrate = 115200;
		GPSHelper::GPSConfig config{};
		config.output_mode = GPSHelper::OutputMode::GPS;
		return driver.configure(baudrate, config);
	}

	void configured()
	{
		CHECK(configure() == 0);
		CHECK(receiver.version_queries == 1);
		CHECK(receiver.message_group_configs == 1);
		CHECK(receiver.rate_configs == 1);
	}

	/** Feed one message and report what receive() claimed was updated. */
	int feed(const Bytes &payload)
	{
		receiver.queue(payload);
		return driver.receive(100);
	}

	static bool warned(const char *needle)
	{
		return std::any_of(gps_test_warnings.begin(), gps_test_warnings.end(),
		[needle](const std::string & warning) { return warning.find(needle) != std::string::npos; });
	}
};

/** Every fix mode AN0037 defines, and the sensor_gnss_s fix type it has to produce. */
static void checkFixModeMapping()
{
	const std::pair<uint8_t, uint8_t> expected[] = {
		{0, 0}, {1, 0}, {2, sensor_gnss_s::FIX_TYPE_2D}, {3, sensor_gnss_s::FIX_TYPE_3D},
		{4, sensor_gnss_s::FIX_TYPE_RTCM_CODE_DIFFERENTIAL},
		{5, sensor_gnss_s::FIX_TYPE_RTK_FLOAT}, {6, sensor_gnss_s::FIX_TYPE_RTK_FIXED},
		{7, 0},
	};

	for (const auto &[fix_mode, fix_type] : expected) {
		Fixture f;
		f.configured();
		Pvt pvt;
		pvt.fix_mode = fix_mode;
		CHECK(f.feed(pvtPayload(pvt)) == 1);
		CHECK(f.position.fix_type == fix_type);
		// Velocity is only trustworthy from a 2D fix up.
		CHECK(f.position.vel_ned_valid == (fix_mode >= 2));
	}
}

int main()
{
	const struct {
		const char *name;
		void (*run)();
	} cases[] = {
		{"configure-succeeds", [] { Fixture f; f.configured(); }},
		{
			"configure-rejects-pre-uav-firmware", [] {
				Fixture f;
				f.receiver.odm_major = 239;
				CHECK(f.configure() < 0);
				// The rejection has to name the version, or nobody can tell which build to flash.
				CHECK(Fixture::warned("239.7.32"));
				CHECK(f.receiver.message_group_configs == 0);
			}
		},
		{
			"configure-continues-without-version-reply", [] {
				Fixture f;
				f.receiver.send_receiver_info = false;
				CHECK(f.configure() == 0);
				CHECK(Fixture::warned("no Receiver Info and Version reply"));
			}
		},
		{
			"configure-fails-without-a-receiver", [] {
				Fixture f;
				f.receiver.answer_version_query = false;
				CHECK(f.configure() < 0);
				CHECK(Fixture::warned("no SkyTraq receiver found"));
			}
		},
		{
			"configure-fails-when-message-group-is-nacked", [] {
				Fixture f;
				f.receiver.nack_message_group = true;
				CHECK(f.configure() < 0);
				CHECK(Fixture::warned("failed to enable SkyTraq navigation message group"));
				CHECK(f.receiver.rate_configs == 0);
			}
		},
		{
			"pvt-fields", [] {
				Fixture f;
				f.configured();
				CHECK(f.feed(pvtPayload(Pvt{})) == 1);
				CHECK(f.position.fix_type == sensor_gnss_s::FIX_TYPE_3D);
				CHECK(f.position.latitude == 24.784879);
				CHECK(f.position.longitude == 121.008696);
				CHECK(std::fabs(f.position.altitude_msl - 97.26) < 1e-9);
				CHECK(std::fabs(f.position.altitude_ellipsoid - 116.86) < 1e-9);
				CHECK(std::fabs(f.position.eph - 0.351f) < 1e-6f);
				CHECK(std::fabs(f.position.epv - 0.499f) < 1e-6f);
				CHECK(std::fabs(f.position.hdop - 0.37f) < 1e-6f);
				CHECK(std::fabs(f.position.vdop - 0.58f) < 1e-6f);
				CHECK(std::fabs(f.position.vel_north - 0.1f) < 1e-6f);
				CHECK(std::fabs(f.position.vel_east + 0.2f) < 1e-6f);
				CHECK(std::fabs(f.position.vel_down - 0.3f) < 1e-6f);
				CHECK(std::fabs(f.position.speed_accuracy - 0.026f) < 1e-6f);
				CHECK(f.position.vel_ned_valid);
				CHECK(f.position.satellites_used == 12);
				CHECK(f.position.time_utc_usec != 0);
			}
		},
		{"pvt-fix-mode-mapping", checkFixModeMapping},
		{
			"pvt-without-valid-time-reports-no-utc", [] {
				Fixture f;
				f.configured();
				Pvt pvt;
				pvt.valid = 0x01; // time of week valid, week number not
				CHECK(f.feed(pvtPayload(pvt)) == 1);
				CHECK(f.position.time_utc_usec == 0);
			}
		},
		{
			"interference-states-pass-through", [] {
				Fixture f;
				f.configured();
				Pvt pvt;
				pvt.flags = uint8_t((2 << 1) | (3 << 3)); // spoofing mitigated, jamming detected
				CHECK(f.feed(pvtPayload(pvt)) == 1);
				CHECK(f.position.spoofing_state == sensor_gnss_s::SPOOFING_STATE_MITIGATED);
				CHECK(f.position.jamming_state == sensor_gnss_s::JAMMING_STATE_DETECTED);
			}
		},
		{
			"heading-is-reported-as-a-relative-position", [] {
				Fixture f;
				f.configured();
				Pvt pvt;
				pvt.flags = 0x01;             // bit 0: vehicle heading valid
				pvt.heading_cent_deg = 27000; // 270 deg, past the wrap point
				pvt.heading_acc_cent_deg = 7;
				CHECK(f.feed(pvtPayload(pvt)) == 1);
				CHECK(f.receiver.relative_messages.size() == 1);
				const sensor_gnss_relative_s &relative = f.receiver.relative_messages.back();
				CHECK(relative.heading_valid);
				CHECK(std::fabs(relative.heading + M_PI_F / 2) < 1e-4f);  // 270 deg -> -pi/2
				CHECK(std::fabs(relative.heading_accuracy - 0.07f * M_DEG_TO_RAD_F) < 1e-6f);
				// Navigation PVT carries no baseline length: unreported, not zero.
				CHECK(std::isnan(relative.position_length));
			}
		},
		{
			"heading-invalid-reports-nan", [] {
				Fixture f;
				f.configured();
				Pvt pvt;
				pvt.flags = 0x00;
				pvt.heading_cent_deg = 1026; // stale value the receiver still ships
				CHECK(f.feed(pvtPayload(pvt)) == 1);
				CHECK(f.receiver.relative_messages.size() == 1);
				const sensor_gnss_relative_s &relative = f.receiver.relative_messages.back();
				CHECK(!relative.heading_valid);
				CHECK(std::isnan(relative.heading));
				CHECK(std::isnan(relative.heading_accuracy));
			}
		},
		{
			"short-pvt-claims-no-update", [] {
				Fixture f;
				f.configured();
				Pvt pvt;
				pvt.truncate_to = 70; // firmware predating the UAV message set
				CHECK(f.feed(pvtPayload(pvt)) < 0);
				CHECK(Fixture::warned("this driver needs 86"));
				CHECK(f.position.timestamp == 0);
			}
		},
		{
			"out-of-range-position-claims-no-update", [] {
				Fixture f;
				f.configured();
				CHECK(f.feed(pvtPayload(Pvt{})) == 1);
				const uint64_t good_timestamp = f.position.timestamp;
				const double good_latitude = f.position.latitude;

				Pvt corrupt;
				corrupt.latitude = 91.0; // a corrupted frame can still pass the 8-bit XOR
				CHECK(f.feed(pvtPayload(corrupt)) < 0);
				CHECK(Fixture::warned("out-of-range position"));
				// The previous fix must not be re-announced as a new one.
				CHECK(f.position.timestamp == good_timestamp);
				CHECK(f.position.latitude == good_latitude);
			}
		},
		{
			"corrupted-frame-is-dropped", [] {
				Fixture f;
				f.configured();
				f.receiver.queueCorrupted(pvtPayload(Pvt{}));
				CHECK(f.driver.receive(100) < 0);
				CHECK(f.position.timestamp == 0);
			}
		},
		{
			"svid-to-prn-mapping", [] {
				Fixture f;
				f.configured();
				const std::vector<Sv> svs{
					{0, 1}, {0, 32},            // GPS       -> 1, 32
					{1, 120}, {1, 145},         // SBAS      -> 33, 58
					{2, 1}, {2, 24},            // GLONASS   -> 65, 88
					{3, 1}, {3, 36},            // Galileo   -> 120, 155
					{4, 193}, {4, 202},         // QZSS      -> 193, 202
					{5, 1}, {5, 33}, {5, 34}, {5, 63}, // BeiDou -> 160, 192, 205, 234
					{6, 1}, {6, 14},            // IRNSS     -> 240, 253
				};
				CHECK(f.feed(satPayload(svs)) == 2);
				const std::vector<uint8_t> expected{
					1, 32, 33, 58, 65, 88, 120, 155, 193, 202, 160, 192, 205, 234, 240, 253};
				CHECK(f.satellite_info.count == expected.size());
				CHECK(std::equal(expected.begin(), expected.end(), f.satellite_info.svid));
				// .prn mirrors .svid, as the PX4 PRN allocation already identifies the constellation.
				CHECK(std::equal(expected.begin(), expected.end(), f.satellite_info.prn));
			}
		},
		{
			"svid-outside-the-documented-range-is-unmapped", [] {
				Fixture f;
				f.configured();
				// Values outside the allocated ranges map to 0 rather than to a PRN that
				// belongs to another constellation.
				CHECK(f.feed(satPayload({{1, 146}, {7, 1}, {0, 33}, {2, 25}})) == 2);
				CHECK(f.satellite_info.count == 4);
				CHECK(f.satellite_info.svid[0] == 0);
				CHECK(f.satellite_info.svid[1] == 0);
				CHECK(f.satellite_info.svid[2] == 0);
				CHECK(f.satellite_info.svid[3] == 0);
			}
		},
		{
			"satellite-fields", [] {
				Fixture f;
				f.configured();
				std::vector<Sv> svs{{0, 1}, {0, 2}};
				svs[0].cn0 = 44;
				svs[0].elevation = 80;
				svs[0].azimuth = 360;
				svs[1].signal_status = 0x0000; // not used in the fix
				svs[1].elevation = -5;         // below the horizon
				CHECK(f.feed(satPayload(svs)) == 2);
				CHECK(f.satellite_info.snr[0] == 44);
				CHECK(f.satellite_info.elevation[0] == 80);
				CHECK(f.satellite_info.azimuth[0] == 255);
				CHECK(f.satellite_info.used[0] == 1);
				CHECK(f.satellite_info.used[1] == 0);
				CHECK(f.satellite_info.elevation[1] == 0);
			}
		},
		{
			"satellites-are-capped-at-the-topic-size", [] {
				Fixture f;
				f.configured();
				CHECK(f.feed(satPayload(moreSatellitesThanFit())) == 2);
				CHECK(f.satellite_info.count == satellite_info_s::SAT_INFO_MAX_SATELLITES);
			}
		},
		{
			"truncated-sat-claims-no-update", [] {
				Fixture f;
				f.configured();
				CHECK(f.feed(satPayload({{0, 1}, {0, 2}})) == 2);
				const uint64_t good_timestamp = f.satellite_info.timestamp;

				// Announce more satellites than the message actually carries.
				CHECK(f.feed(satPayload({{0, 1}}, 8)) < 0);
				CHECK(f.satellite_info.timestamp == good_timestamp);
			}
		},
	};

	bool success = true;

	for (const auto &test : cases) {
		try {
			test.run();
			std::printf("PASS %s\n", test.name);

		} catch (const std::exception &error) {
			std::fprintf(stderr, "FAIL %s: %s\n", test.name, error.what());
			success = false;
		}
	}

	return success ? 0 : 1;
}
