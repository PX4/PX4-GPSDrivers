#include "ubx.h"

#include <algorithm>
#include <cstdio>
#include <deque>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Keep checks active in Release, too.
#define CHECK(condition) do { if (!(condition)) { \
			throw std::runtime_error(std::string("line ") + std::to_string(__LINE__) + ": " #condition); \
		} } while (0)

using Bytes = std::vector<uint8_t>;

static uint32_t littleEndian(const Bytes &bytes, size_t offset, size_t width)
{
	CHECK(offset + width <= bytes.size());
	uint32_t value = 0;

	for (size_t i = 0; i < width; ++i) {
		value |= uint32_t(bytes[offset + i]) << (8 * i);
	}

	return value;
}

static Bytes packet(uint16_t message, const Bytes &payload)
{
	Bytes bytes{0xb5, 0x62, uint8_t(message), uint8_t(message >> 8),
		    uint8_t(payload.size()), uint8_t(payload.size() >> 8)};
	bytes.insert(bytes.end(), payload.begin(), payload.end());
	uint8_t a = 0, b = 0;

	for (size_t i = 2; i < bytes.size(); ++i) {
		a += bytes[i];
		b += a;
	}

	bytes.push_back(a);
	bytes.push_back(b);
	return bytes;
}

enum class SurveyReply { stopped, active, valid, silent, bad_checksum, bad_length };

class Receiver
{
public:
	std::vector<SurveyReply> replies{SurveyReply::stopped};
	bool reject_disable = false;
	bool reject_start = false;
	bool fail_poll_write = false;
	size_t read_chunk = 7; // Exercise packet fragmentation through the real parser.
	unsigned polls = 0;
	unsigned starts = 0;
	unsigned status_callbacks = 0;
	unsigned rtcm_enables = 0;
	std::vector<uint32_t> modes;
	std::map<uint32_t, uint32_t> start_settings;
	gps_abstime disabled_at = 0;
	gps_abstime started_at = 0;

	static int callback(GPSCallbackType type, void *data, int size, void *user)
	{
		return static_cast<Receiver *>(user)->handle(type, data, size);
	}

	void survey(SurveyReply reply)
	{
		if (reply == SurveyReply::silent) {
			return;
		}

		// NAV-SVIN has a 40-byte payload: validity at offset 36, active at 37.
		Bytes payload(40, 0);
		payload[36] = reply == SurveyReply::valid;
		payload[37] = reply == SurveyReply::active;

		if (reply == SurveyReply::bad_length) {
			payload.push_back(0);
		}

		Bytes bytes = packet(UBX_MSG_NAV_SVIN, payload);

		if (reply == SurveyReply::bad_checksum) {
			bytes.back() ^= 0xff;
		}

		queue(bytes);
	}

	struct Satellite {
		uint8_t gnss_id;
		uint8_t sv_id;
		uint8_t cno{0};
		int8_t elev{0};
		int16_t azim{0};
		uint32_t flags{0};
	};

	// NAV-SAT: 8-byte header with numSvs at offset 5, then 12 bytes per satellite:
	// gnssId, svId, cno, elev, azim (2), prRes (2), flags (4).
	void satellites(const std::vector<Satellite> &list)
	{
		Bytes payload(8, 0);
		payload[4] = 1;
		payload[5] = uint8_t(list.size());

		for (const Satellite &sat : list) {
			const Bytes satellite{sat.gnss_id, sat.sv_id, sat.cno, uint8_t(sat.elev),
					      uint8_t(sat.azim), uint8_t(uint16_t(sat.azim) >> 8), 0, 0,
					      uint8_t(sat.flags), uint8_t(sat.flags >> 8), uint8_t(sat.flags >> 16), uint8_t(sat.flags >> 24)};
			payload.insert(payload.end(), satellite.begin(), satellite.end());
		}

		queue(packet(UBX_MSG_NAV_SAT, payload));
	}

	struct Signal {
		uint8_t gnss_id;
		uint8_t sv_id;
		uint8_t sig_id;
		uint8_t cno{0};
		uint8_t quality{0};
		uint16_t flags{0};
	};

	// NAV-SIG: 8-byte header with numSigs at offset 5, then 16 bytes per signal: gnssId, svId, sigId,
	// freqId, prRes (2), cno, qualityInd, corrSource, ionoModel, sigFlags (2), reserved (4).
	void signals(const std::vector<Signal> &list)
	{
		Bytes payload(8, 0);
		payload[5] = uint8_t(list.size());

		for (const Signal &sig : list) {
			const Bytes signal{sig.gnss_id, sig.sv_id, sig.sig_id, 0, 0, 0, sig.cno, sig.quality, 0, 0,
					   uint8_t(sig.flags), uint8_t(sig.flags >> 8), 0, 0, 0, 0};
			payload.insert(payload.end(), signal.begin(), signal.end());
		}

		queue(packet(UBX_MSG_NAV_SIG, payload));
	}

private:
	Bytes outgoing;
	std::deque<uint8_t> incoming;

	void queue(const Bytes &bytes)
	{
		incoming.insert(incoming.end(), bytes.begin(), bytes.end());
	}

	void process(const Bytes &bytes)
	{
		const auto message = uint16_t(littleEndian(bytes, 2, 2));
		const Bytes payload(bytes.begin() + 6, bytes.end() - 2);
		CHECK(bytes == packet(message, payload)); // Validate outgoing framing/checksum.

		if (message == UBX_MSG_MON_VER) {
			CHECK(payload.empty());
			Bytes version(70, 0);
			memcpy(version.data(), "HPG 1.32", 8);
			memcpy(version.data() + 30, "00190000", 8);
			memcpy(version.data() + 40, "MOD=ZED-F9P", 11);
			queue(packet(message, version));
			return;
		}

		if (message == UBX_MSG_NAV_SVIN) {
			CHECK(payload.empty());
			CHECK(!modes.empty() && modes.back() == 0);
			++polls;
			survey(replies.at(std::min<size_t>(polls - 1, replies.size() - 1)));
			return;
		}

		CHECK(message == UBX_MSG_CFG_VALSET);
		CHECK(payload.size() >= 4);
		std::map<uint32_t, uint32_t> settings;

		for (size_t i = 4; i < payload.size();) {
			const uint32_t key = littleEndian(payload, i, 4);
			i += 4;
			const unsigned size_code = (key >> 28) & 7;
			CHECK(size_code >= 1 && size_code <= 4);
			const size_t width = size_code <= 2 ? 1 : size_code == 3 ? 2 : 4;
			settings[key] = littleEndian(payload, i, width);
			i += width;
		}

		bool reject = false;
		const auto mode = settings.find(UBX_CFG_KEY_TMODE_MODE);

		if (mode != settings.end()) {
			modes.push_back(mode->second);

			if (mode->second == 0) {
				disabled_at = gps_test_time;
				reject = reject_disable;

			} else if (mode->second == 1) {
				++starts;
				started_at = gps_test_time;
				start_settings = settings;
				reject = reject_start;
			}
		}

		if (settings[UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1005_I2C + 1] == 1) {
			++rtcm_enables;
		}

		queue(packet(reject ? UBX_MSG_ACK_NAK : UBX_MSG_ACK_ACK,
		{uint8_t(message), uint8_t(message >> 8)}));
	}

	int handle(GPSCallbackType type, void *data, int size)
	{
		if (type == GPSCallbackType::readDeviceData) {
			int timeout;
			memcpy(&timeout, data, sizeof(timeout));
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

			return static_cast<int>(count);
		}

		if (type == GPSCallbackType::writeDeviceData) {
			const auto *bytes = static_cast<const uint8_t *>(data);

			if (fail_poll_write && outgoing.empty() && size >= 4
			    && bytes[2] == 0x01 && bytes[3] == 0x3b) {
				return -1;
			}

			outgoing.insert(outgoing.end(), bytes, bytes + size);

			if (outgoing.size() >= 6 && outgoing.size() == littleEndian(outgoing, 4, 2) + 8) {
				const Bytes complete = std::move(outgoing);
				outgoing.clear();
				process(complete);
			}

			return size;
		}

		if (type == GPSCallbackType::surveyInStatus) {
			++status_callbacks;
		}

		return 0;
	}
};

// GPSHelper keeps its NMEA helpers protected
struct NmeaHelpers : GPSHelper {
	using GPSHelper::nmeaSatellite;
};

static GPSDriverUBX::Settings settings(bool sat_info_per_signal)
{
	GPSDriverUBX::Settings settings{};
	settings.sat_info_per_signal = sat_info_per_signal;
	return settings;
}

struct Fixture {
	Receiver receiver;
	sensor_gnss_s position{};
	satellite_info_s satellite_info{};
	GPSDriverUBX driver;

	explicit Fixture(bool sat_info_per_signal = false) :
		driver{GPSHelper::Interface::UART, Receiver::callback, &receiver, &position, &satellite_info,
		       settings(sat_info_per_signal)}
	{
		gps_test_time = 0;
		driver.setSurveyInSpecs(12500, 60);
	}

	int configure()
	{
		unsigned baudrate = 115200;
		GPSHelper::GPSConfig config{};
		config.output_mode = GPSHelper::OutputMode::RTCM;
		return driver.configure(baudrate, config);
	}

	void success(unsigned expected_polls)
	{
		CHECK(configure() == 0);
		CHECK(driver.receiverReady());
		CHECK(receiver.modes == std::vector<uint32_t>({0, 1}));
		CHECK(receiver.polls == expected_polls);
		CHECK(receiver.starts == 1);
		CHECK(receiver.start_settings.at(UBX_CFG_KEY_TMODE_SVIN_MIN_DUR) == 60);
		CHECK(receiver.start_settings.at(UBX_CFG_KEY_TMODE_SVIN_ACC_LIMIT) == 12500);
		CHECK(receiver.start_settings.at(UBX_CFG_KEY_MSGOUT_UBX_NAV_SVIN_I2C + 1) == 5);
		CHECK(receiver.start_settings.at(UBX_CFG_KEY_MSGOUT_UBX_NAV_SVIN_I2C + 3) == 5);
		CHECK(receiver.status_callbacks == 0);
		CHECK(receiver.rtcm_enables == 0);
	}

	void timeout()
	{
		CHECK(configure() < 0);
		CHECK(!driver.receiverReady());
		CHECK(receiver.modes == std::vector<uint32_t>({0}));
		CHECK(receiver.starts == 0);
		CHECK(receiver.polls > 1 && receiver.polls <= 31);
		CHECK(gps_test_time - receiver.disabled_at >= 3000000);
		CHECK(gps_test_time - receiver.disabled_at < 3300000);
		CHECK(receiver.status_callbacks == 0);
		CHECK(receiver.rtcm_enables == 0);
	}
};

int main()
{
	const struct {
		const char *name;
		void (*run)();
	} cases[] = {
		{"already-stopped", [] { Fixture f; f.success(1); }},
		{
			"silent-then-stopped", [] {
				Fixture f;
				f.receiver.replies = {SurveyReply::silent, SurveyReply::stopped};
				f.success(2);
			}
		},
		{
			"delayed-stop", [] {
				Fixture f;
				f.receiver.replies = {SurveyReply::active, SurveyReply::active, SurveyReply::stopped};
				f.success(3);
				CHECK(f.receiver.started_at - f.receiver.disabled_at >= 200000);
			}
		},
		{
			"completed-survey-is-not-stopped", [] {
				Fixture f;
				f.receiver.replies = {SurveyReply::valid, SurveyReply::stopped};
				f.success(2);
			}
		},
		{
			"bad-checksum-is-not-confirmation", [] {
				Fixture f;
				f.receiver.replies = {SurveyReply::bad_checksum, SurveyReply::stopped};
				f.success(2);
			}
		},
		{
			"bad-length-is-not-confirmation", [] {
				Fixture f;
				f.receiver.replies = {SurveyReply::bad_length, SurveyReply::stopped};
				f.success(2);
			}
		},
		{"active-timeout", [] { Fixture f; f.receiver.replies = {SurveyReply::active}; f.timeout(); }},
		{"valid-timeout", [] { Fixture f; f.receiver.replies = {SurveyReply::valid}; f.timeout(); }},
		{"silent-timeout", [] { Fixture f; f.receiver.replies = {SurveyReply::silent}; f.timeout(); }},
		{
			"reconfigure-does-not-reuse-stop-confirmation", [] {
				Fixture f;
				f.success(1);
				f.receiver = Receiver{};
				f.receiver.replies = {SurveyReply::silent};
				f.timeout();
			}
		},
		{
			"disable-nak", [] {
				Fixture f;
				f.receiver.reject_disable = true;
				CHECK(f.configure() < 0);
				CHECK(!f.driver.receiverReady());
				CHECK(f.receiver.modes == std::vector<uint32_t>({0}));
				CHECK(f.receiver.polls == 0 && f.receiver.starts == 0);
			}
		},
		{
			"start-nak", [] {
				Fixture f;
				f.receiver.reject_start = true;
				CHECK(f.configure() < 0);
				CHECK(!f.driver.receiverReady());
				CHECK(f.receiver.modes == std::vector<uint32_t>({0, 1}));
				CHECK(f.receiver.polls == 1 && f.receiver.starts == 1);
			}
		},
		{
			"poll-write-failure", [] {
				Fixture f;
				f.receiver.fail_poll_write = true;
				CHECK(f.configure() < 0);
				CHECK(!f.driver.receiverReady());
				CHECK(f.receiver.modes == std::vector<uint32_t>({0}));
				CHECK(f.receiver.starts == 0);
			}
		},
		{
			"configured-status-callback", [] {
				Fixture f;
				f.success(1);
				f.receiver.survey(SurveyReply::active);
				f.driver.receive(100);
				CHECK(f.receiver.status_callbacks == 1);
			}
		},
		{
			"fixed-base-does-not-poll", [] {
				Fixture f;
				f.driver.setBasePosition(47.0, 8.0, 500.0f, 1000.0f);
				CHECK(f.configure() == 0);
				CHECK(f.driver.receiverReady());
				CHECK(f.receiver.modes == std::vector<uint32_t>({2}));
				CHECK(f.receiver.polls == 0 && f.receiver.starts == 0);
				CHECK(f.receiver.rtcm_enables == 1);
			}
		},
		{
			"nav-sat-identity", [] {
				Fixture f;
				f.success(1);
				// BeiDou above C37 and QZSS keep their own numbers; IMES (gnssId 4) has no satellites.
				f.receiver.satellites({{3, 1}, {3, 5}, {3, 6}, {3, 37}, {3, 45}, {6, 1}, {0, 32}, {2, 1}, {5, 1}, {4, 1}});
				f.driver.receive(100); // Returns only for a position; the satellites are parsed on the way.
				const std::vector<uint8_t> gnss_ids{3, 3, 3, 3, 3, 6, 0, 2, 5};
				const std::vector<uint8_t> prns{1, 5, 6, 37, 45, 1, 32, 1, 193};
				CHECK(f.satellite_info.count == gnss_ids.size());
				CHECK(std::equal(gnss_ids.begin(), gnss_ids.end(), f.satellite_info.gnss_id));
				CHECK(std::equal(prns.begin(), prns.end(), f.satellite_info.prn));
			}
		},
		{
			"nav-sat-entries", [] {
				Fixture f;
				f.success(1);
				// flags: bits 2..0 qualityInd, bit 3 svUsed, bits 5..4 health, bit 17 rtcmCorrUsed
				f.receiver.satellites({
					{0, 5, 42, 45, 360, 7u | 1u << 3 | 1u << 4 | 1u << 17},
					{6, 255, 30, -91, 100, 4u | 2u << 4},
					{5, 2, 0, -5, 200, 1u},
				});
				f.driver.receive(100);
				const satellite_info_s &s = f.satellite_info;
				CHECK(s.count == 3);
				CHECK(s.gnss_id[0] == 0 && s.prn[0] == 5 && s.band[0] == 0);
				CHECK(s.elevation[0] == 45 && s.azimuth[0] == 0 && s.cn0[0] == 42);
				CHECK(s.tracking[0] == satellite_info_s::TRACKING_CARRIER);
				CHECK(s.flags[0] == (satellite_info_s::FLAG_USED | satellite_info_s::FLAG_CORRECTIONS));
				CHECK(s.gnss_id[1] == 6 && s.prn[1] == 0);
				CHECK(s.elevation[1] == satellite_info_s::ELEVATION_UNKNOWN &&s.azimuth[1] == satellite_info_s::AZIMUTH_UNKNOWN);
				CHECK(s.tracking[1] == satellite_info_s::TRACKING_CODE &&s.flags[1] == satellite_info_s::FLAG_UNHEALTHY);
				CHECK(s.gnss_id[2] == 5 && s.prn[2] == 194 && s.elevation[2] == -5 && s.azimuth[2] == 200);
				CHECK(s.cn0[2] == 0 && s.tracking[2] == satellite_info_s::TRACKING_NONE &&s.flags[2] == 0);
			}
		},
		{
			"nav-sig-entries", [] {
				Fixture f(true);
				f.success(1);
				f.receiver.satellites({{0, 5, 40, 45, 90}, {2, 11, 25, -91, 0}});
				f.driver.receive(100);
				CHECK(f.satellite_info.count == 0); // per signal, NAV-SAT only gives the geometry
				// sigFlags: bits 1..0 health, bit 3 prUsed, bit 7 crCorrUsed
				f.receiver.signals({
					{0, 5, 0, 40, 7, 1u << 3},  // GPS L1 C/A
					{0, 5, 6, 30, 4},           // GPS L5 I
					{0, 5, 7, 33, 5, 1u << 7},  // GPS L5 Q, merges with L5 I
					{2, 11, 5, 25, 0, 2u},      // Galileo E5b I, unhealthy
					{3, 20, 4, 28},             // BeiDou B3I, no NAV-SAT geometry
					{6, 3, 1, 20},              // unknown GLONASS sigId
				});
				f.driver.receive(100);
				const satellite_info_s &s = f.satellite_info;
				CHECK(s.count == 4);
				CHECK(s.gnss_id[0] == 0 && s.prn[0] == 5 && s.band[0] == 1 && s.elevation[0] == 45 && s.azimuth[0] == 90);
				CHECK(s.cn0[0] == 40 && s.tracking[0] == satellite_info_s::TRACKING_CARRIER &&s.flags[0] == satellite_info_s::FLAG_USED);
				CHECK(s.gnss_id[1] == 0 && s.prn[1] == 5 && s.band[1] == 5 && s.cn0[1] == 33);
				CHECK(s.tracking[1] == satellite_info_s::TRACKING_CARRIER &&s.flags[1] == satellite_info_s::FLAG_CORRECTIONS);
				CHECK(s.gnss_id[2] == 2 && s.prn[2] == 11 && s.band[2] == 7 && s.elevation[2] == satellite_info_s::ELEVATION_UNKNOWN);
				CHECK(s.flags[2] == satellite_info_s::FLAG_UNHEALTHY);
				CHECK(s.gnss_id[3] == 3 && s.prn[3] == 20 && s.band[3] == 6 && s.azimuth[3] == satellite_info_s::AZIMUTH_UNKNOWN);
			}
		},
		{
			"nmea-satellite-numbering", [] {
				struct Case {
					const char *sentence;
					int id;
					uint8_t gnss_id;
					uint8_t prn;
				};

				// NMEA 4.10+ restarts the IDs per talker (Galileo 11 is not GPS 11); NMEA 4.0 uses one range under GN
				const Case cases[] {
					{"$GPGSV", 5, satellite_info_s::GNSS_ID_GPS, 5},
					{"$GPGSV", 46, satellite_info_s::GNSS_ID_SBAS, 133},
					{"$GLGSV", 70, satellite_info_s::GNSS_ID_GLONASS, 6},
					{"$GLGSV", 6, satellite_info_s::GNSS_ID_GLONASS, 6},
					{"$GAGSV", 11, satellite_info_s::GNSS_ID_GALILEO, 11},
					{"$GBGSV", 45, satellite_info_s::GNSS_ID_BEIDOU, 45},
					{"$GQGSV", 2, satellite_info_s::GNSS_ID_QZSS, 194},
					{"$GNGSV", 305, satellite_info_s::GNSS_ID_GALILEO, 5},
					{"$GNGSV", 410, satellite_info_s::GNSS_ID_BEIDOU, 10},
				};

				for (const Case &c : cases)
				{
					uint8_t gnss_id = 255;
					uint8_t prn = 0;
					CHECK(NmeaHelpers::nmeaSatellite(reinterpret_cast<const uint8_t *>(c.sentence), c.id, gnss_id, prn));
					CHECK(gnss_id == c.gnss_id && prn == c.prn);
				}

				uint8_t gnss_id = 0;
				uint8_t prn = 0;
				CHECK(!NmeaHelpers::nmeaSatellite(reinterpret_cast<const uint8_t *>("$GAGSV"), 40, gnss_id, prn));
			}
		},
		{
			"nav-sig-not-requested", [] {
				Fixture f;
				f.success(1);
				f.receiver.signals({{0, 5, 0, 40, 7}});
				f.receiver.satellites({{0, 5, 40, 45, 90}});
				f.driver.receive(100);
				CHECK(f.satellite_info.count == 1 && f.satellite_info.band[0] == 0);
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
