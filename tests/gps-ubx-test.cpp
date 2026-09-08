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
	int poll_read_error = 0;
	unsigned failed_reads = 0;
	size_t read_chunk = 7; // Exercise packet fragmentation through the real parser.
	unsigned comms_polls = 0;
	bool fail_comms_write = false;
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

	void bufferWarning(bool valid_checksum = true)
	{
		const std::string warning = "txbuf alloc";
		Bytes bytes = packet(UBX_MSG_INF_ERROR, Bytes(warning.begin(), warning.end()));

		if (!valid_checksum) {
			bytes.back() ^= 0xff;
		}

		queue(bytes);
	}

	void queue(const Bytes &bytes)
	{
		incoming.insert(incoming.end(), bytes.begin(), bytes.end());
	}

private:
	Bytes outgoing;
	std::deque<uint8_t> incoming;

	void process(const Bytes &bytes)
	{
		const auto message = uint16_t(littleEndian(bytes, 2, 2));
		const Bytes payload(bytes.begin() + 6, bytes.end() - 2);
		CHECK(bytes == packet(message, payload)); // Validate outgoing framing/checksum.

		if (message == UBX_MSG_MON_COMMS) {
			CHECK(payload.empty());
			++comms_polls;
			return;
		}

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

			if (polls > 0 && poll_read_error < 0) {
				++failed_reads;
				gps_test_time += 1000;
				return poll_read_error;
			}

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

			if (fail_comms_write && outgoing.empty() && size >= 4
			    && bytes[2] == 0x0a && bytes[3] == 0x36) {
				++comms_polls;
				return -1;
			}

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

struct Fixture {
	Receiver receiver;
	sensor_gps_s position{};
	GPSDriverUBX driver{GPSHelper::Interface::UART, Receiver::callback, &receiver,
			    &position, nullptr, GPSDriverUBX::Settings{}};

	Fixture()
	{
		gps_test_time = 0;
		gps_test_warnings.clear();
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

	void readFailure(int error)
	{
		receiver.poll_read_error = error;
		CHECK(configure() < 0);
		CHECK(!driver.receiverReady());
		CHECK(receiver.failed_reads == 1);
		CHECK(receiver.polls == 1);
		CHECK(receiver.modes == std::vector<uint32_t>({0}));
		CHECK(receiver.starts == 0);
		CHECK(receiver.status_callbacks == 0);
		CHECK(receiver.rtcm_enables == 0);
		CHECK(gps_test_time - receiver.disabled_at < 100000);

		if (error == GPSHelper::ReadCancelled) {
			CHECK(gps_test_warnings.empty());

		} else {
			CHECK(gps_test_warnings == std::vector<std::string>{"ubx poll_or_read err"});
		}
	}
};

static Bytes commsPayload()
{
	Bytes payload(88, 0);
	payload[1] = 2;
	payload[2] = 2;
	// USB: 11800 bytes pending, current usage 100%, historical peak 101%.
	payload[9] = 3;
	payload[10] = 0x18;
	payload[11] = 0x2e;
	payload[16] = 100;
	payload[17] = 101;
	payload[18] = 12;
	payload[24] = 3;
	payload[26] = 4;
	payload[44] = 0x40;
	payload[45] = 0xe2;
	payload[46] = 1;
	// UART2: no current congestion, despite a historical peak of 108%.
	payload[48] = 1;
	payload[49] = 2;
	payload[57] = 108;
	return payload;
}

static void commsDiagnostics()
{
	Fixture f;
	const Bytes reply = packet(UBX_MSG_MON_COMMS, commsPayload());
	f.receiver.queue(reply);
	f.driver.receive(100);
	CHECK(gps_test_warnings.empty());
	CHECK(f.receiver.comms_polls == 0);
	f.receiver.bufferWarning();
	f.driver.receive(100);
	CHECK(f.receiver.comms_polls == 1);
	CHECK(gps_test_warnings == std::vector<std::string>{"ubx msg: txbuf alloc"});
	gps_test_warnings.clear();
	f.receiver.queue(reply);
	CHECK(f.driver.receive(100) < 0); // Diagnostic traffic alone is not a position update.
	const std::vector<std::string> expected{
		"MON-COMMS after txbuf: txErrors=0x02 ports=2 (snapshot after warning)",
		"MON-COMMS USB port=0x0300 txPending=11800 txUsage=100% txPeakUsage=101% "
		"rxPending=12 rxUsage=3% overrunErrs=4 skipped=123456",
		"MON-COMMS UART2 port=0x0201 txPending=0 txUsage=0% txPeakUsage=108% "
		"rxPending=0 rxUsage=0% overrunErrs=0 skipped=0"
	};
	CHECK(gps_test_warnings == expected);
	gps_test_warnings.clear();
	f.receiver.queue(reply);
	f.driver.receive(100);
	CHECK(gps_test_warnings.empty());
}

static void invalidCommsDiagnostics()
{
	Bytes payload = commsPayload();
	Bytes corrupt = packet(UBX_MSG_MON_COMMS, payload);
	corrupt.back() ^= 0xff;
	std::vector<Bytes> invalid{
		corrupt,
		packet(UBX_MSG_MON_COMMS, Bytes(7, 0)),
		packet(UBX_MSG_MON_COMMS, Bytes(87, 0)),
		packet(UBX_MSG_MON_COMMS, Bytes(368, 0))
	};
	payload[0] = 1;
	invalid.push_back(packet(UBX_MSG_MON_COMMS, payload));
	payload[0] = 0;
	payload[1] = 3;
	invalid.push_back(packet(UBX_MSG_MON_COMMS, payload));
	payload[1] = 255;
	invalid.push_back(packet(UBX_MSG_MON_COMMS, payload));

	for (const auto &reply : invalid) {
		Fixture f;
		f.receiver.bufferWarning();
		f.driver.receive(100);
		gps_test_warnings.clear();
		f.receiver.queue(reply);
		f.driver.receive(100);
		CHECK(gps_test_warnings.empty());
		// Malformed input must not consume the pending reply or lose framing.
		f.receiver.queue(packet(UBX_MSG_MON_COMMS, Bytes(8, 0)));
		f.driver.receive(100);
		CHECK(gps_test_warnings == std::vector<std::string>{
			"MON-COMMS after txbuf: txErrors=0x00 ports=0 (snapshot after warning)"});
	}
}

static void expiredCommsDiagnostics()
{
	Fixture f;
	f.receiver.bufferWarning();
	f.driver.receive(100);
	CHECK(f.receiver.comms_polls == 1);
	gps_test_warnings.clear();
	gps_test_time += 2000000;
	f.receiver.queue(packet(UBX_MSG_MON_COMMS, commsPayload()));
	f.driver.receive(100);
	CHECK(gps_test_warnings.empty());
	// Expiration must allow a later warning to obtain a fresh snapshot.
	gps_test_time += 5000000;
	f.receiver.bufferWarning();
	f.driver.receive(100);
	CHECK(f.receiver.comms_polls == 2);
	gps_test_warnings.clear();
	f.receiver.queue(packet(UBX_MSG_MON_COMMS, Bytes(8, 0)));
	f.driver.receive(100);
	CHECK(gps_test_warnings.size() == 1);
}

int main()
{
	const struct {
		const char *name;
		void (*run)();
	} cases[] = {
		{"comms-diagnostic-values", commsDiagnostics},
		{"comms-malformed-replies", invalidCommsDiagnostics},
		{"comms-expired-reply", expiredCommsDiagnostics},
		{"buffer-warning-rate-limit", [] {
			Fixture f;
			f.receiver.bufferWarning(false);
			f.driver.receive(100);
			CHECK(f.receiver.comms_polls == 0);
			f.receiver.bufferWarning();
			f.driver.receive(100);
			CHECK(f.receiver.comms_polls == 1);
			f.receiver.bufferWarning();
			f.driver.receive(100);
			CHECK(f.receiver.comms_polls == 1);
			gps_test_time += 5000000;
			f.receiver.bufferWarning();
			f.driver.receive(100);
			CHECK(f.receiver.comms_polls == 2);
		}},
		{"buffer-poll-failure-rate-limit", [] {
			Fixture f;
			f.receiver.fail_comms_write = true;
			f.receiver.bufferWarning();
			f.driver.receive(100);
			f.receiver.bufferWarning();
			f.driver.receive(100);
			CHECK(f.receiver.comms_polls == 1);
			gps_test_time += 5000000;
			f.receiver.fail_comms_write = false;
			f.receiver.bufferWarning();
			f.driver.receive(100);
			CHECK(f.receiver.comms_polls == 2);
		}},
		{"already-stopped", [] { Fixture f; f.success(1); }},
		{"poll-read-failure", [] { Fixture f; f.readFailure(-1); }},
		{"poll-read-errno", [] { Fixture f; f.readFailure(-EIO); }},
		{"poll-read-cancelled", [] { Fixture f; f.readFailure(GPSHelper::ReadCancelled); }},
		{"silent-then-stopped", [] {
			Fixture f;
			f.receiver.replies = {SurveyReply::silent, SurveyReply::stopped};
			f.success(2);
		}},
		{"delayed-stop", [] {
			Fixture f;
			f.receiver.replies = {SurveyReply::active, SurveyReply::active, SurveyReply::stopped};
			f.success(3);
			CHECK(f.receiver.started_at - f.receiver.disabled_at >= 200000);
		}},
		{"completed-survey-is-not-stopped", [] {
			Fixture f;
			f.receiver.replies = {SurveyReply::valid, SurveyReply::stopped};
			f.success(2);
		}},
		{"bad-checksum-is-not-confirmation", [] {
			Fixture f;
			f.receiver.replies = {SurveyReply::bad_checksum, SurveyReply::stopped};
			f.success(2);
		}},
		{"bad-length-is-not-confirmation", [] {
			Fixture f;
			f.receiver.replies = {SurveyReply::bad_length, SurveyReply::stopped};
			f.success(2);
		}},
		{"active-timeout", [] { Fixture f; f.receiver.replies = {SurveyReply::active}; f.timeout(); }},
		{"valid-timeout", [] { Fixture f; f.receiver.replies = {SurveyReply::valid}; f.timeout(); }},
		{"silent-timeout", [] { Fixture f; f.receiver.replies = {SurveyReply::silent}; f.timeout(); }},
		{"reconfigure-does-not-reuse-stop-confirmation", [] {
			Fixture f;
			f.success(1);
			f.receiver = Receiver{};
			f.receiver.replies = {SurveyReply::silent};
			f.timeout();
		}},
		{"disable-nak", [] {
			Fixture f;
			f.receiver.reject_disable = true;
			CHECK(f.configure() < 0);
			CHECK(!f.driver.receiverReady());
			CHECK(f.receiver.modes == std::vector<uint32_t>({0}));
			CHECK(f.receiver.polls == 0 && f.receiver.starts == 0);
		}},
		{"start-nak", [] {
			Fixture f;
			f.receiver.reject_start = true;
			CHECK(f.configure() < 0);
			CHECK(!f.driver.receiverReady());
			CHECK(f.receiver.modes == std::vector<uint32_t>({0, 1}));
			CHECK(f.receiver.polls == 1 && f.receiver.starts == 1);
		}},
		{"poll-write-failure", [] {
			Fixture f;
			f.receiver.fail_poll_write = true;
			CHECK(f.configure() < 0);
			CHECK(!f.driver.receiverReady());
			CHECK(f.receiver.modes == std::vector<uint32_t>({0}));
			CHECK(f.receiver.starts == 0);
		}},
		{"configured-status-callback", [] {
			Fixture f;
			f.success(1);
			f.receiver.survey(SurveyReply::active);
			f.driver.receive(100);
			CHECK(f.receiver.status_callbacks == 1);
		}},
		{"fixed-base-does-not-poll", [] {
			Fixture f;
			f.driver.setBasePosition(47.0, 8.0, 500.0f, 1000.0f);
			CHECK(f.configure() == 0);
			CHECK(f.driver.receiverReady());
			CHECK(f.receiver.modes == std::vector<uint32_t>({2}));
			CHECK(f.receiver.polls == 0 && f.receiver.starts == 0);
			CHECK(f.receiver.rtcm_enables == 1);
		}},
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
