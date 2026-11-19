#include "nmea.h"

#include <algorithm>
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

// Append the checksum and line ending to an NMEA sentence body (without '$').
static std::string sentence(const std::string &body)
{
	uint8_t checksum = 0;

	for (char c : body) {
		checksum ^= uint8_t(c);
	}

	char tail[8];
	snprintf(tail, sizeof(tail), "*%02X\r\n", checksum);
	return "$" + body + tail;
}

static std::string gga(const std::string &time)
{
	return sentence("GPGGA," + time + ",4717.11399,N,00833.91590,E,1,08,1.0,499.6,M,48.0,M,,");
}

static std::string rmc(const std::string &time)
{
	return sentence("GPRMC," + time + ",A,4717.11399,N,00833.91590,E,0.004,77.52,191126,,");
}

// VTG carries velocity but no time, so it completes an epoch started by a position sentence.
static std::string vtg()
{
	return sentence("GPVTG,77.52,T,,M,0.004,N,0.008,K,A");
}

class Receiver
{
public:
	std::deque<uint8_t> incoming;
	size_t read_chunk = 7; // Exercise sentence fragmentation through the real parser.

	static int callback(GPSCallbackType type, void *data, int size, void *user)
	{
		return static_cast<Receiver *>(user)->handle(type, data, size);
	}

	void queue(const std::string &text)
	{
		incoming.insert(incoming.end(), text.begin(), text.end());
	}

private:
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
			return size;
		}

		return 0;
	}
};

struct Fixture {
	Receiver receiver;
	sensor_gnss_s position{};
	satellite_info_s satellite_info{};
	GPSDriverNMEA driver{Receiver::callback, &receiver, &position, &satellite_info};

	Fixture()
	{
		gps_test_time = 0;
	}

	// Feed the stream and count the position reports the driver emits before it runs dry.
	unsigned reports(const std::vector<std::string> &stream)
	{
		for (const auto &text : stream) {
			receiver.queue(text);
		}

		unsigned count = 0;

		while (driver.receive(100) > 0) {
			++count;
		}

		CHECK(receiver.incoming.empty());
		return count;
	}
};

int main()
{
	const struct {
		const char *name;
		void (*run)();
	} cases[] = {
		{
			"one-report-per-epoch", [] {
				Fixture f;
				CHECK(f.reports({gga("120000.00"), vtg(), gga("120000.50"), vtg()}) == 2);
			}
		},
		{
			"duplicate-epoch-is-not-reported-twice", [] {
				Fixture f;
				// RMC repeats the GGA epoch; the trailing VTG must not complete a second report.
				CHECK(f.reports({gga("120000.00"), vtg(), rmc("120000.00"), vtg()}) == 1);
			}
		},
		{
			"older-epoch-is-ignored", [] {
				Fixture f;
				CHECK(f.reports({gga("120001.00"), vtg(), gga("120000.00"), vtg()}) == 1);
			}
		},
		{
			"midnight-rollover", [] {
				Fixture f;
				CHECK(f.reports({gga("235959.50"), vtg(), gga("000000.00"), vtg(), gga("000000.50"), vtg()}) == 3);
			}
		},
		{
			"midnight-rollover-rmc", [] {
				Fixture f;
				CHECK(f.reports({rmc("235959.50"), rmc("000000.00"), rmc("000000.50")}) == 3);
			}
		},
		{
			"empty-time-is-not-an-epoch", [] {
				Fixture f;
				// Without a time the sentence cannot be told apart from the previous epoch, nor taken for a rollover.
				CHECK(f.reports({gga("235959.50"), vtg(), gga(""), vtg()}) == 1);
			}
		},
	};

	int failures = 0;

	for (const auto &test : cases) {
		try {
			test.run();
			printf("PASS %s\n", test.name);

		} catch (const std::exception &e) {
			printf("FAIL %s: %s\n", test.name, e.what());
			++failures;
		}
	}

	return failures ? 1 : 0;
}
