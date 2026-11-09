#include "nmea.h"

#include <algorithm>
#include <cstdio>
#include <deque>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

// Keep checks active in Release, too.
#define CHECK(condition) do { if (!(condition)) { \
			throw std::runtime_error(std::string("line ") + std::to_string(__LINE__) + ": " #condition); \
		} } while (0)

// Build "$<body>*CS\r\n" with a valid checksum.
static std::string sentence(const std::string &body)
{
	uint8_t checksum = 0;

	for (char c : body) {
		checksum ^= static_cast<uint8_t>(c);
	}

	char tail[8];
	snprintf(tail, sizeof(tail), "*%02X\r\n", checksum);
	return "$" + body + tail;
}

struct Satellite {
	int prn;
	int elevation;
	int azimuth;
	int snr; // < 0 leaves the field empty
};

// All GSV pages of one constellation/signal; signal < 0 omits the NMEA 4.10 signal ID field.
static std::string gsv(const char *talker, const std::vector<Satellite> &sats, int signal = -1)
{
	const int pages = std::max<int>(1, static_cast<int>((sats.size() + 3) / 4));
	std::string out;

	for (int page = 0; page < pages; page++) {
		std::string body = std::string(talker) + "GSV," + std::to_string(pages) + "," + std::to_string(page + 1) + ","
				   + std::to_string(sats.size());

		for (size_t i = page * 4; i < sats.size() && i < size_t(page * 4 + 4); i++) {
			char record[32];
			snprintf(record, sizeof(record), ",%02d,%02d,%03d,", sats[i].prn, sats[i].elevation, sats[i].azimuth);
			body += record;

			if (sats[i].snr >= 0) {
				body += std::to_string(sats[i].snr);
			}
		}

		if (signal >= 0) {
			body += "," + std::to_string(signal);
		}

		out += sentence(body);
	}

	return out;
}

// One GSA sentence; system_id < 0 omits the NMEA 4.10 system ID field.
static std::string gsa(const char *talker, const std::vector<int> &used, int system_id = -1)
{
	std::string body = std::string(talker) + "GSA,A,3";

	for (size_t i = 0; i < 12; i++) {
		body += ",";

		if (i < used.size()) {
			body += std::to_string(used[i]);
		}
	}

	body += ",1.50,0.80,1.20";

	if (system_id >= 0) {
		body += "," + std::to_string(system_id);
	}

	return sentence(body);
}

static std::string gga()
{
	return sentence("GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,");
}

class Receiver
{
public:
	std::deque<uint8_t> incoming;

	static int callback(GPSCallbackType type, void *data1, int data2, void *user)
	{
		return static_cast<Receiver *>(user)->handle(type, data1, data2);
	}

	int handle(GPSCallbackType type, void *data, int size)
	{
		if (type == GPSCallbackType::readDeviceData) {
			if (incoming.empty()) {
				int timeout;
				memcpy(&timeout, data, sizeof(timeout));
				gps_test_time += uint64_t(timeout) * 1000 + 1;
				return 0;
			}

			gps_test_time += 1000;
			const size_t count = std::min<size_t>({incoming.size(), size_t(size), 13});

			for (size_t i = 0; i < count; ++i) {
				static_cast<uint8_t *>(data)[i] = incoming.front();
				incoming.pop_front();
			}

			return static_cast<int>(count);
		}

		return 0;
	}
};

struct Fixture {
	Receiver receiver;
	sensor_gnss_s position{};
	satellite_info_s satellite_info{};
	GPSDriverNMEA driver{Receiver::callback, &receiver, &position, &satellite_info};
	unsigned satellite_updates = 0;

	Fixture() { gps_test_time = 0; }

	void feed(const std::string &data)
	{
		receiver.incoming.insert(receiver.incoming.end(), data.begin(), data.end());

		while (!receiver.incoming.empty()) {
			const int ret = driver.receive(100);

			if (ret > 0 && (ret & 2)) {
				satellite_updates++;
			}
		}
	}

	std::set<int> svids() const
	{
		return std::set<int>(satellite_info.svid, satellite_info.svid + satellite_info.count);
	}

	int index(int svid) const
	{
		for (int i = 0; i < satellite_info.count; i++) {
			if (satellite_info.svid[i] == svid) {
				return i;
			}
		}

		return -1;
	}

	bool used(int svid) const { return index(svid) >= 0 && satellite_info.used[index(svid)]; }
	int snr(int svid) const { return index(svid) >= 0 ? satellite_info.snr[index(svid)] : -1; }
};

static const std::vector<Satellite> gps_sats{
	{2, 45, 120, 40}, {5, 30, 200, 35}, {12, 60, 300, 42}, {15, 20, 90, 30}, {25, 10, 45, -1}, {29, 70, 359, 44},
	{46, 35, 210, 38}, // SBAS, NMEA number 46 -> 133
};
static const std::vector<Satellite> glonass_sats{{65, 40, 100, 33}, {66, 25, 250, 28}, {74, 55, 10, 36}, {81, 15, 180, -1}, {82, 50, 270, 31}};
static const std::vector<Satellite> galileo_sats{{2, 50, 60, 39}, {5, 35, 140, 37}, {7, 20, 220, 33}, {30, 65, 320, 41}};

static void multi_constellation_epoch()
{
	Fixture f;
	f.feed(gga() + gsa("GN", {2, 5, 12, 29}, 1) + gsa("GN", {65, 74}, 2) + gsa("GN", {2, 30}, 3)
	       + gsv("GP", gps_sats, 1) + gsv("GL", glonass_sats, 1) + gsv("GA", galileo_sats, 7));

	// Before: every talker restarted at index 0, so only the last constellation's satellites survived.
	CHECK(f.satellite_info.count == 16);
	CHECK(f.svids() == std::set<int>({2, 5, 12, 15, 25, 29, 133, 65, 66, 74, 81, 82, 212, 215, 217, 240}));
	CHECK(f.satellite_updates == 3);

	// GSA decides "used", not a non-zero SNR, and Galileo E2 does not collide with GPS G2.
	CHECK(f.used(2) && f.used(5) && f.used(12) && f.used(29));
	CHECK(!f.used(15) && !f.used(133));
	CHECK(f.used(65) && f.used(74) && !f.used(66));
	CHECK(f.used(212) && f.used(240) && !f.used(215));

	// Azimuth uses satellite_info's 0-255 scale instead of wrapping degrees into a byte.
	CHECK(f.satellite_info.azimuth[f.index(29)] == 254);
	CHECK(f.satellite_info.azimuth[f.index(12)] == 212);
	CHECK(f.satellite_info.elevation[f.index(12)] == 60);
	CHECK(f.snr(25) == 0 && f.snr(29) == 44);
	CHECK(f.satellite_info.prn[f.index(133)] == 133);
}

static void next_epoch_replaces_only_its_constellation()
{
	Fixture f;
	f.feed(gga() + gsv("GP", gps_sats) + gsv("GL", glonass_sats) + gsv("GA", galileo_sats));
	CHECK(f.satellite_info.count == 16);

	// GPS now sees fewer satellites; GLONASS and Galileo keep theirs until their own GSV arrives.
	f.feed(gga() + gsv("GP", {{2, 46, 121, 41}, {12, 61, 301, 43}}));
	CHECK(f.satellite_info.count == 11);
	CHECK(f.index(5) < 0 && f.index(133) < 0);
	CHECK(f.snr(2) == 41);
	CHECK(f.index(65) >= 0 && f.index(240) >= 0);

	// Unused slots are cleared rather than keeping the previous epoch's satellites.
	uint8_t *end = f.satellite_info.svid + satellite_info_s::SAT_INFO_MAX_SATELLITES;
	CHECK(std::all_of(f.satellite_info.svid + f.satellite_info.count, end, [](uint8_t s) { return s == 0; }));
}

static void signals_merge_into_one_entry_per_satellite()
{
	Fixture f;
	// L1 C/A (signal 1) then L5 (signal 8): G2 is stronger on L5, G7 is only tracked on L5.
	f.feed(gga() + gsv("GP", {{2, 45, 120, 40}, {5, 30, 200, 35}}, 1) + gsv("GP", {{2, 45, 120, 46}, {7, 25, 80, 30}}, 8));
	CHECK(f.svids() == std::set<int>({2, 5, 7}));
	CHECK(f.snr(2) == 46 && f.snr(5) == 35);

	// The next epoch's first signal starts a new round instead of accumulating forever.
	f.feed(gga() + gsv("GP", {{5, 31, 201, 36}}, 1));
	CHECK(f.svids() == std::set<int>({5}));
}

static void overflow_keeps_used_then_strongest()
{
	Fixture f;
	std::vector<Satellite> gps, glonass, galileo, beidou;
	std::vector<int> used_gps;

	for (int i = 0; i < 16; i++) { gps.push_back({1 + i, 30, 10 * i, 20 + i}); }

	for (int i = 0; i < 12; i++) { glonass.push_back({65 + i, 30, 10 * i, 25 + i}); }

	for (int i = 0; i < 12; i++) { galileo.push_back({1 + i, 30, 10 * i, 30 + i}); }

	for (int i = 0; i < 12; i++) { beidou.push_back({6 + i, 30, 10 * i, 15 + i}); }

	// The four weakest GPS satellites are used in the fix, so they must survive.
	used_gps = {1, 2, 3, 4};

	f.feed(gga() + gsa("GN", used_gps, 1) + gsv("GP", gps) + gsv("GL", glonass) + gsv("GA", galileo) + gsv("GB", beidou));

	CHECK(f.satellite_info.count == satellite_info_s::SAT_INFO_MAX_SATELLITES);

	for (int prn : used_gps) {
		CHECK(f.used(prn));
	}

	// 52 reported, 40 kept: 4 used plus the 36 strongest unused (SNR 26 and up, and two of the three at 25:
	// a tie never displaces a satellite that is already in the table).
	std::vector<int> unused_snrs;

	for (int i = 0; i < f.satellite_info.count; i++) {
		if (!f.satellite_info.used[i]) {
			unused_snrs.push_back(f.satellite_info.snr[i]);
		}
	}

	CHECK(unused_snrs.size() == 36);
	CHECK(*std::min_element(unused_snrs.begin(), unused_snrs.end()) == 25);
	CHECK(std::count_if(unused_snrs.begin(), unused_snrs.end(), [](int snr) { return snr >= 26; }) == 34);
	CHECK(f.index(211) >= 0 && f.index(222) >= 0); // all of Galileo, the strongest constellation, is kept
}

static void legacy_gps_only_receiver()
{
	Fixture f;
	// NMEA 2.3, no GSA: keep the old "tracked means used" behavior.
	f.feed(gga() + gsv("GP", {{3, 10, 30, 0}, {6, 40, 90, 38}, {9, 20, 180, -1}}));
	CHECK(f.svids() == std::set<int>({3, 6, 9}));
	CHECK(f.used(6) && !f.used(3) && !f.used(9));
}

static void combined_talker_gsa_without_system_id()
{
	Fixture f;
	// NMEA 4.0 GNGSA: the system is inferred from the satellite number.
	const std::string gsv_gps = gsv("GP", {{2, 45, 120, 40}, {5, 30, 200, 35}});
	const std::string gsv_glonass = gsv("GL", {{65, 40, 100, 33}, {74, 55, 10, 36}});
	f.feed(gga() + gsa("GN", {2, 74}) + gsa("GN", {65}) + gsv_gps + gsv_glonass);
	CHECK(f.used(2) && f.used(74) && f.used(65) && !f.used(5));

	// The next epoch's GSA replaces the used set.
	f.feed(gga() + gsa("GN", {5}) + gsv_gps);
	CHECK(f.used(5) && !f.used(2) && !f.used(65));
}

static void beidou_and_qzss_numbering()
{
	Fixture f;
	const std::string gsv_beidou = gsv("GB", {{1, 40, 100, 30}, {6, 40, 100, 31}, {37, 40, 100, 32}, {42, 40, 100, 33}});
	f.feed(gga() + gsv_beidou + gsv("GQ", {{2, 40, 100, 34}}));
	// Same numbering as GPSDriverUBX's NAV-SAT conversion: C1 -> 159, C6 -> 33, C37 -> 64, C38+ -> 255, J2 -> 194
	CHECK(f.svids() == std::set<int>({159, 33, 64, 255, 194}));
}

int main()
{
	const struct {
		const char *name;
		void (*run)();
	} cases[] = {
		{"multi-constellation-epoch", multi_constellation_epoch},
		{"next-epoch-replaces-only-its-constellation", next_epoch_replaces_only_its_constellation},
		{"signals-merge-into-one-entry-per-satellite", signals_merge_into_one_entry_per_satellite},
		{"overflow-keeps-used-then-strongest", overflow_keeps_used_then_strongest},
		{"legacy-gps-only-receiver", legacy_gps_only_receiver},
		{"combined-talker-gsa-without-system-id", combined_talker_gsa_without_system_id},
		{"beidou-and-qzss-numbering", beidou_and_qzss_numbering},
	};

	int failures = 0;

	for (const auto &test_case : cases) {
		try {
			test_case.run();
			printf("PASS %s\n", test_case.name);

		} catch (const std::exception &error) {
			printf("FAIL %s: %s\n", test_case.name, error.what());
			failures++;
		}
	}

	return failures == 0 ? 0 : 1;
}
