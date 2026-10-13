/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file gps-femto-test.cpp
 *
 * Feeds Femtomes binary frames through the real parser. A CRC-valid frame must not be read beyond the payload it
 * carries: a UAVGPS shorter than its struct is dropped, and a UAVSTATUS reports only the satellite records that
 * are actually present, whatever count it announces. The receive buffer is reused between frames, so the missing
 * bytes would otherwise come from the previous frame.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>

#include "femtomes.h"
#include "crc.h"

using Bytes = std::vector<uint8_t>;

static int failures = 0;

#define CHECK(cond) \
	do { \
		if (!(cond)) { \
			std::printf("CHECK failed: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
			++failures; \
		} \
	} while (0)

static void put16(Bytes &b, uint16_t v)
{
	b.push_back(uint8_t(v));
	b.push_back(uint8_t(v >> 8));
}

static void put32(Bytes &b, uint32_t v)
{
	for (int i = 0; i < 4; ++i) {
		b.push_back(uint8_t(v >> (8 * i)));
	}
}

// 28-byte header, payload, and the CRC32 the driver computes over header then payload
static Bytes femtoFrame(uint16_t message_id, const Bytes &payload)
{
	Bytes header{0xaa, 0x44, 0x12, 28};
	put16(header, message_id);
	header.push_back(0); // messagetype
	header.push_back(0); // portaddr
	put16(header, uint16_t(payload.size()));
	put16(header, 0);    // sequence
	header.push_back(0); // idletime
	header.push_back(0); // timestatus
	put16(header, 0);    // week
	put32(header, 0);    // tow
	put32(header, 0);    // recvstatus
	put16(header, 0);    // resv
	put16(header, 0);    // recvswver

	Bytes frame = header;
	frame.insert(frame.end(), payload.begin(), payload.end());

	uint32_t crc = calculateCRC32(uint32_t(header.size()), header.data(), 0);
	Bytes payload_copy = payload;
	crc = calculateCRC32(uint32_t(payload_copy.size()), payload_copy.data(), crc);
	put32(frame, crc);
	return frame;
}

class Receiver
{
public:
	static int callback(GPSCallbackType type, void *data, int size, void *user)
	{
		return static_cast<Receiver *>(user)->handle(type, data, size);
	}

	void queue(const Bytes &bytes)
	{
		_incoming.insert(_incoming.end(), bytes.begin(), bytes.end());
	}

private:
	std::deque<uint8_t> _incoming;

	int handle(GPSCallbackType type, void *data, int size)
	{
		if (type == GPSCallbackType::readDeviceData) {
			int timeout;
			memcpy(&timeout, data, sizeof(timeout));

			if (_incoming.empty()) {
				// nothing more to read: move the clock past the driver's deadline
				gps_test_time += uint64_t(timeout) * 1000 + 1;
				return 0;
			}

			gps_test_time += 1000;
			const size_t count = std::min(_incoming.size(), size_t(size));

			for (size_t i = 0; i < count; ++i) {
				static_cast<uint8_t *>(data)[i] = _incoming.front();
				_incoming.pop_front();
			}

			return static_cast<int>(count);
		}

		return 0; // writes are accepted and ignored
	}
};

// the 40 fixed bytes of a UAVSTATUS payload (sat_number last) followed by one 8-byte record per satellite id
static Bytes uavStatus(uint32_t announced_sat_number, const std::vector<uint8_t> &svids)
{
	Bytes p(36, 0);
	put32(p, announced_sat_number);

	for (uint8_t svid : svids) {
		p.push_back(svid); // svid
		p.push_back(1);    // system_id
		p.push_back(40);   // cn0
		p.push_back(45);   // ele
		put16(p, 180);     // azi
		put16(p, 0);       // status
	}

	return p;
}

static Bytes uavGps(int32_t lat_1e7, int32_t lon_1e7, uint8_t fix_type, uint8_t sats)
{
	femto_uav_gps_t gps{};
	gps.lat = lat_1e7;
	gps.lon = lon_1e7;
	gps.fix_type = fix_type;
	gps.satellites_used = sats;
	Bytes p(sizeof(gps));
	memcpy(p.data(), &gps, sizeof(gps));
	return p;
}

int main()
{
	Receiver receiver;
	sensor_gnss_s position{};
	satellite_info_s satellites{};
	GPSDriverFemto driver{Receiver::callback, &receiver, &position, &satellites};
	gps_test_time = 0;

	// A UAVSTATUS that carries the two records it announces
	receiver.queue(femtoFrame(8017, uavStatus(2, {5, 7})));
	CHECK(driver.receive(100) == 2);
	CHECK(satellites.count == 2);
	CHECK(satellites.svid[0] == 5);
	CHECK(satellites.svid[1] == 7);

	// A UAVSTATUS announcing three satellites but carrying one record: only that record may be reported
	receiver.queue(femtoFrame(8017, uavStatus(3, {9})));
	CHECK(driver.receive(100) == 2);
	CHECK(satellites.count == 1);
	CHECK(satellites.svid[0] == 9);

	// A UAVSTATUS whose fixed part is cut short (20 of 40 bytes) is dropped
	receiver.queue(femtoFrame(8017, Bytes(20, 0)));
	CHECK(driver.receive(100) == -1);
	CHECK(satellites.count == 1);

	// A complete UAVGPS
	receiver.queue(femtoFrame(8001, uavGps(473000000, 85000000, 3, 12)));
	CHECK(driver.receive(100) == 1);
	CHECK(std::fabs(position.latitude - 47.3) < 1e-9);
	CHECK(position.fix_type == 3);
	CHECK(position.satellites_used == 12);

	// A UAVGPS carrying only its first 20 bytes: nothing may be published from the reused buffer
	position = {};
	receiver.queue(femtoFrame(8001, Bytes(20, 0)));
	CHECK(driver.receive(100) == -1);
	CHECK(position.fix_type == 0);
	CHECK(position.satellites_used == 0);
	CHECK(position.timestamp == 0);

	std::printf("%s\n", failures ? "FAILED" : "PASSED");
	return failures ? 1 : 0;
}
