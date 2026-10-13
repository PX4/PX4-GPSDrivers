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
 * @file gps-erb-test.cpp
 *
 * Feeds ERB frames through the real Emlid Reach parser. A checksum-valid frame whose payload is shorter than the
 * message it announces must not produce a position: the parser's buffer is reused between frames, so the missing
 * fields would otherwise be read from the previous frame.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>

#include "emlid_reach.h"

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

static void putDouble(Bytes &b, double v)
{
	uint8_t raw[sizeof(v)];
	memcpy(raw, &v, sizeof(v));
	b.insert(b.end(), raw, raw + sizeof(raw));
}

// sync bytes, id, 16-bit payload length, payload, and the Fletcher checksum over everything after the sync bytes
static Bytes erbFrame(uint8_t id, const Bytes &payload)
{
	Bytes frame{0x45, 0x52, id, uint8_t(payload.size() & 0xff), uint8_t(payload.size() >> 8)};
	frame.insert(frame.end(), payload.begin(), payload.end());

	uint8_t cka = 0;
	uint8_t ckb = 0;

	for (size_t i = 2; i < frame.size(); ++i) {
		cka = uint8_t(cka + frame[i]);
		ckb = uint8_t(ckb + cka);
	}

	frame.push_back(cka);
	frame.push_back(ckb);
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

static Bytes navStatus(uint32_t time_gps, uint8_t fix_type, uint8_t fix_status, uint8_t sats)
{
	Bytes p;
	put32(p, time_gps);
	put16(p, 0); // weekGPS
	p.push_back(fix_type);
	p.push_back(fix_status);
	p.push_back(sats);
	return p;
}

static Bytes velocity(uint32_t time_gps)
{
	Bytes p;
	put32(p, time_gps);

	for (int i = 0; i < 6; ++i) {
		put32(p, 0); // velN, velE, velD, speed, heading, speedAccuracy
	}

	return p;
}

int main()
{
	Receiver receiver;
	sensor_gnss_s position{};
	satellite_info_s satellites{};
	GPSDriverEmlidReach driver{Receiver::callback, &receiver, &position, &satellites};
	gps_test_time = 0;

	// A complete position: NAV_STATUS (RTK fix, 12 satellites), GEODIC_POSITION and VELOCITY_NED for the same time
	Bytes full_position;
	put32(full_position, 1000);
	putDouble(full_position, 8.5);   // longitude
	putDouble(full_position, 47.3);  // latitude
	putDouble(full_position, 500.0); // ellipsoid altitude
	putDouble(full_position, 450.0); // MSL altitude
	put32(full_position, 20);        // accHorizontal, mm
	put32(full_position, 30);        // accVertical, mm

	receiver.queue(erbFrame(0x03, navStatus(1000, 3, 1, 12)));
	receiver.queue(erbFrame(0x02, full_position));
	receiver.queue(erbFrame(0x05, velocity(1000)));

	CHECK(driver.receive(100) == 1);
	CHECK(std::fabs(position.latitude - 47.3) < 1e-9);
	CHECK(std::fabs(position.longitude - 8.5) < 1e-9);
	CHECK(position.fix_type == 6);
	CHECK(position.satellites_used == 12);

	// A GEODIC_POSITION carrying only its 4-byte time, with a valid checksum, followed by a matching velocity.
	// It announces a position it does not carry, so no position may come out of it.
	position = {};
	Bytes short_position;
	put32(short_position, 2000);

	receiver.queue(erbFrame(0x02, short_position));
	receiver.queue(erbFrame(0x05, velocity(2000)));

	CHECK(driver.receive(100) == -1);
	CHECK(position.latitude == 0.0);
	CHECK(position.longitude == 0.0);
	CHECK(position.timestamp == 0);

	// The parser still accepts the next complete frame set
	receiver.queue(erbFrame(0x02, full_position));
	receiver.queue(erbFrame(0x05, velocity(1000)));

	CHECK(driver.receive(100) == 1);
	CHECK(std::fabs(position.latitude - 47.3) < 1e-9);

	std::printf("%s\n", failures ? "FAILED" : "PASSED");
	return failures ? 1 : 0;
}
