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

/**
 * @file quectel.cpp
 *
 * @author Hongbin Chu <hongbin.chu@qq.com>
 */

#include "quectel.h"

#include <cmath>
#include <stdio.h>
#include <string.h>
#include <ctime>

#define COMMNAD_TIMEOUT_MS  2000


#define MAX(X, Y)        ((X) > (Y) ? (X) : (Y))
#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A' - 0xA)))

#define GNSS_SYSTEM_ID_GPS     1
#define GNSS_SYSTEM_ID_GLONASS 2
#define GNSS_SYSTEM_ID_GALILEO 3
#define GNSS_SYSTEM_ID_BDS     4
#define GNSS_SYSTEM_ID_QZSS    5

GPSDriverQuectel::GPSDriverQuectel(GPSCallbackPtr callback, void* callback_user, struct sensor_gps_s* gps_position) 
    : GPSBaseStationSupport(callback, callback_user), _gps_position(gps_position)
{
    msgFlagInit();
}

GPSDriverQuectel::~GPSDriverQuectel()
{
    delete _rtcm_parsing;
}

int GPSDriverQuectel::configure(unsigned& baudrate, const GPSConfig& config)
{
    UNUSED(baudrate);
    UNUSED(config);

    if(!_rtcm_parsing)
    {
        _rtcm_parsing = new RTCMParsing();
    }

    _rtcm_parsing->reset();

    uint8_t vesion[100]{0};
    if(query_module_verion(vesion) != false)
    {
        QL_INFO("Module version: %s", vesion);
    }
    else
    {
        QL_ERR("Failed to query module version");
    }

    if(get_work_mode(&_work_mode))
    {
        QL_INFO("Current work mode: %s", _work_mode == QL_WORK_MODE_BASE_STATION ? "Base Station" : "Rover");
    }
    else
    {
        QL_ERR("Failed to get current work mode");
    }

    // set the pqtm message output
    set_cmd_msgrate((uint8_t*)"PQTMEPE", 1, 2, COMMNAD_TIMEOUT_MS);
    set_cmd_msgrate((uint8_t*)"PQTMVEL", 1, 1, COMMNAD_TIMEOUT_MS);

    return 0;
}

int GPSDriverQuectel::wait_message(const uint8_t* msg_name, unsigned timeout)
{
    uint8_t buf[64]{0};

    gps_abstime time_started = gps_absolute_time();

    do
    {
        int ret = read(buf, sizeof(buf), 1);

        for(int i = 0; i < ret; i++)
        {
            int len = parseChar(buf[i]);
            if(len > 0)
            {
                QL_DEBUG("Parse message: %.*s", len, _rx_buffer);
                if(strstr((char*)_rx_buffer, (char*)msg_name) != nullptr)
                {
                    return len;  // found the message
                }
            }
        }
    } while(fabs((double)(gps_absolute_time() - time_started)) <= timeout * 1000);

    return 0;  // timeout, no message found
}

int GPSDriverQuectel::receive(unsigned timeout)
{
    uint8_t buf[QL_RX_BUFF_LENGTH]{0};
    /* timeout additional to poll */
    gps_abstime time_started = gps_absolute_time();

    int handled = 0;

    while(true)
    {
        int ret = read(buf, sizeof(buf), timeout);

        if(ret > 0)
        {
            for(int i = 0; i < ret; i++)
            {
                int len = parseChar(buf[i]);
                if(len > 0)
                {
                    handled |= handleMessage(len);
                }
            }

            if(handled > 0)
            {
                return handled;
            }
        }

        if(ret < 0)
        {
            return -1;
        }

        /* in case we keep trying but only get crap from GPS */
        if(time_started + timeout * 1000 < gps_absolute_time())
        {
            return -1;
        }
    }
}

void GPSDriverQuectel::msgFlagInit()
{
    _sat_num_gga = 0;
    _sat_num_gns = 0;
    _sat_num_gsv = 0;
    _sat_num_gpgsv = 0;
    _sat_num_glgsv = 0;
    _sat_num_gagsv = 0;
    _sat_num_gbgsv = 0;
    _sat_num_gqgsv = 0;

    _TIME_received = false;
    _POS_received = false;
    _ALT_received = false;
    _SVNUM_received = false;
    _SVINFO_received = false;
    _FIX_received = false;
    _DOP_received = false;
    _VEL_received = false;
    _EPH_received = false;
    _HEAD_received = false;

    _is_frame_end = false;

    memset(_gps_used_svid, 0, sizeof(_gps_used_svid));
    memset(_gln_used_svid, 0, sizeof(_gln_used_svid));
    memset(_bds_used_svid, 0, sizeof(_bds_used_svid));
    memset(_gal_used_svid, 0, sizeof(_gal_used_svid));
    memset(_qzss_used_svid, 0, sizeof(_qzss_used_svid));
}

unsigned char
GPSDriverQuectel::checkXOR(const unsigned char* pData, const unsigned int lentgh)
{
    unsigned char result = 0;
    unsigned int i = 0;

    if((NULL == pData) || (lentgh < 1))
    {
        return 0;
    }
    for(i = 0; i < lentgh; i++)
    {
        result ^= *(pData + i);
    }

    return result;
}

/**
 * Parse a single byte of the NMEA0183 stream
 *
 * @param data the byte to parse
 * @return the length of the packet if a complete packet has been received, 0 otherwise
 */
int GPSDriverQuectel::parseChar(uint8_t data)
{
    int ret = 0;

    if(_rtcm_parsing)
    {
        if(_rtcm_parsing->addByte(data))
        {
            gotRTCMMessage(_rtcm_parsing->message(), _rtcm_parsing->messageLength());
            _decode_state = QL_DECODE_NONE;
            _rtcm_parsing->reset();
            return 0;
        }
    }

    switch(_decode_state)
    {
        /* First, look for sync */
        case QL_DECODE_NONE:
        {
            if(data == '$')
            {
                _rx_count = 0;
                _rx_buffer[_rx_count++] = data;
                _decode_state = QL_DECODE_HEADER;
            }
        }
        break;

        case QL_DECODE_HEADER:
        {
            _rx_buffer[_rx_count++] = data;

            if(data == '*')
            {
                _decode_state = QL_DECODE_CHECKSUM1;
            }

            if((_rx_count >= QL_NMEA_MSG_LENGTH) || (_rx_count >= QL_RX_BUFF_LENGTH))
            {
                /* Too long, reset */
                _decode_state = QL_DECODE_NONE;
            }
        }
        break;
        case QL_DECODE_CHECKSUM1:
        {
            _rx_ck_a = data;
            _rx_buffer[_rx_count++] = data;
            _decode_state = QL_DECODE_CHECKSUM2;
        }
        break;
        case QL_DECODE_CHECKSUM2:
        {
            _rx_ck_b = data;
            _rx_buffer[_rx_count++] = data;

            uint8_t checksum = checkXOR(_rx_buffer + 1, _rx_count - 4);
 
            /* Check if we have a valid checksum */
            if((HEXDIGIT_CHAR(checksum >> 4) == _rx_ck_a) && (HEXDIGIT_CHAR(checksum & 0x0F) == _rx_ck_b))
            {
                ret = _rx_count;
                _rx_buffer[_rx_count] = 0;  // null-terminate the string
            }
            else
            {
                QL_ERR("checksum error, rx_buffer: %.*s\r\n", _rx_count, _rx_buffer);
            }

            _decode_state = QL_DECODE_NONE;
        }

        break;
    }

    return ret;
}

int GPSDriverQuectel::handleMessage(int packet_len)
{
    _rx_buffer[packet_len] = 0;

    if(_is_frame_end)
    {
        msgFlagInit();
    }

    char* bufptr = (char*)_rx_buffer;
    // need skip preamble
    if(strstr(bufptr, "GGA") != nullptr)
    {
        if(decode_msg_gga(bufptr + 6) < 0)
        {
            QL_ERR("decode_msg_gga failed");
        }
    }
    else if(strstr(bufptr, "RMC") != nullptr)
    {
        if(decode_msg_rmc(bufptr + 6) < 0)
        {
            QL_ERR("decode_msg_rmc failed");
        }
    }
    else if(strstr(bufptr, "GSA") != nullptr)
    {
        if(decode_msg_gsa(bufptr + 6) < 0)
        {
            QL_ERR("decode_msg_gsa failed");
        }
    }
    else if(strstr(bufptr, "GSV") != nullptr)
    {
        if(decode_msg_gsv(bufptr + 6) < 0)
        {
            QL_ERR("decode_msg_gsv failed");
        }
    }
    else if(strstr(bufptr, "VTG") != nullptr)
    {
        if(decode_msg_vtg(bufptr + 6) < 0)
        {
            QL_ERR("decode_msg_vtg failed");
        }
    }
    else if(strstr(bufptr, "$PQTMVEL") != nullptr)
    {
        if(decode_msg_pqtmvel(bufptr + strlen("$PQTMVEL")) < 0)
        {
            QL_ERR("decode_msg_pqtmvel failed");
        }
        _is_frame_end = true;
    }
    else if(strstr(bufptr, "$PQTMEPE") != nullptr)
    {
        if(decode_msg_pqtmepe(bufptr + strlen("$PQTMEPE")) < 0)
        {
            QL_ERR("decode_msg_pqtmepe failed");
        }
    }
    else if(strstr(bufptr, "$PQTMSVINSTATUS") != nullptr)
    {
        if(decode_msg_pqtmsvinstatus(bufptr + strlen("$PQTMSVINSTATUS")) < 0)
        {
            QL_ERR("decode_msg_pqtmsvinstatus failed");
        }
    }
    else
    {
        /// QL_INFO("Unknown message: %s", bufptr);
    }

    if(_is_frame_end)
    {
        return 1;
    }

    return 0;
}

bool GPSDriverQuectel::is_used_svid(uint8_t svid, uint8_t* used_svid)
{
    uint8_t* ptr = used_svid;

    if(ptr == nullptr)
    {
        return false;
    }

    while(*ptr != 0)
    {
        if(*ptr == svid)
        {
            return true;
        }
        ptr++;
    }

    return false;
}

int GPSDriverQuectel::decode_msg_gga(char* bufptr)
{
    /*
      Time, position, and fix related data
      An example of the GBS message string is:
      $xxGGA,time,lat,NS,long,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
      $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F
      $GNGGA,092721.00,2926.688113,N,11127.771644,E,2,08,1.11,106.3,M,-20,M,1.0,3721*53

      Note - The data string exceeds the nmea standard length.
      GGA message fields
      Field   Meaning
      0   Message ID $GPGGA
      1   UTC of position fix
      2   Latitude
      3   Direction of latitude:
      N: North
      S: South
      4   Longitude
      5   Direction of longitude:
      E: East
      W: West
      6   GPS Quality indicator:
      0: Fix not valid
      1: GPS fix
      2: Differential GPS fix, OmniSTAR VBS
      4: Real-Time Kinematic, fixed integers
      5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK
      7   Number of SVs in use, range from 00 through to 24+
      8   HDOP
      9   Orthometric height (MSL reference)
      10  M: unit of measure for orthometric height is meters
      11  Geoid separation
      12  M: geoid separation measured in meters
      13  Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
      14  Reference station ID, range 0000-4095. A null field when any reference station ID is selected and no corrections are received1.
      15
      The checksum data, always begins with *
    */
    char* endp;
    double utc_time = 0.0, lat = 0.0, lon = 0.0;
    float alt = 0.f, geoid_h = 0.f;
    float hdop = 99.9f, dgps_age = NAN;
    int num_of_sv = 0, fix_quality = 0;
    char ns = '?', ew = '?';

    if(bufptr && *(++bufptr) != ',')
    {
        utc_time = strtod(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        lat = strtod(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        ns = *(bufptr++);
    }

    if(bufptr && *(++bufptr) != ',')
    {
        lon = strtod(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        ew = *(bufptr++);
    }

    if(bufptr && *(++bufptr) != ',')
    {
        fix_quality = strtol(bufptr, &endp, 10);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        num_of_sv = strtol(bufptr, &endp, 10);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        hdop = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        alt = strtof(bufptr, &endp);
        bufptr = endp;
    }

    while(*(++bufptr) != ',')
    {
    }  // skip M

    if(bufptr && *(++bufptr) != ',')
    {
        geoid_h = strtof(bufptr, &endp);
        bufptr = endp;
    }

    while(*(++bufptr) != ',')
    {
    }  // skip M

    if(bufptr && *(++bufptr) != ',')
    {
        dgps_age = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(ns == 'S')
    {
        lat = -lat;
    }

    if(ew == 'W')
    {
        lon = -lon;
    }

    /* convert from degrees, minutes and seconds to degrees */
    _gps_position->longitude_deg = int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0;
    _gps_position->latitude_deg = int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0;
    _gps_position->hdop = hdop;
    _gps_position->altitude_msl_m = (double)alt;
    _gps_position->altitude_ellipsoid_m = (double)(alt + geoid_h);
    _sat_num_gga = (uint8_t)num_of_sv;

    if(fix_quality <= 0)
    {
        _gps_position->fix_type = 0;
    }
    else
    {
        _gps_position->fix_type = (uint8_t)fix_quality;
    }

    if(!_POS_received && (_last_POS_timeUTC < utc_time))
    {
        _last_POS_timeUTC = utc_time;
        _POS_received = true;
    }

    _ALT_received = true;
    _SVNUM_received = true;
    _FIX_received = true;

    _gps_position->c_variance_rad = 0.1f;
    _gps_position->timestamp = gps_absolute_time();

    return 1;
}

int GPSDriverQuectel::decode_msg_rmc(char* bufptr)
{
    /*
    Position, velocity, and time
    The RMC string is:

    $xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
    The Talker ID ($--) will vary depending on the satellite system used for the position solution:
    $GNRMC,092721.00,A,2926.688113,N,11127.771644,E,0.780,,200520,,,D,V*1D

    GPRMC message fields
    Field	Meaning
    0	Message ID $GPRMC
    1	UTC of position fix
    2	Status A=active or V=void
    3	Latitude
    4	Longitude
    5	Speed over the ground in knots
    6	Track angle in degrees (True)
    7	Date
    8	Magnetic variation in degrees
    9	The checksum data, always begins with *
    */
    char* endp;
    double utc_time = 0.0;
    char Status = 'V';
    double lat = 0.0, lon = 0.0;
    float ground_speed_K = 0.f;
    float track_true = 0.f;
    int nmea_date = 0;
    float Mag_var = 0.f;
    char ns = '?', ew = '?';

    if(bufptr && *(++bufptr) != ',')
    {
        utc_time = strtod(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        Status = *(bufptr++);
    }

    if(bufptr && *(++bufptr) != ',')
    {
        lat = strtod(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        ns = *(bufptr++);
    }

    if(bufptr && *(++bufptr) != ',')
    {
        lon = strtod(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        ew = *(bufptr++);
    }

    if(bufptr && *(++bufptr) != ',')
    {
        ground_speed_K = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        track_true = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        nmea_date = static_cast<int>(strtol(bufptr, &endp, 10));
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        Mag_var = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(ns == 'S')
    {
        lat = -lat;
    }

    if(ew == 'W')
    {
        lon = -lon;
    }

    if(Status == 'V')
    {
        _gps_position->fix_type = 0;
    }

    float track_rad = track_true * M_PI_F / 180.0f;  // rad in range [0, 2pi]

    if(track_rad > M_PI_F)
    {
        track_rad -= 2.f * M_PI_F;  // rad in range [-pi, pi]
    }

    float velocity_ms = ground_speed_K / 1.9438445f;
    float velocity_north = velocity_ms * cosf(track_rad);
    float velocity_east = velocity_ms * sinf(track_rad);

    /* convert from degrees, minutes and seconds to degrees */
    _gps_position->latitude_deg = int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0;
    _gps_position->longitude_deg = int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0;

    _gps_position->vel_m_s = velocity_ms;
    _gps_position->vel_n_m_s = velocity_north;
    _gps_position->vel_e_m_s = velocity_east;
    _gps_position->cog_rad = track_rad;
    _gps_position->vel_ned_valid = true; /**< Flag to indicate if NED speed is valid */
    _gps_position->c_variance_rad = 0.1f;
    _gps_position->s_variance_m_s = 0;
    _gps_position->timestamp = gps_absolute_time();
    _last_timestamp_time = gps_absolute_time();

#ifndef NO_MKTIME
    int utc_hour = static_cast<int>(utc_time / 10000);
    int utc_minute = static_cast<int>((utc_time - utc_hour * 10000) / 100);
    double utc_sec = static_cast<double>(utc_time - utc_hour * 10000 - utc_minute * 100);
    int nmea_day = static_cast<int>(nmea_date / 10000);
    int nmea_mth = static_cast<int>((nmea_date - nmea_day * 10000) / 100);
    int nmea_year = static_cast<int>(nmea_date - nmea_day * 10000 - nmea_mth * 100) + 2000;  // Quectel uses 2000 as base year
    /*
     * convert to unix timestamp
     */
    struct tm timeinfo = {};
    timeinfo.tm_year = nmea_year - 1900;  // The year starts counting from 1900
    timeinfo.tm_mon = nmea_mth - 1;       // The month starts counting from 0, so you need to subtract 1
    timeinfo.tm_mday = nmea_day;
    timeinfo.tm_hour = utc_hour;
    timeinfo.tm_min = utc_minute;
    timeinfo.tm_sec = int(utc_sec);
    timeinfo.tm_isdst = 0;

    struct tm tm_start = {};
    tm_start.tm_year = 1980 - 1900;  // The year starts counting from 1900
    tm_start.tm_mon = 0;  // 1st January
    tm_start.tm_mday = 6;
    tm_start.tm_hour = 0;
    tm_start.tm_min = 0;
    tm_start.tm_sec = 0;
    tm_start.tm_isdst = 0;

    // GPS epoch starts at 1980-01-06 00:00:00 UTC, so we need to convert the time to microseconds since then
    _gps_position->time_utc_usec = (uint64_t)difftime(mktime(&timeinfo), mktime(&tm_start));
    _gps_position->time_utc_usec = _gps_position->time_utc_usec * 1000 * 1000;  // convert to microseconds
    _gps_position->time_utc_usec += (uint64_t)((utc_sec - (int)utc_sec) * 1000 * 1000);  // add the fractional seconds in microseconds

    uint32_t gps_week = (uint32_t)(_gps_position->time_utc_usec / 1000000 / (7 * 24 * 3600));
    uint32_t gps_time_of_week = (uint32_t)((_gps_position->time_utc_usec / 1000000 % (7 * 24 * 3600)));

    QL_DEBUG("GPS time: %04d-%02d-%02d %02d:%02d:%02.3f UTC, week: %u, time of week: %u",
            nmea_year, nmea_mth, nmea_day, utc_hour, utc_minute, utc_sec, gps_week, gps_time_of_week);

#else
    NMEA_UNUSED(utc_time);
    NMEA_UNUSED(nmea_date);
    _gps_position->time_utc_usec = 0;
#endif

    if(!_POS_received && (_last_POS_timeUTC < utc_time))
    {
        _last_POS_timeUTC = utc_time;
        _POS_received = true;
    }

    if(!_VEL_received && (_last_VEL_timeUTC < utc_time))
    {
        _last_VEL_timeUTC = utc_time;
        _VEL_received = true;
    }

    _TIME_received = true;

    return 1;
}

int GPSDriverQuectel::decode_msg_gsa(char* bufptr)
{
    /*
    GPS DOP and active satellites
    An example of the GSA message string is:
    $GPGSA,<1>,<2>,<3>,<3>,,,,,<3>,<3>,<3>,<4>,<5>,<6>*<7><CR><LF>
    $GNGSA,A,3,82,67,74,68,73,80,83,,,,,,0.99,0.53,0.84,2*09
    $GNGSA,A,3,12,19,06,17,02,09,28,05,,,,,2.38,1.10,2.11,1*05
    $GNGSA,A,3,27,04,16,08,09,26,31,11,,,,,1.96,1.05,1.65,1*08

    GSA message fields
    Field	Meaning
    0	Message ID $GPGSA
    1	Mode 1, M = manual, A = automatic
    2	Mode 2, Fix type, 1 = not available, 2 = 2D, 3 = 3D
    3	PRN number, 01 through 32 for GPS, 33 through 64 for SBAS, 64+ for GLONASS
    4 	PDOP: 0.5 through 99.9
    5	HDOP: 0.5 through 99.9
    6	VDOP: 0.5 through 99.9
    7	The checksum data, always begins with *
    */
    char* endp;
    char M_pos = ' ';
    int fix_mode = 0;
    uint8_t sat_id[12]{0};
    float pdop = 99.9f, hdop = 99.9f, vdop = 99.9f;
    int sys_id = 0;
    int sat_count = 0;
    uint8_t* used_svid = nullptr;

    if(bufptr && *(++bufptr) != ',')
    {
        M_pos = *(bufptr++);
    }

    if(bufptr && *(++bufptr) != ',')
    {
        fix_mode = strtol(bufptr, &endp, 10);
        bufptr = endp;
    }

    for(int y = 0; y < 12; y++)
    {
        if(bufptr && *(++bufptr) != ',')
        {
            sat_count++;
            sat_id[y] = (uint8_t)strtoul(bufptr, &endp, 10);
            bufptr = endp;
        }
    }

    if(bufptr && *(++bufptr) != ',')
    {
        pdop = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        hdop = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        vdop = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        sys_id = strtol(bufptr, &endp, 10);
        bufptr = endp;
    }

    if(fix_mode <= 1)
    {
        _gps_position->fix_type = 0;
    }
    else
    {
        _gps_position->hdop = static_cast<float>(hdop);
        _gps_position->vdop = static_cast<float>(vdop);
        _DOP_received = true;

        if(sys_id == GNSS_SYSTEM_ID_GPS)
        {
            memcpy(_gps_used_svid, sat_id, sat_count);
        }
        else if(sys_id == GNSS_SYSTEM_ID_GLONASS)
        {
            memcpy(_gln_used_svid, sat_id, sat_count);
        }
        else if(sys_id == GNSS_SYSTEM_ID_GALILEO)
        {
            memcpy(_gal_used_svid, sat_id, sat_count);
        }
        else if(sys_id == GNSS_SYSTEM_ID_BDS)
        {
            memcpy(_bds_used_svid, sat_id, sat_count);
        }
        else if(sys_id == GNSS_SYSTEM_ID_QZSS)
        {
            memcpy(_qzss_used_svid, sat_id, sat_count);
        }
        else
        {
        }
    }

    return 1;
}

int GPSDriverQuectel::decode_msg_gsv(char* bufptr)
{
    /*
        The GSV message string identifies the number of SVs in view, the PRN numbers, elevations, azimuths, and SNR values. An example of the GSV message string is:

        $GPGSV,4,1,13,02,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67

        GSV message fields
        Field   Meaning
        0   Message ID $GPGSV
        1   Total number of messages of this type in this cycle
        2   Message number
        3   Total number of SVs visible
        4   SV PRN number
        5   Elevation, in degrees, 90 maximum
        6   Azimuth, degrees from True North, 000 through 359
        7   SNR, 00 through 99 dB (null when not tracking)
        8-11    Information about second SV, same format as fields 4 through 7
        12-15   Information about third SV, same format as fields 4 through 7
        16-19   Information about fourth SV, same format as fields 4 through 7
        20  The checksum data, always begins with *
        */
    char* endp;
    uint8_t all_page_num = 0, this_page_num = 0, tot_sv_visible = 0;
    satellite_info_s* satellite_info = nullptr;
    uint8_t* used_svid = nullptr;
    uint8_t* sat_num_gsv = nullptr;
    uint8_t signal_id = 0;
    char* lastComma = nullptr;
    uint8_t init = 0;

    struct gsv_sat
    {
        uint8_t svid;
        uint8_t elevation;
        uint8_t azimuth;
        uint8_t snr;
    } sat[4]{};

    if(bufptr && *(++bufptr) != ',')
    {
        all_page_num = (uint8_t)strtoul(bufptr, &endp, 10);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        this_page_num = (uint8_t)strtoul(bufptr, &endp, 10);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        tot_sv_visible = (uint8_t)strtoul(bufptr, &endp, 10);
        bufptr = endp;
    }

    if((this_page_num < 1) || (this_page_num > all_page_num))
    {
        return 0;
    }
    lastComma = strrchr((char*)_rx_buffer, ',');
    if(lastComma && *(++lastComma) != ',')
    {
        signal_id = (uint8_t)strtoul(lastComma, &endp, 10);
    }

    if(memcmp(_rx_buffer, "$GP", 3) == 0)
    {
        sat_num_gsv = &_sat_num_gpgsv;
        satellite_info = &_gps_satellite_info;
        used_svid = _gps_used_svid;
    }
    else if(memcmp(_rx_buffer, "$GL", 3) == 0)
    {
        sat_num_gsv = &_sat_num_glgsv;
        satellite_info = &_gln_satellite_info;
        used_svid = _gln_used_svid;
    }
    else if(memcmp(_rx_buffer, "$GA", 3) == 0)
    {
        sat_num_gsv = &_sat_num_gagsv;
        satellite_info = &_gal_satellite_info;
        used_svid = _gal_used_svid;
    }
    else if(memcmp(_rx_buffer, "$GB", 3) == 0)
    {
        sat_num_gsv = &_sat_num_gbgsv;
        satellite_info = &_bds_satellite_info;
        used_svid = _bds_used_svid;
    }
    else if(memcmp(_rx_buffer, "$GQ", 3) == 0)
    {
        sat_num_gsv = &_sat_num_gbgsv;
        satellite_info = &_bds_satellite_info;
        used_svid = _bds_used_svid;
    }
    else
    {
        return 0;
    }

    if((this_page_num == 1) && (signal_id == 1) && satellite_info)
    {
        memset(satellite_info->svid, 0, sizeof(satellite_info->svid));
        memset(satellite_info->used, 0, sizeof(satellite_info->used));
        memset(satellite_info->snr, 0, sizeof(satellite_info->snr));
        memset(satellite_info->elevation, 0, sizeof(satellite_info->elevation));
        memset(satellite_info->azimuth, 0, sizeof(satellite_info->azimuth));
    }

    int end = 4;

    if(this_page_num == all_page_num)
    {
        end = tot_sv_visible - (this_page_num - 1) * 4;

        _SVNUM_received = true;
        _SVINFO_received = true;

        *sat_num_gsv += tot_sv_visible;

        if(satellite_info)
        {
            satellite_info->count = satellite_info_s::SAT_INFO_MAX_SATELLITES;
            satellite_info->timestamp = gps_absolute_time();
        }
    }
    if((this_page_num == 1) && (signal_id != 1))
    {
        for(int y = 0; y < satellite_info_s::SAT_INFO_MAX_SATELLITES; y++)
        {
            if(satellite_info->svid[y] == 0)
            {
                break;
            }
            init++;
        }
    }

    if(satellite_info)
    {
        int offset = 0;

        for(int y = 0; y < end; y++)
        {
            if(bufptr && *(++bufptr) != ',')
            {
                sat[y].svid = (uint8_t)strtoul(bufptr, &endp, 10);
                bufptr = endp;
            }

            if(bufptr && *(++bufptr) != ',')
            {
                sat[y].elevation = (uint8_t)strtoul(bufptr, &endp, 10);
                bufptr = endp;
            }

            if(bufptr && *(++bufptr) != ',')
            {
                sat[y].azimuth = (uint8_t)strtoul(bufptr, &endp, 10);
                bufptr = endp;
            }

            if(bufptr && *(++bufptr) != ',')
            {
                sat[y].snr = (uint8_t)strtoul(bufptr, &endp, 10);
                bufptr = endp;
            }

            offset = init + (y + (this_page_num - 1) * 4);
            if(offset > satellite_info_s::SAT_INFO_MAX_SATELLITES)
            {
                break;
            }

            satellite_info->svid[offset] = sat[y].svid;
            satellite_info->snr[offset] = sat[y].snr;
            satellite_info->elevation[offset] = sat[y].elevation;
            satellite_info->azimuth[offset] = sat[y].azimuth;

            if(is_used_svid(sat[y].svid, used_svid))
            {
                satellite_info->used[offset] = 1;
            }
            else
            {
                satellite_info->used[offset] = 0;
            }
            satellite_info->signal[offset] = signal_id;
        }
    }

    return 1;
}

int GPSDriverQuectel::decode_msg_vtg(char* bufptr)
{
    /*$GNVTG,,T,,M,0.683,N,1.265,K*30
      $GNVTG,,T,,M,0.780,N,1.445,K*33

    Field	Meaning
    0	Message ID $GPVTG
    1	Track made good (degrees true)
    2	T: track made good is relative to true north
    3	Track made good (degrees magnetic)
    4	M: track made good is relative to magnetic north
    5	Speed, in knots
    6	N: speed is measured in knots
    7	Speed over ground in kilometers/hour (kph)
    8	K: speed over ground is measured in kph
    9	The checksum data, always begins with *
    */
    char* endp;
    float track_true = 0.f;
    char T;
    float track_mag = 0.f;
    char M;
    float ground_speed = 0.f;
    char N;
    float ground_speed_K = 0.f;
    char K;

    if(bufptr && *(++bufptr) != ',')
    {
        track_true = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        T = *(bufptr++);
    }

    if(bufptr && *(++bufptr) != ',')
    {
        track_mag = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        M = *(bufptr++);
    }

    if(bufptr && *(++bufptr) != ',')
    {
        ground_speed = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        N = *(bufptr++);
    }

    if(bufptr && *(++bufptr) != ',')
    {
        ground_speed_K = strtof(bufptr, &endp);
        bufptr = endp;
    }

    if(bufptr && *(++bufptr) != ',')
    {
        K = *(bufptr++);
    }

    float track_rad = track_true * M_PI_F / 180.0f;  // rad in range [0, 2pi]

    if(track_rad > M_PI_F)
    {
        track_rad -= 2.f * M_PI_F;  // rad in range [-pi, pi]
    }

    float velocity_ms = ground_speed / 1.9438445f;
    float velocity_north = velocity_ms * cosf(track_rad);
    float velocity_east = velocity_ms * sinf(track_rad);

    _gps_position->vel_m_s = velocity_ms;
    _gps_position->vel_n_m_s = velocity_north;
    _gps_position->vel_e_m_s = velocity_east;
    _gps_position->cog_rad = track_rad;
    _gps_position->vel_ned_valid = true; /** Flag to indicate if NED speed is valid */
    _gps_position->c_variance_rad = 0.1f;
    _gps_position->s_variance_m_s = 0;

    if(!_VEL_received)
    {
        _VEL_received = true;
    }

    if(_sat_num_gga > 0)
    {
        _gps_position->satellites_used = _sat_num_gga;
    }
    else if(_SVNUM_received && _SVINFO_received && _FIX_received)
    {
        _sat_num_gsv = (uint8_t)(_sat_num_gpgsv + _sat_num_glgsv + _sat_num_gagsv + _sat_num_gbgsv + _sat_num_gqgsv);
        _gps_position->satellites_used = MAX(_sat_num_gns, _sat_num_gsv);
    }

    if(_VEL_received && _POS_received)
    {
        _gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
        _rate_count_vel++;
        _rate_count_lat_lon++;
    }

    return 1;
}

int GPSDriverQuectel::decode_msg_pqtmvel(char* bufptr)
{
    char* endp;
    uint8_t version = 0;

    QL_DEBUG("PARSE PQTMVEL: %s", bufptr);

    if(bufptr && *(++bufptr) != ',')
    {
        version = (uint8_t)strtoul(bufptr, &endp, 10);
        bufptr = endp;
    }

    if(version == 1)
    {
        /**
         *$PQTMVEL,1,051648.000,0.003,-0.006,-0.001,0.007,0.007,299.249,0.075,0.076,359.999*68
         *
         *Field	Meaning
         *0	Message ID $PQTMVEL
         *1	version
         *2	UTC time
         *3	North velocity (m/s)
         *4	East velocity (m/s)
         *5	Down velocity (m/s)
         *6	2D speed (m/s)
         *7	3D speed (m/s)
         *8	Heading (Degree)
         *9	Estimate of 2D speed accuracy (m/s)
         *10  Estimate of 3D speed accuracy (m/s)
         *11  Estimate of heading accuracy (Degree)
         */

        float utc_time;
        float vel_n = 0, vel_e = 0, vel_d = 0;
        float grd_spd = 0, spd = 0;
        float heading = 0;
        float grd_spd_acc = 0, spd_acc = 0;
        float heading_acc = 0;

        if(bufptr && *(++bufptr) != ',')
        {
            utc_time = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            vel_n = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            vel_e = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            vel_d = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            grd_spd = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            spd = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            heading = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            grd_spd_acc = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            spd_acc = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            heading_acc = strtof(bufptr, &endp);
            bufptr = endp;
        }

        _gps_position->vel_n_m_s = vel_n;
        _gps_position->vel_e_m_s = vel_e;
        _gps_position->vel_d_m_s = vel_d;
        _gps_position->heading = heading;
        _gps_position->heading_accuracy = heading_acc;
        _gps_position->timestamp = gps_absolute_time();

        _msg_pqtmvel.timestamp = gps_absolute_time();
        _msg_pqtmvel.time_format = utc_time;
        _msg_pqtmvel.vel_n = vel_n;
        _msg_pqtmvel.vel_e = vel_e;
        _msg_pqtmvel.vel_d = vel_d;
        _msg_pqtmvel.grd_spd = grd_spd;
        _msg_pqtmvel.spd = spd;
        _msg_pqtmvel.heading = heading;
        _msg_pqtmvel.grd_spd_acc = grd_spd_acc;
        _msg_pqtmvel.spd_acc = spd_acc;
        _msg_pqtmvel.heading_acc = heading_acc;

        return 1;
    }
    else
    {
    }

    return 0;
}

int GPSDriverQuectel::decode_msg_pqtmepe(char* bufptr)
{
    char* endp;
    uint8_t version = 0;
    float epe_north = 0, epe_east = 0, epe_down = 0;
    float epe_2d = 0, epe_3d = 0;

    QL_DEBUG("PARSE PQTMEPE: %s", bufptr);
    if(bufptr && *(++bufptr) != ',')
    {
        version = (uint8_t)strtoul(bufptr, &endp, 10);
        bufptr = endp;
    }

    if(version == 2)
    {
        /**
         $PQTMEPE,2,1.6247,1.6247,4.0980,2.2977,4.6982*69

         Field	Meaning
         0	Message ID $PQTMEPE
         1	version
         2	Estimated north error.
         3	Estimated east error.
         4	Estimated down error.
         5	Estimated 2D position error.
         6	Estimated 3D position error.
         */

        if(bufptr && *(++bufptr) != ',')
        {
            epe_north = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            epe_east = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            epe_down = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            epe_2d = strtof(bufptr, &endp);
            bufptr = endp;
        }

        if(bufptr && *(++bufptr) != ',')
        {
            epe_3d = strtof(bufptr, &endp);
            bufptr = endp;
        }

        _gps_position->eph = epe_2d;
        _gps_position->epv = epe_down;
        _gps_position->timestamp = gps_absolute_time();
        _msg_pqtmepe.timestamp = gps_absolute_time();

        // estimated position error
        _msg_pqtmepe.epe_north = epe_north;
        _msg_pqtmepe.epe_east = epe_east;
        _msg_pqtmepe.epe_down = epe_down;
        _msg_pqtmepe.epe_2d = epe_2d;
        _msg_pqtmepe.epe_3d = epe_3d;

        return 1;
    }
    else
    {
    }

    return 0;  // unsupported version
}

int GPSDriverQuectel::decode_msg_pqtmsvinstatus(char* bufptr)
{
    char* endp;
    uint8_t version = 0;
    uint32_t tow  = 0;
    uint8_t res0, res1;
    uint8_t valid;
    uint32_t num_obs = 0;
    uint32_t cfg_dur = 0;
    double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
    double acc = 0.0f;
    SurveyInStatus status;

    QL_DEBUG("PARSE PQTMSVINSTATUS: %s", bufptr);
    if(bufptr && *(++bufptr) != ',')
    {
        version = (uint8_t)strtoul(bufptr, &endp, 10);
        bufptr = endp;
    }

    if(version == 1)
    {
        /**
         * $PQTMSVINSTATUS,1,1000,1,,01,20,100,-2484434.3645,4875976.9741,3266161.3412,1.2415*3C
         * Field	Meaning
         * 0	Message ID $PQTMSVINSTATUS
         * 1	version
         * 2	GPS time of week. (ms)
         * 3	Survey-in position validity flag. 0 = invalid, 1 = In-progress, 2 = valid
         * 4	Reserved.always empty
         * 5	Reserved. 
         * 6	Number of position observations used during Survey-in
         * 7	Same as <CFG_CNT> field (minimum positioning times in Survey-in mode) configured via PQTMCFGSVIN command
         * 8	Current Survey-in mean position along X axis of GNSS Module Series Description ECEF coordinate system.
         * 9	Current Survey-in mean position along Y axis of GNSS Module Series Description ECEF coordinate system.
         * 10	Current Survey-in mean position along Z axis of GNSS Module Series Description ECEF coordinate system.
         * 11	Current Survey-in mean position accuracy
         */

        if(bufptr && *(++bufptr) != ',')
        {
            tow = strtoul(bufptr, &endp, 10);
            bufptr = endp;
        }
        if(bufptr && *(++bufptr) != ',')
        {
            valid = (uint8_t)strtoul(bufptr, &endp, 10);
            bufptr = endp;
        }
        if(bufptr && *(++bufptr) != ',')
        {
            res0 = (uint8_t)strtoul(bufptr, &endp, 10);
            bufptr = endp;
        }
        if(bufptr && *(++bufptr) != ',')
        {
            res1 = (uint8_t)strtoul(bufptr, &endp, 10);
            bufptr = endp;
        }
        if(bufptr && *(++bufptr) != ',')
        {
            num_obs = strtoul(bufptr, &endp, 10);
            bufptr = endp;
        }
        if(bufptr && *(++bufptr) != ',')
        {
            cfg_dur = strtoul(bufptr, &endp, 10);
            bufptr = endp;
        }
        if(bufptr && *(++bufptr) != ',')
        {
            mean_x = strtod(bufptr, &endp);
            bufptr = endp;
        }
        if(bufptr && *(++bufptr) != ',')
        {
            mean_y = strtod(bufptr, &endp);
            bufptr = endp;
        }
        if(bufptr && *(++bufptr) != ',')
        {
            mean_z = strtod(bufptr, &endp);
            bufptr = endp;
        }
        if(bufptr && *(++bufptr) != ',')
        {
            acc = strtod(bufptr, &endp);
            bufptr = endp;
        }

        status.flags = valid ==2 ? 1 : 0;  // 1 = valid, 0 = invalid
        status.duration = num_obs;
        status.mean_accuracy = acc <= 0 ? 0 : (uint32_t)(acc * 1000);  // convert m to mm

        if(status.flags == 1)
        {
            double mean_lat = 0.0, mean_lon = 0.0, mean_alt = 0.0;
            ecef_to_wgs84(mean_x, mean_y, mean_z, mean_lat, mean_lon, mean_alt);
            status.latitude = mean_lat;
            status.longitude = mean_lon;
            status.altitude = (float)mean_alt;
        }
        else
        {
            status.latitude = NAN;
            status.longitude = NAN;
            status.altitude = NAN;   
        }

        surveyInStatus(status);

        return 1;
    }

    return 0;
}

int GPSDriverQuectel::encode_cmd(uint8_t* buf, uint16_t maxBufLength, const char* fmt, ...)
{
    int len = 0;
    va_list args;
    va_start(args, fmt);

    buf[0] = '$';  // start with '$'
    len += 1;      // increment length for '$'
    len += vsnprintf((char*)(buf + 1), maxBufLength - 1, fmt, args);
    va_end(args);

    if(len <= 1 || len >= maxBufLength)
    {
        QL_ERR("Command encoding failed: buffer overflow");
        return 0;
    }

    uint8_t checksum = checkXOR(buf + 1, len - 1);  // skip the '$' character
    snprintf((char*)buf + len, maxBufLength - len, "*%02X\r\n", checksum);

    return (int)strlen((char*)buf);
}

bool GPSDriverQuectel::query_module_verion(uint8_t* version)
{
    uint8_t cmd[64]{0};
    int cmd_len = 0;

    if(version == nullptr)
    {
        QL_ERR("Version buffer is null");
        return false;
    }

    cmd_len = encode_cmd(cmd, sizeof(cmd), "PQTMVERNO");

    write(cmd, cmd_len);

    if(wait_message((const uint8_t*)"PQTMVERNO", COMMNAD_TIMEOUT_MS) == 0)
    {
        QL_ERR("Failed to get PQTMVERNO message");
        return false;
    }

    memcpy(version, _rx_buffer + strlen("$PQTMVERNO,"), _rx_count - 3 - strlen("$PQTMVERNO,"));  // skip the '$' and '*xx' at the end

    return true;
}

bool GPSDriverQuectel::get_work_mode(ql_work_mode_t* work_mode)
{
    uint8_t cmd[64]{0};
    int cmd_len = 0;

    if(work_mode == nullptr)
    {
        QL_ERR("Work mode buffer is null");
        return false;
    }

    cmd_len = encode_cmd(cmd, sizeof(cmd), "PQTMCFGRCVRMODE,R");

    write(cmd, cmd_len);

    if(wait_message((const uint8_t*)"PQTMCFGRCVRMODE,OK", COMMNAD_TIMEOUT_MS) == 0)
    {
        QL_ERR("Failed to repsond to PQTMCFGRCVRMODE command");
        return false;
    }

    if(sscanf((const char*)_rx_buffer, "$PQTMCFGRCVRMODE,OK,%d", (int*)&_work_mode) != 1)
    {
        QL_ERR("Failed to parse PQTMCFGRCVRMODE response");
        return false;
    }
    *work_mode = _work_mode;

    return true;
}

bool GPSDriverQuectel::set_work_mode(ql_work_mode_t work_mode)
{
    uint8_t cmd[64]{0};
    int cmd_len = 0;

    if((work_mode != QL_WORK_MODE_ROVER) && (work_mode != QL_WORK_MODE_BASE_STATION))
    {
        QL_ERR("Invalid work mode: %d", work_mode);
        return false;
    }

    cmd_len = encode_cmd(cmd, sizeof(cmd), "PQTMCFGRCVRMODE,W,%d", work_mode);

    write(cmd, cmd_len);

    if(wait_message((const uint8_t*)"PQTMCFGRCVRMODE,OK", COMMNAD_TIMEOUT_MS) == 0)
    {
        QL_ERR("Failed to repsond to PQTMCFGRCVRMODE command");
        return false;
    }

    _work_mode = work_mode;

    return true;
}

bool GPSDriverQuectel::set_cmd_msgrate(uint8_t* msg_name, uint8_t msg_rate, uint8_t msg_ver, uint16_t timeout_ms)
{
    uint8_t cmd[64]{0};
    int cmd_len = 0;

    cmd_len = encode_cmd(cmd, sizeof(cmd), "PQTMCFGMSGRATE,W,%s,%d,%d", msg_name, msg_rate, msg_ver);
    write(cmd, cmd_len);

    if(wait_message((const uint8_t*)"PQTMCFGMSGRATE,OK", timeout_ms) == 0)
    {
        return false;
    }

    QL_INFO("GET %s SUCCESS, DATA: %.*s", msg_name, _rx_count, _rx_buffer);

    return true;
}

bool GPSDriverQuectel::set_save_config_to_nvm()
{
    uint8_t cmd[64]{0};
    int cmd_len = 0;
    int timeout_ms = 1000;  // default timeout

    cmd_len = encode_cmd(cmd, sizeof(cmd), "PQTMSAVEPAR");

    write(cmd, cmd_len);

    if(wait_message((const uint8_t*)"PQTMSAVEPAR,OK", timeout_ms) == 0)
    {
        QL_ERR("Failed to save configuration to NVM");
        return false;
    }

    QL_INFO("Configuration saved to NVM successfully");

    return true;
}

bool GPSDriverQuectel::set_restore_config_to_nvm()
{
    uint8_t cmd[64]{0};
    int cmd_len = 0;
    int timeout_ms = 1000;  // default timeout

    cmd_len = encode_cmd(cmd, sizeof(cmd), "PQTMRESTOREPAR");

    write(cmd, cmd_len);

    if(wait_message((const uint8_t*)"PQTMRESTOREPAR,OK", timeout_ms) == 0)
    {
        QL_ERR("Failed to restore configuration from NVM");
        return false;
    }

    QL_INFO("Configuration retore to NVM successfully");

    return true;
}

bool GPSDriverQuectel::set_reset_module()
{
    uint8_t cmd[64]{0};
    int cmd_len = 0;
    int timeout_ms = 5000;  // default timeout

    cmd_len = encode_cmd(cmd, sizeof(cmd), "PQTMSRR");

    write(cmd, cmd_len);

    if(wait_message((const uint8_t*)"PQTMVER", timeout_ms) == 0)
    {
        QL_ERR("Failed to reset module");
        return false;
    }

    // wait for module to load completely
    delayMilliseconds(200);

    QL_INFO("Module reset successfully");

    return true;
}

bool GPSDriverQuectel::setSurveyInSpecs(uint32_t survey_in_acc_limit, uint32_t survey_in_min_dur)
{
    uint8_t cmd[128]{0};
    int cmd_len = 0;
    int timeout_ms = 1000;  // default timeout
    SurveyInSettings survey_in;

    cmd_len = encode_cmd(cmd, sizeof(cmd), "PQTMCFGSVIN,W,1,%d,%d,0,0,0", survey_in_acc_limit, survey_in_min_dur);

    write(cmd, cmd_len);

    if(wait_message((const uint8_t*)"PQTMCFGSVIN,OK", timeout_ms) == 0)
    {
        QL_ERR("Failed to set survey-in specs");
        return false;
    }

    GPSBaseStationSupport::setSurveyInSpecs(survey_in_acc_limit, survey_in_min_dur);
    survey_in = GPSBaseStationSupport::_base_settings.settings.survey_in;

    QL_INFO("Setting survey-in specs: acc_limit=%d, min_dur=%d", survey_in.acc_limit, survey_in.min_dur);

    QL_INFO("Set survey-in specs OK");
}

bool GPSDriverQuectel::setBasePosition(double latitude, double longitude, float altitude, float position_accuracy)
{
    uint8_t cmd[128]{0};
    int cmd_len = 0;
    double ecef_x, ecef_y, ecef_z;
    FixedPositionSettings fixed_position;

    wgs84_to_ecef(latitude, longitude, altitude, ecef_x, ecef_y, ecef_z);

    QL_INFO("ECEF: x=%.2f, y=%.2f, z=%.2f", ecef_x, ecef_y, ecef_z);

    cmd_len = encode_cmd(cmd, sizeof(cmd), "PQTMCFGSVIN,W,2,0,%.4f,%.4f,%.4f,%.4f", position_accuracy, ecef_x, ecef_y, ecef_z);

    write(cmd, cmd_len);

    if(wait_message((const uint8_t*)"PQTMCFGSVIN,OK", COMMNAD_TIMEOUT_MS) == 0)
    {
        QL_ERR("Failed to set survey-in specs");
        return false;
    }

    QL_INFO("Set survey-in specs OK");

    GPSBaseStationSupport::setBasePosition(latitude, longitude, altitude, position_accuracy);
    fixed_position = _base_settings.settings.fixed_position;
    QL_INFO("Setting base position: lat=%.6f, lon=%.6f, alt=%.2f, acc=%.4f",
            fixed_position.latitude, fixed_position.longitude, fixed_position.altitude, fixed_position.position_accuracy);
}

bool GPSDriverQuectel::setOutputSurveyInStatus(bool enable)
{

    uint8_t msgrate = enable ? 1 : 0;

    if(set_cmd_msgrate((uint8_t*)"PQTMSVINSTATUS", msgrate, 1, COMMNAD_TIMEOUT_MS) != true)
    {
        QL_ERR("Failed to set message rate for PQTMSVINSTATUS");
        return false;
    }

    QL_INFO("Set output survey-in status: %s", enable ? "enabled" : "disabled");

    return true;
}

void GPSDriverQuectel::wgs84_to_ecef(double lat, double lon, double alt, double& x, double& y, double& z)
{
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double e_sq = f * (2.0 - f);
    const double pi = 3.14159265358979323846;

    double lat_rad = lat * pi / 180.0;
    double lon_rad = lon * pi / 180.0;

    double N = a / sqrt(1 - e_sq * sin(lat_rad) * sin(lat_rad));

    // Calculate ECEF coordinates
    x = (N + alt) * cos(lat_rad) * cos(lon_rad);
    y = (N + alt) * cos(lat_rad) * sin(lon_rad);
    z = (N * (1 - e_sq) + alt) * sin(lat_rad);
}

void GPSDriverQuectel::ecef_to_wgs84(double x, double y, double z, double& lat, double& lon, double& alt)
{
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double e_sq = f * (2 - f);
    const double pi = 3.14159265358979323846;

    lon = atan2(y, x);

    double p = sqrt(x * x + y * y);
    double theta = atan2(z * a, p * (1 - f) * a);
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    // Iterative computation for latitude
    lat = atan2(z + e_sq * (1 - f) * a * pow(sin_theta, 3),
                p - e_sq * a * pow(cos_theta, 3));

    double N = a / sqrt(1 - e_sq * sin(lat) * sin(lat));
    alt = p / cos(lat) - N;

    // Iteratively improve latitude and altitude
    double prev_lat = 0.0;
    int iter = 0;
    while (fabs(lat - prev_lat) > 1e-11 && iter < 10) {
        prev_lat = lat;
        N = a / sqrt(1 - e_sq * sin(lat) * sin(lat));
        alt = p / cos(lat) - N;
        lat = atan2(z, p * (1 - e_sq * N / (N + alt)));
        iter++;
    }

    lat = lat * 180.0 / pi;
    lon = lon * 180.0 / pi;
}