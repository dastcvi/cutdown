/* Author: Alex St. Clair
 * Filename: Cutdown_GPS.h
 * Created: 12-17-18
 * 
 * Defines a driver to interface with the uBlox MAX-M8 GPS
 * 
 * This driver implements a polling interface using the uBlox UBX protocol in lieu of
 * reading NMEA strings output at a standard interval. This minimizes bus traffic (only
 * one binary UBX message is necessary) and removes the need to asynchronously buffer
 * NMEA strings for parsing.
 */

#ifndef CUTDOWN_GPS_H
#define CUTDOWN_GPS_H

#include "Cutdown_GPS_Messages.h"
#include "Cutdown_Pinout.h"
#include "Arduino.h"
#include "SERCOM.h"
#include "wiring_private.h"
#include <stdint.h>

#define GPS_MSG_TIMEOUT	    2000 // in ms
#define SYNC1               0xB5
#define SYNC2               0x62

#define R_EARTH (6371.0)
#define TO_RAD  (3.1415926536 / 180.0)

typedef struct {
    float longitude; // degrees
    float latitude;  // degrees
    float height;    // kilometers
    uint8_t num_satellites;
    GPS_FIX_TYPE_t fix_type;
} GPS_Data_t;

class Cutdown_GPS {
public:
    Cutdown_GPS() { };
    ~Cutdown_GPS() { };

    void init(void);

    // turns off the default NMEA outputs
    bool stop_nmea(void);

    // attempts to update the GPS fix (will only update lat/long/height if 3D fix,
    // but will still update num_satellites regardless)
    GPS_FIX_TYPE_t update_fix(void);

    // use the Haversine formula to calculate the current distance from the given coords
    float distance_from(double lat_origin, double long_origin);

    GPS_Data_t gps_data = {0};
private:
    void transmit_ubx(uint8_t * buffer, uint16_t length);
    bool verify_ack(void);

    bool verify_checksum(unsigned long start_time, uint8_t * buffer, uint16_t length);

    void calculate_checksum(uint8_t * buffer, uint16_t length, uint8_t * cka, uint8_t * ckb);

    bool get_check_bytes(unsigned long start_time, uint8_t * cka, uint8_t * ckb);
    bool get_sync_bytes(unsigned long start_time);
};

#endif