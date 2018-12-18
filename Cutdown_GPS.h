/* Author: Alex St. Clair
 * Filename: Cutdown_GPS.h
 * Created: 12-17-18
 * 
 * Defines a driver to interface with the uBlox MAX-M8 GPS
 */

#ifndef CUTDOWN_GPS_H
#define CUTDOWN_GPS_H

#include "Cutdown_GPS_Messages.h"
#include "Cutdown_Pinout.h"
#include "Arduino.h"
#include "SERCOM.h"
#include "wiring_private.h"
#include <stdint.h>

#define ACK_TIMEOUT	        1000 // in ms
#define SYNC1               0xB5
#define SYNC2               0x62

typedef struct {
    float longitude;
    float latitude;
    float height;
    uint8_t num_satellites;
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

    bool set_reference(void);
    float get_distance(void);

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