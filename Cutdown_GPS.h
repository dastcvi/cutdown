/* Author: Alex St. Clair
 * Filename: Cutdown_GPS.h
 * Created: 12-17-18
 * 
 * Defines a driver to interface with the uBlox MAX-M8 GPS
 */

#ifndef CUTDOWN_GPS_H
#define CUTDOWN_GPS_H

#include "Cutdown_Pinout.h"
#include "Arduino.h"
#include "SERCOM.h"
#include "wiring_private.h"
#include <stdint.h>

#define ACK_TIMEOUT	        1000 // in ms
#define SYNC1               0xB5
#define SYNC2               0x62

typedef struct {
    int32_t longitude;
    int32_t latitude;
    int32_t height;
} GPS_Data_t;

class Cutdown_GPS {
public:
    Cutdown_GPS() { };
    ~Cutdown_GPS() { };

    void init(void);

    void test(void);

    bool update_fix(void);
    bool set_reference(void);
    float get_distance(void);
private:
    void transmit_ubx(uint8_t * buffer, uint16_t length);
    bool verify_ack(void);

    void calculate_checksum(uint8_t * buffer, uint16_t length, uint8_t * cka, uint8_t * ckb);

    bool get_check_bytes(unsigned long start_time, uint8_t * cka, uint8_t * ckb);
    bool get_sync_bytes(unsigned long start_time);
};

#endif