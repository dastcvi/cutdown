/* Author: Alex St. Clair
 * Filename: Cutdown_Configure.h
 * Created: 11-30-18
 * 
 * Defines a driver for setting and maintaining configurations
 */

#ifndef CUTDOWN_CONFIGURE_H
#define CUTDOWN_CONFIGURE_H

#define Serial SERIAL_PORT_USBVIRTUAL

#define DEFAULT_TIMER	        20  // s (2 hours)
#define DEFAULT_BACKUP_TIMER    7500  // s (2 hours 5 minutes)
#define DEFAULT_HEIGHT          30.0  // km
#define DEFAULT_DISTANCE        100.0 // km
#define DEFAULT_CRITICAL_VOLT   10.8  // V
#define DEFAULT_LOW_VOLT        11.1  // V
#define SPSC_LATITUDE           40.011
#define SPSC_LONGITUDE          -105.246

#include <stdint.h>

// commands (all of form "cmd,value\n" in ASCII)
#define PRIMARY_TIMER    "TPRI"
#define BACKUP_TIMER     "TBCK"
#define TRIGGER_HEIGHT   "HEIGHT"
#define TRIGGER_DISTANCE "DIST"
#define CRITICAL_VOLT    "VCRIT"
#define LOW_VOLT         "VLOW"

typedef struct {
    uint16_t primary_timer;
    uint16_t backup_timer;
    float trigger_height;
    float trigger_distance;
    float critical_batt_voltage;
    float low_batt_voltage;
    float origin_lat;
    float origin_long;
} Cutdown_Configuration_t;

extern Cutdown_Configuration_t cutdown_config;

void config_init(void);

void config_update(void);

#endif