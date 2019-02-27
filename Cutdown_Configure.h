/* Author: Alex St. Clair
 * Filename: Cutdown_Configure.h
 * Created: 11-30-18
 * 
 * Defines a driver for setting and maintaining configurations
 */

#ifndef CUTDOWN_CONFIGURE_H
#define CUTDOWN_CONFIGURE_H

#define Serial SERIAL_PORT_USBVIRTUAL

#define DEFAULT_SYSTEM_MODE     MODE_CUTAWAY

#if (DEFAULT_SYSTEM_MODE == MODE_CUTAWAY)
    #define DEFAULT_TIMER           18000 // s (5 hours)
    #define DEFAULT_BACKUP_TIMER    18300 // s (5 hours 5 minutes)
#elif (DEFAULT_SYSTEM_MODE == MODE_CUTDOWN)
    #define DEFAULT_TIMER           7200  // s (2 hours)
    #define DEFAULT_BACKUP_TIMER    7500  // s (2 hours 5 minutes)
#endif

#define DEFAULT_HEIGHT          30.0  // km
#define DEFAULT_DISTANCE        100.0 // km
#define DEFAULT_CRITICAL_VOLT   10.8  // V
#define DEFAULT_LOW_VOLT        11.1  // V
#define DEFAULT_CEILING         700   // hPA ~= 10k feet ~= 3 km

#define SPSC_LATITUDE           40.011
#define SPSC_LONGITUDE          -105.246
#define LARAMIE_LATITUDE        41.312907
#define LARAMIE_LONGITUDE       -105.660194

#include <stdint.h>

// commands (all of form "cmd,value\n" in ASCII)
#define PRIMARY_TIMER    "TPRI"
#define BACKUP_TIMER     "TBCK"
#define TRIGGER_HEIGHT   "HEIGHT"
#define TRIGGER_DISTANCE "DIST"
#define CRITICAL_VOLT    "VCRIT"
#define LOW_VOLT         "VLOW"
#define SYSTEM_MODE      "MODE"
#define CUTAWAY_CEILING  "CEIL"

typedef enum : uint8_t {
    MODE_CUTDOWN = 0,
    MODE_CUTAWAY = 1
} Cutdown_Mode_t;

typedef struct {
    uint16_t primary_timer;
    uint16_t backup_timer;
    float trigger_height;
    float trigger_distance;
    float critical_batt_voltage;
    float low_batt_voltage;
    float origin_lat;
    float origin_long;
    float cutaway_ceiling;  // minimum pressure for cutaway
    Cutdown_Mode_t system_mode;
} Cutdown_Configuration_t;

extern Cutdown_Configuration_t cutdown_config;
extern bool update_backup_timer;

void config_init(void);

void config_update(void);

#endif