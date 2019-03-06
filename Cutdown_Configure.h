/* Author: Alex St. Clair
 * Filename: Cutdown_Configure.h
 * Created: 11-30-18
 * 
 * Defines a driver for setting and maintaining configurations
 */

#ifndef CUTDOWN_CONFIGURE_H
#define CUTDOWN_CONFIGURE_H

#include <stdint.h>

#define CURRENT_CONFIG_VERSION  0xA5C90002

#define Serial SERIAL_PORT_USBVIRTUAL

// commands (all of form "cmd,value\n" in ASCII)
#define CMD_PRIMARY_TIMER    "TPRI"
#define CMD_BACKUP_TIMER     "TBCK"
#define CMD_TRIGGER_HEIGHT   "HEIGHT"
#define CMD_TRIGGER_DISTANCE "DIST"
#define CMD_CRITICAL_VOLT    "VCRIT"
#define CMD_LOW_VOLT         "VLOW"
#define CMD_SYSTEM_MODE      "MODE"
#define CMD_CUTAWAY_CEILING  "CEIL"
#define CMD_SERIAL_NUMBER    "SN"
#define CMD_LATITUDE         "LAT"
#define CMD_LONGITUDE        "LONG"
#define CMD_TEMP_SETPOINT    "TEMP"
#define CMD_MENU             "MENU"
#define CMD_READ_CONFIGS     "READ"

typedef enum : uint8_t {
    MODE_CUTDOWN = 0,
    MODE_CUTAWAY = 1
} Cutdown_Mode_t;

/* Important notes:
 *  - this struct is mirrored in flash-emulated EEPROM (FEE)
 *    - FEE can be written only 100k times
 *    - the struct should be updated in FEE when changed in unarmed mode
 *    - the struct is loaded from FEE on boot
 *  - the "config version" member should be manually incremented when the definition of the struct is changed
 *    - the full 32-bit value is used to validate the struct to the software version each time it is used
 *    - the 0xA5C9 part is just added verification
 *  - a separate script flashes the initial struct to FEE
 *  - the "primary_timer_remaining" field is updated once per minute to minimize writes
 */
typedef struct {
    // housekeeping
    uint32_t config_version;
    uint16_t serial_number;
    // timers
    uint16_t primary_timer;
    uint16_t primary_timer_remaining;
    uint16_t backup_timer;
    // triggers
    float trigger_height;
    float trigger_distance;
    float cutaway_ceiling;  // minimum pressure for cutaway
    // battery monitoring setpoints
    float low_batt_voltage;
    float critical_batt_voltage;
    // location when armed
    float origin_lat;
    float origin_long;
    // thermal control
    float temp_set_point;
    // mode of operation
    Cutdown_Mode_t system_mode;
    bool ceiling_reached; // non-volatile storage in case cutaway reboots
} Cutdown_Configuration_t;

extern Cutdown_Configuration_t cutdown_config;
extern bool update_backup_timer;

bool load_config_from_fee(void);

void write_config_to_fee(void);

void config_check_serial(void);

void display_menu(void);

void display_fee(void);

#endif