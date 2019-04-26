/* Author: Alex St. Clair
 * Filename: Cutdown_Configure.h
 * Created: 11-30-18
 * 
 * Defines a driver for setting and maintaining configurations
 */

#ifndef CUTDOWN_CONFIGURE_H
#define CUTDOWN_CONFIGURE_H

#include <stdint.h>

#define CURRENT_CONFIG_VERSION  0xA5C9000A

#define Serial SERIAL_PORT_USBVIRTUAL

// commands (all of form "cmd,value\n" in ASCII)
#define CMD_PRIMARY_TIMER    "TPRI"
#define CMD_BACKUP_TIMER     "TBCK"
#define CMD_TRIGGER_HEIGHT   "HEIGHT"
#define CMD_TRIGGER_DISTANCE "DIST"
#define CMD_CRITICAL_VOLT    "VCRIT"
#define CMD_LOW_VOLT         "VLOW"
#define CMD_FLY_VOLT         "VFLY"
#define CMD_SYSTEM_MODE      "MODE"
#define CMD_CUTAWAY_CEILING  "CEIL"
#define CMD_SERIAL_NUMBER    "SN"
#define CMD_LATITUDE         "LAT"
#define CMD_LONGITUDE        "LONG"
#define CMD_TEMP_SETPOINT    "TEMP"
#define CMD_BURST_RATE       "BURST"
#define CMD_SINK_SAMPLES     "SINK"
#define CMD_SQUIB_MODE       "SQUIBS"
#define CMD_TRIGGER_SET      "TRIG"
#define CMD_MENU             "MENU"
#define CMD_READ_CONFIGS     "READ"

typedef enum : uint8_t {
    MODE_CUTDOWN = 0,
    MODE_CUTAWAY = 1
} Cutdown_Mode_t;

typedef enum : uint8_t {
    ONE_SQUIB = 1,
    TWO_SQUIB = 2
} Squib_Mode_t;

typedef enum : uint8_t {
    TRIG_NONE = 1,
    TRIG_TIMER = 2,
    TRIG_GPSH = 3,
    TRIG_GPSD = 4,
    TRIG_ALT = 5,
    TRIG_BURST = 6,
    TRIG_SINK = 7
} Trigger_t;

/* Important notes:
 *  - this struct is mirrored in flash-emulated EEPROM (FEE)
 *    - FEE can be written only 100k times
 *    - the struct should be updated in FEE when changed in unarmed mode
 *    - the struct is loaded from FEE on boot
 *  - the "config version" member should be manually incremented when the definition of the struct is changed
 *    - the full 32-bit value is used to validate the struct to the software version each time it is used
 *    - the 0xA5C9 part is just added verification
 *    - the number can also be incremented arbitrarily to enforce a recommissioning of the board upon flashing
 *    - the number should never be decremented
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
    float min_fly_voltage;
    float low_batt_voltage;
    float critical_batt_voltage;
    // location when armed
    float origin_lat;
    float origin_long;
    // thermal control
    float temp_set_point;
    // burst/floater detection
    float burst_fall_rate;
    uint16_t sink_num_samples;
    // mode of operation
    Cutdown_Mode_t system_mode;
    Squib_Mode_t squib_mode;
    Trigger_t trigger_type;
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