/* Author: Alex St. Clair
 * Filename: Cutdown_Configure.h
 * Created: 11-30-18
 * 
 * Defines a driver for setting and maintaining configurations
 */

#ifndef CUTDOWN_CONFIGURE_H
#define CUTDOWN_CONFIGURE_H

#define Serial SERIAL_PORT_USBVIRTUAL

#define DEFAULT_TIMER	        15
#define DEFAULT_BACKUP_TIMER    20
#define DEFAULT_CRITICAL_VOLT   10.8
#define DEFAULT_LOW_VOLT        11.1

#include <stdint.h>

// commands (all of form "cmd,value\n" in ASCII)
#define PRIMARY_TIMER    "TPRI"
#define BACKUP_TIMER     "TBCK"
#define CRITICAL_VOLT    "VCRIT"
#define LOW_VOLT         "VLOW"

typedef struct {
    uint16_t primary_timer;
    uint16_t backup_timer;
    float critical_batt_voltage;
    float low_batt_voltage;
} Cutdown_Configuration_t;

extern Cutdown_Configuration_t cutdown_config;

void config_init(void);

void config_update(void);

#endif