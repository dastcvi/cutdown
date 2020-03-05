/* Author: Alex St. Clair
 * Filename: Cutdown_Logger.h
 * Created: 1-9-19
 *
 * Defines functions to log debug info over the future radio
 * interface to a separate logging device.
 */

#ifndef CUTDOWN_LOGGER_H
#define CUTDOWN_LOGGER_H

#include "Cutdown_Pinout.h"
#include "Arduino.h"
#include "SERCOM.h"
#include "wiring_private.h"
#include <stdint.h>

// choose the lowest level to log to USB (select LOG_NONE to mute entirely)
#define USB_LOGGER_LEVEL    LOG_INFO
#define SERIAL_LOGGER_LEVEL LOG_DEBUG

// ascending order of importance
typedef enum : uint8_t {
    LOG_DEBUG,
    LOG_INFO,
    LOG_ERROR,
    LOG_NONE
} Logger_Level_t;

void logger_init(void);

void cutdown_log(Logger_Level_t level, char * message);
void cutdown_log(Logger_Level_t level, char * message, float f1);
void cutdown_log(Logger_Level_t level, char * m1, float f1, char * m2, float f2);
void cutdown_log(Logger_Level_t level, char * message, int32_t i1);
void cutdown_log(Logger_Level_t level, char * message, uint32_t i1);

#endif