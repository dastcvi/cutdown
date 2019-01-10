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

void logger_init(void);

void cutdown_log(char * message);
void cutdown_log(char * message, float f1);
void cutdown_log(char * m1, float f1, char * m2, float f2);
void cutdown_log(char * message, int32_t i1);
void cutdown_log(char * message, uint32_t i1);

#endif