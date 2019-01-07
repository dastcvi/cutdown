/* Author: Alex St. Clair
 * Filename: Cutdown_Monitor.h
 * Created: 1-7-19
 * 
 * Defines functions to monitor both the health of the system and
 * also the various cutdown triggers.
 */

#ifndef CUTDOWN_MONITOR_H
#define CUTDOWN_MONITOR_H

#include "Cutdown_Pinout.h"
#include "Cutdown_OLED.h"
#include "Cutdown_ADC.h"
#include <stdint.h>

bool check_batteries(Cutdown_ADC * adc);

#endif