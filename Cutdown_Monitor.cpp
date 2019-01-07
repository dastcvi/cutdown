/* Author: Alex St. Clair
 * Filename: Cutdown_Monitor.cpp
 * Created: 1-7-19
 * 
 * Implements functions to monitor both the health of the system and
 * also the various cutdown triggers.
 */

#include "Cutdown_Monitor.h"
#include "Cutdown_Configure.h"

// returns false if low battery, powers the system down if both critical
bool check_batteries(Cutdown_ADC * adc)
{
    float batt1 = adc->v_batt1.read();
    float batt2 = adc->v_batt2.read();
    bool critical = false;
    bool nominal = true;

    if (batt1 > 5.0f) { // assume not plugged in if <5V
        if (batt1 < cutdown_config.critical_batt_voltage) {
            critical = true;
            nominal = false;
        } else if (batt1 < cutdown_config.low_batt_voltage) {
            nominal = false;
        }
    }

    if (batt2 > 5.0f) { // assume not plugged in if <5V
        if (batt2 < cutdown_config.critical_batt_voltage) {
            critical &= true; // only critical if both are critical
            nominal = false;
        } else if (batt2 < cutdown_config.low_batt_voltage) {
            nominal = false;
        }
    }

    if (critical) cutdown_poweroff();

    return nominal;
}