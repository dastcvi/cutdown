/* Author: Alex St. Clair
 * Filename: Cutdown_ADC.h
 * Created: 12-17-18
 * 
 * Defines a driver to use the SAMD21 ADC through the Arduino IDE
 */

#include "Cutdown_ADC.h"

// ADC_Channel methods ------------------------------------------
ADC_Channel::ADC_Channel(float last, float divide, uint8_t pin)
{
    last_voltage = last;
    divider = divide;
    pin_number = pin;
}

float ADC_Channel::read(void)
{
    last_voltage = (float) analogRead(pin_number) / ADC_MAX * REFERENCE_VOLTAGE / divider;

    return last_voltage;
}

float ADC_Channel::check(void)
{
    return last_voltage;
}

// Cutdown_ADC methods ------------------------------------------
Cutdown_ADC::Cutdown_ADC() :
    thermistor (0.0f,  THERM_DIVIDE,  THERMISTOR),
    squib1     (0.0f,  SQUIB1_DIVIDE, VMON_SQUIB1),
    squib2     (0.0f,  SQUIB1_DIVIDE, VMON_SQUIB2),
    v_3v3a     (0.0f,  V3V3A_DIVIDE,  VMON_3V3A),
    v_3v3b     (0.0f,  V3V3B_DIVIDE,  VMON_3V3B),
    v_batt1    (0.0f,  VBATT1_DIVIDE, VMON_BATT1),
    v_batt2    (0.0f,  VBATT2_DIVIDE, VMON_BATT2),
    v_batt     (0.0f,  VBATT_DIVIDE,  VMON_VBATT)
{ }

void Cutdown_ADC::init(void)
{
    analogReference(AR_INTERNAL2V23); // internal 2.23V reference
    analogReadResolution(12); // 12-bit resolution
}