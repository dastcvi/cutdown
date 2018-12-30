/* Author: Alex St. Clair
 * Filename: Cutdown_ADC.h
 * Created: 12-17-18
 * 
 * Defines a driver to use the SAMD21 ADC through the Arduino IDE
 */

#include "Cutdown_ADC.h"

// returns the temperature in celsius given the MCP9700A thermistor voltage
float calculate_temperature(float voltage)
{
    return (voltage - THERM_OFFSET) * THERM_COEFFICIENT;
}

// returns the temperature in fahrenheit given the MCP9700A thermistor voltage
float calculate_fahrenheit(float voltage)
{
    return ((voltage - THERM_OFFSET) * THERM_COEFFICIENT) * (1.8f) + 32;
}

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
    v_3v3a     (3.3f,  V3V3A_DIVIDE,  VMON_3V3A),
    v_3v3b     (3.3f,  V3V3B_DIVIDE,  VMON_3V3B),
    v_batt1    (12.0f, VBATT1_DIVIDE, VMON_BATT1),  // needs to start above low-battery threshold
    v_batt2    (12.0f, VBATT2_DIVIDE, VMON_BATT2),  // needs to start above low-battery threshold
    v_batt     (12.0f, VBATT_DIVIDE,  VMON_VBATT)   // needs to start above low-battery threshold
{ }

void Cutdown_ADC::init(void)
{
    analogReference(AR_INTERNAL2V23); // internal 2.23V reference
    analogReadResolution(12); // 12-bit resolution
}