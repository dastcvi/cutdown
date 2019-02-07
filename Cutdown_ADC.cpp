/* Author: Alex St. Clair
 * Filename: Cutdown_ADC.h
 * Created: 12-17-18
 * 
 * Defines a driver to use the SAMD21 ADC through the Arduino IDE
 */

#include "Cutdown_ADC.h"
#include "Cutdown_Logger.h"

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

// returns true if the squib is present and not fired (or shorted), false if not
bool check_squib(float voltage)
{
    return (voltage < SQUIB_THRESHOLD);
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
    thermistor  (0.0f,  THERM_DIVIDE, THERMISTOR),
    squib_pri   (0.0f,  SQUIB_DIVIDE, VMON_SQUIB_PRI),
    squib_bck   (0.0f,  SQUIB_DIVIDE, VMON_SQUIB_BCK),
    v_batt_pri  (12.0f, VBATT_DIVIDE, VMON_BATT_PRI),  // needs to start above low-battery threshold
    v_batt_bck  (12.0f, VBATT_DIVIDE, VMON_BATT_BCK)   // needs to start above low-battery threshold
{ }

void Cutdown_ADC::init(void)
{
    analogReference(AR_INTERNAL2V23); // internal 2.23V reference
    analogReadResolution(12); // 12-bit resolution
}

// TODO: add hysteresis!
void Cutdown_ADC::thermal_control(void)
{
    static bool heating = false;
    float temp = calculate_temperature(thermistor.read());

    cutdown_log("temp ", temp);

    if (TEMP_SETPOINT > temp && !heating) {
        digitalWrite(HEATER_GATE, HIGH);
        cutdown_log("Heater on");
        heating = true;
    } else if (TEMP_SETPOINT < temp && heating) {
        digitalWrite(HEATER_GATE, LOW);
        cutdown_log("Heater off");
        heating = false;
    }
}