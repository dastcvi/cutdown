/* Author: Alex St. Clair
 * Filename: Cutdown_ADC.h
 * Created: 12-17-18
 * 
 * Defines a driver to use the SAMD21 ADC through the Arduino IDE
 */

#ifndef CUTDOWN_ADC_H
#define CUTDOWN_ADC_H

#include "Cutdown_Pinout.h"
#include "Arduino.h"
#include "wiring_private.h"
#include <stdint.h>

// MCP9700A thermistor constants
#define THERM_OFFSET	    (0.58f)
#define THERM_COEFFICIENT	(110.0f)
#define TEMP_SETPOINT       (5.0f) // 5 C

// General ADC constants
#define REFERENCE_VOLTAGE	(2.23f)
#define ADC_MAX	            (4095.0f)

// General squib constants
#define SQUIB_THRESHOLD     (1.32f)

// Voltage dividers by channel
#define THERM_DIVIDE    (1.0f)    // direct
#define SQUIB1_DIVIDE   (1.0f)    // direct
#define SQUIB2_DIVIDE   (1.0f)    // direct
#define V3V3A_DIVIDE    (0.5f)    // 3v3a   - 10k   - ADC - 10k - GND
#define V3V3B_DIVIDE    (0.5f)    // 3v3a   - 10k   - ADC - 10k - GND
#define VBATT1_DIVIDE   (0.1104f) // vbatt1 - 80.6k - ADC - 10k - GND
#define VBATT2_DIVIDE   (0.1104f) // vbatt2 - 80.6k - ADC - 10k - GND
#define VBATT_DIVIDE    (0.1104f) // vbatt  - 80.6k - ADC - 10k - GND

// returns the temperature in celsius given the MCP9700A thermistor voltage
float calculate_temperature(float voltage);

// returns the temperature in fahrenheit given the MCP9700A thermistor voltage
float calculate_fahrenheit(float voltage);

// returns true if the squib is present and not fired (or shorted), false if not
bool check_squib(float voltage);

class ADC_Channel {
public:
    ADC_Channel(float last, float divide, uint8_t pin);

    // read the channel and return the calculated result
    float read(void);

    // check the last voltage without forcing a new read
    float check(void);

    float last_voltage;
    float divider;
    uint8_t pin_number;
};

class Cutdown_ADC {
public:
    Cutdown_ADC();
    ~Cutdown_ADC() { };
    void init(void);

    // simple thermal control, should be called every 1-5s
    void thermal_control(void);

    // Channels
    ADC_Channel thermistor;
    ADC_Channel squib1;
    ADC_Channel squib2;
    ADC_Channel v_3v3a;
    ADC_Channel v_3v3b;
    ADC_Channel v_batt1;
    ADC_Channel v_batt2;
    ADC_Channel v_batt;
};

#endif
