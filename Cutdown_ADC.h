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

// General ADC constants
#define REFERENCE_VOLTAGE	(2.23f)
#define ADC_MAX	            (4095.0f)

// Voltage dividers by channel
#define THERM_DIVIDE    (1.0f)    // direct
#define SQUIB1_DIVIDE   (1.0f)    // direct
#define SQUIB2_DIVIDE   (1.0f)    // direct
#define V3V3A_DIVIDE    (0.5f)    // 3v3a   - 10k   - ADC - 10k - GND
#define V3V3B_DIVIDE    (0.5f)    // 3v3a   - 10k   - ADC - 10k - GND
#define VBATT1_DIVIDE   (0.1104f) // vbatt1 - 80.6k - ADC - 10k - GND
#define VBATT2_DIVIDE   (0.1104f) // vbatt2 - 80.6k - ADC - 10k - GND
#define VBATT_DIVIDE    (0.1104f) // vbatt  - 80.6k - ADC - 10k - GND

class ADC_Channel {
public:
    ADC_Channel(float last, float divide, uint8_t pin);

    float read(void);

    float last_voltage;
    float divider;
    uint8_t pin_number;
};

class Cutdown_ADC {
public:
    Cutdown_ADC();
    ~Cutdown_ADC() { };
    void init(void);

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
