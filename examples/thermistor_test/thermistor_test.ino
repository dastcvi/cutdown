#include <Cutdown_Pinout.h>
#include <Cutdown_OLED.h>
#include <Cutdown_ADC.h>
#include <Arduino.h>
#include "wiring_private.h"

Cutdown_OLED oled;
Cutdown_ADC adc;

void setup() {
  adc.init();
  oled.init();
  delay(1000);
}

void loop() {
  String celsius = "C: ";
  String fahrenheit = "F: ";
  
  adc.thermistor.read();

  celsius += String(calculate_temperature(adc.thermistor.check()));
  fahrenheit += String(calculate_fahrenheit(adc.thermistor.check()));

  oled.write_line((char *) celsius.c_str(), LINE1);
  oled.write_line((char *) fahrenheit.c_str(), LINE2);

  // battery safety
  if (adc.v_batt1.read() < 11.0 || adc.v_batt2.read() < 11.0) {
    if (adc.v_batt1.check() < 10.5 || adc.v_batt2.check() < 10.5) {
      Serial.println("Battery critical");
      cutdown_poweroff();
    }
    
    Serial.println("LOW BATTERY!");
    oled.write_line("Low battery", LINE1);
    oled.write_line("Low battery", LINE2);
  }

  delay(1000);
}
