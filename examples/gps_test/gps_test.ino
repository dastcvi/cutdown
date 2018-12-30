#include <Cutdown_Pinout.h>
#include <Cutdown_OLED.h>
#include <Cutdown_ADC.h>
#include <Cutdown_GPS.h>
#include <Arduino.h>
#include "wiring_private.h"

Cutdown_OLED oled;
Cutdown_GPS gps;
Cutdown_ADC adc;

void setup() {
  Serial.begin(115200);
  Serial.println("Started");

  adc.init();
  gps.init();
  oled.init();

  // OLED SPI only works without serial monitor
  oled.write_line("Started GPS", LINE1);
  
  delay(1000);

  if (gps.stop_nmea()) {
    Serial.println("Stopped NMEA strings");
  } else {
    Serial.println("Error stopping NMEA strings");
  }

  delay(1000);
}

void loop() {
  GPS_FIX_TYPE_t fix = gps.update_fix();
  if (fix == FIX_3D) {
    Serial.println("Full 3D fix");
    oled.write_line("Fix: Full 3-D", LINE2);
  } else if (fix == FIX_2D) {
    Serial.println("2D fix");
    oled.write_line("Fix: 2-D Only", LINE2);
  } else if (fix == FIX_DEADRECKON) {
    Serial.println("Dead reckoning fix");
    oled.write_line("Fix: Dead Reckon", LINE2);
  } else if (fix == FIX_TIMEONLY) {
    Serial.println("Time only fix");
    oled.write_line("Fix: Time Only", LINE2);
  } else if (fix == FIX_GNSS) {
    Serial.println("GNSS fix");
    oled.write_line("Fix: GNSS", LINE2);
  } else {
    Serial.println("No fix");
    oled.write_line("Fix: None", LINE2);
  }
  
  Serial.print("Location: "); Serial.print(gps.gps_data.latitude, 5);
  Serial.print(", "); Serial.println(gps.gps_data.longitude, 5);
  Serial.print("Height: "); Serial.println(gps.gps_data.height);
  Serial.print("Sats: "); Serial.println(gps.gps_data.num_satellites);
  Serial.println();

  Serial.print("3v3a:   "); Serial.print(adc.v_3v3a.read());     Serial.println(" V");
  Serial.print("3v3b:   "); Serial.print(adc.v_3v3b.read());     Serial.println(" V");
  Serial.print("Batt1:  "); Serial.print(adc.v_batt1.read());    Serial.println(" V");
  Serial.print("Batt2:  "); Serial.print(adc.v_batt2.read());    Serial.println(" V");
  Serial.print("Squib2: "); Serial.print(adc.squib2.read());     Serial.println(" V");
  Serial.print("Therm:  "); Serial.print(adc.thermistor.read()); Serial.println(" V");
  Serial.println();

  // battery safety
  if (adc.v_batt1.check() < 11.0 || adc.v_batt2.check() < 11.0) {
    if (adc.v_batt1.check() < 10.5 || adc.v_batt2.check() < 10.5) {
      Serial.println("Battery critical");
      cutdown_poweroff();
    }
    
    Serial.println("LOW BATTERY!");
    oled.write_line("Low battery", LINE1);
    oled.write_line("Low battery", LINE2);
  }

  delay(3000);
}
