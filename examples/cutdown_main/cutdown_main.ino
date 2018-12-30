/* Author:   Alex St. Clair
 * Filename: cutdown_main.ino
 * Created:  11-19-18
 * 
 * This is the simple main arduino script to run the
 * SAMD21 MCU on the Cutdown board.
 * 
 * It should be programmed over USB with the Arduino
 * IDE target board set as the Adafruit Feather M0
 */

#include <Cutdown.h>

Cutdown cutdown;

void setup() {
  Serial.begin(115200);
  cutdown.init();
}

void loop() {
  cutdown.run();
}
