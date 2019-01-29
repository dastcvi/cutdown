/* Author: Alex St. Clair
 * Filename: Cutdown_Pinout.c
 * Created: 11-30-18
 * 
 * Implements the necessary cutdown pinmux
 */

#include "Cutdown_Pinout_RevA.h"

void cutdown_pinmux(void)
{
    /* arming and firing setup */
    pinMode(SYSTEM_ARM, INPUT);
    pinMode(SQUIB1_GATE, OUTPUT);
    digitalWrite(SQUIB1_GATE, LOW);
    pinMode(SQUIB2_GATE, OUTPUT);
    digitalWrite(SQUIB2_GATE, LOW);
    pinMode(SQUIB_FIRED, OUTPUT);   // revA-specific
    digitalWrite(SQUIB_FIRED, LOW); // revA-specific
    pinMode(POWER_OFF, OUTPUT);
    digitalWrite(POWER_OFF, LOW);
    pinMode(HEATER_GATE, OUTPUT);
    digitalWrite(HEATER_GATE, LOW);
}

void cutdown_poweroff(void)
{
    digitalWrite(POWER_OFF, HIGH);
    delay(50);
    digitalWrite(POWER_OFF, LOW);
    delay(50);
}