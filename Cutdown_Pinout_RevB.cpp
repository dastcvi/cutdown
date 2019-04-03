/* Author: Alex St. Clair
 * Filename: Cutdown_Pinout.c
 * Created: 11-30-18
 * 
 * Implements the necessary cutdown pinmux
 */

#include "Cutdown_Pinout_RevB.h"

void cutdown_pinmux(void)
{
    /* arming and firing setup */
    pinMode(SYSTEM_ARM_N, INPUT);
    pinMode(SQUIB_PRI_GATE, OUTPUT);
    digitalWrite(SQUIB_PRI_GATE, LOW);
    pinMode(SQUIB_BCK_GATE, OUTPUT);
    digitalWrite(SQUIB_BCK_GATE, LOW);
    pinMode(POWER_OFF, OUTPUT);
    digitalWrite(POWER_OFF, LOW);
    pinMode(HEATER_GATE, OUTPUT);
    digitalWrite(HEATER_GATE, LOW);
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);
}

void cutdown_poweroff(void)
{
    digitalWrite(POWER_OFF, HIGH);
    delay(50);
    digitalWrite(POWER_OFF, LOW);
    delay(50);
}