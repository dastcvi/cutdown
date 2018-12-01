/* Author: Alex St. Clair
 * Filename: Cutdown_Pinout.c
 * Created: 11-30-18
 * 
 * Implements the necessary cutdown pinmux
 */

#include "Cutdown_Pinout.h"

void cutdown_pinmux(void)
{
    /* arming and firing setup */
    pinMode(SYSTEM_ARM, INPUT);
    pinMode(SQUIB1_GATE, OUTPUT);
    digitalWrite(SQUIB1_GATE, LOW);
    pinMode(SQUIB2_GATE, OUTPUT);
    digitalWrite(SQUIB2_GATE, LOW);
}