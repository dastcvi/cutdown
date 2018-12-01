/* Author: Alex St. Clair
 * Filename: Cutdown.cpp
 * Created: 11-29-18
 * 
 * Implements a class that represents the cutdown controller and
 * performs its responsibilities.
 */

#include "Cutdown.h"


Cutdown::Cutdown() :
    oled()
{
    state = ST_UNARMED;
    cutdown_timer = DEFAULT_TIMER;
}

/* called in arduino setup() */
void Cutdown::init(void)
{
    cutdown_pinmux();
    oled.init();
    attiny.init();
}

/* called in arduino loop() */
void Cutdown::run(void)
{
    uint16_t timer_val = 0;
    bool timer_success = false;
    char line1[16] = "";
    char line2[16] = "";

    timer_success = attiny.write_timer(timer_val);
    timer_val = attiny.read_timer();

    sprintf(line1, "success: %d", timer_success);
    sprintf(line2, "timer: %u", timer_val);

    oled.write_line(line1, LINE1);
    oled.write_line(line2, LINE2);

    while(1);

    switch (state) {
        case ST_UNARMED:
            unarmed();
            break;
        case ST_ARMED:
            armed();
            break;
        case ST_FIRE:
            fire();
            break;
        case ST_FINISHED:
            finished();
            break;
        default:
            // this shouldn't happen
            break;
    }
}

void Cutdown::unarmed(void)
{
    oled.clear();
    oled.write_line("Cutdown", LINE1);
    oled.write_line("Unarmed", LINE2);

    while (digitalRead(SYSTEM_ARM) == LOW);

    state = ST_ARMED;
}

void Cutdown::armed(void)
{
    cutdown_timer = DEFAULT_TIMER;
    char timer_string[16] = "";

    oled.clear();
    oled.write_line("System", LINE1);
    oled.write_line("Armed", LINE2);

    delay(2000);

    while (cutdown_timer > 0)
    {
        if (digitalRead(SYSTEM_ARM) == LOW) {
            state = ST_UNARMED;
            return;
        }

        sprintf(timer_string, "%d s", cutdown_timer--);

        oled.clear();
        oled.write_line("Time remaining", LINE1);
        oled.write_line(timer_string, LINE2);
        
        delay(990); // todo: replace with timer interrupts
    }

    state = ST_FIRE;
}

void Cutdown::fire(void)
{
    oled.clear();
    oled.write_line("Firing 1", LINE1);

    digitalWrite(SQUIB1_GATE, HIGH);
    delay(1000); // should only need a few ms
    digitalWrite(SQUIB1_GATE, LOW);

    oled.write_line("Fired 1", LINE2);

    state = ST_FINISHED;
}

void Cutdown::finished(void)
{
    while (1); // todo: add vmon of batteries
}