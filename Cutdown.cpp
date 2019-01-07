/* Author: Alex St. Clair
 * Filename: Cutdown.cpp
 * Created: 11-29-18
 * 
 * Implements a class that represents the cutdown controller and
 * performs its responsibilities.
 */

#include "Cutdown.h"
#include "Cutdown_Configure.h"
#include "Adafruit_ZeroTimer.h"

Adafruit_ZeroTimer timer3 = Adafruit_ZeroTimer(3);
static uint8_t timer_seconds = 0;

// vector the timer ISR to the Adafruit_ZeroTimer
void TC3_Handler()
{
    Adafruit_ZeroTimer::timerHandler(3);
}

// ISR for the timer
static void t3_isr(void)
{
    __disable_irq();
    timer_seconds++;
    __enable_irq();
}

// waits until >= check_value seconds have passed, returns the seconds, otherwise 0
static uint8_t wait_timer(uint8_t check_value)
{
    uint8_t ret_value = 0;
    
    while (ret_value == 0) {
        __disable_irq();
        if (timer_seconds >= check_value) {
            ret_value = timer_seconds;
            timer_seconds = 0;
        }
        __enable_irq();
        
        delay(1); // no spamming the critical section
    }

    return ret_value;
}

// initialize timer for 1 Hz operation
static void timer_init(void)
{
    timer3.configure(TC_CLOCK_PRESCALER_DIV1024,
                     TC_COUNTER_SIZE_16BIT,
                     TC_WAVE_GENERATION_MATCH_PWM // interrupt on match
                     );

    timer3.setPeriodMatch(46875, 46875, 0); // 48 MHz / 1024 / 46875 = 1 Hz
    timer3.setCallback(true, TC_CALLBACK_CC_CHANNEL0, t3_isr);
    timer3.enable(true);
}

// print the time in h/m/s format for the oled
static void print_time(char display_buffer[17], uint16_t timer_seconds)
{
    uint8_t display_seconds = 0;
    uint8_t display_minutes = 0;
    uint8_t display_hours = 0;

    display_seconds = timer_seconds % 60;
    timer_seconds /= 60; // get minutes
    display_minutes = timer_seconds % 60;
    display_hours = timer_seconds / 60;

    snprintf(display_buffer, 17, "%uh %um %us", display_hours, display_minutes, display_seconds);
}

Cutdown::Cutdown() :
    oled()
{
    state = ST_UNARMED;
    cutdown_timer = DEFAULT_TIMER;
}

// called in arduino setup()
void Cutdown::init(void)
{
    cutdown_pinmux();
    oled.init();
    attiny.init();
    adc.init();
    config_init();
    timer_init();

    // allows a small wait, and aligns timing
    wait_timer(1);
}

/* called in arduino loop() */
void Cutdown::run(void)
{
    // check for an out of bounds state
    if (state < 0 || state >= NUM_STATES) state = ST_UNARMED;

    // call the method for the new state
    (this->*(state_array[state]))();
}

void Cutdown::unarmed(void)
{
    attiny.disarm();

    oled.clear();
    oled.write_line("Cutdown", LINE1);
    oled.write_line("Unarmed", LINE2);

    while (digitalRead(SYSTEM_ARM) == LOW) {
        config_update();
        oled.init(); // revA workaround, serial for configuring kills the OLED
        oled.clear();
        oled.write_line("Cutdown", LINE1);
        oled.write_line("Unarmed", LINE2);
        wait_timer(1);
    }

    state = ST_ARMED;
}

void Cutdown::armed(void)
{
    cutdown_timer = cutdown_config.primary_timer;
    char line1[17] = "";
    char line2[17] = "";

    snprintf(line1, 17, "Backup timer:");
    if (attiny.write_timer(cutdown_config.backup_timer)) {
        print_time(line2, cutdown_config.backup_timer);
    } else {
        snprintf(line2, 17, "FAILED TO SET");
    }

    oled.clear();
    oled.write_line(line1, LINE1);
    oled.write_line(line2, LINE2);

    wait_timer(2);

    oled.clear();
    oled.write_line("System", LINE1);
    oled.write_line("Armed", LINE2);

    wait_timer(2);

    attiny.arm(); // revA workaround, will be replaced in hardware

    while (cutdown_timer > 0)
    {
        if (digitalRead(SYSTEM_ARM) == LOW) {
            state = ST_UNARMED;
            return;
        }

        print_time(line2, (uint16_t) cutdown_timer);

        oled.clear();
        oled.write_line("Time remaining", LINE1);
        oled.write_line(line2, LINE2);
        
        // wait for 1 second, but count all that pass in case the loop was slow
        cutdown_timer -= wait_timer(1);
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

    // keep the backup MCU from firing another squib
#ifndef DEMO_BACKUP_TIMER
    digitalWrite(SQUIB_FIRED, HIGH);
#endif

    delay(1000);

    oled.clear();
    oled.write_line("Fired", LINE1);
    state = ST_FINISHED;
}

void Cutdown::finished(void)
{
    // battery safety
    if (adc.v_batt1.read() < 11.1 || adc.v_batt2.read() < 11.1) {
        if (adc.v_batt1.check() < 10.8 || adc.v_batt2.check() < 10.8) {
            cutdown_poweroff();
        }

    oled.clear();
    oled.write_line("Low battery", LINE1);
    }

    wait_timer(5);
}