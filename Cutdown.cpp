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


// OLED information messages
char * oled_messages[OI_NUM_INFO] = 
    {"Primary timer",
     "Backup timer",
     "Horizontal dist",
     "Height",
     "Battery 1",
     "Battery 2",
     "Temperature"};


// globals
Adafruit_ZeroTimer timer3 = Adafruit_ZeroTimer(3);
static uint8_t timer_seconds = 0;


// locally defined functions
static void t3_isr(void);
static uint8_t wait_timer(uint8_t check_value);
static void timer_init(void);
static void print_time(char display_buffer[17], uint16_t timer_seconds);


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


Cutdown::Cutdown()
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
    gps.init();
    config_init();
    timer_init();

    // allows a small wait, and aligns timing
    wait_timer(2);
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
    oled.write_line("SYSTEM UNARMED", LINE1);

    while (digitalRead(SYSTEM_ARM) == LOW) {
        config_update();
        
        oled.init(); // revA workaround, serial for configuring kills the OLED
        oled.clear();
        oled.write_line("SYSTEM UNARMED", LINE1);

        if (!check_batteries()) {
            oled.write_line("!Low Battery!", LINE2);
        }

        wait_timer(2);
    }

    //state = ST_GPSWAIT;
    state = ST_ARMED;
}


void Cutdown::gps_wait(void)
{
    uint16_t seconds_waiting = 0;
    char line2[17] = "";

    oled.clear();
    oled.write_line("Waiting on GPS", LINE1);

    if (!gps.stop_nmea()) {
        oled.write_line("GPS Config Error", LINE2);
        wait_timer(5);
        state = ST_UNARMED;
        return;
    }

    // wait for a fix for up to five minutes
    while (FIX_3D != gps.update_fix() && seconds_waiting < GPS_WAIT_TIME) {
        snprintf(line2, 17, "Num sats: %d", gps.gps_data.num_satellites);
        oled.write_line(line2, LINE2);

        // ensure no low batteries
        if (!check_batteries()) {
            oled.write_line("!Low Battery!", LINE2);
        }

        seconds_waiting += wait_timer(2);

        // ensure still armed before moving on
        if (digitalRead(SYSTEM_ARM) == LOW) {
            state = ST_UNARMED;
            return;
        }
    }

    if (gps.gps_data.fix_type == FIX_3D) {
        cutdown_config.origin_lat = gps.gps_data.latitude;
        cutdown_config.origin_long = gps.gps_data.longitude;
        state = ST_ARMED;
    } else {
        oled.clear();
        oled.write_line("GPS Timeout", LINE1);
        oled.write_line("No Fix Found", LINE2);
        wait_timer(5);
        state = ST_UNARMED;
    }
}


void Cutdown::armed(void)
{
    char line1[17] = "";
    char line2[17] = "";
    uint8_t loop_counter = 0;
    bool trigger_met = false;

    // load the timer with the configured value
    cutdown_timer = cutdown_config.primary_timer;

    // load and verify the backup timer with the configured value
    if (!attiny.write_timer(cutdown_config.backup_timer)) {
        oled.clear();
        oled.write_line("Backup timer", LINE1);
        oled.write_line("Failed to set", LINE2);
        wait_timer(5);
        state = ST_UNARMED;
        return;
    }

    oled.clear();
    oled.write_line("System", LINE1);
    oled.write_line("Armed", LINE2);
    wait_timer(2);

    attiny.arm(); // revA workaround, will be replaced in hardware

    while (!trigger_met)
    {
        for (uint8_t itr = 0; itr < 4; itr++) {
            if (itr == 0) {
                cycle_oled_info(true); // cycle to new type of info
            } else if (itr == 1) {
                cycle_oled_info(false); // update info without changing type
                if (gps_trigger()) trigger_met = true;
            } else if (itr == 2) {
                cycle_oled_info(false); // update info without changing type
                if (!check_batteries()) {
                    oled.write_line("!Low Battery!", LINE2);
                }
            } else {
                cycle_oled_info(false); // update info without changing type
                adc.thermistor.read();
            }
            
            // wait for 1 second, but count all that pass in case the loop was slow
            cutdown_timer -= wait_timer(1);

            if (cutdown_timer <= 0) trigger_met = true;

            // ensure still armed before moving on
            if (digitalRead(SYSTEM_ARM) == LOW) {
                state = ST_UNARMED;
                return;
            }
        }
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


// returns false if low battery, powers the system down if both critical
bool Cutdown::check_batteries()
{
    float batt1 = adc.v_batt1.read();
    float batt2 = adc.v_batt2.read();
    bool critical = false;
    bool nominal = true;

    if (batt1 > 5.0f) { // assume not plugged in if <5V
        if (batt1 < cutdown_config.critical_batt_voltage) {
            critical = true;
            nominal = false;
        } else if (batt1 < cutdown_config.low_batt_voltage) {
            nominal = false;
        }
    }

    if (batt2 > 5.0f) { // assume not plugged in if <5V
        if (batt2 < cutdown_config.critical_batt_voltage) {
            critical &= true; // only critical if both are critical
            nominal = false;
        } else if (batt2 < cutdown_config.low_batt_voltage) {
            nominal = false;
        }
    }

    if (critical) cutdown_poweroff();

    return nominal;
}


bool Cutdown::gps_trigger()
{
    // attempt to get a new reading
    if (FIX_3D != gps.update_fix()) return false;

    // see if we've reached the height trigger
    if (gps.gps_data.height >= cutdown_config.trigger_height) return true;

    // see if we've reached the distance trigger
    if (gps.distance_from(cutdown_config.origin_lat, cutdown_config.origin_long) >= cutdown_config.trigger_distance) return true;

    return false;
}


void Cutdown::cycle_oled_info(bool cycle)
{
    String line2_str = "";
    static char line2[17] = "";
    static uint8_t cycle_count = OI_NUM_INFO-1;
    
    if (cycle) {
        // update the number
        cycle_count = (++cycle_count) % OI_NUM_INFO;

        // write the info description
        oled.clear();
        oled.write_line(oled_messages[cycle_count], LINE1);
    }

    switch (cycle_count) {
    case OI_TPRI:
        print_time(line2, (uint16_t) cutdown_timer);
        oled.write_line(line2, LINE2);
        break;
    case OI_TBCK:
        print_time(line2, attiny.read_timer());
        oled.write_line(line2, LINE2);
        break;
    case OI_DISTANCE:
        line2_str = String(gps.gps_data.displacement);
        line2_str += " km";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case OI_HEIGHT:
        line2_str = String(gps.gps_data.height);
        line2_str += " km";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case OI_BATT1:
        line2_str = String(adc.v_batt1.check());
        line2_str += " V";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case OI_BATT2:
        line2_str = String(adc.v_batt2.check());
        line2_str += " V";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case OI_TEMP:
        line2_str = String(calculate_temperature(adc.thermistor.check()));
        line2_str += " C";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    default:
        break;
    }
}