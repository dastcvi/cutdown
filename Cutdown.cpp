/* Author: Alex St. Clair
 * Filename: Cutdown.cpp
 * Created: 11-29-18
 * 
 * Implements a class that represents the cutdown controller and
 * performs its responsibilities.
 */


#include "Cutdown.h"
#include "Cutdown_Logger.h"
#include "Cutdown_Configure.h"
#include "Adafruit_ZeroTimer.h"


// OLED information messages
char * oled_messages[OI_NUM_INFO] = 
    {"Primary timer",
     "Backup timer",
     "Squib status",
     "GPS solution age",
     "Distance trigger",
     "Current distance",
     "Height trigger",
     "Current height",
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
    reboot_detected = false;
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
    logger_init();

    // if the arm signal is already high, assume unplanned reboot
    if (arm_signal()) {
        state = ST_GPSWAIT; // skip unarmed
        reboot_detected = true;
    }

    // allows a small wait, and aligns timing
    wait_timer(2);

    cutdown_log("Initialized");
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
    // can come here after reboot if GPS fails to lock
    if (!reboot_detected) {
        attiny.disarm(); // revA-specific
    }

    oled.clear();
    oled.write_line("SYSTEM UNARMED", LINE1);

    cutdown_log("Unarmed entered");

    while (!arm_signal()) {
        adc.thermal_control();

        config_update();
        
        oled.init(); // revA-specific workaround, serial for configuring kills the OLED
        oled.clear(); // revA-specific
        oled.write_line("SYSTEM UNARMED", LINE1); // revA-specific

        if (!check_batteries()) {
            oled.write_line("!Low Battery!", LINE2);
        }

        wait_timer(2);
    }

    state = ST_GPSWAIT;
}


void Cutdown::gps_wait(void)
{
    uint16_t seconds_waiting = 0;
    char line2[17] = "";

    oled.clear();
    oled.write_line("Waiting on GPS", LINE1);

    if (!gps.stop_nmea()) {
        cutdown_log("Error muting nmea");
        oled.write_line("GPS Config Error", LINE2);
        wait_timer(5);
        state = ST_UNARMED;
        return;
    }

    if (!gps.set_airborne()) {
        cutdown_log("Error setting to airborne");
        oled.write_line("GPS Config Error", LINE2);
        wait_timer(5);
        state = ST_UNARMED;
        return;
    }

    cutdown_log("GPS configured");

    // wait for a fix for up to five minutes
    while (FIX_3D != gps.update_fix() && seconds_waiting < GPS_WAIT_TIME) {
        adc.thermal_control();

        snprintf(line2, 17, "Num sats: %d", gps.gps_data.num_satellites);
        oled.write_line(line2, LINE2);

        // ensure no low batteries
        if (!check_batteries()) {
            oled.write_line("!Low Battery!", LINE2);
        }

        seconds_waiting += wait_timer(2);

        // ensure still armed before moving on
        if (!arm_signal()) {
            reboot_detected = false; // user reset
            state = ST_UNARMED;
            return;
        }
    }

    if (gps.gps_data.fix_type == FIX_3D) {
        if (!reboot_detected) {
            cutdown_config.origin_lat = gps.gps_data.latitude;
            cutdown_config.origin_long = gps.gps_data.longitude;
            cutdown_log("GPS lock, origin set");
        }
        cutdown_log("GPS lock");
        state = ST_ARMED;
    } else {
        oled.clear();
        oled.write_line("GPS Timeout", LINE1);
        oled.write_line("No Fix Found", LINE2);
        cutdown_log("GPS timeout, no lock found");
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

    cutdown_log("Armed entered");

    // set the backup timer if this is the initial arming
    if (!reboot_detected) {
        cutdown_log("First arm");

        // load the timer with the configured value
        cutdown_timer = cutdown_config.primary_timer;

        // load and verify the backup timer with the configured value
        if (!attiny.write_timer(cutdown_config.backup_timer)) {
            oled.clear();
            oled.write_line("Backup timer", LINE1);
            oled.write_line("Failed to set", LINE2);
            cutdown_log("Failed to set backup timer");
            wait_timer(5);
            state = ST_UNARMED;
            return;
        }

        oled.clear();
        oled.write_line("System", LINE1);
        oled.write_line("Armed", LINE2);
        cutdown_log("System armed");
        wait_timer(2);

        attiny.arm(); // revA-specific workaround, will be replaced in hardware
    }

    while (!trigger_met) {
        adc.thermal_control();

        for (uint8_t itr = 0; itr < 4; itr++) {
            if (itr == 0) {
                cycle_oled_info(true); // cycle to new type of info
            } else if (itr == 1) {
                cycle_oled_info(false); // update info without changing type
                if (gps_trigger()) {
                    trigger_met = true;
                    cutdown_log("GPS trigger met");
                }
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

            if (cutdown_timer <= 0) {
                trigger_met = true;
                cutdown_log("Timer met: %d", cutdown_timer);
            }

            // ensure still armed before moving on
            if (!arm_signal()) {
                reboot_detected = false; // user reset
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
    oled.write_line("Trigger Reached!", LINE1);
    cutdown_log("Fire state!");
    wait_timer(1);

    oled.write_line("Firing: 5", LINE2);
    wait_timer(1);
    oled.write_line("Firing: 4", LINE2);
    wait_timer(1);
    oled.write_line("Firing: 3", LINE2);
    wait_timer(1);
    oled.write_line("Firing: 2", LINE2);
    wait_timer(1);
    oled.write_line("Firing: 1", LINE2);
    wait_timer(1);

    digitalWrite(SQUIB2_GATE, HIGH);
    delay(1000); // should only need a few ms
    digitalWrite(SQUIB2_GATE, LOW);

    // if that didn't work, try again
    if (check_squib(adc.squib2.read())) {
        oled.write_line("Squib failed", LINE2);
        cutdown_log("Fire 1 failed");
        delay(1000);

        digitalWrite(SQUIB2_GATE, HIGH);
        delay(1000); // should only need a few ms
        digitalWrite(SQUIB2_GATE, LOW);

        // if that didn't work, fire the backup
        if (check_squib(adc.squib2.read())) {
            cutdown_log("Fire 2 failed, firing backup");
            oled.write_line("Firing backup", LINE2);
            delay(1000);

            digitalWrite(SQUIB1_GATE, HIGH);
            delay(1000); // should only need a few ms
            digitalWrite(SQUIB1_GATE, LOW);

            oled.write_line("Fired backup ", LINE2);
            cutdown_log("Fired backup");
        } else {
            oled.write_line("Squib succeeded", LINE2);
            cutdown_log("Fire success");
        }
    } else {
        oled.write_line("Squib succeeded", LINE2);
        cutdown_log("Fire success");
    }

    // keep the backup MCU from firing another squib
#ifndef DEMO_BACKUP_TIMER
    digitalWrite(SQUIB_FIRED, HIGH);  // revA-specific
#endif

    wait_timer(10);
    state = ST_FINISHED;
}


void Cutdown::finished(void)
{
    adc.thermal_control();

    if (!check_batteries()) {
        oled.write_line("!Low Battery!", LINE2);
    }

    wait_timer(5);
}


inline bool Cutdown::arm_signal(void)
{
    return (digitalRead(SYSTEM_ARM) == HIGH);
}


// returns false if low battery, powers the system down if both critical
bool Cutdown::check_batteries(void)
{
    float batt1 = adc.v_batt1.read();
    float batt2 = adc.v_batt2.read();
    bool critical = false;
    bool nominal = true;

    cutdown_log("b1 ", batt1, ", b2 ", batt2);

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

    if (critical) {
        cutdown_log("Critical battery, power down");
        cutdown_poweroff();
    }

    return nominal;
}


bool Cutdown::gps_trigger()
{
    float dist = 0.0f;

    // only check the trigger with a valid, new 3-D fix
    if (FIX_3D != gps.update_fix()) {
        cutdown_log("no gps fix");
        gps.gps_data.height = GPS_INVALID_FLOAT;
        gps.gps_data.displacement = GPS_INVALID_FLOAT;
        return false;
    }

    cutdown_log("lat ", gps.gps_data.latitude, ", long ", gps.gps_data.longitude);
    cutdown_log("height ", gps.gps_data.height);

    // see if we've reached the height trigger
    if (gps.gps_data.height >= cutdown_config.trigger_height) return true;

    // see if we've reached the distance trigger
    dist = gps.distance_from(cutdown_config.origin_lat, cutdown_config.origin_long);
    cutdown_log("dist ", dist);
    if (dist >= cutdown_config.trigger_distance) return true;

    return false;
}


void Cutdown::cycle_oled_info(bool cycle)
{
    String line2_str = "";
    static char line2[17] = "";
    static uint8_t cycle_count = OI_NUM_INFO-1;
    uint16_t backup_timer = 0;
    
    if (cycle) {
        // update the number
        cycle_count = (++cycle_count) % OI_NUM_INFO;

        // write the info description
        oled.clear();
        oled.write_line(oled_messages[cycle_count], LINE1);
    }

    switch (cycle_count) {
    case OI_TPRI:
        if (cycle) {
            cutdown_log("tpri %u", cutdown_timer);
        }
        print_time(line2, (uint16_t) cutdown_timer);
        oled.write_line(line2, LINE2);
        break;
    case OI_TBCK:
        backup_timer = attiny.read_timer();
        if (cycle) {
            cutdown_log("tbck %u", (uint32_t) backup_timer);
        }
        print_time(line2, backup_timer);
        oled.write_line(line2, LINE2);
        break;
    case OI_SQUIB:
        if (check_squib(adc.squib2.read())) {
            oled.write_line("Squib OK", LINE2);
            if (cycle) {
                cutdown_log("Squib OK");
            }
        } else {
            oled.write_line("!Error!", LINE2);
            if (cycle) {
                cutdown_log("Squib error");
            }
        }
        break;
    case OI_LASTGPS:
        if (gps.gps_data.sol_time == GPS_NO_SOLUTION) {
            oled.write_line("No solution", LINE2);
        } else {
            line2_str = String((float) (millis() - gps.gps_data.sol_time) / 1000.0f);
            line2_str += " s";
            oled.write_line((char *) line2_str.c_str(), LINE2);
        }
        break;
    case OI_SET_DISTANCE:
        line2_str = String(cutdown_config.trigger_distance);
        line2_str += " km";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case OI_DISTANCE:
        if (gps.gps_data.displacement == GPS_INVALID_FLOAT) {
            oled.write_line("No fix", LINE2);
        } else {
            line2_str = String(gps.gps_data.displacement);
            line2_str += " km";
            oled.write_line((char *) line2_str.c_str(), LINE2);
        }
        break;
    case OI_SET_HEIGHT:
        line2_str = String(cutdown_config.trigger_height);
        line2_str += " km";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case OI_HEIGHT:
        if (gps.gps_data.height == GPS_INVALID_FLOAT) {
            oled.write_line("No fix", LINE2);
        } else {
            line2_str = String(gps.gps_data.height);
            line2_str += " km";
            oled.write_line((char *) line2_str.c_str(), LINE2);
        }
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