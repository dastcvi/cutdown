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
static void print_time(char * display_buffer, uint16_t timer_seconds)
{
    uint8_t display_seconds = 0;
    uint8_t display_minutes = 0;
    uint8_t display_hours = 0;

    display_seconds = timer_seconds % 60;
    timer_seconds /= 60; // get minutes
    display_minutes = timer_seconds % 60;
    display_hours = timer_seconds / 60;

    snprintf(display_buffer, 12, "%uh %um %us", display_hours, display_minutes, display_seconds);
}


Cutdown::Cutdown()
{
    state = ST_UNARMED;
    cutdown_timer = DEFAULT_TIMER;
    last_pressure = 1000.0f;
}


// called in arduino setup()
void Cutdown::init(void)
{
    cutdown_pinmux();
    oled.init();
    attiny.init();
    adc.init();
    gps.init();
    //Wire.begin();
    //baro.begin();
    config_init();
    timer_init();
    logger_init();

    // allows a small wait, and aligns timing
    wait_timer(2);

    // if the arm signal is already high, assume unplanned reboot
    if (arm_signal()) {
        if (cutdown_config.system_mode == MODE_CUTAWAY) {
            state = ST_CUTAWAY;
        } else {
            state = ST_CUTDOWN;
        }
    } else {
        if (attiny.write_timer(cutdown_config.backup_timer)) {
            cutdown_log("Wrote backup timer: %u", (uint32_t) cutdown_config.backup_timer);
        } else {
            cutdown_log("Error writing backup timer");
            oled.write_line("TBCK: error!", LINE2);
            wait_timer(5);
        }
        state = ST_UNARMED;
    }

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
    bool loop_toggle = true;

    attiny.disarm();

    oled.clear();
    if (cutdown_config.system_mode == MODE_CUTAWAY) {
        oled.write_line("CUTAWAY UNARMED", LINE1);
    } else {
        oled.write_line("CUTDOWN UNARMED", LINE1);
    }

    cutdown_log("Unarmed entered");

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

    while (!arm_signal()) {
        // different operation every other loop
        if (loop_toggle) {
            cutdown_timer = cutdown_config.primary_timer; // in case updated
            cycle_oled_info(true);
            if (cutdown_config.system_mode == MODE_CUTAWAY) {
                oled.write_line("CUTAWAY UNARMED", LINE1);
            } else {
                oled.write_line("CUTDOWN UNARMED", LINE1);
            }

            adc.thermal_control();

            pressure_log();

            if (!check_batteries(false)) {
                oled.write_line(" !Low Battery!  ", LINE2);
            }
        } else {
            config_update();
            oled.init();
            oled.clear();
            cycle_oled_info(false);

            if (update_backup_timer) {
                update_backup_timer = false;
                if (attiny.write_timer(cutdown_config.backup_timer)) {
                    cutdown_log("Wrote backup timer: %u", (uint32_t) cutdown_config.backup_timer);
                } else {
                    cutdown_log("Error updating backup timer from terminal");
                    oled.write_line("TBCK: error!", LINE2);
                    wait_timer(5);
                }
            }
        }

        loop_toggle ^= true; // toggle

        wait_timer(1);
    }

    state = ST_ARM;
}


void Cutdown::arm(void)
{
    bool gps_locked = false;
    bool loop_toggle = true;

    oled.clear();
    if (cutdown_config.system_mode == MODE_CUTAWAY) {
        oled.write_line("CUTAWAY GPS WAIT", LINE1);
    } else {
        oled.write_line("CUTDOWN GPS WAIT", LINE1);
    }

    attiny.arm();

    cutdown_log("Arming: waiting on GPS lock");

    while (!gps_locked) {
        if (loop_toggle) {
            cycle_oled_info(true);
                if (cutdown_config.system_mode == MODE_CUTAWAY) {
                    oled.write_line("CUTAWAY GPS WAIT", LINE1);
                } else {
                    oled.write_line("CUTDOWN GPS WAIT", LINE1);
                }

            adc.thermal_control();

            pressure_log();

            if (!check_batteries(false)) {
                oled.write_line(" !Low Battery!  ", LINE2);
            }
        } else {
            cycle_oled_info(false);
            gps_locked = (FIX_3D == gps.update_fix());
        }

        loop_toggle ^= true; // toggle

        wait_timer(1);

        // ensure still armed
        if (!arm_signal()) {
            state = ST_UNARMED;
            return;
        }
    }

    // set the flight origin based on the first lock
    cutdown_config.origin_lat = gps.gps_data.latitude;
    cutdown_config.origin_long = gps.gps_data.longitude;

    cutdown_log("GPS lock acquired, origin set");

    // switch to the configured armed state
    if (cutdown_config.system_mode == MODE_CUTAWAY) {
        state = ST_CUTAWAY;
    } else {
        state = ST_CUTDOWN;
    }
}


void Cutdown::cutdown(void)
{
    bool loop_toggle = true;
    bool trigger_met = false;

    oled.clear();
    oled.write_line("ARMED:", LINE1);
    oled.write_line("CUTDOWN", LINE2);

    cutdown_log("Armed in cutdown mode");

    // load the timer with the configured value
    cutdown_timer = cutdown_config.primary_timer;

    while (!trigger_met) {
        if (loop_toggle) {
            cycle_oled_info(true);
            oled.write_line("Armed: Cutdown", LINE1);

            adc.thermal_control();

            pressure_log();

            if (!check_batteries(true)) {
                oled.write_line(" !Low Battery!  ", LINE2);
            }
        } else {
            cycle_oled_info(false);

            if (gps_trigger()) {
                trigger_met = true;
                cutdown_log("GPS trigger met");
            }
        }

        loop_toggle ^= true;

        // wait for 1 second, but count all that pass in case the loop was slow
        cutdown_timer -= wait_timer(1);

        if (cutdown_timer <= 0) {
            trigger_met = true;
            cutdown_log("Timer met: %d", cutdown_timer);
        }

        // ensure still armed before moving on
        if (!arm_signal()) {
            state = ST_UNARMED;
            return;
        }
    }

    state = ST_FIRE;
}


void Cutdown::cutaway(void)
{
    bool loop_toggle = true;
    bool trigger_met = false;

    oled.clear();
    oled.write_line("ARMED:", LINE1);
    oled.write_line("CUTAWAY", LINE2);

    cutdown_log("Armed in cutaway mode");

    // load the timer with the configured value
    cutdown_timer = cutdown_config.primary_timer;

    while (!trigger_met) {
        if (loop_toggle) {
            cycle_oled_info(true);
            oled.write_line("Armed: Cutaway", LINE1);

            adc.thermal_control();

            if (!check_batteries(true)) {
                oled.write_line(" !Low Battery!  ", LINE2);
            }
        } else {
            cycle_oled_info(false);

            if (pressure_trigger()) {
                trigger_met = true;
                cutdown_log("Pressure trigger met");
            }
        }

        loop_toggle ^= true;

        // wait for 1 second, but count all that pass in case the loop was slow
        cutdown_timer -= wait_timer(1);

        if (cutdown_timer <= 0) {
            trigger_met = true;
            cutdown_log("Timer met: %d", cutdown_timer);
        }

        // ensure still armed before moving on
        if (!arm_signal()) {
            state = ST_UNARMED;
            return;
        }
    }

    state = ST_FIRE;
}


void Cutdown::fire(void)
{
    bool gps_fixed = false;
    float cut_height = 0.0f;

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

    // get a final GPS location before cutdown (NOT used for cutaway)
    gps_fixed = (FIX_3D == gps.update_fix());
    cut_height = gps.gps_data.height;

    digitalWrite(SQUIB_PRI_GATE, HIGH);
    delay(1000); // should only need a few ms
    digitalWrite(SQUIB_PRI_GATE, LOW);

    // if that didn't work, try again
    if (check_squib(adc.squib_pri.read())) {
        oled.write_line("Squib failed", LINE2);
        cutdown_log("Fire 1 failed");
        delay(1000);

        digitalWrite(SQUIB_PRI_GATE, HIGH);
        delay(1000); // should only need a few ms
        digitalWrite(SQUIB_PRI_GATE, LOW);

        // if that didn't work, fire the backup (which may not be present)
        if (check_squib(adc.squib_pri.read())) {
            cutdown_log("Fire 2 failed, firing backup");
            oled.write_line("Firing backup", LINE2);
            delay(1000);

            digitalWrite(SQUIB_BCK_GATE, HIGH);
            delay(1000); // should only need a few ms
            digitalWrite(SQUIB_BCK_GATE, LOW);

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

    // if cutdown mode, try to ensure that we're actually falling
    if (cutdown_config.system_mode == MODE_CUTDOWN && gps_fixed) {
        delay(1000); // allow to fall for a bit

        // make sure we aren't still going up
        if (FIX_3D == gps.update_fix() && cut_height < gps.gps_data.height) {
            cutdown_log("Still ascending after fire!");

            // re-fire primary
            digitalWrite(SQUIB_PRI_GATE, HIGH);
            delay(1000);
            digitalWrite(SQUIB_PRI_GATE, LOW);

            // re-fire backup
            digitalWrite(SQUIB_BCK_GATE, HIGH);
            delay(1000);
            digitalWrite(SQUIB_BCK_GATE, LOW);
        }
    }

    wait_timer(10);
    state = ST_FINISHED;
}


void Cutdown::finished(void)
{
    adc.thermal_control();

    gps_log();
    pressure_log();

    if (!check_batteries(false)) {
        oled.write_line(" !Low Battery!  ", LINE2);
    }

    wait_timer(5);
}


inline bool Cutdown::arm_signal(void)
{
    return (digitalRead(SYSTEM_ARM) == HIGH); // revA
}


// returns false if low battery, powers the system down if both critical
bool Cutdown::check_batteries(bool critical_stage)
{
    float batt1 = adc.v_batt_pri.read();
    float batt2 = adc.v_batt_bck.read();
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
        if (!critical_stage) cutdown_poweroff(); // don't power off if mid-flight
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


void Cutdown::gps_log()
{
    // only check the trigger with a valid, new 3-D fix
    if (FIX_3D != gps.update_fix()) {
        cutdown_log("no gps fix");
        gps.gps_data.height = GPS_INVALID_FLOAT;
        gps.gps_data.displacement = GPS_INVALID_FLOAT;
        return;
    }

    cutdown_log("lat ", gps.gps_data.latitude, ", long ", gps.gps_data.longitude);
    cutdown_log("height ", gps.gps_data.height);
}


bool Cutdown::pressure_trigger()
{
    static float last_pressures[10] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90}; // ensure no false static reading on reboot
    static uint8_t pressure_index = 0;
    static bool reached_ceiling = false; // todo: make non-volatile
    
    float running_avg = 0.0f;

    //last_pressure = baro.getPressure() / 100.0f;
    last_pressure = 850; // revA can't communicate with the pressure sensor

    // add to the last 10 pressures
    last_pressures[pressure_index] = last_pressure;
    pressure_index = (pressure_index + 1) % 10;

    cutdown_log("press ", last_pressure);

    if (last_pressure < cutdown_config.cutaway_ceiling) {
        if (!reached_ceiling) {
            reached_ceiling = true;
            cutdown_log("Reached ceiling");
        }
    } else if (reached_ceiling && last_pressure > cutdown_config.cutaway_ceiling) {
        // get the current running average
        for (int i = 0; i < 10; i++) {
            running_avg += last_pressures[i];
        }
        running_avg /= 10;
    
        // if running average and last pressure are within 1 hPa, trigger
        if (last_pressure - running_avg < 1.0f && last_pressure - running_avg > -1.0f) {
            return true;
        }
    }

    return false;
}


void Cutdown::pressure_log(void)
{
    // float mpl_temp = 0.0f;

    // last_pressure = baro.getPressure() / 100.0f;
    // mpl_temp = baro.getTemperature();

    // cutdown_log("press ", last_pressure);
    // cutdown_log("temp2 ", mpl_temp);
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

        // clear for the new info
        oled.clear();
    }

    switch (cycle_count) {
    case OI_TPRI:
        if (cycle) {
            cutdown_log("tpri %u", cutdown_timer);
        }
        snprintf(line2, 17, "PRI: ");
        print_time(line2+5, (uint16_t) cutdown_timer);
        oled.write_line(line2, LINE2);
        break;
    case OI_TBCK:
        backup_timer = attiny.read_timer();
        if (cycle) {
            cutdown_log("tbck %u", (uint32_t) backup_timer);
        }
        snprintf(line2, 17, "BCK: ");
        print_time(line2+5, (uint16_t) backup_timer);
        oled.write_line(line2, LINE2);
        break;
    case OI_SQUIB_PRI:
        if (check_squib(adc.squib_pri.read())) {
            oled.write_line("PRI Squib OK", LINE2);
            if (cycle) {
                cutdown_log("PRI Squib OK");
            }
        } else {
            oled.write_line("PRI squib error!", LINE2);
            if (cycle) {
                cutdown_log("PRI squib error!");
            }
        }
        break;
    case OI_SQUIB_BCK:
        // if (check_squib(adc.squib_bck.read())) {
        //     oled.write_line("BCK Squib OK", LINE2);
        //     if (cycle) {
        //         cutdown_log("BCK Squib OK");
        //     }
        // } else {
        //     oled.write_line("BCK squib error!", LINE2);
        //     if (cycle) {
        //         cutdown_log("BCK squib error!");
        //     }
        // }
        oled.write_line("BCK squib?", LINE2); // revA pinout issue
        break;
    case OI_LASTGPS:
        if (gps.gps_data.sol_time == GPS_NO_SOLUTION) {
            oled.write_line("No GPS solution", LINE2);
        } else {
            line2_str = "GPS age: ";
            line2_str += String((float) (millis() - gps.gps_data.sol_time) / 1000.0f);
            line2_str += "s";
            oled.write_line((char *) line2_str.c_str(), LINE2);
        }
        break;
    case OI_SET_DISTANCE:
        line2_str = "SetD: ";
        line2_str += String(cutdown_config.trigger_distance);
        line2_str += " km";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case OI_DISTANCE:
        if (gps.gps_data.displacement == GPS_INVALID_FLOAT) {
            oled.write_line("No GPS solution", LINE2);
        } else {
            line2_str = "CurD: ";
            line2_str += String(gps.gps_data.displacement);
            line2_str += " km";
            oled.write_line((char *) line2_str.c_str(), LINE2);
        }
        break;
    case OI_SET_HEIGHT:
        line2_str = "SetH: ";
        line2_str += String(cutdown_config.trigger_height);
        line2_str += " km";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case OI_HEIGHT:
        if (gps.gps_data.height == GPS_INVALID_FLOAT) {
            oled.write_line("No GPS solution", LINE2);
        } else {
            line2_str = "CurH: ";
            line2_str += String(gps.gps_data.height);
            line2_str += " km";
            oled.write_line((char *) line2_str.c_str(), LINE2);
        }
        break;
    case OI_PRESSURE:
        // line2_str = "P: ";
        // line2_str += String(last_pressure);
        // line2_str += " hPa";
        // oled.write_line((char *) line2_str.c_str(), LINE2);
        oled.write_line("No pressure read", LINE2);
        break;
    case OI_BATT1:
        line2_str = "PRI: ";
        line2_str += String(adc.v_batt_pri.check());
        line2_str += " V";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case OI_BATT2:
        line2_str = "BCK: ";
        line2_str += String(adc.v_batt_bck.check());
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