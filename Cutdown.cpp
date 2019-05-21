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
#include "Cutdown_Commission.h"
#include "Adafruit_ZeroTimer.h"
#include "Adafruit_SleepyDog.h"


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
    last_pressure = 1000.0f;
    last_fee_write = 65000; // will be overwritten
    backup_timer = 65000; // will be overwritten
    bool gps_ok = false;
    bool squibs_ok = false;
    bool pressure_ok = false;
    bool tpri_ok = false;
    bool tbck_ok = false;
    bool batt_ok = false;
    bool temp_ok = false;
}


// called in arduino setup()
void Cutdown::init(void)
{
    delay(2000);

    // if bad config AND not armed, enter commissioning (WILL NEED POWER CYCLE TO RUN NORMALLY!)
    if (!load_config_from_fee() && !arm_signal()) {
        cutdown_log(LOG_ERROR, "Config error, entering commission mode!");
        commission_setup();
        while(1) commission_loop();
    }

    // logging header
    if (cutdown_config.system_mode == MODE_CUTAWAY) {
        cutdown_log(LOG_INFO, "System powered on, mode: CUTAWAY");
    } else if (cutdown_config.system_mode == MODE_CUTDOWN) {
        cutdown_log(LOG_INFO, "System powered on, mode: CUTDOWN");
    } else {
        cutdown_log(LOG_ERROR, "System powered on, mode: ERROR");
    }
    cutdown_log(LOG_INFO, "Serial Number: %u", (uint32_t) cutdown_config.serial_number);

    // start drivers
    cutdown_pinmux();
    oled.init();
    attiny.init();
    adc.init();
    gps.init();
    Wire.begin();
    baro.begin();
    timer_init();
    logger_init();
    Watchdog.enable(8000); // loose 8 second reset period

    // allows a small wait, and aligns timing
    wait_timer(2);

    // initialize last timer write to current remaining timer
    last_fee_write = cutdown_config.primary_timer_remaining;

    Watchdog.reset();

    // if the arm signal is already high, assume unplanned reboot
    if (arm_signal()) {
        if (cutdown_config.system_mode == MODE_CUTAWAY) {
            state = ST_CUTAWAY;
        } else {
            state = ST_CUTDOWN;
        }
    } else {
        // ONLY if not armed, check the ATtiny's timer
        if (attiny.read_timer() == cutdown_config.backup_timer) {
            cutdown_log(LOG_INFO, "Backup timer matches config");
        } else {
            if (attiny.write_timer(cutdown_config.backup_timer)) {
                cutdown_log(LOG_INFO, "Wrote backup timer: %u", (uint32_t) cutdown_config.backup_timer);
            } else {
                cutdown_log(LOG_ERROR, "Error writing backup timer");
                oled.write_line("TBCK: error!", LINE2);
                wait_timer(5);
            }
        }
        state = ST_UNARMED;
    }

    Watchdog.reset();
    backup_timer = attiny.read_timer();

    cutdown_log(LOG_INFO, "Initialized");
}


/* called in arduino loop(), runs the state machine */
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

    cutdown_log(LOG_INFO, "Unarmed entered");

    Watchdog.reset();

    // reset cutaway ceiling tracker and low altitude timer
    cutdown_config.ceiling_reached = false;

    // reset the timer
    cutdown_config.primary_timer_remaining = cutdown_config.primary_timer;
    last_fee_write = cutdown_config.primary_timer_remaining;
    write_config_to_fee();

    // update the OLED
    oled.clear();
    if (cutdown_config.system_mode == MODE_CUTAWAY) {
        oled.write_line("CUTAWAY UNARMED", LINE1);
    } else {
        oled.write_line("CUTDOWN UNARMED", LINE1);
    }

    // re-initialize the GPS
    if (!gps.stop_nmea()) {
        cutdown_log(LOG_ERROR, "Error muting nmea");
        oled.write_line("GPS Config Error", LINE2);
        Watchdog.reset();
        wait_timer(5);
        state = ST_UNARMED;
        return;
    }
    if (!gps.set_airborne()) {
        cutdown_log(LOG_ERROR, "Error setting to airborne");
        oled.write_line("GPS Config Error", LINE2);
        Watchdog.reset();
        wait_timer(5);
        state = ST_UNARMED;
        return;
    }
    cutdown_log(LOG_INFO, "GPS configured");

    // re-write backup timer
    if (!attiny.write_timer(cutdown_config.backup_timer)) {
        cutdown_log(LOG_ERROR, "Error writing backup timer in unarmed");
        oled.write_line("TBCK: WriteError", LINE2);
        Watchdog.reset();
        wait_timer(5);
        state = ST_UNARMED;
        return;
    }

    // wait to be armed, while checking for config updates
    while (!arm_signal()) {
        Watchdog.reset();
        
        // get new OLED info
        cycle_oled_info();
        if (cutdown_config.system_mode == MODE_CUTAWAY) {
            oled.write_line("CUTAWAY UNARMED", LINE1);
        } else {
            oled.write_line("CUTDOWN UNARMED", LINE1);
        }

        // different operation every other loop
        if (loop_toggle) {
            adc.thermal_control();
            pressure_log();
            gps.update_fix();

            if (!check_batteries(false)) {
                oled.write_line(" !Low Battery!  ", LINE2);
            }
        } else {
            // check for new config messages
            config_check_serial();

            // update the backup timer if changed
            if (update_backup_timer) {
                update_backup_timer = false;
                if (attiny.write_timer(cutdown_config.backup_timer)) {
                    cutdown_log(LOG_INFO, "Wrote backup timer: %u", (uint32_t) cutdown_config.backup_timer);
                } else {
                    cutdown_log(LOG_ERROR, "Error updating backup timer from terminal");
                    oled.write_line("TBCK: error!", LINE2);
                    wait_timer(5);
                }
            }
            
            // reset primary timer in case updated
            cutdown_config.primary_timer_remaining = cutdown_config.primary_timer;
            last_fee_write = cutdown_config.primary_timer_remaining;
        }

        loop_toggle ^= true; // toggle

        wait_timer(1);
    }

    Watchdog.reset();

    // loop exited => armed
    if (cutdown_config.system_mode == MODE_CUTDOWN) {
        state = ST_ARM; // go to the wait for GPS state (required because first lock is kept as origin for triggers)
    } else {
        state = ST_CUTAWAY; // cutaway doesn't need GPS! skip wait state
    }
}


void Cutdown::arm(void)
{
    bool gps_locked = false;
    bool loop_toggle = true;

    cutdown_log(LOG_INFO, "Arming: waiting on GPS lock");

    Watchdog.reset();

    // update to OLED info for this state
    oled.clear();
    if (cutdown_config.system_mode == MODE_CUTAWAY) {
        oled.write_line("CUTAWAY GPS WAIT", LINE1);
    } else {
        oled.write_line("CUTDOWN GPS WAIT", LINE1);
    }

    while (!gps_locked) {
        Watchdog.reset();
        cycle_oled_info();
        if (cutdown_config.system_mode == MODE_CUTAWAY) {
            oled.write_line("CUTAWAY GPS WAIT", LINE1);
        } else {
            oled.write_line("CUTDOWN GPS WAIT", LINE1);
        }
        
        if (loop_toggle) {
            adc.thermal_control();
            pressure_log();

            if (!check_batteries(false)) {
                oled.write_line(" !Low Battery!  ", LINE2);
            }
        } else {
            // read the GPS and check for a fix
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
    write_config_to_fee(); // with new GPS location

    cutdown_log(LOG_INFO, "GPS lock acquired, origin set");

    Watchdog.reset();

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

    cutdown_log(LOG_INFO, "Armed in cutdown mode");

    Watchdog.reset();

    // update to oled info for this state
    oled.clear();
    oled.write_line("ARMED:", LINE1);
    oled.write_line("CUTDOWN", LINE2);

    // audible armed warning
    digitalWrite(BUZZER, HIGH);
    wait_timer(2);
    digitalWrite(BUZZER, LOW);

    // wait for a trigger or to be unarmed
    while (!trigger_met) {
        Watchdog.reset();
        cycle_oled_armed();

        if (loop_toggle) {
            adc.thermal_control();
            //pressure_log(); // REMOVED: pressure driver is blocking, not needed for cutdown

            if (!check_batteries(true)) {
                oled.write_line(" !Low Battery!  ", LINE2);
            }
        } else {
            // check the GPS trigger
            if (gps_trigger()) {
                trigger_met = true;
                cutdown_log(LOG_INFO, "GPS trigger met");
            }
        }

        loop_toggle ^= true;

        // wait for 1 second, but count all that pass in case the loop was slow
        decrement_timer(wait_timer(1));

        // check the timer trigger
        if (cutdown_config.primary_timer_remaining <= 0) {
            trigger_met = true;
            cutdown_config.trigger_type = TRIG_TIMER;
            cutdown_log(LOG_INFO, "Timer met: %d", (uint32_t) cutdown_config.primary_timer_remaining);
        }

        // ensure still armed before moving on
        if (!arm_signal()) {
            state = ST_UNARMED;
            return;
        }
    }

    Watchdog.reset();

    state = ST_FIRE;
}


void Cutdown::cutaway(void)
{
    bool loop_toggle = true;
    bool trigger_met = false;

    cutdown_log(LOG_INFO, "Armed in cutaway mode");

    Watchdog.reset();

    // update to oled info for this state
    oled.clear();
    oled.write_line("ARMED:", LINE1);
    oled.write_line("CUTAWAY", LINE2);

    // audible armed warning
    digitalWrite(BUZZER, HIGH);
    wait_timer(2);
    digitalWrite(BUZZER, LOW);

    // wait for a trigger or to be unarmed
    while (!trigger_met) {
        Watchdog.reset();
        cycle_oled_armed();
    
        if (loop_toggle) {
            adc.thermal_control();

            if (!check_batteries(true)) {
                oled.write_line(" !Low Battery!  ", LINE2);
            }
        } else {
            // check for a pressure trigger
            if (pressure_trigger()) {
                trigger_met = true;
                cutdown_log(LOG_INFO, "Pressure trigger met");
            }
        }

        loop_toggle ^= true;

        // wait for 1 second, but count all that pass in case the loop was slow
        decrement_timer(wait_timer(1));

        // check the timer trigger
        if (cutdown_config.primary_timer_remaining <= 0) {
            trigger_met = true;
            cutdown_config.trigger_type = TRIG_TIMER;
            cutdown_log(LOG_INFO, "Timer met: %d", (uint32_t) cutdown_config.primary_timer_remaining);
        }

        // ensure still armed before moving on
        if (!arm_signal()) {
            state = ST_UNARMED;
            return;
        }
    }

    Watchdog.reset();

    state = ST_FIRE;
}


void Cutdown::fire(void)
{
    bool gps_fixed = false;
    float cut_height = 0.0f;

    // insurance in case of a reboot in this loop
    cutdown_config.primary_timer_remaining = 60;
    write_config_to_fee();

    // inform of firing on oled
    oled.clear();
    oled.write_line("Trigger Reached!", LINE1);
    cutdown_log(LOG_INFO, "Fire state!");
    wait_timer(1);

    Watchdog.reset();

    // perform countdown with audible warnings
    oled.write_line("Firing: 5", LINE2);
    digitalWrite(BUZZER, HIGH);
    wait_timer(1);
    oled.write_line("Firing: 4", LINE2);
    digitalWrite(BUZZER, LOW);
    wait_timer(1);
    oled.write_line("Firing: 3", LINE2);
    digitalWrite(BUZZER, HIGH);
    wait_timer(1);
    oled.write_line("Firing: 2", LINE2);
    digitalWrite(BUZZER, LOW);
    wait_timer(1);
    oled.write_line("Firing: 1", LINE2);
    digitalWrite(BUZZER, HIGH);
    wait_timer(1);

    Watchdog.reset();

    // get a final GPS location before cutdown (NOT used for cutaway)
    gps_fixed = (FIX_3D == gps.update_fix());
    cut_height = gps.gps_data.height;

    // fire the primary squib!
    digitalWrite(BUZZER, LOW);
    digitalWrite(SQUIB_PRI_GATE, HIGH);
    delay(1000); // should only need a few ms
    digitalWrite(SQUIB_PRI_GATE, LOW);

    // if that didn't work, try again
    if (check_squib(adc.squib_pri.read())) {
        oled.write_line("Squib failed", LINE2);
        cutdown_log(LOG_ERROR, "Fire 1 failed");
        delay(1000);

        // re-fire the primary squib!
        digitalWrite(SQUIB_PRI_GATE, HIGH);
        delay(1000); // should only need a few ms
        digitalWrite(SQUIB_PRI_GATE, LOW);

        // if that didn't work, fire the backup (which may not be present)
        if (check_squib(adc.squib_pri.read())) {
            cutdown_log(LOG_ERROR, "Fire 2 failed, firing backup");
            oled.write_line("Firing backup", LINE2);
            delay(1000);

            // fire the backup squib!
            digitalWrite(SQUIB_BCK_GATE, HIGH);
            delay(1000); // should only need a few ms
            digitalWrite(SQUIB_BCK_GATE, LOW);

            oled.write_line("Fired backup ", LINE2);
            cutdown_log(LOG_INFO, "Fired backup");
        } else {
            oled.write_line("Squib succeeded", LINE2);
            cutdown_log(LOG_INFO, "Fire success");
        }
    } else {
        oled.write_line("Squib succeeded", LINE2);
        cutdown_log(LOG_INFO, "Fire success");
    }

    Watchdog.reset();

    // if cutdown mode, try to ensure that we're actually falling
    if (cutdown_config.system_mode == MODE_CUTDOWN && gps_fixed) {
        delay(1000); // allow to fall for a bit

        // make sure we aren't still going up
        if (FIX_3D == gps.update_fix() && cut_height < gps.gps_data.height) {
            cutdown_log(LOG_ERROR, "Still ascending after fire!");

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

    Watchdog.reset();

    // now that we've succeeded write to be very large in case of reboot
    cutdown_config.primary_timer_remaining = 64800; // 18h
    write_config_to_fee();

    wait_timer(1);
    state = ST_FINISHED;
}


void Cutdown::finished(void)
{
    Watchdog.reset();

    adc.thermal_control();

    gps_log();
    pressure_log();

    if (!check_batteries(false)) {
        oled.write_line(" !Low Battery!  ", LINE2);
    }

    wait_timer(5);
}


// true if armed, false if not
bool Cutdown::arm_signal(void)
{
    return (digitalRead(SYSTEM_ARM_N) == LOW);
}


// returns false if low battery, powers the system down if both critical and !critical_stage
bool Cutdown::check_batteries(bool critical_stage)
{
    float batt1 = adc.v_batt_pri.read();
    float batt2 = adc.v_batt_bck.read();
    bool critical = false;
    bool nominal = true;
    bool fly_voltage = true;

    cutdown_log(LOG_DEBUG, "b1 ", batt1, ", b2 ", batt2);

    if (batt1 > 5.0f) { // assume not plugged in if <5V
        if (batt1 < cutdown_config.critical_batt_voltage) {
            critical = true;
            nominal = false;
            fly_voltage = false;
        } else if (batt1 < cutdown_config.low_batt_voltage) {
            nominal = false;
            fly_voltage = false;
        } else if (batt1 < cutdown_config.min_fly_voltage) {
            fly_voltage = false;
        }
    }

    if (batt2 > 5.0f) { // assume not plugged in if <5V
        if (batt2 < cutdown_config.critical_batt_voltage) {
            critical &= true; // only critical if both are critical
            nominal = false;
            fly_voltage = false;
        } else if (batt2 < cutdown_config.low_batt_voltage) {
            critical = false;
            nominal = false;
            fly_voltage = false;
        } else if (batt2 < cutdown_config.min_fly_voltage) {
            critical = false;
            fly_voltage = false;
        }
    }

    batt_ok = fly_voltage;

    if (critical) {
        cutdown_log(LOG_ERROR, "Critical battery, power down");
        if (!critical_stage) cutdown_poweroff(); // don't power off if mid-flight
    }

    return nominal;
}


// must be run at 0.5 Hz for burst trigger
bool Cutdown::gps_trigger()
{
    static float oldest_height = 0.0f; // two heights ago
    static float last_height = 0.0f;
    static uint16_t num_sinking = 0;
    float dist = 0.0f;

    // cycle the heights before getting a new one
    oldest_height = last_height;
    last_height = gps.gps_data.height;

    // only check the trigger with a valid, new 3-D fix
    if (FIX_3D != gps.update_fix()) {
        gps_ok = false;
        cutdown_log(LOG_DEBUG, "no gps fix");
        gps.gps_data.height = GPS_INVALID_FLOAT;
        gps.gps_data.displacement = GPS_INVALID_FLOAT;
        return false;
    }

    gps_ok = true;
    cutdown_log(LOG_DEBUG, "lat ", gps.gps_data.latitude, ", long ", gps.gps_data.longitude);
    cutdown_log(LOG_DEBUG, "height ", gps.gps_data.height);

    // see if we've reached the height trigger
    if (gps.gps_data.height >= cutdown_config.trigger_height) {
        cutdown_config.trigger_type = TRIG_GPSH;
        return true;
    }

    // see if we've reached the distance trigger
    dist = gps.distance_from(cutdown_config.origin_lat, cutdown_config.origin_long);
    cutdown_log(LOG_DEBUG, "dist ", dist);
    if (dist >= cutdown_config.trigger_distance) {
        cutdown_config.trigger_type = TRIG_GPSD;
        return true;
    }

    // before checking burst and sink, ensure we're above 3 km and have valid readings
    if (gps.gps_data.height > 3.0 && last_height != GPS_INVALID_FLOAT && oldest_height != GPS_INVALID_FLOAT) {
        // see if we've reached the burst trigger (burst rate multiplied by two since measurements are at 0.5 Hz)
        // heights are multiplied by 1000 to convert from km to m
        if ((oldest_height - last_height)*1000.0 > (2*cutdown_config.burst_fall_rate) 
        && (last_height - gps.gps_data.height)*1000.0 > (2*cutdown_config.burst_fall_rate)) {
            cutdown_config.trigger_type = TRIG_BURST;
            return true;
        }
        
        // check if we're sinking and if the trigger has been met
        if (last_height > gps.gps_data.height) {
            num_sinking++;
            if (num_sinking >= cutdown_config.sink_num_samples) {
                cutdown_config.trigger_type = TRIG_SINK;
                return true;
            }
        } else {
            num_sinking = 0;
        }
    }

    return false;
}


void Cutdown::gps_log()
{
    // only check the trigger with a valid, new 3-D fix
    if (FIX_3D != gps.update_fix()) {
        gps_ok = false;
        cutdown_log(LOG_DEBUG, "no gps fix");
        gps.gps_data.height = GPS_INVALID_FLOAT;
        gps.gps_data.displacement = GPS_INVALID_FLOAT;
        return;
    }

    gps_ok = true;
    cutdown_log(LOG_DEBUG, "lat ", gps.gps_data.latitude, ", long ", gps.gps_data.longitude);
    cutdown_log(LOG_DEBUG, "height ", gps.gps_data.height);
}


bool Cutdown::pressure_trigger()
{
    static float last_pressures[10] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90}; // ensure no false static reading on reboot
    static uint16_t low_altitude_timer = 65000; // will be overwritten on start
    static uint8_t pressure_index = 0;
    static bool low_altitude_started = false;
    float mpl_temp = 0.0f;
    
    float running_avg = 0.0f;

    // get a new pressure reading
    last_pressure = baro.getPressure() / 100.0f;
    cutdown_log(LOG_DEBUG, "press ", last_pressure);
    
    // ok for launch if below ceiling and reasonable value
    pressure_ok = (last_pressure < 1050.0f && last_pressure > cutdown_config.cutaway_ceiling);
    
    // add to the last 10 pressures
    last_pressures[pressure_index] = last_pressure;
    pressure_index = (pressure_index + 1) % 10;

    // log this sensor's temperature reading too
    mpl_temp = baro.getTemperature();
    cutdown_log(LOG_DEBUG, "temp2 ", mpl_temp);

    // check if we're above or below the cutaway ceiling
    if (last_pressure < cutdown_config.cutaway_ceiling) {
        // set that we've reached the ceiling if we haven't yet
        if (!cutdown_config.ceiling_reached) {
            cutdown_config.ceiling_reached = true;
            write_config_to_fee();
            last_fee_write = cutdown_config.primary_timer_remaining;
            cutdown_log(LOG_INFO, "Reached ceiling");
        }
    } else if (cutdown_config.ceiling_reached && last_pressure > cutdown_config.cutaway_ceiling) {
        // run the low altitude timer
        if (!low_altitude_started) {
            low_altitude_started = true;
            low_altitude_timer = cutdown_config.low_alt_timer / 2; // workaround for the fact that this function called at 0.5 Hz
            cutdown_log(LOG_INFO, "Started low pressure timer");
        } else {
            low_altitude_timer -= 1;
            if (low_altitude_timer == 0) {
                cutdown_config.trigger_type = TRIG_LOWA;
                return true;
            }
        }
        
        // get the current running average
        for (int i = 0; i < 10; i++) {
            running_avg += last_pressures[i];
        }
        running_avg /= 10;
    
        // if running average and last pressure are within 1 hPa, trigger
        if (last_pressure - running_avg < 1.0f && last_pressure - running_avg > -1.0f) {
            cutdown_config.trigger_type = TRIG_ALT;
            return true;
        }
    }

    return false;
}


void Cutdown::pressure_log(void)
{
    float mpl_temp = 0.0f;

    last_pressure = baro.getPressure() / 100.0f;
    mpl_temp = baro.getTemperature();
    
    // ok for launch if below ceiling and reasonable value
    pressure_ok = (last_pressure < 1050.0f && last_pressure > cutdown_config.cutaway_ceiling);

    cutdown_log(LOG_DEBUG, "press ", last_pressure);
    cutdown_log(LOG_DEBUG, "temp2 ", mpl_temp);
}


void Cutdown::cycle_oled_info()
{
    String line2_str = "";
    static char line2[17] = "";
    static uint8_t cycle_count = OI_NUM_INFO-1;
    static uint8_t screen_count = 0;
    
    // clear for the new info
    oled.clear();
    
    // cycle through screens
    screen_count = (screen_count + 1) % 4; // keep screen for four cycles
    if (screen_count == 0) {
        // update the number
        cycle_count = (++cycle_count) % OI_NUM_INFO;
    }

    switch (cycle_count) {
    case OI_TPRI:
        if (screen_count == 0) {
            cutdown_log(LOG_DEBUG, "tpri %u", (uint32_t) cutdown_config.primary_timer_remaining);
        }
        snprintf(line2, 17, "PRI: ");
        print_time(line2+5, (uint16_t) cutdown_config.primary_timer_remaining);
        oled.write_line(line2, LINE2);
        break;
    case OI_TBCK:
        backup_timer = attiny.read_timer();
        if (screen_count == 0) {
            cutdown_log(LOG_DEBUG, "tbck %u", (uint32_t) backup_timer);
        }
        snprintf(line2, 17, "BCK: ");
        print_time(line2+5, (uint16_t) backup_timer);
        oled.write_line(line2, LINE2);
        break;
    case OI_TLOWA:
        snprintf(line2, 17, "LOWA: ");
        print_time(line2+6, (uint16_t) cutdown_config.low_alt_timer);
        oled.write_line(line2, LINE2);
        break;
    case OI_SQUIB_PRI:
        if (check_squib(adc.squib_pri.read())) {
            oled.write_line("PRI Squib OK", LINE2);
            if (screen_count == 0) {
                cutdown_log(LOG_DEBUG, "PRI Squib OK");
            }
        } else {
            oled.write_line("PRI squib error!", LINE2);
            if (screen_count == 0) {
                cutdown_log(LOG_DEBUG, "PRI squib error!");
            }
        }
        break;
    case OI_SQUIB_BCK:
        if (check_squib(adc.squib_bck.read())) {
            oled.write_line("BCK Squib OK", LINE2);
            if (screen_count == 0) {
                cutdown_log(LOG_DEBUG, "BCK Squib OK");
            }
        } else {
            oled.write_line("BCK squib error!", LINE2);
            if (screen_count == 0) {
                cutdown_log(LOG_DEBUG, "BCK squib error!");
            }
        }
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
            oled.write_line("Origin not set", LINE2);
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
        line2_str = "P: ";
        line2_str += String(last_pressure);
        line2_str += " hPa";
        oled.write_line((char *) line2_str.c_str(), LINE2);
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


bool Cutdown::check_ok_to_fly()
{
    bool ok_to_fly = true;

    // check squib(s)
    squibs_ok = check_squib(adc.squib_pri.read());
    if (cutdown_config.squib_mode == TWO_SQUIB) {
        squibs_ok &= check_squib(adc.squib_bck.read());
    }
    ok_to_fly &= squibs_ok;

    // check temperature (warmer than 5C below set point)
    temp_ok = calculate_temperature(adc.thermistor.read()) > cutdown_config.temp_set_point - 5.0f;
    ok_to_fly &= temp_ok;

    // check batteries (updated in their own function)
    ok_to_fly &= batt_ok;

    // check timers
    backup_timer = attiny.read_timer();
    tpri_ok = (cutdown_config.primary_timer_remaining > 0 && cutdown_config.primary_timer_remaining <= cutdown_config.primary_timer);
    tbck_ok = (backup_timer > 0 && backup_timer <= cutdown_config.backup_timer);
    ok_to_fly &= tpri_ok & tbck_ok;

    // check GPS or pressure depending on mode (updated in their own functions)
    if (cutdown_config.system_mode == MODE_CUTDOWN) {
        ok_to_fly &= gps_ok;
    } else {
        ok_to_fly &= pressure_ok;
    }

    return ok_to_fly;
}


void Cutdown::cycle_oled_armed()
{
    if (check_ok_to_fly()) {
        if (cutdown_config.system_mode == MODE_CUTDOWN) {
            cutdown_oled_ready();
        } else {
            cutaway_oled_ready();
        }
    } else {
        if (cutdown_config.system_mode == MODE_CUTDOWN) {
            cutdown_oled_fault();
        } else {
            cutaway_oled_fault();
        }
    }
}


void Cutdown::cutdown_oled_ready()
{
    static uint8_t screen = 0;
    static uint8_t cycle = 0;
    String line2_str = "";

    // clear for the new info
    oled.clear();
    oled.write_line("CUTDOWN READY", LINE1);

    cycle = (cycle + 1) % 4;
    if (cycle == 0) {
        screen = (screen + 1) % 2;
    }

    switch (screen) {
    case 0:
        line2_str += "H:";
        line2_str += String(gps.gps_data.height);
        line2_str += " D:";
        line2_str += String(gps.gps_data.displacement);
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case 1:
        line2_str += "T:";
        line2_str += String(cutdown_config.primary_timer_remaining);
        line2_str += " BT:";
        line2_str += String(backup_timer); // updated in check_ok_to_fly()
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    default:
        break;
    }
}


void Cutdown::cutaway_oled_ready()
{
    static uint8_t screen = 0;
    static uint8_t cycle = 0;
    String line2_str = "";

    // clear for the new info
    oled.clear();
    oled.write_line("CUTAWAY READY", LINE1);

    cycle = (cycle + 1) % 4;
    if (cycle == 0) {
        screen = (screen + 1) % 2;
    }

    switch (screen) {
    case 0:
        line2_str += "P:";
        line2_str += String(last_pressure);
        line2_str += " hPa";
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case 1:
        line2_str += "T:";
        line2_str += String(cutdown_config.primary_timer_remaining);
        line2_str += " BT:";
        line2_str += String(backup_timer); // updated in check_ok_to_fly()
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    default:
        break;
    }
}


void Cutdown::cutdown_oled_fault()
{
    static uint8_t screen = 0;
    static uint8_t cycle = 0;
    String line2_str = "";

    // clear for the new info
    oled.clear();
    oled.write_line("CUTDOWN FAULT", LINE1);

    cycle = (cycle + 1) % 4;
    if (cycle == 0) {
        screen = (screen + 1) % 3;
    }

    switch (screen) {
    case 0:
        line2_str += "GPS:";
        line2_str += (gps_ok ? "OK" : "FT");
        line2_str += " SQB:";
        line2_str += (squibs_ok ? "OK" : "FT");
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case 1:
        line2_str += "TPRI:";
        line2_str += (tpri_ok ? "OK" : "FT");
        line2_str += " TBCK:";
        line2_str += (tbck_ok ? "OK" : "FT");
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case 2:
        line2_str += "BATT:";
        line2_str += (batt_ok ? "OK" : "FT");
        line2_str += " TEMP:";
        line2_str += (temp_ok ? "OK" : "FT");
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    default:
        break;
    }
}


void Cutdown::cutaway_oled_fault()
{
    static uint8_t screen = 0;
    static uint8_t cycle = 0;
    String line2_str = "";

    // clear for the new info
    oled.clear();
    oled.write_line("CUTAWAY FAULT", LINE1);

    cycle = (cycle + 1) % 4;
    if (cycle == 0) {
        screen = (screen + 1) % 3;
    }

    switch (screen) {
    case 0:
        line2_str += "PRESS:";
        line2_str += (pressure_ok ? "OK" : "FT");
        line2_str += " SQB:";
        line2_str += (squibs_ok ? "OK" : "FT");
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case 1:
        line2_str += "TPRI:";
        line2_str += (tpri_ok ? "OK" : "FT");
        line2_str += " TBCK:";
        line2_str += (tbck_ok ? "OK" : "FT");
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    case 2:
        line2_str += "BATT:";
        line2_str += (batt_ok ? "OK" : "FT");
        line2_str += " TEMP:";
        line2_str += (temp_ok ? "OK" : "FT");
        oled.write_line((char *) line2_str.c_str(), LINE2);
        break;
    default:
        break;
    }
}


// to avoid roll-over bug with unsigned int, decrement only one second even if multiple pass (should be very low probability)
void Cutdown::decrement_timer(uint8_t seconds)
{
    cutdown_config.primary_timer_remaining -= 1;

    if (last_fee_write - cutdown_config.primary_timer_remaining > 59) {
        last_fee_write = cutdown_config.primary_timer_remaining;
        write_config_to_fee();
    }
}