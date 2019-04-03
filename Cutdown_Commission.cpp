/* Author: Alex St. Clair
 * Filename: Cutdown_Commission.cpp
 * Created: 1-9-19
 * 
 * Used to initially flash the Flash-Emulated EEPROM
 * and test hardware when a board is commissioned.
 */

#include "Cutdown_Commission.h"
#include <Cutdown.h>
#include <Cutdown_Configure.h>

#define SPSC_LATITUDE           40.011
#define SPSC_LONGITUDE          -105.246
#define LARAMIE_LATITUDE        41.312907
#define LARAMIE_LONGITUDE       -105.660194
#define MCMURDO_LATITUDE        -77.848943
#define MCMURDO_LONGITUDE       166.661243
#define PAWNEE_LATITUDE         40.768439
#define PAWNEE_LONGITUDE        -104.638230

// -------- CHANGE CONFIGS HERE --------
#define SERIAL_NUMBER           122

#define SYSTEM_MODE             MODE_CUTDOWN
#define SQUIB_MODE              ONE_SQUIB

#define CUTAWAY_TIMER           18000 // s (5 hours)
#define CUTAWAY_BACKUP_TIMER    18300 // s (5 hours 5 minutes)

#define CUTDOWN_TIMER           7200  // s (>18 hours)
#define CUTDOWN_BACKUP_TIMER    7500  // s (>18 hours)

#define CUTDOWN_HEIGHT          30.0  // km
#define CUTDOWN_DISTANCE        100.0 // km

#define CUTAWAY_CEILING         700   // hPA ~= 10k ft ~= 3 km

#define FLY_VOLTAGE_SETPOINT    11.8  // V
#define LOW_VOLTAGE_SETPOINT    11.1  // V
#define CRIT_VOLTAGE_SETPOINT   10.6  // V

#define DEFAULT_LATITUDE        MCMURDO_LATITUDE
#define DEFAULT_LONGITUDE       MCMURDO_LONGITUDE

#define DEFAULT_TEMP_SETPOINT   5.0 // C
// -------------------------------------

Cutdown commission_cutdown;
Cutdown_Configuration_t config_compare = {0};

bool write_config(void)
{
    bool success = true;

    cutdown_config.config_version = CURRENT_CONFIG_VERSION;
    cutdown_config.serial_number = SERIAL_NUMBER;

    if (SYSTEM_MODE == MODE_CUTAWAY) {
        cutdown_config.system_mode = MODE_CUTAWAY;
        cutdown_config.primary_timer = CUTAWAY_TIMER;
        cutdown_config.primary_timer_remaining = CUTAWAY_TIMER;
        cutdown_config.backup_timer = CUTAWAY_BACKUP_TIMER;
    } else if (SYSTEM_MODE == MODE_CUTDOWN) {
        cutdown_config.system_mode = MODE_CUTDOWN;
        cutdown_config.primary_timer = CUTDOWN_TIMER;
        cutdown_config.primary_timer_remaining = CUTDOWN_TIMER;
        cutdown_config.backup_timer = CUTDOWN_BACKUP_TIMER;
    } else {
        Serial.println("Invalid system mode for config!");
        return false;
    }

    cutdown_config.trigger_height = CUTDOWN_HEIGHT;
    cutdown_config.trigger_distance = CUTDOWN_DISTANCE;
    cutdown_config.cutaway_ceiling = CUTAWAY_CEILING;

    cutdown_config.min_fly_voltage = FLY_VOLTAGE_SETPOINT;
    cutdown_config.low_batt_voltage = LOW_VOLTAGE_SETPOINT;
    cutdown_config.critical_batt_voltage = CRIT_VOLTAGE_SETPOINT;

    cutdown_config.origin_lat = DEFAULT_LATITUDE;
    cutdown_config.origin_long = DEFAULT_LONGITUDE;

    cutdown_config.temp_set_point = DEFAULT_TEMP_SETPOINT;
    cutdown_config.squib_mode = SQUIB_MODE;
    cutdown_config.trigger_type = TRIG_NONE;
    cutdown_config.ceiling_reached = false;

    Serial.println("Writing config to FEE");

    write_config_to_fee();

    config_compare = cutdown_config;

    Serial.println("Reading back config from FEE");

    load_config_from_fee();

    Serial.println("Verifying config read from FEE");

    success &= (config_compare.config_version == cutdown_config.config_version);
    Serial.print("Version: 0x"); Serial.println(cutdown_config.config_version, HEX);

    success &= (config_compare.serial_number == cutdown_config.serial_number);
    Serial.print("Serial #: "); Serial.println(cutdown_config.serial_number);

    success &= (config_compare.primary_timer == cutdown_config.primary_timer);
    Serial.print("Primary timer: "); Serial.println(cutdown_config.primary_timer);

    success &= (config_compare.primary_timer_remaining == cutdown_config.primary_timer_remaining);
    Serial.print("Remaining primary timer: "); Serial.println(cutdown_config.primary_timer_remaining);

    success &= (config_compare.backup_timer == cutdown_config.backup_timer);
    Serial.print("Backup timer: "); Serial.println(cutdown_config.backup_timer);

    success &= (config_compare.trigger_height == cutdown_config.trigger_height);
    Serial.print("Trigger height: "); Serial.println(cutdown_config.trigger_height);

    success &= (config_compare.trigger_distance == cutdown_config.trigger_distance);
    Serial.print("Trigger distance: "); Serial.println(cutdown_config.trigger_distance);

    success &= (config_compare.cutaway_ceiling == cutdown_config.cutaway_ceiling);
    Serial.print("Cutaway ceiling: "); Serial.println(cutdown_config.cutaway_ceiling);

    success &= (config_compare.min_fly_voltage == cutdown_config.min_fly_voltage);
    Serial.print("Min fly voltage: "); Serial.println(cutdown_config.min_fly_voltage);

    success &= (config_compare.low_batt_voltage == cutdown_config.low_batt_voltage);
    Serial.print("Low voltage: "); Serial.println(cutdown_config.low_batt_voltage);

    success &= (config_compare.critical_batt_voltage == cutdown_config.critical_batt_voltage);
    Serial.print("Crit voltage: "); Serial.println(cutdown_config.critical_batt_voltage);

    success &= (config_compare.origin_lat == cutdown_config.origin_lat);
    Serial.print("Latitude: "); Serial.println(cutdown_config.origin_lat);

    success &= (config_compare.origin_long == cutdown_config.origin_long);
    Serial.print("Longitude: "); Serial.println(cutdown_config.origin_long);

    success &= (config_compare.temp_set_point == cutdown_config.temp_set_point);
    Serial.print("Temp set point: "); Serial.println(cutdown_config.temp_set_point);

    success &= (config_compare.ceiling_reached == cutdown_config.ceiling_reached);
    Serial.print("Ceiling reached: "); Serial.println(cutdown_config.ceiling_reached);

    success &= (config_compare.squib_mode == cutdown_config.squib_mode);
    Serial.print("Squibs: "); Serial.println(cutdown_config.squib_mode);

    success &= (config_compare.trigger_type == cutdown_config.trigger_type);
    Serial.print("Trigger result: "); Serial.println(cutdown_config.trigger_type);

    success &= (config_compare.system_mode == cutdown_config.system_mode);
    Serial.print("Mode: ");
    if (cutdown_config.system_mode == MODE_CUTDOWN) {
        Serial.println("CUTDOWN");
    } else if (cutdown_config.system_mode == MODE_CUTAWAY) {
        Serial.println("CUTAWAY");
    } else {
        Serial.println("ERROR!");
    }

    return success;
}

void commission_setup(void)
{
    delay(5000);

    if (load_config_from_fee()) {
        Serial.println("Config already in place, overwriting");
    }

    // write initial configurations to Flash-Emulated EEPROM
    if (write_config()) {
        Serial.println("\nConfig write successful\n");
    } else {
        Serial.println("\nERROR: invalid config\n");
        while(1);
    }

    delay(1000);

    Serial.println("Initializing drivers");

    cutdown_pinmux();
    commission_cutdown.oled.init();
    commission_cutdown.attiny.init();
    commission_cutdown.adc.init();
    commission_cutdown.gps.init();
    Wire.begin();
    commission_cutdown.baro.begin();

    Serial.println("Initialized drivers\n");

    delay(1000);

    Serial.println("Writing OLED test lines");
    commission_cutdown.oled.write_line("abcdefghijklmnop",LINE1);
    commission_cutdown.oled.write_line("qrstuvwxyz123456",LINE2);
    Serial.println("Completed OLED lines\n");

    delay(1000);

    Serial.println("Writing ATtiny timer");
    if (commission_cutdown.attiny.write_timer(cutdown_config.backup_timer)) {
        Serial.println("Successfully wrote ATtiny timer\n");
    } else {
        Serial.println("ERROR: unable to write ATtiny timer\n");
    }

    delay(1000);

    Serial.println("Checking ADC values");
    Serial.print("Primary squib: ");
    Serial.print(commission_cutdown.adc.squib_pri.read());
    if (check_squib(commission_cutdown.adc.squib_pri.check())) {
        Serial.println(" (appears in place)");
    } else {
        Serial.println(" (appears missing)");
    }
    Serial.print("Backup squib: ");
    Serial.print(commission_cutdown.adc.squib_bck.read());
    if (check_squib(commission_cutdown.adc.squib_bck.check())) {
        Serial.println(" (appears in place)");
    } else {
        Serial.println(" (appears missing)");
    }
    Serial.print("Batt pri: ");
    Serial.println(commission_cutdown.adc.v_batt_pri.read());
    Serial.print("Batt sec: ");
    Serial.println(commission_cutdown.adc.v_batt_bck.read());
    Serial.print("Thermistor: ");
    Serial.print(commission_cutdown.adc.thermistor.read());
    Serial.print(" (");
    Serial.print(calculate_temperature(commission_cutdown.adc.thermistor.check()));
    Serial.println(" C)");
    Serial.println("Checked all ADC values\n");

    delay(1000);

    Serial.println("Checking GPS");
    if (commission_cutdown.gps.stop_nmea()) {
        Serial.println("Muted NMEA strings");
    } else {
        Serial.println("ERROR: unable to mute NMEA strings");
    }
    if (commission_cutdown.gps.set_airborne()) {
        Serial.println("Set airborne");
    } else {
        Serial.println("ERROR: unable to set airborne");
    }
    Serial.println("Finished checking GPS\n");

    delay(1000);

    Serial.println("Checking pressure sensor");
    Serial.print("Pressure: ");
    Serial.print(commission_cutdown.baro.getPressure() / 100.0f);
    Serial.println(" hPa");
    Serial.print("Temperature: ");
    Serial.print(commission_cutdown.baro.getTemperature());
    Serial.println(" C");
    Serial.println("Finished checking pressure sensor\n");

    if (commission_cutdown.arm_signal()) {
        Serial.println("System reads armed");
    } else {
        Serial.println("System reads unarmed");
    }
}

void commission_loop(void)
{
    if (commission_cutdown.arm_signal()) {
        Serial.println("System switched to armed");
        delay(10000);
        Serial.println("Firing pri squib in 1 s");
        delay(1000);
        digitalWrite(SQUIB_PRI_GATE, HIGH);
        delay(1000);
        digitalWrite(SQUIB_PRI_GATE, LOW);
        Serial.println("Firing sec squib in 1 s");
        delay(1000);
        digitalWrite(SQUIB_BCK_GATE, HIGH);
        delay(1000);
        digitalWrite(SQUIB_BCK_GATE, LOW);
        Serial.println("Powering off in 10 seconds unless disarmed");
        delay(10000);
        if (commission_cutdown.arm_signal()) {
            cutdown_poweroff();
        }
    }
}