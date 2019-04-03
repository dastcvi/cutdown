/* Author: Alex St. Clair
 * Filename: Cutdown_Configure.h
 * Created: 11-30-18
 * 
 * Defines a driver for setting and maintaining configurations
 */


#include "Cutdown_Configure.h"
#include "Arduino.h"
#include <FlashStorage.h>
#include <string.h>
#include <stdlib.h>


static void read_command(void);
static bool process_command(void);
static int parse_int(void);
static float parse_float(void);

Cutdown_Configuration_t cutdown_config = {0};
static char command_buffer[8] = "";
static char value_buffer[8] = "";
bool update_backup_timer = false;

// Macro from FlashStorage creating flash-emulated EEPROM storage object called config_flasheeprom
FlashStorage(config_flasheeprom, Cutdown_Configuration_t);


bool load_config_from_fee(void)
{
    cutdown_config = config_flasheeprom.read();
    return cutdown_config.config_version == CURRENT_CONFIG_VERSION;
}

void write_config_to_fee(void)
{
    config_flasheeprom.write(cutdown_config);
}

void config_check_serial(void)
{
    if (!Serial.available()) return;

    read_command();

    if (process_command()) {
        write_config_to_fee();
    }
}

static void read_command(void)
{
    uint8_t itr = 0;
    char temp_char = '\0';

    // read in a full command
    while (Serial.available() && itr < 7) {
        temp_char = Serial.read();
        if (temp_char == ',' || temp_char == '\n') break;
        command_buffer[itr++] = temp_char;
    }
    command_buffer[itr] = '\0';

    // check for command w/out value
    if (temp_char == '\n') {
        value_buffer[0] = '\0';
        while (Serial.available()) Serial.read();
        return;
    }

    // read in the value
    itr = 0;
    while (Serial.available() && itr < 7) {
        temp_char = Serial.read();
        if (temp_char == '\n') break;
        value_buffer[itr++] = temp_char;
    }
    value_buffer[itr] = '\0';

    // clear the serial port (synchronization)
    while (Serial.available()) Serial.read();
}

static int parse_int(void)
{
    int value = 0;
    value = atoi(value_buffer);
    if (value <= 0) {
        Serial.println("Invalid config int, must be >0");
    }

    return value;
}

static float parse_float(void)
{
    float value = 0.0f;
    value = (float) atof(value_buffer);
    if (value == 0.0f) {
        Serial.println("Invalid config float, must be valid and !=0!");
    }

    return value;
}

static bool process_command(void)
{
    int int_value = 0;
    float float_value = 0.0f;

    if (0 == (strcmp(command_buffer, CMD_PRIMARY_TIMER))) {
        if ((int_value = parse_int()) <= 0) return false;

        cutdown_config.primary_timer = (uint16_t) int_value;
        cutdown_config.primary_timer_remaining = (uint16_t) int_value;
        Serial.print("Set primary timer (s): ");
        Serial.println(cutdown_config.primary_timer);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_BACKUP_TIMER))) {
        if ((int_value = parse_int()) <= 0) return false;

        cutdown_config.backup_timer = (uint16_t) int_value;
        update_backup_timer = true;
        Serial.print("Set backup timer (s): ");
        Serial.println(cutdown_config.backup_timer);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_TRIGGER_HEIGHT))) {
        if ((float_value = parse_float()) == 0.0f) return false;

        cutdown_config.trigger_height = float_value;
        Serial.print("Set trigger height (km): ");
        Serial.println(cutdown_config.trigger_height);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_TRIGGER_DISTANCE))) {
        if ((float_value = parse_float()) == 0.0f) return false;

        cutdown_config.trigger_distance = float_value;
        Serial.print("Set trigger distance (km): ");
        Serial.println(cutdown_config.trigger_distance);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_CRITICAL_VOLT))) {
        if ((float_value = parse_float()) == 0.0f) return false;

        cutdown_config.critical_batt_voltage = float_value;
        Serial.print("Set critical battery voltage (V): ");
        Serial.println(cutdown_config.critical_batt_voltage);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_LOW_VOLT))) {
        if ((float_value = parse_float()) == 0.0f) return false;

        cutdown_config.low_batt_voltage = float_value;
        Serial.print("Set low battery voltage (V): ");
        Serial.println(cutdown_config.low_batt_voltage);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_FLY_VOLT))) {
        if ((float_value = parse_float()) == 0.0f) return false;

        cutdown_config.min_fly_voltage = float_value;
        Serial.print("Set min fly voltage (V): ");
        Serial.println(cutdown_config.min_fly_voltage);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_SQUIB_MODE))) {
        if ((int_value = parse_int()) <= 0) return false;

        if (int_value > 2) {
            Serial.println("Invalid squib number, must be 1 or 2");
            return false;
        }

        cutdown_config.squib_mode = (Squib_Mode_t) int_value;
        Serial.print("Set squib number to: ");
        Serial.println(cutdown_config.squib_mode);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_TRIGGER_SET))) {
        if ((int_value = parse_int()) <= 0) return false;

        if (int_value > 5) {
            Serial.println("Invalid trigger type, must be 1-5");
            return false;
        }

        cutdown_config.trigger_type = (Trigger_t) int_value;
        Serial.print("Set trigger type to: ");
        Serial.println(cutdown_config.trigger_type);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_SYSTEM_MODE))) {
        if (0 == (strcmp(value_buffer, "cutdown")) || 0 == (strcmp(value_buffer, "CUTDOWN"))) {
            cutdown_config.system_mode = MODE_CUTDOWN;
            Serial.println("Set mode to cutdown");
            return true;
        }
        
        if (0 == (strcmp(value_buffer, "cutaway")) || 0 == (strcmp(value_buffer, "CUTAWAY"))) {
            cutdown_config.system_mode = MODE_CUTAWAY;
            Serial.println("Set mode to cutaway");
            return true;
        }

        Serial.println("Invalid mode selection");
        return false;
    }

    if (0 == (strcmp(command_buffer, CMD_CUTAWAY_CEILING))) {
        if ((float_value = parse_float()) == 0.0f) return false;

        cutdown_config.cutaway_ceiling = float_value;
        Serial.print("Set cutaway ceiling (hPa): ");
        Serial.println(cutdown_config.cutaway_ceiling);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_SERIAL_NUMBER))) {
        if ((int_value = parse_int()) <= 0) return false;

        cutdown_config.serial_number = (uint16_t) int_value;
        Serial.print("Set serial number: ");
        Serial.println(cutdown_config.serial_number);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_LATITUDE))) {
        if ((float_value = parse_float()) == 0.0f) return false;

        cutdown_config.origin_lat = float_value;
        Serial.print("Set origin latitude: ");
        Serial.println(cutdown_config.origin_lat);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_LONGITUDE))) {
        if ((float_value = parse_float()) == 0.0f) return false;

        cutdown_config.origin_long = float_value;
        Serial.print("Set origin longitude: ");
        Serial.println(cutdown_config.origin_long);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_TEMP_SETPOINT))) {
        if ((float_value = parse_float()) == 0.0f) return false;

        cutdown_config.temp_set_point = float_value;
        Serial.print("Set temperature setpoint (C): ");
        Serial.println(cutdown_config.temp_set_point);
        return true;
    }

    if (0 == (strcmp(command_buffer, CMD_MENU))) {
        display_menu();
        return false;
    }

    if (0 == (strcmp(command_buffer, CMD_READ_CONFIGS))) {
        display_fee();
        return false;
    }

    Serial.println("Invalid config command");
    return false;
}

void display_menu(void)
{
    Serial.println("\n--- Config Menu ---\n"); delay(1);
    
    Serial.println("Command format: CMD,value"); delay(1);
    Serial.println("TPRI,(uint16) primary timer [s]"); delay(1);
    Serial.println("TBCK,(uint16) backup timer [s]"); delay(1);
    Serial.println("HEIGHT,(float) cutdown height [km]"); delay(1);
    Serial.println("DIST,(float) cutown distance [km]"); delay(1);
    Serial.println("CEIL,(float) cutaway ceiling [hPa]"); delay(1);
    Serial.println("VCRIT,(float) critical (shutdown) voltage [V]"); delay(1);
    Serial.println("VLOW,(float) low (warning) voltage [V]"); delay(1);
    Serial.println("VFLY,(float) minimum ok to fly voltage [V]"); delay(1);
    Serial.println("MODE,(CUTDOWN/CUTAWAY)"); delay(1);
    Serial.println("SN,(int) serial number"); delay(1);
    Serial.println("LAT,(float) starting latitude (written upon arming)"); delay(1);
    Serial.println("LONG,(float) starting longitude (written upon arming)"); delay(1);
    Serial.println("TEMP,(float) temperature set point [C]"); delay(1);
    Serial.println("SQUIBS,(1/2) number of squibs"); delay(1);
    Serial.println("TRIG,(1-5) trigger result"); delay(1);
    
    Serial.println("\nSpecial commands:"); delay(1);
    Serial.println("MENU (print menu)"); delay(1);
    Serial.println("READ (read+display FEE contents)"); delay(1);
    
    Serial.println("\n-------------------\n");
}

void display_fee(void)
{
    if (!load_config_from_fee()) {
        Serial.println("Invalid config version in FEE! Update any config to re-write");
        return;
    }

    Serial.println("\n--- FEE Contents ---");
    Serial.print("Version: 0x"); Serial.println(cutdown_config.config_version, HEX); delay(1);
    Serial.print("Serial #: "); Serial.println(cutdown_config.serial_number); delay(1);
    Serial.print("Primary timer: "); Serial.println(cutdown_config.primary_timer); delay(1);
    Serial.print("Remaining primary timer: "); Serial.println(cutdown_config.primary_timer_remaining); delay(1);
    Serial.print("Backup timer: "); Serial.println(cutdown_config.backup_timer); delay(1);
    Serial.print("Trigger height: "); Serial.println(cutdown_config.trigger_height); delay(1);
    Serial.print("Trigger distance: "); Serial.println(cutdown_config.trigger_distance); delay(1);
    Serial.print("Cutaway ceiling: "); Serial.println(cutdown_config.cutaway_ceiling); delay(1);
    Serial.print("Fly voltage: "); Serial.println(cutdown_config.min_fly_voltage); delay(1);
    Serial.print("Low voltage: "); Serial.println(cutdown_config.low_batt_voltage); delay(1);
    Serial.print("Crit voltage: "); Serial.println(cutdown_config.critical_batt_voltage); delay(1);
    Serial.print("Latitude: "); Serial.println(cutdown_config.origin_lat); delay(1);
    Serial.print("Longitude: "); Serial.println(cutdown_config.origin_long); delay(1);
    Serial.print("Temp set point: "); Serial.println(cutdown_config.temp_set_point); delay(1);
    Serial.print("Ceiling reached?: "); Serial.println(cutdown_config.ceiling_reached); delay(1);
    Serial.print("Squib mode: "); Serial.println(cutdown_config.squib_mode); delay(1);
    Serial.print("Trigger result: "); Serial.println(cutdown_config.trigger_type); delay(1);
    Serial.print("Mode: ");
    if (cutdown_config.system_mode == MODE_CUTDOWN) {
        Serial.println("CUTDOWN");
    } else if (cutdown_config.system_mode == MODE_CUTAWAY) {
        Serial.println("CUTAWAY");
    } else {
        Serial.println("ERROR!");
    }
    Serial.println("--------------------\n");
}