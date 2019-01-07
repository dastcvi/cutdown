/* Author: Alex St. Clair
 * Filename: Cutdown_Configure.h
 * Created: 11-30-18
 * 
 * Defines a driver for setting and maintaining configurations
 */

#include "Cutdown_Configure.h"
#include "Arduino.h"
#include <string.h>
#include <stdlib.h>

static void read_command(void);
static void process_command(void);
static int parse_int(void);
static float parse_float(void);

Cutdown_Configuration_t cutdown_config = {0};
static char command_buffer[8] = "";
static char value_buffer[8] = "";

void config_init(void)
{
    cutdown_config.primary_timer = DEFAULT_TIMER;
    cutdown_config.backup_timer = DEFAULT_BACKUP_TIMER;
    cutdown_config.critical_batt_voltage = DEFAULT_CRITICAL_VOLT;
    cutdown_config.low_batt_voltage = DEFAULT_LOW_VOLT;
}

void config_update(void)
{
    if (!Serial.available()) return;

    read_command();

    process_command();
}

static void read_command(void)
{
    uint8_t itr = 0;
    char temp_char = '\0';

    // read in a full command
    while (Serial.available() && itr < 7) {
        temp_char = Serial.read();
        if (temp_char == ',') break;
        command_buffer[itr++] = temp_char;
    }
    command_buffer[itr] = '\0';

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
    if (value <= 0.0f) {
        Serial.println("Invalid config float, must be >0");
    }

    return value;
}

static void process_command(void)
{
    int int_value = 0;
    float float_value = 0.0f;

    if (0 == (strcmp(command_buffer, PRIMARY_TIMER))) {
        if ((int_value = parse_int()) <= 0) return;

        cutdown_config.primary_timer = (uint16_t) int_value;
        Serial.print("Set primary timer: ");
        Serial.println(cutdown_config.primary_timer);
        return;
    }

    if (0 == (strcmp(command_buffer, BACKUP_TIMER))) {
        if ((int_value = parse_int()) <= 0) return;

        cutdown_config.backup_timer = (uint16_t) int_value;
        Serial.print("Set backup timer: ");
        Serial.println(cutdown_config.backup_timer);
        return;
    }

    if (0 == (strcmp(command_buffer, CRITICAL_VOLT))) {
        if ((float_value = parse_float()) <= 0.0f) return;

        cutdown_config.critical_batt_voltage = float_value;
        Serial.print("Set critical battery voltage: ");
        Serial.println(cutdown_config.critical_batt_voltage);
        return;
    }

    if (0 == (strcmp(command_buffer, LOW_VOLT))) {
        if ((float_value = parse_float()) <= 0.0f) return;

        cutdown_config.low_batt_voltage = float_value;
        Serial.print("Set low battery voltage: ");
        Serial.println(cutdown_config.low_batt_voltage);
        return;
    }

    Serial.println("Invalid config command");
}