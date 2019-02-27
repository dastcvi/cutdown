/* Author: Alex St. Clair
 * Filename: Cutdown.h
 * Created: 11-29-18
 * 
 * Defines a class that represents the cutdown controller and
 * performs its responsibilities.
 */

#ifndef CUTDOWN_H
#define CUTDOWN_H

#include "Cutdown_Pinout_RevA.h"
#include "Cutdown_OLED.h"
#include "Cutdown_ATtiny.h"
#include "Cutdown_ADC.h"
#include "Cutdown_GPS.h"
#include <Wire.h>
#include "wiring_private.h"
//#include <Adafruit_MPL3115A2.h>

#define WAIT_FOR_GPS
#define GPS_WAIT_TIME   180 // s (3 min)

// uncomment if the backup timer should fire regardless of the primary
//#define DEMO_BACKUP_TIMER

// indices for state array (must be in same order as in array!)
typedef enum {
    ST_UNARMED = 0,
    ST_ARM,
    ST_CUTDOWN,
    ST_CUTAWAY,
    ST_FIRE,
    ST_FINISHED,
    NUM_STATES
} State_t;

// indices for the different OLED information types
typedef enum : uint8_t {
    OI_TPRI = 0,
    OI_TBCK,
    OI_SQUIB_PRI,
    OI_SQUIB_BCK,
    OI_LASTGPS,
    OI_SET_DISTANCE,
    OI_DISTANCE,
    OI_SET_HEIGHT,
    OI_HEIGHT,
    OI_PRESSURE,
    OI_BATT1,
    OI_BATT2,
    OI_TEMP,
    OI_NUM_INFO
} OLED_Info_t;

class Cutdown {
public:
    // Constructor/Destructor
    Cutdown();
    ~Cutdown() { };

    // Public interface
    void init();
    void run();
private:
    // State functions
    void unarmed(void);
    void arm(void);
    void cutdown(void);
    void cutaway(void);
    void fire(void);
    void finished(void);

    // State variable
    State_t state;

    // State array (items must be in same order as in State_t enum!)
    void (Cutdown::*state_array[NUM_STATES])(void) = {
		&Cutdown::unarmed,
        &Cutdown::arm,
        &Cutdown::cutdown,
        &Cutdown::cutaway,
		&Cutdown::fire,
		&Cutdown::finished
	};

    // Timer value
    int32_t cutdown_timer;

    // Pressure holder
    float last_pressure;

    // Driver instances
    Cutdown_OLED oled;
    Cutdown_ATtiny attiny;
    Cutdown_ADC adc;
    Cutdown_GPS gps;
    //Adafruit_MPL3115A2 baro;

    // Helper functions
    inline bool arm_signal();
    bool check_batteries(bool critical_stage);
    bool gps_trigger();
    void gps_log();
    bool pressure_trigger();
    void pressure_log();
    void cycle_oled_info(bool cycle);
};

#endif