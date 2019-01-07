/* Author: Alex St. Clair
 * Filename: Cutdown.h
 * Created: 11-29-18
 * 
 * Defines a class that represents the cutdown controller and
 * performs its responsibilities.
 */

#ifndef CUTDOWN_H
#define CUTDOWN_H

#include "Cutdown_Pinout.h"
#include "Cutdown_OLED.h"
#include "Cutdown_ATtiny.h"
#include "Cutdown_ADC.h"

#define DEFAULT_TIMER	        15
#define DEFAULT_BACKUP_TIMER	20

// uncomment if the backup timer should fire regardless of the primary
#define DEMO_BACKUP_TIMER

// indexes for state array (must be in same order as in array!)
typedef enum {
    ST_UNARMED = 0,
    ST_ARMED,
    ST_FIRE,
    ST_FINISHED,
    NUM_STATES
} State_t;

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
    void armed(void);
    void fire(void);
    void finished(void);

    // State variable
    State_t state;

    // State array (items must be in same order as in State_t enum!)
    void (Cutdown::*state_array[NUM_STATES])(void) = {
		&Cutdown::unarmed,
		&Cutdown::armed,
		&Cutdown::fire,
		&Cutdown::finished
	};

    // Timer value
    int32_t cutdown_timer;

    // Driver instances
    Cutdown_OLED oled;
    Cutdown_ATtiny attiny;
    Cutdown_ADC adc;
};

#endif