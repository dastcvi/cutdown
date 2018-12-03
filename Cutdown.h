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

#define DEFAULT_TIMER	        15
#define DEFAULT_BACKUP_TIMER	20

// uncomment if the backup timer should fire regardless of the primary
#define DEMO_BACKUP_TIMER

typedef enum {
    ST_UNARMED,
    ST_ARMED,
    ST_FIRE,
    ST_FINISHED
} State_t;

class Cutdown {
public:
    Cutdown();
    ~Cutdown() { };
    void init();
    void run();
private:
    void unarmed(void);
    void armed(void);
    void fire(void);
    void finished(void);

    State_t state;

    uint16_t cutdown_timer;

    Cutdown_OLED oled;
    Cutdown_ATtiny attiny;
};

#endif