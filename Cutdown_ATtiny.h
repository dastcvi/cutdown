/* Author: Alex St. Clair
 * Filename: Cutdown_ATtiny.h
 * Created: 11-30-18
 * 
 * Defines a driver to interface with the ATtiny85V backup MCU
 */

#ifndef CUTDOWN_ATTINY_H
#define CUTDOWN_ATTINY_H

#include "Cutdown_Pinout_RevA.h"
#include "Arduino.h"
#include "SERCOM.h"
#include "SPI.h"
#include "wiring_private.h"
#include <stdint.h>

/* this typedef is matched in the backup MCU's driver */
typedef enum {
	CMD_EMPTY = 0x00,
	RD_TIMER_HI = 0x01,
	RD_TIMER_LO = 0x02,
	WR_TIMER_HI = 0x03,
	WR_TIMER_LO = 0x04,
	CMD_ARM     = 0x05, /* software workaround for hardware error (arming signal same as SCK, had initially intended UART, not SPI) */
	CMD_DISARM  = 0x06,
} SPI_Command_t;

class Cutdown_ATtiny {
public:
    Cutdown_ATtiny();
    ~Cutdown_ATtiny() { };
    void init(void);
    uint16_t read_timer(void);
    bool write_timer(uint16_t timer_val);
    void arm(void);
    void disarm(void);
private:
    SPIClass attiny_spi;
    SPISettings attiny_spi_settings;
};

#endif