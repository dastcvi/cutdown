/* Author: Alex St. Clair
 * Filename: Cutdown_ATtiny.h
 * Created: 11-30-18
 * 
 * Defines a driver to interface with the ATtiny85V backup MCU
 */

#ifndef CUTDOWN_ATTINY_H
#define CUTDOWN_ATTINY_H

#include "Cutdown_Pinout.h"
#include "Arduino.h"
#include "SERCOM.h"
#include "SPI.h"
#include "wiring_private.h"
#include <stdint.h>


class Cutdown_ATtiny {
public:
    Cutdown_ATtiny();
    ~Cutdown_ATtiny() { };
    void init(void);
    void write_byte(uint8_t data);
private:
    SPIClass attiny_spi;
    SPISettings attiny_spi_settings;
};

#endif