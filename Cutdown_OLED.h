/* Author: Alex St. Clair
 * Filename: Cutdown_OLED.h
 * Created: 11-30-18
 *
 * Defines a driver for the NHD-0216CW-AW3 2x16 OLED character display
 */

#ifndef CUTDOWN_OLED_H
#define CUTDOWN_OLED_H

#include "Cutdown_Pinout.h"
#include "Arduino.h"
#include "SERCOM.h"
#include "SPI.h"
#include "wiring_private.h"
#include <stdint.h>

#define LINE1	true
#define LINE2	false

class Cutdown_OLED {
public:
    Cutdown_OLED();
    ~Cutdown_OLED() { };
    void init(void);
    void clear(void);
    void write_line(char * data, bool line);
private:
    void write_command(uint8_t cmd);
    void write_data(uint8_t data);

    SPIClass oled_spi;
    SPISettings oled_spi_settings;
};

#endif