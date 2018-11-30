/* Author: Alex St. Clair
 * Filename: Cutdown_OLED.cpp
 * Created: 11-30-18
 * 
 * Implements a driver for the NHD-0216CW-AW3 2x16 OLED character display
 */

#include "Cutdown_OLED.h"

Cutdown_OLED::Cutdown_OLED() :
    oled_spi(OLED_SERCOM, OLED_MISO, OLED_SCK, OLED_MOSI, OLED_TX_PAD, OLED_RX_PAD),
    oled_spi_settings(OLED_SPEED, OLED_ORDER, OLED_MODE)
{ }

void Cutdown_OLED::init(void)
{
    oled_spi.begin();
}

void Cutdown_OLED::clear(void)
{
    ;
}

void Cutdown_OLED::write_line(char * data, bool line)
{
    ;
}

void Cutdown_OLED::write_command(uint8_t cmd)
{
    ;
}

void Cutdown_OLED::write_data(uint8_t data)
{
    ;
}