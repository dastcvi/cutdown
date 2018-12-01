/* Author: Alex St. Clair
 * Filename: Cutdown_ATtiny.cpp
 * Created: 11-30-18
 * 
 * Defines a driver to interface with the ATtiny85V backup MCU
 */

#include "Cutdown_ATtiny.h"

Cutdown_ATtiny::Cutdown_ATtiny() : 
    attiny_spi(ATTINY_SERCOM, ATTINY_MISO, ATTINY_SCK, ATTINY_MOSI, ATTINY_TX_PAD, ATTINY_RX_PAD),
    attiny_spi_settings(ATTINY_SPEED, ATTINY_ORDER, ATTINY_MODE)
{ }

void Cutdown_ATtiny::init()
{
    // start SPI
    attiny_spi.begin();
    pinPeripheral(ATTINY_MISO, ATTINY_MISO_MUX);
    pinPeripheral(ATTINY_MOSI, ATTINY_MOSI_MUX);
    pinPeripheral(ATTINY_SCK, ATTINY_SCK_MUX);
    pinMode(ATTINY_RST, OUTPUT);
    digitalWrite(ATTINY_RST, HIGH); /* keep out of reset */
}

void Cutdown_ATtiny::write_byte(uint8_t data)
{
    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_spi.transfer(data);
    attiny_spi.endTransaction();
}