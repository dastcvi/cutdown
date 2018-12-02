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

uint16_t Cutdown_ATtiny::read_timer(void)
{
    uint16_t attiny_timer = 0;

    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_spi.transfer(RD_TIMER_LO);
    attiny_spi.endTransaction();
    
    delay(1); /* give it time to process the last byte */

    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_timer = attiny_spi.transfer(CMD_EMPTY);
    attiny_spi.endTransaction();
    
    delay(1); /* give it time to process the last byte */

    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_spi.transfer(RD_TIMER_HI);
    attiny_spi.endTransaction();
    
    delay(1); /* give it time to process the last byte */

    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_timer |= ((uint16_t) attiny_spi.transfer(CMD_EMPTY) << 8);
    attiny_spi.endTransaction();

    return attiny_timer;
}

bool Cutdown_ATtiny::write_timer(uint16_t timer_val)
{
    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_spi.transfer(WR_TIMER_LO);
    attiny_spi.endTransaction();
    
    delay(1); /* give it time to process the last byte */

    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_spi.transfer(timer_val & 0xff);
    attiny_spi.endTransaction();
    
    delay(1); /* give it time to process the last byte */

    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_spi.transfer(WR_TIMER_HI);
    attiny_spi.endTransaction();
    
    delay(1); /* give it time to process the last byte */

    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_spi.transfer(timer_val >> 8);
    attiny_spi.endTransaction();
    
    delay(1); /* give it time to process the last byte */

    return (timer_val == read_timer());
}

void Cutdown_ATtiny::arm(void)
{
    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_spi.transfer(CMD_ARM);
    attiny_spi.endTransaction();
}

void Cutdown_ATtiny::disarm(void)
{
    attiny_spi.beginTransaction(attiny_spi_settings);
    attiny_spi.transfer(CMD_DISARM);
    attiny_spi.endTransaction();
}