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
    // start SPI
    oled_spi.begin();
    pinPeripheral(OLED_MISO, OLED_MISO_MUX);
    pinPeripheral(OLED_MOSI, OLED_MOSI_MUX);
    pinPeripheral(OLED_SCK, OLED_SCK_MUX);
    pinMode(OLED_RES_N, OUTPUT);
    digitalWrite(OLED_RES_N, HIGH);

    delay(1000);

    // reset oled
    digitalWrite(OLED_RES_N, LOW);
    delay(100);
    digitalWrite(OLED_RES_N, HIGH);
    delay(10);

    // initialization routine from datasheet
    // http://www.newhavendisplay.com/specs/NHD-0216CW-AW3.pdf
    write_command(0x2A); //function set (extended command set)
    write_command(0x71); //function selection A
    write_data(0x00);    // disable internal VDD regulator (2.8V I/O). data(0x5C) = enable regulator (5V I/O)
    write_command(0x28); //function set (fundamental command set)
    write_command(0x08); //display off, cursor off, blink off
    write_command(0x2A); //function set (extended command set)
    write_command(0x79); //OLED command set enabled
    write_command(0xD5); //set display clock divide ratio/oscillator frequency
    write_command(0x70); //set display clock divide ratio/oscillator frequency
    write_command(0x78); //OLED command set disabled
    write_command(0x08); //extended function set (2-lines)
    write_command(0x06); //COM SEG direction
    write_command(0x72); //function selection B
    write_data(0x00);    //ROM CGRAM selection
    write_command(0x2A); //function set (extended command set)
    write_command(0x79); //OLED command set enabled
    write_command(0xDA); //set SEG pins hardware configuration
    write_command(0x10); //set SEG pins hardware configuration
    write_command(0xDC); //function selection C
    write_command(0x00); //function selection C
    write_command(0x81); //set contrast control
    write_command(0x7F); //set contrast control
    write_command(0xD9); //set phase length
    write_command(0xF1); //set phase length
    write_command(0xDB); //set VCOMH deselect level
    write_command(0x40); //set VCOMH deselect level
    write_command(0x78); //OLED command set disabled
    write_command(0x28); //function set (fundamental command set)
    write_command(0x01); //clear display
    write_command(0x80); //set DDRAM address to 0x00
    write_command(0x0C); //display ON
    delay(100); //delay
}

void Cutdown_OLED::clear(void)
{
    write_command(0x01); //clear display
}

void Cutdown_OLED::write_line(char * data, bool line)
{
    uint8_t count = 0;

    if (line == LINE1) {
        write_command(0x02); //return home
    } else {
        write_command(0xC0); //line 2
    }

    while (count < 16 && data[count] != '\0') {
        write_data(data[count++]);
    }
}

void Cutdown_OLED::write_command(uint8_t cmd)
{
    uint8_t cmd1 = cmd & 0xF; /* lower four bits */
    uint8_t cmd2 = (cmd & 0xF0) >> 4; /* upper four bits */

    oled_spi.beginTransaction(oled_spi_settings);
    oled_spi.transfer(0x1F); /* start byte: R/W=0, D/C=0 */
    oled_spi.transfer(cmd1); /* lower four bits of command */
    oled_spi.transfer(cmd2); /* upper four bits of command */
    oled_spi.endTransaction();
}

void Cutdown_OLED::write_data(uint8_t data)
{
    uint8_t cmd1 = data & 0xF; /* lower four bits */
    uint8_t cmd2 = (data & 0xF0) >> 4; /* upper four bits */

    oled_spi.beginTransaction(oled_spi_settings);
    oled_spi.transfer(0x5F); /* start byte: R/W=0, D/C=1 */
    oled_spi.transfer(cmd1); /* lower four bits of command */
    oled_spi.transfer(cmd2); /* upper four bits of command */
    oled_spi.endTransaction();
}