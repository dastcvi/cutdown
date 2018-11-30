/* Author: Alex St. Clair
 * Filename: Cutdown_Pinout.h
 * Created: 11-29-18
 * 
 * Defines pins and other hardware features for the cutdown board
 */

#ifndef CUTDOWN_PINOUT_H
#define CUTDOWN_PINOUT_H

#include "Arduino.h"
#include "SERCOM.h"
#include "UART.h"
#include "SPI.h"
#include "wiring_private.h"

// pinouts from the arduino variant file:
// https://github.com/arduino/ArduinoCore-samd/blob/master/variants/arduino_zero/variant.cpp#L17

// GPIO
#define SQUIB1_GATE      2  /* PA14 */
#define SQUIB2_GATE      5  /* PA15 */
#define BUZZER           11 /* PA16 */
#define POWER_OFF        20 /* PA22 */
#define HEATER_GATE      27 /* PA28 */
#define SYSTEM_ARM       30 /* PB22 */
#define SQUIB_FIRED      31 /* PB23 */

// ADC
#define THERMISTOR       A0 /* PA02 */
#define VMON_SQUIB1      -1 /* PA03 */
#define VMON_SQUIB2      A3 /* PA04 */
#define VMON_3V3A        A4 /* PA05 */
#define VMON_3V3B        -1 /* PA06 */
#define VMON_BATT1       -1 /* PA07 */
#define VMON_BATT2       A5 /* PB02 */
#define VMON_VBATT       -1 /* PB03 */

// OLED
#define OLED_MISO        16 /* PB09, 4.1 */
#define OLED_MOSI        15 /* PB08, 4.0 */
#define OLED_SCK         24 /* PB11, 4.3 */
#define OLED_CS          23 /* PB10, 4.2 */
#define OLED_RES         26 /* PA27 */
#define OLED_TX_PAD      SPI_PAD_0_SCK_3
#define OLED_RX_PAD      SERCOM_RX_PAD_1
#define OLED_SERCOM      (&sercom4)
#define OLED_MISO_MUX    PIO_SERCOM_ALT
#define OLED_MOSI_MUX    PIO_SERCOM_ALT
#define OLED_SCK_MUX     PIO_SERCOM_ALT
#define OLED_SPEED       100000
#define OLED_MODE        SPI_MODE3
#define OLED_ORDER       LSBFIRST

// ATtiny
#define ATTINY_MISO      4  /* PA08, 0.0 */
#define ATTINY_MOSI      1  /* PA10, 0.2 */
#define ATTINY_SCK       0  /* PA11, 0.3 */
#define ATTINY_RST       3  /* PA09, 0.1 */
#define ATTINY_TX_PAD    SPI_PAD_2_SCK_3
#define ATTINY_RX_PAD    SERCOM_RX_PAD_0
#define ATTINY_SERCOM    (&sercom0)
#define ATTINY_MISO_MUX  PIO_SERCOM
#define ATTINY_MOSI_MUX  PIO_SERCOM
#define ATTINY_SCK_MUX   PIO_SERCOM

// Altimeter
#define ALTIM_SDA        22 /* PA12, 2.0 */
#define ALTIM_SCL        38 /* PA13, 2.1 */
#define ALTIM_SERCOM     (&sercom2)
#define ALTIM_SDA_MUX    PIO_SERCOM
#define ALTIM_SCL_MUX    PIO_SERCOM

// GPS
#define GPS_TX           13 /* PA17, 1.1 */
#define GPS_RX           36 /* PA18, 1.2 */
#define GPS_RESET        12 /* PA19 */
#define GPS_TX_PAD       UART_TX_PAD2
#define GPS_RX_PAD       SERCOM_RX_PAD_1
#define GPS_SERCOM       (&sercom1)
#define GPS_TX_MUX       PIO_SERCOM
#define GPS_RX_MUX       PIO_SERCOM

#endif