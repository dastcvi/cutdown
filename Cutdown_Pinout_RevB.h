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
#define SQUIB_PRI_GATE   31 /* PB23 */
#define SQUIB_BCK_GATE   30 /* PB22 */
#define BUZZER           8  /* PA06 */
#define POWER_OFF        15 /* PB08 */
#define HEATER_GATE	     5  /* PA15 */
#define SYSTEM_ARM_N     12 /* PA19, active-low */

// ADC
#define THERMISTOR       19 /* PB02 */
#define VMON_SQUIB_PRI   18 /* PA05 */
#define VMON_SQUIB_BCK   17 /* PA04 */
#define VMON_BATT_PRI    14 /* PA02 */
#define VMON_BATT_BCK    9  /* PA07 */

// OLED
#define OLED_MISO        22 /* PA12, 4.0 */
#define OLED_MOSI        23 /* PB10, 4.2 */
#define OLED_SCK         24 /* PB11, 4.3*/
#define OLED_RES_N       2  /* PA14, active-low */
#define OLED_TX_PAD      (SPI_PAD_2_SCK_3)
#define OLED_RX_PAD      (SERCOM_RX_PAD_0)
#define OLED_SERCOM      (&sercom4)
#define OLED_MISO_MUX    (PIO_SERCOM_ALT)
#define OLED_MOSI_MUX    (PIO_SERCOM_ALT)
#define OLED_SCK_MUX     (PIO_SERCOM_ALT)
#define OLED_SPEED       100000
#define OLED_MODE        (SPI_MODE3)
#define OLED_ORDER       (LSBFIRST)

// ATtiny
#define ATTINY_MISO      4  /* PA08, 0.0 */
#define ATTINY_MOSI      1  /* PA10, 0.2 */
#define ATTINY_SCK       0  /* PA11, 0.3 */
#define ATTINY_RST_N     3  /* PA09, 0.1, active-low */
#define ATTINY_TX_PAD    (SPI_PAD_2_SCK_3)
#define ATTINY_RX_PAD    (SERCOM_RX_PAD_0)
#define ATTINY_SERCOM    (&sercom0)
#define ATTINY_MISO_MUX  (PIO_SERCOM)
#define ATTINY_MOSI_MUX  (PIO_SERCOM)
#define ATTINY_SCK_MUX   (PIO_SERCOM)
#define ATTINY_SPEED     100000
#define ATTINY_MODE      (SPI_MODE3)
#define ATTINY_ORDER     (MSBFIRST)

// Barometric Altimeter
#define BARO_SDA         20 /* PA22, 2.0 */
#define BARO_SCL         21 /* PA23, 2.1 */
#define BARO_SERCOM      (&sercom3)
#define BARO_SDA_MUX     (PIO_SERCOM)
#define BARO_SCL_MUX     (PIO_SERCOM)

// GPS
#define GPS_TX           13 /* PA17, 1.1 */
#define GPS_RX           10 /* PA18, 1.2 */
#define GPS_RESET        12 /* PA19 */
#define GPS_TX_PAD       (UART_TX_PAD_2)
#define GPS_RX_PAD       (SERCOM_RX_PAD_1)
#define GPS_SERCOM       (&sercom1)
#define GPS_TX_MUX       (PIO_SERCOM)
#define GPS_RX_MUX       (PIO_SERCOM)

// Future Radio (also logger)
#define RADIO_TX         6  /* PA20, 5.2 (note: backwards in schematic) */
#define RADIO_RX         7  /* PA21, 5.3 (note: backwards in schematic) */
#define RADIO_TX_PAD     (UART_TX_PAD_2)
#define RADIO_RX_PAD     (SERCOM_RX_PAD_3)
#define RADIO_SERCOM     (&sercom5)
#define RADIO_TX_MUX     (PIO_SERCOM)
#define RADIO_RX_MUX     (PIO_SERCOM)

void cutdown_pinmux(void);

void cutdown_poweroff(void);

#endif