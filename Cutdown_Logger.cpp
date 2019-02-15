/* Author: Alex St. Clair
 * Filename: Cutdown_Logger.cpp
 * Created: 1-9-19
 * 
 * Implements functions to log debug info over the future radio
 * interface to a separate logging device.
 */

#include "Cutdown_Logger.h"

Uart Logger_Serial(RADIO_SERCOM, RADIO_RX, RADIO_TX, RADIO_RX_PAD, RADIO_TX_PAD);

char logger_buffer[64] = "";
String logger_string = "";

void SERCOM5_Handler()
{
  Logger_Serial.IrqHandler();
}

void logger_init(void)
{
    Logger_Serial.begin(115200);
    pinPeripheral(RADIO_RX, RADIO_RX_MUX);
    pinPeripheral(RADIO_TX, RADIO_TX_MUX);
}

void cutdown_log(char * message)
{
#ifdef LOGGER_SERIAL
    Logger_Serial.print(millis());
    Logger_Serial.print(": ");
    Logger_Serial.println(message);
#endif
#ifdef LOGGER_USB
    Serial.print(millis());
    Serial.print(": ");
    Serial.println(message);
#endif
}

void cutdown_log(char * message, float f1)
{
    logger_string = message;
    logger_string += String(f1, 4);
    cutdown_log((char *) logger_string.c_str());
}

void cutdown_log(char * m1, float f1, char * m2, float f2)
{
    logger_string = m1;
    logger_string += String(f1, 4);
    logger_string += m2;
    logger_string += String(f2, 4);
    cutdown_log((char *) logger_string.c_str());
}

void cutdown_log(char * message, int32_t i1)
{
    snprintf(logger_buffer, 64, message, i1);
    cutdown_log(logger_buffer);
}

void cutdown_log(char * message, uint32_t u1)
{
    snprintf(logger_buffer, 64, message, u1);
    cutdown_log(logger_buffer);
}
