/* Author: Alex St. Clair
 * Filename: Cutdown_GPS.cpp
 * Created: 12-17-18
 * 
 * Implements a driver to interface with the uBlox MAX-M8 GPS
 */

#include "Cutdown_GPS.h"
#include "Cutdown_GPS_Messages.h"

Uart GPS_Serial(GPS_SERCOM, GPS_RX, GPS_TX, GPS_RX_PAD, GPS_TX_PAD);

void SERCOM1_Handler()
{
  GPS_Serial.IrqHandler();
}

void Cutdown_GPS::init(void)
{
    GPS_Serial.begin(9600);
    pinPeripheral(GPS_RX, GPS_RX_MUX);
    pinPeripheral(GPS_TX, GPS_TX_MUX);
}

void Cutdown_GPS::test(void)
{
    // transmit_ubx(UBX_Stop_GGA, sizeof(UBX_Stop_GGA));
    // if (verify_ack()) {
    //     Serial.println("Verified ACK");
    // } else {
    //     Serial.println("Error with ACK");
    // }

    // transmit_ubx(UBX_Stop_GLL, sizeof(UBX_Stop_GLL));
    // if (verify_ack()) {
    //     Serial.println("Verified ACK");
    // } else {
    //     Serial.println("Error with ACK");
    // }

    // transmit_ubx(UBX_Stop_GSA, sizeof(UBX_Stop_GSA));
    // if (verify_ack()) {
    //     Serial.println("Verified ACK");
    // } else {
    //     Serial.println("Error with ACK");
    // }

    // transmit_ubx(UBX_Stop_GSV, sizeof(UBX_Stop_GSV));
    // if (verify_ack()) {
    //     Serial.println("Verified ACK");
    // } else {
    //     Serial.println("Error with ACK");
    // }

    // transmit_ubx(UBX_Stop_RMC, sizeof(UBX_Stop_RMC));
    // if (verify_ack()) {
    //     Serial.println("Verified ACK");
    // } else {
    //     Serial.println("Error with ACK");
    // }

    // transmit_ubx(UBX_Stop_VTG, sizeof(UBX_Stop_VTG));
    // if (verify_ack()) {
    //     Serial.println("Verified ACK");
    // } else {
    //     Serial.println("Error with ACK");
    // }

    // delay(5000);

    while(1) {
        while (GPS_Serial.available()) {
            Serial.print((char) GPS_Serial.read());
        }
    }
}

void Cutdown_GPS::transmit_ubx(uint8_t * buffer, uint16_t length)
{
    for (int i = 0; i < length; i++) {
        GPS_Serial.write(buffer[i]);
    }
}

bool Cutdown_GPS::verify_ack(void)
{
    UBX_ACK_NAK_t message = {0};
    unsigned long start_time = millis();
    uint8_t bytes_read = 0;
    uint8_t cka = 0;
    uint8_t ckb = 0;
    uint8_t cka_calc = 0;
    uint8_t ckb_calc = 0;
    
    // wait for sync bytes with timeout
    if (!get_sync_bytes(start_time)) return false;

    // read the message
    while ((bytes_read < sizeof(UBX_ACK_NAK_t)) && (millis() - start_time < ACK_TIMEOUT)) {
        if (GPS_Serial.available()) {
            message.buffer[bytes_read++] = GPS_Serial.read();
        }
    }

    // get and verify the checksum
    if (!get_check_bytes(start_time, &cka, &ckb)) return false;
    calculate_checksum(message.buffer, sizeof(message), &cka_calc, &ckb_calc);
    if ((cka != cka_calc) || (ckb != ckb_calc)) return false;

    // verify the ack
    if (message.fields.msg_class != ACK_NAK_CLASS) return false;
    if (message.fields.msg_id != ACK_ID) return false;
    return true;
}

void Cutdown_GPS::calculate_checksum(uint8_t * buffer, uint16_t length, uint8_t * cka, uint8_t * ckb)
{
    int i = 0;
    *cka = 0;
    *ckb = 0;

    for (i = 0; i < length; i++) {
        *cka = *cka + buffer[i];
        *ckb = *ckb + *cka;
    }
}

bool Cutdown_GPS::get_sync_bytes(unsigned long start_time)
{
    uint8_t temp_byte = 0;

    // wait for the first sync byte
    while ((temp_byte != SYNC1) && (millis() - start_time < ACK_TIMEOUT)) {
        if (GPS_Serial.available()) {
            temp_byte = GPS_Serial.read();
        }
    }

    if (temp_byte != SYNC1) return false; // timeout

    // ensure the second sync byte
    while (millis() - start_time < ACK_TIMEOUT) {
        if (GPS_Serial.available()) {
            temp_byte = GPS_Serial.read();
            break;
        }
    }

    return (temp_byte == SYNC2);
}

bool Cutdown_GPS::get_check_bytes(unsigned long start_time, uint8_t * cka, uint8_t * ckb)
{
    bool byte_received = false;
    
    // get the first checksum byte
    while (millis() - start_time < ACK_TIMEOUT) {
        if (GPS_Serial.available()) {
            *cka = GPS_Serial.read();
            byte_received = true;
            break;
        }
    }

    if (!byte_received) return false; // timeout

    // get the second checksum byte
    byte_received = false;
    while (millis() - start_time < ACK_TIMEOUT) {
        if (GPS_Serial.available()) {
            *ckb = GPS_Serial.read();
            byte_received = true;
            break;
        }
    }

    return byte_received;
}