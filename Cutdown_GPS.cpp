/* Author: Alex St. Clair
 * Filename: Cutdown_GPS.cpp
 * Created: 12-17-18
 * 
 * Implements a driver to interface with the uBlox MAX-M8 GPS
 * 
 * This driver implements a polling interface using the uBlox UBX protocol in lieu of
 * reading NMEA strings output at a standard interval. This minimizes bus traffic (only
 * one binary UBX message is necessary) and removes the need to asynchronously buffer
 * NMEA strings for parsing.
 */

#include "Cutdown_GPS.h"
#include <math.h>

Uart GPS_Serial(GPS_SERCOM, GPS_RX, GPS_TX, GPS_RX_PAD, GPS_TX_PAD);

// messages to send to the receiver
uint8_t UBX_Poll_PVT[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
uint8_t UBX_Stop_GGA[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F};
uint8_t UBX_Stop_GLL[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
uint8_t UBX_Stop_GSA[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};
uint8_t UBX_Stop_GSV[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
uint8_t UBX_Stop_RMC[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};
uint8_t UBX_Stop_VTG[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};

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

bool Cutdown_GPS::stop_nmea(void)
{
    bool success = true;

    transmit_ubx(UBX_Stop_GGA, sizeof(UBX_Stop_GGA));
    success &= verify_ack();

    transmit_ubx(UBX_Stop_GLL, sizeof(UBX_Stop_GLL));
    success &= verify_ack();

    transmit_ubx(UBX_Stop_GSA, sizeof(UBX_Stop_GSA));
    success &= verify_ack();

    transmit_ubx(UBX_Stop_GSV, sizeof(UBX_Stop_GSV));
    success &= verify_ack();

    transmit_ubx(UBX_Stop_RMC, sizeof(UBX_Stop_RMC));
    success &= verify_ack();

    transmit_ubx(UBX_Stop_VTG, sizeof(UBX_Stop_VTG));
    success &= verify_ack();

    return success;
}

GPS_FIX_TYPE_t Cutdown_GPS::update_fix(void)
{
    static UBX_NAV_PVT_t message = {0};
    unsigned long start_time = millis();
    uint8_t bytes_read = 0;

    // transmit the message to request the info
    transmit_ubx(UBX_Poll_PVT, sizeof(UBX_Poll_PVT));

    // wait for sync bytes with timeout
    if (!get_sync_bytes(start_time)) return FIX_NONE;

    // read the message
    while ((bytes_read < sizeof(UBX_NAV_PVT_t)) && (millis() - start_time < GPS_MSG_TIMEOUT)) {
        if (GPS_Serial.available()) {
            message.buffer[bytes_read++] = GPS_Serial.read();
        }
    }

    // ensure we haven't timed out
    if (bytes_read != sizeof(UBX_NAV_PVT_t)) return FIX_NONE;

    // get and verify the checksum
    if (!verify_checksum(start_time, message.buffer, sizeof(message))) return FIX_NONE;

    // verify the message
    if (message.fields.msg_class != NAV_PVT_CLASS) return FIX_NONE;
    if (message.fields.msg_id != NAV_PVT_ID) return FIX_NONE;
    if (message.fields.msg_length != sizeof(UBX_NAV_PVT_t) - 4) return FIX_NONE;

    gps_data.num_satellites = message.fields.num_satellites;
    if (message.fields.fix_type == FIX_3D) {
        gps_data.longitude = (float) message.fields.longitude / (10000000.0f); // undo chip scaling by 1e-7 to get degrees
        gps_data.latitude = (float) message.fields.latitude / (10000000.0f); // undo chip scaling by 1e-7 to get degrees
        gps_data.height = (float) message.fields.sealevel_height / (1000000.0f); // convert mm to km
        gps_data.fix_type = FIX_3D;
    }

    return (GPS_FIX_TYPE_t) message.fields.fix_type;
}

// use the Haversine formula to calculate the current distance from the given coords
// based on: https://rosettacode.org/wiki/Haversine_formula#C
// calculation time is ~1ms
float Cutdown_GPS::distance_from(double lat_origin, double long_origin)
{
    double dx, dy, dz;
    double curr_lat = gps_data.latitude;
    double curr_long = gps_data.longitude;

    curr_long -= long_origin;
    curr_long *= TO_RAD;
    lat_origin *= TO_RAD;
    curr_lat *= TO_RAD;

    dz = sin(lat_origin) - sin(curr_lat);
    dx = cos(curr_long) * cos(lat_origin) - cos(curr_lat);
    dy = sin(curr_long) * cos(lat_origin);

    gps_data.displacement = (float) (asin(sqrt(dx*dx + dy*dy + dz*dz) / 2) * 2 * R_EARTH);

    return gps_data.displacement;
}

void Cutdown_GPS::transmit_ubx(uint8_t * buffer, uint16_t length)
{
    for (int i = 0; i < length; i++) {
        GPS_Serial.write(buffer[i]);
    }
}

bool Cutdown_GPS::verify_ack(void)
{
    static UBX_ACK_NAK_t message = {0};
    unsigned long start_time = millis();
    uint8_t bytes_read = 0;
    
    // wait for sync bytes with timeout
    if (!get_sync_bytes(start_time)) return false;

    // read the message
    while ((bytes_read < sizeof(UBX_ACK_NAK_t)) && (millis() - start_time < GPS_MSG_TIMEOUT)) {
        if (GPS_Serial.available()) {
            message.buffer[bytes_read++] = GPS_Serial.read();
        }
    }

    // ensure we haven't timed out
    if (bytes_read != sizeof(UBX_ACK_NAK_t)) return false;

    // get and verify the checksum
    if (!verify_checksum(start_time, message.buffer, sizeof(message))) return false;

    // verify the ack
    if (message.fields.msg_class != ACK_NAK_CLASS) return false;
    if (message.fields.msg_id != ACK_ID) return false;
    if (message.fields.msg_length != sizeof(UBX_ACK_NAK_t) - 4) return false;
    return true;
}

bool Cutdown_GPS::verify_checksum(unsigned long start_time, uint8_t * buffer, uint16_t length)
{
    uint8_t cka = 0;
    uint8_t ckb = 0;
    uint8_t cka_calc = 0;
    uint8_t ckb_calc = 0;

    // get and verify the checksum
    if (!get_check_bytes(start_time, &cka, &ckb)) return false;
    calculate_checksum(buffer, length, &cka_calc, &ckb_calc);
    if ((cka != cka_calc) || (ckb != ckb_calc)) return false;
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
    while ((temp_byte != SYNC1) && (millis() - start_time < GPS_MSG_TIMEOUT)) {
        if (GPS_Serial.available()) {
            temp_byte = GPS_Serial.read();
        }
    }

    if (temp_byte != SYNC1) return false; // timeout

    // ensure the second sync byte
    while (millis() - start_time < GPS_MSG_TIMEOUT) {
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
    while (millis() - start_time < GPS_MSG_TIMEOUT) {
        if (GPS_Serial.available()) {
            *cka = GPS_Serial.read();
            byte_received = true;
            break;
        }
    }

    if (!byte_received) return false; // timeout

    // get the second checksum byte
    byte_received = false;
    while (millis() - start_time < GPS_MSG_TIMEOUT) {
        if (GPS_Serial.available()) {
            *ckb = GPS_Serial.read();
            byte_received = true;
            break;
        }
    }

    return byte_received;
}