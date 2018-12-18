/* Author: Alex St. Clair
 * Filename: Cutdown_GPS_Messages.h
 * Created: 12-18-18
 * 
 * Defines uBlox MAX-M8 GPS messages (UBX protocol), see the interface manual:
 * https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf
 */

#ifndef CUTDOWN_GPS_MESSAGES_H
#define CUTDOWN_GPS_MESSAGES_H

#include <stdint.h>

/*********************************************************************
 * messages to receive from u-blox (without sync/checksum bytes)
 *********************************************************************/

#define ACK_NAK_CLASS	0x05
#define ACK_ID          0x01
#define NAK_ID          0x00

typedef union UBX_ACK_NAK {
    uint8_t buffer[6];
    struct fields {
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t msg_length;
        uint8_t ack_class;
        uint8_t ack_id;
    } fields;
} UBX_ACK_NAK_t;

#define NAV_PVT_CLASS	0x01
#define NAV_PVT_ID      0x07

// u-blox position/velocity/time message (does not include sync/checksum bytes)
// SAMD21 and GPS are little-endian, so a union of the struct with the raw buffer works
typedef union UBX_NAV_PVT {
    uint8_t buffer[96];
    struct fields {
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t msg_length;
        uint32_t time_of_week;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t valid_flags;
        uint32_t time_accuracy;
        int32_t nanosecond;
        uint8_t fix_type;
        uint8_t flags;
        uint8_t flags2;
        uint8_t num_satellites;
        int32_t longitude;
        int32_t latitude;
        int32_t ellipsoid_height;
        int32_t sealevel_height;
        uint32_t horizontal_accuracy;
        uint32_t vertical_accuracy;
        int32_t north_velocity;
        int32_t east_velocity;
        int32_t down_velocity;
        int32_t ground_speed;
        int32_t heading;
        uint32_t speed_accuracy;
        uint32_t heading_accuracy;
        uint16_t dop_position;
        uint8_t reserved[6];
        int32_t vehicle_heading;
        int16_t magnetic_declination;
        uint16_t magnetic_accuracy;
    } fields;
} UBX_NAV_PVT_t;

// for the PVT fix_type field
typedef enum GPS_FIX_TYPE : uint8_t {
    FIX_NONE = 0,
    FIX_DEADRECKON = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GNSS = 4,
    FIX_TIMEONLY = 5
} GPS_FIX_TYPE_t;

#endif