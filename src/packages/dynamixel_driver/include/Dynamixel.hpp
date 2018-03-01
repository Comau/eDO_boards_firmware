/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <stdint.h>
#include <core/hw/GPIO.hpp>

#if !defined(DYNAMIXEL_BUFFER_SIZE)
#define DYNAMIXEL_BUFFER_SIZE 64
#endif

namespace core {
namespace dynamixel_driver {
enum command {
    PING       = 0x01,
    READ_DATA  = 0x02,
    WRITE_DATA = 0x03,
    REG_WRITE  = 0x04,
    ACTION     = 0x05,
    RESET      = 0x06,
    SYNC_WRITE = 0x83
};

enum address {
    MODEL_NUMBER_L         = 0x00,
    MODEL_NUMBER_H         = 0x01,
    FIRMWARE_VERSION       = 0x02,
    ID                     = 0x03,
    BAUDRATE               = 0x04,
    RETURN_DELAY           = 0x05,
    CW_ANGLE_LIMIT_L       = 0x06,
    CW_ANGLE_LIMIT_H       = 0x07,
    CCW_ANGLE_LIMIT_L      = 0x08,
    CCW_ANGLE_LIMIT_H      = 0x09,
    TEMPERATURE_HIGH_LIMIT = 0x0B,
    VOLTAGE_LOW_LIMIT      = 0x0C,
    VOLTAGE_HIGH_LIMIT     = 0x0D,
    MAX_TORQUE_L           = 0x0E,
    MAX_TORQUE_H           = 0x0F,
    STATUS_RETURN_LEVEL    = 0x10,
    ALARM_LED              = 0x11,
    ALARM_SHUTDOWN         = 0x12,
    TORQUE_ENABLE          = 0x18,
    LED                    = 0x19,
    CW_COMPLIANCE_MARGIN   = 0x1A,
    CW_COMPLIANCE_SLOPE    = 0x1B,
    CCW_COMPLIANCE_MARGIN  = 0x1C,
    CCW_COMPLIANCE_SLOPE   = 0x1D,
    GOAL_POSITION_L        = 0x1E,
    GOAL_POSITION_H        = 0x1F,
    MOVING_SPEED_L         = 0x20,
    MOVING_SPEED_H         = 0x21,
    TORQUE_LIMIT_L         = 0x22,
    TORQUE_LIMIT_H         = 0x23,
    PRESENT_POSITION_L     = 0x24,
    PRESENT_POSITION_H     = 0x25,
    PRESENT_SPEED_L        = 0x26,
    PRESENT_SPEED_H        = 0x27,
    PRESENT_LOAD_L         = 0x28,
    PRESENT_LOAD_H         = 0x29,
    PRESENT_VOLTAGE        = 0x2A,
    PRESENT_TEMPERATURE    = 0x2B,
    REGISTERED             = 0x2C,
    MOVING                 = 0x2E,
    LOCK                   = 0x2F,
    PUNCH_L                = 0x30,
    PUNCH_H                = 0x31
};

class Servo;

class Network
{
public:
    Network(
        SerialDriver*  sdp,
        core::hw::Pad& tx_enable
    );

    void
    start(
        uint32_t baudrate
    );

    uint8_t
    scan(
        uint8_t id = 0xFF
    );

    bool
    add(
        Servo* servop
    );

    Servo*
    getNext(
        Servo* servop
    );

    Servo*
    get(
        uint8_t id
    );

    void
    sendCommand(
        uint8_t id,
        command cmd
    );

    uint8_t*
    readData(
        uint8_t id,
        address addr,
        uint8_t len
    );

    void
    writeData(
        uint8_t  id,
        address  addr,
        uint8_t* params,
        uint8_t  len
    );

    bool
    ping(
        uint8_t id
    );

    bool
    ping(
        uint8_t  id,
        uint8_t* error
    );


private:
    bool
    read(
        uint8_t id,
        uint8_t len
    );

    void
    write(
        uint8_t id,
        uint8_t len
    );


private:
    SerialDriver*  _sdp;
    core::hw::Pad& _tx_enable;
    Servo*         _headp;
    uint8_t        _buffer[DYNAMIXEL_BUFFER_SIZE];
};

class Servo
{
    friend class Network;

public:
    Servo(
        Network& net,
        uint8_t  id
    );

    bool
    ping(
        void
    );

    bool
    enable(
        void
    );

    bool
    disable(
        void
    );

    bool
    setLed(
        bool state
    );

    bool
    setPosition(
        float position
    );

    bool
    setSpeed(
        float speed
    );

    bool
    setTorqueLimit(
        float torque
    );

    uint8_t
    getId();

    uint16_t
    getModel();

    uint8_t
    getFirmware();

    uint16_t
    getPosition();

    uint8_t
    getTemperature();

    uint8_t
    getError();

    uint8_t
    getIsMoving();


private:
    Network& _net;
    Servo*   _nextp;
    uint8_t  _id;
};
}
}
