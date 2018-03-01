#pragma once

#include <r2p/Message.hpp>

namespace r2p {
struct SwitchMsg:
    public Message {
    uint32_t state;
}

R2P_PACKED;

struct DynamixelScanMsg:
    public Message {
    uint8_t  id;
    uint8_t  firmware;
    uint16_t model;
}

R2P_PACKED;

struct DynamixelCommandMsg:
    public Message {
    float    position;
    uint8_t  id;
    uint16_t speed;
}

R2P_PACKED;

struct DynamixelStateMsg:
    public Message {
    float   position;
    uint8_t id;
    uint8_t error;
    uint8_t is_moving;
    uint8_t temperature;
}

R2P_PACKED;

struct DynamixelRegisterMsg:
    public Message {
    uint8_t  id;
    uint8_t  address;
    uint8_t  length;
    uint16_t data;
}

R2P_PACKED;
}
