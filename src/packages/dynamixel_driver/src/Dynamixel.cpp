/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include "ch.h"
#include "hal.h"

#include <core/dynamixel_driver/Dynamixel.hpp>

#define READ_BACK_ON_WRITE 0

/*
 * UART driver configuration structure.
 */
static SerialConfig serial_cfg = {
    1000000,
    0,
    0,
    0
};

namespace core {
namespace dynamixel_driver {
Network::Network(
    SerialDriver*  sdp,
    core::hw::Pad& tx_enable
) :
    _sdp(sdp),
    _tx_enable(tx_enable),
    _headp(0)
{}

// baudrate setting is not implemented
void
Network::start(
    uint32_t baudrate
)
{
    (void)baudrate;

    _tx_enable.clear(); // RX MODE
    sdStart(_sdp, &serial_cfg);
}

bool
Network::read(
    uint8_t id,
    uint8_t len
)
{
    uint8_t checksum = 0;

    int n = sdReadTimeout(_sdp, _buffer, len + 4, MS2ST(10));

    if (n != len + 4) {
        return false;
    }

    if (_buffer[2] != id) {
        return false;
    }

    if (_buffer[3] != len) {
        return false;
    }

    for (int i = 2; i < (len + 3); i++) {
        checksum += _buffer[i];
    }

    checksum = ~checksum;

    if (_buffer[len + 3] != checksum) {
        return false;
    }

    return true;
} // Network::read

void
Network::write(
    uint8_t id,
    uint8_t len
)
{
    uint8_t checksum = 0;

    _buffer[0] = 0xFF;  // Start of packet
    _buffer[1] = 0xFF;  // Start of packet
    _buffer[2] = id;  // ID (0xFE -> broadcast)
    _buffer[3] = len; // LEN

    for (int i = 2; i < (len + 3); i++) {
        checksum += _buffer[i];
    }

    _buffer[len + 3] = ~checksum;

    //chThdSleepMilliseconds(1);
    _tx_enable.set();// TX MODE
    chThdSleepMicroseconds(100);//chThdSleepMilliseconds(1);
    sdWrite(_sdp, _buffer, len + 4);
#if READ_BACK_ON_WRITE
    sdRead(_sdp, _buffer, len + 4);
#endif
    chThdSleepMicroseconds(100);// chThdSleepMilliseconds(1);
    _tx_enable.clear();// RX MODE
    // chThdSleepMilliseconds(1);
} // Network::write

void
Network::sendCommand(
    uint8_t id,
    command cmd
)
{
    _buffer[4] = cmd; // INSTRUCTION

    write(id, 2);
}

uint8_t*
Network::readData(
    uint8_t id,
    address addr,
    uint8_t len
)
{
    _buffer[4] = READ_DATA;
    _buffer[5] = addr;
    _buffer[6] = len;

    chSysLock();
    chIQResetI(&(_sdp)->iqueue);
    chSysUnlock();

    write(id, 4);
    if (read(id, len + 2) == false)
      return((uint8_t*)NULL);
    return &_buffer[4];
}

void
Network::writeData(
    uint8_t  id,
    address  addr,
    uint8_t* data,
    uint8_t  len
)
{
    _buffer[4] = WRITE_DATA;
    _buffer[5] = addr;

    for (int i = 6; i < (len + 6); i++) {
        _buffer[i] = *data;
        data++;
    }

    write(id, len + 3); // LEN (3 + N data)
}

bool
Network::ping(
    uint8_t id
)
{
    sendCommand(id, PING);
    return read(id, 2);
}

bool
Network::ping(
    uint8_t  id,
    uint8_t* error
)
{
    sendCommand(id, PING);

    if (read(id, 2)) {
        *error = _buffer[4];
        return true;
    }

    return false;
}

uint8_t
Network::scan(
    uint8_t id
)
{
    do {
        id++;

        if (ping(id)) {
            break;
        }
    } while (id < 0xFE);

    if (id == 0xFE) {
        id = 0xFF;
    }

    return id;
}

bool
Network::add(
    Servo* servo
)
{
    Servo* servop = _headp;

    while (servop) {
        if (servo == servop) {
            return false;
        }

        servop = servop->_nextp;
    }

    servo->_nextp = _headp;
    _headp = servo;

    return true;
}

Servo*
Network::getNext(
    Servo* servo
)
{
    if (servo == 0) {
        return _headp;
    }

    return servo->_nextp;
}

Servo*
Network::get(
    uint8_t id
)
{
    Servo* servop = _headp;

    while (servop) {
        if (id == servop->_id) {
            return servop;
        }

        servop = servop->_nextp;
    }

    return 0;
}

Servo::Servo(
    Network& net,
    uint8_t  id
) :
    _net(net),
    _nextp(0),
    _id(id)
{
    uint8_t status_return_level = 1;

    net.writeData(id, STATUS_RETURN_LEVEL, (uint8_t*)&status_return_level, 1);

    net.add(this);
}

bool
Servo::ping(
    void
)
{
    return _net.ping(_id);
}

bool
Servo::enable(
    void
)
{
    uint8_t data = 1;

    _net.writeData(_id, TORQUE_ENABLE, &data, 1);

    return true;
}

bool
Servo::disable(
    void
)
{
    uint8_t data = 0;

    _net.writeData(_id, TORQUE_ENABLE, &data, 1);

    return true;
}

bool
Servo::setLed(
    bool state
)
{
    _net.writeData(_id, LED, (uint8_t*)&state, 1);

    return true;
}

bool
Servo::setPosition(
  uint16_t goal_position
)
{
  if (goal_position > 1023) {
    goal_position = 1023;
  }

  _net.writeData(_id, GOAL_POSITION_L, (uint8_t*)&goal_position, 2);

  return true;
}

bool
Servo::setSpeed(
    uint16_t current_speed
)
{
  if (current_speed > 1023) {
    current_speed = 0;
  }

  _net.writeData(_id, MOVING_SPEED_L, (uint8_t*)&current_speed, 2);

  return true;
}

bool
Servo::setTorqueLimit(
    uint16_t current_torque
)
{
    if (current_torque > 1023) {
      current_torque = 0;
    }
    _net.writeData(_id, TORQUE_LIMIT_L, (uint8_t*)&current_torque, 2);

    return true;
}

uint8_t
Servo::getId()
{
    return _id;
}

uint16_t
Servo::getModel(bool *pb_state)
{
  uint8_t* tmp = _net.readData(_id, MODEL_NUMBER_L, 2);
  if (tmp == (uint8_t*)NULL)
  {
    *pb_state = false;
    return 0;
  }
  *pb_state = true;
  return tmp[2] << 8 | tmp[1];
}

uint8_t
Servo::getFirmware(bool *pb_state)
{
  uint8_t* tmp = _net.readData(_id, FIRMWARE_VERSION, 1);
  if (tmp == (uint8_t*)NULL)
  {
    *pb_state = false;
    return 0;
  }
  *pb_state = true;
  return tmp[1];
}

uint16_t
Servo::getPosition(bool *pb_state)
{
    uint8_t* tmp = _net.readData(_id, PRESENT_POSITION_L, 2);
    if (tmp == (uint8_t*)NULL)
    {
      *pb_state = false;
      return 0;
    }
    *pb_state = true;
    return tmp[2] << 8 | tmp[1];
}

uint8_t
Servo::getIsMoving(bool *pb_state)
{
    uint8_t* tmp = _net.readData(_id, MOVING, 1);
    if (tmp == (uint8_t*)NULL)
    {
      *pb_state = false;
      return 0;
    }
    *pb_state = true;
    return tmp[1];
}

uint8_t
Servo::getTemperature(bool *pb_state)
{
  uint8_t* tmp = _net.readData(_id, PRESENT_TEMPERATURE, 1);
  if (tmp == (uint8_t*)NULL)
  {
    *pb_state = false;
    return 0;
  }
  *pb_state = true;
  return tmp[1];
}

uint8_t
Servo::getError()
{
    uint8_t error = 0;

    _net.ping(_id, &error);

    return error;
}
/* 180315 GC Added */
uint16_t Servo::getCurrentLoad(bool *pb_state)
{
  uint8_t* tmp = _net.readData(_id, PRESENT_LOAD_L, 2);
  if (tmp == (uint8_t*)NULL)
  {
    *pb_state = false;
    return 0;
  }
  *pb_state = true;
  return tmp[2] << 8 | tmp[1];
}
uint16_t
Servo::getMaxTorque(bool *pb_state)
{
  uint8_t* tmp = _net.readData(_id, MAX_TORQUE_L, 2);
  if (tmp == (uint8_t*)NULL)
  {
    *pb_state = false;
    return 0;
  }
  *pb_state = true;
  return tmp[2] << 8 | tmp[1];
}
uint16_t
Servo::getTorqueLimit(bool *pb_state)
{
  uint8_t* tmp = _net.readData(_id, TORQUE_LIMIT_L, 2);
  if (tmp == (uint8_t*)NULL)
  {
    *pb_state = false;
    return 0;
  }
  *pb_state = true;
  return tmp[2] << 8 | tmp[1];
}

uint16_t
Servo::getAngleLimit(bool *pb_state, bool isCW) /* 180511 GC Added */
{
  uint8_t* tmp;
  
  if (isCW == true)
  {
    tmp = _net.readData(_id, CW_ANGLE_LIMIT_L, 2);
  }
  else
  {
    tmp = _net.readData(_id, CCW_ANGLE_LIMIT_L, 2);
  }
  if (tmp == (uint8_t*)NULL)
  {
    *pb_state = false;
    return 0;
  }
  *pb_state = true;
  return tmp[2] << 8 | tmp[1];
}

bool
Servo::setComplianceMargin(bool isCW, uint8_t data)
{
  if (isCW == true)
  {
    _net.writeData(_id, CW_COMPLIANCE_MARGIN, (uint8_t*)&data, 1);
  }
  else
  {
    _net.writeData(_id, CCW_COMPLIANCE_MARGIN, (uint8_t*)&data, 1);
  }

  return true;
}
bool
Servo::setComplianceSlope(bool isCW, uint8_t data)
{
  if (isCW == true)
  {
    _net.writeData(_id, CW_COMPLIANCE_SLOPE, (uint8_t*)&data, 1);
  }
  else
  {
    _net.writeData(_id, CCW_COMPLIANCE_SLOPE, (uint8_t*)&data, 1);
  }

  return true;
}
bool
Servo::setPunch(
    uint16_t data
)
{
  if (data > 1023) {
    data = 32;
  }
  _net.writeData(_id, PUNCH_L, (uint8_t*)&data, 2);

  return true;
}
bool
Servo::setAngleLimit(bool isCW, uint16_t data)
{
  if (isCW == true)
  {
    _net.writeData(_id, CW_ANGLE_LIMIT_L, (uint8_t*)&data, 2);
  }
  else
  {
    _net.writeData(_id, CCW_ANGLE_LIMIT_L, (uint8_t*)&data, 2);
  }

  return true;
}
bool
Servo::setAlarmShutdown(uint8_t data)
{
  _net.writeData(_id, ALARM_SHUTDOWN, (uint8_t*)&data, 1);
  return true;
}

}
}

