/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <core/QEI_driver/QEI.hpp>
#include <Module.hpp>

#include <core/utils/math/Constants.hpp>

namespace core {
namespace QEI_driver {
QEI::QEI(
    core::hw::QEI& qei
) : _qei(qei) {}

QEI::~QEI()
{}

bool
QEI::probe()
{
    return true;
}

QEI_Delta::QEI_Delta(
    const char* name,
    QEI&        device
) : CoreConfigurable<QEI_DeltaConfiguration>::CoreConfigurable(name),
    _timestamp(core::os::Time::IMMEDIATE),
    _device(device)
{}

QEI_Delta::~QEI_Delta()
{}

bool
QEI_Delta::init()
{
    return true;
}

bool
QEI_Delta::configure()
{
    return isConfigured();
}

bool
QEI_Delta::start()
{
    CORE_ASSERT(isConfigured());

    _device._qei.enable();
    _timestamp = core::os::Time::IMMEDIATE;
    return true;
}

bool
QEI_Delta::stop()
{
    _device._qei.disable();
    return true;
}

bool
QEI_Delta::update()
{
    return true;
}       // QEI_Delta::update

void
QEI_Delta::get(
    DataType& data
)
{
    data = (360.0f * (float)_device._qei.getDelta()) / (float)configuration().ticks;

    if ((uint8_t)configuration().invert) {
        data = -data;
    }
}
/*
void
QEI_Delta::getspeed(
    DataType& data
)
{
    data = (360.0f * (float)_device._qei.getDelta()) / (float)configuration().ticks;

    if ((uint8_t)configuration().invert) {
        data = -data;
    }
}*/

bool
QEI_Delta::waitUntilReady()
{
    // FIXME !!!!!
    /*
       if (_timestamp != core::os::Time::IMMEDIATE) {
       core::os::Thread::sleep_until(_timestamp + core::os::Time::ms(configuration.period));
       }

       _timestamp = core::os::Time::now();
     */

    chThdSleepMilliseconds((uint16_t)configuration().period);

    return true;
}

QEI_Position::QEI_Position(
    const char* name,
    QEI&        device
) : CoreConfigurable<QEI_PositionConfiguration>::CoreConfigurable(name),
    _timestamp(core::os::Time::IMMEDIATE),
    _device(device)
{}

QEI_Position::~QEI_Position()
{}

bool
QEI_Position::init()
{
    return true;
}

bool
QEI_Position::configure()
{
    return isConfigured();
}

bool
QEI_Position::start()
{
    CORE_ASSERT(isConfigured());

    _device._qei.enable();
    _timestamp = core::os::Time::IMMEDIATE;
    return true;
}

bool
QEI_Position::stop()
{
    _device._qei.disable();
    return true;
}

bool
QEI_Position::update()
{
    return true;
}

void
QEI_Position::get(
    DataType& data
)
{
    data = (_device._qei.getCount() / (float)configuration().ticks) * (2.0f * core::utils::math::constants::pi<float>());

    if ((uint8_t)configuration().invert) {
        data = -data;
    }
}

bool
QEI_Position::waitUntilReady()
{
    core::os::Thread::sleep(core::os::Time::ms((uint16_t)configuration().period));

    return true;
}
}
}
