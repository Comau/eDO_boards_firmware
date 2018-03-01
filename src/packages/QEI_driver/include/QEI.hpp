/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/hw/QEI.hpp>
#include <core/utils/BasicSensor.hpp>
#include <core/common_msgs/Float32.hpp>
#include <core/QEI_driver/QEI_DeltaConfiguration.hpp>
#include <core/QEI_driver/QEI_PositionConfiguration.hpp>


namespace core {
namespace QEI_driver {
class QEI
{
public:
    QEI(
        core::hw::QEI& qei
    );

    virtual
    ~QEI();

public:
    bool
    probe();


public:
    core::hw::QEI& _qei;
};

class QEI_Delta:
    public core::utils::BasicSensor<float>,
    public core::mw::CoreConfigurable<core::QEI_driver::QEI_DeltaConfiguration>
{
public:
    QEI_Delta(
        const char* name,
        QEI&        device
    );

    virtual
    ~QEI_Delta();

private:
public:
    bool
    init();

    bool
    configure();

    bool
    start();

    bool
    stop();

    bool
    waitUntilReady();

    bool
    update();

    void
    get(
        DataType& data
    );

    void
    getspeed(
           DataType& data
       );


protected:
    core::os::Time _timestamp;

private:
    QEI& _device;
};

class QEI_Position:
    public core::utils::BasicSensor<float>,
    public core::mw::CoreConfigurable<core::QEI_driver::QEI_PositionConfiguration>
{
public:
    QEI_Position(
        const char* name,
        QEI&        device
    );

    virtual
    ~QEI_Position();

public:
    bool
    init();

    bool
    configure();

    bool
    start();

    bool
    stop();

    bool
    waitUntilReady();

    bool
    update();

    void
    get(
        DataType& data
    );


protected:
    core::os::Time _timestamp;

private:
    QEI& _device;

public:
    struct Converter {
        using FROM = float;
        using TO   = core::common_msgs::Float32;

        static inline void
        _(
            const FROM& from,
            TO*         to
        )
        {
            to->value = from;
        }
    };
};
}
}
