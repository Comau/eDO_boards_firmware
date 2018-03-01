/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */



#pragma once
// --- INCLUDES -------------------------------------------------------------------------------
#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <core/utils/BasicSensor.hpp>
#include <core/utils/BasicActuator.hpp>

#include <ModuleConfiguration.hpp>

#include <core/control_pid/ControlPIDNodeConfiguration.hpp>

#include <core/interpolation_package/InterpNode.hpp>
#include <core/control_pid/ControlPID_Node.hpp>

#include <core/master_package/MasterNodeConfiguration.hpp>
#include <comau_edo/edo_msgs/EdoJointInit.hpp>
#include <comau_edo/edo_msgs/EdoJointCalibration.hpp>
#include <comau_edo/edo_msgs/EdoJointReset.hpp>
#include <comau_edo/edo_msgs/EdoJointVersion.hpp>



// --- DEFINITION ------------------------------------------------------------------------------
namespace core {
namespace master_package {
class MasterNode:
    public mw::CoreNode,
    public mw::CoreConfigurable<MasterNodeConfiguration>
{
public:
    MasterNode(
        const char*                        name,
	    core::control_pid::ControlNode&    controller,
	    core::interpolation_package::InterpNode& interpolator,
        os::Thread::Priority               priority = os::Thread::PriorityEnum::NORMAL

    );
    virtual
    ~MasterNode();


private:
    bool
    onInitialize();

    bool
    onConfigure();

    bool
    onPrepareHW();

    bool
    onPrepareMW();

    bool
    onStart();

    bool
    onLoop();

    bool
    onStop();

    static bool
    init_callback(
        const comau_edo::edo_msgs::EdoJointInit& msg,
        void*                                  context
    );

    static bool
    calibration_callback(
        const comau_edo::edo_msgs::EdoJointCalibration& msg,
        void*                                  context
    );

    static bool
    reset_callback(
        const comau_edo::edo_msgs::EdoJointReset& msg,
        void*                                  context
    );

    static bool
    version_callback(
        const comau_edo::edo_msgs::EdoJointVersion& msg,
        void*                                  context
    );

private:

    int   _identity;	//Joint ID
    float _reduction;	//Reduction ratio

    core::control_pid::ControlNode& _controller;
    core::interpolation_package::InterpNode& _interpolator;

    mw::Subscriber<comau_edo::edo_msgs::EdoJointInit, 2> _subscriber_init;
    mw::Subscriber<comau_edo::edo_msgs::EdoJointCalibration, 2> _subscriber_calibration;
    mw::Subscriber<comau_edo::edo_msgs::EdoJointReset, 2> _subscriber_reset;
    mw::Subscriber<comau_edo::edo_msgs::EdoJointVersion, 2> _subscriber_version;

};
}
}
