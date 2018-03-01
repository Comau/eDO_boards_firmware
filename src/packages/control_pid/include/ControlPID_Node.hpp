/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */



#pragma once
// --- INCLUDES -----------------------------------------------------------------
#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <core/utils/BasicSensor.hpp>
#include <core/utils/BasicActuator.hpp>

#include <ModuleConfiguration.hpp>

#include <core/control_pid/ControlPIDNodeConfiguration.hpp>
#include <core/control_msgs/Ctrlpnt_f32.hpp>
#include <core/control_msgs/PID_param.hpp>
#include <core/common_msgs/Float32.hpp>
#include <core/control_msgs/Encoder_State.hpp>
#include <core/control_pid/PID2.hpp>
#include <comau_edo/edo_msgs/EdoJointVersion.hpp>

#define J_STATE_ACK_INIT		1
#define J_STATE_ACK_CALIB		2
#define J_STATE_ACK_CONFIG		3
#define J_STATE_ACK_RESET		4
#define J_STATE_ACK_VERSION		5
// bits
#define J_STATE_UNDERVOLTAGE		5
#define J_STATE_OVERCURRENT		6
#define J_STATE_UNCALIB			7


// --- DEFINITION -------------------------------------------------------------
namespace core {
namespace control_pid{
class ControlNode:
    public mw::CoreNode,
    public mw::CoreConfigurable<ControlPIDNodeConfiguration>,
    public PID2
{
public:
    ControlNode(
        const char*                        name,
        core::utils::BasicSensor<float>&   encoder,
        core::utils::BasicActuator<float>& motor,
        os::Thread::Priority               priority = os::Thread::PriorityEnum::NORMAL
    );

    bool
	communication_setup(const int ID, const float red_ratio);

    bool version_publish(const int &id);

    void
    ISR_updateCurrent(
        float new_sample
    );

    void
    ISR_updateVoltage(
        float new_sample
    );

    void
	setack(const uint8_t flag);
	
    void setStatusMask(const uint8_t bit, const bool set);

    void
    disengage_brake(const uint32_t & time, const float & offset);

    void
    pos_calibration(void);

    virtual
    ~ControlNode();


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
    setpoint_callback(
        const core::control_msgs::Ctrlpnt_f32& msg,
        void*                                  context
    );

    static bool
    parameters_callback(
        const core::control_msgs::PID_param& msg,
        void*                                context
    );

    float
    filterCurrent();

    float
    filterVoltage();


private:


    //Devices
    core::utils::BasicSensor<float>&   _encoder;
    core::utils::BasicActuator<float>& _motor;

    //Communication channels
    mw::Subscriber<core::control_msgs::Ctrlpnt_f32, 2> _subscriber_setpoint;
    mw::Publisher<core::control_msgs::Encoder_State>   _encoder_publisher;
    mw::Subscriber<core::control_msgs::PID_param, 2>   _subscriber_parameters;
    mw::Publisher<comau_edo::edo_msgs::EdoJointVersion>   _version_publisher;


    //Internal variables
    float _target;								//Angular position target
    float _accpos;								//Accumultade angular position
    float _calibpos;							//Angular position calibration offset
    float _calibtarget;							//Target position calibration offset
    float _ff_vel;								//Velocity feedforward
    float _ff_torque;							//Torque feedforward
    float _currvel;								//Motor speed
    int   _ackindex;							//
    uint8_t _ack;								//Acknowledge/Errors
    uint8_t _jnt_err;							//Joint warnings/errors

    char _topic_contr[16];						//Topic name to receive the setpoints
    char _topic_enc[16];						//Topic name to publish the joint state
    char _topic_param[16];						//Topic name to receive PID parameters
    char _topic_version[16];					//Topic name to publish firmware version


    core::os::Time _last_encoder_timestamp;
    core::os::Time _last_encoder_measure;
    core::os::Time _last_supply_drop_timestamp;

    float    _reduction_ratio;
    float    _accumulator_A;
    uint32_t _counter_A;
    float    _accumulator_V;
    uint32_t _counter_V;
    uint32_t _curr_alert_cntr;
    float    _motor_current;
    float    _motor_supply;

    bool     _comm_ready;

    uint32_t _calibrat_cntr;
    float    _current_offset;

    uint32_t _disengage_cntr;
    uint32_t _disengageSteps;
	float _disengage_frequency;
    float _disengageOffset;

    bool     _pos_calibration;
};
}
}
