/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>

#include <core/interpolation_package/InterpNodeConfiguration.hpp>
#include <comau_edo/edo_msgs/EdoJointCtrl.hpp>
#include <core/control_msgs/Ctrlpnt_f32.hpp>

#define INTERPOLATION_HERMITE 1

namespace core {
namespace interpolation_package {
class InterpNode:
    public mw::CoreNode,
    public mw::CoreConfigurable<InterpNodeConfiguration>
{
public:
	InterpNode(
        const char*                        name,
        os::Thread::Priority               priority = os::Thread::PriorityEnum::NORMAL
    );


    bool
	communication_setup(
		const int ID
	);

    void
    pos_calibration(void);


    virtual
    ~InterpNode();


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
    callback_setpnt(
        const comau_edo::edo_msgs::EdoJointCtrl& msg,
        void*                                    context
    );


private:
    // [ang, vel, acc, ffv, fft]
    mw::Subscriber<comau_edo::edo_msgs::EdoJointCtrl, 2> _setpoint_subscriber;

    enum {
        SETPNT_LEN = 2
    };

    comau_edo::edo_msgs::EdoJointCtrl _setpoint[SETPNT_LEN];

	uint8_t _head_setp;   // The last setpoint received by USB, it is on the head of the queue.
	uint8_t _num_setp;    // The queue is a maximum of SETPNT_LEN setpoints.
	uint8_t _tail_setp;   // The setpoin to interpolate, extracted from the tail of the queue.
	bool _interp_active;
	float _current_pos;
	char _topic_setpnt[16];
	char _topic_contr[16];
	float _arm_pos[3];
	float _R_rasp;
	
    // [ang, ffv, fft]
    mw::Publisher<core::control_msgs::Ctrlpnt_f32>  _control_publisher;

    // Store the interpolation starting point
    comau_edo::edo_msgs::EdoJointCtrl _startpoint;
	uint8_t _interp_counter;

    core::os::Time _last_control_timestamp;
	bool     _comm_ready;

    bool     _pos_calibration;

#ifdef INTERPOLATION_HERMITE
    float
	interpolate(float step, float startp, float startv, float endp, float endv);
#endif


};
}
}
