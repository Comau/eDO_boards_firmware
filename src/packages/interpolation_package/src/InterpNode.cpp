/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <stdio.h>
#include <Module.hpp>

#include <core/interpolation_package/InterpNode.hpp>
#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>

#include <math.h>


namespace core {
namespace interpolation_package {
InterpNode::InterpNode(
    const char*                        name,
    os::Thread::Priority               priority
) :
    CoreNode::CoreNode(name, priority),
    CoreConfigurable::CoreConfigurable(name),
	_head_setp(0),
	_num_setp(0),
	_tail_setp(0),
	_interp_active(false),
	_current_pos(0.0f),
	_interp_counter(0),
	_last_control_timestamp(core::os::Time::ms(0)),
	_comm_ready(false),
	_pos_calibration(false)
{
	_arm_pos[0] = 0.0f;
	_arm_pos[1] = 0.0f;
	_arm_pos[2] = 0.0f;
	
	for (int i = 0; i < SETPNT_LEN; i++) {
		_setpoint[i].position       = 0.0f;
		_setpoint[i].velocity       = 0.0f;
		_setpoint[i].acceleration   = 0.0f;
		_setpoint[i].feedfwvelocity = 0.0f;
		_setpoint[i].feedfwtorque   = 0.0f;
	}

	_startpoint.position       = 0.0f;
	_startpoint.velocity       = 0.0f;
	_startpoint.acceleration   = 0.0f;
	_startpoint.feedfwvelocity = 0.0f;
	_startpoint.feedfwtorque   = 0.0f;

    _workingAreaSize = 1024;

    sprintf(_topic_setpnt, "nojoint");
    sprintf(_topic_contr, "nocontr");
}

InterpNode::~InterpNode()
{
    teardown();
}

bool InterpNode::onInitialize()
{
    bool success = true;

    return success;
}

bool InterpNode::onConfigure()
{
    bool success = true;

    return success;
}

bool InterpNode::onPrepareHW()
{
    bool success = true;

    return success;
}

bool InterpNode::onPrepareMW()
{
	_setpoint_subscriber.set_callback(InterpNode::callback_setpnt);

    return true;
}

bool InterpNode::onStart()
{
    bool success = true;

	_last_control_timestamp = core::os::Time::now();

    return success;
}

#ifdef INTERPOLATION_HERMITE
float InterpNode::interpolate(float step, float startp, float startv, float endp, float endv)
{
	float s2 = step  * step;
	float s3 = s2 * step;

	float h2 = (-2.0f * s3) + (3.0f * s2);
	float h1 = -h2 + 1.0f;
	float h3 = s3 - (2.0f * s2) + step;
	float h4 = s3 - s2;

	float current_point = (h1 * startp) + (h2 * endp);
	current_point += (h3 * startv) + (h4 * endv);

	return current_point;
}
#endif

bool InterpNode::onLoop()
{
	float current_step = 0.0f;
	float current_ffv  = 0.0f;
	float current_fft  = 0.0f;

    core::control_msgs::Ctrlpnt_f32* ang_setpnt;

    // USB sends a setpoint every 10msec to UDC.
    //uint8_t num_of_steps = 10 / configuration().interp_interval;
    uint8_t num_of_steps = 10;



    /* Use the spin just to fetch the messages; do not pause here */
	this->spin(core::os::Time::IMMEDIATE);

	// Activates the interpolator as soon as the queue is 50% (+1 message) full.
	if (not _interp_active) {
		if (_num_setp == 1 + (SETPNT_LEN / 2)) {
			_interp_active = true;
			// Fetch the first setpoint from the queue.
			_num_setp -= 1;
			_tail_setp = (_tail_setp + 1) % SETPNT_LEN;
		}
	}

	if (_interp_active) {
		// Reset the interpolation cycle
		if (_interp_counter == num_of_steps) {
			_startpoint = _setpoint[_tail_setp];
			_interp_counter = 0;
			// Fetch next setpoint from the queue.
			if (_num_setp > 0) {
				_num_setp -= 1;
				_tail_setp = (_tail_setp + 1) % SETPNT_LEN;
			}
		}

		// Calculates interpolated value at current step
		_interp_counter++;

		current_step = (float)_interp_counter / (float)num_of_steps;

#ifdef INTERPOLATION_HERMITE
		// Spline cubic Hermite interpolation
		_current_pos = interpolate(current_step, _startpoint.position, _startpoint.velocity,
				_setpoint[_tail_setp].position, _setpoint[_tail_setp].velocity);

		current_ffv = interpolate(current_step,	_startpoint.feedfwvelocity, _startpoint.acceleration,
				_setpoint[_tail_setp].feedfwvelocity, _setpoint[_tail_setp].acceleration);
		current_ffv = current_ffv;
#else
		// Linear interpolation
		_current_pos  = _startpoint.position;
		_current_pos += (_setpoint[_tail_setp].position - _startpoint.position) * current_step;

		current_ffv  = _startpoint.feedfwvelocity;
		current_ffv += (_setpoint[_tail_setp].feedfwvelocity - _startpoint.feedfwvelocity) * current_step;
#endif
		// Torque is always linear interpolated
		current_fft  = _startpoint.feedfwtorque;
		current_fft += (_setpoint[_tail_setp].feedfwtorque - _startpoint.feedfwtorque) * current_step;
	}

#if 0
	//When calibration is ongoing, do not interpolate
	if (_pos_calibration)
	{
		_current_pos  = _setpoint[_tail_setp].position;

		//End of calibration condition
		if (fabs(_current_pos) < 0.01)
		{
			_pos_calibration = false;
		}
	}
#endif

	// Publish the interpolated control point to the PID chain
	if (_comm_ready && (_control_publisher.alloc(ang_setpnt))) 
	{
		ang_setpnt->value = _current_pos;
		ang_setpnt->ffv   = _startpoint.acceleration;
		ang_setpnt->fft   = current_fft;
		ang_setpnt->arm_position[0] = _arm_pos[0];
		ang_setpnt->arm_position[1] = _arm_pos[1];
		ang_setpnt->arm_position[2] = _arm_pos[2];
		ang_setpnt->R_rasp = _R_rasp;
		_control_publisher.publish(ang_setpnt);
	}

	//_last_control_timestamp += core::os::Time::ms(configuration().interp_interval);
	_last_control_timestamp += core::os::Time::ms(1);

	core::os::Time now = core::os::Time::now();
	if (now < _last_control_timestamp) {
	    chThdSleepUntil(_last_control_timestamp.ticks());
	}

    return true;
} // InterpNode::onLoop

bool InterpNode::onStop()
{
    bool success = true;

    return success;
}

bool InterpNode::callback_setpnt( const comau_edo::edo_msgs::EdoJointCtrl& msg, void* context )
{
	InterpNode* _this = static_cast<InterpNode*>(context);

	// Drop the setpoint if the queue is full
	if (_this->_num_setp < SETPNT_LEN) {
		_this->_head_setp = (_this->_head_setp + 1) % SETPNT_LEN;
		_this->_num_setp += 1;
		// Insert the new setpoint into queue head
		_this->_setpoint[_this->_head_setp] = msg;
		_this->_arm_pos[0] = msg.arm_position[0];
		_this->_arm_pos[1] = msg.arm_position[1];
		_this->_arm_pos[2] = msg.arm_position[2];
		_this->_R_rasp     = msg.R_rasp;
	}

    return true;
}


bool InterpNode::communication_setup( const int ID )
{  //char _topic_setpnt[16];
   sprintf(_topic_setpnt, "j%d_setpnt",ID);
   this->subscribe(_setpoint_subscriber, _topic_setpnt);

   //char _topic_contr[16];
   sprintf(_topic_contr, "j%d_contr",ID);
   this->advertise(_control_publisher,   _topic_contr);

   this->_comm_ready=true;
   return true;

}

void InterpNode::pos_calibration(void)
{
	_pos_calibration = true;
}


}
}
