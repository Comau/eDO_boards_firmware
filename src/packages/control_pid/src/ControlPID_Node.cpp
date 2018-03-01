/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */


// --- INCLUDES -------------------------------------------------------------------------
#include <Module.hpp>
#include <core/control_pid/ControlPID_Node.hpp>
#include <stdio.h>
#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>

#include <core/hw/PWM.hpp>
#include <core/hw/ADC.hpp>
#include <core/hw/GPIO.hpp>

#include <math.h>

// Current offset calibration time (in steps, 1ms each one)
#define STOP_CURR_CALIB_CNTR 20
// Time spent in brake disengage (in steps, 1ms each one)
#define DISENGAGE_STEPS 1000
#define DISENGAGE_OFFS  (1.0f * _this->_reduction_ratio) // 1 deg

#define OVERCURRENT_THS		3.0f
#define UNDERVOLTAGE_THS	4.0f

#define EDO_JOINT_FW_MAJOR      2
#define EDO_JOINT_FW_MINOR      0
#define EDO_JOINT_FW_REVISION   1502
#define EDO_JOINT_FW_SVN        427

#define EDO_DISENGAGE_NFASE  2

// --- MODULE -----------------------------------------------------------------

extern Module module;

// --- DEFINITION -------------------------------------------------------------------------
namespace core {
namespace control_pid {
ControlNode::ControlNode(
    const char*                        name,
    core::utils::BasicSensor<float>&   encoder,
    core::utils::BasicActuator<float>& motor,
    os::Thread::Priority               priority
) :
    CoreNode::CoreNode(name, priority),
    CoreConfigurable::CoreConfigurable(name),  			//Makes the node configurable
    _encoder(encoder),
    _motor(motor),
    _target(0.0f),
	_accpos(0.0f),
	_calibpos(0.0f),
	_calibtarget(0.0f),
	_ff_vel(0.0f),
	_ff_torque(0.0f),
	_currvel(0.0f),
	_ackindex(0),
    _last_encoder_timestamp(0),
    _last_encoder_measure(0),
	_last_supply_drop_timestamp(0),
    _reduction_ratio(-231.0f*5.33333),  //231 for large joint 186 for small joint
	_accumulator_A(0.0f),
	_counter_A(0),
	_accumulator_V(0.0f),
	_counter_V(0),
	_curr_alert_cntr(0),
	_motor_current(0.0f),
	_motor_supply(0.0f),
	_comm_ready(false),
	_calibrat_cntr(0),
	_current_offset(0.0f),
	_disengage_frequency(0),
	_disengage_cntr(0),
	_pos_calibration(false),
	_disengageOffset(2.0),
	_disengageSteps(1000)
{
    _workingAreaSize = 2048;
    setStatusMask(J_STATE_UNCALIB, true);
}

ControlNode::~ControlNode()
{
    teardown();
}


bool
ControlNode::onInitialize()
{
    bool success = true;

    success &= _encoder.init();
    success &= _motor.init();

    return success;
}

bool
ControlNode::onConfigure()
{
    bool success = true;

      overrideConfiguration();

      //Recovery of the saved position (if valid)
      if (overridingConfiguration().pos_validity == 1)
      {
    	  _accpos = overridingConfiguration().position;
      }

	  success &= _encoder.configure();
	  success &= _motor.configure();

	  config_unsafe(	1.0,		//kp
						10000,		//ti
						0.0,		//td
						1.0,		//ts
						-46816.0,	//min
						46816.0, 	//max
						100000.0, 	//sat
						0, 			//kff

						1, 			//kpv
						0.0, 		//tiv
						0.0, 		//tdv
						1.0, 		//tsv
						-93631.0, 	//minv
						93631.0, 	//maxv
						100000.0, 	//satv
						0.0, 		//kfv

						1,			//kpt
						0.0,		//tit
						0.0,		//tdt
						1.0,		//tst
						-12.0,		//mint
						12.0,		//maxt
						100000000,	//satt
						0.0);		//kft

    return success;
}

bool
ControlNode::onPrepareHW()
{
    bool success = true;

    return success;
}

bool
ControlNode::onPrepareMW()
{
	//Registers the subscribers callback
    _subscriber_setpoint.set_callback(ControlNode::setpoint_callback);
    _subscriber_parameters.set_callback(ControlNode::parameters_callback);
    bool success = true;
    return success;
}

bool
ControlNode::onStart()
{
    bool success = true;

    success &= _encoder.start();
    success &= _motor.start();

    module.current_sense_adc.setChannelCallback(0, [&](core::hw::ADCConversionGroup::SampleType x)
    	{
            core::os::SysLock::ISRScope lock;
            this->ISR_updateCurrent(x);
            static uint32_t tmp = 1;
            //In this way we have about 18 equally distribuited current measure points each 1 msec.
            tmp = (tmp + 233) % 4096; // 233 = (4096 / 17,578)    PWM = 17578 Hz
            if (tmp == 0) tmp = 1;
            core::hw::PWM_1::driver->tim->CCR[3] = tmp;
        });

    module.voltage_sense_adc.setChannelCallback(0, [&](core::hw::ADCConversionGroup::SampleType x)
    	{
			core::os::SysLock::ISRScope lock;
            this->ISR_updateVoltage(x);
    	});

    //Init the timer as disabled
	_last_supply_drop_timestamp = core::os::Time::INFINITE - core::os::Time::s(10);

    return success;
}

bool
ControlNode::onLoop()
{
    //Get the current time
    core::os::Time now = core::os::Time::now();


    //Spin with timeout 5ms - Normally the timeout does not expire,
    //a setpoint every 1msec comes from the interpolator.
    if (!this->spin(core::os::Time::ms(5))) {

    	//The timout is expired, this is a SW fail then we have to maintain a safe condition.

    	set(_target, 0, 0);

    	//If the encoder works...
        if (_encoder.update())
        {
			float deltapos;				//Encoder accumulator
			float speed;
			float temp;
			int deltatime;				//Time elapsed from the last encoder read

			_encoder.get(deltapos);
			//Get the currrent time
			core::os::Time now_time = core::os::Time::now();
			//Time elapsed between two encoder read
			deltatime = now_time.raw - _last_encoder_measure.raw;

			//Motore speed
			speed = (deltapos * 1000000) / deltatime;

			_last_encoder_measure = now_time;
			_currvel = speed;
			_accpos += deltapos;

			//Apply the control
			temp = update(_accpos , _currvel, _motor_current);
			_motor.setI(temp);
        }
        else {
        	// ...the encoder does not work, just hold the motor brake on
            _motor.setI(0);
        }
    }

    // --- Calibration of the ADC current measure offset -------------------------------------------------
    if (_calibrat_cntr < STOP_CURR_CALIB_CNTR) {
    	// Phase 1: collects measures
    	_calibrat_cntr++;
    	_current_offset += filterCurrent();
    }
    else
	if (_calibrat_cntr == STOP_CURR_CALIB_CNTR) {
    	// Phase 2: calculates offset
		_current_offset /= _calibrat_cntr;
    	_calibrat_cntr++;
	}
    else {
		//Phase 3: apply MA FIR to the motor current ADC measure
		_motor_current = filterCurrent() - _current_offset;
    }
    // --- End of current measure calibration

    _motor_supply = filterVoltage();

    if (_motor_supply > UNDERVOLTAGE_THS)
    {
    	//Reset the timer if the supply is OK.
    	_last_supply_drop_timestamp = now;

        // Disengage the electric brake
    	module.brake.set();
    	
    	// Clear undervoltage alarm bit
    	setStatusMask(J_STATE_UNDERVOLTAGE, false);
    }
    else
    {
        // Engage the electric brake
    	module.brake.clear();
    	
 	// Set undervoltage alarm bit
    	setStatusMask(J_STATE_UNDERVOLTAGE, true);
    }

    //When the timer is expired, save the current position
    if (now > _last_supply_drop_timestamp + core::os::Time::ms(500))
    {
    	//Save only if previously calibrated
        if (overridingConfiguration().pos_validity == 1)
    	{
			overridingConfiguration().position  = _accpos - _calibpos;

			module.configurations().saveTo(module.configurationStorage());
    	}

		//Disable the timer (less 10 seconds to avoid wrap-around errors)
		_last_supply_drop_timestamp = core::os::Time::INFINITE - core::os::Time::s(10);
    }

    // ---  BEGIN : PUBLICATION OF JOINT STATE EVERY 10ms ----------------------------------------------------

    //When 10ms are elapsed---
    if (now > _last_encoder_timestamp + core::os::Time::ms(10))
    {
		//Overcurrent alarm management
    	if (fabs(_motor_current) > OVERCURRENT_THS)
    	{
    		_curr_alert_cntr++;
    		//Enter alarm state if the overcurrent is confirmed for 0.5sec.
    		if (_curr_alert_cntr >= 50)
    			setStatusMask(J_STATE_OVERCURRENT, true);
    		//Overcurrent alarm is NOT reversible.
    	}
    	else
    	{
    		//Measure OK; reset the counter
    		_curr_alert_cntr = 0;
    	}

    	//Prepare the state message
    	core::control_msgs::Encoder_State* encoder;
        if (_comm_ready &&
        	_encoder_publisher.alloc(encoder))
        {
        	//Fill in the message fields
			encoder->position    = (_accpos - _calibpos) / _reduction_ratio;
			encoder->velocity    = _currvel / _reduction_ratio;
			encoder->current     = _motor_current;
			encoder->commandFlag = _ack;
			//Public the message
			_encoder_publisher.publish(encoder);

			//If there is an active ACK...
	        core::os::SysLock::Scope lock;
			if (_ackindex > 0)
			{
				//...decrement the counter...
				_ackindex--;
			}
			else
			{
				//...when zero, clear the ack bits
				_ack &= 0xF0;
			}
        }

        //Update the time counter
        _last_encoder_timestamp = now;
    }


    return true;
}
// --- END: ControlNode::onLoop ------------------------------------------------------

bool
ControlNode::onStop()
{
    bool success = true;

    success &= _encoder.stop();
    success &= _motor.stop();

    return success;
}


float tmp_shock = 0.0;

// --- SETPOINT CALLBACK AND CONTROL LOOP ------------------------------------
bool
ControlNode::setpoint_callback(
    const core::control_msgs::Ctrlpnt_f32& msg,
    void*                                  context
)
{
    ControlNode* _this = static_cast<ControlNode*>(context);

    //If the calibration procedure is running...
	if (_this->_pos_calibration)
	{
		//...store the current position and...
		_this->_calibpos = _this->_accpos;
		_this->hold_pos(true);

		//...check to exit the calibration procedure as soon as the received target is zero.
		if (fabs(msg.value) < 0.01f)
		{
			//Set the calibration offset to the target.
			_this->_calibtarget = _this->_calibpos;
			_this->_pos_calibration = false;
			_this->hold_pos(false);
			//Calibrated!
			_this->overridingConfiguration().pos_validity = 1;
			//Reset the uncalibrated error, if active.
			_this->setStatusMask(J_STATE_UNCALIB, false);
		}
	}

    //Get the targets from the message
    _this->_target    = msg.value * _this->_reduction_ratio;
    _this->_ff_vel    = msg.ffv * _this->_reduction_ratio;
    _this->_ff_torque = msg.fft / _this->_reduction_ratio;
    //Apply the calibration correction
    _this->_target += _this->_calibtarget;

	//When the motor supply is off do not accumulate integration errors,
	//to avoid a sudden reaction when the supply will be provided.
	if (_this->_motor_supply <= UNDERVOLTAGE_THS)
	{
		//Set zero the integral terms
		_this->reset_i();
		//Hold the position loop open
		_this->hold_pos(true);
	}
	else
	{
		//Disengage the brake only if the motor is supplied
		if(_this->_disengage_cntr > 0)
		{
			if(_this->_disengage_cntr == _this->_disengageSteps)
				_this->hold_pos(false);
				
		   tmp_shock =  _this->_disengageOffset * sin ( 2.0 * 3.14159265 * _this->_disengage_frequency * _this->_disengage_cntr );
		   
		   _this->_disengage_cntr --;
		}		
		else
		{
		   tmp_shock = 0.0;
		}
		_this->_target += tmp_shock;
	}

    //Set the target and the feedforwards
    _this->set(_this->_target, _this->_ff_vel, _this->_ff_torque);

    if (_this->_encoder.update()) 
	{
		float deltapos;
		float speed;
		float temp;
		int deltatime;

		_this->_encoder.get(deltapos);

		core::os::Time now_time = core::os::Time::now();

		deltatime = now_time.raw - _this->_last_encoder_measure.raw;

		speed = (deltapos * 1000000.0f) / deltatime;

		_this->_last_encoder_measure = now_time;
		_this->_currvel = speed;
		_this->_accpos += deltapos;

		//Apply the control
		temp=_this->update(_this->_accpos, _this->_currvel, _this->_motor_current);

	    if (_this->_calibrat_cntr > STOP_CURR_CALIB_CNTR)
		{
	    	_this->_motor.setI(temp);
	    }
	    else
		{
	    	// Do not control during current offset calibration
            _this->_motor.setI(0);
	    }
    }
    else 
	{
    	//The encoder does not work, just hold the motor brake on
		_this->_motor.setI(0);
	}

    return true;
}


bool
ControlNode::parameters_callback(
		const core::control_msgs::PID_param& msg,
		void*                            context
)
{
	ControlNode* _this = static_cast<ControlNode*>(context);

	//Set the new PID parameters configuration
	_this->config(msg.kp,  msg.ti,  msg.td,  1, -msg.max,  msg.max,  msg.sat,  msg.kff,
				  msg.kpv, msg.tiv, msg.tdv, 1, -msg.maxv, msg.maxv, msg.satv, msg.kffv,
				  msg.kpt, msg.tit, msg.tdt, 1, -msg.maxt, msg.maxt, msg.satt, msg.kfft,
				  msg.kt);

	_this->setack(J_STATE_ACK_CONFIG);

	return true;
}


void
ControlNode::ISR_updateCurrent(
    float new_sample
)
{
	// The ISR lock is done by the caller

	// Calculates the filtered current
	_accumulator_A += new_sample;
	_counter_A += 1;
}


void
ControlNode::ISR_updateVoltage(
    float new_sample
)
{
	// The ISR lock is done by the caller

	// Calculates the filtered voltage
	_accumulator_V += new_sample;
	_counter_V += 1;
}


void
ControlNode::setack(const uint8_t flag)
{
	if (flag >= 0xF0) // bits 0-3 are reserved for ack
		return;
    
	_ack |= flag;
	_ackindex = 8;
}

void ControlNode::setStatusMask(const uint8_t bit, const bool set)
{
	if(set)
		_ack |= 1 << bit;
	else
		_ack &= ~(1 << bit);	
}

// --- START THE BRAKE DISENGAGMENT - CALLED BY THE MASTERNODE ------------
void
ControlNode::disengage_brake(const uint32_t & time, const float & offset)
{
	_disengageSteps  = time;
	_disengageOffset = offset * _reduction_ratio;
	_disengage_frequency = 1.0 / (_disengageSteps / EDO_DISENGAGE_NFASE );
	_disengage_cntr  = _disengageSteps; // meglio caricarlo per ultimo, e' l'abilitatore allo sblocco
}

// --- START THE POSITION CALIBRATION - CALLED BY THE MASTERNODE ------------
void
ControlNode::pos_calibration(void)
{
	_pos_calibration = true;
}

// --- SET THE COMMUNICATION TOPICS - CALLED BY THE MASTERNODE ------------
bool
ControlNode::communication_setup(
		const int ID,
		const float red_ratio
)
{
   sprintf(_topic_contr, "j%d_contr",ID);
   this->subscribe(_subscriber_setpoint, _topic_contr);

   sprintf(_topic_enc, "j%d_state",ID);
   this->advertise(_encoder_publisher, _topic_enc);

   sprintf(_topic_param, "j%d_param",ID);
   this->subscribe(_subscriber_parameters, _topic_param);

   sprintf(_topic_version, "j%d_version",ID);
   this->advertise(_version_publisher, _topic_version);

   this->_reduction_ratio = red_ratio;

   _comm_ready = true;

   return true;

}

bool ControlNode::version_publish(const int &id)
{
	comau_edo::edo_msgs::EdoJointVersion* fw_version;
	if (_version_publisher.alloc(fw_version))
	{
		//Fill in the message fields
		fw_version->id       = id;
		fw_version->major    = EDO_JOINT_FW_MAJOR;
		fw_version->minor    = EDO_JOINT_FW_MINOR;
		fw_version->revision = EDO_JOINT_FW_REVISION;
		fw_version->svn      = EDO_JOINT_FW_SVN;
			//Public the message
		return _version_publisher.publish(fw_version);
	}

	return false;
}

float
ControlNode::filterCurrent()
    {
        float measure;

        core::os::SysLock::Scope lock;

        // Moving average FIR.
        if (_counter_A > 0) {
        	measure = _accumulator_A / (float)_counter_A;
        }
        else {
			measure = 0.0f;
        }
        _counter_A = 0;
        _accumulator_A = 0.0f;

        measure = measure * (3.3f/4095.0f); // sample to Volts
        measure = measure - (3.3f/2.0f); // Center to +- 1.65V
        measure = measure / 0.11f; // Convert to current

        // Returns the filtered current
        return measure;
}

float
ControlNode::filterVoltage()
    {
        float measure;

        core::os::SysLock::Scope lock;

        // Moving average FIR.
        if (_counter_V > 0) {
        	measure = _accumulator_V / (float)_counter_V;
        }
        else {
			measure = 0.0f;
        }
        _counter_V = 0;
        _accumulator_V = 0.0f;

        measure = measure * (48.0f/4095.0f); // sample to Volts

        // Returns the filtered voltage
        return measure;
}



}
}



