/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */
 
 /*
 * Modification history:
 * 180313 GC OnLoop timing
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
#define STOP_CURR_CALIB_CNTR 250
#define DISA_CURR_CALIB_CNTR 350
// Time spent in brake disengage (in steps, 1ms each one)
//#define DISENGAGE_OFFS  (1.0f * _this->_reduction_ratio) // 1 deg

#define OVERCURRENT_THS		3.0f
#define UNDERVOLTAGE_THS	5.0f
#define DYN_MOD_CNTR_MAX    2

#define EDO_JOINT_FW_MAJOR      3
#define EDO_JOINT_FW_MINOR      0
#define EDO_JOINT_FW_REVISION   1910
#define EDO_JOINT_FW_SVN        876

#define EDO_DISENGAGE_SIN       5000

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
    _current(0.0f),
    _ff_torque(0.0f),
    _currvel(0.0f),
    _ErrorCheckSafe(0),
    _ackindex(0),
    _identity(-1),
    _last_encoder_timestamp(core::os::Time::now()),
    _last_encoder_measure(core::os::Time::now()),
    _last_supply_drop_timestamp(core::os::Time::now()),
    _ms5_timestamp(core::os::Time::ms(5)),
    _ms10_timestamp(core::os::Time::ms(10)),
    _ms500_timestamp(core::os::Time::ms(500)),
    _s10_timestamp(core::os::Time::s(10)),
    _infinite_timestamp(core::os::Time::INFINITE - core::os::Time::s(10)),

    _reduction_ratio(-231.0f*5.33333),  //231 for large joint 186 for small joint
    _accumulator_A(0.0f),
    _counter_A(0),
    _accumulator_V(0.0f),
    _counter_V(0),
    _curr_alert_cntr(0),
    _motor_current(0.0f),
    _motor_supply(0.0f),
    _comm_ready(false),
    _calibrat_cntr(DISA_CURR_CALIB_CNTR+1),
    _current_offset(0.0f),
    _pos_calibration(false),
    _current_calibration(false),
    _current_calibration_completed(false),
    _target_predisengage(0.0f),
    _current_limit(0.0f),
	_res_diff_limit(0.3f),
    _fllerr(false),
    _fllerr_power(true),
	_velerr(false),
	_set_Enable(true),
    _misFiltered(0.0f),
	_savepos_flag(false)
{
    _workingAreaSize = 2560;
    setStatusMask(J_STATE_UNCALIB, true);
}

ControlNode::~ControlNode()
{
    teardown();
}

bool ControlNode::onInitialize()
{
    bool success = true;

    success &= _encoder.init();
    success &= _motor.init();
	
    return success;
}

bool ControlNode::onConfigure()
{
  bool success = true;

  overrideConfiguration();

  //Recovery of the saved position
	overridingConfiguration().pos_validity = 0;
    _accpos = configuration().position;


  success &= _encoder.configure();
  success &= _motor.configure();

  config_unsafe(CONTROL_DEFAULT_PARAMETERS);
  
  return success;
}

bool ControlNode::onPrepareHW()
{
    bool success = true;

    return success;
}

bool ControlNode::onPrepareMW()
{
    //Registers the subscribers callback
    _subscriber_setpoint.set_callback(ControlNode::setpoint_callback);
    _subscriber_parameters.set_callback(ControlNode::parameters_callback);
    bool success = true;
    return success;
}

bool ControlNode::onStart()
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
	_last_supply_drop_timestamp = _infinite_timestamp;

    return success;
}

bool ControlNode::onLoop()
{
    //Get the current time
    core::os::Time now_1;
    bool collDetflag = false;
	float vr_dynFiltered (0.0f);
	//float vr_misFiltered (0.0f);
    int   vi_identity;

    // Spin with timeout 5ms - Normally the timeout does not expire,
    // a setpoint every 1msec comes from the interpolator.
    if (!this->spin(core::os::Time::ms(5))) {

      // The timeout is expired, this is a SW fail then we have to maintain a safe condition.

      set(_target, 0, 0);

      //If the encoder works...
      if (_encoder.update())
      {
        float deltapos;  //Encoder accumulator
        float speed;
        float temp;
        int deltatime;   //Time elapsed from the last encoder read

        _encoder.get(deltapos);
        //Get the currrent time
        core::os::Time now_2 = core::os::Time::now();
        //Time elapsed between two encoder read
        deltatime = now_2.raw - _last_encoder_measure.raw;

        //Motore speed
        if (deltatime > 0)
          speed = (deltapos * 1000000) / deltatime;
        else
          speed = 0;

        _last_encoder_measure = now_2;
        _currvel = speed;
        _accpos += deltapos;

        //Apply the control
        temp = update(_accpos , _currvel, _motor_current, _reduction_ratio, _brake_status, &_fllerr, &_velerr);
        _motor.setI(temp);

      }
      else
      {
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
    {
      if (_calibrat_cntr == STOP_CURR_CALIB_CNTR)
      {
		// Phase 2: calculates offset
        _current_offset /= _calibrat_cntr;
		_motor_current = - (filterCurrent() - _current_offset);
        _calibrat_cntr++;
        reset_collDetFilter();
      }
	  else if (_calibrat_cntr < DISA_CURR_CALIB_CNTR)
	  {
		_motor_current = - (filterCurrent() - _current_offset);
		_calibrat_cntr++;
	  }
	  else if (_calibrat_cntr == DISA_CURR_CALIB_CNTR)
	  {
		 _motor_current = - (filterCurrent() - _current_offset);
		 _calibrat_cntr++;
		 _current_calibration_completed = true;
         setStatusMask(J_STATE_UNCALIB, false);		 
	  }
      else
      {
        //Phase 3: apply MA FIR to the motor current ADC measure
        _motor_current = - (filterCurrent() - _current_offset); //GA_attention: introduced a minus sign to fit the dynamic model
	  }
    }

    // --- End of current measure calibration
    _motor_supply = filterVoltage();

    now_1 = core::os::Time::now();
    if (_motor_supply > UNDERVOLTAGE_THS)
    {
      //Reset the timer if the supply is OK.
      _last_supply_drop_timestamp = now_1;
	  
	  if(_set_Enable)			//If false the Raspberry is not working so don't disengage the electric brake
	  {
      // Disengage the electric brake
      module.brake.set();
	  palClearPad(GPIOA,4);
	  
	  }
	  _savepos_flag=true;
    }
    else
    {
      // Engage the electric brake
		module.brake.clear();
		palSetPad(GPIOA,4);
				//The Raspberry is working correctly, motor supply is cut
		
		if (_savepos_flag==true)
        {
			overridingConfiguration().position  = _accpos - _calibpos;
			module.configurations().saveTo(module.configurationStorage());
			_savepos_flag=false;
         }
	  
		_set_Enable=true;
		
      // Set undervoltage alarm bit
      setStatusMask(J_STATE_UNDERVOLTAGE, true);
    }

    // ---  BEGIN : PUBLICATION OF JOINT STATE EVERY 10ms ----------------------------------------------------

    //When 10ms are elapsed---
    if (now_1 < _last_encoder_timestamp)
       _last_encoder_timestamp = now_1;
    if ((now_1 -_last_encoder_timestamp) >= _ms10_timestamp)
    {
      //Overcurrent alarm management
      if (fabs(_motor_current) > OVERCURRENT_THS)
      {
        _curr_alert_cntr++;
        //Enter alarm state if the overcurrent is confirmed for 0.1sec.
        if (_curr_alert_cntr >= 100)
        {
          setStatusMask(J_STATE_UNDERVOLTAGE, true);
        }
		else if (_curr_alert_cntr >= 200)
        {
		  setStatusMask(J_STATE_OVERCURRENT, true);
        }
      }
      else
      {
        setStatusMask(J_STATE_OVERCURRENT, false);
        //Measure OK; reset the counter
        _curr_alert_cntr = 0;
      }
	  
	  vi_identity =_identity;
    
     (void)collDetect(_motor_current, _current, &collDetflag, &_misFiltered, &vr_dynFiltered, vi_identity, _current_limit, _res_diff_limit);

    //Prepare the state message
	  if(collDetflag && _current_calibration_completed && _fllerr_power && _brake_status == 2)
	  {
        // Set undervoltage alarm bit
        setStatusMask(J_STATE_UNDERVOLTAGE, true);
        palSetPad(GPIOA,11); //Red Led up
		
		// Engage electric  brake: h-bridge in Z
		module.brake.clear();
		palClearPad(GPIOA,4);
		_set_Enable=false;

      }
      else
      {
        //palClearPad(GPIOA,11); //Red Led down
	  }
	  
      core::control_msgs::Encoder_State* encoder;
      if (_comm_ready &&
          _encoder_publisher.alloc(encoder))
      {
        //Fill in the message fields
        encoder->position    = (_accpos - _calibpos) / _reduction_ratio; //posizione misurata [Deg_mot] 
        encoder->velocity    = _currvel / _reduction_ratio;  //velocità misurata [Deg/s_mot] 
		encoder->current     = _motor_current;  //corrente misurata filtrata [A]
		//encoder->current     = _misFiltered;  //corrente misurata filtrata [A]
		
		//encoder->position    = _disengage_cntr;
		//encoder->velocity    = _accpos / _reduction_ratio;
		//encoder->current     = _predisengage_cntr;		 
        //encoder->velocity    = vr_dynFiltered; // MF Only for MONI to debug coll. detection
        //encoder->velocity    = (_target - _calibtarget) / _reduction_ratio; // GC Only for MONI
        
     	encoder->commandFlag = _ack;
		encoder->R_jnt = _cur_res_joint;
		//encoder->R_jnt = _motor_supply;
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
      _last_encoder_timestamp = now_1;
    }
    return true;
}
// --- END: ControlNode::onLoop ------------------------------------------------------

bool ControlNode::onStop()
{
    bool success = true;

    success &= _encoder.stop();
    success &= _motor.stop();

    return success;
}

// --- SETPOINT CALLBACK AND CONTROL LOOP ------------------------------------
bool ControlNode::setpoint_callback(const core::control_msgs::Ctrlpnt_f32& msg, void* context)
{
  ControlNode* _this = static_cast<ControlNode*>(context);

  float tmp_shock = 0.0;
  
/* ------------- CALIBRATION ------------- */
  if (_this->_pos_calibration)
  {
    //...store the current position and...
    _this->_calibpos = _this->_accpos;
    _this->hold_pos(true);

    _this->reset_trgFilter();
	
    //...check to exit the calibration procedure as soon as the received target is zero.
    if (fabs(msg.value) < 0.01f)
    {
      //Set the calibration offset to the target.
      _this->_calibtarget = _this->_calibpos;
      _this->_pos_calibration = false;

      _this->hold_pos(false);
      //Calibrated!
      _this->overridingConfiguration().pos_validity = 1;
      //Following Error Detection ON
      _this->_fllerr_power = true;
    }
  }
  
  if(_this->_current_calibration)
  {
    //Re-calculate the currentt offset
    _this->_calibrat_cntr = 0;
    _this->_current_offset = 0.0f;
    _this->_current_calibration = false;
	_this->_current_calibration_completed = false;
  }

  // Get the targets from the message
  _this->_target       = msg.value * _this->_reduction_ratio;
  _this->_ff_vel       = msg.ffv   * _this->_reduction_ratio;
  _this->_current      = msg.ffv;
  _this->_ff_torque    = msg.fft   / _this->_reduction_ratio;
  _this->_cur_res_rasp = msg.R_rasp;
  
  // Apply the calibration correction
  _this->_target += _this->_calibtarget;
   
  // When the motor supply is off do not accumulate integration errors,
  // to avoid a sudden reaction when the supply will be provided.
  _this->_disengage_status = false;
  _this->_update_target_predisengage = false;
  
  if (_this->_motor_supply <= UNDERVOLTAGE_THS)
  {
    // Set zero the integral terms
    _this->reset_i();
    // Hold the position loop open
    _this->hold_pos(true);
    // il sistema non e' sfrenato
    _this->_brake_status = 0;
  }
  else
  {
    // Disengage the brake only if the motor is supplied    

    if(_this->_brake_status==-1)
    {
      _this->reset_trgFilter();
      _this->reset_collDetFilter();
      _this->hold_pos(false);
      _this->_pos[0]=msg.arm_position[0];
      _this->_pos[1]=msg.arm_position[1];
      _this->_pos[2]=msg.arm_position[2];
	  _this->_brake_status = 1;
    }

    if(_this->_brake_status==1)
    {
      /* ------------- DISENGAGE ------------- */
      tmp_shock = _this->Disengage_onLoop(_this->_identity,_this->_target, _this->_target_predisengage);
    }
	// Azzero flag errori
	if(_this->_brake_status!=2)
	{
	  _this->setStatusMask(J_STATE_UNDERVOLTAGE, false);
	  _this->_ErrorCheckSafe = 0;
	  _this->_fllerr = false;
	}
  }

    float deltapos;
    float speed;
    float temp;
    int   deltatime;

    _this->_encoder.get(deltapos);

    core::os::Time now_time = core::os::Time::now();
    // deltatime is in milliseconds
    deltatime = now_time.raw - _this->_last_encoder_measure.raw;
    if (deltatime > 0)
	  speed = (deltapos * 1000000.0f) / deltatime;
    else
      speed = 0;
    _this->_last_encoder_measure = now_time;
    _this->_currvel = speed;
    _this->_accpos += deltapos;
    
	// Salvo posizione all' inzio della sfrenatura
    if (_this->_update_target_predisengage)
    {
      _this->_target_predisengage = _this->_accpos;
    }
	
    // Set the target and the feedforwards
    if (_this->_disengage_status)
    {
      _this->set(_this->_target_predisengage+tmp_shock, 0.0f, 0.0f);
      _this->_target = _this->_target_predisengage;
    }
    else
    {
      _this->set(_this->_target, _this->_ff_vel, _this->_ff_torque);
    }

/* ------------- Apply the CONTROL ------------- */

	temp = _this->update(_this->_accpos, _this->_currvel, _this->_motor_current, _this->_reduction_ratio, _this->_brake_status,  &_this->_fllerr, &_this->_velerr);
		  
/* ------------- Errors and Safety check ------------- */

	if (_this->_brake_status == 2)
	{

		_this->_ErrorCheckSafe = _this->safetyCheck(_this->_misFiltered, _this->_fllerr, _this->_fllerr_power);
	    
		if (_this->_ErrorCheckSafe > 0 || _this->_velerr==true)
	    { 	 
         // Set undervoltage alarm bit
          _this->setStatusMask(J_STATE_UNDERVOLTAGE, true);
          palSetPad(GPIOA,11); //Red Led up
		
		  // Engage electric  brake: h-bridge in Z
		  module.brake.clear();
		  palClearPad(GPIOA,4);
		  _this->_set_Enable=false;
	    }
	    else
	    {
	      //palClearPad(GPIOA,11); //Red Led down
	    }
	}
      
/* ------------- CALIBRATION managment ------------- */
	if (_this->_calibrat_cntr > STOP_CURR_CALIB_CNTR + 100)
	{
	  _this->_motor.setI(temp);
	}
	else
	{
	  // Do not control during current offset calibration
	  _this->set(_this->_target, _this->_ff_vel, _this->_ff_torque);
	  //The encoder does not work, just hold the motor brake on
	  _this->_motor.setI(0);
	}
	
    return true;
}

bool ControlNode::parameters_callback(const core::control_msgs::PID_param& msg, void* context)
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

void ControlNode::ISR_updateCurrent(float new_sample)
{
	// The ISR lock is done by the caller

	// Calculates the filtered current
	_accumulator_A += new_sample;
	_counter_A += 1;
}

void ControlNode::ISR_updateVoltage(float new_sample)
{
	// The ISR lock is done by the caller

	// Calculates the filtered voltage
	_accumulator_V += new_sample;
	_counter_V += 1;
}

void ControlNode::setack(const uint8_t flag)
{
  // Low nibble bits 0-3 are reserved for the ack code
  if ((flag & J_STATE_ACK_MASK) == 0)
    return;// If there is not a valid code, exit

  _ack |= flag;
  _ackindex = J_STATE_ACK_CYCLES;
}

void ControlNode::setStatusMask(const uint8_t bit, const bool set)
{
  if (set)
    _ack |= 1 << bit;
  else
    _ack &= ~(1 << bit);
}

void ControlNode::set_coll_threshold(float coll_limit)
{
  _current_limit  = coll_limit;
}

void ControlNode::set_doublecheck_threshold(float diff_limit)
{
  _res_diff_limit = diff_limit;
}

// --- START THE BRAKE DISENGAGMENT - CALLED BY THE MASTERNODE ------------
void ControlNode::disengage_brake(const uint32_t & time, const float & offset)
{
	palClearPad(GPIOA,11);
	
	if((int)time == SIN)
	{
		Disengage_Init(SIN, offset, _reduction_ratio);
	}
	else
	{
		if(overridingConfiguration().pos_validity == 1)
		{
			Disengage_Init(STD, offset, _reduction_ratio);
		}
		else
		{
			Disengage_Init(UNC, offset, _reduction_ratio);
		}
	}
}

// --- START THE POSITION CALIBRATION - CALLED BY THE MASTERNODE ------------
void
ControlNode::pos_calibration(void)
{
	_pos_calibration = true;
	_fllerr_power    = false;
}

void
ControlNode::current_calibration(void)
{
	_current_calibration = true;
}

// --- SET THE COMMUNICATION TOPICS - CALLED BY THE MASTERNODE ------------
bool ControlNode::communication_setup( const int ID, const float red_ratio )
{
   sprintf(_topic_contr, "j%d_contr",ID);
   this->subscribe(_subscriber_setpoint, _topic_contr);

   sprintf(_topic_enc, "j%d_state",ID);
   this->advertise(_encoder_publisher, _topic_enc);			//POSIZIONI DEGLI ALTRI GIUNTI

   sprintf(_topic_param, "j%d_param",ID);
   this->subscribe(_subscriber_parameters, _topic_param);
   
   sprintf(_topic_version, "j%d_version",ID);
   this->advertise(_version_publisher, _topic_version);

   this->_reduction_ratio = red_ratio;

   _comm_ready = true;
   _identity=ID;
   
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

float ControlNode::filterVoltage()
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

unsigned int ControlNode::safetyCheck(float vr_CurFiltered, bool _fllerr, bool _fllerr_power)
{
 /* -------------------------------- SAFETY CHECK --------------------------- */
	  if (_fllerr & _fllerr_power)
	  {
	     _ErrorCheckSafe = 1;
	  }
	return _ErrorCheckSafe;
}

}
}