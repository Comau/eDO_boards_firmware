/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

// #include "hal.h"   // Just for using the yellow led
#include <core/joint/ROSSerialNode.hpp>

#include <core/mw/Middleware.hpp>
#include <core/common.hpp>

#include <Module.hpp>

#define DEVELOPMENT_RELEASE (1==0)
#define ENABLE_VERSION_HANDLING (1==0)

#define EDO_JOINT_FW_MAJOR      2
#define EDO_JOINT_FW_MINOR      0
#define EDO_JOINT_FW_REVISION   1804
#define EDO_JOINT_FW_SVN        458

enum INIT_MODE {
	DISCOVERY_MODE = 0,
	SET_MODE,
	CANCEL_MODE
};


namespace core {
namespace joint {

ROSSerialNode::ROSSerialNode(
    core::joint::Joint (&joints)[MAX_JOINTS],
    const char*                name,
    core::os::Thread::Priority priority
) :
    CoreNode::CoreNode(name, priority),
    CoreConfigurable(name),
    _static_joints(joints)
{
  _workingAreaSize = 1024;

  _initialized = false;
  _joints_mask = 0;
  _number_of_joints = 0;
  _led_prescaler = 10;
  _state_joint_mask_checkFlag = false;


}

ROSSerialNode::~ROSSerialNode()
{
    teardown();
}

bool
ROSSerialNode::onConfigure()
{
    return true;
}

bool
ROSSerialNode::onPrepareHW()
{
    //nh.getHardware()->setIOChannel(Module::stream);
    return true;
}

bool
ROSSerialNode::onPrepareMW()
{

  // Initialize ROS NodeHandle
  nh.initNode();

  /* Initialize Joint Nodes */
  _jnt_state_pub_msg.joints_mask = (uint64_t)_joints_mask;
  _jnt_state_pub_msg.joints_length = _number_of_joints;
  _jnt_state_pub_msg.joints = new edo_core_msgs::JointState[MAX_JOINTS];
  _jnt_state_pub = new ros::Publisher("usb_jnt_state", &_jnt_state_pub_msg);
  _jnt_fw_version_pub = new ros::Publisher("usb_jnt_version", &_jnt_fw_version_pub_msg);

  nh.advertise(*_jnt_state_pub);
  nh.advertise(*_jnt_fw_version_pub);

  this->advertise(publisher_init, "joint_init");
  this->advertise(publisher_reset, "joint_reset");
  this->advertise(publisher_calib, "joint_calib");
  this->advertise(publisher_version_req, "joint_version");

  startJoints();

  nh.logdebug("ROSSerial node ready");

  return true;
}

inline bool
ROSSerialNode::onLoop()
{
  nh.spinOnce();
  /* Use the spin just to fetch the messages; do not pause here */
  this->spin(core::os::Time::IMMEDIATE);
  core::os::Time now_1 = core::os::Time::now();

  if (now_1 < _last_publish_timestamp)
    _last_publish_timestamp = now_1;
  if ((now_1 - _last_publish_timestamp) >= core::os::Time::ms(10))
  {
    if (--_led_prescaler == 0)
    {
      palTogglePad(GPIOA,3); // Toggle del led giallo
      _led_prescaler = 10;
    }
    if ((_initialized == true) && (_number_of_joints > 0))
    {
      uint32_t sm_state_joint_mask = 0;
      // Publish joint state, joint_mask will contain only joints with an updated state
      uint32_t sm_joint_mask = _joints_mask;
      uint8_t  sd_commandFlag, sd_commandFlagXxx, sd_commandFlagPrev, sd_commandFlagXxxPrev;
      for (uint8_t j = 0; j < _number_of_joints; j++, sm_joint_mask >>= 1)
      {
        if ((sm_joint_mask & 1) != 0)
        {
          if (_static_joints[j].stateUpdated())
          {
            sm_state_joint_mask |= (1 << j);
            _static_joints[j].setStateUpdated(false);
            _state_joint_mask_checkCnt[j] = 0;
            _state_joint_mask_checkFlag   = false;
            sd_commandFlag = _static_joints[j].getCommandFlag(); // Made a local copy of the Command Flags
            sd_commandFlagPrev = _static_joints[j].getCommandFlagPrev(); // Made a local copy of the previous Command Flags
            // Are there some bits different? 
            if(sd_commandFlag ^ sd_commandFlagPrev) 
            { // Yes
              _static_joints[j].setCommandFlagPrev(sd_commandFlag);   // Update the previous image
#if 0
              sd_commandFlagXxx     = sd_commandFlag     & J_STATE_ACK_MASK; // Use the acknowledge bits only
              sd_commandFlagXxxPrev = sd_commandFlagPrev & J_STATE_ACK_MASK; // Use the acknowledge bits only
              if(sd_commandFlagXxx ^ sd_commandFlagXxxPrev) 
              { // Yes
                if (sd_commandFlagXxx & ~sd_commandFlagXxxPrev) 
                { /* Positive edge transition -- Rising Edge */
                  nh.loginfo(_static_joints[j].compileLogInfoMsg(J_ACK_TE, sd_commandFlagXxx));
                }
                if (~sd_commandFlagXxx & sd_commandFlagXxxPrev) 
                { /* Negative edge transition -- Falling Edge */
                  nh.loginfo(_static_joints[j].compileLogInfoMsg(J_ACK_FE, sd_commandFlagXxx));
                }
              }
#endif
#if 1
              sd_commandFlagXxx     = sd_commandFlag     & J_STATE_ERR_MASK; // Use the acknowledge bits only
              sd_commandFlagXxxPrev = sd_commandFlagPrev & J_STATE_ERR_MASK; // Use the acknowledge bits only
              if(sd_commandFlagXxx ^ sd_commandFlagXxxPrev) 
              { // Yes
                if (sd_commandFlagXxx & ~sd_commandFlagXxxPrev) 
                { /* Positive edge transition -- Rising Edge */
                  nh.loginfo(_static_joints[j].compileLogInfoMsg(J_ERR_TE, sd_commandFlagXxx));
                }
#if 0
                if (~sd_commandFlagXxx & sd_commandFlagXxxPrev) 
                { /* Negative edge transition -- Falling Edge */
                  nh.loginfo(_static_joints[j].compileLogInfoMsg(J_ERR_FE, sd_commandFlagXxx));
                }
#endif
              }
#endif
            }
          }
          else
          {
            _state_joint_mask_checkCnt[j] ++;
            if ( _state_joint_mask_checkCnt[j] >= EDO_JOINT_MAX_CHECK_CNT)
            {
              _state_joint_mask_checkFlag = true;
            }
          }
       
          if ( _state_joint_mask_checkFlag)
          {  
            _jnt_state_pub_msg.joints[j].position    = 0;
            _jnt_state_pub_msg.joints[j].velocity    = 0;
            _jnt_state_pub_msg.joints[j].current     = 0;
            _jnt_state_pub_msg.joints[j].commandFlag = _static_joints[j].getCommandFlag() | 0x20;
			_jnt_state_pub_msg.joints[j].R_jnt       = 0;
          }
          else
          {
            _jnt_state_pub_msg.joints[j].commandFlag = _static_joints[j].getCommandFlag();
            _jnt_state_pub_msg.joints[j].position    = _static_joints[j].getPos();
            _jnt_state_pub_msg.joints[j].velocity    = _static_joints[j].getVel();
            _jnt_state_pub_msg.joints[j].current     = _static_joints[j].getCurrent();
			_jnt_state_pub_msg.joints[j].R_jnt       = _static_joints[j].getCurRes();
	
			/* char buf[32];
			sprintf(buf, "CurRes: %f", _static_joints[j].getCurRes());
			nh.loginfo(buf); */
	      }
        }
        else
        {
          _jnt_state_pub_msg.joints[j].position    = 0;
          _jnt_state_pub_msg.joints[j].velocity    = 0;
          _jnt_state_pub_msg.joints[j].current     = 0;
          _jnt_state_pub_msg.joints[j].commandFlag = 0;
		  _jnt_state_pub_msg.joints[j].R_jnt       = 0;
        }
      }
      
       // Pubblico comunque anche se non sono arrivati tutti
      _jnt_state_pub_msg.joints_mask   |= (uint64_t)(sm_state_joint_mask & JOINT_MASK);
      _jnt_state_pub_msg.joints_length  = _number_of_joints;

      if(_jnt_state_pub != nullptr)
        _jnt_state_pub->publish(&_jnt_state_pub_msg);
    }
    _last_publish_timestamp = now_1;
  }
  core::os::Thread::sleep(core::os::Time::ms(2));

#if 0  
  if(!joints_manager.areOk()){
    // Handle error
    nh.logwarn("USB: joints manager not ok");
  }
#endif
  return true;
}    // ROSSerialNode::onLoop

void
ROSSerialNode::callback_ros_jntctrl_(
    const edo_core_msgs::JointControlArray& msg
)
{
	uint32_t sm_joint_mask = _joints_mask;
	const edo_core_msgs::JointControl* jntCtrl;
  
	if(!_initialized)
		return;
	
	if((msg.joints_length != _number_of_joints) && (_number_of_joints != 0)){
		//nh.logwarn("rx joint control with wrong length");
		return;
	}
	
	/* Forward the received message to each joint */
	comau_edo::edo_msgs::EdoJointCtrl* jntCtrlMsg;
	
	for (uint8_t j = 0; j < _number_of_joints; j++, sm_joint_mask >>= 1)
	{
		if ((sm_joint_mask & 1) != 0)
		{
		  jntCtrl = &msg.joints[j];
		  if(_static_joints[j]._publisher_ctrl.alloc(jntCtrlMsg))
		  {
			  jntCtrlMsg->position       = jntCtrl->position;
			  jntCtrlMsg->velocity       = jntCtrl->velocity;
			  jntCtrlMsg->acceleration   = jntCtrl->current;
			  jntCtrlMsg->feedfwvelocity = jntCtrl->ff_velocity;
			  jntCtrlMsg->feedfwtorque   = jntCtrl->ff_current;
			  jntCtrlMsg->R_rasp         = jntCtrl->R_rasp;
			  
			  // saving current position of joints 1,2,3 from the encoder
			  jntCtrlMsg->arm_position[0] = _static_joints[0].getPos();
			  jntCtrlMsg->arm_position[1] = _static_joints[1].getPos();
			  jntCtrlMsg->arm_position[2] = _static_joints[2].getPos();
			  
			  if(!_static_joints[j]._publisher_ctrl.publish(jntCtrlMsg))
			  {
				  // Handle error
				  char buf[32];
				  sprintf(buf, "usb j %d ctrl pub", j+1);
				  nh.logerror(buf);
			  }
		  }
		}
	}
}

uint8_t ROSSerialNode::getJointIDsFromMask(uint32_t mask, uint8_t ids_array[], uint8_t size)
{
  uint8_t sd_last_jnt_number;
  uint8_t i;
  
  for(i = 0, sd_last_jnt_number = 0; i < size; i++, mask = mask >> 1)
  {
    ids_array[i] = 0;
    if((mask & 1) != 0) { // joint i+1 is in mask
      ids_array[i] = i+1;
      sd_last_jnt_number = i+1;
    }
  }
  return sd_last_jnt_number;
}

void
ROSSerialNode::callback_ros_jntconfig_(
	const edo_core_msgs::JointConfigurationArray &msg
)
{
	uint32_t joints_mask;
	const edo_core_msgs::JointConfiguration* jntConfig;
	core::control_msgs::PID_param* jntConfigMsg;
  
	if(!_initialized){
		nh.logwarn("jncfg: not init");
		return;
	}
	
	if(msg.joints_length > _number_of_joints){
		nh.logwarn("jcnfg: wrong len");
		return;
	}

	joints_mask = (uint32_t)msg.joints_mask;  // Use a local uint32_t mask, just to avoid long long
		
	/* Forward the received message to each joint */	
	for (uint8_t j = 0; j < msg.joints_length; j++, joints_mask >>= 1)
	{
		// Does this axis exist?
		if (joints_mask & 1)
		{	// Yes	
		  jntConfig = &msg.joints[j];
		  if(_static_joints[j]._publisher_config.alloc(jntConfigMsg))
		  {
		  	jntConfigMsg->kp   = jntConfig->kp;
		  	jntConfigMsg->ti   = jntConfig->ti;
		  	jntConfigMsg->td   = jntConfig->td;
		  	jntConfigMsg->sat  = jntConfig->sat;
		  	jntConfigMsg->kff  = jntConfig->kff;
		  	jntConfigMsg->max  = jntConfig->max;
		  	jntConfigMsg->kpv  = jntConfig->kpv;
		  	jntConfigMsg->tiv  = jntConfig->tiv;
		  	jntConfigMsg->tdv  = jntConfig->tdv;
		  	jntConfigMsg->satv = jntConfig->satv;
		  	jntConfigMsg->kffv = jntConfig->kffv;
		  	jntConfigMsg->maxv = jntConfig->maxv;
		  	jntConfigMsg->kpt  = jntConfig->kpt;
		  	jntConfigMsg->tit  = jntConfig->tit;
		  	jntConfigMsg->tdt  = jntConfig->tdt;
		  	jntConfigMsg->satt = jntConfig->satt;
		  	jntConfigMsg->kfft = jntConfig->kfft;
		  	jntConfigMsg->maxt = jntConfig->maxt;
		  	jntConfigMsg->kt   = jntConfig->kt;

		  	if(!_static_joints[j]._publisher_config.publish(jntConfigMsg))
		  	{
					char buf[32];
					sprintf(buf, "Error@%d", __LINE__);
					nh.loginfo(buf);
		  	}
		  }
		}
		core::os::Thread::sleep(core::os::Time::ms(10));
	}
}

void
ROSSerialNode::callback_ros_jntinit_(
	const edo_core_msgs::JointInit &msg
)
{
  uint8_t sd_nr_joints;
  
  uint32_t rx_mask = (uint32_t)msg.joints_mask;
  if((rx_mask | JOINT_MASK) > JOINT_MASK){
    // Handle error: 
    nh.logwarn("ji: invalid mask");
    return;
  }
  
  switch (msg.mode){
    case DISCOVERY_MODE:
      {
        uint8_t ids[32];
        sd_nr_joints = getJointIDsFromMask(rx_mask, ids, sizeof(ids));
        if(startStopJoints(ids, sd_nr_joints))
        {
          if (sd_nr_joints > MAX_JOINTS)
            sd_nr_joints = MAX_JOINTS;
          _number_of_joints = sd_nr_joints;
          _joints_mask = rx_mask & JOINT_MASK;
          if(!_initialized)
          {
            _initialized = true;
            char buf[32];
            sprintf(buf, "ji: %d j M:%ld", sd_nr_joints, (unsigned long) rx_mask);
            nh.loginfo(buf);
          }
          else
          {
            char buf[32];
            sprintf(buf, "ji: again %d j M:%ld", sd_nr_joints, (unsigned long) rx_mask);
            nh.loginfo(buf);
          }
        }
      }
    break;
    case SET_MODE:
      if(!_initialized){
        // Handle error
        nh.logwarn("ji: set id not init");
        return;
      }
    break;
    case CANCEL_MODE:
      if(!_initialized){
        // Handle error
        nh.logwarn("ji: cncl id not init");
        return;
      }
    break;
    
    default:
    break;
  }
  
  /* Forward init msg to joints */
  comau_edo::edo_msgs::EdoJointInit* jntInitMsg;
  if(publisher_init.alloc(jntInitMsg)){
    jntInitMsg->mode = msg.mode;
    jntInitMsg->joints_mask = msg.joints_mask;
    jntInitMsg->reduction = msg.reduction_factor;
    if(!publisher_init.publish(jntInitMsg)){
      // Handle error
      nh.logwarn("ji: errPub");
    }
  }
}

void
ROSSerialNode::callback_ros_jntcalib_(
	const edo_core_msgs::JointCalibration &msg
)
{
	uint64_t rx_mask = msg.joints_mask;
	if((rx_mask | JOINT_MASK) > JOINT_MASK){
		// Handle error:
		nh.logwarn("jc: invalid mask");
		return;
	}
	
	/* Forward msg to joints */
	comau_edo::edo_msgs::EdoJointCalibration* CANmsg;
	if(publisher_calib.alloc(CANmsg)){
		CANmsg->joints_mask = msg.joints_mask;
		if(!publisher_calib.publish(CANmsg)){
			// Handle error
			nh.logwarn("jc: errPub");
		}
	}
}

void
ROSSerialNode::callback_ros_jntreset_(
	const edo_core_msgs::JointReset &msg
)
{
	uint64_t rx_mask = msg.joints_mask;
	if((rx_mask | JOINT_MASK) > JOINT_MASK){
		// Handle error: 
		nh.logwarn("jr: invalid mask");
		return;
	}
	
	if(rx_mask == 0){
		Module::reset();
		return;
	}
	
	/* Forward msg to joints */
	comau_edo::edo_msgs::EdoJointReset* CANmsg;
	if(publisher_reset.alloc(CANmsg)){
		CANmsg->joints_mask = msg.joints_mask;
		CANmsg->disengage_steps = msg.disengage_steps;
		CANmsg->disengage_offset = msg.disengage_offset;
		if(!publisher_reset.publish(CANmsg)){
			// Handle error
			nh.logwarn("jr: errPub");
		}
	}
}

void
ROSSerialNode::callback_ros_jntversion_(
	const std_msgs::UInt8 &msg
)
{
#if DEVELOPMENT_RELEASE
	nh.loginfo("Rx jnt version req");
#endif

	/* Send USB version to ROS if data is 0 */
	if(msg.data == 0){
		_jnt_fw_version_pub_msg.id = 0;
		_jnt_fw_version_pub_msg.majorRev = EDO_JOINT_FW_MAJOR;
		_jnt_fw_version_pub_msg.minorRev = EDO_JOINT_FW_MINOR;
		_jnt_fw_version_pub_msg.revision = EDO_JOINT_FW_REVISION;
		_jnt_fw_version_pub_msg.svn = EDO_JOINT_FW_SVN;
		if(_jnt_fw_version_pub != nullptr)
			_jnt_fw_version_pub->publish(&_jnt_fw_version_pub_msg);
		return;
	}

#if ENABLE_VERSION_HANDLING
	/* Send firmware version request to joints */
	comau_edo::edo_msgs::EdoJointVersion* CANmsg;
	if(_number_of_joints >= msg.data && publisher_version_req.alloc(CANmsg)){
		CANmsg->id = msg.data;
		if(!publisher_version_req.publish(CANmsg)){
			// Handle error
			char buf[32];
			sprintf(buf, "jv: joint %d errPub", msg.data);
			nh.logwarn(buf);
		}
	}
	else
	{
		char buf[32];
		sprintf(buf, "jv: Nr joint %d too high", msg.data);
		nh.logwarn(buf);
	}
#endif
}

bool ROSSerialNode::startJoints()
{	
	for (uint8_t j = 0; j < MAX_JOINTS; j++)
	{
		joints_manager.add(_static_joints[j]);		
	}
	
	joints_manager.setup();
	joints_manager.run();

	return true;
}

bool ROSSerialNode::startStopJoints(uint8_t joint_ids[], uint8_t number_of_joints)
{
  if(number_of_joints > MAX_JOINTS){
    nh.logerror("ji: Nr joints too high");
    return false;
  }
  for (uint8_t i = 0; i < MAX_JOINTS; i++)
  {
    bool found = false;
    for (uint8_t j = 0; j < number_of_joints; j++)
    {
      if(joint_ids[j] == _static_joints[i].get_id()) // joint ID match
      {
        if((_static_joints[i].state() != State::LOOPING) && (_static_joints[i].state() != State::STARTING))
          _static_joints[i].execute(Action::START);
        found = true;
        break;
      }
    }
    if(!found)
    {
      _static_joints[i].execute(Action::STOP);
      char buf[32];
      sprintf(buf, "ji: joint %d stopped", i+1);
      nh.logerror(buf);
      // Just log the error, this could be ok on a 6 axes robot without the gripper 
    }
  }

  return true;
}

void ROSSerialNode::setCtrlSub(ros::Subscriber<edo_core_msgs::JointControlArray> *_ros_sub_jntctrl)
{
	if(_ros_sub_jntctrl != nullptr){
		nh.subscribe(*_ros_sub_jntctrl);
	}
}

void ROSSerialNode::setConfigSub(ros::Subscriber<edo_core_msgs::JointConfigurationArray> *_ros_sub_jntconfig)
{
	if(_ros_sub_jntconfig != nullptr){
		nh.subscribe(*_ros_sub_jntconfig);
	}
}

void ROSSerialNode::setInitSub(ros::Subscriber<edo_core_msgs::JointInit> *_ros_sub_jntinit)
{
	if(_ros_sub_jntinit != nullptr){
		nh.subscribe(*_ros_sub_jntinit);
	}
}

void ROSSerialNode::setCalibSub(ros::Subscriber<edo_core_msgs::JointCalibration> *_ros_sub_jntcalib)
{
	if(_ros_sub_jntcalib != nullptr){
		nh.subscribe(*_ros_sub_jntcalib);
	}
}

void ROSSerialNode::setResetSub(ros::Subscriber<edo_core_msgs::JointReset> *_ros_sub_jntreset)
{
	if(_ros_sub_jntreset != nullptr){
		nh.subscribe(*_ros_sub_jntreset);
	}
}

void ROSSerialNode::setVersionSub(ros::Subscriber<std_msgs::UInt8> *_ros_sub_jntversion)
{
	if(_ros_sub_jntversion != nullptr){
		nh.subscribe(*_ros_sub_jntversion);
	}
}

}
}
