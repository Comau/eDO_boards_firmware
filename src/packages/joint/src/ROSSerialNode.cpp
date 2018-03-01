/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <core/joint/ROSSerialNode.hpp>

#include <core/mw/Middleware.hpp>
#include <core/common.hpp>

#define EDO_JOINT_FW_MAJOR      2
#define EDO_JOINT_FW_MINOR      0
#define EDO_JOINT_FW_REVISION   1710
#define EDO_JOINT_FW_SVN        300

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
    _workingAreaSize = 512;

	_initialized = false;
	_joints_mask = 0x00;
	_number_of_joints = 0;

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
ROSSerialNode::onPrepareMW()
{
	// Initialize ROS NodeHandle
    nh.initNode();
	
	/* Initialize Joint Nodes */
	 _jnt_state_pub_msg.joints_mask = _joints_mask;
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
	
	nh.logdebug("USB: ros node ready");

    return true;
}

inline bool
ROSSerialNode::onLoop()
{
	nh.spinOnce();
	/* Use the spin just to fetch the messages; do not pause here */
	this->spin(core::os::Time::IMMEDIATE);
    core::os::Time now = core::os::Time::now();

    if (now > _last_publish_timestamp + core::os::Time::ms(10)) {		
		// Publish joint state, joint_mask will contain only joints with an updated state
		uint64_t state_joint_mask = 0;
		uint64_t bit1 = 1;
		for (uint8_t j = 0; j < _number_of_joints; j++)
		{
			if(_static_joints[j].stateUpdated()){
				state_joint_mask |= (bit1 << (_static_joints[j].get_id() - 1));
				_static_joints[j].setStateUpdated(false);
				if(((_static_joints[j].getCommandFlag() & 0x0F) > 0)
					&& ((_static_joints[j].getCommandFlag() & 0x0F) <= 0x0F)){
					char msg[50];
					sprintf(msg, "Joint ACK rx from %d", _static_joints[j].get_id());
					nh.loginfo(msg);
				}
			}
			
			_jnt_state_pub_msg.joints[j].position = _static_joints[j].getPos();
			_jnt_state_pub_msg.joints[j].velocity = _static_joints[j].getVel();
			_jnt_state_pub_msg.joints[j].current = _static_joints[j].getCurrent();
			_jnt_state_pub_msg.joints[j].commandFlag = _static_joints[j].getCommandFlag();
		}

		_jnt_state_pub_msg.joints_mask = state_joint_mask;
		
		if(_jnt_state_pub != nullptr)
			_jnt_state_pub->publish(&_jnt_state_pub_msg);
        _last_publish_timestamp = now;
    }
	
	if(!joints_manager.areOk()){
		// Handle error
		nh.logwarn("USB: joints manager not ok");
	}
	
	core::os::Thread::sleep(core::os::Time::ms(2));

    return true;
}    // ROSSerialNode::onLoop

void
ROSSerialNode::callback_ros_jntctrl_(
    const edo_core_msgs::JointControlArray& msg
)
{
	if(!_initialized)
		return;
	
	if(msg.joints_length != _number_of_joints && _number_of_joints != 0){
		//nh.logwarn("rx joint control with wrong length");
		return;
	}
	
	/* Forward the received message to each joint */
	comau_edo::edo_msgs::EdoJointCtrl* jntCtrlMsg;
	
	for (uint8_t j = 0; j < msg.joints_length; j++)
	{
		const edo_core_msgs::JointControl* jntCtrl = &msg.joints[j];
		
		if(_static_joints[j]._publisher_ctrl.alloc(jntCtrlMsg)){
			jntCtrlMsg->position = jntCtrl->position;
			jntCtrlMsg->velocity = jntCtrl->velocity;
			jntCtrlMsg->acceleration = jntCtrl->current;
			jntCtrlMsg->feedfwvelocity = jntCtrl->ff_velocity;
			jntCtrlMsg->feedfwtorque = jntCtrl->ff_current;

			if(!_static_joints[j]._publisher_ctrl.publish(jntCtrlMsg)){
				// Handle error
				nh.logerror("usb joint ctrl pub");
			}
		}
	}
}

uint8_t ROSSerialNode::getJointIDsFromMask(uint64_t mask, uint8_t ids_array[], uint8_t size)
{
	uint8_t array_size = 0;
	uint8_t bit1 = 0x01;
	
	if(ids_array == nullptr)
		return 0;
	
	for(uint8_t i = 0; i < size; i++)
	{
		if( (mask & bit1) == bit1){ // joint i+1 is in mask
			ids_array[array_size] = i+1;
			array_size++;
		}
		mask = mask >> 1;
	}
	return array_size;
}

void
ROSSerialNode::callback_ros_jntconfig_(
	const edo_core_msgs::JointConfigurationArray &msg
)
{
	if(!_initialized){
		nh.logwarn("rx joint config not initialized");
		return;
	}
	
	if(msg.joints_length > _number_of_joints){
		nh.logwarn("rx joint config with wrong length");
		return;
	}
	/* Forward the received message to each joint */
	core::control_msgs::PID_param* jntConfigMsg;
	
	for (uint8_t j = 0; j < msg.joints_length; j++)
	{
		const edo_core_msgs::JointConfiguration* jntConfig = &msg.joints[j];
		
		if((uint8_t(msg.joints_mask >> j) & 0x01) != 0x01){
			//nh.loginfo("pid config: joint not in mask");
			continue;
		}
		
		if(_static_joints[j]._publisher_config.alloc(jntConfigMsg))
		{
			jntConfigMsg->kp = jntConfig->kp;
			jntConfigMsg->ti = jntConfig->ti;
			jntConfigMsg->td = jntConfig->td;
			jntConfigMsg->sat = jntConfig->sat;
			jntConfigMsg->kff = jntConfig->kff;
			jntConfigMsg->max = jntConfig->max;
			jntConfigMsg->kpv = jntConfig->kpv;
			jntConfigMsg->tiv = jntConfig->tiv;
			jntConfigMsg->tdv = jntConfig->tdv;
			jntConfigMsg->satv = jntConfig->satv;
			jntConfigMsg->kffv = jntConfig->kffv;
			jntConfigMsg->maxv = jntConfig->maxv;
			jntConfigMsg->kpt = jntConfig->kpt;
			jntConfigMsg->tit = jntConfig->tit;
			jntConfigMsg->tdt = jntConfig->tdt;
			jntConfigMsg->satt = jntConfig->satt;
			jntConfigMsg->kfft = jntConfig->kfft;
			jntConfigMsg->maxt = jntConfig->maxt;
			jntConfigMsg->kt = jntConfig->kt;

			_static_joints[j]._publisher_config.publish(jntConfigMsg);
		}
	}
}

void
ROSSerialNode::callback_ros_jntinit_(
	const edo_core_msgs::JointInit &msg
)
{
	uint64_t rx_mask = msg.joints_mask;
	if((rx_mask | JOINT_MASK) > JOINT_MASK){
		// Handle error: joint IDs not supported
		nh.logwarn("jnt_init: joint mask not supported");
		return;
	}
	
	switch (msg.mode){
		case DISCOVERY_MODE:
			uint8_t ids[64];
			_number_of_joints = getJointIDsFromMask(rx_mask, ids, uint8_t(64));
			if(!startStopJoints(ids, _number_of_joints)){
				// handle error
			}

			_jnt_state_pub_msg.joints_length = _number_of_joints;
			_joints_mask = rx_mask;
			if(!_initialized)
				_initialized = true;
			
		break;
		case SET_MODE:
			if(!_initialized){
				// Handle error
				nh.logwarn("jnt_init: set id not initialized");
				return;
			}				
		break;
		case CANCEL_MODE:
			if(!_initialized){
				// Handle error
				nh.logwarn("jnt_init: cancel id not initialized");
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
			nh.logwarn("jnt_init: error publish");
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
		// Handle error: joint IDs not supported
		nh.logwarn("jnt_calib: joint mask not supported");
		return;
	}
	
	/* Forward msg to joints */
	comau_edo::edo_msgs::EdoJointCalibration* CANmsg;
	if(publisher_calib.alloc(CANmsg)){
		CANmsg->joints_mask = msg.joints_mask;
		if(!publisher_calib.publish(CANmsg)){
			// Handle error
			nh.logwarn("jnt_calib: error publish");
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
		// Handle error: joint IDs not supported
		nh.logwarn("jnt_reset: joint mask not supported");
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
			nh.logwarn("jnt_reset: error publish");
		}
	}
}

void
ROSSerialNode::callback_ros_jntversion_(
	const std_msgs::UInt8 &msg
)
{
	nh.loginfo("Rx jnt version req");

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


	/* Send firmware version request to joints */
	comau_edo::edo_msgs::EdoJointVersion* CANmsg;
	if(_number_of_joints >= msg.data && publisher_version_req.alloc(CANmsg)){
		CANmsg->id = msg.data;
		if(!publisher_version_req.publish(CANmsg)){
			// Handle error
			nh.logwarn("jnt_version: error publish");
		}
	} else
		nh.logwarn("ev1");
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
		nh.logerror("Number of joints too high");
		return false;
	}
	for (uint8_t i = 0; i < MAX_JOINTS; i++)
	{
		bool found = false;
		for (uint8_t j = 0; j < number_of_joints; j++)
		{
			if(joint_ids[j] == _static_joints[i].get_id()){ // joint ID match
				if(_static_joints[i].state() != State::LOOPING && _static_joints[i].state() != State::STARTING)
					_static_joints[i].execute(Action::START);
				found = true;
				break;
			}
		}
		if(!found)
			_static_joints[i].execute(Action::STOP);
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
