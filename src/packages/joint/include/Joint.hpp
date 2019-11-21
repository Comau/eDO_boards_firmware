/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/Publisher.hpp>

#include <core/joint/SubscriberConfiguration.hpp>
#include <core/common_msgs/Float32.hpp>
#include <comau_edo/edo_msgs/EdoJointCtrl.hpp>
#include <core/control_msgs/PID_param.hpp>
#include <core/control_msgs/Encoder_State.hpp>
#include <string.h>
#include <stdio.h>
#include <comau_edo/edo_msgs/EdoJointVersion.hpp>
#ifndef MAX_JOINTS
// See also ROSSerialNode.hpp
#define MAX_JOINTS  7  /* Create a limited amount of node servers on the USB board */
#define JOINT_MASK  0x0000000000000007F /* Mask consistent with the MAX_JOINT definition */
#endif
//#include <core/joint/ROSSerialNode.hpp>
#define J_ACK_TE 0
#define J_ACK_FE 1
#define J_ERR_TE 2
#define J_ERR_FE 3
#define J_STATE  4

namespace core {
namespace joint {

class ROSSerialNode;

class Joint:
	public core::mw::CoreNode,
	public core::mw::CoreConfigurable<SubscriberConfiguration>
{

public:
	Joint(const char* name, 
		core::os::Thread::Priority priority = core::os::Thread::PriorityEnum::NORMAL,
		uint8_t id = 0
	);
	
	virtual	~Joint();

	void set_id(uint8_t id);

	uint8_t	get_id(void);

	bool set_state_topic(const char * topic, size_t size);
	
	bool set_ctrl_topic(const char * topic, size_t size);
	
	bool set_config_topic(const char * topic, size_t size);
	
	bool set_version_topic(const char * topic, size_t size);

	void set_ros_node(core::joint::ROSSerialNode *node_ptr);

	const float & getPos(void);
	const float & getVel(void);
	const float & getCurrent(void);
	const uint8_t & getCommandFlag(void);
	const float & getCurRes(void);
	const uint8_t & getCommandFlagPrev(void);
	const bool & stateUpdated(void);
	void setStateUpdated(const bool & updated);
	void setCommandFlagPrev(const uint8_t & updated);
	const char * compileLogInfoMsg(uint8_t msgType, uint8_t data);

// Publishers and subscribers
	core::mw::Subscriber<core::control_msgs::Encoder_State, (2 * MAX_JOINTS)> _subscriber_state;
	core::mw::Publisher<comau_edo::edo_msgs::EdoJointCtrl> _publisher_ctrl;
	core::mw::Publisher<core::control_msgs::PID_param> _publisher_config;
	core::mw::Subscriber<comau_edo::edo_msgs::EdoJointVersion, 2> _subscriber_version;

private:

	uint8_t _id;
	float   _pos;
	float   _vel;
	float   _current;
	uint8_t _commandFlag; // ack
	float   _cur_res_joint;
	uint8_t _commandFlagPrev; // Previous image of the command flag
	bool    _state_updated;
	
	char _state_topic[10];
	char _ctrl_topic[10];
	char _config_topic[10];
	char _version_topic[16];
	char _msg[32];
	core::joint::ROSSerialNode *_rosNode;
// CoreNode events to override
private:

	bool
	onConfigure();

	bool
	onPrepareMW();

	bool
	onLoop();

	bool
	onStart();

	bool
	onStop();

	static bool
	callback_state_(
	const core::control_msgs::Encoder_State& msg,
	void*                             context
	);

	static bool
	callback_version_(
	const comau_edo::edo_msgs::EdoJointVersion& msg,
	void*                             context
	);
};
}
}
