/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <core/joint/Joint.hpp>

#include <core/mw/Middleware.hpp>
#include <core/common.hpp>
#include <core/joint/ROSSerialNode.hpp>

namespace core {
namespace joint {
Joint::Joint(
    const char*                name,
    core::os::Thread::Priority priority,
    uint8_t id
) :
    CoreNode::CoreNode(name, priority),
    CoreConfigurable(name),
    _id(id)
{
	_workingAreaSize = 256;
	_pos = 0.0;
	_vel = 0.0;
	_current = 0.0;
	_commandFlag = 0;
	_commandFlagPrev = 0;
	_state_updated = false;
	sprintf(_state_topic,   "j%d_state",   id);
	sprintf(_ctrl_topic,    "j%d_setpnt",  id);
	sprintf(_config_topic,  "j%d_param",   id);
	sprintf(_version_topic, "j%d_version", id);
}

Joint::~Joint()
{
	teardown();
}

bool
Joint::onConfigure()
{
    return true;
}

bool
Joint::onPrepareMW()
{
	_subscriber_state.set_callback(Joint::callback_state_);
	this->subscribe(_subscriber_state, _state_topic);
	this->advertise(_publisher_ctrl, _ctrl_topic);
	this->advertise(_publisher_config, _config_topic);
	// _subscriber_version.set_callback(Joint::callback_version_);
	// this->subscribe(_subscriber_version, _version_topic);
	
	return true;
}

inline bool
Joint::onLoop()
{
	if (!this->spin(core::os::Time::ms(2))) {
		// Handle error
    }
	
    return true;
}

bool
Joint::onStart()
{
    return true;
}

bool
Joint::onStop()
{
    return true;
}

bool
Joint::callback_state_(
    const core::control_msgs::Encoder_State& msg,
    void*                             context
)
{
  Joint* _this = static_cast<Joint*>(context);

  _this->_pos = msg.position;
  _this->_vel = msg.velocity;
  _this->_current = msg.current;
  _this->_commandFlag = msg.commandFlag;
  _this->_state_updated = true;

  return true;
}

bool
Joint::callback_version_(
    const comau_edo::edo_msgs::EdoJointVersion& msg,
    void*                             context
)
{
    Joint* _this = static_cast<Joint*>(context);

    if(_this->_rosNode != nullptr){

      _this->_rosNode->_jnt_fw_version_pub_msg.id = msg.id;
      _this->_rosNode->_jnt_fw_version_pub_msg.majorRev = msg.major;
      _this->_rosNode->_jnt_fw_version_pub_msg.minorRev = msg.minor;
      _this->_rosNode->_jnt_fw_version_pub_msg.revision = msg.revision;
      _this->_rosNode->_jnt_fw_version_pub_msg.svn = msg.svn;

      if (_this->_rosNode->_jnt_fw_version_pub != nullptr)
        return _this->_rosNode->_jnt_fw_version_pub->publish(&_this->_rosNode->_jnt_fw_version_pub_msg);
    }

    return true;
}

void
Joint::set_id(
		uint8_t id
)
{
	_id = id;
}

uint8_t
Joint::get_id(
		void
)
{
	return _id;
}

void
Joint::setStateUpdated(
		const bool & updated
)
{
	_state_updated = updated;
}

void 
Joint::setCommandFlagPrev(
    const uint8_t & updated
)
{
	_commandFlagPrev = updated;
}

const bool &
Joint::stateUpdated(
		void
)
{
	return _state_updated;
}

bool
Joint::set_state_topic(
		const char * topic, size_t size
)
{
	if(size >= sizeof(_state_topic))
		return false;
	strcpy(_state_topic, topic);
	return true;
}

bool
Joint::set_ctrl_topic(
		const char * topic, size_t size
)
{
	if(size >= sizeof(_ctrl_topic))
		return false;
	strcpy(_ctrl_topic, topic);
	return true;
}

bool
Joint::set_config_topic(
		const char * topic, size_t size
)
{
	if(size >= sizeof(_config_topic))
		return false;
	strcpy(_config_topic, topic);
	return true;
}

bool
Joint::set_version_topic(
		const char * topic, size_t size
)
{
	if(size >= sizeof(_version_topic))
		return false;
	strcpy(_version_topic, topic);
	return true;
}

const float & Joint::getPos()
{
	return _pos;
}

const float & Joint::getVel()
{
	return _vel;
}

const float & Joint::getCurrent()
{
	return _current;
}

const uint8_t & Joint::getCommandFlag()
{
	return _commandFlag;
}

const uint8_t & Joint::getCommandFlagPrev()
{
	return _commandFlagPrev;
}

static const char *spc_msgTypes[] = {
  "J ACK(%d) te %d", 
  "J ACK(%d) fe %d",
  "J ERR(%d) te %d",
  "J ERR(%d) fe %d",
  "J STATE(%d)  %d"
};

const char * Joint::compileLogInfoMsg(uint8_t msgType, uint8_t data)
{
	sprintf(_msg, spc_msgTypes[msgType], data, _id);
	return &(_msg[0]);
}

void Joint::set_ros_node(core::joint::ROSSerialNode  *node_ptr)
{
	_rosNode = node_ptr;
}
}
}
