/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Subscriber.hpp>

#include <core/joint/SubscriberConfiguration.hpp>
#include <core/common_msgs/Float32.hpp>
#include <string.h>
#include <ros.h>
#include <core/joint/Joint.hpp>
#include <edo_core_msgs/JointControl.h>
#include <edo_core_msgs/JointControlArray.h>
#include <edo_core_msgs/JointState.h>
#include <edo_core_msgs/JointStateArray.h>
#include <edo_core_msgs/JointConfigurationArray.h>
#include <edo_core_msgs/JointConfiguration.h>
#include <edo_core_msgs/JointInit.h>
#include <edo_core_msgs/JointCalibration.h>
#include <edo_core_msgs/JointReset.h>
#include <comau_edo/edo_msgs/EdoJointCtrl.hpp>
#include <comau_edo/edo_msgs/EdoJointInit.hpp>
#include <comau_edo/edo_msgs/EdoJointReset.hpp>
#include <comau_edo/edo_msgs/EdoJointCalibration.hpp>
#include <comau_edo/edo_msgs/EdoJointVersion.hpp>
#include <core/control_msgs/PID_param.hpp>
#include <edo_core_msgs/JointFwVersion.h>
#include <edo_core_msgs/JointFwVersionArray.h>
#include <std_msgs/UInt8.h>

#define MAX_JOINTS	10
#define JOINT_MASK	0x000000000000003FF


namespace core {
namespace joint {
class ROSSerialNode:
    public core::mw::CoreNode,
    public core::mw::CoreConfigurable<SubscriberConfiguration>
{
public:
	// Publishers and subscribers
	core::mw::Publisher<comau_edo::edo_msgs::EdoJointInit> publisher_init;
	core::mw::Publisher<comau_edo::edo_msgs::EdoJointReset> publisher_reset;
	core::mw::Publisher<comau_edo::edo_msgs::EdoJointCalibration> publisher_calib;
	core::mw::Publisher<comau_edo::edo_msgs::EdoJointVersion> publisher_version_req;
	core::mw::Subscriber<comau_edo::edo_msgs::EdoJointVersion, MAX_JOINTS> subscriber_jnt_version;
	
	ROSSerialNode(
		core::joint::Joint (&joints)[MAX_JOINTS],
        const char*                name,
        core::os::Thread::Priority priority = core::os::Thread::PriorityEnum::NORMAL
    );
	
    virtual
    ~ROSSerialNode();

    void
    callback_ros_jntctrl_(
        const edo_core_msgs::JointControlArray& msg
    );
	
	void
    callback_ros_jntconfig_(
        const edo_core_msgs::JointConfigurationArray& msg
	);
	
	void
    callback_ros_jntinit_(
        const edo_core_msgs::JointInit& msg
	);
	
	void
    callback_ros_jntcalib_(
        const edo_core_msgs::JointCalibration& msg
	);
	
	void
    callback_ros_jntreset_(
        const edo_core_msgs::JointReset& msg
	);
	
	void
    callback_ros_jntversion_(
        const std_msgs::UInt8 & msg
	);
	
	void
	setCtrlSub(ros::Subscriber<edo_core_msgs::JointControlArray> *_ros_sub_jntctrl);
	
	void
	setConfigSub(ros::Subscriber<edo_core_msgs::JointConfigurationArray> *_ros_sub_jntconfig);
	
	void
	setInitSub(ros::Subscriber<edo_core_msgs::JointInit> *_ros_sub_jntinit);
	
	void
	setCalibSub(ros::Subscriber<edo_core_msgs::JointCalibration> *_ros_sub_jntcalib);
	
	void
	setResetSub(ros::Subscriber<edo_core_msgs::JointReset> *_ros_sub_jntreset);
	
	void
	setVersionSub(ros::Subscriber<std_msgs::UInt8> *_ros_sub_jntversion);

	ros::Publisher *_jnt_fw_version_pub;
	edo_core_msgs::JointFwVersion _jnt_fw_version_pub_msg;

private:
	
	core::joint::Joint **_joints;
	core::joint::Joint (&_static_joints)[MAX_JOINTS];

	ros::NodeHandle nh;
	ros::Publisher *_jnt_state_pub;
	edo_core_msgs::JointStateArray _jnt_state_pub_msg;
	
	bool _initialized;
	uint64_t _joints_mask;
	uint8_t _number_of_joints;
	
	core::os::Time _last_publish_timestamp;
	
	core::mw::CoreNodeManager joints_manager;

private:
	bool startJoints();
	
	bool startStopJoints(uint8_t ids_array[], uint8_t number_of_joints);
	
	uint8_t
	getJointIDsFromMask(uint64_t mask, uint8_t ids_array[], uint8_t size);
	
	bool
	onConfigure();

	bool
	onPrepareMW();

	bool
	onLoop();

};
}
}
