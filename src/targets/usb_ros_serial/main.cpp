/* USB Module template file
 *
 */

#include "hal.h"   // Just for using the yellow led
#include <ModuleConfiguration.hpp>
#include <Module.hpp>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

// --- BOARD IMPL -------------------------------------------------------------

// --- MODULE -----------------------------------------------------------------
Module module;

// *** DO NOT MOVE THE CODE ABOVE THIS COMMENT *** //

#include <stdio.h>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>
#include <core/common_msgs/Float32.hpp>
#include <edo_core_msgs/JointControl.h>
#include <edo_core_msgs/JointControlArray.h>
#include <edo_core_msgs/JointState.h>
#include <edo_core_msgs/JointStateArray.h>
#include <edo_core_msgs/JointConfigurationArray.h>
#include <edo_core_msgs/JointConfiguration.h>
#include <edo_core_msgs/JointInit.h>
#include <edo_core_msgs/JointCalibration.h>
#include <edo_core_msgs/JointReset.h>
#include <edo_core_msgs/JointFwVersionArray.h>

// --- NODES ------------------------------------------------------------------
#include <core/led/Publisher.hpp>
#include <core/led/Subscriber.hpp>
#include <core/joint/Joint.hpp>
#include <core/joint/ROSSerialNode.hpp>
#include <core/StringBuffer.hpp>

// --- TYPES ------------------------------------------------------------------

// --- CONFIGURATIONS ---------------------------------------------------------
core::led::PublisherConfiguration  led_publisher_configuration_default;
core::led::SubscriberConfiguration led_subscriber_configuration_default;

// --- NODES ------------------------------------------------------------------
core::led::Publisher  led_publisher("led_pub", core::os::Thread::PriorityEnum::LOWEST);
core::led::Subscriber led_subscriber("led_sub", core::os::Thread::PriorityEnum::LOWEST);
core::joint::Joint staticJoints[MAX_JOINTS] = {
   {"j1", core::os::Thread::PriorityEnum::NORMAL, 1}
#if MAX_JOINTS > 1
  ,{"j2", core::os::Thread::PriorityEnum::NORMAL, 2}
#if MAX_JOINTS > 2
  ,{"j3", core::os::Thread::PriorityEnum::NORMAL, 3}
#if MAX_JOINTS > 3
  ,{"j4", core::os::Thread::PriorityEnum::NORMAL, 4}
#if MAX_JOINTS > 4
  ,{"j5", core::os::Thread::PriorityEnum::NORMAL, 5}
#if MAX_JOINTS > 5
  ,{"j6", core::os::Thread::PriorityEnum::NORMAL, 6}
#if MAX_JOINTS > 6
  ,{"j7", core::os::Thread::PriorityEnum::NORMAL, 7}
#endif
#endif
#endif
#endif
#endif
#endif
};
core::joint::ROSSerialNode ros_node(staticJoints, "ros_serial_node", core::os::Thread::PriorityEnum::NORMAL);

// --- DEVICE CONFIGURATION ---------------------------------------------------

void msg_jnt_ctrl_cb(const edo_core_msgs::JointControlArray& msg)
{
  ros_node.callback_ros_jntctrl_(msg);
}

void msg_jnt_config_cb(const edo_core_msgs::JointConfigurationArray& msg)
{
  ros_node.callback_ros_jntconfig_(msg);
}

void msg_jnt_init_cb(const edo_core_msgs::JointInit& msg)
{
  ros_node.callback_ros_jntinit_(msg);
}

void msg_jnt_calibration_cb(const edo_core_msgs::JointCalibration& msg)
{
  ros_node.callback_ros_jntcalib_(msg);
}

void msg_jnt_reset_cb(const edo_core_msgs::JointReset& msg)
{
  ros_node.callback_ros_jntreset_(msg);
}

void msg_jnt_version_cb(const std_msgs::UInt8 & msg)
{
  ros_node.callback_ros_jntversion_(msg);
}

ros::Subscriber<edo_core_msgs::JointControlArray> subObjCtrl("algo_jnt_ctrl", &msg_jnt_ctrl_cb);
ros::Subscriber<edo_core_msgs::JointConfigurationArray> subObjConfig("machine_jnt_config", &msg_jnt_config_cb);
ros::Subscriber<edo_core_msgs::JointInit> subObjInit("machine_init", &msg_jnt_init_cb);
ros::Subscriber<edo_core_msgs::JointCalibration> subObjCalib("machine_jnt_calib", &msg_jnt_calibration_cb);
ros::Subscriber<edo_core_msgs::JointReset> subObjReset("machine_jnt_reset", &msg_jnt_reset_cb);
ros::Subscriber<std_msgs::UInt8> subObjVersion("machine_jnt_version", &msg_jnt_version_cb);


// --- MAIN -------------------------------------------------------------------
extern "C" {
    int
    main(
        void
    )
    {

        module.initialize();

        // Device configurations

        // Default configuration
        led_publisher_configuration_default.topic  = "led";
        led_publisher_configuration_default.led    = 1;
        led_subscriber_configuration_default.topic = "led";

        ros_node.setCtrlSub(&subObjCtrl);
        ros_node.setConfigSub(&subObjConfig);
        ros_node.setInitSub(&subObjInit);
        ros_node.setCalibSub(&subObjCalib);
        ros_node.setResetSub(&subObjReset);
        ros_node.setVersionSub(&subObjVersion);

        for (uint8_t j = 0; j < MAX_JOINTS; j++)
        {
          staticJoints[j].set_ros_node(&ros_node);
        }

        // Add configurable objects to the configuration manager...
        module.configurations().add(led_publisher, led_publisher_configuration_default);
        module.configurations().add(led_subscriber, led_subscriber_configuration_default);

        // ... and load the configuration
        module.configurations().loadFrom(module.configurationStorage());

        // Add nodes to the node manager...
        module.nodes().add(led_publisher);
        module.nodes().add(led_subscriber);
        module.nodes().add(ros_node);

        // ... and let's play!
        module.nodes().setup();
        module.nodes().run();
        palSetPad(GPIOA,3); // set del led giallo
        // Is everything going well?
        for (;;) {
            if (!module.nodes().areOk()) {
                module.halt("This must not happen!");
            }

            core::os::Thread::sleep(core::os::Time::ms(500));

            // Remember to feed the (watch)dog!
            module.keepAlive();
        }

        return core::os::Thread::OK;
    } // main
}
