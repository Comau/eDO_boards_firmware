#define YOU_WANT_TO_USE_EDO_JOINT_MESSAGES true
#define YOU_WANT_TO_SET_THE_BAUDRATE false
#define YOU_WANT_TO_SET_THE_CENTER false

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

#include <cstdlib>

// --- BOARD IMPL -------------------------------------------------------------
#include <core/dynamixel_driver/Dynamixel.hpp>

// --- MODULE -----------------------------------------------------------------
Module module;

// *** DO NOT MOVE THE CODE ABOVE THIS COMMENT *** //

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>
#include <core/dynamixel_msgs/SwitchMsg.hpp>
#include <core/dynamixel_msgs/DynamixelCommandMsg.hpp>
#include <core/dynamixel_msgs/DynamixelRegisterMsg.hpp>
#include <core/dynamixel_msgs/DynamixelScanMsg.hpp>
#include <core/dynamixel_msgs/DynamixelStateMsg.hpp>

#if YOU_WANT_TO_USE_EDO_JOINT_MESSAGES
#include <comau_edo/edo_msgs/EdoJointCtrl.hpp>
#include <core/control_msgs/Encoder_State.hpp>
#include <comau_edo/edo_msgs/EdoJointVersion.hpp>
#include <comau_edo/edo_msgs/EdoJointInit.hpp>
#define J_STATE_ACK_INIT		1
#define J_STATE_ACK_CALIB		2
#define J_STATE_ACK_CONFIG		3
#define J_STATE_ACK_RESET		4
#define J_STATE_ACK_VERSION		5
#define EDO_JOINT_FW_MAJOR      2
#define EDO_JOINT_FW_MINOR      0
#define EDO_JOINT_FW_REVISION   2202
#define EDO_JOINT_FW_SVN        435
#endif

// --- NODES ------------------------------------------------------------------
#include <core/led/Subscriber.hpp>

// --- CONFIGURATIONS ---------------------------------------------------------
core::led::SubscriberConfiguration led_subscriber_configuration_default;

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_sub", core::os::Thread::PriorityEnum::LOWEST);

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>

void
dynamixel_node(
    void* arg
)
{
    (void)arg;

    core::mw::Node node("dynamixel_sub");
    core::mw::Publisher<core::dynamixel_msgs::DynamixelScanMsg>         scan_pub;
    core::mw::Publisher<core::dynamixel_msgs::DynamixelStateMsg>        state_pub;
    core::mw::Subscriber<core::dynamixel_msgs::DynamixelCommandMsg, 5>  command_sub;
    core::mw::Subscriber<core::dynamixel_msgs::DynamixelRegisterMsg, 5> register_sub;

    core::dynamixel_msgs::DynamixelScanMsg*     scan_msgp;
    core::dynamixel_msgs::DynamixelStateMsg*    state_msgp;
    core::dynamixel_msgs::DynamixelCommandMsg*  command_msgp;
    core::dynamixel_msgs::DynamixelRegisterMsg* register_msgp;
    systime_t last_state = 0;

    chRegSetThreadName("dynamixel_sub");

    node.advertise(scan_pub, "dynamixel_scan");
    node.advertise(state_pub, "dynamixel_state");
    node.subscribe(command_sub, "dynamixel_cmd");
    node.subscribe(register_sub, "dynamixel_reg");

    core::dynamixel_driver::Network& dynet = Module::servo_network;

    dynet.start(0);

    chThdSleepMilliseconds(100);

#if YOU_WANT_TO_SET_THE_BAUDRATE
    uint8_t baudrate = 1; // 1 = 1000000 7 = 250000
    dynet.writeData(0xFE, core::dynamixel_driver::address::BAUDRATE, &baudrate, 1);
#endif

    bool has_servo = false;

    while(!has_servo) {
        uint8_t id = 0xFF;

		while ((id = dynet.scan(id)) != 0xFF) {
			core::dynamixel_driver::Servo* servo = new core::dynamixel_driver::Servo(dynet, id);

			if (scan_pub.alloc(scan_msgp)) {
				scan_msgp->id       = id;
				scan_msgp->model    = servo->getModel();
				scan_msgp->firmware = servo->getFirmware();
				scan_pub.publish(scan_msgp);
			}

			servo->enable();
			has_servo = true;
		}

		if(!has_servo) {
			Module::led_red.set();
            core::os::Thread::sleep(core::os::Time::ms(500));
			Module::led_red.clear();
		}
    }

    while (1) {
        systime_t now = chVTGetSystemTimeX();

        if (node.spin(core::os::Time::ms(1000))) {
            while (command_sub.fetch(command_msgp)) {
                core::dynamixel_driver::Servo* servo = dynet.get(command_msgp->id);

                if (servo) {
                    servo->setLed(true);
                    servo->setSpeed(command_msgp->speed);
                    servo->setPosition(command_msgp->position);
                }

                command_sub.release(*command_msgp);
            }
        } else {
            core::dynamixel_driver::Servo* servo = 0;

            while ((servo = dynet.getNext(servo)) != 0) {
                servo->setLed(false);
            }
        }

        while (register_sub.fetch(register_msgp)) {
            dynet.writeData(register_msgp->id, (core::dynamixel_driver::address)register_msgp->address, (uint8_t*)&register_msgp->data, register_msgp->length);
            register_sub.release(*register_msgp);
        }

        if ((now - last_state) > MS2ST(200)) {
            core::dynamixel_driver::Servo* servo = 0;

            while ((servo = dynet.getNext(servo)) != 0) {
                if (state_pub.alloc(state_msgp)) {
                    state_msgp->id          = servo->getId();
                    state_msgp->error       = servo->getError();
                    state_msgp->position    = servo->getPosition();
                    state_msgp->is_moving   = servo->getIsMoving();
                    state_msgp->temperature = servo->getTemperature();

                    state_pub.publish(state_msgp);
                }
            }
        }

        last_state = now;
    }
} // dynamixel_node

#if YOU_WANT_TO_USE_EDO_JOINT_MESSAGES
void
proxy_node(
    void* arg
)
{
    core::mw::Node node("proxy_node");
    core::mw::Subscriber<comau_edo::edo_msgs::EdoJointCtrl, 5>  edo_joint_sub;
    core::mw::Publisher<core::dynamixel_msgs::DynamixelCommandMsg> dynamixel_cmd_pub;
    core::mw::Publisher<core::control_msgs::Encoder_State> encoder_pub;
    core::mw::Publisher<comau_edo::edo_msgs::EdoJointVersion> version_pub;
    core::mw::Subscriber<comau_edo::edo_msgs::EdoJointVersion, 2> version_sub;
    core::mw::Subscriber<comau_edo::edo_msgs::EdoJointInit, 2> discovery_sub;
    int ackindex = 0;
    uint8_t ack = 0; //Acknowledge/Errors
    systime_t last_time = 0;
    bool version_req = false;

    node.subscribe(edo_joint_sub, "j7_setpnt");
    node.advertise(dynamixel_cmd_pub, "dynamixel_cmd");
    node.subscribe(version_sub, "joint_version");
    node.subscribe(discovery_sub, "joint_init");
    node.advertise(version_pub, "j7_version");
    node.advertise(encoder_pub, "j7_state");

    // Do not publish commands when not needed
    core::dynamixel_msgs::DynamixelCommandMsg last_command;
    // Trigger first publish
    last_command.id = 0;

    while (true) {
    	systime_t now = chVTGetSystemTimeX();

        if (node.spin(core::os::Time::ms(10))) {
        	comau_edo::edo_msgs::EdoJointCtrl* edo_joint;
        	comau_edo::edo_msgs::EdoJointInit* edo_discovery;
        	comau_edo::edo_msgs::EdoJointVersion* edo_version;

        	while(discovery_sub.fetch(edo_discovery)){
        		if(edo_discovery->mode == 0 && (edo_discovery->joints_mask & (1uL << (7 - 1)))){
        			ack |= J_STATE_ACK_INIT;
        			ackindex = 8;
        		}
        		discovery_sub.release(*edo_discovery);
        	}

        	while(version_sub.fetch(edo_version)){
        		if(edo_version->id == 7){
        			ack |= J_STATE_ACK_VERSION;
        			ackindex = 8;
        			version_req = true;
        		}
        		version_sub.release(*edo_version);
        	}

        	while(edo_joint_sub.fetch(edo_joint)) {
        		 core::dynamixel_msgs::DynamixelCommandMsg* dynamixel_cmd;
        		 // Do not publish commands when not needed
        		 if((1 != last_command.id) || (edo_joint->position != last_command.position) || (edo_joint->velocity != last_command.speed)) {
					 if(dynamixel_cmd_pub.alloc(dynamixel_cmd)) {
						 dynamixel_cmd->id = 1;
						 dynamixel_cmd->position = edo_joint->position;
						 dynamixel_cmd->speed = edo_joint->velocity;

						 // Do not publish commands when not needed
						 last_command = *dynamixel_cmd;

						 dynamixel_cmd_pub.publish(dynamixel_cmd);
					 }
    			 }
        		 edo_joint_sub.release(*edo_joint);
        	 }
        }

        if(version_req){
        	version_req = false;
        	comau_edo::edo_msgs::EdoJointVersion* version;

			if (version_pub.alloc(version))
			{
				//Fill in the message fields
				version->id       = 7;
				version->major    = EDO_JOINT_FW_MAJOR;
				version->minor    = EDO_JOINT_FW_MINOR;
				version->revision = EDO_JOINT_FW_REVISION;
				version->svn      = EDO_JOINT_FW_SVN;
				//Public the message
				version_pub.publish(version);
			}
        }
        if ((now - last_time) > MS2ST(10)) {
        	core::control_msgs::Encoder_State* encoder;
			if (encoder_pub.alloc(encoder))
			{
				//Fill in the message fields
				encoder->position    = last_command.position;
				encoder->velocity    = last_command.speed;
				encoder->current     = 0; // TODO: add current reading
				if (ackindex > 0)
					ackindex--;
				else
					//...when zero, clear the ack bits
					ack &= 0xF0;
				encoder->commandFlag = ack;
				//Public the message
				encoder_pub.publish(encoder);
			}
			last_time = now;
        }
    }
}
#endif

#if YOU_WANT_TO_SET_THE_CENTER
float speed = 20.0f;
float position = 150.0f;

void
pub_node(
    void* arg
)
{
    core::mw::Node pub_node("dyn_p_node");
    core::mw::Publisher<core::dynamixel_msgs::DynamixelCommandMsg> pub_cmd;

    pub_node.advertise(pub_cmd, "dynamixel_cmd");

    while (true) {
    	core::dynamixel_msgs::DynamixelCommandMsg* msgp;

        if (pub_cmd.alloc(msgp)) {
            msgp->speed = speed;
            msgp->position = position;
            msgp->id = 1;
            pub_cmd.publish(*msgp);
        }

        core::os::Thread::sleep(core::os::Time::ms(100));
    }
} // pub_node
#endif

/*
 * Application entry point.
 */
extern "C" {
    int
    main(
        void
    )
    {
    	module.initialize();

        // Default configuration
        led_subscriber_configuration_default.topic = "led";

        // Add configurable objects to the configuration manager...
        module.configurations().add(led_subscriber, led_subscriber_configuration_default);

        // ... and load the configuration
        module.configurations().loadFrom(module.configurationStorage());

        // Add nodes to the node manager...
        module.nodes().add(led_subscriber);

        // ... and let's play!
        module.nodes().setup();
        module.nodes().run();

        // Custom nodes
        core::os::Thread::create_heap(nullptr, 2048, core::os::Thread::PriorityEnum::NORMAL + 1, dynamixel_node, nullptr);

#if YOU_WANT_TO_USE_EDO_JOINT_MESSAGES
        core::os::Thread::create_heap(nullptr, 2048, core::os::Thread::PriorityEnum::NORMAL + 1, proxy_node, nullptr);
#endif

#if YOU_WANT_TO_SET_THE_CENTER
        core::os::Thread::create_heap(nullptr, 2048, core::os::Thread::PriorityEnum::NORMAL + 1, pub_node, nullptr);
#endif

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
