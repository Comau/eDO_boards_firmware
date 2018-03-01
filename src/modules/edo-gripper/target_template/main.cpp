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
