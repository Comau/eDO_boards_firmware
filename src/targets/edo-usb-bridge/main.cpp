#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>
#include <core/common_msgs/String64.hpp>
#include <core/mw/BootMsg.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/led/Subscriber.hpp>
#include <core/led/Publisher.hpp>

// --- BOARD IMPL -------------------------------------------------------------

// --- MISC -------------------------------------------------------------------

// *** DO NOT MOVE ***
Module module;

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);
core::led::Publisher  led_publisher("led_publisher");

#include <core/mw/CoreMessage.hpp>

namespace bootloader_msgs {
CORE_MESSAGE_BEGIN(Raw48)
CORE_MESSAGE_FIELD(data, UINT8, 48)   // 48 bytes long payload
CORE_MESSAGE_END
}

bool
bootloader_callback(
    const core::mw::bootloader::BootMsg& msg,
    void*                                context
)
{
	if (msg.command == core::mw::bootloader::MessageType::IHEX_WRITE) {
		Module::led.write(1);
	}

	if (msg.command == core::mw::bootloader::MessageType::ACK) {
		Module::led.write(0);
        }
    return true;
}

/*
 * Test node.
 */
void
bootloader_sub_node(
    void* arg
)
{
    core::mw::Node node("boot_sub");
    core::mw::Publisher<core::mw::bootloader::BootMsg>     pub;
    core::mw::Subscriber<core::mw::bootloader::BootMsg, 5> sub;

    node.subscribe(sub, "BOOTLOADER");

    sub.set_callback(bootloader_callback);

    for (;;) {
        node.spin(core::os::Time::ms(500));
    }
}

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

        // Led publisher node
        core::led::PublisherConfiguration led_publisher_configuration;
        led_publisher_configuration.topic = "led";
        led_publisher_configuration.led   = 1;
        led_publisher.setConfiguration(led_publisher_configuration);
        module.nodes().add(led_publisher);

        // Led subscriber node
        core::led::SubscriberConfiguration led_subscriber_configuration;
        led_subscriber_configuration.topic = "led";
        led_subscriber.setConfiguration(led_subscriber_configuration);
        module.nodes().add(led_subscriber);

        // ... and let's play!
        module.nodes().setup();
        module.nodes().run();

        core::os::Thread::create_heap(NULL, 2048, core::os::Thread::PriorityEnum::NORMAL + 5, bootloader_sub_node, nullptr);

        // Is everything going well?
        for (;;) {
            if (!module.nodes().areOk()) {
                module.halt("This must not happen!");
            }

            module.keepAlive();

            core::os::Thread::sleep(core::os::Time::ms(500));
        }

        return core::os::Thread::OK;
    } // main
}
