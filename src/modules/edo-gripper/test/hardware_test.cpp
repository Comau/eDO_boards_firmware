#include <cstdio>

#include "ch.h"
#include "hal.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>
#include <r2p/msg/std_msgs.hpp>
#include <dynamixel/include/Dynamixel.hpp>
#include <dynamixel/include/DynamixelMsg.hpp>


static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = {
    1000000, 100, 60
};

r2p::Middleware r2p::Middleware::instance(MODULE_NAME, "BOOT_"MODULE_NAME);

/*
 * Test node
 */
msg_t
test_pub_node(
    void* arg
)
{
    r2p::Node node("test_pub");
    r2p::Publisher<r2p::String64Msg> pub;
    r2p::String64Msg* msgp;
    uint16_t*         uuid = (uint16_t*)0x1FFFF7AC;

    (void)arg;
    chRegSetThreadName("test_pub");

    node.advertise(pub, "test");

    while (!pub.alloc(msgp)) {
        chThdSleepMilliseconds(1000);
    }

    sprintf(msgp->data, "\n\n"MODULE_NAME " module [0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]", uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5]);
    pub.publish(msgp);
    chThdSleepMilliseconds(100);

    return CH_SUCCESS;
} // test_pub_node

/*
 * Dynamixel test node
 */
msg_t
dynamixel_test_node(
    void* arg
)
{
    r2p::Node node("dynamixel_test");
    r2p::Publisher<r2p::DynamixelScanMsg> scan_pub;
    r2p::DynamixelScanMsg* scan_msgp;

    (void)arg;
    chRegSetThreadName("dynamixel_test");

    palSetPadMode(GPIOB, 1, PAL_MODE_OUTPUT_PUSHPULL); // TXEN
    palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL); // RXEN
    palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7)); // UART3 TX
    palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7)); // UART3 RX

    /* Power the Dynamixel bus */
    palSetPad(GPIOA, GPIOA_SIGN);
    chThdSleepMilliseconds(100);

    node.advertise(scan_pub, "dynamixel_scan");

    dynamixel::Network dynet(&SD3, GPIOB, 1);
    uint8_t id = 0xFF;
    dynet.start(1000000);
    chThdSleepMilliseconds(100);

    while ((id = dynet.scan(id)) != 0xFF) {
        dynamixel::Servo* servo = new dynamixel::Servo(dynet, id);

        if (scan_pub.alloc(scan_msgp)) {
            scan_msgp->id       = id;
            scan_msgp->model    = servo->getModel();
            scan_msgp->firmware = servo->getFirmware();
            scan_pub.publish(scan_msgp);
        }

        servo->setLed(true);
        chThdSleepMilliseconds(200);
        servo->setLed(false);
        chThdSleepMilliseconds(10);
    }

    chThdSleepMilliseconds(100);

    for (bool led = true;; led = !led) {
        dynamixel::Servo* servo = 0;

        while ((servo = dynet.getNext(servo)) != 0) {
            servo->getTemperature();
        }

        chThdSleepMilliseconds(500);
    }

    return CH_SUCCESS;
} // dynamixel_test_node

/*
 * Application entry point.
 */
extern "C" {
    int
    main(
        void
    )
    {
        halInit();
        chSysInit();

        r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
        rtcantra.initialize(rtcan_config);
        r2p::Middleware::instance.start();

        r2p::ledsub_conf ledsub_conf = {
            "led"
        };
        r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);
        r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, test_pub_node, NULL);
        r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, dynamixel_test_node, NULL);

        for (;;) {
            r2p::Thread::sleep(r2p::Time::ms(500));
        }

        return CH_SUCCESS;
    } // main
}
