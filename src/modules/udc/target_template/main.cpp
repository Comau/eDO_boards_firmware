/* uDC Module template file
 *
 */

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- BOARD IMPL -------------------------------------------------------------
#include <core/hw/PWM.hpp>
#include <core/hw/ADC.hpp>
#include <core/hw/GPIO.hpp>

// --- MODULE -----------------------------------------------------------------
Module module;

// *** DO NOT MOVE THE CODE ABOVE THIS COMMENT *** //

// --- DRIVERS ----------------------------------------------------------------
#include <core/QEI_driver/QEI.hpp>
#include <core/MC33926_driver/MC33926.hpp>
#include <core/basic_sensors/TimeAverage.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/sensor_publisher/Publisher.hpp>
#include <core/actuator_subscriber/Subscriber.hpp>
#include <core/led/Subscriber.hpp>

// --- DRIVERS ----------------------------------------------------------------
core::QEI_driver::QEI
qei(
    Module::qei1
);


core::QEI_driver::QEI_Position encoder("qei", qei);
core::MC33926_driver::MC33926
mc33926(
    Module::hbridge_in1,
    Module::hbridge_in2,
    Module::hbridge_enable,
    Module::hbridge_d1
);


core::MC33926_driver::MC33926_SignMagnitude hbridge("hbridge", mc33926);
core::basic_sensors::TimeAverage average_current("avg_i");

// --- TYPES ------------------------------------------------------------------
using QEI_Publisher  = core::sensor_publisher::Publisher_<core::QEI_driver::QEI_Position>;
using I_Publisher    = core::sensor_publisher::Publisher_<core::basic_sensors::TimeAverage>;
using PWM_Subscriber = core::actuator_subscriber::Subscriber_<core::MC33926_driver::MC33926_SignMagnitude>;

// --- CONFIGURATIONS ---------------------------------------------------------
core::led::SubscriberConfiguration led_subscriber_configuration_default;
core::QEI_driver::QEI_PositionConfiguration   encoder_configuration_default;
core::basic_sensors::TimeAverageConfiguration average_current_configuration_default;
QEI_Publisher::ConfigurationType  encoder_publisher_configuration_default;
I_Publisher::ConfigurationType    i_publisher_configuration_default;
PWM_Subscriber::ConfigurationType pwm_subscriber_configuration_default;

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_sub", core::os::Thread::PriorityEnum::LOWEST);
QEI_Publisher         encoder_publisher("enc_pub", encoder, core::os::Thread::PriorityEnum::NORMAL);
I_Publisher           i_publisher("i_pub", average_current, core::os::Thread::PriorityEnum::NORMAL);
PWM_Subscriber        motor_subscriber("pwm_sub", hbridge, core::os::Thread::PriorityEnum::NORMAL);

// --- DEVICE CONFIGURATION ---------------------------------------------------
core::hw::QEI::Configuration qei_configuration = {
    QEI_MODE_QUADRATURE, QEI_BOTH_EDGES, QEI_DIRINV_FALSE,
};

core::hw::PWMMaster::Configuration pwm_configuration = {
    36000000,/* 36MHz PWM clock.   */
    4096,   /* 12-bit PWM, 9KHz frequency. */
    nullptr,
    {
        {PWM_OUTPUT_ACTIVE_HIGH,NULL                     },
        {PWM_OUTPUT_ACTIVE_HIGH,NULL                     },
        {PWM_OUTPUT_DISABLED,NULL                     },
        {PWM_OUTPUT_DISABLED,NULL                     }
    },
    0,
};

core::hw::ADCConversionGroup::Configuration current_sense_adc_configuration = {
    TRUE,   // circular
    0,   // num channels
    nullptr,   // end callback
    nullptr,   // error callback
    ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(10),   // CFGR
    ADC_TR(0, 4095),                                  // TR1
    {                                                 // SMPR[2]
        ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_2P5),
        0
    },
    {                                                // SQR[4]
        ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1),
        0,
        0,
        0
    },
};

// --- MAIN -------------------------------------------------------------------
extern "C" {
    int
    main()
    {
        module.initialize();

        module.encoder1.setMode(Module::Encoder1::Mode::QEI_GPIO);
        module.encoder2.setMode(Module::Encoder2::Mode::GPIO);

        // Device configurations
        module.qei1.start(qei_configuration);

        module.current_sense_adc.start(current_sense_adc_configuration);
        module.current_sense_adc.setChannelCallback(0, [&](core::hw::ADCConversionGroup::SampleType x) {
            core::os::SysLock::ISRScope lock;
            float tmp = (float)(x) * (3.3f / (4095.0f * 330.0f * 0.0024f)); //3.3V 330ohm 0.24% High side current
            average_current.addSample(tmp);
        });

        module.pwm.start(pwm_configuration);
        core::hw::PWM_1::driver->tim->CR1 &= ~STM32_TIM_CR1_CEN; // Disable the counter
        core::hw::PWM_1::driver->tim->CR1 |= STM32_TIM_CR1_CMS(2); // Set center aligned mode

        core::hw::PWM_1::driver->tim->CR2 &= ~STM32_TIM_CR2_MMS2_MASK;
        core::hw::PWM_1::driver->tim->CR2 |= STM32_TIM_CR2_MMS2(7);

        core::hw::PWM_1::driver->tim->CCMR2 &= ~STM32_TIM_CCMR2_OC4M_MASK;
        core::hw::PWM_1::driver->tim->CCMR2  = STM32_TIM_CCMR2_OC4M(7);

        core::hw::PWM_1::driver->tim->EGR  |= STM32_TIM_EGR_CC4G;
        core::hw::PWM_1::driver->tim->DIER |= STM32_TIM_DIER_CC4IE;

        core::hw::PWM_1::driver->tim->CCR[3] = 2048;

        core::hw::PWM_1::driver->tim->CR1 |= STM32_TIM_CR1_CEN; // Enable the counter

        // Default configuration
        encoder_configuration_default.period = 100;
        encoder_configuration_default.ticks  = 1024;
        encoder_configuration_default.invert = 0;
        average_current_configuration_default.period  = 100;
        led_subscriber_configuration_default.topic    = "led";
        encoder_publisher_configuration_default.topic = "encoder";
        i_publisher_configuration_default.topic       = "i";
        pwm_subscriber_configuration_default.topic    = "pwm";

        // Add configurable objects to the configuration manager...
        module.configurations().add(encoder, encoder_configuration_default);
        module.configurations().add(average_current, average_current_configuration_default);
        module.configurations().add(led_subscriber, led_subscriber_configuration_default);
        module.configurations().add(encoder_publisher, encoder_publisher_configuration_default);
        module.configurations().add(i_publisher, i_publisher_configuration_default);
        module.configurations().add(motor_subscriber, pwm_subscriber_configuration_default);

        // ... and load the configuration
        module.configurations().loadFrom(module.configurationStorage());

        // Add nodes to the node manager...
        module.nodes().add(led_subscriber);
        module.nodes().add(encoder_publisher);
        module.nodes().add(i_publisher);
        module.nodes().add(motor_subscriber);

        // ... and let's play!
        module.nodes().setup();
        module.nodes().run();

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
