/* Comau e.DO Joint module template file
 *
 */

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- BOARD IMPL -------------------------------------------------------------
#include <core/hw/PWM.hpp>
#include <core/hw/ADC.hpp>
#include <core/hw/GPIO.hpp>
#include <core/hw/I2C.hpp>

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
core::basic_sensors::TimeAverage average_current2("avg_i2");

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
        ADC_SQR1_SQ1_N(ADC_CHANNEL_IN4), //IN3 = opamp 4 = inut
        0,
        0,
        0
    },
};

core::hw::ADCConversionGroup::Configuration current_sense2_adc_configuration = {
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
        ADC_SQR1_SQ1_N(ADC_CHANNEL_IN3),
        0,
        0,
        0
    },
};

core::hw::ADCConversionGroup::Configuration voltage_sense_adc_configuration = {
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
        ADC_SQR1_SQ1_N(ADC_CHANNEL_IN5),
        0,
        0,
        0
    },
};

core::hw::ADCConversionGroup::Configuration encoder_temp_adc_configuration = {
    TRUE,   // circular
    0,   // num channels
    nullptr,   // end callback
    nullptr,   // error callback
    ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(10),   // CFGR
    ADC_TR(0, 4095),                                  // TR1
    {                                                 // SMPR[2]
        ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_2P5) | ADC_SMPR1_SMP_AN2(ADC_SMPR_SMP_2P5),
        0
    },
    {                                                // SQR[4]
        ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN2),
        0,
        0,
        0
    },
};
#include "ch.h"

static const I2CConfig _i2c_config = {
  STM32_TIMINGR_PRESC(0x00) |
  STM32_TIMINGR_SCLDEL(12) | STM32_TIMINGR_SDADEL(0) |
  STM32_TIMINGR_SCLH(140)  | STM32_TIMINGR_SCLL(206),
  0,
  0
};
// --- MAIN -------------------------------------------------------------------
extern "C" {
    int
    main()
    {
        module.initialize();

#if 1
        // Check IMU
        
        core::hw::I2CMaster_<core::hw::I2C_1> i2c;
        i2c.start(_i2c_config);

        i2c.acquireBus();
        uint8_t command = 0x0F;
        uint8_t         _rxBuffer[6];
        volatile bool success = true;
        success &= i2c.exchange(0b1101010, 1, (uint8_t*)(&command), 1, _rxBuffer, core::os::Time::ms(500));
        i2c.releaseBus();

        module.led_yellow.clear();

        if(_rxBuffer[0] == 105) {
        	module.led_yellow.set();
        }
#endif

        module.encoder.setMode(Module::Encoder::Mode::QEI_GPIO);

        // Device configurations
        module.qei1.start(qei_configuration);

#if 0
        OPAMP2->CSR &= ~OPAMP2_CSR_OPAMP2EN; // Turn it off

        OPAMP2->CSR &= ~OPAMP2_CSR_FORCEVP; // Normal mode
        OPAMP2->CSR &= ~OPAMP2_CSR_VPSEL;
        OPAMP2->CSR |= OPAMP2_CSR_VPSEL_1 | OPAMP2_CSR_VPSEL_0; // PA7
        OPAMP2->CSR &= ~OPAMP2_CSR_VMSEL;
        OPAMP2->CSR |= OPAMP2_CSR_VMSEL_1; // PGA Mode
        OPAMP2->CSR &= ~OPAMP2_CSR_TCMEN; // No timer control
        OPAMP2->CSR &= ~OPAMP2_CSR_CALON; // Normal mode
        OPAMP2->CSR &= ~OPAMP2_CSR_USERTRIM; // Factory trimming

        OPAMP2->CSR &= ~OPAMP2_CSR_PGGAIN;
        //OPAMP2->CSR |= (0b1101) << 14; // Gain 4, feedback on PA5
        OPAMP2->CSR |= OPAMP2_CSR_PGGAIN_0;

        OPAMP2->CSR |= OPAMP2_CSR_OPAMP2EN; // Turn it on
#endif

        module.current_sense_adc.start(current_sense_adc_configuration);
        module.current_sense_adc.setChannelCallback(0, [&](core::hw::ADCConversionGroup::SampleType x) {
            core::os::SysLock::ISRScope lock;
            float tmp = (float)(x);
            average_current.addSample(tmp);
        });

        module.current_sense2_adc.start(current_sense2_adc_configuration);
        module.current_sense2_adc.setChannelCallback(0, [&](core::hw::ADCConversionGroup::SampleType x) {
            core::os::SysLock::ISRScope lock;
            float tmp = (float)(x) * (3.3f / (4095.0f * 330.0f * 0.0024f)); //3.3V 330ohm 0.24% High side current
            average_current2.addSample(tmp);
        });

        module.voltage_sense_adc.start(voltage_sense_adc_configuration);
        module.voltage_sense_adc.setChannelCallback(0, [&](core::hw::ADCConversionGroup::SampleType x) {
            core::os::SysLock::ISRScope lock;
            float tmp = (float)(x);
        });

        module.encoder_temp_adc.start(encoder_temp_adc_configuration);
        module.encoder_temp_adc.setChannelCallback(0, [&](core::hw::ADCConversionGroup::SampleType x) {
            core::os::SysLock::ISRScope lock;
            float tmp = (float)(x);
        });
        module.encoder_temp_adc.setChannelCallback(1, [&](core::hw::ADCConversionGroup::SampleType x) {
            core::os::SysLock::ISRScope lock;
            float tmp = (float)(x);
//            average_current.addSample(tmp);
        });

        module.pwm.start(pwm_configuration);
        core::hw::PWM_1::driver->tim->CR1 &= ~STM32_TIM_CR1_CEN; // Disable the counter
        core::hw::PWM_1::driver->tim->CR1 |= STM32_TIM_CR1_CMS(0); // Set center aligned mode

        core::hw::PWM_1::driver->tim->CR2 &= ~STM32_TIM_CR2_MMS2_MASK;
        core::hw::PWM_1::driver->tim->CR2 |= STM32_TIM_CR2_MMS2(7);

        core::hw::PWM_1::driver->tim->CCMR2 &= ~STM32_TIM_CCMR2_OC4M_MASK;
        core::hw::PWM_1::driver->tim->CCMR2  = STM32_TIM_CCMR2_OC4M(7);

        core::hw::PWM_1::driver->tim->EGR  |= STM32_TIM_EGR_CC4G;
        core::hw::PWM_1::driver->tim->DIER |= STM32_TIM_DIER_CC4IE;

        core::hw::PWM_1::driver->tim->CCR[3] = 128;

        core::hw::PWM_1::driver->tim->CR1 |= STM32_TIM_CR1_CEN; // Enable the counter

        // Default configuration
        encoder_configuration_default.period = 100;
        encoder_configuration_default.ticks  = 1024;
        encoder_configuration_default.invert = 0;
        average_current_configuration_default.period  = 100;
        led_subscriber_configuration_default.topic    = "led";
        encoder_publisher_configuration_default.topic = "encoder";
        i_publisher_configuration_default.topic       = "current";
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
