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
//
// --- DRIVERS ----------------------------------------------------------------
#include <core/QEI_driver/QEI.hpp>
#include <core/MC33926_driver/MC33926.hpp>
#include <core/basic_sensors/TimeAverage.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>

// --- NODES ------------------------------------------------------------------

#include <core/led/Subscriber.hpp>

//Added
#include <core/control_pid/ControlPID_Node.hpp>
#include <core/master_package/MasterNode.hpp>
#include <core/interpolation_package/InterpNode.hpp>

// --- DRIVERS ----------------------------------------------------------------
core::QEI_driver::QEI
qei(
    Module::qei1
);

//Modified: Position to delta
core::QEI_driver::QEI_Delta encoder("qei", qei);

core::MC33926_driver::MC33926
mc33926(
    Module::hbridge_in1,
    Module::hbridge_in2,
    Module::hbridge_enable,
    Module::hbridge_d1
);


core::MC33926_driver::MC33926_SignMagnitude hbridge("hbridge", mc33926);


// --- CONFIGURATIONS ---------------------------------------------------------
core::led::SubscriberConfiguration led_subscriber_configuration_default;
//Added
core::QEI_driver::QEI_Delta::ConfigurationType    encoder_configuration_default;
core::master_package::MasterNode::ConfigurationType master_node_configuration_default;
core::control_pid::ControlNode::ConfigurationType control_node_configuration_default;


// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_sub", core::os::Thread::PriorityEnum::LOWEST);
//Added
core::control_pid::ControlNode controller("control", encoder, hbridge, core::os::Thread::PriorityEnum::NORMAL + 1);
core::interpolation_package::InterpNode interpolator("interpolation", core::os::Thread::PriorityEnum::NORMAL + 1);
core::master_package::MasterNode master_node("Master", controller, interpolator, core::os::Thread::PriorityEnum::NORMAL);

// --- DEVICE CONFIGURATION ---------------------------------------------------
core::hw::QEI::Configuration qei_configuration = {
    QEI_MODE_QUADRATURE, QEI_BOTH_EDGES, QEI_DIRINV_FALSE,
};

core::hw::PWMMaster::Configuration pwm_configuration = {
    72000000,/* 72MHz PWM clock.   */
    4096,   /* 12-bit PWM, 18KHz frequency. */
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
    TRUE,   	// circular
    0,   		// num channels
    nullptr, 	// end callback
    nullptr, 	// error callback
    ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(10),	// CFGR
    ADC_TR(0, 4095),                   					// TR1
    {                    								// SMPR[2]
        ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_2P5),
        0
    },
    {                             						// SQR[4]
        ADC_SQR1_SQ1_N(ADC_CHANNEL_IN4), //IN3 = opamp 4 = inut
        0,
        0,
        0
    },
};

core::hw::ADCConversionGroup::Configuration current_sense2_adc_configuration = {
    TRUE,		// circular
    0, 			// num channels
    nullptr, 	// end callback
    nullptr, 	// error callback
    ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(10),	// CFGR
    ADC_TR(0, 4095),                       				// TR1
    {                                					// SMPR[2]
        ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_2P5),
        0
    },
    {                                   				// SQR[4]
        ADC_SQR1_SQ1_N(ADC_CHANNEL_IN3),
        0,
        0,
        0
    },
};

core::hw::ADCConversionGroup::Configuration voltage_sense_adc_configuration = {
    TRUE,		// circular
    0,			// num channels
    nullptr, 	// end callback
    nullptr, 	// error callback
    ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(10), 	// CFGR
    ADC_TR(0, 4095),                    				// TR1
    {                                   				// SMPR[2]
        ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_2P5),
        0
    },
    {                                  					// SQR[4]
        ADC_SQR1_SQ1_N(ADC_CHANNEL_IN5),
        0,
        0,
        0
    },
};

core::hw::ADCConversionGroup::Configuration encoder_temp_adc_configuration = {
    TRUE,		// circular
    0, 			// num channels
    nullptr,	// end callback
    nullptr, 	// error callback
    ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(10), 	// CFGR
    ADC_TR(0, 4095),                        			// TR1
    {                                  					// SMPR[2]
        ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_2P5) | ADC_SMPR1_SMP_AN2(ADC_SMPR_SMP_2P5),
        0
    },
    {                       							// SQR[4]
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

#if 0
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

        module.voltage_sense_adc.start(voltage_sense_adc_configuration);

        module.encoder_temp_adc.start(encoder_temp_adc_configuration);

        module.pwm.start(pwm_configuration);
        core::hw::PWM_1::driver->tim->CR1 &= ~STM32_TIM_CR1_CEN; // Disable the counter
        core::hw::PWM_1::driver->tim->CR1 |= STM32_TIM_CR1_CMS(0); // Set center aligned mode

        core::hw::PWM_1::driver->tim->CR2 &= ~STM32_TIM_CR2_MMS2_MASK;
        core::hw::PWM_1::driver->tim->CR2 |= STM32_TIM_CR2_MMS2(7);

        core::hw::PWM_1::driver->tim->CCMR2 &= ~STM32_TIM_CCMR2_OC4M_MASK;
		core::hw::PWM_1::driver->tim->CCMR2 |= STM32_TIM_CCMR2_OC4M(7) | STM32_TIM_CCMR2_OC4PE;

        core::hw::PWM_1::driver->tim->EGR  |= STM32_TIM_EGR_CC4G;
        core::hw::PWM_1::driver->tim->DIER |= STM32_TIM_DIER_CC4IE;

		core::hw::PWM_1::driver->tim->CCR[3] = 1;

        core::hw::PWM_1::driver->tim->CR1 |= STM32_TIM_CR1_CEN; // Enable the counter

        // Default configuration
        encoder_configuration_default.period = 1000;
        encoder_configuration_default.ticks  = 256*4;
        encoder_configuration_default.invert = 0;
        led_subscriber_configuration_default.topic    = "led";
		
        master_node_configuration_default.Configured = -1;
        master_node_configuration_default.Reduction  = 0.0f;

        control_node_configuration_default.pos_validity = 0;
        control_node_configuration_default.position = 0.0f;
		

        // Add configurable objects to the configuration manager...
        module.configurations().add(encoder, encoder_configuration_default);
        module.configurations().add(led_subscriber, led_subscriber_configuration_default);
        module.configurations().add(master_node,master_node_configuration_default);
        module.configurations().add(controller,control_node_configuration_default);
		


#if 0
	    master_node.overrideConfiguration();
	    controller.overrideConfiguration();
	    master_node.overridingConfiguration().Configured = 1;
	    master_node.overridingConfiguration().Reduction = -1232.0f;//992.0f
	    controller.overridingConfiguration().pos_validity = 0;
	    controller.overridingConfiguration().position = 0.0f;
	    module.configurations().saveTo(module.configurationStorage());
        module.configurations().saveTo(module.configurationStorage());
#endif


        // ... and load the configuration
        module.configurations().loadFrom(module.configurationStorage());

        encoder.overrideConfiguration();
	    master_node.overrideConfiguration();
	    controller.overrideConfiguration();
	    led_subscriber.overrideConfiguration();

        // Add nodes to the node manager...
        module.nodes().add(led_subscriber);
		module.nodes().add(controller);
		module.nodes().add(interpolator);
        module.nodes().add(master_node);

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
