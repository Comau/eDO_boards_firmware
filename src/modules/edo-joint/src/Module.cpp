/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <core/mw/Middleware.hpp>
#include <core/mw/transport/RTCANTransport.hpp>

#include <core/snippets/CortexMxFaultHandlers.h>

#include <core/hw/GPIO.hpp>
#include <core/hw/QEI.hpp>
#include <core/hw/PWM.hpp>
#include <core/hw/IWDG.hpp>
#include <core/os/Thread.hpp>

#include <Module.hpp>

#include <core/QEI_driver/QEI.hpp>
#include <core/MC33926_driver/MC33926.hpp>
#include <core/hw/ADC.hpp>
// LED
static core::hw::Pad_<core::hw::GPIO_A, 12> _led;
static core::hw::Pad_<core::hw::GPIO_A, 11> _led_yellow;

// ENCODER
static core::hw::QEI_<core::hw::QEI_3> _encoder_device;

// H-BRIDGE
core::hw::PWMMaster_<core::hw::PWM_1> _pwm_master;
core::hw::PWMChannel_<core::hw::PWMMaster_<core::hw::PWM_1>::PWM, 0> _pwm_channel_0;
core::hw::PWMChannel_<core::hw::PWMMaster_<core::hw::PWM_1>::PWM, 1> _pwm_channel_1;
core::hw::Pad_<core::hw::GPIO_B, 1>  _hbridge_enable;
core::hw::Pad_<core::hw::GPIO_A, 10> _hbridge_d1;
core::hw::Pad_<core::hw::GPIO_B, 10> _hbridge_d2;
core::hw::Pad_<core::hw::GPIO_B, 15> _hbridge_slew;
core::hw::Pad_<core::hw::GPIO_A, 4>  _hbridge_status_flag;

// RELAY
core::hw::Pad_<core::hw::GPIO_B, 2>  _brake;

// ENCODER1
core::hw::Pad_<core::hw::GPIO_B, 5> _e_a;
core::hw::Pad_<core::hw::GPIO_B, 4> _e_b;
core::hw::Pad_<core::hw::GPIO_B, 3> _e_i;
core::hw::Pad_<core::hw::GPIO_A, 0> _e_analog;

// ADC
static core::hw::ADCConversionGroup_<core::hw::ADC_1, 2, 1> _encoder_temp_adc;
static core::hw::ADCConversionGroup_<core::hw::ADC_2, 1, 1> _current_sense_adc;
static core::hw::ADCConversionGroup_<core::hw::ADC_3, 1, 1> _voltage_sense_adc;
static core::hw::ADCConversionGroup_<core::hw::ADC_4, 1, 1> _current_sense2_adc;

// MODULE DEVICES
core::hw::QEI&       Module::qei1 = _encoder_device;
core::hw::PWMMaster& Module::pwm  = _pwm_master;
core::hw::ADCConversionGroup& Module::current_sense_adc = _current_sense_adc;
core::hw::ADCConversionGroup& Module::current_sense2_adc = _current_sense2_adc;
core::hw::ADCConversionGroup& Module::voltage_sense_adc = _voltage_sense_adc;
core::hw::ADCConversionGroup& Module::encoder_temp_adc = _encoder_temp_adc;

core::hw::PWMChannel& Module::hbridge_in1         = _pwm_channel_0;
core::hw::PWMChannel& Module::hbridge_in2         = _pwm_channel_1;
core::hw::Pad&        Module::hbridge_enable      = _hbridge_enable;
core::hw::Pad&        Module::hbridge_d1          = _hbridge_d1;
core::hw::Pad&        Module::hbridge_d2          = _hbridge_d2;
core::hw::Pad&        Module::hbridge_slew        = _hbridge_slew;
core::hw::Pad&        Module::hbridge_status_flag = _hbridge_status_flag;

core::hw::Pad&        Module::brake = _brake;

core::hw::Pad& Module::Encoder::a      = _e_a;
core::hw::Pad& Module::Encoder::b      = _e_b;
core::hw::Pad& Module::Encoder::i      = _e_i;
core::hw::Pad& Module::Encoder::analog = _e_analog;

core::hw::Pad& Module::led_yellow = _led_yellow;

// SYSTEM STUFF
static core::os::Thread::Stack<1024> management_thread_stack;
static core::mw::RTCANTransport      rtcantra(&RTCAND1);

RTCANConfig rtcan_config = {
    1000000, 100, 60
};


Module::Module()
{}

bool
Module::initialize()
{
#ifdef _DEBUG
    	FAULT_HANDLERS_ENABLE(true);
#else
    	FAULT_HANDLERS_ENABLE(false);
#endif

    static bool initialized = false;

    if (!initialized) {
        core::mw::CoreModule::initialize();
        qeiInit();

        _led_yellow.clear();

        core::mw::Middleware::instance().initialize(name(), management_thread_stack, management_thread_stack.size(), core::os::Thread::LOWEST);
        rtcantra.initialize(rtcan_config, canID());
        core::mw::Middleware::instance().start();

        initialized = true;
    }

    return initialized;
} // Board::initialize

void
Module::Encoder::setMode(
    Module::Encoder::Mode mode
)
{
    switch (mode) {
      case Module::Encoder::Mode::QEI_ANALOG:
          _e_a.setMode(core::hw::Pad::Mode::ALTERNATE_2);
          _e_b.setMode(core::hw::Pad::Mode::ALTERNATE_2);
          _e_i.setMode(core::hw::Pad::Mode::INPUT_PULLUP);
          _e_analog.setMode(core::hw::Pad::Mode::INPUT_ANALOG);
          break;
      case Module::Encoder::Mode::GPIO_ANALOG:
          _e_a.setMode(core::hw::Pad::Mode::INPUT_PULLUP);
          _e_b.setMode(core::hw::Pad::Mode::INPUT_PULLUP);
          _e_i.setMode(core::hw::Pad::Mode::INPUT_PULLUP);
          _e_analog.setMode(core::hw::Pad::Mode::INPUT_ANALOG);
          break;
      case Module::Encoder::Mode::QEI_GPIO:
          _e_a.setMode(core::hw::Pad::Mode::ALTERNATE_2);
          _e_b.setMode(core::hw::Pad::Mode::ALTERNATE_2);
          _e_i.setMode(core::hw::Pad::Mode::INPUT_PULLUP);
          _e_analog.setMode(core::hw::Pad::Mode::INPUT_PULLUP);
          break;
      case Module::Encoder::Mode::GPIO:
      default:
          _e_a.setMode(core::hw::Pad::Mode::INPUT_PULLUP);
          _e_b.setMode(core::hw::Pad::Mode::INPUT_PULLUP);
          _e_i.setMode(core::hw::Pad::Mode::INPUT_PULLUP);
          _e_analog.setMode(core::hw::Pad::Mode::INPUT_PULLUP);
          break;
    } // switch
} // Module::Encoder::setMode

// ----------------------------------------------------------------------------
// CoreModule STM32FlashConfigurationStorage
// ----------------------------------------------------------------------------
#include <core/snippets/CoreModuleSTM32FlashConfigurationStorage.hpp>
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// CoreModule HW specific implementation
// ----------------------------------------------------------------------------
#include <core/snippets/CoreModuleHWSpecificImplementation.hpp>
// ----------------------------------------------------------------------------
