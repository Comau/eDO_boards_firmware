/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/os/Time.hpp>

#include <core/sensor_msgs/Delta_f32.hpp>

namespace ModuleConfiguration {
// --- CONSTANTS --------------------------------------------------------------
#ifdef CORE_MODULE_NAME
static const char* MODULE_NAME __attribute__((unused)) = CORE_MODULE_NAME;
#else
static const char* MODULE_NAME __attribute__((unused)) = "UDC-X";
#endif

static const core::os::Time PUBLISHER_RETRY_DELAY = core::os::Time::ms(500);
static const core::os::Time SUBSCRIBER_SPIN_TIME  = core::os::Time::ms(2000);

static const std::size_t SUBSCRIBER_QUEUE_LENGTH = 5;

// --- TYPES ------------------------------------------------------------------
using QEI_DELTA_DATATYPE = core::sensor_msgs::Delta_f32;
}
