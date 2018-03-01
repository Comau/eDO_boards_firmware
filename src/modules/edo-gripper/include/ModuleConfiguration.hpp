#pragma once

#include <core/os/Time.hpp>

namespace ModuleConfiguration {
// --- CONSTANTS --------------------------------------------------------------
#ifdef CORE_MODULE_NAME
static const char* MODULE_NAME __attribute__((unused)) = CORE_MODULE_NAME;
#else
static const char* MODULE_NAME __attribute__((unused)) = "DYNAMIXEL";
#endif

static const core::os::Time PUBLISHER_RETRY_DELAY = core::os::Time::ms(500);
static const core::os::Time SUBSCRIBER_SPIN_TIME  = core::os::Time::ms(2000);

static const std::size_t SUBSCRIBER_QUEUE_LENGTH = 5;
}
