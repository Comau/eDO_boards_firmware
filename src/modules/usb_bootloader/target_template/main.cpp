/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <nil.h>
#include <hal.h>

#include <core/bootloader/bootloader.hpp>
#include <core/bootloader/blinker.hpp>

#include <core/bootloader/hw/hw_utils.hpp>

THD_TABLE_BEGIN
THD_TABLE_ENTRY(
    bootloaderThreadWorkingArea,
    "Bootloader",
    bootloaderThread,
    NULL
)
THD_TABLE_ENTRY(
    blinkerThreadWorkingArea,
    "Blinker",
    blinkerThread,
    NULL
)
THD_TABLE_END

int
main(
    void
)
{
    hw::Watchdog::freezeOnDebug();
    hw::Watchdog::enable(hw::Watchdog::Period::_6400_ms);

    halInit();
    chSysInit();

    if (hw::getResetSource() == hw::ResetSource::WATCHDOG) {
        if ((hw::getNVR() != hw::Watchdog::Reason::NO_APPLICATION) && (hw::getNVR() != hw::Watchdog::Reason::USER_REQUEST) && (hw::getNVR() != hw::Watchdog::Reason::BOOT_APPLICATION)) {
            while (1) {
                // *********************************************** //
                // *** WE HAVE A REAL FAULT... DEAL WITH IT!!! *** //
                // *********************************************** //
            }
        }
    }

    while (true) {
        // ************************* //
        // *** DO NOT SLEEP HERE *** //
        // ************************* //
    }
} // main
