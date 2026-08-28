#pragma once

#include <cstdint>

namespace SystemBootloader
{

// Restore the volatile H7 boot-address shadow after the ROM bootloader starts
// the application.
void restore_application_boot_address();

// Select factory system memory in the volatile H7 boot-address shadow and
// reset the MCU into the hardware ROM boot path.
[[noreturn]] void request_and_reset();

}  // namespace SystemBootloader
