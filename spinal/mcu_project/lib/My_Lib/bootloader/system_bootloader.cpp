#include "bootloader/system_bootloader.h"

#include "stm32h7xx_hal.h"

namespace
{

constexpr uint32_t kApplicationBootAddress = 0x08000000UL;
constexpr uint32_t kSystemMemoryBootAddress = 0x1ff00000UL;

}  // namespace

namespace SystemBootloader
{

void restore_application_boot_address()
{
  // BOOT_ADD0 in SYSCFG->UR2 is a volatile shadow register, not a permanent
  // option-byte write. Restore it whenever the application starts so later
  // software resets keep selecting application Flash.
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_SYSCFG_CM7BootAddConfig(SYSCFG_BOOT_ADDR0, kApplicationBootAddress);
  __DSB();
}

[[noreturn]] void request_and_reset()
{
  // A direct branch into newer H743 ROM bootloaders can ACK UART activation
  // while NACKing every command. Changing the volatile boot-address shadow and
  // resetting enters the same hardware boot path as asserting BOOT0.
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_SYSCFG_CM7BootAddConfig(SYSCFG_BOOT_ADDR0, kSystemMemoryBootAddress);
  __DSB();
  NVIC_SystemReset();
  while (true) {}
}

}  // namespace SystemBootloader
