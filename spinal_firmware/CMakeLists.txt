cmake_minimum_required(VERSION 3.16)
project(spinal_firmware)

find_package(ament_cmake REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(STM32_PROGRAMMER "" CACHE FILEPATH
  "Optional path to STM32_Programmer_CLI (2.23 or newer)")
set(STM32_PROGRAMMER_DEB "" CACHE FILEPATH
  "Optional path to ST's STM32CubeProgrammer 2.23 ARM64 Debian package")
set(STM32_PROGRAMMER_DEB_URL
  "https://drive.usercontent.google.com/download?id=1gX2pNBjZ8pFm_DzgV1kgPRpxuWnluyDC&export=download&confirm=t"
  CACHE STRING "URL for the STM32CubeProgrammer 2.23 ARM64 Debian package")
set(STM32_PROGRAMMER_DEB_SHA256
  "46b844fd135627290d2d0af3d3897debfa4247ec6af7d1cea3e0e9b0fb0bd31d"
  CACHE STRING "SHA-256 for the STM32CubeProgrammer 2.23 ARM64 Debian package")

set(FIRMWARE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/firmware")
set(FIRMWARE_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/firmware_build")
set(FIRMWARE_ELF "${FIRMWARE_BUILD_DIR}/spinal.elf")
set(FIRMWARE_BIN "${FIRMWARE_BUILD_DIR}/spinal.bin")
set(FIRMWARE_HEX "${FIRMWARE_BUILD_DIR}/spinal.hex")

add_custom_target(check_stm32_tools
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/bootstrap_stm32_tools.sh" --check-only
  USES_TERMINAL
  VERBATIM
  COMMENT "Checking STM32 cross-build tools")

# This target is intentionally explicit: a normal colcon build must never run
# sudo or apt. Invoke setup_stm32_tools once on a new Ubuntu/Debian host.
add_custom_target(setup_stm32_tools
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/bootstrap_stm32_tools.sh"
  USES_TERMINAL
  VERBATIM
  COMMENT "Installing STM32 cross-build tools")

add_custom_target(check_stm32_programmer
  COMMAND "${CMAKE_COMMAND}" -E env
    "STM32_PROGRAMMER=${STM32_PROGRAMMER}"
    "STM32_PROGRAMMER_DEB=${STM32_PROGRAMMER_DEB}"
    "STM32_PROGRAMMER_DEB_URL=${STM32_PROGRAMMER_DEB_URL}"
    "STM32_PROGRAMMER_DEB_SHA256=${STM32_PROGRAMMER_DEB_SHA256}"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/bootstrap_stm32_programmer.sh"
    --check-only
  USES_TERMINAL
  VERBATIM
  COMMENT "Checking the STM32 programming tool")

add_custom_target(setup_stm32_programmer
  COMMAND "${CMAKE_COMMAND}" -E env
    "STM32_PROGRAMMER=${STM32_PROGRAMMER}"
    "STM32_PROGRAMMER_DEB=${STM32_PROGRAMMER_DEB}"
    "STM32_PROGRAMMER_DEB_URL=${STM32_PROGRAMMER_DEB_URL}"
    "STM32_PROGRAMMER_DEB_SHA256=${STM32_PROGRAMMER_DEB_SHA256}"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/bootstrap_stm32_programmer.sh"
  USES_TERMINAL
  VERBATIM
  COMMENT "Installing the STM32 programming tool for this host")

add_custom_target(setup_stm32_environment)
add_dependencies(setup_stm32_programmer setup_stm32_tools)
add_dependencies(setup_stm32_environment setup_stm32_programmer)

file(GLOB_RECURSE FIRMWARE_INPUTS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/lib/My_Lib/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/lib/My_Lib/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/lib/My_Lib/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/lib/microros/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/lib/microros/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/lib/microros/*.a"
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/boards/stm32H7_v2/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/boards/stm32H7_v2/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/boards/stm32H7_v2/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/boards/stm32H7_v2/*.s"
  "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/mcu_project/boards/stm32H7_v2/STM32CubeIDE/*.ld")

add_custom_command(
  OUTPUT "${FIRMWARE_ELF}" "${FIRMWARE_BIN}" "${FIRMWARE_HEX}"
  COMMAND "${CMAKE_COMMAND}"
    -S "${FIRMWARE_SOURCE_DIR}"
    -B "${FIRMWARE_BUILD_DIR}"
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-toolchain.cmake
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
  COMMAND "${CMAKE_COMMAND}" --build "${FIRMWARE_BUILD_DIR}" --parallel
  DEPENDS
    ${FIRMWARE_INPUTS}
    "${FIRMWARE_SOURCE_DIR}/CMakeLists.txt"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-toolchain.cmake"
  COMMENT "Cross-compiling STM32H743 spinal firmware"
  VERBATIM)

add_custom_target(stm32_firmware ALL
  DEPENDS "${FIRMWARE_ELF}" "${FIRMWARE_BIN}" "${FIRMWARE_HEX}")

set(STM32_FLASH_PORT "/dev/ttyUSB0" CACHE STRING
  "FT232 serial device used by micro-ROS and the STM32 ROM bootloader")
set(STM32_FLASH_BAUD "115200" CACHE STRING
  "UART baud rate used by the STM32 ROM bootloader")
set(STM32_FLASH_BACKEND "auto" CACHE STRING
  "STM32 programmer backend: auto or cubeprogrammer")
set_property(CACHE STM32_FLASH_BACKEND PROPERTY STRINGS
  auto cubeprogrammer)
if(NOT STM32_FLASH_BACKEND MATCHES "^(auto|cubeprogrammer)$")
  message(FATAL_ERROR
    "STM32_FLASH_BACKEND must be auto or cubeprogrammer")
endif()

set(STM32_PROGRAMMER_ARGUMENTS)
if(STM32_PROGRAMMER)
  list(APPEND STM32_PROGRAMMER_ARGUMENTS --programmer "${STM32_PROGRAMMER}")
endif()

# Explicit, non-ALL targets because they reset and rewrite attached hardware.
# SWD is the fully unattended default on the tested STM32H743 Rev V board.
# UART programming and verification work, but ROM v0x92 does not return to the
# application after GO; that path needs a physical power cycle at the end.
add_custom_target(flash_stm32_swd
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/scripts/flash_firmware.py"
    --firmware "${FIRMWARE_BIN}"
    --port "${STM32_FLASH_PORT}"
    --interface swd
    --backend "${STM32_FLASH_BACKEND}"
    ${STM32_PROGRAMMER_ARGUMENTS}
  USES_TERMINAL
  VERBATIM
  COMMENT "Quiescing spinal and flashing firmware through ST-LINK/SWD")
add_dependencies(flash_stm32_swd stm32_firmware)

add_custom_target(flash_stm32_uart
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/../spinal/scripts/flash_firmware.py"
    --firmware "${FIRMWARE_BIN}"
    --port "${STM32_FLASH_PORT}"
    --baud "${STM32_FLASH_BAUD}"
    --interface uart
    --backend "${STM32_FLASH_BACKEND}"
    ${STM32_PROGRAMMER_ARGUMENTS}
  USES_TERMINAL
  VERBATIM
  COMMENT "Entering the STM32 ROM bootloader and flashing through FT232/UART")
add_dependencies(flash_stm32_uart stm32_firmware)

# Keep the short target name as the safe, verified path for this board revision.
add_custom_target(flash_stm32)
add_dependencies(flash_stm32 flash_stm32_swd)

install(FILES
  "${FIRMWARE_ELF}"
  "${FIRMWARE_BIN}"
  "${FIRMWARE_HEX}"
  DESTINATION share/${PROJECT_NAME}/firmware)

ament_package()
