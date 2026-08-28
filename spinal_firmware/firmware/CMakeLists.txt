cmake_minimum_required(VERSION 3.16)
project(spinal_stm32h743 LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

get_filename_component(REPOSITORY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(MCU_PROJECT_DIR "${REPOSITORY_DIR}/spinal/mcu_project")
set(BOARD_DIR "${MCU_PROJECT_DIR}/boards/stm32H7_v2")
set(MY_LIB_DIR "${MCU_PROJECT_DIR}/lib/My_Lib")
set(MICROROS_DIR "${MCU_PROJECT_DIR}/lib/microros")
set(FREERTOS_DIR "${BOARD_DIR}/Middlewares/Third_Party/FreeRTOS")
set(LWIP_DIR "${BOARD_DIR}/Middlewares/Third_Party/LwIP")
set(LINKER_SCRIPT "${BOARD_DIR}/STM32CubeIDE/STM32H743VITX_FLASH.ld")
set(MICROROS_LIBRARY "${MICROROS_DIR}/lib/libmicroros.a")

if(NOT EXISTS "${MICROROS_LIBRARY}")
  message(FATAL_ERROR
    "${MICROROS_LIBRARY} is missing. Run spinal/scripts/make_microros_libraries.py first.")
endif()

file(GLOB_RECURSE MY_LIB_SOURCES CONFIGURE_DEPENDS "${MY_LIB_DIR}/*.cpp")
file(GLOB_RECURSE MICROROS_SOURCES CONFIGURE_DEPENDS "${MICROROS_DIR}/*.cpp")
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS
  "${BOARD_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/*.c")
file(GLOB_RECURSE FREERTOS_SOURCES CONFIGURE_DEPENDS
  "${FREERTOS_DIR}/Source/*.c")
file(GLOB_RECURSE LWIP_SOURCES CONFIGURE_DEPENDS
  "${LWIP_DIR}/src/*.c"
  "${LWIP_DIR}/system/OS/*.c")
file(GLOB BOARD_C_SOURCES CONFIGURE_DEPENDS "${BOARD_DIR}/Src/*.c")

# The Cube project keeps main.cpp as a symlink to main.c so its USER CODE blocks
# can contain C++. Compile main.c once, explicitly as C++.
set(MAIN_SOURCE "${BOARD_DIR}/Src/main.c")
list(REMOVE_ITEM BOARD_C_SOURCES "${MAIN_SOURCE}")
set_source_files_properties("${MAIN_SOURCE}" PROPERTIES LANGUAGE CXX)

set(FIRMWARE_SOURCES
  ${MY_LIB_SOURCES}
  ${MICROROS_SOURCES}
  ${HAL_SOURCES}
  ${FREERTOS_SOURCES}
  ${LWIP_SOURCES}
  ${BOARD_C_SOURCES}
  "${MAIN_SOURCE}"
  "${BOARD_DIR}/Drivers/BSP/Components/lan8742/lan8742.c"
  "${BOARD_DIR}/STM32CubeIDE/Application/User/syscalls.c"
  "${BOARD_DIR}/STM32CubeIDE/Application/User/sysmem.c"
  "${BOARD_DIR}/STM32CubeIDE/Application/Startup/startup_stm32h743vitx.s")

add_executable(spinal ${FIRMWARE_SOURCES})
set_target_properties(spinal PROPERTIES OUTPUT_NAME spinal SUFFIX ".elf")

target_compile_definitions(spinal PRIVATE
  USE_HAL_DRIVER
  STM32H743xx
  STMCUBEMX_NEW_VERSION
  $<$<CONFIG:Debug>:DEBUG>)

target_include_directories(spinal PRIVATE
  "${BOARD_DIR}/Drivers/CMSIS/Include"
  "${BOARD_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
  "${BOARD_DIR}/Inc"
  "${BOARD_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
  "${BOARD_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc"
  "${FREERTOS_DIR}/Source/include"
  "${FREERTOS_DIR}/Source/CMSIS_RTOS"
  "${FREERTOS_DIR}/Source/portable/GCC/ARM_CM4F"
  "${MY_LIB_DIR}/configs/STM32H7_v2"
  "${MY_LIB_DIR}"
  "${MICROROS_DIR}"
  "${MICROROS_DIR}/include"
  "${LWIP_DIR}/src/include"
  "${LWIP_DIR}/system"
  "${LWIP_DIR}/src/include/netif/ppp"
  "${LWIP_DIR}/src/include/lwip"
  "${LWIP_DIR}/src/include/lwip/apps"
  "${LWIP_DIR}/src/include/lwip/priv"
  "${LWIP_DIR}/src/include/lwip/prot"
  "${LWIP_DIR}/src/include/netif"
  "${LWIP_DIR}/src/include/compat/posix"
  "${LWIP_DIR}/src/include/compat/posix/arpa"
  "${LWIP_DIR}/src/include/compat/posix/net"
  "${LWIP_DIR}/src/include/compat/posix/sys"
  "${LWIP_DIR}/src/include/compat/stdc"
  "${LWIP_DIR}/system/arch"
  "${BOARD_DIR}/Drivers/BSP/Components/lan8742")

target_compile_options(spinal PRIVATE
  -mcpu=cortex-m7
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -mthumb
  -ffunction-sections
  -fdata-sections
  -Wall
  -fstack-usage
  --specs=nano.specs
  $<$<CONFIG:Debug>:-O0>
  $<$<CONFIG:Debug>:-g3>
  $<$<NOT:$<CONFIG:Debug>>:-Os>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>)

target_link_options(spinal PRIVATE
  -mcpu=cortex-m7
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -mthumb
  -T${LINKER_SCRIPT}
  --specs=nosys.specs
  --specs=nano.specs
  -static
  -Wl,--gc-sections
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/spinal.map)

target_link_libraries(spinal PRIVATE
  -Wl,--start-group
  "${MICROROS_LIBRARY}"
  c
  m
  stdc++
  supc++
  -Wl,--end-group)

add_custom_command(TARGET spinal POST_BUILD
  COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:spinal>" "${CMAKE_CURRENT_BINARY_DIR}/spinal.bin"
  COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:spinal>" "${CMAKE_CURRENT_BINARY_DIR}/spinal.hex"
  COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:spinal>"
  COMMENT "Generating spinal.bin and spinal.hex")
