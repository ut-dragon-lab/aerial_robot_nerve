cmake_minimum_required(VERSION 3.5)
project(spinal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# ADDED
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(spinal_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(SPINAL_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/mcu_project/lib/My_Lib)

include_directories(
  ${SPINAL_DIRS}
)

add_definitions(-DSIMULATION)

add_library(spinal_math
  ${SPINAL_DIRS}/math/AP_Math.cpp
  ${SPINAL_DIRS}/math/matrix3.cpp
  ${SPINAL_DIRS}/math/vector2.cpp
  ${SPINAL_DIRS}/math/vector3.cpp
  ${SPINAL_DIRS}/math/quaternion.cpp
)

add_library(spinal_state_estimation INTERFACE)

target_include_directories(spinal_state_estimation
  INTERFACE
  $<BUILD_INTERFACE:${SPINAL_DIRS}>
  $<INSTALL_INTERFACE:include/My_Lib>
)

target_link_libraries(spinal_state_estimation
  INTERFACE
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

ament_export_dependencies(spinal_msgs std_msgs)

install(
  TARGETS spinal_state_estimation spinal_math
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY ${SPINAL_DIRS}/
  DESTINATION include/My_Lib
  FILES_MATCHING PATTERN "*.h"
)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

install(FILES
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

install(PROGRAMS
  scripts/topic_hz_checker.py
  DESTINATION lib/${PROJECT_NAME}
  )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include include/My_Lib)

ament_package()
