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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(SPINAL_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/mcu_project/lib/My_Lib)

include_directories(
  ${SPINAL_DIRS}
)

add_definitions(-DSIMULATION)

add_library(spinal_math
  ${SPINAL_DIRS}/math/AP_Math.cpp
  ${SPINAL_DIRS}/math/matrix3.cpp
  ${SPINAL_DIRS}/math/location.cpp
  ${SPINAL_DIRS}/math/vector2.cpp
  ${SPINAL_DIRS}/math/vector3.cpp
  ${SPINAL_DIRS}/math/quaternion.cpp
)

set_target_properties(spinal_math PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(spinal_flight_control
  ${SPINAL_DIRS}/flight_control/flight_control.cpp
  ${SPINAL_DIRS}/flight_control/attitude/attitude_control.cpp
  ${SPINAL_DIRS}/flight_control/simulation/flight_control_ros_module.cpp
  ${SPINAL_DIRS}/thruster/simulation/thruster_manager.cpp
  ${SPINAL_DIRS}/thruster/simulation/thruster_ros_module.cpp
)

set_target_properties(spinal_flight_control PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(spinal_flight_control
  PUBLIC
  $<BUILD_INTERFACE:${SPINAL_DIRS}>
  $<INSTALL_INTERFACE:include/My_Lib>
)

target_link_libraries(spinal_flight_control
  PUBLIC
  spinal_math
)

ament_target_dependencies(spinal_flight_control
  PUBLIC
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  spinal_msgs
  std_msgs
  std_srvs
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

add_executable(spinal_namespace_bridge
  src/spinal_namespace_bridge.cpp
)

ament_target_dependencies(spinal_namespace_bridge
  rclcpp
)

ament_export_dependencies(rclcpp rclcpp_lifecycle spinal_msgs sensor_msgs std_msgs std_srvs)

install(
  TARGETS spinal_state_estimation spinal_math spinal_flight_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS spinal_namespace_bridge
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
