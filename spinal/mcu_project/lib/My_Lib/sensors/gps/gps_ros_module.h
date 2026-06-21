#pragma once

#include "ros_utils/ros_module_base.hpp"
#include <rmw_microros/rmw_microros.h>
#include "sensors/gps/gps_ublox.h"

#include <spinal_msgs/msg/gps.h>
#include <spinal_msgs/msg/gps_full.h>
#include <std_msgs/msg/u_int8.h>


class GpsRosModule final : public RosModuleBase
{
public:
  GpsRosModule(): RosModuleBase()
  {}

  void init_hw(UART_HandleTypeDef* huart, GPIO_TypeDef* led_port, uint16_t led_pin);

  void create_entities(rcl_node_t& node) override;

  void update() override;

  void gpsConfigCallback(const std_msgs__msg__UInt8& config_msg){}

  GPS* getGpsHw() {return &gps_;}

  void publish() override;

private:

  GPS gps_;

  rcl_subscription_t gps_config_sub_ = rcl_get_zero_initialized_subscription();
  rcl_publisher_t gps_pub_ = rcl_get_zero_initialized_publisher();
  spinal_msgs__msg__Gps gps_msg_;
  std_msgs__msg__UInt8 gps_config_msg_;
};
