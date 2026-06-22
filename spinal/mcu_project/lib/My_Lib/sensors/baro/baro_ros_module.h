#pragma once

#include <std_msgs/msg/u_int8.h>

#include <ros_utils/ros_module_base.hpp>
#include "sensors/baro/baro_ms5611.h"

class BaroRosModule : public RosModuleBase
{
public:
  BaroRosModule()
  : RosModuleBase(
      RosModuleEntityCapacity()
        .max_subscriptions(1)
        .max_publishers(0)
        .max_services(0)
        .max_timers(0))
  {}

  void init_hw(I2C_HandleTypeDef* hi2c,
               GPIO_TypeDef* baro_ctrl_port, uint16_t baro_ctrl_pin)
  {
    baro_.init_hw(hi2c, baro_ctrl_port, baro_ctrl_pin);
  }

  void create_entities(rcl_node_t& node) override;
  void update() override { baro_.update(); }
  Baro* getBaroHw() {return &baro_;}

private:
  Baro baro_;

  rcl_subscription_t baro_config_sub_{};
  std_msgs__msg__UInt8 baro_config_msg_{};

  static void baroConfigCallbackStatic(const void* msgin);
  static BaroRosModule* self_;
};
