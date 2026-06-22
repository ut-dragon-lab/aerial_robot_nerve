#pragma once

#include <std_msgs/msg/float32.h>

#include <ros_utils/ros_module_base.hpp>
#include "battery_status/battery_status.h"

class BatteryStatusRosModule final : public RosModuleBase
{
public:
  BatteryStatusRosModule()
  : RosModuleBase(
      RosModuleEntityCapacity()
        .max_subscriptions(1)
        .max_publishers(1)
        .max_services(0)
        .max_timers(0))
  {}

  void init_hw(ADC_HandleTypeDef* hadc, bool is_adc_measure = true);

  BatteryStatus* getBatteryCore() { return &battery_status_; }

  void create_entities(rcl_node_t& node) override;
  void update() override;
  void update(float voltage);
  void publish() override;

private:
  BatteryStatus battery_status_;

  rcl_subscription_t adc_scale_sub_{};
  rcl_publisher_t voltage_status_pub_{};

  std_msgs__msg__Float32 adc_scale_msg_{};
  std_msgs__msg__Float32 voltage_status_msg_{};

  static BatteryStatusRosModule* instance_;
  static void adcScaleCallbackStatic_(const void* msgin);
};
