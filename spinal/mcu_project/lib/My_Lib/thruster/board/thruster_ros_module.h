#pragma once

#include <cstddef>
#include <cstdint>

#include <rmw_microros/rmw_microros.h>

#include <spinal_msgs/msg/esc_telemetry_array.h>
#include <spinal_msgs/msg/motor_info.h>
#include <spinal_msgs/msg/pwm_info.h>
#include <spinal_msgs/msg/pwm_test.h>
#include <spinal_msgs/msg/pwms.h>

#include <ros_utils/ros_module_base.hpp>
#include "thruster/board/thruster_manager.h"

class ThrusterRosModule final : public RosModuleBase
{
public:
  ThrusterRosModule()
  : RosModuleBase(
      RosModuleEntityCapacity()
        .max_subscriptions(2)
        .max_publishers(2)
        .max_services(0)
        .max_timers(0))
  {}

  void init_hw(TIM_HandleTypeDef* htim_primary, TIM_HandleTypeDef* htim_secondary)
  {
    thruster_.init(htim_primary, htim_secondary);
  }

#if DSHOT
  void init_dshot_telemetry(UART_HandleTypeDef* huart, int num_motor_mag_pole = 14)
  {
    thruster_.initDShotTelemetry(huart, num_motor_mag_pole);
  }
#endif

  void setBatteryStatus(BatteryStatus* battery) { thruster_.setBatteryStatus(battery); }

  ThrusterManager* getThrusterManager() { return &thruster_; }
  void sendCommand() { thruster_.sendCommand(); }
  bool updateTelemetry() { return thruster_.updateTelemetry(); }

  void create_entities(rcl_node_t& node) override;
  void publish() override;

private:
  static constexpr size_t MAX_PWM_INFO_MOTOR_INFO_SIZE = MAX_THRUSTER_MOTOR_INFO_NUM;
  static constexpr size_t MAX_PWM_TEST_INDEX_SIZE = MAX_THRUSTER_NUM;
  static constexpr size_t MAX_PWM_TEST_PWM_SIZE = MAX_THRUSTER_NUM;

  ThrusterManager thruster_;

  rcl_subscription_t pwm_info_sub_{};
  rcl_subscription_t pwm_test_sub_{};
  rcl_publisher_t pwms_pub_{};
  rcl_publisher_t esc_telem_pub_{};

  spinal_msgs__msg__PwmInfo pwm_info_msg_{};
  spinal_msgs__msg__MotorInfo pwm_info_motor_info_buf_[MAX_PWM_INFO_MOTOR_INFO_SIZE]{};

  spinal_msgs__msg__PwmTest pwm_test_msg_{};
  uint8_t pwm_test_motor_index_buf_[MAX_PWM_TEST_INDEX_SIZE]{};
  float pwm_test_pwms_buf_[MAX_PWM_TEST_PWM_SIZE]{};

  spinal_msgs__msg__Pwms pwms_msg_{};
  uint16_t pwms_motor_value_buf_[MAX_THRUSTER_NUM]{};

  spinal_msgs__msg__ESCTelemetryArray esc_telem_msg_{};

  void configure_message_storage_();
  void fillPwms_();
  void fillEscTelemetry_(const EscTelemetrySnapshot& snapshot);

  static ThrusterRosModule* instance_;
  static void pwmInfoCallbackStatic_(const void* msgin);
  static void pwmTestCallbackStatic_(const void* msgin);
};
