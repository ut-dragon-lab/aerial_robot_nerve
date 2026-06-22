#pragma once

#include <cstddef>
#include <cstdint>

#include <rmw_microros/rmw_microros.h>

#include <spinal_msgs/msg/board_info.h>
#include <spinal_msgs/msg/joint_profile.h>
#include <spinal_msgs/msg/joint_profiles.h>
#include <spinal_msgs/msg/servo_control_cmd.h>
#include <spinal_msgs/msg/servo_info.h>
#include <spinal_msgs/msg/servo_state.h>
#include <spinal_msgs/msg/servo_states.h>
#include <spinal_msgs/msg/servo_torque_cmd.h>
#include <spinal_msgs/msg/servo_torque_states.h>
#include <spinal_msgs/srv/get_board_info.h>
#include <spinal_msgs/srv/set_direct_servo_config.h>

#include <ros_utils/ros_module_base.hpp>
#include "servo/servo.h"

class DirectServoRosModule final : public RosModuleBase
{
public:
  DirectServoRosModule()
  : RosModuleBase(
      RosModuleEntityCapacity()
        .max_subscriptions(3)
        .max_publishers(2)
        .max_services(2)
        .max_timers(0))
  {}

  bool init_hw(UART_HandleTypeDef* huart, osMutexId* mutex = nullptr);

  DirectServo* getServoCore() { return &servo_; }
  bool connected() const { return servo_.connected(); }

  void create_entities(rcl_node_t& node) override;
  void update() override;
  void publish() override;

private:
  static constexpr size_t MAX_SERVO_CONFIG_DATA_SIZE = 4;
  static constexpr size_t MAX_SERVO_CONFIG_RES_DATA_SIZE = 1;

  DirectServo servo_;

  rcl_subscription_t servo_ctrl_sub_{};
  rcl_subscription_t servo_torque_ctrl_sub_{};
  rcl_subscription_t joint_profiles_sub_{};
  rcl_publisher_t servo_state_pub_{};
  rcl_publisher_t servo_torque_state_pub_{};
  rcl_service_t servo_config_srv_{};
  rcl_service_t board_info_srv_{};

  spinal_msgs__msg__ServoControlCmd servo_ctrl_msg_{};
  uint8_t servo_ctrl_index_buf_[MAX_SERVO_NUM]{};
  int16_t servo_ctrl_angles_buf_[MAX_SERVO_NUM]{};

  spinal_msgs__msg__ServoTorqueCmd servo_torque_ctrl_msg_{};
  uint8_t servo_torque_ctrl_index_buf_[MAX_SERVO_NUM]{};
  uint8_t servo_torque_ctrl_enable_buf_[MAX_SERVO_NUM]{};

  spinal_msgs__msg__JointProfiles joint_profiles_msg_{};
  spinal_msgs__msg__JointProfile joint_profiles_buf_[MAX_SERVO_NUM]{};

  spinal_msgs__msg__ServoStates servo_state_msg_{};
  spinal_msgs__msg__ServoState servo_state_buf_[MAX_SERVO_NUM]{};

  spinal_msgs__msg__ServoTorqueStates servo_torque_state_msg_{};
  uint8_t servo_torque_state_buf_[MAX_SERVO_NUM]{};

  spinal_msgs__srv__SetDirectServoConfig_Request servo_config_req_{};
  spinal_msgs__srv__SetDirectServoConfig_Response servo_config_res_{};
  int32_t servo_config_req_data_buf_[MAX_SERVO_CONFIG_DATA_SIZE]{};
  float servo_config_res_data_buf_[MAX_SERVO_CONFIG_RES_DATA_SIZE]{};

  spinal_msgs__srv__GetBoardInfo_Request board_info_req_{};
  spinal_msgs__srv__GetBoardInfo_Response board_info_res_{};
  spinal_msgs__msg__BoardInfo board_info_buf_[1]{};
  spinal_msgs__msg__ServoInfo board_servo_info_buf_[MAX_SERVO_NUM]{};

  void configure_message_storage_();
  void fillServoStates_();
  void fillServoTorqueStates_();
  void fillBoardInfo_();

  static DirectServoRosModule* instance_;
  static void servoControlCallbackStatic_(const void* msgin);
  static void servoTorqueControlCallbackStatic_(const void* msgin);
  static void jointProfilesCallbackStatic_(const void* msgin);
  static void servoConfigCallbackStatic_(const void* req_msg, void* res_msg);
  static void boardInfoCallbackStatic_(const void* req_msg, void* res_msg);
};
