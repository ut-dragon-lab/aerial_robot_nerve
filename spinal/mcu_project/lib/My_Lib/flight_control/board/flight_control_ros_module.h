#pragma once

#ifndef SIMULATION

#include <cstddef>
#include <cstdint>

#include <std_msgs/msg/u_int8.h>
#include <std_srvs/srv/set_bool.h>

#include <spinal_msgs/msg/desire_coord.h>
#include <spinal_msgs/msg/flight_config_cmd.h>
#include <spinal_msgs/msg/four_axis_command.h>
#include <spinal_msgs/msg/p_matrix_pseudo_inverse_with_inertia.h>
#include <spinal_msgs/msg/p_matrix_pseudo_inverse_unit.h>
#include <spinal_msgs/msg/roll_pitch_yaw_term.h>
#include <spinal_msgs/msg/roll_pitch_yaw_terms.h>
#include <spinal_msgs/msg/torque_allocation_matrix_inv.h>
#include <spinal_msgs/msg/uav_info.h>
#include <spinal_msgs/msg/vector3_int16.h>

#include <ros_utils/ros_module_base.hpp>

#include "flight_control/flight_control.h"
#include "servo/servo.h"
#include "state_estimate/state_estimate.h"
#include "thruster/board/thruster_manager.h"

class FlightControlRosModule final : public RosModuleBase
{
public:
  FlightControlRosModule()
  : RosModuleBase(
      RosModuleEntityCapacity()
        .max_subscriptions(8)
        .max_publishers(3)
        .max_services(1)
        .max_timers(0))
  {}

  void init_hw(
    StateEstimate* estimator,
    ThrusterManager* thruster,
    DirectServo* servo = nullptr,
    osMutexId* control_mutex = nullptr);

  FlightControl* getFlightControlCore() { return &flight_control_; }

  void create_entities(rcl_node_t& node) override;
  void update() override;
  void publish() override;

private:
  static constexpr size_t MAX_FOUR_AXIS_BASE_THRUST_SIZE = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  static constexpr size_t MAX_RPY_TERMS_SIZE = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  static constexpr size_t MAX_P_MATRIX_SIZE = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  static constexpr size_t MAX_TORQUE_ALLOC_SIZE = MAX_FLIGHT_CONTROL_MOTOR_NUM;

  FlightControl flight_control_;
  osMutexId* control_mutex_{nullptr};

  rcl_subscription_t flight_config_sub_{};
  rcl_subscription_t uav_info_sub_{};
  rcl_subscription_t gimbal_dof_sub_{};
  rcl_subscription_t four_axis_cmd_sub_{};
  rcl_subscription_t rpy_gain_sub_{};
  rcl_subscription_t p_matrix_sub_{};
  rcl_subscription_t torque_allocation_sub_{};
  rcl_subscription_t offset_rot_sub_{};

  rcl_publisher_t config_ack_pub_{};
  rcl_publisher_t control_term_pub_{};
  rcl_publisher_t control_feedback_state_pub_{};

  rcl_service_t att_control_srv_{};

  spinal_msgs__msg__FlightConfigCmd flight_config_msg_{};
  spinal_msgs__msg__UavInfo uav_info_msg_{};
  std_msgs__msg__UInt8 gimbal_dof_msg_{};

  spinal_msgs__msg__FourAxisCommand four_axis_cmd_msg_{};
  float four_axis_base_thrust_buf_[MAX_FOUR_AXIS_BASE_THRUST_SIZE]{};

  spinal_msgs__msg__RollPitchYawTerms rpy_gain_msg_{};
  spinal_msgs__msg__RollPitchYawTerm rpy_gain_buf_[MAX_RPY_TERMS_SIZE]{};

  spinal_msgs__msg__PMatrixPseudoInverseWithInertia p_matrix_msg_{};
  spinal_msgs__msg__PMatrixPseudoInverseUnit p_matrix_buf_[MAX_P_MATRIX_SIZE]{};

  spinal_msgs__msg__TorqueAllocationMatrixInv torque_allocation_msg_{};
  spinal_msgs__msg__Vector3Int16 torque_allocation_buf_[MAX_TORQUE_ALLOC_SIZE]{};

  spinal_msgs__msg__DesireCoord offset_rot_msg_{};

  std_msgs__msg__UInt8 config_ack_msg_{};
  spinal_msgs__msg__RollPitchYawTerms control_term_msg_{};
  spinal_msgs__msg__RollPitchYawTerm control_term_buf_[MAX_RPY_TERMS_SIZE]{};
  spinal_msgs__msg__RollPitchYawTerm control_feedback_state_msg_{};

  std_srvs__srv__SetBool_Request att_control_req_{};
  std_srvs__srv__SetBool_Response att_control_res_{};

  void configure_message_storage_();
  void fillControlTerms_(const FlightControlRpyTerms& src);
  void fillControlFeedback_(const FlightControlRpyTerm& src);

  void lock_control_();
  void unlock_control_();

  static FlightControlRosModule* instance_;
  static void flightConfigCallbackStatic_(const void* msgin);
  static void uavInfoCallbackStatic_(const void* msgin);
  static void gimbalDofCallbackStatic_(const void* msgin);
  static void fourAxisCommandCallbackStatic_(const void* msgin);
  static void rpyGainCallbackStatic_(const void* msgin);
  static void pMatrixCallbackStatic_(const void* msgin);
  static void torqueAllocationCallbackStatic_(const void* msgin);
  static void offsetRotCallbackStatic_(const void* msgin);
  static void attitudeControlCallbackStatic_(const void* req_msg, void* res_msg);
};

#endif // !SIMULATION
