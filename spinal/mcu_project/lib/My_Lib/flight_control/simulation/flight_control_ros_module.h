#pragma once

#ifdef SIMULATION

#include <cstddef>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spinal_msgs/msg/desire_coord.hpp>
#include <spinal_msgs/msg/flight_config_cmd.hpp>
#include <spinal_msgs/msg/four_axis_command.hpp>
#include <spinal_msgs/msg/p_matrix_pseudo_inverse_with_inertia.hpp>
#include <spinal_msgs/msg/roll_pitch_yaw_term.hpp>
#include <spinal_msgs/msg/roll_pitch_yaw_terms.hpp>
#include <spinal_msgs/msg/torque_allocation_matrix_inv.hpp>
#include <spinal_msgs/msg/uav_info.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "flight_control/flight_control.h"
#include "state_estimate/state_estimate.h"
#include "thruster/simulation/thruster_manager.h"

class FlightControlRosModule final
{
public:
  FlightControlRosModule() = default;
  ~FlightControlRosModule() = default;

  void init(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
    StateEstimate* estimator,
    ThrusterManager* thruster);

  FlightControl* getFlightControlCore() { return &flight_control_; }

  void activate();
  void deactivate();
  void update() { flight_control_.update(); }
  void publish();

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  FlightControl flight_control_;
  ThrusterManager* thruster_{nullptr};
  bool initialized_{false};

  rclcpp::Subscription<spinal_msgs::msg::FlightConfigCmd>::SharedPtr flight_config_sub_;
  rclcpp::Subscription<spinal_msgs::msg::UavInfo>::SharedPtr uav_info_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gimbal_dof_sub_;
  rclcpp::Subscription<spinal_msgs::msg::FourAxisCommand>::SharedPtr four_axis_cmd_sub_;
  rclcpp::Subscription<spinal_msgs::msg::RollPitchYawTerms>::SharedPtr rpy_gain_sub_;
  rclcpp::Subscription<spinal_msgs::msg::PMatrixPseudoInverseWithInertia>::SharedPtr p_matrix_sub_;
  rclcpp::Subscription<spinal_msgs::msg::TorqueAllocationMatrixInv>::SharedPtr torque_allocation_sub_;
  rclcpp::Subscription<spinal_msgs::msg::DesireCoord>::SharedPtr offset_rot_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sim_voltage_sub_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr config_ack_pub_;
  rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::RollPitchYawTerms>::SharedPtr control_term_pub_;
  rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::RollPitchYawTerm>::SharedPtr control_feedback_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>::SharedPtr gyro_moment_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_control_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr att_control_srv_;

  void configureRosIo_();
  void flightConfigCallback_(const spinal_msgs::msg::FlightConfigCmd::SharedPtr msg);
  void uavInfoCallback_(const spinal_msgs::msg::UavInfo::SharedPtr msg);
  void gimbalDofCallback_(const std_msgs::msg::UInt8::SharedPtr msg);
  void fourAxisCommandCallback_(const spinal_msgs::msg::FourAxisCommand::SharedPtr msg);
  void rpyGainCallback_(const spinal_msgs::msg::RollPitchYawTerms::SharedPtr msg);
  void pMatrixCallback_(const spinal_msgs::msg::PMatrixPseudoInverseWithInertia::SharedPtr msg);
  void torqueAllocationCallback_(const spinal_msgs::msg::TorqueAllocationMatrixInv::SharedPtr msg);
  void offsetRotCallback_(const spinal_msgs::msg::DesireCoord::SharedPtr msg);
  void simVoltageCallback_(const std_msgs::msg::Float32::SharedPtr msg);
  void attitudeControlCallback_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);
};

#endif // SIMULATION
