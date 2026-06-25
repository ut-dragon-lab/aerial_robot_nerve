#pragma once

#ifdef SIMULATION

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <spinal_msgs/msg/pwm_info.hpp>
#include <spinal_msgs/msg/pwm_test.hpp>
#include <spinal_msgs/msg/pwms.hpp>

#include "thruster/simulation/thruster_manager.h"

class ThrusterRosModule
{
public:
  ThrusterRosModule() = default;
  ~ThrusterRosModule() = default;

  void init(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node);

  ThrusterManager* getThrusterManager() { return &thruster_; }
  void sendCommand() { thruster_.sendCommand(); }
  bool updateTelemetry() { return thruster_.updateTelemetry(); }
  void activate();
  void deactivate();
  void publish();

private:
  ThrusterManager thruster_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  bool initialized_{false};

  rclcpp::Subscription<spinal_msgs::msg::PwmInfo>::SharedPtr pwm_info_sub_;
  rclcpp::Subscription<spinal_msgs::msg::PwmTest>::SharedPtr pwm_test_sub_;
  rclcpp_lifecycle::LifecyclePublisher<spinal_msgs::msg::Pwms>::SharedPtr pwms_pub_;

  void configureRosIo_();
  void pwmInfoCallback_(const spinal_msgs::msg::PwmInfo::SharedPtr msg);
  void pwmTestCallback_(const spinal_msgs::msg::PwmTest::SharedPtr msg);
};

#endif // SIMULATION
