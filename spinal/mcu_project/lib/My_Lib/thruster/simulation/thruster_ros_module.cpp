#ifdef SIMULATION

#include "thruster/simulation/thruster_ros_module.h"

#include <algorithm>
#include <functional>

void ThrusterRosModule::init(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
{
  node_ = node;
  if (!initialized_) {
    configureRosIo_();
    initialized_ = true;
  }
}

void ThrusterRosModule::activate()
{
  if (pwms_pub_) pwms_pub_->on_activate();
}

void ThrusterRosModule::deactivate()
{
  if (pwms_pub_) pwms_pub_->on_deactivate();
}

void ThrusterRosModule::publish()
{
  if (!pwms_pub_) return;
  if (!thruster_.motorPwmPublishReady(true)) return;

  spinal_msgs::msg::Pwms msg;
  const size_t n =
    std::min(static_cast<size_t>(thruster_.getMotorNumber()), static_cast<size_t>(MAX_THRUSTER_NUM));
  msg.motor_value.resize(n);
  for (size_t i = 0; i < n; ++i) {
    msg.motor_value[i] = thruster_.getMotorPwmRosValue(static_cast<uint8_t>(i));
  }
  msg.control_mode = thruster_.getControlMode();
  pwms_pub_->publish(msg);
}

void ThrusterRosModule::configureRosIo_()
{
  if (!node_) return;

  pwm_info_sub_ = node_->create_subscription<spinal_msgs::msg::PwmInfo>(
    "motor_info",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ThrusterRosModule::pwmInfoCallback_, this, std::placeholders::_1));

  pwm_test_sub_ = node_->create_subscription<spinal_msgs::msg::PwmTest>(
    "pwm_test",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ThrusterRosModule::pwmTestCallback_, this, std::placeholders::_1));

  pwms_pub_ = node_->create_publisher<spinal_msgs::msg::Pwms>("motor_pwms", rclcpp::QoS(1));
}

void ThrusterRosModule::pwmInfoCallback_(const spinal_msgs::msg::PwmInfo::SharedPtr msg)
{
  if (!msg) return;

  ThrusterPwmInfo info;
  info.min_pwm = msg->min_pwm;
  info.max_pwm = msg->max_pwm;
  info.min_thrust = msg->min_thrust;
  info.force_landing_thrust = msg->force_landing_thrust;
  info.pwm_conversion_mode = msg->pwm_conversion_mode;
  info.motor_info_count =
    std::min(msg->motor_info.size(), static_cast<size_t>(MAX_THRUSTER_MOTOR_INFO_NUM));

  for (size_t i = 0; i < info.motor_info_count; ++i) {
    info.motor_info[i].voltage = msg->motor_info[i].voltage;
    info.motor_info[i].max_thrust = msg->motor_info[i].max_thrust;
    for (size_t j = 0; j < 5; ++j) {
      info.motor_info[i].polynominal[j] = msg->motor_info[i].polynominal[j];
    }
  }

  (void)thruster_.applyPwmInfo(info);
}

void ThrusterRosModule::pwmTestCallback_(const spinal_msgs::msg::PwmTest::SharedPtr msg)
{
  if (!msg) return;

  ThrusterPwmTestCommand cmd;
  cmd.motor_index_count = std::min(msg->motor_index.size(), static_cast<size_t>(MAX_THRUSTER_NUM));
  cmd.pwms_count = std::min(msg->pwms.size(), static_cast<size_t>(MAX_THRUSTER_NUM));
  for (size_t i = 0; i < cmd.motor_index_count; ++i) {
    cmd.motor_index[i] = msg->motor_index[i];
  }
  for (size_t i = 0; i < cmd.pwms_count; ++i) {
    cmd.pwms[i] = msg->pwms[i];
  }
  thruster_.applyPwmTest(cmd);
}

#endif // SIMULATION
