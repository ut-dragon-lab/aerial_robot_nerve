// -*- mode: c++ -*-
// SPDX-License-Identifier: BSD-3-Clause
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

namespace
{
struct TopicSpec
{
  std::string topic;
  std::string type;
};

std::string strip_slashes(std::string value)
{
  while (!value.empty() && value.front() == '/') value.erase(value.begin());
  while (!value.empty() && value.back() == '/') value.pop_back();
  return value;
}

std::string make_absolute_topic(const std::string &ns, const std::string &topic)
{
  const std::string clean_ns = strip_slashes(ns);
  const std::string clean_topic = strip_slashes(topic);

  if (clean_topic.empty()) return "/";
  if (clean_ns.empty()) return "/" + clean_topic;
  return "/" + clean_ns + "/" + clean_topic;
}

bool parse_spec(const std::string &text, TopicSpec &spec)
{
  const auto sep = text.find(':');
  if (sep == std::string::npos) return false;

  spec.topic = text.substr(0, sep);
  spec.type = text.substr(sep + 1);
  return !strip_slashes(spec.topic).empty() && !spec.type.empty();
}

const std::vector<std::string> kDefaultRootToNamespaceTopics = {
  "imu:spinal_msgs/msg/Imu",
  "flight_config_ack:std_msgs/msg/UInt8",
  "rpy/pid:spinal_msgs/msg/RollPitchYawTerms",
  "rpy/feedback_state:spinal_msgs/msg/RollPitchYawTerm",
  "motor_pwms:spinal_msgs/msg/Pwms",
  "esc_telem:spinal_msgs/msg/ESCTelemetryArray",
  "battery_voltage_status:std_msgs/msg/Float32",
  "gps:spinal_msgs/msg/Gps",
  "encoder_angle:std_msgs/msg/UInt16",
};

const std::vector<std::string> kDefaultNamespaceToRootTopics = {
  "flight_config_cmd:spinal_msgs/msg/FlightConfigCmd",
  "uav_info:spinal_msgs/msg/UavInfo",
  "gimbal_dof:std_msgs/msg/UInt8",
  "four_axes/command:spinal_msgs/msg/FourAxisCommand",
  "rpy/gain:spinal_msgs/msg/RollPitchYawTerms",
  "p_matrix_pseudo_inverse_inertia:spinal_msgs/msg/PMatrixPseudoInverseWithInertia",
  "torque_allocation_matrix_inv:spinal_msgs/msg/TorqueAllocationMatrixInv",
  "desire_coordinate:spinal_msgs/msg/DesireCoord",
  "motor_info:spinal_msgs/msg/PwmInfo",
  "pwm_test:spinal_msgs/msg/PwmTest",
  "gps_config_cmd:std_msgs/msg/UInt8",
  "baro_config_cmd:std_msgs/msg/UInt8",
  "set_adc_scale:std_msgs/msg/Float32",
};
}  // namespace

class SpinalNamespaceBridge : public rclcpp::Node
{
public:
  SpinalNamespaceBridge() : Node("spinal_namespace_bridge")
  {
    robot_namespace_ = declare_parameter<std::string>("robot_namespace", "mini_quadrotor");
    root_to_namespace_topics_ = declare_parameter<std::vector<std::string>>(
        "root_to_namespace_topics", kDefaultRootToNamespaceTopics);
    namespace_to_root_topics_ = declare_parameter<std::vector<std::string>>(
        "namespace_to_root_topics", kDefaultNamespaceToRootTopics);

    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    best_effort_qos.best_effort();

    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    reliable_qos.reliable();

    for (const auto &spec_text : root_to_namespace_topics_)
    {
      add_relay(spec_text, "", robot_namespace_, best_effort_qos, reliable_qos);
    }

    for (const auto &spec_text : namespace_to_root_topics_)
    {
      add_relay(spec_text, robot_namespace_, "", reliable_qos, reliable_qos);
    }
  }

private:
  void add_relay(const std::string &spec_text, const std::string &input_ns, const std::string &output_ns,
                 const rclcpp::QoS &input_qos, const rclcpp::QoS &output_qos)
  {
    TopicSpec spec;
    if (!parse_spec(spec_text, spec))
    {
      RCLCPP_WARN(get_logger(), "Skip invalid relay spec: '%s'", spec_text.c_str());
      return;
    }

    const std::string input_topic = make_absolute_topic(input_ns, spec.topic);
    const std::string output_topic = make_absolute_topic(output_ns, spec.topic);
    if (input_topic == output_topic)
    {
      RCLCPP_INFO(get_logger(), "Skip relay with identical endpoints: %s", input_topic.c_str());
      return;
    }

    auto publisher = create_generic_publisher(output_topic, spec.type, output_qos);
    auto subscription = create_generic_subscription(
        input_topic, spec.type, input_qos,
        [publisher, input_topic, output_topic, logger = get_logger()](
            std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
          try
          {
            publisher->publish(*msg);
          }
          catch (const std::exception &e)
          {
            RCLCPP_WARN(logger, "Failed to relay %s -> %s: %s", input_topic.c_str(), output_topic.c_str(), e.what());
          }
        });

    publishers_.push_back(std::move(publisher));
    subscriptions_.push_back(std::move(subscription));

    RCLCPP_INFO(get_logger(), "Relay %s -> %s [%s]", input_topic.c_str(), output_topic.c_str(), spec.type.c_str());
  }

  std::string robot_namespace_;
  std::vector<std::string> root_to_namespace_topics_;
  std::vector<std::string> namespace_to_root_topics_;
  std::vector<std::shared_ptr<rclcpp::GenericPublisher>> publishers_;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpinalNamespaceBridge>());
  rclcpp::shutdown();
  return 0;
}
