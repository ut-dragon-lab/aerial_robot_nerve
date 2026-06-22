#include "battery_status/battery_status_ros_module.h"

BatteryStatusRosModule* BatteryStatusRosModule::instance_ = nullptr;

void BatteryStatusRosModule::init_hw(ADC_HandleTypeDef* hadc, bool is_adc_measure)
{
  battery_status_.init(hadc, is_adc_measure);
}

void BatteryStatusRosModule::create_entities(rcl_node_t& node)
{
  reserve_entities();
  instance_ = this;

  (void)init_publisher_default(
    node,
    voltage_status_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "battery_voltage_status");

  (void)init_subscription_default(
    node,
    adc_scale_sub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "set_adc_scale",
    &adc_scale_msg_,
    &BatteryStatusRosModule::adcScaleCallbackStatic_,
    ON_NEW_DATA);
}

void BatteryStatusRosModule::update()
{
  if (!battery_status_.update()) return;
  publish();
}

void BatteryStatusRosModule::update(float voltage)
{
  if (!battery_status_.update(voltage)) return;
  publish();
}

void BatteryStatusRosModule::publish()
{
  if (ros_ready_ == nullptr) return;
  if (!ros_ready_->load(std::memory_order_acquire)) return;
  if (!battery_status_.voltagePublishReady(true)) return;

  voltage_status_msg_.data = battery_status_.getVoltage();

  lock_ros_();
  if (ros_ready_->load(std::memory_order_acquire)) {
    (void)rcl_publish(&voltage_status_pub_, &voltage_status_msg_, nullptr);
  }
  unlock_ros_();
}

void BatteryStatusRosModule::adcScaleCallbackStatic_(const void* msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;

  const auto* msg = reinterpret_cast<const std_msgs__msg__Float32*>(msgin);
  instance_->battery_status_.setAdcScale(msg->data);
}
