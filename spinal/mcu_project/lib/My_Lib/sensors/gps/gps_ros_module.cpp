#pragma once

#include "sensors/gps/gps_ros_module.h"

GpsRosModule* gps_ros_instance_ = nullptr;

void gpsConfigCallbackStatic(const void * msgin)
{
  if (gps_ros_instance_ == nullptr) return;
  if (msgin == nullptr) return;
  gps_ros_instance_->gpsConfigCallback(*reinterpret_cast<const std_msgs__msg__UInt8*>(msgin));
}

void GpsRosModule::init_hw(UART_HandleTypeDef* huart,
                           GPIO_TypeDef* led_port, uint16_t led_pin)
{
  gps_.init(huart, led_port, led_pin);
}

void GpsRosModule::create_entities(rcl_node_t& node)
{
  reserve_entities();
  (void)init_publisher_default(
    node,
    gps_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, Gps),
    "gps");

  (void)init_subscription_default(
    node,
    gps_config_sub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "gps_config_cmd",
    &gps_config_msg_,
    &gpsConfigCallbackStatic,
    ON_NEW_DATA);
}

void GpsRosModule::update()
{
  gps_.update();
}

void GpsRosModule::publish()
{
  if (ros_ready_ == nullptr) return;
  if (!ros_ready_->load(std::memory_order_acquire)) return;

  GPS_State st;

  // lock_ros_();
  if (!ros_ready_->load(std::memory_order_acquire)) { unlock_ros_(); return; }

  while (gps_.pop_state_snapshot(st))
  {
    const uint64_t t_ms = rmw_uros_epoch_millis();

    gps_msg_.stamp.sec     = (int32_t)(t_ms / 1000ULL);
    gps_msg_.stamp.nanosec = (uint32_t)((t_ms % 1000ULL) * 1000000ULL);

    gps_msg_.location[0] = st.location.lat / 1e7L;
    gps_msg_.location[1] = st.location.lng / 1e7L;
    gps_msg_.velocity[0] = st.velocity.x;
    gps_msg_.velocity[1] = st.velocity.y;
    gps_msg_.sat_num      = st.num_sats;

    (void)rcl_publish(&gps_pub_, &gps_msg_, nullptr);
  }

  // unlock_ros_();
}
