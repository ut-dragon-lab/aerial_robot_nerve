#include "thruster/board/thruster_ros_module.h"

ThrusterRosModule* ThrusterRosModule::instance_ = nullptr;

void ThrusterRosModule::create_entities(rcl_node_t& node)
{
  reserve_entities();
  instance_ = this;

  spinal_msgs__msg__PwmInfo__init(&pwm_info_msg_);
  spinal_msgs__msg__PwmTest__init(&pwm_test_msg_);
  spinal_msgs__msg__Pwms__init(&pwms_msg_);
  spinal_msgs__msg__ESCTelemetryArray__init(&esc_telem_msg_);

  configure_message_storage_();

  (void)init_subscription_default(
    node,
    pwm_info_sub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, PwmInfo),
    "motor_info",
    &pwm_info_msg_,
    &ThrusterRosModule::pwmInfoCallbackStatic_,
    ON_NEW_DATA);

  (void)init_subscription_default(
    node,
    pwm_test_sub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, PwmTest),
    "pwm_test",
    &pwm_test_msg_,
    &ThrusterRosModule::pwmTestCallbackStatic_,
    ON_NEW_DATA);

  (void)init_publisher_default(
    node,
    pwms_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, Pwms),
    "motor_pwms");

  (void)init_publisher_default(
    node,
    esc_telem_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, ESCTelemetryArray),
    "esc_telem");
}

void ThrusterRosModule::publish()
{
  if (ros_ready_ == nullptr) return;
  if (!ros_ready_->load(std::memory_order_acquire)) return;

  if (thruster_.motorPwmPublishReady(true)) {
    fillPwms_();
    (void)rcl_publish(&pwms_pub_, &pwms_msg_, nullptr);
  }

  EscTelemetrySnapshot snapshot;
  if (thruster_.consumeEscTelemetrySnapshot(snapshot)) {
    fillEscTelemetry_(snapshot);
    (void)rcl_publish(&esc_telem_pub_, &esc_telem_msg_, nullptr);
  }
}

void ThrusterRosModule::configure_message_storage_()
{
  pwm_info_msg_.motor_info.data = pwm_info_motor_info_buf_;
  pwm_info_msg_.motor_info.size = 0;
  pwm_info_msg_.motor_info.capacity = MAX_PWM_INFO_MOTOR_INFO_SIZE;

  pwm_test_msg_.motor_index.data = pwm_test_motor_index_buf_;
  pwm_test_msg_.motor_index.size = 0;
  pwm_test_msg_.motor_index.capacity = MAX_PWM_TEST_INDEX_SIZE;
  pwm_test_msg_.pwms.data = pwm_test_pwms_buf_;
  pwm_test_msg_.pwms.size = 0;
  pwm_test_msg_.pwms.capacity = MAX_PWM_TEST_PWM_SIZE;

  pwms_msg_.motor_value.data = pwms_motor_value_buf_;
  pwms_msg_.motor_value.size = 0;
  pwms_msg_.motor_value.capacity = MAX_THRUSTER_NUM;
  pwms_msg_.control_mode = ThrusterControlMode::CONTROL_MODE_NONE;
}

void ThrusterRosModule::fillPwms_()
{
  const size_t motor_num = thruster_.getMotorNumber();
  pwms_msg_.motor_value.size = motor_num > MAX_THRUSTER_NUM ? MAX_THRUSTER_NUM : motor_num;
  pwms_msg_.control_mode = thruster_.getControlMode();

  for (size_t i = 0; i < pwms_msg_.motor_value.size; ++i) {
    pwms_motor_value_buf_[i] = thruster_.getMotorPwmRosValue(static_cast<uint8_t>(i));
  }
}

void ThrusterRosModule::fillEscTelemetry_(const EscTelemetrySnapshot& snapshot)
{
  const uint64_t t_ms = rmw_uros_epoch_millis();
  esc_telem_msg_.stamp.sec = static_cast<int32_t>(t_ms / 1000ULL);
  esc_telem_msg_.stamp.nanosec = static_cast<uint32_t>((t_ms % 1000ULL) * 1000000ULL);

  auto fill_one = [](spinal_msgs__msg__ESCTelemetry& dst, const EscTelemetryData& src) {
    dst.temperature = src.temperature;
    dst.voltage = src.voltage;
    dst.current = src.current;
    dst.consumption = src.consumption;
    dst.rpm = src.rpm;
    dst.crc_error = src.crc_error;
  };

  fill_one(esc_telem_msg_.esc_telemetry_1, snapshot.esc[0]);
  fill_one(esc_telem_msg_.esc_telemetry_2, snapshot.esc[1]);
  fill_one(esc_telem_msg_.esc_telemetry_3, snapshot.esc[2]);
  fill_one(esc_telem_msg_.esc_telemetry_4, snapshot.esc[3]);
}

void ThrusterRosModule::pwmInfoCallbackStatic_(const void* msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;

  const auto* msg = reinterpret_cast<const spinal_msgs__msg__PwmInfo*>(msgin);

  ThrusterPwmInfo info;
  info.min_pwm = msg->min_pwm;
  info.max_pwm = msg->max_pwm;
  info.min_thrust = msg->min_thrust;
  info.force_landing_thrust = msg->force_landing_thrust;
  info.pwm_conversion_mode = msg->pwm_conversion_mode;
  info.motor_info_count = msg->motor_info.size;
  if (info.motor_info_count > MAX_THRUSTER_MOTOR_INFO_NUM) {
    info.motor_info_count = MAX_THRUSTER_MOTOR_INFO_NUM;
  }

  for (size_t i = 0; i < info.motor_info_count; ++i) {
    const spinal_msgs__msg__MotorInfo& src = msg->motor_info.data[i];
    ThrusterMotorInfo& dst = info.motor_info[i];
    dst.voltage = src.voltage;
    dst.max_thrust = src.max_thrust;
    for (size_t j = 0; j < 5; ++j) {
      dst.polynominal[j] = src.polynominal[j];
    }
  }

  (void)instance_->thruster_.applyPwmInfo(info);
}

void ThrusterRosModule::pwmTestCallbackStatic_(const void* msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;

  const auto* msg = reinterpret_cast<const spinal_msgs__msg__PwmTest*>(msgin);

  ThrusterPwmTestCommand cmd;
  cmd.motor_index_count = msg->motor_index.size;
  if (cmd.motor_index_count > MAX_THRUSTER_NUM) {
    cmd.motor_index_count = MAX_THRUSTER_NUM;
  }
  cmd.pwms_count = msg->pwms.size;
  if (cmd.pwms_count > MAX_THRUSTER_NUM) {
    cmd.pwms_count = MAX_THRUSTER_NUM;
  }

  for (size_t i = 0; i < cmd.motor_index_count; ++i) {
    cmd.motor_index[i] = msg->motor_index.data[i];
  }
  for (size_t i = 0; i < cmd.pwms_count; ++i) {
    cmd.pwms[i] = msg->pwms.data[i];
  }

  instance_->thruster_.applyPwmTest(cmd);
}
