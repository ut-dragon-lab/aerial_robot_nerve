#include "servo/servo_ros_module.h"

DirectServoRosModule* DirectServoRosModule::instance_ = nullptr;

bool DirectServoRosModule::init_hw(UART_HandleTypeDef* huart, osMutexId* mutex)
{
  return servo_.init(huart, mutex);
}

void DirectServoRosModule::create_entities(rcl_node_t& node)
{
  reserve_entities();
  instance_ = this;

  spinal_msgs__msg__ServoControlCmd__init(&servo_ctrl_msg_);
  spinal_msgs__msg__ServoTorqueCmd__init(&servo_torque_ctrl_msg_);
  spinal_msgs__msg__JointProfiles__init(&joint_profiles_msg_);
  spinal_msgs__msg__ServoStates__init(&servo_state_msg_);
  spinal_msgs__msg__ServoTorqueStates__init(&servo_torque_state_msg_);
  spinal_msgs__srv__SetDirectServoConfig_Request__init(&servo_config_req_);
  spinal_msgs__srv__SetDirectServoConfig_Response__init(&servo_config_res_);
  spinal_msgs__srv__GetBoardInfo_Request__init(&board_info_req_);
  spinal_msgs__srv__GetBoardInfo_Response__init(&board_info_res_);

  configure_message_storage_();

  (void)init_subscription_default(
    node,
    servo_ctrl_sub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, ServoControlCmd),
    "servo/target_states",
    &servo_ctrl_msg_,
    &DirectServoRosModule::servoControlCallbackStatic_,
    ON_NEW_DATA);

  (void)init_subscription_default(
    node,
    servo_torque_ctrl_sub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, ServoTorqueCmd),
    "servo/torque_enable",
    &servo_torque_ctrl_msg_,
    &DirectServoRosModule::servoTorqueControlCallbackStatic_,
    ON_NEW_DATA);

  (void)init_subscription_default(
    node,
    joint_profiles_sub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, JointProfiles),
    "joint_profiles",
    &joint_profiles_msg_,
    &DirectServoRosModule::jointProfilesCallbackStatic_,
    ON_NEW_DATA);

  (void)init_publisher_default(
    node,
    servo_state_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, ServoStates),
    "servo/states");

  (void)init_publisher_default(
    node,
    servo_torque_state_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, ServoTorqueStates),
    "servo/torque_states");

  (void)init_service_default(
    node,
    servo_config_srv_,
    ROSIDL_GET_SRV_TYPE_SUPPORT(spinal_msgs, srv, SetDirectServoConfig),
    "direct_servo_config",
    &servo_config_req_,
    &servo_config_res_,
    &DirectServoRosModule::servoConfigCallbackStatic_);

  (void)init_service_default(
    node,
    board_info_srv_,
    ROSIDL_GET_SRV_TYPE_SUPPORT(spinal_msgs, srv, GetBoardInfo),
    "get_board_info",
    &board_info_req_,
    &board_info_res_,
    &DirectServoRosModule::boardInfoCallbackStatic_);
}

void DirectServoRosModule::update()
{
  servo_.update();
}

void DirectServoRosModule::publish()
{
  if (ros_ready_ == nullptr) return;
  if (!ros_ready_->load(std::memory_order_acquire)) return;
  if (!servo_.connected()) return;

  if (servo_.statePublishReady(true)) {
    fillServoStates_();
    (void)rcl_publish(&servo_state_pub_, &servo_state_msg_, nullptr);
    servo_.markStatePublished();
  }

  if (servo_.torqueStatePublishReady()) {
    fillServoTorqueStates_();
    (void)rcl_publish(&servo_torque_state_pub_, &servo_torque_state_msg_, nullptr);
    servo_.markTorqueStatePublished();
  }
}

void DirectServoRosModule::configure_message_storage_()
{
  servo_ctrl_msg_.index.data = servo_ctrl_index_buf_;
  servo_ctrl_msg_.index.size = 0;
  servo_ctrl_msg_.index.capacity = MAX_SERVO_NUM;
  servo_ctrl_msg_.angles.data = servo_ctrl_angles_buf_;
  servo_ctrl_msg_.angles.size = 0;
  servo_ctrl_msg_.angles.capacity = MAX_SERVO_NUM;

  servo_torque_ctrl_msg_.index.data = servo_torque_ctrl_index_buf_;
  servo_torque_ctrl_msg_.index.size = 0;
  servo_torque_ctrl_msg_.index.capacity = MAX_SERVO_NUM;
  servo_torque_ctrl_msg_.torque_enable.data = servo_torque_ctrl_enable_buf_;
  servo_torque_ctrl_msg_.torque_enable.size = 0;
  servo_torque_ctrl_msg_.torque_enable.capacity = MAX_SERVO_NUM;

  joint_profiles_msg_.joints.data = joint_profiles_buf_;
  joint_profiles_msg_.joints.size = 0;
  joint_profiles_msg_.joints.capacity = MAX_SERVO_NUM;

  servo_state_msg_.servos.data = servo_state_buf_;
  servo_state_msg_.servos.size = 0;
  servo_state_msg_.servos.capacity = MAX_SERVO_NUM;

  servo_torque_state_msg_.torque_enable.data = servo_torque_state_buf_;
  servo_torque_state_msg_.torque_enable.size = 0;
  servo_torque_state_msg_.torque_enable.capacity = MAX_SERVO_NUM;

  servo_config_req_.data.data = servo_config_req_data_buf_;
  servo_config_req_.data.size = 0;
  servo_config_req_.data.capacity = MAX_SERVO_CONFIG_DATA_SIZE;
  servo_config_res_.data.data = servo_config_res_data_buf_;
  servo_config_res_.data.size = 0;
  servo_config_res_.data.capacity = MAX_SERVO_CONFIG_RES_DATA_SIZE;

  board_info_res_.boards.data = board_info_buf_;
  board_info_res_.boards.size = 1;
  board_info_res_.boards.capacity = 1;
  board_info_buf_[0].servos.data = board_servo_info_buf_;
  board_info_buf_[0].servos.size = 0;
  board_info_buf_[0].servos.capacity = MAX_SERVO_NUM;
}

void DirectServoRosModule::fillServoStates_()
{
  const uint64_t t_ms = rmw_uros_epoch_millis();
  servo_state_msg_.stamp.sec = static_cast<int32_t>(t_ms / 1000ULL);
  servo_state_msg_.stamp.nanosec = static_cast<uint32_t>((t_ms % 1000ULL) * 1000000ULL);

  const size_t servo_num = servo_.getServoNum();
  servo_state_msg_.servos.size = servo_num;

  for (size_t i = 0; i < servo_num; ++i) {
    const ServoData& s = servo_.getServoData(i);
    spinal_msgs__msg__ServoState& servo_state = servo_state_buf_[i];
    servo_state.index = static_cast<uint8_t>(i);
    servo_state.angle = static_cast<int16_t>(s.present_position_);
    servo_state.temp = s.present_temp_;
    servo_state.load = s.present_current_;
    servo_state.error = s.hardware_error_status_;
  }
}

void DirectServoRosModule::fillServoTorqueStates_()
{
  const size_t servo_num = servo_.getServoNum();
  servo_torque_state_msg_.torque_enable.size = servo_num;

  for (size_t i = 0; i < servo_num; ++i) {
    const ServoData& s = servo_.getServoData(i);
    servo_torque_state_buf_[i] = s.torque_enable_ ? 1 : 0;
  }
}

void DirectServoRosModule::fillBoardInfo_()
{
  board_info_res_.boards.size = 1;
  spinal_msgs__msg__BoardInfo& board = board_info_buf_[0];
  board.slave_id = servo_.getBoardId();
  board.imu_send_data_flag = servo_.getImuSendDataFlag();
  board.dynamixel_ttl_rs485_mixed = servo_.getDynamixelTtlRs485Mixed();
  board.servo_pulley_skip_thresh = 0;

  const size_t servo_num = servo_.getServoNum();
  board.servos.size = servo_num;
  for (size_t i = 0; i < servo_num; ++i) {
    const ServoData& s = servo_.getServoData(i);
    spinal_msgs__msg__ServoInfo& info = board_servo_info_buf_[i];
    info.id = s.id_;
    info.p_gain = s.p_gain_;
    info.i_gain = s.i_gain_;
    info.d_gain = s.d_gain_;
    info.profile_velocity = s.profile_velocity_;
    info.current_limit = s.current_limit_;
    info.send_data_flag = static_cast<uint8_t>(s.send_data_flag_);
    info.external_encoder_flag = static_cast<uint8_t>(s.external_encoder_flag_);
    info.joint_resolution = static_cast<int16_t>(s.joint_resolution_);
    info.servo_resolution = static_cast<int16_t>(s.servo_resolution_);
  }
}

void DirectServoRosModule::servoControlCallbackStatic_(const void* msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;

  const auto* msg = reinterpret_cast<const spinal_msgs__msg__ServoControlCmd*>(msgin);
  if (msg->index.size != msg->angles.size) return;

  (void)instance_->servo_.applyServoControlCommand(
    msg->index.data,
    msg->angles.data,
    msg->index.size);
}

void DirectServoRosModule::servoTorqueControlCallbackStatic_(const void* msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;

  const auto* msg = reinterpret_cast<const spinal_msgs__msg__ServoTorqueCmd*>(msgin);
  if (msg->index.size != msg->torque_enable.size) return;

  (void)instance_->servo_.applyServoTorqueCommand(
    msg->index.data,
    msg->torque_enable.data,
    msg->index.size);
}

void DirectServoRosModule::jointProfilesCallbackStatic_(const void* msgin)
{
  if (instance_ == nullptr || msgin == nullptr) return;

  const auto* msg = reinterpret_cast<const spinal_msgs__msg__JointProfiles*>(msgin);
  DirectServoJointProfile profiles[MAX_SERVO_NUM]{};
  const size_t profile_num = (msg->joints.size > MAX_SERVO_NUM) ? MAX_SERVO_NUM : msg->joints.size;

  for (size_t i = 0; i < profile_num; ++i) {
    const spinal_msgs__msg__JointProfile& src = msg->joints.data[i];
    profiles[i].servo_id = src.servo_id;
    profiles[i].angle_sgn = src.angle_sgn;
    profiles[i].angle_scale = src.angle_scale;
    profiles[i].zero_point_offset = src.zero_point_offset;
  }

  instance_->servo_.applyJointProfiles(profiles, profile_num);
}

void DirectServoRosModule::servoConfigCallbackStatic_(const void* req_msg, void* res_msg)
{
  if (instance_ == nullptr || req_msg == nullptr || res_msg == nullptr) return;

  const auto* req = reinterpret_cast<const spinal_msgs__srv__SetDirectServoConfig_Request*>(req_msg);
  auto* res = reinterpret_cast<spinal_msgs__srv__SetDirectServoConfig_Response*>(res_msg);

  res->success = instance_->servo_.applyConfigCommand(
    req->command,
    req->data.data,
    req->data.size);
  res->data.size = 0;
}

void DirectServoRosModule::boardInfoCallbackStatic_(const void* req_msg, void* res_msg)
{
  (void)req_msg;
  if (instance_ == nullptr || res_msg == nullptr) return;

  auto* res = reinterpret_cast<spinal_msgs__srv__GetBoardInfo_Response*>(res_msg);
  instance_->fillBoardInfo_();
  res->boards = instance_->board_info_res_.boards;
}
