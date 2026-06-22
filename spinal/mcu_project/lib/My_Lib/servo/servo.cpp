/**
******************************************************************************
* File Name          : servo.cpp
* Description        : Universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
******************************************************************************
*/

#include "servo/servo.h"

DirectServo::DirectServo()
{
  for (uint8_t i = 0; i < MAX_SERVO_NUM; ++i) {
    joint_profiles_[i].servo_id = i;
    joint_profiles_[i].angle_sgn = 1;
    joint_profiles_[i].angle_scale = 1.0f;
    joint_profiles_[i].zero_point_offset = 0;
  }
}

bool DirectServo::init(UART_HandleTypeDef* huart, osMutexId* mutex)
{
#if !STM32H7_V2
#ifdef STM32H7
  const uint32_t raw_baudrate = huart->Init.BaudRate;
  HAL_UART_DeInit(huart);
  huart->Init.BaudRate = 1000000;
  HAL_UART_Init(huart);
#endif
#endif

  servo_handler_.init(huart, mutex);

  if (servo_handler_.getServoNum() == 0) {
#if !STM32H7_V2
#ifdef STM32H7
    HAL_UART_DeInit(huart);
    huart->Init.BaudRate = raw_baudrate;
    HAL_UART_Init(huart);
#endif
#endif
    connected_ = false;
    return false;
  }

  servo_last_pub_time_ = 0;
  servo_torque_last_pub_time_ = 0;
  connected_ = true;
  return true;
}

void DirectServo::update()
{
  if (!connected_) return;
  servo_handler_.update();
}

bool DirectServo::applyServoControlCommand(const uint8_t* index, const int16_t* angles, size_t count)
{
  if (index == nullptr || angles == nullptr) return false;

  for (size_t i = 0; i < count; ++i) {
    if (!setGoalPosition_(index[i], static_cast<int32_t>(angles[i]))) return false;
  }

  return true;
}

bool DirectServo::applyServoTorqueCommand(const uint8_t* index, const uint8_t* torque_enable, size_t count)
{
  if (index == nullptr || torque_enable == nullptr) return false;

  for (size_t i = 0; i < count; ++i) {
    if (!isValidServoIndex_(index[i])) return false;

    ServoData& s = servo_handler_.getServo()[index[i]];
    s.torque_enable_ = torque_enable[i] != 0;
    servo_handler_.setTorqueFromPresetnPos(index[i]);
  }

  return true;
}

void DirectServo::applyJointProfiles(const DirectServoJointProfile* profiles, size_t count)
{
  if (profiles == nullptr) return;

  const size_t n = (count > MAX_SERVO_NUM) ? MAX_SERVO_NUM : count;
  for (size_t i = 0; i < n; ++i) {
    joint_profiles_[i] = profiles[i];
  }
}

bool DirectServo::applyConfigCommand(uint8_t command, const int32_t* data, size_t data_size)
{
  if (data == nullptr && data_size > 0) return false;

  if (command == DirectServoConfigCommand::SET_DYNAMIXEL_TTL_RS485_MIXED) {
    if (data_size < 1) return false;
    FlashMemory::erase();
    FlashMemory::write();
    return true;
  }

  if (data_size < 1) return false;

  const uint8_t servo_index = static_cast<uint8_t>(data[0]);
  if (!isValidServoIndex_(servo_index)) return false;

  ServoData& s = servo_handler_.getServo()[servo_index];

  switch (command) {
    case DirectServoConfigCommand::SET_SERVO_HOMING_OFFSET:
      if (data_size < 2 || s.torque_enable_) return false;
      s.calib_value_ = data[1];
      servo_handler_.setHomingOffset(servo_index);
      return true;

    case DirectServoConfigCommand::SET_SERVO_PID_GAIN:
      if (data_size < 4 || s.torque_enable_) return false;
      s.p_gain_ = data[1];
      s.i_gain_ = data[2];
      s.d_gain_ = data[3];
      servo_handler_.setPositionGains(servo_index);
      FlashMemory::erase();
      FlashMemory::write();
      return true;

    case DirectServoConfigCommand::SET_SERVO_PROFILE_VEL:
      if (data_size < 2) return false;
      s.profile_velocity_ = data[1];
      servo_handler_.setProfileVelocity(servo_index);
      FlashMemory::erase();
      FlashMemory::write();
      return true;

    case DirectServoConfigCommand::SET_SERVO_SEND_DATA_FLAG:
      if (data_size < 2) return false;
      s.send_data_flag_ = data[1];
      FlashMemory::erase();
      FlashMemory::write();
      return true;

    case DirectServoConfigCommand::SET_SERVO_CURRENT_LIMIT:
      if (data_size < 2) return false;
      s.current_limit_ = data[1];
      servo_handler_.setCurrentLimit(servo_index);
      return true;

    case DirectServoConfigCommand::SET_SERVO_EXTERNAL_ENCODER_FLAG:
      if (data_size < 2 || s.torque_enable_) return false;
      s.external_encoder_flag_ = data[1];
      s.first_get_pos_flag_ = true;
      if (!s.external_encoder_flag_) {
        s.servo_resolution_ = 1;
        s.joint_resolution_ = 1;
        s.resolution_ratio_ = 1;
      }
      FlashMemory::erase();
      FlashMemory::write();
      return true;

    case DirectServoConfigCommand::SET_SERVO_RESOLUTION_RATIO:
      if (data_size < 3 || s.torque_enable_) return false;
      s.joint_resolution_ = data[1];
      s.servo_resolution_ = data[2];
      s.hardware_error_status_ &= ((1 << RESOLUTION_RATIO_ERROR) - 1);

      if (s.servo_resolution_ == 65535 || s.joint_resolution_ == 65535) {
        s.hardware_error_status_ |= (1 << RESOLUTION_RATIO_ERROR);
        s.resolution_ratio_ = 1;
      } else {
        s.resolution_ratio_ = static_cast<float>(s.servo_resolution_) /
                              static_cast<float>(s.joint_resolution_);
        s.first_get_pos_flag_ = true;
        FlashMemory::erase();
        FlashMemory::write();
      }
      return true;

    default:
      return false;
  }
}

bool DirectServo::statePublishReady(bool flag_send_asap) const
{
  if (!connected_) return false;

  const uint32_t now_time = HAL_GetTick();
  if (flag_send_asap && servo_handler_.getStateUpdatedFlag()) return true;

  return now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL_MS;
}

bool DirectServo::torqueStatePublishReady() const
{
  if (!connected_) return false;

  const uint32_t now_time = HAL_GetTick();
  return now_time - servo_torque_last_pub_time_ >= SERVO_TORQUE_PUB_INTERVAL_MS;
}

void DirectServo::markStatePublished()
{
  servo_last_pub_time_ = HAL_GetTick();
  servo_handler_.setStateUpdatedFlag(false);
}

void DirectServo::markTorqueStatePublished()
{
  servo_torque_last_pub_time_ = HAL_GetTick();
}

void DirectServo::torqueEnable(const std::map<uint8_t, float>& servo_map)
{
  for (const auto& servo : servo_map) {
    const uint8_t index = servo.first;
    if (!isValidServoIndex_(index)) return;

    ServoData& s = servo_handler_.getServo()[index];
    if (servo.second && !s.torque_enable_) {
      s.torque_enable_ = true;
      servo_handler_.setTorque(index);
    } else if (!servo.second && s.torque_enable_) {
      s.torque_enable_ = false;
      servo_handler_.setTorque(index);
    }
  }
}

void DirectServo::setGoalAngle(const std::map<uint8_t, float>& servo_map, uint8_t value_type)
{
  for (const auto& servo : servo_map) {
    const uint8_t index = servo.first;
    if (!isValidServoIndex_(index)) return;

    int32_t goal_pos = 0;
    if (value_type == ValueType::BIT) {
      goal_pos = static_cast<int32_t>(servo.second);
    } else if (value_type == ValueType::RADIAN) {
      const DirectServoJointProfile& joint_prof = joint_profiles_[index];
      if (joint_prof.angle_scale == 0.0f) return;
      goal_pos = static_cast<int32_t>(
        servo.second * joint_prof.angle_sgn / joint_prof.angle_scale +
        joint_prof.zero_point_offset);
    } else {
      return;
    }

    if (!setGoalPosition_(index, goal_pos)) return;
  }
}

uint16_t DirectServo::getDynamixelTtlRs485Mixed() const
{
#if DYNAMIXEL
  return servo_handler_.getTTLRS485Mixed();
#else
  return 0;
#endif
}

bool DirectServo::isValidServoIndex_(uint8_t index) const
{
  return index < servo_handler_.getServoNum();
}

bool DirectServo::setGoalPosition_(uint8_t index, int32_t goal_pos)
{
  if (!isValidServoIndex_(index)) return false;

  ServoData& s = servo_handler_.getServo()[index];
  s.setGoalPosition(goal_pos);
  if (!s.torque_enable_) {
    s.torque_enable_ = true;
    servo_handler_.setTorque(index);
  }

  return true;
}
