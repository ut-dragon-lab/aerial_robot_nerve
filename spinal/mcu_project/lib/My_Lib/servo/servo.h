/**
******************************************************************************
* File Name          : servo.h
* Description        : Universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
******************************************************************************
*/

#ifndef APPLICATION_SERVO_TEMP_SERVO_H_
#define APPLICATION_SERVO_TEMP_SERVO_H_

#include <cstddef>
#include <cstdint>
#include <map>

#include <config.h>

#include "flashmemory/flashmemory.h"
#include "servo/drivers/Dynamixel/dynamixel_serial.h"
#include "servo/drivers/kondo_servo/kondo_servo.h"

class Initializer;

namespace ValueType
{
  enum
  {
    BIT = 0,
    RADIAN = 1
  };
}

namespace DirectServoConfigCommand
{
  enum
  {
    SET_SERVO_HOMING_OFFSET = 1,
    SET_SERVO_PID_GAIN = 2,
    SET_SERVO_PROFILE_VEL = 3,
    SET_SERVO_SEND_DATA_FLAG = 4,
    SET_SERVO_CURRENT_LIMIT = 5,
    SET_DYNAMIXEL_TTL_RS485_MIXED = 6,
    SET_SERVO_EXTERNAL_ENCODER_FLAG = 7,
    SET_SERVO_RESOLUTION_RATIO = 8
  };
}

struct DirectServoJointProfile
{
  uint8_t servo_id{0};
  int8_t angle_sgn{1};
  float angle_scale{1.0f};
  int16_t zero_point_offset{0};
};

class DirectServo
{
public:
  DirectServo();
  ~DirectServo() = default;

  bool init(UART_HandleTypeDef* huart, osMutexId* mutex = nullptr);
  void update();

  bool connected() const { return connected_; }

  bool applyServoControlCommand(const uint8_t* index, const int16_t* angles, size_t count);
  bool applyServoTorqueCommand(const uint8_t* index, const uint8_t* torque_enable, size_t count);
  void applyJointProfiles(const DirectServoJointProfile* profiles, size_t count);
  bool applyConfigCommand(uint8_t command, const int32_t* data, size_t data_size);

  bool statePublishReady(bool flag_send_asap) const;
  bool torqueStatePublishReady() const;
  void markStatePublished();
  void markTorqueStatePublished();

  void torqueEnable(const std::map<uint8_t, float>& servo_map);
  void setGoalAngle(const std::map<uint8_t, float>& servo_map, uint8_t value_type = ValueType::BIT);

#if KONDO
  KondoServo& getServoHandler() { return servo_handler_; }
  KondoServo& getServoHnadler() { return servo_handler_; }
#else
  DynamixelSerial& getServoHandler() { return servo_handler_; }
  DynamixelSerial& getServoHnadler() { return servo_handler_; }
#endif

  unsigned int getServoNum() const { return servo_handler_.getServoNum(); }
  const ServoData& getServoData(size_t index) const { return servo_handler_.getServo()[index]; }

  uint8_t getBoardId() const { return 0; }
  uint8_t getImuSendDataFlag() const { return 1; }
  uint16_t getDynamixelTtlRs485Mixed() const;

  uint32_t rad2Pos(float angle, float scale, uint32_t zero_point_pos) const
  {
    return static_cast<uint32_t>(angle / scale + zero_point_pos);
  }

private:
  static constexpr uint32_t SERVO_PUB_INTERVAL_MS = 20;        // 50Hz
  static constexpr uint32_t SERVO_TORQUE_PUB_INTERVAL_MS = 1000; // 1Hz

  bool isValidServoIndex_(uint8_t index) const;
  bool setGoalPosition_(uint8_t index, int32_t goal_pos);

  DirectServoJointProfile joint_profiles_[MAX_SERVO_NUM]{};

#if KONDO
  KondoServo servo_handler_;
#else
  DynamixelSerial servo_handler_;
#endif

  uint32_t servo_last_pub_time_{0};
  uint32_t servo_torque_last_pub_time_{0};
  bool connected_{false};

  friend class Initializer;
};

#endif /* APPLICATION_SERVO_SERVO_H_ */
