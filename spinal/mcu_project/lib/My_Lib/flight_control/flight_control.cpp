#include "flight_control/flight_control.h"

#ifndef SIMULATION
#include <map>
#endif

#ifdef SIMULATION
#include "thruster/simulation/thruster_manager.h"
#else
#include "servo/servo.h"
#include "thruster/board/thruster_manager.h"
#endif

void FlightControl::init(
  StateEstimate* estimator,
  ThrusterManager* thruster,
#ifndef SIMULATION
  DirectServo* servo
#else
  void* servo
#endif
)
{
#ifdef SIMULATION
  (void)servo;
#endif

  estimator_ = estimator;
  thruster_ = thruster;
#ifndef SIMULATION
  servo_ = servo;
#endif

  att_controller_.init(estimator_);
  start_control_flag_ = false;
  force_landing_flag_ = false;
  gimbal_set_flag_ = false;
  config_ack_pending_ = false;
  physical_motor_count_ = 0;
}

void FlightControl::update()
{
  ThrusterControlLimits limits;
  if (thruster_ != nullptr) {
    limits = thruster_->getControlLimits();
  }

  att_controller_.setThrusterLimits(limits);
  att_controller_.update();

  if (!force_landing_flag_ && att_controller_.getForceLandingFlag()) {
    force_landing_flag_ = true;
    setConfigAck_(FlightControlCommand::FORCE_LANDING_CMD);
  }

  if (thruster_ != nullptr) {
    thruster_->setRotorDivider(att_controller_.getRotorDivider());
    (void)thruster_->outputThrust(
      att_controller_.getTargetThrust(),
      att_controller_.getThrusterCount(),
      start_control_flag_);
  }

  applyGimbalOutput_();
}

bool FlightControl::applyFlightConfig(uint8_t command)
{
  switch (command) {
    case FlightControlCommand::ARM_ON_CMD:
    {
      ThrusterControlLimits limits;
      if (thruster_ != nullptr) {
        limits = thruster_->getControlLimits();
      }

      att_controller_.setThrusterLimits(limits);
      if (!att_controller_.activated()) return false;

      force_landing_flag_ = false;
      att_controller_.setForceLandingFlag(false);

      start_control_flag_ = true;
      att_controller_.setStartControlFlag(true);

      setConfigAck_(FlightControlCommand::ARM_ON_CMD);
      return true;
    }

    case FlightControlCommand::ARM_OFF_CMD:
      start_control_flag_ = false;
      att_controller_.setStartControlFlag(false);
      force_landing_flag_ = false;
      att_controller_.setForceLandingFlag(false);
      setConfigAck_(FlightControlCommand::ARM_OFF_CMD);
      return true;

    case FlightControlCommand::FORCE_LANDING_CMD:
      force_landing_flag_ = true;
      att_controller_.setForceLandingFlag(true);
      setConfigAck_(FlightControlCommand::FORCE_LANDING_CMD);
      return true;

    case FlightControlCommand::INTEGRATION_CONTROL_ON_CMD:
      att_controller_.setIntegrateFlag(true);
      return true;

    case FlightControlCommand::INTEGRATION_CONTROL_OFF_CMD:
      att_controller_.setIntegrateFlag(false);
      return true;

    default:
      break;
  }

  return false;
}

void FlightControl::applyUavInfo(uint8_t motor_num, int8_t uav_model)
{
  physical_motor_count_ = motor_num;
  att_controller_.setUavModel(uav_model);

  if (thruster_ != nullptr) {
    thruster_->setRotorDivider(att_controller_.getRotorDivider());
  }

  configureMotorCount_();
}

void FlightControl::applyGimbalDof(uint8_t gimbal_dof)
{
  if (gimbal_dof != 0 && !gimbal_set_flag_) {
    att_controller_.setGimbalDof(gimbal_dof);
    att_controller_.setRotorCoef(gimbal_dof + 1);
    gimbal_set_flag_ = true;
    configureMotorCount_();
  }
}

bool FlightControl::applyFourAxisCommand(const FlightControlFourAxisCommand& cmd)
{
  return att_controller_.applyFourAxisCommand(cmd);
}

bool FlightControl::applyRpyGains(const FlightControlRpyTerms& gains)
{
  return att_controller_.applyRpyGains(gains);
}

bool FlightControl::applyPMatrixInertia(const FlightControlPMatrixPseudoInverseWithInertia& msg)
{
  return att_controller_.applyPMatrixInertia(msg);
}

bool FlightControl::applyTorqueAllocationMatrixInv(const FlightControlTorqueAllocationMatrixInv& msg)
{
  return att_controller_.applyTorqueAllocationMatrixInv(msg);
}

void FlightControl::applyOffsetRotation(const FlightControlDesireCoord& msg)
{
  att_controller_.applyOffsetRotation(msg);
}

void FlightControl::setAttitudeControlFlag(bool flag)
{
  att_controller_.setAttitudeControlFlag(flag);
}

bool FlightControl::consumeConfigAck(uint8_t& ack)
{
  if (!config_ack_pending_) return false;
  ack = config_ack_;
  config_ack_pending_ = false;
  return true;
}

void FlightControl::configureMotorCount_()
{
  if (physical_motor_count_ == 0) return;

  uint16_t allocation_count =
    static_cast<uint16_t>(physical_motor_count_) * static_cast<uint16_t>(att_controller_.getRotorCoef());
  if (allocation_count > MAX_FLIGHT_CONTROL_MOTOR_NUM) {
    allocation_count = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  }

  att_controller_.setMotorNumber(allocation_count);
}

void FlightControl::setConfigAck_(uint8_t ack)
{
  config_ack_ = ack;
  config_ack_pending_ = true;
}

void FlightControl::applyGimbalOutput_()
{
#ifndef SIMULATION
  if (servo_ == nullptr || !servo_->connected()) return;

  const uint8_t gimbal_dof = att_controller_.getGimbalDof();
  if (gimbal_dof == 0) return;

  const uint16_t thruster_count = att_controller_.getThrusterCount();
  const float* target_gimbal_angles = att_controller_.getTargetGimbalAngles();

  std::map<uint8_t, float> gimbal_map;

  if (gimbal_dof == 2) {
    for (uint16_t i = 0; i < thruster_count; ++i) {
      gimbal_map[static_cast<uint8_t>(2 * i)] =
        start_control_flag_ ? target_gimbal_angles[2 * i] : 0.0f;
      gimbal_map[static_cast<uint8_t>(2 * i + 1)] =
        start_control_flag_ ? target_gimbal_angles[2 * i + 1] : 0.0f;
    }
  } else if (gimbal_dof == 1) {
    for (uint16_t i = 0; i < thruster_count; ++i) {
      gimbal_map[static_cast<uint8_t>(i)] =
        start_control_flag_ ? target_gimbal_angles[i] : 0.0f;
    }
  }

  if (gimbal_map.empty()) return;

  if (start_control_flag_) {
    servo_->setGoalAngle(gimbal_map, ValueType::RADIAN);
  } else {
    servo_->torqueEnable(gimbal_map);
  }
#endif
}
