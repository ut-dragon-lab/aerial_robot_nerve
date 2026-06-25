#include "flight_control/attitude/attitude_control.h"

#include <cmath>

#ifdef SIMULATION
#include <chrono>
#endif

namespace
{
int16_t toInt16(float value)
{
  return static_cast<int16_t>(value);
}
}

AttitudeController::AttitudeController()
{
  offset_rot_.identity();
  inertia_.identity();
  reset();
}

void AttitudeController::init(StateEstimate* estimator)
{
  estimator_ = estimator;
  reset();
}

void AttitudeController::reset()
{
  for (size_t i = 0; i < MAX_FLIGHT_CONTROL_MOTOR_NUM; ++i) {
    target_thrust_[i] = 0.0f;
    target_gimbal_angles_[i] = 0.0f;
    gyro_moment_compensation_[i] = 0.0f;

    base_thrust_term_[i] = 0.0f;
    roll_pitch_term_[i] = 0.0f;
    yaw_term_[i] = 0.0f;
    extra_yaw_pi_term_[i] = 0.0f;

    for (size_t j = 0; j < 3; ++j) {
      thrust_p_gain_[i][j] = 0.0f;
      thrust_i_gain_[i][j] = 0.0f;
      thrust_d_gain_[i][j] = 0.0f;
      torque_allocation_matrix_inv_[i][j] = 0.0f;
    }

    for (size_t j = 0; j < 4; ++j) {
      p_matrix_pseudo_inverse_[i][j] = 0.0f;
    }

    control_term_.motors[i] = FlightControlRpyTerm();
  }

  for (size_t i = 0; i < 3; ++i) {
    target_angle_[i] = 0.0f;
    error_angle_i_[i] = 0.0f;

    torque_p_gain_[i] = 0.0f;
    torque_i_gain_[i] = 0.0f;
    torque_d_gain_[i] = 0.0f;
  }

  max_yaw_term_index_ = -1;
  integrate_flag_ = false;
  failsafe_ = false;
  flight_command_last_stamp_ = nowMillis_();
  update_last_time_ = 0;
  control_term_pub_last_time_ = 0;
  control_feedback_state_pub_last_time_ = 0;
  control_feedback_state_ = FlightControlRpyTerm();
  control_term_.motors_count = motor_number_;
}

bool AttitudeController::update()
{
  if (!start_control_flag_ || !att_control_flag_) {
    clearOutputTargets_();
    return false;
  }

  if (estimator_ == nullptr || estimator_->getAttEstimator() == nullptr) {
    clearOutputTargets_();
    return false;
  }

  if (motor_number_ == 0 || !limits_.configured) {
    clearOutputTargets_();
    return false;
  }

  const uint32_t now = nowMillis_();
  const float dt = deltaTime_();

  if (failsafe_ && !force_landing_flag_ &&
      static_cast<int32_t>(now - flight_command_last_stamp_) >
      static_cast<int32_t>(FlightControlConstants::FLIGHT_COMMAND_TIMEOUT_MS)) {
    setForceLandingFlag(true);
  }

  ap::Matrix3f base_rot = estimator_->getAttEstimator()->getRotation();
  ap::Vector3f base_vel = estimator_->getAttEstimator()->getAngular();
  ap::Matrix3f rot = base_rot * offset_rot_.transposed();
  ap::Vector3f vel = offset_rot_ * base_vel;
  ap::Vector3f angles;
  rot.to_euler(&angles.x, &angles.y, &angles.z);

  if (!force_landing_flag_ &&
      (fabsf(angles[FlightControlAxis::X]) > FlightControlConstants::MAX_TILT_ANGLE ||
       fabsf(angles[FlightControlAxis::Y]) > FlightControlConstants::MAX_TILT_ANGLE)) {
    setForceLandingFlag(true);
    error_angle_i_[FlightControlAxis::X] = 0.0f;
    error_angle_i_[FlightControlAxis::Y] = 0.0f;
  }

  if (force_landing_flag_) {
    target_angle_[FlightControlAxis::X] = 0.0f;
    target_angle_[FlightControlAxis::Y] = 0.0f;
    target_angle_[FlightControlAxis::Z] = 0.0f;

    for (size_t i = 0; i < motor_number_; ++i) {
      extra_yaw_pi_term_[i] = 0.0f;
    }
  }

  const ap::Vector3f gyro_moment = vel % (inertia_ * vel);

  float error_angle[3]{};
  for (size_t axis = 0; axis < 3; ++axis) {
    error_angle[axis] = target_angle_[axis] - angles[axis];
    if (integrate_flag_) {
      error_angle_i_[axis] += error_angle[axis] * dt;
    }

    if (axis == FlightControlAxis::X) {
      control_feedback_state_.roll_p = toInt16(error_angle[axis] * 1000.0f);
      control_feedback_state_.roll_i = toInt16(error_angle_i_[axis] * 1000.0f);
      control_feedback_state_.roll_d = toInt16(vel[axis] * 1000.0f);
    } else if (axis == FlightControlAxis::Y) {
      control_feedback_state_.pitch_p = toInt16(error_angle[axis] * 1000.0f);
      control_feedback_state_.pitch_i = toInt16(error_angle_i_[axis] * 1000.0f);
      control_feedback_state_.pitch_d = toInt16(vel[axis] * 1000.0f);
    } else if (axis == FlightControlAxis::Z) {
      control_feedback_state_.yaw_d = toInt16(vel[axis] * 1000.0f);
    }
  }

  for (size_t i = 0; i < motor_number_; ++i) {
    for (size_t axis = 0; axis < 3; ++axis) {
      const float p_term = error_angle[axis] * thrust_p_gain_[i][axis];
      const float i_term = error_angle_i_[axis] * thrust_i_gain_[i][axis];
      const float d_term = -vel[axis] * thrust_d_gain_[i][axis];

      if (axis == FlightControlAxis::X) {
        roll_pitch_term_[i] = p_term + i_term + d_term;
        control_term_.motors[i].roll_p = toInt16(p_term * 1000.0f);
        control_term_.motors[i].roll_i = toInt16(i_term * 1000.0f);
        control_term_.motors[i].roll_d = toInt16(d_term * 1000.0f);
      } else if (axis == FlightControlAxis::Y) {
        roll_pitch_term_[i] += p_term + i_term + d_term;
        control_term_.motors[i].pitch_p = toInt16(p_term * 1000.0f);
        control_term_.motors[i].pitch_i = toInt16(i_term * 1000.0f);
        control_term_.motors[i].pitch_d = toInt16(d_term * 1000.0f);
      } else if (axis == FlightControlAxis::Z) {
        yaw_term_[i] = extra_yaw_pi_term_[i] + d_term;
        control_term_.motors[i].yaw_d = toInt16(d_term * 1000.0f);
      }
    }

    const float gyro_moment_compensate =
      p_matrix_pseudo_inverse_[i][0] * gyro_moment.x +
      p_matrix_pseudo_inverse_[i][1] * gyro_moment.y +
      p_matrix_pseudo_inverse_[i][2] * gyro_moment.z;
    gyro_moment_compensation_[i] = gyro_moment_compensate;
    roll_pitch_term_[i] += gyro_moment_compensate;
  }

  if (force_landing_flag_) {
    float total_thrust = 0.0f;
    for (size_t i = 0; i < motor_number_; ++i) {
      total_thrust += base_thrust_term_[i];
    }

    const float average_thrust = motor_number_ > 0 ? total_thrust / motor_number_ : 0.0f;
    if (average_thrust > limits_.force_landing_thrust && average_thrust > 0.0f) {
      for (size_t i = 0; i < motor_number_; ++i) {
        base_thrust_term_[i] -=
          base_thrust_term_[i] / average_thrust * FlightControlConstants::FORCE_LANDING_INTEGRAL;
      }
    }
  }

  return computeTargetThrust_();
}

bool AttitudeController::applyFourAxisCommand(const FlightControlFourAxisCommand& cmd)
{
  if (!start_control_flag_) return false;

  if (fabsf(cmd.angles[0]) > FlightControlConstants::MAX_TILT_ANGLE ||
      fabsf(cmd.angles[1]) > FlightControlConstants::MAX_TILT_ANGLE) {
    setForceLandingFlag(true);
  }

  if (cmd.base_thrust_count != motor_number_) return false;

  if (force_landing_flag_) {
    float total_thrust = 0.0f;
    for (size_t i = 0; i < motor_number_; ++i) {
      total_thrust += cmd.base_thrust[i];
    }
    const float average_thrust = motor_number_ > 0 ? total_thrust / motor_number_ : 0.0f;
    if (average_thrust < limits_.force_landing_thrust) return false;
  }

  if (!failsafe_) failsafe_ = true;
  flight_command_last_stamp_ = nowMillis_();

  target_angle_[FlightControlAxis::X] = cmd.angles[0];
  target_angle_[FlightControlAxis::Y] = cmd.angles[1];

  for (size_t i = 0; i < motor_number_; ++i) {
    base_thrust_term_[i] = cmd.base_thrust[i];

    if (max_yaw_term_index_ != -1 && thrust_d_gain_[max_yaw_term_index_][FlightControlAxis::Z] != 0.0f) {
      extra_yaw_pi_term_[i] =
        cmd.angles[FlightControlAxis::Z] *
        thrust_d_gain_[i][FlightControlAxis::Z] /
        thrust_d_gain_[max_yaw_term_index_][FlightControlAxis::Z];
    }
  }

  return true;
}

bool AttitudeController::applyRpyGains(const FlightControlRpyTerms& gains)
{
  if (gains.motors_count != motor_number_ && gains.motors_count != 1) return false;

  if (gains.motors_count == 1) {
    torque_p_gain_[FlightControlAxis::X] = gains.motors[0].roll_p * 0.001f;
    torque_p_gain_[FlightControlAxis::Y] = gains.motors[0].pitch_p * 0.001f;
    torque_i_gain_[FlightControlAxis::X] = gains.motors[0].roll_i * 0.001f;
    torque_i_gain_[FlightControlAxis::Y] = gains.motors[0].pitch_i * 0.001f;
    torque_d_gain_[FlightControlAxis::X] = gains.motors[0].roll_d * 0.001f;
    torque_d_gain_[FlightControlAxis::Y] = gains.motors[0].pitch_d * 0.001f;
    torque_d_gain_[FlightControlAxis::Z] = gains.motors[0].yaw_d * 0.001f;
    thrustGainMapping_();
  } else {
    for (size_t i = 0; i < motor_number_; ++i) {
      thrust_p_gain_[i][FlightControlAxis::X] = gains.motors[i].roll_p * 0.001f;
      thrust_i_gain_[i][FlightControlAxis::X] = gains.motors[i].roll_i * 0.001f;
      thrust_d_gain_[i][FlightControlAxis::X] = gains.motors[i].roll_d * 0.001f;
      thrust_p_gain_[i][FlightControlAxis::Y] = gains.motors[i].pitch_p * 0.001f;
      thrust_i_gain_[i][FlightControlAxis::Y] = gains.motors[i].pitch_i * 0.001f;
      thrust_d_gain_[i][FlightControlAxis::Y] = gains.motors[i].pitch_d * 0.001f;
      thrust_d_gain_[i][FlightControlAxis::Z] = gains.motors[i].yaw_d * 0.001f;
    }
  }

  maxYawGainIndex_();
  return true;
}

bool AttitudeController::applyPMatrixInertia(const FlightControlPMatrixPseudoInverseWithInertia& msg)
{
  if (motor_number_ == 0) return false;
  if (msg.pseudo_inverse_count != motor_number_) return false;

  for (size_t i = 0; i < motor_number_; ++i) {
    p_matrix_pseudo_inverse_[i][0] = msg.pseudo_inverse[i].r * 0.001f;
    p_matrix_pseudo_inverse_[i][1] = msg.pseudo_inverse[i].p * 0.001f;
    p_matrix_pseudo_inverse_[i][2] = msg.pseudo_inverse[i].y * 0.001f;
  }

  inertia_ = ap::Matrix3f(
    msg.inertia[0] * 0.001f, msg.inertia[3] * 0.001f, msg.inertia[5] * 0.001f,
    msg.inertia[3] * 0.001f, msg.inertia[1] * 0.001f, msg.inertia[4] * 0.001f,
    msg.inertia[5] * 0.001f, msg.inertia[4] * 0.001f, msg.inertia[2] * 0.001f);

  return true;
}

bool AttitudeController::applyTorqueAllocationMatrixInv(const FlightControlTorqueAllocationMatrixInv& msg)
{
  if (motor_number_ == 0 || !start_control_flag_) return false;
  if (msg.rows_count != motor_number_) return false;

  for (size_t i = 0; i < motor_number_; ++i) {
    torque_allocation_matrix_inv_[i][FlightControlAxis::X] = msg.rows[i].x * 0.001f;
    torque_allocation_matrix_inv_[i][FlightControlAxis::Y] = msg.rows[i].y * 0.001f;
    torque_allocation_matrix_inv_[i][FlightControlAxis::Z] = msg.rows[i].z * 0.001f;
  }

  thrustGainMapping_();
  maxYawGainIndex_();
  return true;
}

void AttitudeController::applyOffsetRotation(const FlightControlDesireCoord& msg)
{
  offset_rot_.from_euler(msg.roll, msg.pitch, msg.yaw);
}

bool AttitudeController::activated() const
{
  return motor_number_ > 0 &&
         uav_model_ >= FlightControlUavModel::DRONE &&
         limits_.configured &&
         limits_.max_thrust > 0.0f;
}

void AttitudeController::setStartControlFlag(bool start_control_flag)
{
  start_control_flag_ = start_control_flag;
  if (!start_control_flag_) {
    reset();
  } else {
    update_last_time_ = nowMillis_();
  }
}

void AttitudeController::setUavModel(int8_t uav_model)
{
  uav_model_ = uav_model;
  rotor_devider_ = (uav_model_ == FlightControlUavModel::DRAGON) ? 2 : 1;
}

void AttitudeController::setMotorNumber(uint16_t motor_number)
{
  if (motor_number > MAX_FLIGHT_CONTROL_MOTOR_NUM) {
    motor_number = MAX_FLIGHT_CONTROL_MOTOR_NUM;
  }

  if (motor_number_ > 0 && motor_number_ != motor_number && start_control_flag_) {
    motor_number_ = 0;
    control_term_.motors_count = 0;
    return;
  }

  if (motor_number == 0) return;

  motor_number_ = motor_number;
  control_term_.motors_count = motor_number_;
}

uint16_t AttitudeController::getThrusterCount() const
{
  if (rotor_coef_ == 0) return motor_number_;
  return motor_number_ / rotor_coef_;
}

size_t AttitudeController::getTargetGimbalAngleCount() const
{
  return getThrusterCount() * gimbal_dof_;
}

bool AttitudeController::controlTermPublishReady(bool update_last_time)
{
  const uint32_t now = nowMillis_();
  if (now - control_term_pub_last_time_ <= FlightControlConstants::CONTROL_TERM_PUB_INTERVAL_MS) return false;
  if (update_last_time) control_term_pub_last_time_ = now;
  return true;
}

bool AttitudeController::controlFeedbackStatePublishReady(bool update_last_time)
{
  const uint32_t now = nowMillis_();
  if (now - control_feedback_state_pub_last_time_ <=
      FlightControlConstants::CONTROL_FEEDBACK_STATE_PUB_INTERVAL_MS) {
    return false;
  }
  if (update_last_time) control_feedback_state_pub_last_time_ = now;
  return true;
}

void AttitudeController::thrustGainMapping_()
{
  for (size_t i = 0; i < motor_number_; ++i) {
    thrust_p_gain_[i][FlightControlAxis::X] =
      torque_allocation_matrix_inv_[i][FlightControlAxis::X] * torque_p_gain_[FlightControlAxis::X];
    thrust_i_gain_[i][FlightControlAxis::X] =
      torque_allocation_matrix_inv_[i][FlightControlAxis::X] * torque_i_gain_[FlightControlAxis::X];
    thrust_d_gain_[i][FlightControlAxis::X] =
      torque_allocation_matrix_inv_[i][FlightControlAxis::X] * torque_d_gain_[FlightControlAxis::X];
    thrust_p_gain_[i][FlightControlAxis::Y] =
      torque_allocation_matrix_inv_[i][FlightControlAxis::Y] * torque_p_gain_[FlightControlAxis::Y];
    thrust_i_gain_[i][FlightControlAxis::Y] =
      torque_allocation_matrix_inv_[i][FlightControlAxis::Y] * torque_i_gain_[FlightControlAxis::Y];
    thrust_d_gain_[i][FlightControlAxis::Y] =
      torque_allocation_matrix_inv_[i][FlightControlAxis::Y] * torque_d_gain_[FlightControlAxis::Y];
    thrust_d_gain_[i][FlightControlAxis::Z] =
      torque_allocation_matrix_inv_[i][FlightControlAxis::Z] * torque_d_gain_[FlightControlAxis::Z];
  }
}

void AttitudeController::maxYawGainIndex_()
{
  float max_yaw_gain = 0.0f;
  max_yaw_term_index_ = -1;
  for (size_t i = 0; i < motor_number_; ++i) {
    if (thrust_d_gain_[i][FlightControlAxis::Z] > max_yaw_gain) {
      max_yaw_gain = thrust_d_gain_[i][FlightControlAxis::Z];
      max_yaw_term_index_ = static_cast<int>(i);
    }
  }
}

bool AttitudeController::computeTargetThrust_()
{
  if (!limits_.configured || limits_.max_thrust <= 0.0f) return false;

  const uint16_t thruster_count = getThrusterCount();
  if (thruster_count == 0) return false;

  float base_thrust_decreasing_rate = 0.0f;
  float yaw_decreasing_rate = 0.0f;
  const float thrust_limit = limits_.max_thrust;

  float max_thrust = 0.0f;
  uint16_t max_thrust_index = 0;
  for (uint16_t i = 0; i < thruster_count; ++i) {
    const float thrust = groupThrust_(i);
    if (max_thrust < thrust) {
      max_thrust = thrust;
      max_thrust_index = i;
    }
  }

  if (start_control_flag_) {
    float residual_term = thrust_limit - max_thrust / rotor_devider_;

    if (residual_term < 0.0f && base_thrust_term_[max_thrust_index] > 0.0f) {
      base_thrust_decreasing_rate =
        residual_term / (base_thrust_term_[max_thrust_index] / rotor_devider_);
      yaw_decreasing_rate = -1.0f;
    } else if (max_yaw_term_index_ != -1 && fabsf(base_thrust_term_[0]) > 0.0f) {
      max_thrust = 0.0f;
      float min_thrust = 10000.0f;
      uint16_t min_thrust_index = 0;

      for (uint16_t i = 0; i < thruster_count; ++i) {
        const float thrust = groupThrust_(i);
        if (max_thrust < thrust) {
          max_thrust = thrust;
          max_thrust_index = i;
        }
        if (min_thrust > thrust) {
          min_thrust = thrust;
          min_thrust_index = i;
        }
      }

      const float residual_term_max = thrust_limit - max_thrust / rotor_devider_;
      const float residual_term_min = min_thrust / rotor_devider_ - limits_.min_thrust;
      uint16_t thrust_index = 0;
      if (residual_term_min < residual_term_max) {
        residual_term = residual_term_min;
        thrust_index = min_thrust_index;
      } else {
        residual_term = residual_term_max;
        thrust_index = max_thrust_index;
      }

      if (residual_term < 0.0f && fabsf(yaw_term_[thrust_index]) > 0.0f) {
        yaw_decreasing_rate = residual_term / (fabsf(yaw_term_[thrust_index]) / rotor_devider_);
      }

      if (yaw_decreasing_rate < -1.0f) yaw_decreasing_rate = -1.0f;
      if (yaw_decreasing_rate > 0.0f) yaw_decreasing_rate = 0.0f;
    } else {
      yaw_decreasing_rate = -1.0f;
    }
  }

  for (size_t i = 0; i < motor_number_; ++i) {
    target_thrust_[i] =
      roll_pitch_term_[i] +
      (1.0f + base_thrust_decreasing_rate) * base_thrust_term_[i] +
      (1.0f + yaw_decreasing_rate) * yaw_term_[i];
  }

  for (uint16_t i = 0; i < thruster_count; ++i) {
    if (!start_control_flag_) {
      target_thrust_[i] = 0.0f;
      continue;
    }

    if (gimbal_dof_ == 2) {
      ap::Vector3f f_i;
      f_i.x = target_thrust_[i * 3];
      f_i.y = target_thrust_[i * 3 + 1];
      f_i.z = target_thrust_[i * 3 + 2];

      const float gimbal_candidate_roll = atan2f(-f_i.y, f_i.z);
      const float gimbal_candidate_pitch =
        atan2f(f_i.x, -f_i.y * sinf(gimbal_candidate_roll) + f_i.z * cosf(gimbal_candidate_roll));
      target_thrust_[i] = ap::pythagorous3(f_i.x, f_i.y, f_i.z);

      if (std::isfinite(gimbal_candidate_roll) && std::isfinite(gimbal_candidate_pitch)) {
        target_gimbal_angles_[2 * i] = (target_gimbal_angles_[2 * i] + gimbal_candidate_roll) * 0.5f;
        target_gimbal_angles_[2 * i + 1] = (target_gimbal_angles_[2 * i + 1] + gimbal_candidate_pitch) * 0.5f;
      }
    } else if (gimbal_dof_ == 1) {
      ap::Vector3f f_i;
      f_i.x = target_thrust_[i * 2];
      f_i.z = target_thrust_[i * 2 + 1];

      const float gimbal_candidate = atan2f(-f_i.x, f_i.z);
      target_thrust_[i] = ap::pythagorous2(f_i.x, f_i.z);

      if (std::isfinite(gimbal_candidate)) {
        target_gimbal_angles_[i] = (target_gimbal_angles_[i] + gimbal_candidate) * 0.5f;
      }
    }
  }

  for (size_t i = thruster_count; i < MAX_FLIGHT_CONTROL_MOTOR_NUM; ++i) {
    target_thrust_[i] = 0.0f;
  }

  return true;
}

float AttitudeController::groupThrust_(uint16_t group_index) const
{
  if (gimbal_dof_ == 2) {
    const size_t base = rotor_coef_ * group_index;
    return ap::pythagorous3(
      base_thrust_term_[base] + roll_pitch_term_[base],
      base_thrust_term_[base + 1] + roll_pitch_term_[base + 1],
      base_thrust_term_[base + 2] + roll_pitch_term_[base + 2]);
  }

  if (gimbal_dof_ == 1) {
    const size_t base = rotor_coef_ * group_index;
    return ap::pythagorous2(
      base_thrust_term_[base] + roll_pitch_term_[base],
      base_thrust_term_[base + 1] + roll_pitch_term_[base + 1]);
  }

  return base_thrust_term_[group_index] + roll_pitch_term_[group_index];
}

void AttitudeController::clearOutputTargets_()
{
  for (size_t i = 0; i < MAX_FLIGHT_CONTROL_MOTOR_NUM; ++i) {
    target_thrust_[i] = 0.0f;
    target_gimbal_angles_[i] = 0.0f;
  }
}

float AttitudeController::deltaTime_()
{
  const uint32_t now = nowMillis_();
  if (update_last_time_ == 0) {
    update_last_time_ = now;
    return 0.0f;
  }

  const float dt = static_cast<float>(now - update_last_time_) * 0.001f;
  update_last_time_ = now;
  return dt;
}

uint32_t AttitudeController::nowMillis_() const
{
#ifdef SIMULATION
  using Clock = std::chrono::steady_clock;
  const auto now = Clock::now().time_since_epoch();
  return static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
#else
  return HAL_GetTick();
#endif
}
