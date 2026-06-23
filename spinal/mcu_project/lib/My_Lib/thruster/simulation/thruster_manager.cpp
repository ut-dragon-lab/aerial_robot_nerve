#ifdef SIMULATION

#include "thruster/simulation/thruster_manager.h"

#include <chrono>
#include <cmath>

#include "math/AP_Math.h"

ThrusterManager::ThrusterManager()
{
  clearTargets_();
}

void ThrusterManager::setMotorNumber(uint16_t motor_number)
{
  motor_number_ = motor_number > MAX_THRUSTER_NUM ? MAX_THRUSTER_NUM : motor_number;
}

bool ThrusterManager::applyPwmInfo(const ThrusterPwmInfo& info)
{
  force_landing_thrust_ = info.force_landing_thrust;

  min_duty_ = info.min_pwm;
  max_duty_ = info.max_pwm;
  pwm_conversion_mode_ = static_cast<int8_t>(info.pwm_conversion_mode);
  min_thrust_ = info.min_thrust;

  motor_info_count_ = info.motor_info_count;
  if (motor_info_count_ > MAX_THRUSTER_MOTOR_INFO_NUM) {
    motor_info_count_ = MAX_THRUSTER_MOTOR_INFO_NUM;
  }

  for (size_t i = 0; i < motor_info_count_; ++i) {
    motor_info_[i] = info.motor_info[i];
  }

  if (sim_voltage_ == 0.0f && motor_info_count_ > 0) {
    sim_voltage_ = motor_info_[0].voltage;
  }

  motor_ref_index_ = 0;
  v_factor_ = 1.0f;
  voltage_update_last_time_ = 0;

  return configured();
}

void ThrusterManager::applyPwmTest(const ThrusterPwmTestCommand& cmd)
{
  if (cmd.motor_index_count && cmd.motor_index_count != cmd.pwms_count) return;

  if (cmd.pwms_count && !pwm_test_flag_) {
    pwm_test_flag_ = true;
  } else if (!cmd.pwms_count && pwm_test_flag_) {
    pwm_test_flag_ = false;
    clearTargets_();
    return;
  }

  if (cmd.motor_index_count) {
    uint16_t motor_number = motor_number_;
    for (size_t i = 0; i < cmd.motor_index_count; ++i) {
      const uint8_t motor_index = cmd.motor_index[i];
      if (motor_index >= MAX_THRUSTER_NUM) continue;
      if (motor_number <= motor_index) {
        motor_number = motor_index + 1;
      }

      if (cmd.pwms[i] >= ThrusterConstants::IDLE_DUTY && cmd.pwms[i] <= ThrusterConstants::MAX_PWM) {
        pwm_test_value_[motor_index] = cmd.pwms[i];
      } else {
        pwm_test_value_[motor_index] = ThrusterConstants::IDLE_DUTY;
      }
    }
    setMotorNumber(motor_number);
  } else if (cmd.pwms_count) {
    const float pwm = cmd.pwms[0];
    if (motor_number_ == 0) {
      setMotorNumber(MAX_THRUSTER_NUM);
    }
    for (size_t i = 0; i < MAX_THRUSTER_NUM; ++i) {
      if (pwm >= ThrusterConstants::IDLE_DUTY && pwm <= ThrusterConstants::MAX_PWM) {
        pwm_test_value_[i] = pwm;
      } else {
        pwm_test_value_[i] = ThrusterConstants::IDLE_DUTY;
      }
    }
  }
}

bool ThrusterManager::outputThrust(const float* target_thrust, size_t motor_count, bool start_control)
{
  if (target_thrust == nullptr) return false;

  const size_t n = (motor_count > MAX_THRUSTER_NUM) ? MAX_THRUSTER_NUM : motor_count;
  setMotorNumber(static_cast<uint16_t>(n));

  if (pwm_test_flag_) {
    for (size_t i = 0; i < MAX_THRUSTER_NUM; ++i) {
      target_pwm_[i] = pwm_test_value_[i];
      target_thrust_[i] = 0.0f;
    }
    return true;
  }

  if (!configured()) return false;

  updateVoltageFactor_();

  for (size_t i = 0; i < n; ++i) {
    target_thrust_[i] = start_control ? target_thrust[i] : 0.0f;
    target_pwm_[i] = start_control ? convertThrustToDuty_(target_thrust_[i]) : ThrusterConstants::IDLE_DUTY;

    if (target_pwm_[i] < min_duty_) {
      target_pwm_[i] = min_duty_;
    } else if (target_pwm_[i] > max_duty_) {
      target_pwm_[i] = max_duty_;
    }
  }

  for (size_t i = n; i < MAX_THRUSTER_NUM; ++i) {
    target_thrust_[i] = 0.0f;
    target_pwm_[i] = ThrusterConstants::IDLE_DUTY;
  }

  return true;
}

void ThrusterManager::writeDuty(const float* target_duty, size_t motor_count)
{
  if (target_duty == nullptr) return;

  const size_t n = (motor_count > MAX_THRUSTER_NUM) ? MAX_THRUSTER_NUM : motor_count;
  setMotorNumber(static_cast<uint16_t>(n));

  for (size_t i = 0; i < n; ++i) {
    target_pwm_[i] = target_duty[i];
  }
}

void ThrusterManager::sendCommand()
{
  if (!pwm_test_flag_) return;

  for (size_t i = 0; i < MAX_THRUSTER_NUM; ++i) {
    target_pwm_[i] = pwm_test_value_[i];
    target_thrust_[i] = 0.0f;
  }
}

float ThrusterManager::getTargetPwm(uint8_t index) const
{
  if (index >= MAX_THRUSTER_NUM) return ThrusterConstants::IDLE_DUTY;
  return target_pwm_[index];
}

float ThrusterManager::getTargetThrust(uint8_t index) const
{
  if (index >= MAX_THRUSTER_NUM) return 0.0f;
  return target_thrust_[index];
}

bool ThrusterManager::motorPwmPublishReady(bool update_last_time)
{
  const uint32_t now = nowMillis_();
  if (now - pwm_pub_last_time_ <= ThrusterConstants::PWM_PUB_INTERVAL_MS) return false;

  if (update_last_time) {
    pwm_pub_last_time_ = now;
  }
  return true;
}

uint16_t ThrusterManager::getMotorPwmRosValue(uint8_t index) const
{
  if (index >= MAX_THRUSTER_NUM) return 0;
  return static_cast<uint16_t>(target_pwm_[index] * 2000.0f);
}

float ThrusterManager::convertThrustToDuty_(float target_thrust) const
{
  float scaled_thrust = v_factor_ * target_thrust / rotor_devider_;
  float target_pwm = 0.0f;
  if (scaled_thrust < 0.0f) scaled_thrust = 0.0f;

  switch (pwm_conversion_mode_) {
    case ThrusterPwmConversionMode::SQRT_MODE:
    {
      const float sqrt_tmp =
        motor_info_[motor_ref_index_].polynominal[1] * motor_info_[motor_ref_index_].polynominal[1] -
        4.0f * 10.0f * motor_info_[motor_ref_index_].polynominal[2] *
        (motor_info_[motor_ref_index_].polynominal[0] - scaled_thrust);
      if (sqrt_tmp > 0.0f) {
        target_pwm =
          (-motor_info_[motor_ref_index_].polynominal[1] + sqrt_tmp * ap::inv_sqrt(sqrt_tmp)) /
          (2.0f * motor_info_[motor_ref_index_].polynominal[2]);
      }
      break;
    }
    case ThrusterPwmConversionMode::POLYNOMINAL_MODE:
    {
      const float tenth_scaled_thrust = scaled_thrust * 0.1f;
      constexpr int max_dimenstional = 4;
      target_pwm = motor_info_[motor_ref_index_].polynominal[max_dimenstional];
      for (int j = max_dimenstional - 1; j >= 0; j--) {
        target_pwm = target_pwm * tenth_scaled_thrust + motor_info_[motor_ref_index_].polynominal[j];
      }
      break;
    }
    default:
      break;
  }

  return target_pwm / 100.0f;
}

void ThrusterManager::updateVoltageFactor_()
{
  if (motor_info_count_ == 0) return;
  if (nowMillis_() - voltage_update_last_time_ <= 500) return;

  const float voltage = currentVoltage_();
  if (voltage <= 0.0f) return;

  float min_voltage_diff = 1e6f;
  for (size_t i = 0; i < motor_info_count_; ++i) {
    const float voltage_diff = fabsf(voltage - motor_info_[i].voltage);
    if (min_voltage_diff > voltage_diff) {
      motor_ref_index_ = static_cast<uint8_t>(i);
      min_voltage_diff = voltage_diff;
    }
  }

  switch (pwm_conversion_mode_) {
    case ThrusterPwmConversionMode::SQRT_MODE:
      v_factor_ = (motor_info_[motor_ref_index_].voltage / voltage) *
                  (motor_info_[motor_ref_index_].voltage / voltage);
      break;
    case ThrusterPwmConversionMode::POLYNOMINAL_MODE:
      v_factor_ = motor_info_[motor_ref_index_].voltage / voltage *
                  ap::inv_sqrt(voltage / motor_info_[motor_ref_index_].voltage);
      break;
    default:
      break;
  }

  if (min_thrust_ > 0.0f) {
    min_duty_ = convertThrustToDuty_(min_thrust_);
  }

  voltage_update_last_time_ = nowMillis_();
}

float ThrusterManager::currentVoltage_() const
{
  if (sim_voltage_ > 0.0f) {
    return sim_voltage_;
  }

  if (motor_info_count_ > 0) {
    return motor_info_[0].voltage;
  }

  return 0.0f;
}

void ThrusterManager::clearTargets_()
{
  for (size_t i = 0; i < MAX_THRUSTER_NUM; ++i) {
    target_thrust_[i] = 0.0f;
    target_pwm_[i] = ThrusterConstants::IDLE_DUTY;
    pwm_test_value_[i] = ThrusterConstants::IDLE_DUTY;
  }
}

uint32_t ThrusterManager::nowMillis_() const
{
  using Clock = std::chrono::steady_clock;
  const auto now = Clock::now().time_since_epoch();
  return static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

#endif // SIMULATION
