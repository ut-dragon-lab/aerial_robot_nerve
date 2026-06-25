#pragma once

#include <cstddef>
#include <cstdint>

#include <config.h>

#include "battery_status/battery_status.h"
#include "thruster/thruster_types.h"
#include "thruster/board/drivers/pwm_esc.h"

#if DSHOT
#include "thruster/board/drivers/dshot_esc.h"
#endif

class ThrusterManager
{
public:
  ThrusterManager();
  ~ThrusterManager() = default;

  void init(TIM_HandleTypeDef* htim_primary, TIM_HandleTypeDef* htim_secondary);

#if DSHOT
  void initDShotTelemetry(UART_HandleTypeDef* huart, int num_motor_mag_pole = 14);
#endif

  void setBatteryStatus(BatteryStatus* battery) { battery_ = battery; }
  void setMotorNumber(uint16_t motor_number);
  void setRotorDivider(uint8_t rotor_devider) { rotor_devider_ = rotor_devider == 0 ? 1 : rotor_devider; }

  bool applyPwmInfo(const ThrusterPwmInfo& info);
  void applyPwmTest(const ThrusterPwmTestCommand& cmd);

  bool outputThrust(const float* target_thrust, size_t motor_count, bool start_control);
  void writeDuty(const float* target_duty, size_t motor_count);
  void sendCommand();

  bool configured() const { return motor_info_count_ > 0 && max_duty_ > min_duty_; }

  float getTargetPwm(uint8_t index) const;
  float getTargetThrust(uint8_t index) const;
  uint16_t getMotorNumber() const { return motor_number_; }
  float getForceLandingThrust() const { return force_landing_thrust_; }
  uint8_t getControlMode() const;
  ThrusterControlLimits getControlLimits();

  bool motorPwmPublishReady(bool update_last_time = false);
  uint16_t getMotorPwmRosValue(uint8_t index) const;

  bool updateTelemetry();
  bool escTelemetryEnabled() const;
  bool consumeEscTelemetrySnapshot(EscTelemetrySnapshot& snapshot);

private:
  PwmEsc pwm_esc_;

#if DSHOT
  DShotEsc dshot_esc_;
#endif

  BatteryStatus* battery_{nullptr};

  uint16_t motor_number_{0};
  uint8_t rotor_devider_{1};

  float target_thrust_[MAX_THRUSTER_NUM]{};
  float target_pwm_[MAX_THRUSTER_NUM]{};
  float pwm_test_value_[MAX_THRUSTER_NUM]{};

  float min_duty_{ThrusterConstants::IDLE_DUTY};
  float max_duty_{ThrusterConstants::IDLE_DUTY};
  float min_thrust_{0.0f};
  float force_landing_thrust_{0.0f};
  int8_t pwm_conversion_mode_{-1};
  ThrusterMotorInfo motor_info_[MAX_THRUSTER_MOTOR_INFO_NUM]{};
  size_t motor_info_count_{0};
  uint8_t motor_ref_index_{0};
  float v_factor_{1.0f};
  uint32_t voltage_update_last_time_{0};
  uint32_t pwm_pub_last_time_{0};
  bool pwm_test_flag_{false};
  bool start_control_flag_{false};

  float convertThrustToDuty_(float target_thrust) const;
  void updateVoltageFactor_();
  float currentVoltage_() const;
  void clearTargets_();
};
