#pragma once

#include <cstddef>
#include <cstdint>

#ifndef MAX_THRUSTER_NUM
#define MAX_THRUSTER_NUM 10
#endif

#ifndef MAX_THRUSTER_MOTOR_INFO_NUM
#define MAX_THRUSTER_MOTOR_INFO_NUM 16
#endif

namespace ThrusterConstants
{
constexpr float IDLE_DUTY = 0.5f;
constexpr float MAX_PWM = 1.0f;
constexpr uint32_t PWM_PUB_INTERVAL_MS = 100;
}

namespace ThrusterPwmConversionMode
{
enum
{
  SQRT_MODE = 0,
  POLYNOMINAL_MODE = 1
};
}

namespace ThrusterControlMode
{
enum
{
  CONTROL_MODE_NONE = 0,
  CONTROL_MODE_TEST = 1,
  CONTROL_MODE_START = 2
};
}

struct ThrusterMotorInfo
{
  float voltage{0.0f};
  float max_thrust{0.0f};
  float polynominal[5]{};
};

struct ThrusterPwmInfo
{
  float min_pwm{ThrusterConstants::IDLE_DUTY};
  float max_pwm{ThrusterConstants::IDLE_DUTY};
  float min_thrust{0.0f};
  float force_landing_thrust{0.0f};
  uint8_t pwm_conversion_mode{ThrusterPwmConversionMode::SQRT_MODE};
  ThrusterMotorInfo motor_info[MAX_THRUSTER_MOTOR_INFO_NUM]{};
  size_t motor_info_count{0};
};

struct ThrusterPwmTestCommand
{
  uint8_t motor_index[MAX_THRUSTER_NUM]{};
  size_t motor_index_count{0};
  float pwms[MAX_THRUSTER_NUM]{};
  size_t pwms_count{0};
};

struct ThrusterControlLimits
{
  bool configured{false};
  float min_thrust{0.0f};
  float max_thrust{0.0f};
  float force_landing_thrust{0.0f};
};

struct EscTelemetryData
{
  int8_t temperature{0};
  uint16_t voltage{0};
  uint16_t current{0};
  uint16_t consumption{0};
  uint32_t rpm{0};
  uint8_t crc_error{0};
};

struct EscTelemetrySnapshot
{
  EscTelemetryData esc[4]{};
};
