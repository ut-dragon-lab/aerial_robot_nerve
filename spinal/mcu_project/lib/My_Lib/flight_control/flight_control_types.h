#pragma once

#include <cstddef>
#include <cstdint>

#include "thruster/thruster_types.h"

#ifndef MAX_FLIGHT_CONTROL_MOTOR_NUM
#define MAX_FLIGHT_CONTROL_MOTOR_NUM MAX_THRUSTER_NUM
#endif

namespace FlightControlConstants
{
constexpr float FORCE_LANDING_INTEGRAL = 0.0025f; // 500Hz * 0.0025 = 1.25 N / sec
constexpr uint32_t FLIGHT_COMMAND_TIMEOUT_MS = 500;
constexpr float MAX_TILT_ANGLE = 1.0f;
constexpr uint32_t CONTROL_TERM_PUB_INTERVAL_MS = 100;
constexpr uint32_t CONTROL_FEEDBACK_STATE_PUB_INTERVAL_MS = 25;
}

namespace FlightControlAxis
{
enum
{
  X = 0,
  Y = 1,
  Z = 2
};
}

namespace FlightControlCommand
{
enum
{
  ARM_ON_CMD = 0,
  ARM_OFF_CMD = 1,
  FORCE_LANDING_CMD = 2,
  INTEGRATION_CONTROL_ON_CMD = 160,
  INTEGRATION_CONTROL_OFF_CMD = 161
};
}

namespace FlightControlUavModel
{
enum
{
  DRONE = 0,
  HYDRUS = 16,
  HYDRUS_XI = 17,
  DRAGON = 32
};
}

struct FlightControlRpyTerm
{
  int16_t roll_p{0};
  int16_t roll_i{0};
  int16_t roll_d{0};
  int16_t pitch_p{0};
  int16_t pitch_i{0};
  int16_t pitch_d{0};
  int16_t yaw_d{0};
};

struct FlightControlRpyTerms
{
  FlightControlRpyTerm motors[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  size_t motors_count{0};
};

struct FlightControlFourAxisCommand
{
  float angles[3]{};
  float base_thrust[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  size_t base_thrust_count{0};
};

struct FlightControlPMatrixPseudoInverseUnit
{
  int16_t r{0};
  int16_t p{0};
  int16_t y{0};
};

struct FlightControlPMatrixPseudoInverseWithInertia
{
  FlightControlPMatrixPseudoInverseUnit pseudo_inverse[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  size_t pseudo_inverse_count{0};
  int16_t inertia[6]{};
};

struct FlightControlVector3Int16
{
  int16_t x{0};
  int16_t y{0};
  int16_t z{0};
};

struct FlightControlTorqueAllocationMatrixInv
{
  FlightControlVector3Int16 rows[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  size_t rows_count{0};
};

struct FlightControlDesireCoord
{
  float roll{0.0f};
  float pitch{0.0f};
  float yaw{0.0f};
};
