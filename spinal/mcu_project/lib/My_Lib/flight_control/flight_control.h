#pragma once

#include <cstddef>
#include <cstdint>

#include "flight_control/attitude/attitude_control.h"

class StateEstimate;
class ThrusterManager;

#ifndef SIMULATION
class DirectServo;
#endif

class FlightControl
{
public:
  FlightControl() = default;
  ~FlightControl() = default;

  void init(
    StateEstimate* estimator,
    ThrusterManager* thruster,
#ifndef SIMULATION
    DirectServo* servo = nullptr
#else
    void* servo = nullptr
#endif
  );

  void update();

  bool applyFlightConfig(uint8_t command);
  void applyUavInfo(uint8_t motor_num, int8_t uav_model);
  void applyGimbalDof(uint8_t gimbal_dof);
  bool applyFourAxisCommand(const FlightControlFourAxisCommand& cmd);
  bool applyRpyGains(const FlightControlRpyTerms& gains);
  bool applyPMatrixInertia(const FlightControlPMatrixPseudoInverseWithInertia& msg);
  bool applyTorqueAllocationMatrixInv(const FlightControlTorqueAllocationMatrixInv& msg);
  void applyOffsetRotation(const FlightControlDesireCoord& msg);
  void setAttitudeControlFlag(bool flag);

  bool consumeConfigAck(uint8_t& ack);

  AttitudeController& getAttitudeController() { return att_controller_; }
  const AttitudeController& getAttitudeController() const { return att_controller_; }

  bool startControl() const { return start_control_flag_; }
  bool forceLanding() const { return force_landing_flag_; }
  uint8_t physicalMotorCount() const { return physical_motor_count_; }

private:
  StateEstimate* estimator_{nullptr};
  ThrusterManager* thruster_{nullptr};

#ifndef SIMULATION
  DirectServo* servo_{nullptr};
#endif

  AttitudeController att_controller_;

  bool start_control_flag_{false};
  bool force_landing_flag_{false};
  bool gimbal_set_flag_{false};
  bool config_ack_pending_{false};
  uint8_t config_ack_{0};
  uint8_t physical_motor_count_{0};

  void configureMotorCount_();
  void setConfigAck_(uint8_t ack);
  void applyGimbalOutput_();
};
