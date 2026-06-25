#pragma once

#include <cstddef>
#include <cstdint>

#include "flight_control/flight_control_types.h"
#include "math/AP_Math.h"
#include "state_estimate/state_estimate.h"

class AttitudeController
{
public:
  AttitudeController();
  ~AttitudeController() = default;

  void init(StateEstimate* estimator);
  void reset();
  bool update();

  bool applyFourAxisCommand(const FlightControlFourAxisCommand& cmd);
  bool applyRpyGains(const FlightControlRpyTerms& gains);
  bool applyPMatrixInertia(const FlightControlPMatrixPseudoInverseWithInertia& msg);
  bool applyTorqueAllocationMatrixInv(const FlightControlTorqueAllocationMatrixInv& msg);
  void applyOffsetRotation(const FlightControlDesireCoord& msg);

  bool activated() const;

  void setStartControlFlag(bool start_control_flag);
  void setUavModel(int8_t uav_model);
  void setMotorNumber(uint16_t motor_number);
  void setGimbalDof(uint8_t gimbal_dof) { gimbal_dof_ = gimbal_dof; }
  void setRotorCoef(uint8_t rotor_coef) { rotor_coef_ = rotor_coef == 0 ? 1 : rotor_coef; }
  void setIntegrateFlag(bool integrate_flag) { integrate_flag_ = integrate_flag; }
  void setForceLandingFlag(bool force_landing_flag) { force_landing_flag_ = force_landing_flag; }
  void setAttitudeControlFlag(bool att_control_flag) { att_control_flag_ = att_control_flag; }
  void setThrusterLimits(const ThrusterControlLimits& limits) { limits_ = limits; }

  uint16_t getMotorNumber() const { return motor_number_; }
  uint16_t getThrusterCount() const;
  uint8_t getGimbalDof() const { return gimbal_dof_; }
  uint8_t getRotorCoef() const { return rotor_coef_; }
  uint8_t getRotorDivider() const { return rotor_devider_; }
  int8_t getUavModel() const { return uav_model_; }
  bool getStartControlFlag() const { return start_control_flag_; }
  bool getIntegrateFlag() const { return integrate_flag_; }
  bool getForceLandingFlag() const { return force_landing_flag_; }
  bool getAttitudeControlFlag() const { return att_control_flag_; }

  const float* getTargetThrust() const { return target_thrust_; }
  const float* getTargetGimbalAngles() const { return target_gimbal_angles_; }
  size_t getTargetGimbalAngleCount() const;
  const float* getGyroMomentCompensation() const { return gyro_moment_compensation_; }

  const FlightControlRpyTerms& getControlTerms() const { return control_term_; }
  const FlightControlRpyTerm& getControlFeedbackState() const { return control_feedback_state_; }

  bool controlTermPublishReady(bool update_last_time = false);
  bool controlFeedbackStatePublishReady(bool update_last_time = false);

private:
  StateEstimate* estimator_{nullptr};

  int8_t uav_model_{-1};
  uint16_t motor_number_{0};
  uint8_t gimbal_dof_{0};
  uint8_t rotor_coef_{1};
  uint8_t rotor_devider_{1};
  bool start_control_flag_{false};
  bool integrate_flag_{false};
  bool force_landing_flag_{false};
  bool att_control_flag_{true};

  float target_angle_[3]{};
  float error_angle_i_[3]{};

  float torque_p_gain_[3]{};
  float torque_i_gain_[3]{};
  float torque_d_gain_[3]{};
  float thrust_p_gain_[MAX_FLIGHT_CONTROL_MOTOR_NUM][3]{};
  float thrust_i_gain_[MAX_FLIGHT_CONTROL_MOTOR_NUM][3]{};
  float thrust_d_gain_[MAX_FLIGHT_CONTROL_MOTOR_NUM][3]{};
  float torque_allocation_matrix_inv_[MAX_FLIGHT_CONTROL_MOTOR_NUM][3]{};
  float base_thrust_term_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float roll_pitch_term_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float yaw_term_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float extra_yaw_pi_term_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float target_thrust_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float target_gimbal_angles_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  float gyro_moment_compensation_[MAX_FLIGHT_CONTROL_MOTOR_NUM]{};
  int max_yaw_term_index_{-1};

  ap::Matrix3f offset_rot_;
  float p_matrix_pseudo_inverse_[MAX_FLIGHT_CONTROL_MOTOR_NUM][4]{};
  ap::Matrix3f inertia_;

  bool failsafe_{false};
  uint32_t flight_command_last_stamp_{0};
  uint32_t update_last_time_{0};
  uint32_t control_term_pub_last_time_{0};
  uint32_t control_feedback_state_pub_last_time_{0};

  ThrusterControlLimits limits_{};
  FlightControlRpyTerms control_term_{};
  FlightControlRpyTerm control_feedback_state_{};

  void thrustGainMapping_();
  void maxYawGainIndex_();
  bool computeTargetThrust_();
  float groupThrust_(uint16_t group_index) const;
  void clearOutputTargets_();

  float deltaTime_();
  uint32_t nowMillis_() const;

  static float limit_(float input, float limit)
  {
    if (input > limit) return limit;
    if (input < -limit) return -limit;
    return input;
  }
};
