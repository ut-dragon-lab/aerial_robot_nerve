#pragma once

#include <cstddef>
#include <cstdint>

#include <config.h>

#include "thruster/board/drivers/esc_base_driver.h"

class PwmEsc final : public EscBaseDriver
{
public:
  PwmEsc() = default;
  ~PwmEsc() override = default;

  void init(TIM_HandleTypeDef* htim_primary,
            TIM_HandleTypeDef* htim_secondary,
            bool use_primary_pwm = true);

  void writeDuty(const float* target_duty, size_t motor_count) override;

private:
  TIM_HandleTypeDef* htim_primary_{nullptr};
  TIM_HandleTypeDef* htim_secondary_{nullptr};
  bool use_primary_pwm_{true};

  void configurePrimaryPwm_();
  void startSecondaryPwm_();
  void writeTimerDuty_(TIM_HandleTypeDef* htim, size_t local_index, float duty);
};
