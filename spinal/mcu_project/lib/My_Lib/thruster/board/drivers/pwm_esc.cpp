#include "thruster/board/drivers/pwm_esc.h"

void PwmEsc::init(TIM_HandleTypeDef* htim_primary,
                  TIM_HandleTypeDef* htim_secondary,
                  bool use_primary_pwm)
{
  htim_primary_ = htim_primary;
  htim_secondary_ = htim_secondary;
  use_primary_pwm_ = use_primary_pwm;

  if (use_primary_pwm_) {
    configurePrimaryPwm_();
  }

  startSecondaryPwm_();
}

void PwmEsc::writeDuty(const float* target_duty, size_t motor_count)
{
  if (target_duty == nullptr) return;

  const size_t n = (motor_count > 8) ? 8 : motor_count;

  for (size_t i = 0; i < n; ++i) {
    if (i < 4) {
      if (use_primary_pwm_) {
        writeTimerDuty_(htim_primary_, i, target_duty[i]);
      }
    } else {
      writeTimerDuty_(htim_secondary_, i - 4, target_duty[i]);
    }
  }
}

void PwmEsc::configurePrimaryPwm_()
{
  if (htim_primary_ == nullptr) return;

  HAL_TIM_PWM_Stop(htim_primary_, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop(htim_primary_);
  HAL_TIM_Base_DeInit(htim_primary_);

  htim_primary_->Init.Prescaler = 3;
  htim_primary_->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim_primary_->Init.Period = 50000;

  TIM_OC_InitTypeDef sConfigOC = {};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  while (HAL_TIM_Base_Init(htim_primary_) != HAL_OK) {}
  while (HAL_TIM_PWM_Init(htim_primary_) != HAL_OK) {}
  while (HAL_TIM_PWM_ConfigChannel(htim_primary_, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {}

  if (htim_primary_->hdma[TIM_DMA_ID_UPDATE] != nullptr) {
    HAL_DMA_DeInit(htim_primary_->hdma[TIM_DMA_ID_UPDATE]);
    htim_primary_->hdma[TIM_DMA_ID_UPDATE] = nullptr;
  }

  HAL_TIM_Base_Start(htim_primary_);

  HAL_TIM_PWM_Start(htim_primary_, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(htim_primary_, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(htim_primary_, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(htim_primary_, TIM_CHANNEL_4);
}

void PwmEsc::startSecondaryPwm_()
{
  if (htim_secondary_ == nullptr) return;

  HAL_TIM_PWM_Start(htim_secondary_, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(htim_secondary_, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(htim_secondary_, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(htim_secondary_, TIM_CHANNEL_4);
}

void PwmEsc::writeTimerDuty_(TIM_HandleTypeDef* htim, size_t local_index, float duty)
{
  if (htim == nullptr) return;

  const uint32_t compare = static_cast<uint32_t>(duty * htim->Init.Period);

  switch (local_index) {
    case 0:
      htim->Instance->CCR1 = compare;
      break;
    case 1:
      htim->Instance->CCR2 = compare;
      break;
    case 2:
      htim->Instance->CCR3 = compare;
      break;
    case 3:
      htim->Instance->CCR4 = compare;
      break;
    default:
      break;
  }
}
