#include "thruster/board/drivers/dshot_esc.h"

#include <cmath>

namespace
{
#ifdef STM32H7
uint32_t motor1_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection1")));
uint32_t motor2_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection2")));
uint32_t motor3_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection3")));
uint32_t motor4_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection4")));
#else
uint32_t motor1_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
uint32_t motor2_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
uint32_t motor3_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
uint32_t motor4_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
#endif
}

void DShotEsc::init(dshot_type_e dshot_type,
                    TIM_HandleTypeDef* htim_motor_1, uint32_t channel_motor_1,
                    TIM_HandleTypeDef* htim_motor_2, uint32_t channel_motor_2,
                    TIM_HandleTypeDef* htim_motor_3, uint32_t channel_motor_3,
                    TIM_HandleTypeDef* htim_motor_4, uint32_t channel_motor_4)
{
  htim_motor_1_ = htim_motor_1;
  channel_motor_1_ = channel_motor_1;
  htim_motor_2_ = htim_motor_2;
  channel_motor_2_ = channel_motor_2;
  htim_motor_3_ = htim_motor_3;
  channel_motor_3_ = channel_motor_3;
  htim_motor_4_ = htim_motor_4;
  channel_motor_4_ = channel_motor_4;

  dshot_set_timer(dshot_type);
  dshot_put_tc_callback_function();
  dshot_start_pwm();
}

void DShotEsc::initTelemetry(UART_HandleTypeDef* huart, int num_motor_mag_pole)
{
  esc_reader_.init(huart);
  esc_reader_.setMotorMagPole(num_motor_mag_pole);
  telemetry_enabled_ = true;
}

void DShotEsc::writeDuty(const float* target_duty, size_t motor_count)
{
  if (target_duty == nullptr) return;

  uint16_t motor_value[4]{};
  const size_t n = (motor_count > 4) ? 4 : motor_count;
  for (size_t i = 0; i < n; ++i) {
    uint16_t motor_v = 0;
    if (target_duty[i] > ThrusterConstants::IDLE_DUTY) {
      motor_v = static_cast<uint16_t>(
        (target_duty[i] - ThrusterConstants::IDLE_DUTY) / (ThrusterConstants::MAX_PWM - ThrusterConstants::IDLE_DUTY) *
        DSHOT_RANGE + DSHOT_MIN_THROTTLE);

      if (motor_v > DSHOT_MAX_THROTTLE) {
        motor_v = DSHOT_MAX_THROTTLE;
      } else if (motor_v < DSHOT_MIN_THROTTLE) {
        motor_v = DSHOT_MIN_THROTTLE;
      }
    }

    motor_value[i] = motor_v;
  }

  writeThrottle(motor_value, 4, telemetry_enabled_);
}

void DShotEsc::writeThrottle(const uint16_t* motor_value_array, size_t motor_count, bool request_telemetry)
{
  if (motor_value_array == nullptr) return;

  uint16_t motor_value[4]{};
  bool is_telemetry_array[4]{false, false, false, false};

  const size_t n = (motor_count > 4) ? 4 : motor_count;
  for (size_t i = 0; i < n; ++i) {
    motor_value[i] = motor_value_array[i];
  }

  if (request_telemetry && telemetry_enabled_ && telemetry_request_pending_) {
    id_telem_ = id_telem_ % 4;
    is_telemetry_array[id_telem_] = true;
    id_telem_prev_ = id_telem_;
    id_telem_++;
    telemetry_request_pending_ = false;
  }

  dshot_prepare_dmabuffer_all(motor_value, is_telemetry_array);
  dshot_dma_start();
  dshot_enable_dma_request();
}

bool DShotEsc::updateTelemetry(EscTelemetrySnapshot* snapshot)
{
  if (!telemetry_enabled_) return false;

  bool snapshot_updated = false;
  if (id_telem_prev_ != -1) {
    esc_reader_.update(telemetry_snapshot_.esc[id_telem_prev_]);
    if (id_telem_prev_ == 3) {
      telemetry_snapshot_ready_ = true;
      snapshot_updated = true;
      if (snapshot != nullptr) {
        *snapshot = telemetry_snapshot_;
      }
    }
    id_telem_prev_ = -1;
  }

  telemetry_request_pending_ = true;
  return snapshot_updated;
}

bool DShotEsc::consumeTelemetrySnapshot(EscTelemetrySnapshot& snapshot)
{
  if (!telemetry_snapshot_ready_) return false;

  snapshot = telemetry_snapshot_;
  telemetry_snapshot_ready_ = false;
  return true;
}

uint32_t DShotEsc::dshot_choose_type(dshot_type_e dshot_type)
{
  switch (dshot_type) {
    case DSHOT600:
      return DSHOT600_HZ;
    case DSHOT300:
      return DSHOT300_HZ;
    case DSHOT150:
      return DSHOT150_HZ;
    default:
      return DSHOT300_HZ;
  }
}

void DShotEsc::dshot_set_timer(dshot_type_e dshot_type)
{
  const uint32_t timer_clock = TIMER_CLOCK;
  const uint16_t dshot_prescaler =
    static_cast<uint16_t>(lrintf(static_cast<float>(timer_clock) / dshot_choose_type(dshot_type) + 0.01f) - 1);

  __HAL_TIM_SET_PRESCALER(htim_motor_1_, dshot_prescaler);
  __HAL_TIM_SET_AUTORELOAD(htim_motor_1_, MOTOR_BITLENGTH);

  __HAL_TIM_SET_PRESCALER(htim_motor_2_, dshot_prescaler);
  __HAL_TIM_SET_AUTORELOAD(htim_motor_2_, MOTOR_BITLENGTH);

  __HAL_TIM_SET_PRESCALER(htim_motor_3_, dshot_prescaler);
  __HAL_TIM_SET_AUTORELOAD(htim_motor_3_, MOTOR_BITLENGTH);

  __HAL_TIM_SET_PRESCALER(htim_motor_4_, dshot_prescaler);
  __HAL_TIM_SET_AUTORELOAD(htim_motor_4_, MOTOR_BITLENGTH);
}

void DShotEsc::dshot_dma_tc_callback(DMA_HandleTypeDef* hdma)
{
  TIM_HandleTypeDef* htim = static_cast<TIM_HandleTypeDef*>(hdma->Parent);

  if (hdma == htim->hdma[TIM_DMA_ID_CC1]) {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
  } else if (hdma == htim->hdma[TIM_DMA_ID_CC2]) {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
  } else if (hdma == htim->hdma[TIM_DMA_ID_CC3]) {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
  } else if (hdma == htim->hdma[TIM_DMA_ID_CC4]) {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
  }
}

void DShotEsc::dshot_put_tc_callback_function()
{
  htim_motor_1_->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshot_dma_tc_callback;
  htim_motor_2_->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = dshot_dma_tc_callback;
  htim_motor_3_->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = dshot_dma_tc_callback;
  htim_motor_4_->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = dshot_dma_tc_callback;
}

void DShotEsc::dshot_start_pwm()
{
  HAL_TIM_PWM_Start(htim_motor_1_, channel_motor_1_);
  HAL_TIM_PWM_Start(htim_motor_2_, channel_motor_2_);
  HAL_TIM_PWM_Start(htim_motor_3_, channel_motor_3_);
  HAL_TIM_PWM_Start(htim_motor_4_, channel_motor_4_);
}

uint16_t DShotEsc::dshot_prepare_packet(uint16_t value, bool dshot_telemetry)
{
  uint16_t packet = (value << 1) | (dshot_telemetry ? 1 : 0);

  unsigned csum = 0;
  unsigned csum_data = packet;

  for (int i = 0; i < 3; i++) {
    csum ^= csum_data;
    csum_data >>= 4;
  }

  csum &= 0xf;
  packet = (packet << 4) | csum;

  return packet;
}

void DShotEsc::dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value, bool is_telemetry)
{
  uint16_t packet = dshot_prepare_packet(value, is_telemetry);

  for (int i = 0; i < 16; i++) {
    motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    packet <<= 1;
  }

  motor_dmabuffer[16] = 0;
  motor_dmabuffer[17] = 0;
}

void DShotEsc::dshot_prepare_dmabuffer_all(const uint16_t* motor_value, const bool* is_telemetry)
{
  dshot_prepare_dmabuffer(motor1_dmabuffer_, motor_value[0], is_telemetry[0]);
  dshot_prepare_dmabuffer(motor2_dmabuffer_, motor_value[1], is_telemetry[1]);
  dshot_prepare_dmabuffer(motor3_dmabuffer_, motor_value[2], is_telemetry[2]);
  dshot_prepare_dmabuffer(motor4_dmabuffer_, motor_value[3], is_telemetry[3]);
}

void DShotEsc::dshot_dma_start()
{
  HAL_DMA_Start_IT(htim_motor_1_->hdma[TIM_DMA_ID_CC1], reinterpret_cast<uint32_t>(motor1_dmabuffer_),
                   reinterpret_cast<uint32_t>(&htim_motor_1_->Instance->CCR1), DSHOT_DMA_BUFFER_SIZE);
  HAL_DMA_Start_IT(htim_motor_2_->hdma[TIM_DMA_ID_CC2], reinterpret_cast<uint32_t>(motor2_dmabuffer_),
                   reinterpret_cast<uint32_t>(&htim_motor_2_->Instance->CCR2), DSHOT_DMA_BUFFER_SIZE);
  HAL_DMA_Start_IT(htim_motor_3_->hdma[TIM_DMA_ID_CC3], reinterpret_cast<uint32_t>(motor3_dmabuffer_),
                   reinterpret_cast<uint32_t>(&htim_motor_3_->Instance->CCR3), DSHOT_DMA_BUFFER_SIZE);
  HAL_DMA_Start_IT(htim_motor_4_->hdma[TIM_DMA_ID_CC4], reinterpret_cast<uint32_t>(motor4_dmabuffer_),
                   reinterpret_cast<uint32_t>(&htim_motor_4_->Instance->CCR4), DSHOT_DMA_BUFFER_SIZE);
}

void DShotEsc::dshot_enable_dma_request()
{
  __HAL_TIM_ENABLE_DMA(htim_motor_1_, TIM_DMA_CC1);
  __HAL_TIM_ENABLE_DMA(htim_motor_2_, TIM_DMA_CC2);
  __HAL_TIM_ENABLE_DMA(htim_motor_3_, TIM_DMA_CC3);
  __HAL_TIM_ENABLE_DMA(htim_motor_4_, TIM_DMA_CC4);
}
