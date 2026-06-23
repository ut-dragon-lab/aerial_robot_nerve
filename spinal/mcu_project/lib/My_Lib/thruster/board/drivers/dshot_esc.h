#pragma once

#include <cstddef>
#include <cstdint>

#include <config.h>

#include "thruster/board/drivers/esc_base_driver.h"
#include "thruster/board/drivers/esc_telem.h"

#ifndef TIMER_CLOCK
#define TIMER_CLOCK 100000000
#endif

#define MHZ_TO_HZ(x) ((x) * 1000000)

#define DSHOT600_HZ MHZ_TO_HZ(12)
#define DSHOT300_HZ MHZ_TO_HZ(6)
#define DSHOT150_HZ MHZ_TO_HZ(3)

#define MOTOR_BIT_0 7
#define MOTOR_BIT_1 14
#define MOTOR_BITLENGTH 20

#define DSHOT_FRAME_SIZE 16
#define DSHOT_DMA_BUFFER_SIZE 18

#define DSHOT_MIN_THROTTLE 48
#define DSHOT_MAX_THROTTLE 2047
#define DSHOT_RANGE (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

#define DSHOT_CMD_SPIN_DIRECTION_1 7
#define DSHOT_CMD_SPIN_DIRECTION_2 8

enum dshot_type_e
{
  DSHOT150,
  DSHOT300,
  DSHOT600
};

class DShotEsc final : public EscBaseDriver
{
public:
  DShotEsc() = default;
  ~DShotEsc() override = default;

  void init(dshot_type_e dshot_type,
            TIM_HandleTypeDef* htim_motor_1, uint32_t channel_motor_1,
            TIM_HandleTypeDef* htim_motor_2, uint32_t channel_motor_2,
            TIM_HandleTypeDef* htim_motor_3, uint32_t channel_motor_3,
            TIM_HandleTypeDef* htim_motor_4, uint32_t channel_motor_4);
  void initTelemetry(UART_HandleTypeDef* huart, int num_motor_mag_pole = 14);

  void writeDuty(const float* target_duty, size_t motor_count) override;
  void writeThrottle(const uint16_t* motor_value_array, size_t motor_count, bool request_telemetry);

  bool telemetryEnabled() const { return telemetry_enabled_; }
  bool updateTelemetry(EscTelemetrySnapshot* snapshot = nullptr);
  bool consumeTelemetrySnapshot(EscTelemetrySnapshot& snapshot);

private:
  TIM_HandleTypeDef* htim_motor_1_{nullptr};
  uint32_t channel_motor_1_{0};
  TIM_HandleTypeDef* htim_motor_2_{nullptr};
  uint32_t channel_motor_2_{0};
  TIM_HandleTypeDef* htim_motor_3_{nullptr};
  uint32_t channel_motor_3_{0};
  TIM_HandleTypeDef* htim_motor_4_{nullptr};
  uint32_t channel_motor_4_{0};

  bool telemetry_enabled_{false};
  int id_telem_{0};
  int id_telem_prev_{-1};
  bool telemetry_request_pending_{false};
  bool telemetry_snapshot_ready_{false};
  EscTelemetrySnapshot telemetry_snapshot_{};
  EscTelemetryReader esc_reader_;

  uint32_t dshot_choose_type(dshot_type_e dshot_type);
  void dshot_set_timer(dshot_type_e dshot_type);
  static void dshot_dma_tc_callback(DMA_HandleTypeDef* hdma);
  void dshot_put_tc_callback_function();
  void dshot_start_pwm();

  uint16_t dshot_prepare_packet(uint16_t value, bool dshot_telemetry);
  void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value, bool is_telemetry);
  void dshot_prepare_dmabuffer_all(const uint16_t* motor_value, const bool* is_telemetry);
  void dshot_dma_start();
  void dshot_enable_dma_request();
};
