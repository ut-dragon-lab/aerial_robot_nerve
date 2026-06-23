#pragma once

#include <cstdint>

#include <config.h>

#include "thruster/thruster_types.h"

#ifndef ESC_BUFFER_SIZE
#define ESC_BUFFER_SIZE 512
#endif

class EscTelemetryReader
{
public:
  EscTelemetryReader() = default;
  ~EscTelemetryReader() = default;

  void init(UART_HandleTypeDef* huart);
  void update(EscTelemetryData& esc);
  bool available();
  int readOneByte();

  void setMotorMagPole(int num_motor_mag_pole) { num_motor_mag_pole_ = num_motor_mag_pole; }

private:
  UART_HandleTypeDef* huart_{nullptr};

  uint8_t step_{0};
  uint8_t msg_id_{0};
  uint16_t payload_length_{0};
  uint16_t payload_counter_{0};
  uint8_t ck_a_{0};
  uint8_t ck_b_{0};
  uint8_t class_{0};

  int num_motor_mag_pole_{14};
};

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
uint8_t get_crc8(uint8_t* buf, uint8_t buf_len);
