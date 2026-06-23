#include "thruster/board/drivers/esc_telem.h"

#include <cstring>

namespace
{
#ifdef STM32H7
uint8_t esc_telem_rx_buf_[ESC_BUFFER_SIZE] __attribute__((section(".EscRxBufferSection")));
#else
uint8_t esc_telem_rx_buf_[ESC_BUFFER_SIZE];
#endif
uint32_t esc_telem_rd_ptr_ = 0;
}

void EscTelemetryReader::init(UART_HandleTypeDef* huart)
{
  huart_ = huart;

  if (huart_ == nullptr) return;

  __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);
  HAL_UART_Receive_DMA(huart_, esc_telem_rx_buf_, ESC_BUFFER_SIZE);

  std::memset(esc_telem_rx_buf_, 0, ESC_BUFFER_SIZE);
  esc_telem_rd_ptr_ = 0;
}

void EscTelemetryReader::update(EscTelemetryData& esc)
{
  if (!available()) return;

  uint8_t buffer[10]{};

  for (int i = 0; i < 10; i++) {
    int data = -1;
    constexpr int max_attempts = 8;
    for (int attempt = 0; attempt < max_attempts; attempt++) {
      data = readOneByte();
      if (data >= 0) {
        break;
      }
    }

    if (data < 0) {
      data = 255;
    }

    buffer[i] = static_cast<uint8_t>(data);
  }

  const uint8_t crc = get_crc8(buffer, 9);
  if (crc == buffer[9]) {
    esc.temperature = static_cast<int8_t>(buffer[0]);
    esc.voltage = static_cast<uint16_t>(buffer[1] << 8 | buffer[2]);
    esc.current = static_cast<uint16_t>(buffer[3] << 8 | buffer[4]);
    esc.consumption = static_cast<uint16_t>(buffer[5] << 8 | buffer[6]);
    const uint16_t erpm = static_cast<uint16_t>(buffer[7] << 8 | buffer[8]);
    esc.rpm = erpm * 100 / (num_motor_mag_pole_ / 2);
  }
  esc.crc_error = static_cast<uint8_t>(crc - buffer[9]);
}

bool EscTelemetryReader::available()
{
  if (huart_ == nullptr || huart_->hdmarx == nullptr) return false;

  const uint32_t dma_write_ptr =
    (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % ESC_BUFFER_SIZE;
  return esc_telem_rd_ptr_ != dma_write_ptr;
}

int EscTelemetryReader::readOneByte()
{
  if (huart_ == nullptr || huart_->hdmarx == nullptr) return -1;

  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE)) {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
    HAL_UART_Receive_DMA(huart_, esc_telem_rx_buf_, ESC_BUFFER_SIZE);
  }

  const uint32_t dma_write_ptr =
    (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % ESC_BUFFER_SIZE;
  int c = -1;
  if (esc_telem_rd_ptr_ != dma_write_ptr) {
    c = static_cast<int>(esc_telem_rx_buf_[esc_telem_rd_ptr_++]);
    esc_telem_rd_ptr_ %= ESC_BUFFER_SIZE;
  }
  return c;
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
  uint8_t crc_u = crc;
  crc_u ^= crc_seed;
  for (uint8_t i = 0; i < 8; i++) {
    crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
  }
  return crc_u;
}

uint8_t get_crc8(uint8_t* buf, uint8_t buf_len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < buf_len; i++) {
    crc = update_crc8(buf[i], crc);
  }
  return crc;
}
