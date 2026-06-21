#include "sensors/gps/gps_backend.h"

namespace
{
#ifdef STM32H7
  uint8_t rx_buf_[GPS_BUFFER_SIZE] __attribute__((section(".GpsRxBufferSection")));
#else
  uint8_t rx_buf_[GPS_BUFFER_SIZE];
#endif
  uint32_t rd_ptr_ = 0;
}

void GPS_Backend::init(UART_HandleTypeDef* huart)
{
  huart_ = huart;

  // use DMA for UART RX
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

  memset(rx_buf_, 0, RX_BUFFER_SIZE);
  HAL_UART_Receive_DMA(huart, rx_buf_, RX_BUFFER_SIZE);
}

bool GPS_Backend::available()
{
  uint32_t dma_write_ptr = (GPS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % GPS_BUFFER_SIZE;
  return (rd_ptr_ != dma_write_ptr);
}

int GPS_Backend::read()
{
  /* handle RX Overrun Error */
  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_NEF | UART_CLEAR_OREF);
    HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE); // restart
  }

  uint32_t dma_write_ptr = (GPS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % GPS_BUFFER_SIZE;
  int c = -1;

  if (rd_ptr_ != dma_write_ptr)
  {
    c = (int)rx_buf_[rd_ptr_++];
    rd_ptr_ %= GPS_BUFFER_SIZE;
  }
  return c;
}

void GPS_Backend::push_state_snapshot()
{
  // Overwrite-oldest ring buffer.
  //
  // Producer: GPS parsing context (processMessage)
  // Consumer: ROS context (publish_from_gps_state)
  //
  // We use monotonic counters so that "full" condition is (head - tail) >= SIZE.
  // On full, advance tail to drop the oldest entries.
  while (true)
  {
    const uint32_t head = q_head_.load(std::memory_order_relaxed);
    uint32_t tail = q_tail_.load(std::memory_order_acquire);

    const uint32_t used = head - tail;
    if (used < STATE_QUEUE_SIZE)
    {
      // There is space; proceed to write.
      const uint32_t idx = head % STATE_QUEUE_SIZE;
      state_queue_[idx] = state_;
      q_head_.store(head + 1U, std::memory_order_release);
      return;
    }

    // Full: drop oldest by advancing tail so that used becomes SIZE-1 after drop.
    // We want tail = head - (SIZE - 1)
    const uint32_t desired_tail = head - (STATE_QUEUE_SIZE - 1U);

    // tail may have advanced already by consumer; ensure we never move it backward.
    if (desired_tail <= tail)
    {
      // Consumer already advanced enough (or races); retry (should fall into "space" case next).
      continue;
    }

    // Attempt to advance tail (drop oldest). If it fails due to race, retry.
    (void)q_tail_.compare_exchange_weak(
      tail,
      desired_tail,
      std::memory_order_acq_rel,
      std::memory_order_acquire
    );
  }
}

bool GPS_Backend::pop_state_snapshot(GPS_State& out)
{
  while (true)
  {
    const uint32_t tail = q_tail_.load(std::memory_order_relaxed);
    const uint32_t head = q_head_.load(std::memory_order_acquire);

    if (tail == head)
    {
      // empty
      return false;
    }

    const uint32_t idx = tail % STATE_QUEUE_SIZE;
    out = state_queue_[idx];

    // Advance tail. If producer overwrote (advanced tail) concurrently, CAS fails and we retry.
    uint32_t expected = tail;
    if (q_tail_.compare_exchange_weak(
          expected,
          tail + 1U,
          std::memory_order_acq_rel,
          std::memory_order_acquire))
    {
      return true;
    }
  }
}
