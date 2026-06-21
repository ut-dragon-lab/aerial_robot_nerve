/*
******************************************************************************
* File Name          : gps_backend.h
* Description        : Basic class of gps node
******************************************************************************
*/
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef GPS_BACKEND_H
#define GPS_BACKEND_H

#include "util/ring_buffer.h"
#include "math/AP_Math.h"
#include "config.h"

#include <atomic>
#include <STM32Hardware.h>

#define GPS_BUFFER_SIZE 512

using namespace ap;

/// GPS status codes
enum GPS_Status {
  NO_GPS = 0,             ///< No GPS connected/detected
  NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
  GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
  GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
  GPS_OK_FIX_3D_DGPS = 4, ///< Receiving valid messages and 3D lock with differential improvements
  GPS_OK_FIX_3D_RTK = 5,  ///< Receiving valid messages and 3D lock, with relative-positioning improvements
};

struct GPS_State {
  uint8_t status;                  ///< driver fix status
  uint32_t time_week_ms;           ///< GPS time (milliseconds from start of GPS week)
  uint32_t utc_time;
  uint16_t utc_year;
  uint8_t utc_month;
  uint8_t utc_day;
  uint8_t utc_hour;
  uint8_t utc_min;
  uint8_t utc_sec;
  int32_t utc_nano;
  uint8_t utc_acc;
  uint8_t utc_valid;
  Location location;               ///< last fix location
  float ground_speed;              ///< ground speed in m/sec
  int32_t ground_course_cd;        ///< ground course in 100ths of a degree
  uint16_t hdop;                   ///< horizontal dilution of precision in cm
  uint16_t vdop;                   ///< vertical dilution of precision in cm
  uint8_t num_sats;                ///< Number of visible satelites
  Vector3f velocity;               ///< 3D velocitiy in m/s, in NED format
  float speed_accuracy;
  float horizontal_accuracy;
  float vertical_accuracy;
  bool have_vertical_velocity;     ///< does this GPS give vertical velocity?
  bool have_speed_accuracy;
  bool have_horizontal_accuracy;
  bool have_vertical_accuracy;
  uint32_t last_gps_time_ms;       ///< the system time we got the last GPS timestamp, milliseconds
  bool mag_valid;
  float mag_dec;
};

struct GPS_timing {
  uint32_t last_fix_time_ms;
  uint32_t last_message_time_ms;
};

class GPS_Backend
{
public:
  GPS_Backend()
  {
    state_.status = NO_FIX;
    state_.mag_valid = false;
  }

  virtual ~GPS_Backend(void) {}

  virtual void update() = 0;

  const GPS_State& getGpsState() { return state_; }
  const Location& location() const { return state_.location; }
  const Location& location(uint8_t instance) const { return state_.location; }
  const Vector3f& velocity() const { return state_.velocity; }
  float ground_speed() const { return state_.ground_speed; }
  uint32_t ground_speed_cm(void) { return ground_speed() * 100; }
  int32_t ground_course_cd() const { return state_.ground_course_cd; }
  uint8_t num_sats() const {  return state_.num_sats; }
  uint32_t time_week_ms() const { return state_.time_week_ms; }
  uint32_t last_fix_time_ms() const { return timing_.last_fix_time_ms; }
  uint32_t last_message_time_ms() const { return timing_.last_message_time_ms; }
  bool getMagValid() const { return state_.mag_valid; }
  float getMagDeclination() const { return state_.mag_dec; }

  void write(const uint8_t data_byte)
  {
    uint8_t data[1];
    data[0] = data_byte;
    HAL_UART_Transmit(huart_, data, 1, 100);
  }

  void write(const uint8_t * data_byte, uint16_t size)
  {
    HAL_UART_Transmit(huart_, (uint8_t *)data_byte, size, 100);
  }

  // ---- State queue (producer: GPS driver, consumer: ROS module) ----
  bool pop_state_snapshot(GPS_State& out);

protected:
  UART_HandleTypeDef *huart_;

  GPS_State state_;
  GPS_timing timing_;

  void init(UART_HandleTypeDef* huart);

  virtual void processMessage() = 0;

  bool available();
  int read();

  // Push a snapshot of current `state_` into the queue.
  // If full, it drops the oldest one and stores the newest snapshot.
  void push_state_snapshot();

private:
  static constexpr uint32_t STATE_QUEUE_SIZE = 8; // fixed length, overwrite oldest when full

  GPS_State state_queue_[STATE_QUEUE_SIZE];

  // Monotonic counters (not modulo). Index = counter % STATE_QUEUE_SIZE
  std::atomic<uint32_t> q_head_{0}; // next write
  std::atomic<uint32_t> q_tail_{0}; // next read
};

#endif // GPS_BACKEND_H
