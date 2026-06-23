#pragma once

#include <cstddef>

class EscBaseDriver
{
public:
  EscBaseDriver() = default;
  virtual ~EscBaseDriver() = default;

  virtual void writeDuty(const float* target_duty, size_t motor_count) = 0;
};
