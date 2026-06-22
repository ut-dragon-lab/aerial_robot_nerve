#pragma once

#include <cstdint>

#include <config.h>
#include "flashmemory/flashmemory.h"

#ifndef VOLTAGE_CHECK_INTERVAL
#define VOLTAGE_CHECK_INTERVAL 20 // ms
#endif

#ifndef ROS_PUB_INTERVAL
#define ROS_PUB_INTERVAL 100 // ms
#endif

#ifndef BATTERY_STATUS_TELE_VOLTAGE_SCALE
#define BATTERY_STATUS_TELE_VOLTAGE_SCALE 1.022f
#endif

class BatteryStatus
{
public:
  BatteryStatus() = default;
  ~BatteryStatus() = default;

  void init(ADC_HandleTypeDef* hadc, bool is_adc_measure = true)
  {
    hadc_ = hadc;
    voltage_ = -1.0f;
    is_adc_measure_ = is_adc_measure;
    ros_pub_last_time_ = HAL_GetTick();

    FlashMemory::addValue(&adc_scale_, sizeof(float));

    if (hadc_ != nullptr) {
      HAL_ADC_Start(hadc_);
    }
  }

  void setAdcScale(float adc_scale)
  {
    adc_scale_ = adc_scale;
    voltage_ = -1.0f;

    FlashMemory::erase();
    FlashMemory::write();
  }

  bool update(float voltage = -1.0f)
  {
    if (voltage == -1.0f) {
      if (!is_adc_measure_ || hadc_ == nullptr) return false;

      if (HAL_ADC_PollForConversion(hadc_, 10) == HAL_OK) {
        adc_value_ = HAL_ADC_GetValue(hadc_);
      }

      HAL_ADC_Start(hadc_);

      voltage = adc_scale_ * adc_value_;
    } else {
      voltage = BATTERY_STATUS_TELE_VOLTAGE_SCALE * voltage;
    }

    if (voltage_ < 0.0f) voltage_ = voltage;

    if (voltage > 0.0f) {
      voltage_ = 0.99f * voltage_ + 0.01f * voltage;
    }

    return true;
  }

  bool voltagePublishReady(bool update_last_time = false)
  {
    const uint32_t now = HAL_GetTick();
    if (now - ros_pub_last_time_ <= ROS_PUB_INTERVAL) return false;

    if (update_last_time) {
      ros_pub_last_time_ = now;
    }
    return true;
  }

  float getVoltage() const { return voltage_; }
  bool isAdcMeasure() const { return is_adc_measure_; }

private:
  ADC_HandleTypeDef* hadc_{nullptr};

  float adc_value_{0.0f};
  float adc_scale_{0.0f};
  float voltage_{-1.0f};

  bool is_adc_measure_{true};

  uint32_t ros_pub_last_time_{0};
};
