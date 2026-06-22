#pragma once

#include <vector>

#include <rcl/rcl.h>
#include <rclc/executor.h>

#include <spinal_msgs/srv/imu_calib.h>

#include <ros_utils/ros_module_base.hpp>

#include "sensors/imu/imu_basic.h"       // IMU, Vector3f
#include "flashmemory/flashmemory.h"

class ImuRosModule : public RosModuleBase
{
public:
  ImuRosModule()
  : RosModuleBase(
      RosModuleEntityCapacity()
        .max_subscriptions(0)
        .max_publishers(0)
        .max_services(1)
        .max_timers(0))
  {}

  void addImu(IMU* imu);

  void create_entities(rcl_node_t& node) override;

private:
  // ---- same as imu_ros_cmd.cpp ----
  static constexpr uint8_t CALIB_DATA_SIZE = 12;   // gyro_bias(3)+acc_bias(3)+mag_bias(3)+mag_scale(3)
  static constexpr uint8_t MAX_IMU_COUNT   = 4;
  static constexpr uint16_t MAX_CALIB_FLOATS = MAX_IMU_COUNT * CALIB_DATA_SIZE;

  std::vector<IMU*> imu_;
  bool first_call_{true};

  rcl_service_t imu_calib_srv_{};
  spinal_msgs__srv__ImuCalib_Request  req_{};
  spinal_msgs__srv__ImuCalib_Response res_{};

  float calib_data_buf_[MAX_CALIB_FLOATS]{};

  void getImuCalibData_();
  static void imuCalibCallbackStatic_(const void* req_msg, void* res_msg);

  static ImuRosModule* instance_;
};
