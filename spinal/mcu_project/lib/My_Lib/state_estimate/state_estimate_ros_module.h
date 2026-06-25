#pragma once

#ifndef SIMULATION

#include <ros_utils/ros_module_base.hpp>
#include <rmw_microros/rmw_microros.h>

#include <spinal_msgs/msg/imu.h>
#include <spinal_msgs/srv/mag_declination.h>

#include "state_estimate/state_estimate.h"

class StateEstimateRosModule final : public RosModuleBase
{
public:
  StateEstimateRosModule()
  : RosModuleBase(
      RosModuleEntityCapacity()
        .max_subscriptions(0)
        .max_publishers(1)
        .max_services(1)
        .max_timers(0))
  {}

  void init_hw(IMU* imu, Baro* baro, GPS* gps)
  {
    estimator_.init(imu, baro, gps);
  }

  void create_entities(rcl_node_t& node) override;
  void update() override;

  StateEstimate* getStateEstimateCore() { return &estimator_; }
  void publish() override;
  uint32_t millisToNextPublish() const;

private:
  static constexpr uint8_t IMU_PUB_INTERVAL_MS = 5; // 200Hz
  uint32_t last_imu_pub_time_ms_{0};

  // core
  StateEstimate estimator_;

  // pub
  rcl_publisher_t imu_pub_{};
  spinal_msgs__msg__Imu imu_msg_{};

  // service
  rcl_service_t mag_declination_srv_{};
  spinal_msgs__srv__MagDeclination_Request mag_declination_req_{};
  spinal_msgs__srv__MagDeclination_Response mag_declination_res_{};

  // trampoline
  static StateEstimateRosModule* instance_;
  static void magDeclinationCallbackStatic(const void * req_msg, void * res_msg);

  void magDeclinationCallback(const spinal_msgs__srv__MagDeclination_Request& req,
                              spinal_msgs__srv__MagDeclination_Response& res);
};

#endif  // !SIMULATION
