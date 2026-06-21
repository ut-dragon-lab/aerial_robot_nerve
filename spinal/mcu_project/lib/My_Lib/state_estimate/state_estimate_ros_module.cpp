#ifndef SIMULATION

#include "state_estimate/state_estimate_ros_module.h"

#include <rcl/error_handling.h>
#include <string.h>

StateEstimateRosModule* StateEstimateRosModule::instance_ = nullptr;
uint32_t now_ms_test;

void StateEstimateRosModule::create_entities(rcl_node_t& node)
{
  reserve_entities();

  // publisher
  (void)init_publisher_default(
    node,
    imu_pub_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(spinal_msgs, msg, Imu),
    "imu");

  spinal_msgs__srv__MagDeclination_Request__init(&mag_declination_req_);
  spinal_msgs__srv__MagDeclination_Response__init(&mag_declination_res_);

  (void)init_service_default(
    node,
    mag_declination_srv_,
    ROSIDL_GET_SRV_TYPE_SUPPORT(spinal_msgs, srv, MagDeclination),
    "mag_declination",
    &mag_declination_req_,
    &mag_declination_res_,
    &magDeclinationCallbackStatic);
  
  instance_ = this;
  last_imu_pub_time_ms_ = HAL_GetTick();
}

void StateEstimateRosModule::update()
{
  estimator_.update();
}

uint32_t StateEstimateRosModule::millisToNextPublish() const
{
  const uint32_t now_ms = HAL_GetTick();
  const uint32_t elapsed_ms = now_ms - last_imu_pub_time_ms_;

  if (elapsed_ms >= IMU_PUB_INTERVAL_MS) return 0;
  return IMU_PUB_INTERVAL_MS - elapsed_ms;
}

void StateEstimateRosModule::magDeclinationCallbackStatic(const void * req_msg, void * res_msg)
{
  if (!instance_) return;
  if (!req_msg || !res_msg) return;

  instance_->magDeclinationCallback(
    *reinterpret_cast<const spinal_msgs__srv__MagDeclination_Request*>(req_msg),
    *reinterpret_cast<spinal_msgs__srv__MagDeclination_Response*>(res_msg));
}

void StateEstimateRosModule::magDeclinationCallback(
  const spinal_msgs__srv__MagDeclination_Request& req,
  spinal_msgs__srv__MagDeclination_Response& res)
{
  // defaults
  res.success = false;

  AttitudeEstimate* att = estimator_.getAttEstimator();
  if (!att) return;

  switch (req.command) {
    case spinal_msgs__srv__MagDeclination_Request__GET_DECLINATION:
      res.data = att->getMagDeclination();
      res.success = true;
      break;

    case spinal_msgs__srv__MagDeclination_Request__SET_DECLINATION:
      att->setMagDeclination(req.data);
      res.success = true;
      break;

    default:
      break;
  }
}


void StateEstimateRosModule::publish()
{
  if (ros_ready_ == nullptr) return;
  if (!ros_ready_->load(std::memory_order_acquire)) return;

  const uint32_t now_ms = HAL_GetTick();
  now_ms_test = now_ms;

  const uint32_t elapsed_ms = now_ms - last_imu_pub_time_ms_;

  if (elapsed_ms < IMU_PUB_INTERVAL_MS) return;

  AttitudeEstimate* att = estimator_.getAttEstimator();
  if (!att) return;

  if (!att->consumeUpdated()) return;

  last_imu_pub_time_ms_ += IMU_PUB_INTERVAL_MS;
  if (now_ms - last_imu_pub_time_ms_ >= IMU_PUB_INTERVAL_MS) {
    last_imu_pub_time_ms_ = now_ms;
  }

  const uint64_t t_ms = rmw_uros_epoch_millis();

  const ap::Vector3f mag  = att->getMagVec();
  const ap::Vector3f acc  = att->getAccVec();
  const ap::Vector3f gyro = att->getGyroVec();
  const ap::Quaternion q  = att->getQuaternion();

  // lock_ros_();

  if (!ros_ready_->load(std::memory_order_acquire)) return;

  // stamp
  imu_msg_.stamp.sec     = (int32_t)(t_ms / 1000ULL);
  imu_msg_.stamp.nanosec = (uint32_t)((t_ms % 1000ULL) * 1000000ULL);

  // values
  imu_msg_.mag[0]  = mag.x;  imu_msg_.mag[1]  = mag.y;  imu_msg_.mag[2]  = mag.z;
  imu_msg_.acc[0]  = acc.x;  imu_msg_.acc[1]  = acc.y;  imu_msg_.acc[2]  = acc.z;
  imu_msg_.gyro[0] = gyro.x; imu_msg_.gyro[1] = gyro.y; imu_msg_.gyro[2] = gyro.z;

  imu_msg_.quaternion[0] = q[1];
  imu_msg_.quaternion[1] = q[2];
  imu_msg_.quaternion[2] = q[3];
  imu_msg_.quaternion[3] = q[0];

  (void)rcl_publish(&imu_pub_, &imu_msg_, nullptr);
  // unlock_ros_();  
}

#endif  // !SIMULATION
