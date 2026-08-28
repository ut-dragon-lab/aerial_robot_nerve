#ifndef SIMULATION

#include "bootloader/bootloader_ros_module.h"

#include <rosidl_runtime_c/string_functions.h>

#include "bootloader/system_bootloader.h"
#include "thruster/board/thruster_manager.h"

BootloaderRosModule* BootloaderRosModule::instance_ = nullptr;

void BootloaderRosModule::init_hw(ThrusterManager* thruster)
{
  thruster_ = thruster;
}

void BootloaderRosModule::create_entities(rcl_node_t& node)
{
  reserve_entities();
  instance_ = this;

  const bool request_initialized = std_srvs__srv__Trigger_Request__init(&request_);
  const bool response_initialized = std_srvs__srv__Trigger_Response__init(&response_);
  messages_initialized_ = request_initialized && response_initialized;

  if (!messages_initialized_) {
    if (request_initialized) {
      std_srvs__srv__Trigger_Request__fini(&request_);
    }
    if (response_initialized) {
      std_srvs__srv__Trigger_Response__fini(&response_);
    }
    return;
  }

  (void)init_service_default(
    node,
    enter_bootloader_srv_,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "enter_bootloader",
    &request_,
    &response_,
    &BootloaderRosModule::enterBootloaderCallbackStatic_);
}

void BootloaderRosModule::destroy_entities(rcl_node_t& node)
{
  RosModuleBase::destroy_entities(node);
  if (messages_initialized_) {
    std_srvs__srv__Trigger_Request__fini(&request_);
    std_srvs__srv__Trigger_Response__fini(&response_);
    request_ = std_srvs__srv__Trigger_Request{};
    response_ = std_srvs__srv__Trigger_Response{};
    messages_initialized_ = false;
  }
}

void BootloaderRosModule::update()
{
  if (!reset_pending_.load(std::memory_order_acquire)) {
    return;
  }

  const uint32_t now = HAL_GetTick();
  if (static_cast<int32_t>(now - reset_at_ms_) >= 0) {
    SystemBootloader::request_and_reset();
  }
}

void BootloaderRosModule::enterBootloaderCallbackStatic_(const void* request, void* response)
{
  (void)request;
  auto* result = static_cast<std_srvs__srv__Trigger_Response*>(response);

  if (instance_ == nullptr || instance_->thruster_ == nullptr) {
    result->success = false;
    (void)rosidl_runtime_c__String__assign(&result->message, "bootloader is not initialized");
    return;
  }

  // Latch all motor outputs at idle before acknowledging the update request.
  instance_->thruster_->stopOutputs();
  instance_->reset_at_ms_ = HAL_GetTick() + kResetDelayMs;
  instance_->reset_pending_.store(true, std::memory_order_release);

  result->success = true;
  (void)rosidl_runtime_c__String__assign(
    &result->message, "motor outputs stopped; ROM bootloader reset scheduled");
}

#endif  // !SIMULATION
