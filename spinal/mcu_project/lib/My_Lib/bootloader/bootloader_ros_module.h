#pragma once

#ifndef SIMULATION

#include <atomic>
#include <cstdint>

#include <std_srvs/srv/trigger.h>

#include <ros_utils/ros_module_base.hpp>

class ThrusterManager;

class BootloaderRosModule final : public RosModuleBase
{
public:
  BootloaderRosModule()
  : RosModuleBase(
      RosModuleEntityCapacity()
        .max_subscriptions(0)
        .max_publishers(0)
        .max_services(1)
        .max_timers(0))
  {}

  void init_hw(ThrusterManager* thruster);
  void create_entities(rcl_node_t& node) override;
  void destroy_entities(rcl_node_t& node) override;
  void update() override;

private:
  static constexpr uint32_t kResetDelayMs = 1500UL;

  ThrusterManager* thruster_{nullptr};
  rcl_service_t enter_bootloader_srv_{};
  std_srvs__srv__Trigger_Request request_{};
  std_srvs__srv__Trigger_Response response_{};
  std::atomic<bool> reset_pending_{false};
  uint32_t reset_at_ms_{0UL};
  bool messages_initialized_{false};

  static BootloaderRosModule* instance_;
  static void enterBootloaderCallbackStatic_(const void* request, void* response);
};

#endif  // !SIMULATION
