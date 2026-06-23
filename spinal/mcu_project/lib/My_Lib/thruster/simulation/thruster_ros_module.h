#pragma once

#ifdef SIMULATION

#include "thruster/simulation/thruster_manager.h"

class ThrusterRosModule
{
public:
  ThrusterRosModule() = default;
  ~ThrusterRosModule() = default;

  ThrusterManager* getThrusterManager() { return &thruster_; }
  void sendCommand() { thruster_.sendCommand(); }
  bool updateTelemetry() { return thruster_.updateTelemetry(); }
  void publish() {}

private:
  ThrusterManager thruster_;
};

#endif // SIMULATION
