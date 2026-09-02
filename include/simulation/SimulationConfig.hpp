#pragma once

#include <cstdint>

#include "common/Types.hpp"

namespace tcas::simulation {

// Holds all timing parameters for the simulation.
// Consumed by SimClock, SimulationTimer, and later
// by the Thread Orchestrator (Module 11).
struct SimulationConfig {
  // Physics update interval in milliseconds.
  // Drives the simulation clock tick rate.
  // Must be greater than zero.
  std::uint32_t physicsPeriodMs{20u};

  // Safety subsystem check interval in milliseconds.
  std::uint32_t safetyPeriodMs{100u};

  // Communication subsystem update interval in milliseconds.
  std::uint32_t communicationPeriodMs{100u};

  // HMI / telemetry refresh interval in milliseconds.
  std::uint32_t hmiPeriodMs{200u};

  // Look-ahead window for the Predictive Position Engine (Module 8).
  // Expressed in seconds.
  // Must be greater than zero.
  TimeSeconds predictionHorizonSeconds{60.0};
};

}  // namespace tcas::simulation
