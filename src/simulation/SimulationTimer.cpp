#include "simulation/SimulationTimer.hpp"

#include <stdexcept>

namespace tcas::simulation {

SimulationTimer::SimulationTimer(const std::uint32_t periodMs,
                                 const std::uint32_t physicsPeriodMs)
    : intervalTicks_{1u} {
  if (periodMs == 0u || physicsPeriodMs == 0u) {
    throw std::invalid_argument(
        "SimulationTimer periods must be greater than zero");
  }

  if (periodMs < physicsPeriodMs) {
    throw std::invalid_argument(
        "SimulationTimer period must not be smaller than physics period");
  }

  // Floor-divide so we always get a whole number of ticks.
  intervalTicks_ = static_cast<SimTimeTick>(periodMs / physicsPeriodMs);
}

bool SimulationTimer::shouldFire(const SimTimeTick tickCount) const noexcept {
  return (tickCount % intervalTicks_) == 0u;
}

SimTimeTick SimulationTimer::intervalTicks() const noexcept {
  return intervalTicks_;
}

}  // namespace tcas::simulation
