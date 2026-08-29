#include "simulation/SimulationTimer.hpp"

#include <cassert>

namespace tcas::simulation
{

SimulationTimer::SimulationTimer(
    const std::uint32_t periodMs,
    const std::uint32_t physicsPeriodMs
)
    : intervalTicks_{ 1u }
{
    assert(periodMs        > 0u && "SimulationTimer: periodMs must be > 0");
    assert(physicsPeriodMs > 0u && "SimulationTimer: physicsPeriodMs must be > 0");
    assert(periodMs >= physicsPeriodMs &&
           "SimulationTimer: periodMs must be >= physicsPeriodMs");

    // Floor-divide so we always get a whole number of ticks.
    intervalTicks_ = static_cast<SimTimeTick>(periodMs / physicsPeriodMs);

    // Guard against a zero interval (would fire every tick regardless).
    if (intervalTicks_ == 0u)
    {
        intervalTicks_ = 1u;
    }
}

bool SimulationTimer::shouldFire(const SimTimeTick tickCount) const noexcept
{
    return (tickCount % intervalTicks_) == 0u;
}

SimTimeTick SimulationTimer::intervalTicks() const noexcept
{
    return intervalTicks_;
}

} // namespace tcas::simulation
