#pragma once

#include "common/Types.hpp"

#include <cstdint>

namespace tcas::simulation
{

// Periodic subsystem fire trigger for the simulation clock.
//
// Determines whether a given subsystem should execute on the current tick.
// Subsystems fire at a configurable interval relative to the physics tick.
//
// Example:
//   Safety period = 100 ms, physics period = 20 ms.
//   The safety subsystem fires every 5 physics ticks (100 / 20 = 5).
//   shouldFire(0) = true
//   shouldFire(1) = false
//   shouldFire(4) = false
//   shouldFire(5) = true
//
// If periodMs is not an exact multiple of physicsPeriodMs, the interval
// is rounded down to the nearest whole tick (floor). The resulting actual
// period is: floor(periodMs / physicsPeriodMs) * physicsPeriodMs ms.
//
// Thread safety: SimulationTimer is stateless and immutable after
// construction — it is safe to query from multiple threads.
class SimulationTimer
{
public:
    // Construct a timer that fires every `periodMs` milliseconds,
    // given a physics tick of `physicsPeriodMs` milliseconds.
    //
    // Throws std::invalid_argument when either period is zero or when
    // periodMs is smaller than physicsPeriodMs.
    SimulationTimer(
        std::uint32_t periodMs,
        std::uint32_t physicsPeriodMs
    );

    // Returns true when the given tick count is a multiple of the
    // computed interval (in ticks). Always returns true for tick 0.
    [[nodiscard]]
    bool shouldFire(SimTimeTick tickCount) const noexcept;

    // The computed firing interval in ticks.
    [[nodiscard]]
    SimTimeTick intervalTicks() const noexcept;

private:
    SimTimeTick intervalTicks_;
};

} // namespace tcas::simulation
