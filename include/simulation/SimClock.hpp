#pragma once

#include "common/Types.hpp"

namespace tcas::simulation
{

// Deterministic discrete-step simulation clock.
//
// Advances in fixed-size time steps (dt) driven by explicit tick() calls.
// Has no dependency on the system wall-clock, making it fully reproducible
// and controllable in unit tests.
//
// All TCAS modules that need to know the current simulation time should
// query SimClock rather than std::chrono or any real-time source.
//
// Thread safety: SimClock is not thread-safe. In a multi-threaded context
// (Module 11), access must be serialised externally.
class SimClock
{
public:
    // Construct a clock with the given time step.
    // dt must be greater than zero.
    // Default: 0.020 s (20 ms), matching simulation.json physics_period_ms.
    explicit SimClock(TimeSeconds dt = 0.020);

    // Advance simulation time by one tick (dt seconds).
    void tick();

    // Reset the clock to t = 0, tick count = 0.
    void reset();

    // Current simulation time in seconds.
    [[nodiscard]]
    TimeSeconds elapsed() const noexcept;

    // Number of ticks that have occurred since construction or last reset().
    [[nodiscard]]
    SimTimeTick tickCount() const noexcept;

    // The fixed time step of this clock in seconds.
    [[nodiscard]]
    TimeSeconds dt() const noexcept;

private:
    TimeSeconds   dt_;
    TimeSeconds   elapsed_;
    SimTimeTick   tickCount_;
};

} // namespace tcas::simulation
