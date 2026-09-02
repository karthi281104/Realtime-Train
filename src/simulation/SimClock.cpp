#include "simulation/SimClock.hpp"

#include <cassert>

namespace tcas::simulation {

SimClock::SimClock(const TimeSeconds dt)
    : dt_{dt}, elapsed_{0.0}, tickCount_{0u} {
  assert(dt_ > 0.0 && "SimClock: dt must be greater than zero");
}

void SimClock::tick() {
  elapsed_ += dt_;
  ++tickCount_;
}

void SimClock::reset() {
  elapsed_ = 0.0;
  tickCount_ = 0u;
}

TimeSeconds SimClock::elapsed() const noexcept { return elapsed_; }

SimTimeTick SimClock::tickCount() const noexcept { return tickCount_; }

TimeSeconds SimClock::dt() const noexcept { return dt_; }

}  // namespace tcas::simulation
