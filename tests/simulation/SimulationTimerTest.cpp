#include <gtest/gtest.h>

#include <stdexcept>

#include "simulation/SimulationTimer.hpp"

using namespace tcas;
using namespace tcas::simulation;

// ============================================================
// Interval Calculation
// ============================================================

TEST(SimulationTimerTest, RejectsZeroPeriod) {
  EXPECT_THROW(SimulationTimer(0u, 20u), std::invalid_argument);
}

TEST(SimulationTimerTest, RejectsZeroPhysicsPeriod) {
  EXPECT_THROW(SimulationTimer(100u, 0u), std::invalid_argument);
}

TEST(SimulationTimerTest, RejectsPeriodSmallerThanPhysicsPeriod) {
  EXPECT_THROW(SimulationTimer(10u, 20u), std::invalid_argument);
}

TEST(SimulationTimerTest, SafetyTimerIntervalIs5Ticks) {
  // 100 ms safety / 20 ms physics = 5 ticks
  const SimulationTimer timer{100u, 20u};

  EXPECT_EQ(timer.intervalTicks(), 5u);
}

TEST(SimulationTimerTest, HmiTimerIntervalIs10Ticks) {
  // 200 ms HMI / 20 ms physics = 10 ticks
  const SimulationTimer timer{200u, 20u};

  EXPECT_EQ(timer.intervalTicks(), 10u);
}

TEST(SimulationTimerTest, EqualPeriodGivesIntervalOfOne) {
  // 20 ms period / 20 ms physics = 1 tick
  const SimulationTimer timer{20u, 20u};

  EXPECT_EQ(timer.intervalTicks(), 1u);
}

TEST(SimulationTimerTest, NonExactMultipleIsFloorDivided) {
  // 110 ms / 20 ms = 5.5 → floor = 5
  const SimulationTimer timer{110u, 20u};

  EXPECT_EQ(timer.intervalTicks(), 5u);
}

// ============================================================
// Firing Behaviour — Safety Timer (every 5 ticks)
// ============================================================

TEST(SimulationTimerTest, SafetyTimerFiresAtTickZero) {
  const SimulationTimer timer{100u, 20u};

  EXPECT_TRUE(timer.shouldFire(0u));
}

TEST(SimulationTimerTest, SafetyTimerDoesNotFireAtTick1) {
  const SimulationTimer timer{100u, 20u};

  EXPECT_FALSE(timer.shouldFire(1u));
}

TEST(SimulationTimerTest, SafetyTimerDoesNotFireAtTick4) {
  const SimulationTimer timer{100u, 20u};

  EXPECT_FALSE(timer.shouldFire(4u));
}

TEST(SimulationTimerTest, SafetyTimerFiresAtTick5) {
  const SimulationTimer timer{100u, 20u};

  EXPECT_TRUE(timer.shouldFire(5u));
}

TEST(SimulationTimerTest, SafetyTimerFiresAtTick10) {
  const SimulationTimer timer{100u, 20u};

  EXPECT_TRUE(timer.shouldFire(10u));
}

TEST(SimulationTimerTest, SafetyTimerFiresRepeatedly) {
  const SimulationTimer timer{100u, 20u};

  // Fire expected at multiples of 5
  for (SimTimeTick tick = 0u; tick <= 100u; ++tick) {
    const bool expected = (tick % 5u == 0u);

    EXPECT_EQ(timer.shouldFire(tick), expected)
        << "Unexpected result at tick " << tick;
  }
}

// ============================================================
// Firing Behaviour — HMI Timer (every 10 ticks)
// ============================================================

TEST(SimulationTimerTest, HmiTimerFiresEvery10Ticks) {
  const SimulationTimer timer{200u, 20u};

  for (SimTimeTick tick = 0u; tick <= 100u; ++tick) {
    const bool expected = (tick % 10u == 0u);

    EXPECT_EQ(timer.shouldFire(tick), expected)
        << "Unexpected result at tick " << tick;
  }
}

// ============================================================
// Firing Behaviour — Every-tick Timer
// ============================================================

TEST(SimulationTimerTest, EveryTickTimerAlwaysFires) {
  // period == physicsPeriodMs → interval = 1 → fires every tick
  const SimulationTimer timer{20u, 20u};

  for (SimTimeTick tick = 0u; tick <= 50u; ++tick) {
    EXPECT_TRUE(timer.shouldFire(tick)) << "Did not fire at tick " << tick;
  }
}

// ============================================================
// Integration — SimClock + SimulationTimer
// ============================================================

TEST(SimulationTimerTest, SafetyTimerFiresCorrectlyWithSimClock) {
  // Simulate 50 physics ticks (20 ms each = 1000 ms total)
  // Safety timer fires every 5 ticks → 11 fires expected (0, 5, 10, ..., 50)

  const SimulationTimer safetyTimer{100u, 20u};

  int fireCount = 0;

  for (SimTimeTick tick = 0u; tick <= 50u; ++tick) {
    if (safetyTimer.shouldFire(tick)) {
      ++fireCount;
    }
  }

  // Fires at: 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50 = 11 times
  EXPECT_EQ(fireCount, 11);
}
