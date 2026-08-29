#include "simulation/SimClock.hpp"

#include <gtest/gtest.h>

using namespace tcas;
using namespace tcas::simulation;

// ============================================================
// Initial State
// ============================================================

TEST(SimClockTest, StartsAtZero)
{
    const SimClock clock;

    EXPECT_DOUBLE_EQ(clock.elapsed(), 0.0);
    EXPECT_EQ(clock.tickCount(), 0u);
}

TEST(SimClockTest, DefaultDtIs20Milliseconds)
{
    const SimClock clock;

    EXPECT_DOUBLE_EQ(clock.dt(), 0.020);
}

TEST(SimClockTest, CustomDtIsRespected)
{
    const SimClock clock{ 0.1 };

    EXPECT_DOUBLE_EQ(clock.dt(), 0.1);
}

// ============================================================
// Tick Advancement
// ============================================================

TEST(SimClockTest, AdvancesElapsedOnOneTick)
{
    SimClock clock;
    clock.tick();

    EXPECT_DOUBLE_EQ(clock.elapsed(), 0.020);
}

TEST(SimClockTest, AdvancesTickCountOnOneTick)
{
    SimClock clock;
    clock.tick();

    EXPECT_EQ(clock.tickCount(), 1u);
}

TEST(SimClockTest, MultipleTicksAccumulateElapsed)
{
    SimClock clock;

    for (int i = 0; i < 10; ++i)
    {
        clock.tick();
    }

    EXPECT_NEAR(clock.elapsed(), 10 * 0.020, 1e-9);
}

TEST(SimClockTest, MultipleTicksAccumulateCount)
{
    SimClock clock;

    for (int i = 0; i < 50; ++i)
    {
        clock.tick();
    }

    EXPECT_EQ(clock.tickCount(), 50u);
}

TEST(SimClockTest, CustomDtAccumulatesCorrectly)
{
    SimClock clock{ 0.1 };

    for (int i = 0; i < 10; ++i)
    {
        clock.tick();
    }

    EXPECT_NEAR(clock.elapsed(), 1.0, 1e-9);
}

// ============================================================
// Monotonic Guarantee
// ============================================================

TEST(SimClockTest, ElapsedNeverDecreasesAcrossTicks)
{
    SimClock clock;
    TimeSeconds previous = clock.elapsed();

    for (int i = 0; i < 100; ++i)
    {
        clock.tick();
        const TimeSeconds current = clock.elapsed();

        EXPECT_GE(current, previous)
            << "Elapsed decreased at tick " << i + 1;

        previous = current;
    }
}

// ============================================================
// Reset
// ============================================================

TEST(SimClockTest, ResetRestoresElapsedToZero)
{
    SimClock clock;

    for (int i = 0; i < 20; ++i)
    {
        clock.tick();
    }

    clock.reset();

    EXPECT_DOUBLE_EQ(clock.elapsed(), 0.0);
}

TEST(SimClockTest, ResetRestoresTickCountToZero)
{
    SimClock clock;

    for (int i = 0; i < 20; ++i)
    {
        clock.tick();
    }

    clock.reset();

    EXPECT_EQ(clock.tickCount(), 0u);
}

TEST(SimClockTest, ClockFunctionsNormallyAfterReset)
{
    SimClock clock;

    for (int i = 0; i < 5; ++i)
    {
        clock.tick();
    }

    clock.reset();
    clock.tick();

    EXPECT_DOUBLE_EQ(clock.elapsed(), 0.020);
    EXPECT_EQ(clock.tickCount(), 1u);
}

// ============================================================
// Large Tick Count (overflow safety check)
// ============================================================

TEST(SimClockTest, HandlesLargeTickCount)
{
    SimClock clock{ 0.020 };

    // Simulate 1 hour of real-time at 20ms ticks
    // = 3600 / 0.020 = 180000 ticks
    const SimTimeTick targetTicks = 180'000u;

    for (SimTimeTick i = 0u; i < targetTicks; ++i)
    {
        clock.tick();
    }

    EXPECT_EQ(clock.tickCount(), targetTicks);
    EXPECT_NEAR(clock.elapsed(), 3600.0, 1e-6);
}
