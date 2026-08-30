#include "physics/KinematicsEngine.hpp"

#include <cmath>
#include <gtest/gtest.h>

using namespace tcas;
using namespace tcas::physics;

// ============================================================
// updatePosition
// ============================================================

TEST(KinematicsEngineTest, PositionConstantVelocity)
{
    // x = 0 + 10*2 + 0 = 20
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updatePosition(0.0, 10.0, 0.0, 2.0),
        20.0
    );
}

TEST(KinematicsEngineTest, PositionWithAcceleration)
{
    // x = 100 + 20*3 + 0.5*2*9 = 100 + 60 + 9 = 169
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updatePosition(100.0, 20.0, 2.0, 3.0),
        169.0
    );
}

TEST(KinematicsEngineTest, PositionZeroDtUnchanged)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updatePosition(50.0, 10.0, 1.0, 0.0),
        50.0
    );
}

TEST(KinematicsEngineTest, PositionNegativeDtUnchanged)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updatePosition(50.0, 10.0, 1.0, -1.0),
        50.0
    );
}

TEST(KinematicsEngineTest, PositionDecelerating)
{
    // x = 0 + 30*1 + 0.5*(-1)*1 = 30 - 0.5 = 29.5
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updatePosition(0.0, 30.0, -1.0, 1.0),
        29.5
    );
}

// ============================================================
// updateVelocity
// ============================================================

TEST(KinematicsEngineTest, VelocityConstantNoAcceleration)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updateVelocity(20.0, 0.0, 1.0, 40.0),
        20.0
    );
}

TEST(KinematicsEngineTest, VelocityClampedAtMaximum)
{
    // v = 30 + 5*5 = 55, clamped to 40
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updateVelocity(30.0, 5.0, 5.0, 40.0),
        40.0
    );
}

TEST(KinematicsEngineTest, VelocityClampedAtZero)
{
    // v = 5 + (-10)*2 = -15, clamped to 0
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updateVelocity(5.0, -10.0, 2.0, 40.0),
        0.0
    );
}

TEST(KinematicsEngineTest, VelocityZeroDtUnchanged)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updateVelocity(15.0, 3.0, 0.0, 40.0),
        15.0
    );
}

// ============================================================
// effectiveDeceleration
// ============================================================

TEST(KinematicsEngineTest, EffectiveDecelerationFlatTrack)
{
    // gradient = 0 => effective = nominal
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::effectiveDeceleration(1.0, 0.0),
        1.0
    );
}

TEST(KinematicsEngineTest, EffectiveDecelerationUphill)
{
    // gradient = 0.05 (5%) => g * 0.05 = 0.4903325
    const double expected = 1.0 + KinematicsEngine::kGravity * 0.05;
    EXPECT_NEAR(
        KinematicsEngine::effectiveDeceleration(1.0, 0.05),
        expected,
        1e-9
    );
}

TEST(KinematicsEngineTest, EffectiveDecelerationDownhill)
{
    // gradient = -0.05 => effective deceleration reduced
    const double expected = 1.0 + KinematicsEngine::kGravity * (-0.05);
    EXPECT_NEAR(
        KinematicsEngine::effectiveDeceleration(1.0, -0.05),
        expected,
        1e-9
    );
}

TEST(KinematicsEngineTest, EffectiveDecelerationSteepDownhillClamped)
{
    // Very steep downgrade: nominal 0.5 + g*(-0.1) = 0.5 - 0.98 = -0.48
    // Clamped to kMinDeceleration
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::effectiveDeceleration(0.5, -0.1),
        KinematicsEngine::kMinDeceleration
    );
}

// ============================================================
// brakingDistance
// ============================================================

TEST(KinematicsEngineTest, BrakingDistanceZeroVelocity)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::brakingDistance(0.0, 1.0),
        0.0
    );
}

TEST(KinematicsEngineTest, BrakingDistanceFlatTrack)
{
    // v=30, a=1.0 => d = 900/2 = 450
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::brakingDistance(30.0, 1.0),
        450.0
    );
}

TEST(KinematicsEngineTest, BrakingDistanceWithGradientFlat)
{
    // Same as flat: gradient=0
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::brakingDistance(30.0, 1.0, 0.0),
        450.0
    );
}

TEST(KinematicsEngineTest, BrakingDistanceUphillShorter)
{
    // Uphill: effective deceleration > 1.0 => shorter braking distance
    const double flat     = KinematicsEngine::brakingDistance(30.0, 1.0, 0.0);
    const double uphill   = KinematicsEngine::brakingDistance(30.0, 1.0, 0.05);
    EXPECT_LT(uphill, flat);
}

TEST(KinematicsEngineTest, BrakingDistanceDownhillLonger)
{
    // Downhill: effective deceleration < 1.0 => longer braking distance
    const double flat     = KinematicsEngine::brakingDistance(30.0, 1.0, 0.0);
    const double downhill = KinematicsEngine::brakingDistance(30.0, 1.0, -0.02);
    EXPECT_GT(downhill, flat);
}

TEST(KinematicsEngineTest, BrakingDistanceNegativeVelocityReturnsZero)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::brakingDistance(-5.0, 1.0),
        0.0
    );
}

// ============================================================
// reactionDistance
// ============================================================

TEST(KinematicsEngineTest, ReactionDistanceTypical)
{
    // v=30, t=1.5 => d = 45
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::reactionDistance(30.0, 1.5),
        45.0
    );
}

TEST(KinematicsEngineTest, ReactionDistanceZeroVelocity)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::reactionDistance(0.0, 1.5),
        0.0
    );
}

TEST(KinematicsEngineTest, ReactionDistanceZeroTime)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::reactionDistance(30.0, 0.0),
        0.0
    );
}

TEST(KinematicsEngineTest, ReactionDistanceDefaultParameter)
{
    const double expected = 30.0 * KinematicsEngine::kDefaultReactionTime;
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::reactionDistance(30.0),
        expected
    );
}

// ============================================================
// safeDistance
// ============================================================

TEST(KinematicsEngineTest, SafeDistanceZeroVelocity)
{
    // Even at rest, safetyMargin must be included
    const double safe = KinematicsEngine::safeDistance(0.0, 1.0, 0.0, 1.5, 50.0);
    EXPECT_DOUBLE_EQ(safe, 50.0);
}

TEST(KinematicsEngineTest, SafeDistanceFlatTrack)
{
    // v=30, a=1.0, gradient=0, reaction=1.5, margin=50
    // reaction: 45, braking: 450, margin: 50 => 545
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::safeDistance(30.0, 1.0, 0.0, 1.5, 50.0),
        545.0
    );
}

TEST(KinematicsEngineTest, SafeDistanceUphillSmallerThanFlat)
{
    const double flat   = KinematicsEngine::safeDistance(30.0, 1.0, 0.0, 1.5, 50.0);
    const double uphill = KinematicsEngine::safeDistance(30.0, 1.0, 0.05, 1.5, 50.0);
    EXPECT_LT(uphill, flat);
}

TEST(KinematicsEngineTest, SafeDistanceDefaultParameters)
{
    // Must compile and return a positive value
    const double safe = KinematicsEngine::safeDistance(20.0, 0.8, 0.0);
    EXPECT_GT(safe, 0.0);
}

TEST(KinematicsEngineTest, SafeDistanceNegativeMarginTreatedAsZero)
{
    const double withMargin    = KinematicsEngine::safeDistance(20.0, 1.0, 0.0, 1.5, 50.0);
    const double withoutMargin = KinematicsEngine::safeDistance(20.0, 1.0, 0.0, 1.5, 0.0);
    EXPECT_LT(withoutMargin, withMargin);
}

// ============================================================
// emergencyStoppingDistance
// ============================================================

TEST(KinematicsEngineTest, EmergencyStoppingDistanceShorterThanService)
{
    // Emergency braking (1.2) > service braking (0.8)
    const double service   = KinematicsEngine::brakingDistance(30.0, 0.8, 0.0);
    const double emergency = KinematicsEngine::emergencyStoppingDistance(30.0, 1.2, 0.0);
    EXPECT_LT(emergency, service);
}

TEST(KinematicsEngineTest, EmergencyStoppingDistanceZeroVelocity)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::emergencyStoppingDistance(0.0, 1.2, 0.0),
        0.0
    );
}

// ============================================================
// speedAfterDistance
// ============================================================

TEST(KinematicsEngineTest, SpeedAfterDistanceAccelerating)
{
    // v^2 = 0 + 2*1*100 = 200 => v = sqrt(200)
    EXPECT_NEAR(
        KinematicsEngine::speedAfterDistance(0.0, 1.0, 100.0),
        std::sqrt(200.0),
        1e-9
    );
}

TEST(KinematicsEngineTest, SpeedAfterDistanceDecelerationToStop)
{
    // v^2 = 400 + 2*(-1)*200 = 400-400 = 0
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::speedAfterDistance(20.0, -1.0, 200.0),
        0.0
    );
}

TEST(KinematicsEngineTest, SpeedAfterDistanceOvershootReturnsZero)
{
    // Train stops before distance d — v^2 < 0 clamps to 0
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::speedAfterDistance(10.0, -1.0, 500.0),
        0.0
    );
}

TEST(KinematicsEngineTest, SpeedAfterZeroDistanceUnchanged)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::speedAfterDistance(20.0, 1.0, 0.0),
        20.0
    );
}

// ============================================================
// Edge cases — extreme values
// ============================================================

TEST(KinematicsEngineTest, VeryHighSpeedBrakingDistance)
{
    // v=100 m/s (360 km/h), a=1.0 => d=5000
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::brakingDistance(100.0, 1.0),
        5000.0
    );
}

TEST(KinematicsEngineTest, VerySmallDtPositionUpdate)
{
    // dt=0.001 s, v=10 => delta = 0.01
    EXPECT_NEAR(
        KinematicsEngine::updatePosition(0.0, 10.0, 0.0, 0.001),
        0.01,
        1e-12
    );
}

TEST(KinematicsEngineTest, PositionStopsAtZeroVelocity)
{
    // v = 10 m/s
    // a = -5 m/s²
    // Train stops after 2 seconds.
    //
    // Stopping distance:
    // d = 10*2 + 0.5*(-5)*2²
    //   = 20 - 10
    //   = 10 m
    //
    // Even though dt = 5 seconds, the train must not
    // continue moving backwards.

    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updatePosition(
            0.0,
            10.0,
            -5.0,
            5.0
        ),
        10.0
    );
}

TEST(KinematicsEngineTest, PositionNegativeVelocityIsClamped)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::updatePosition(
            100.0,
            -10.0,
            0.0,
            1.0
        ),
        100.0
    );
}

TEST(KinematicsEngineTest, PositionDecelerationDoesNotMoveBackward)
{
    EXPECT_GE(
        KinematicsEngine::updatePosition(
            100.0,
            5.0,
            -10.0,
            2.0
        ),
        100.0
    );
}

TEST(KinematicsEngineTest, SpeedAfterDistanceRejectsNegativeVelocity)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::speedAfterDistance(
            -20.0,
            1.0,
            100.0
        ),
        0.0
    );
}

TEST(KinematicsEngineTest, SpeedAfterDistanceZeroVelocityWithZeroDistance)
{
    EXPECT_DOUBLE_EQ(
        KinematicsEngine::speedAfterDistance(
            0.0,
            1.0,
            0.0
        ),
        0.0
    );
}
