#include "sensor/Odometer.hpp"

#include <gtest/gtest.h>

using namespace tcas;
using namespace tcas::sensor;

TEST(OdometerTest, MeasuresIdealKinematicsWithoutDriftAtRest)
{
    Odometer odometer;
    const auto m = odometer.measure(0.0, 0.0, 0.0, 0.02, 1);

    EXPECT_TRUE(m.isValid);
    EXPECT_DOUBLE_EQ(m.rawPosition, 0.0);
    EXPECT_DOUBLE_EQ(m.rawVelocity, 0.0);
    EXPECT_DOUBLE_EQ(m.rawAcceleration, 0.0);
    EXPECT_EQ(m.timestamp, 1u);
}

TEST(OdometerTest, AccumulatesDriftDuringMotion)
{
    SensorNoiseConfig config;
    config.driftRatePerSecond = 0.02; // 2% drift

    Odometer odometer(config);

    // v = 20 m/s, dt = 1.0 s => driftDelta = 20 * 0.02 * 1.0 = 0.4 m
    const auto m = odometer.measure(20.0, 20.0, 0.0, 1.0, 10);

    EXPECT_TRUE(m.isValid);
    EXPECT_DOUBLE_EQ(odometer.accumulatedDrift(), 0.4);
    EXPECT_DOUBLE_EQ(m.rawPosition, 20.4);
    EXPECT_DOUBLE_EQ(m.rawVelocity, 20.0);
}

TEST(OdometerTest, CalibrateResetsDrift)
{
    SensorNoiseConfig config;
    config.driftRatePerSecond = 0.05;

    Odometer odometer(config);
    const auto m1 = odometer.measure(100.0, 25.0, 0.0, 2.0, 1);
    EXPECT_TRUE(m1.isValid);

    EXPECT_GT(odometer.accumulatedDrift(), 0.0);

    // Passing a balise recalibrates odometer
    odometer.calibrate(100.0);

    EXPECT_DOUBLE_EQ(odometer.accumulatedDrift(), 0.0);
}

TEST(OdometerTest, FaultyOdometerReturnsInvalidMeasurement)
{
    Odometer odometer;
    EXPECT_FALSE(odometer.isFaulty());

    odometer.setFaulty(true);
    EXPECT_TRUE(odometer.isFaulty());

    const auto m = odometer.measure(50.0, 15.0, 0.0, 0.02, 5);
    EXPECT_FALSE(m.isValid);
    EXPECT_GT(m.rawPosition, 9000.0); // Faulty spike
}

TEST(OdometerTest, ResetRestoresInitialState)
{
    Odometer odometer;
    odometer.setFaulty(true);
    const auto m2 = odometer.measure(50.0, 20.0, 0.0, 1.0, 1);
    EXPECT_FALSE(m2.isValid);

    odometer.reset();

    EXPECT_FALSE(odometer.isFaulty());
    EXPECT_DOUBLE_EQ(odometer.accumulatedDrift(), 0.0);
}
