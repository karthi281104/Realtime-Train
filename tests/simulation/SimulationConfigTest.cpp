#include "simulation/SimulationConfig.hpp"

#include <gtest/gtest.h>

using namespace tcas;
using namespace tcas::simulation;

// ============================================================
// Default Values (matching simulation.json)
// ============================================================

TEST(SimulationConfigTest, DefaultPhysicsPeriodIs20Ms)
{
    const SimulationConfig config;

    EXPECT_EQ(config.physicsPeriodMs, 20u);
}

TEST(SimulationConfigTest, DefaultSafetyPeriodIs100Ms)
{
    const SimulationConfig config;

    EXPECT_EQ(config.safetyPeriodMs, 100u);
}

TEST(SimulationConfigTest, DefaultCommunicationPeriodIs100Ms)
{
    const SimulationConfig config;

    EXPECT_EQ(config.communicationPeriodMs, 100u);
}

TEST(SimulationConfigTest, DefaultHmiPeriodIs200Ms)
{
    const SimulationConfig config;

    EXPECT_EQ(config.hmiPeriodMs, 200u);
}

TEST(SimulationConfigTest, DefaultPredictionHorizonIs60Seconds)
{
    const SimulationConfig config;

    EXPECT_DOUBLE_EQ(config.predictionHorizonSeconds, 60.0);
}

// ============================================================
// Custom Values (aggregate initialisation)
// ============================================================

TEST(SimulationConfigTest, AcceptsCustomPhysicsPeriod)
{
    const SimulationConfig config{
        .physicsPeriodMs = 10u
    };

    EXPECT_EQ(config.physicsPeriodMs, 10u);
}

TEST(SimulationConfigTest, AcceptsFullCustomConfig)
{
    const SimulationConfig config{
        .physicsPeriodMs       = 10u,
        .safetyPeriodMs        = 50u,
        .communicationPeriodMs = 50u,
        .hmiPeriodMs           = 100u,
        .predictionHorizonSeconds = 30.0
    };

    EXPECT_EQ(config.physicsPeriodMs,        10u);
    EXPECT_EQ(config.safetyPeriodMs,         50u);
    EXPECT_EQ(config.communicationPeriodMs,  50u);
    EXPECT_EQ(config.hmiPeriodMs,           100u);
    EXPECT_DOUBLE_EQ(config.predictionHorizonSeconds, 30.0);
}
