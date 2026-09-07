#include "hmi/PerformanceMetrics.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace tcas::hmi
{

TEST(PerformanceMetricsTest, TracksConflictAndEmergencyMetrics)
{
    PerformanceMetrics metrics;
    orchestrator::WorldState state;
    state.activeConflicts.push_back({
        1, 3, conflict::ConflictType::Junction,
        0, 2, 3.0, 5.0, 4.0});
    state.commands.push_back({
        safety::SafetyCommandType::EmergencyBrake,
        3, 0.0, 3.0, 90.0});

    metrics.observe(state, std::chrono::milliseconds(2));
    const auto result = metrics.snapshot();

    EXPECT_EQ(result.conflictObservations, 1U);
    EXPECT_EQ(result.emergencyBrakeCount, 1U);
    EXPECT_DOUBLE_EQ(result.minimumSeparation, 4.0);
    EXPECT_DOUBLE_EQ(result.minimumTtc, 3.0);
    EXPECT_DOUBLE_EQ(result.maximumHmiLatencyMs, 2.0);
}

} // namespace tcas::hmi
