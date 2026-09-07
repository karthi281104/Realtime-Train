#include "hmi/TelemetryLogger.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace tcas::hmi
{
namespace
{

class TelemetryLoggerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        directory_ = std::filesystem::temp_directory_path() / "tcas_hmi_test";
        std::filesystem::remove_all(directory_);
    }

    void TearDown() override
    {
        std::filesystem::remove_all(directory_);
    }

    std::filesystem::path directory_;
};

} // namespace

TEST_F(TelemetryLoggerTest, WritesTelemetryAndConflictFiles)
{
    {
        TelemetryLogger logger(directory_);
        orchestrator::WorldState state;
        state.simulationTime = 2.0;
        state.trains.push_back({
            1, TrainType::Express, 101, TrainState::Running,
            100.0, 20.0, 0.0});
        state.activeConflicts.push_back({
            1, 3, conflict::ConflictType::Junction,
            0, 2, 3.0, 5.0, 2.0});

        logger.logSnapshot(state);
        logger.logConflicts(state);
    }

    std::ifstream telemetry(directory_ / "train_telemetry.csv");
    std::ifstream conflicts(directory_ / "conflict_reports.log");
    ASSERT_TRUE(telemetry.good());
    ASSERT_TRUE(conflicts.good());

    const std::string telemetryText(
        (std::istreambuf_iterator<char>(telemetry)),
        std::istreambuf_iterator<char>());
    const std::string conflictText(
        (std::istreambuf_iterator<char>(conflicts)),
        std::istreambuf_iterator<char>());
    EXPECT_NE(telemetryText.find("timestamp,train_id"), std::string::npos);
    EXPECT_NE(telemetryText.find("1,0,101"), std::string::npos);
    EXPECT_NE(conflictText.find("train_a=1"), std::string::npos);
}

TEST_F(TelemetryLoggerTest, WritesEventsFile)
{
    {
        TelemetryLogger logger(directory_);
        orchestrator::WorldState state;
        state.simulationTime = 5.0;
        safety::SafetyCommand cmd;
        cmd.type = safety::SafetyCommandType::EmergencyBrake;
        cmd.trainId = 1;
        state.commands.push_back(cmd);
        state.activeConflicts.push_back({
            1, 2, conflict::ConflictType::RearEnd,
            10, 11, 4.5, 6.0, 15.0});

        logger.logSnapshot(state);
        logger.logConflicts(state);
        logger.logEvent(5.0, "OPERATOR_COMMAND", "Pause simulation");
    }

    std::ifstream events(directory_ / "events.log");
    ASSERT_TRUE(events.good());

    const std::string eventsText(
        (std::istreambuf_iterator<char>(events)),
        std::istreambuf_iterator<char>());
    EXPECT_NE(eventsText.find("SAFETY_COMMAND"), std::string::npos);
    EXPECT_NE(eventsText.find("CONFLICT_DETECTED"), std::string::npos);
    EXPECT_NE(eventsText.find("OPERATOR_COMMAND"), std::string::npos);
}

} // namespace tcas::hmi
