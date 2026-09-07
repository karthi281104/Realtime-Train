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

} // namespace tcas::hmi
