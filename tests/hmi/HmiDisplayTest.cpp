#include "hmi/HmiDisplay.hpp"

#include <gtest/gtest.h>

namespace tcas::hmi
{

TEST(HmiDisplayTest, FormatsControlCenterAndTrainState)
{
    orchestrator::WorldState state;
    state.simulationTime = 12.4;
    state.trains.push_back({
        1,
        TrainType::Express,
        101,
        TrainState::Running,
        1240.5,
        22.0,
        0.0});

    const auto output = HmiDisplay::format(state);

    EXPECT_NE(output.find("TCAS CONTROL CENTER"), std::string::npos);
    EXPECT_NE(output.find("TRAIN STATUS"), std::string::npos);
    EXPECT_NE(output.find("1240.50"), std::string::npos);
    EXPECT_NE(output.find("SYSTEM STATE : SAFE"), std::string::npos);
}

TEST(HmiDisplayTest, ReportsConflictsAndCommands)
{
    orchestrator::WorldState state;
    state.activeConflicts.push_back({
        1, 3, conflict::ConflictType::Junction,
        0, 2, 3.42, 5.42, 10.0});
    state.commands.push_back({
        safety::SafetyCommandType::ReduceSpeed,
        3,
        10.0,
        3.42,
        70.0});

    const auto output = HmiDisplay::format(state);

    EXPECT_NE(output.find("ACTIVE CONFLICTS"), std::string::npos);
    EXPECT_NE(output.find("JUNCTION"), std::string::npos);
    EXPECT_NE(output.find("REDUCE SPEED"), std::string::npos);
    EXPECT_NE(output.find("CONFLICT ACTIVE"), std::string::npos);
}

} // namespace tcas::hmi
