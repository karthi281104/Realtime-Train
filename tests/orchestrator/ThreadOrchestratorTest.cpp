#include "communication/CommunicationChannel.hpp"
#include "orchestrator/ThreadOrchestrator.hpp"
#include "train/ExpressTrain.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

namespace tcas::orchestrator
{
namespace
{

std::unique_ptr<ThreadOrchestrator> makeOrchestrator(
    train::TrainManager& manager,
    communication::CommunicationChannel& channel)
{
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.getTrain(1)->setVelocity(10.0);

    static const infrastructure::RailwayNetwork network;
    const OrchestratorConfig config{};
    const SafetyStep safetyStep =
        [](const WorldState&)
        {
            SafetyCycleResult result;
            result.commands.push_back({
                safety::SafetyCommandType::ReduceSpeed,
                1,
                5.0,
                0.0,
                1.0});
            return result;
        };
    const std::vector<TrainId> trainIds{ 1 };
    return std::make_unique<ThreadOrchestrator>(
        network,
        manager,
        channel,
        trainIds,
        config,
        safetyStep);
}

} // namespace

TEST(ThreadOrchestratorTest, StartStopAndRestart)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    auto orchestrator = makeOrchestrator(manager, channel);

    orchestrator->start();
    EXPECT_TRUE(orchestrator->isRunning());
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    orchestrator->stop();
    EXPECT_FALSE(orchestrator->isRunning());

    const auto firstPhysicsCycles = orchestrator->physicsCycles();
    EXPECT_GT(firstPhysicsCycles, 0U);
    EXPECT_GT(orchestrator->safetyCycles(), 0U);
    EXPECT_GT(orchestrator->communicationCycles(), 0U);
    EXPECT_GT(orchestrator->hmiCycles(), 0U);

    orchestrator->start();
    EXPECT_TRUE(orchestrator->isRunning());
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    orchestrator->stop();
    EXPECT_FALSE(orchestrator->isRunning());
    EXPECT_GT(orchestrator->physicsCycles(), firstPhysicsCycles);
}

TEST(ThreadOrchestratorTest, SafetyCommandReachesPhysics)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    auto orchestrator = makeOrchestrator(manager, channel);

    orchestrator->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    orchestrator->stop();

    ASSERT_NE(manager.getTrain(1), nullptr);
    EXPECT_LE(manager.getTrain(1)->velocity(), 5.0);
}

TEST(ThreadOrchestratorTest, SnapshotContainsTrainState)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    auto orchestrator = makeOrchestrator(manager, channel);

    const auto state = orchestrator->snapshot();
    ASSERT_EQ(state.trains.size(), 1U);
    EXPECT_EQ(state.trains.front().id, 1U);
}

TEST(ThreadOrchestratorTest, SafetyResultPopulatesWorldState)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    static const infrastructure::RailwayNetwork network;
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));

    ThreadOrchestrator orchestrator(
        network,
        manager,
        channel,
        std::vector<TrainId>{ 1 },
        OrchestratorConfig{},
        [](const WorldState&)
        {
            SafetyCycleResult result;
            result.activeConflicts.push_back({
                1, 2, conflict::ConflictType::Junction,
                0, 9, 1.0, 2.0, 0.0});
            return result;
        });

    orchestrator.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    orchestrator.stop();

    EXPECT_FALSE(orchestrator.snapshot().activeConflicts.empty());
}

TEST(ThreadOrchestratorTest, PeriodsProduceExpectedCycleRanges)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    auto orchestrator = makeOrchestrator(manager, channel);

    orchestrator->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    orchestrator->stop();

    EXPECT_GE(orchestrator->physicsCycles(), 40U);
    EXPECT_LE(orchestrator->physicsCycles(), 60U);
    EXPECT_GE(orchestrator->safetyCycles(), 8U);
    EXPECT_LE(orchestrator->safetyCycles(), 12U);
    EXPECT_GE(orchestrator->communicationCycles(), 8U);
    EXPECT_LE(orchestrator->communicationCycles(), 12U);
    EXPECT_GE(orchestrator->hmiCycles(), 4U);
    EXPECT_LE(orchestrator->hmiCycles(), 7U);
}

TEST(ThreadOrchestratorTest, ConcurrentSnapshotsRemainConsistent)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    auto orchestrator = makeOrchestrator(manager, channel);

    orchestrator->start();
    for (int iteration = 0; iteration < 500; ++iteration)
    {
        const auto state = orchestrator->snapshot();
        EXPECT_LE(state.trains.size(), 1U);
        if (!state.trains.empty())
        {
            EXPECT_TRUE(std::isfinite(state.trains.front().position));
            EXPECT_TRUE(std::isfinite(state.trains.front().velocity));
        }
        std::this_thread::yield();
    }
    orchestrator->stop();

    EXPECT_FALSE(orchestrator->isRunning());
}

} // namespace tcas::orchestrator
