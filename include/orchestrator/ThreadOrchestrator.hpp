#pragma once

#include "communication/CommunicationChannel.hpp"
#include "hmi/HmiDisplay.hpp"
#include "hmi/PerformanceMetrics.hpp"
#include "hmi/TelemetryLogger.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "orchestrator/CommandQueue.hpp"
#include "orchestrator/WorldState.hpp"
#include "train/TrainManager.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>

namespace tcas::orchestrator
{

struct OrchestratorConfig
{
    std::chrono::milliseconds physicsPeriod{ 20 };
    std::chrono::milliseconds safetyPeriod{ 100 };
    std::chrono::milliseconds communicationPeriod{ 100 };
    std::chrono::milliseconds hmiPeriod{ 200 };
    bool printHmi{ false };
    std::string telemetryDirectory{ "logs" };
};

struct SafetyCycleResult
{
    std::vector<prediction::FutureState> predictions;
    std::vector<conflict::Conflict> activeConflicts;
    std::vector<conflict::ResourceReservation> reservations;
    std::vector<safety::SafetyCommand> commands;
};

using SafetyStep = std::function<SafetyCycleResult(const WorldState&)>;

class ThreadOrchestrator
{
public:
    ThreadOrchestrator(
        const infrastructure::RailwayNetwork& network,
        train::TrainManager& trainManager,
        communication::CommunicationChannel& communicationChannel,
        std::vector<TrainId> trainIds,
        OrchestratorConfig config = {},
        SafetyStep safetyStep = {}
    );

    ~ThreadOrchestrator();

    ThreadOrchestrator(const ThreadOrchestrator&) = delete;
    ThreadOrchestrator& operator=(const ThreadOrchestrator&) = delete;

    void start();
    void stop();

    [[nodiscard]] bool isRunning() const noexcept;
    [[nodiscard]] WorldState snapshot() const;
    [[nodiscard]] std::size_t physicsCycles() const noexcept;
    [[nodiscard]] std::size_t safetyCycles() const noexcept;
    [[nodiscard]] std::size_t communicationCycles() const noexcept;
    [[nodiscard]] std::size_t hmiCycles() const noexcept;

    void setSafetyStep(SafetyStep safetyStep);

private:
    void physicsLoop();
    void safetyLoop();
    void communicationLoop();
    void hmiLoop();
    void updateWorldSnapshotLocked();
    void waitUntil(std::chrono::steady_clock::time_point next);

    const infrastructure::RailwayNetwork& network_;
    train::TrainManager& trainManager_;
    communication::CommunicationChannel& communicationChannel_;
    std::vector<TrainId> trainIds_;
    OrchestratorConfig config_;
    hmi::TelemetryLogger telemetryLogger_;
    hmi::PerformanceMetrics performanceMetrics_;

    mutable std::shared_mutex worldMutex_;
    WorldState worldState_;
    CommandQueue commandQueue_;

    mutable std::mutex safetyStepMutex_;
    SafetyStep safetyStep_;

    std::atomic<bool> running_{ false };
    std::thread physicsThread_;
    std::thread safetyThread_;
    std::thread communicationThread_;
    std::thread hmiThread_;

    mutable std::mutex shutdownMutex_;
    std::condition_variable shutdownCondition_;

    std::atomic<std::size_t> physicsCycles_{ 0 };
    std::atomic<std::size_t> safetyCycles_{ 0 };
    std::atomic<std::size_t> communicationCycles_{ 0 };
    std::atomic<std::size_t> hmiCycles_{ 0 };
};

} // namespace tcas::orchestrator
