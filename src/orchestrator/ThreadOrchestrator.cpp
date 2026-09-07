#include "orchestrator/ThreadOrchestrator.hpp"

#include "physics/KinematicsEngine.hpp"

#include <algorithm>
#include <iostream>

namespace tcas::orchestrator
{

ThreadOrchestrator::ThreadOrchestrator(
    const infrastructure::RailwayNetwork& network,
    train::TrainManager& trainManager,
    communication::CommunicationChannel& communicationChannel,
    std::vector<TrainId> trainIds,
    OrchestratorConfig config,
    SafetyStep safetyStep)
    : network_(network),
      trainManager_(trainManager),
      communicationChannel_(communicationChannel),
      trainIds_(std::move(trainIds)),
      config_(config),
      safetyStep_(std::move(safetyStep))
{
    updateWorldSnapshotLocked();
}

ThreadOrchestrator::~ThreadOrchestrator()
{
    stop();
}

void ThreadOrchestrator::start()
{
    if (running_.exchange(true))
    {
        return;
    }

    physicsThread_ = std::thread(&ThreadOrchestrator::physicsLoop, this);
    safetyThread_ = std::thread(&ThreadOrchestrator::safetyLoop, this);
    communicationThread_ = std::thread(&ThreadOrchestrator::communicationLoop, this);
    hmiThread_ = std::thread(&ThreadOrchestrator::hmiLoop, this);
}

void ThreadOrchestrator::stop()
{
    if (!running_.exchange(false))
    {
        return;
    }

    shutdownCondition_.notify_all();

    if (physicsThread_.joinable())
    {
        physicsThread_.join();
    }
    if (safetyThread_.joinable())
    {
        safetyThread_.join();
    }
    if (communicationThread_.joinable())
    {
        communicationThread_.join();
    }
    if (hmiThread_.joinable())
    {
        hmiThread_.join();
    }
}

bool ThreadOrchestrator::isRunning() const noexcept
{
    return running_.load();
}

WorldState ThreadOrchestrator::snapshot() const
{
    std::shared_lock lock(worldMutex_);
    return worldState_;
}

std::size_t ThreadOrchestrator::physicsCycles() const noexcept
{
    return physicsCycles_.load();
}

std::size_t ThreadOrchestrator::safetyCycles() const noexcept
{
    return safetyCycles_.load();
}

std::size_t ThreadOrchestrator::communicationCycles() const noexcept
{
    return communicationCycles_.load();
}

std::size_t ThreadOrchestrator::hmiCycles() const noexcept
{
    return hmiCycles_.load();
}

void ThreadOrchestrator::setSafetyStep(SafetyStep safetyStep)
{
    std::unique_lock lock(worldMutex_);
    safetyStep_ = std::move(safetyStep);
}

void ThreadOrchestrator::waitUntil(
    const std::chrono::steady_clock::time_point next)
{
    std::unique_lock lock(shutdownMutex_);
    shutdownCondition_.wait_until(
        lock,
        next,
        [this]
        {
            return !running_.load();
        });
}

void ThreadOrchestrator::updateWorldSnapshotLocked()
{
    worldState_.trains.clear();
    worldState_.trains.reserve(trainIds_.size());

    for (const TrainId trainId : trainIds_)
    {
        const auto* train = trainManager_.getTrain(trainId);
        if (train == nullptr)
        {
            continue;
        }

        worldState_.trains.push_back({
            train->id(),
            train->state(),
            train->position(),
            train->velocity(),
            train->acceleration()});
    }
}

void ThreadOrchestrator::physicsLoop()
{
    auto next = std::chrono::steady_clock::now();
    const double dt = config_.physicsPeriod.count() / 1000.0;

    while (running_.load())
    {
        next += config_.physicsPeriod;

        std::unique_lock lock(worldMutex_);
        safety::SafetyCommand command;
        while (commandQueue_.tryPop(&command))
        {
            auto* train = trainManager_.getTrain(command.trainId);
            if (train == nullptr)
            {
                continue;
            }

            switch (command.type)
            {
            case safety::SafetyCommandType::ReduceSpeed:
                train->setVelocity(std::min(train->velocity(), command.targetSpeed));
                break;
            case safety::SafetyCommandType::HoldAtSignal:
            case safety::SafetyCommandType::EmergencyBrake:
                train->setVelocity(0.0);
                train->setAcceleration(0.0);
                train->setState(command.isEmergency()
                    ? TrainState::EmergencyBrake
                    : TrainState::Braking);
                break;
            case safety::SafetyCommandType::NoAction:
                break;
            }
        }

        for (const TrainId trainId : trainIds_)
        {
            auto* train = trainManager_.getTrain(trainId);
            if (train == nullptr)
            {
                continue;
            }

            const auto newPosition = physics::KinematicsEngine::updatePosition(
                train->position(), train->velocity(), train->acceleration(), dt);
            const auto newVelocity = physics::KinematicsEngine::updateVelocity(
                train->velocity(), train->acceleration(), dt, train->maximumSpeed());
            train->setPosition(newPosition);
            train->setVelocity(newVelocity);
        }

        worldState_.simulationTime += dt;
        updateWorldSnapshotLocked();
        ++physicsCycles_;
        lock.unlock();
        waitUntil(next);
    }
}

void ThreadOrchestrator::safetyLoop()
{
    auto next = std::chrono::steady_clock::now();

    while (running_.load())
    {
        next += config_.safetyPeriod;
        const WorldState state = snapshot();
        SafetyStep step;
        {
            std::shared_lock lock(worldMutex_);
            step = safetyStep_;
        }
        if (step)
        {
            step(state, commandQueue_);
        }
        ++safetyCycles_;
        waitUntil(next);
    }
}

void ThreadOrchestrator::communicationLoop()
{
    auto next = std::chrono::steady_clock::now();
    SimTimeTick tick = 0;

    while (running_.load())
    {
        next += config_.communicationPeriod;
        communicationChannel_.step(++tick);
        {
            std::unique_lock lock(worldMutex_);
            worldState_.communicationFailure =
                communicationChannel_.totalDropped() > communicationChannel_.totalDelivered();
        }
        ++communicationCycles_;
        waitUntil(next);
    }
}

void ThreadOrchestrator::hmiLoop()
{
    auto next = std::chrono::steady_clock::now();

    while (running_.load())
    {
        next += config_.hmiPeriod;
        const WorldState state = snapshot();
        if (config_.printHmi)
        {
            std::cout << "[HMI] t=" << state.simulationTime
                      << " trains=" << state.trains.size()
                      << " conflicts=" << state.activeConflicts.size() << '\n';
        }
        ++hmiCycles_;
        waitUntil(next);
    }
}

} // namespace tcas::orchestrator
