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
    telemetryLogger_(config_.telemetryDirectory),
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
    std::lock_guard lock(safetyStepMutex_);
    safetyStep_ = std::move(safetyStep);
}

void ThreadOrchestrator::setSensorFault(bool fault)
{
    std::unique_lock lock(worldMutex_);
    worldState_.sensorFailure = fault;
}

void ThreadOrchestrator::setCommFault(bool fault)
{
    std::unique_lock lock(worldMutex_);
    worldState_.communicationFailure = fault;
}

void ThreadOrchestrator::addTrain(TrainId trainId)
{
    std::unique_lock lock(worldMutex_);
    if (std::find(trainIds_.begin(), trainIds_.end(), trainId) == trainIds_.end())
    {
        trainIds_.push_back(trainId);
        updateWorldSnapshotLocked();
    }
}

void ThreadOrchestrator::removeTrain(TrainId trainId)
{
    std::unique_lock lock(worldMutex_);
    const auto it = std::find(trainIds_.begin(), trainIds_.end(), trainId);
    if (it != trainIds_.end())
    {
        trainIds_.erase(it);
        updateWorldSnapshotLocked();
    }
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
            train->type(),
            0,
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
                train->setAcceleration(std::min(train->acceleration(), 0.0));
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
            std::lock_guard lock(safetyStepMutex_);
            step = safetyStep_;
        }
        if (step)
        {
            try
            {
                const SafetyCycleResult result = step(state);
                {
                    std::unique_lock lock(worldMutex_);
                    worldState_.predictions = result.predictions;
                    worldState_.activeConflicts = result.activeConflicts;
                    worldState_.reservations = result.reservations;
                    worldState_.commands = result.commands;
                    worldState_.decisions = result.decisions;
                }
                for (const auto& command : result.commands)
                {
                    commandQueue_.push(command);
                }
            }
            catch (const std::exception& /*e*/)
            {
                // Safety calculation exception handled; thread remains active
            }
        }
        ++safetyCycles_;
        waitUntil(next);
    }
}

void ThreadOrchestrator::communicationLoop()
{
    auto next = std::chrono::steady_clock::now();
    while (running_.load())
    {
        next += config_.communicationPeriod;
        const auto state = snapshot();
        const auto tick = static_cast<SimTimeTick>(
            std::max(0.0, state.simulationTime) * 1000.0);
        const auto sentBefore = communicationChannel_.totalSent();
        const auto deliveredBefore = communicationChannel_.totalDelivered();
        const auto droppedBefore = communicationChannel_.totalDropped();
        communicationChannel_.step(tick);
        {
            std::unique_lock lock(worldMutex_);
            const auto sent = communicationChannel_.totalSent() - sentBefore;
            const auto delivered = communicationChannel_.totalDelivered() - deliveredBefore;
            const auto dropped = communicationChannel_.totalDropped() - droppedBefore;
            worldState_.communicationFailure =
                sent > 0U && dropped > delivered;
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
        const auto started = std::chrono::steady_clock::now();
        const WorldState state = snapshot();
        telemetryLogger_.logSnapshot(state);
        telemetryLogger_.logConflicts(state);
        performanceMetrics_.observe(
            state,
            std::chrono::steady_clock::now() - started);
        if (config_.printHmi)
        {
            hmi::HmiDisplay::render(state, std::cout);
            const auto metrics = performanceMetrics_.snapshot();
            std::cout << "METRICS\n"
                      << "  conflicts observed : " << metrics.conflictObservations << '\n'
                      << "  emergency brakes   : " << metrics.emergencyBrakeCount << '\n'
                      << "  minimum separation : " << metrics.minimumSeparation << " m\n"
                      << "  minimum TTC        : " << metrics.minimumTtc << " s\n"
                      << "  HMI latency max    : " << metrics.maximumHmiLatencyMs << " ms\n";
        }
        ++hmiCycles_;
        waitUntil(next);
    }
}

} // namespace tcas::orchestrator
