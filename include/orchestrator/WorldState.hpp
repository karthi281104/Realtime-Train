#pragma once

#include "common/Types.hpp"
#include "conflict/Conflict.hpp"
#include "conflict/ResourceReservation.hpp"
#include "prediction/FutureState.hpp"
#include "safety/SafetyCommand.hpp"

#include <vector>

namespace tcas::orchestrator
{

enum class SystemStatus
{
    Ready,
    Running,
    Paused,
    Degraded,
    Shutdown
};

struct SafetyDecision
{
    TrainId yieldingTrain{ 0 };
    TrainId priorityTrain{ 0 };
    double riskScore{ 0.0 };
    safety::SafetyCommandType commandType{
        safety::SafetyCommandType::NoAction };
};

struct TrainSnapshot
{
    TrainId id{ 0 };
    TrainType type{ TrainType::Passenger };
    TrackId trackId{ 0 };
    TrainState state{ TrainState::Idle };
    DistanceMeters position{ 0.0 };
    SpeedMetersPerSecond velocity{ 0.0 };
    AccelerationMetersPerSecondSquared acceleration{ 0.0 };
    bool sensorFailure{ false };
};

struct WorldState
{
    TimeSeconds simulationTime{ 0.0 };
    std::vector<TrainSnapshot> trains;
    std::vector<prediction::FutureState> predictions;
    std::vector<conflict::Conflict> activeConflicts;
    std::vector<conflict::ResourceReservation> reservations;
    std::vector<safety::SafetyCommand> commands;
    bool sensorFailure{ false };
    bool communicationFailure{ false };
    SystemStatus systemStatus{ SystemStatus::Ready };
    std::vector<SafetyDecision> decisions;
};

} // namespace tcas::orchestrator
