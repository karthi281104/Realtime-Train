#pragma once

#include "common/Types.hpp"
#include "conflict/Conflict.hpp"
#include "conflict/ResourceReservation.hpp"
#include "prediction/FutureState.hpp"
#include "safety/SafetyCommand.hpp"

#include <vector>

namespace tcas::orchestrator
{

struct TrainSnapshot
{
    TrainId id{ 0 };
    TrainType type{ TrainType::Passenger };
    TrackId trackId{ 0 };
    TrainState state{ TrainState::Idle };
    DistanceMeters position{ 0.0 };
    SpeedMetersPerSecond velocity{ 0.0 };
    AccelerationMetersPerSecondSquared acceleration{ 0.0 };
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
};

} // namespace tcas::orchestrator
