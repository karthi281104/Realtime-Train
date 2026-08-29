#pragma once

#include <cstdint>

namespace tcas
{

using TrainId = std::uint32_t;
using NodeId = std::uint32_t;
using TrackId = std::uint32_t;
using JunctionId = std::uint32_t;
using PlatformId = std::uint32_t;

using TimeSeconds = double;
using DistanceMeters = double;
using SpeedMetersPerSecond = double;
using AccelerationMetersPerSecondSquared = double;

enum class TrainType
{
    Express,
    Passenger,
    Freight
};

enum class TrainState
{
    Normal,
    Caution,
    Warning,
    Critical,
    EmergencyBrake,
    Stopped,
    Degraded
};

enum class SensorState
{
    Healthy,
    Degraded,
    Failed,
    Recovered
};

enum class CommunicationState
{
    Connected,
    Degraded,
    Disconnected
};

enum class ConflictType
{
    RearEnd,
    HeadOn,
    Junction,
    Platform,
    SharedTrack
};

enum class SafetyCommand
{
    None,
    ReduceSpeed,
    HoldAtSignal,
    EmergencyBrake
};

} // namespace tcas