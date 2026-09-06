#pragma once

#include "common/Types.hpp"

namespace tcas::safety
{

enum class SafetyCommandType
{
    NoAction,
    ReduceSpeed,
    HoldAtSignal,
    EmergencyBrake
};

struct SafetyCommand
{
    SafetyCommandType type{ SafetyCommandType::NoAction };
    TrainId trainId{ 0 };

    SpeedMetersPerSecond targetSpeed{ 0.0 };

    TimeSeconds issuedAt{ 0.0 };

    double riskScore{ 0.0 };

    [[nodiscard]]
    bool isEmergency() const noexcept
    {
        return type == SafetyCommandType::EmergencyBrake;
    }
};

} // namespace tcas::safety