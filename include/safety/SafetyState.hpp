#pragma once

namespace tcas::safety
{

enum class SafetyState
{
    Normal,
    Caution,
    Warning,
    Critical,
    EmergencyBrake,
    Degraded
};

} // namespace tcas::safety