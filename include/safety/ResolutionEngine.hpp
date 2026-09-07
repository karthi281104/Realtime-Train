#pragma once

#include "conflict/Conflict.hpp"
#include "safety/RiskEngine.hpp"
#include "safety/SafetyCommand.hpp"
#include "train/Train.hpp"

namespace tcas::safety
{

struct ResolutionInput
{
    const train::Train& train;

    const conflict::Conflict& conflict;

    const RiskAssessment& risk;

    bool priorityGranted{ false };

    // Caller indication. ResolutionEngine independently validates
    // braking feasibility from the supplied distances.
    bool brakingFeasible{ true };

    DistanceMeters availableDistance{ 0.0 };

    DistanceMeters requiredBrakingDistance{ 0.0 };
};

class ResolutionEngine
{
public:
    [[nodiscard]]
    static SafetyCommand resolve(
        const ResolutionInput& input
    ) noexcept;

private:
    [[nodiscard]]
    static bool calculateBrakingFeasibility(
        const ResolutionInput& input
    ) noexcept;

    [[nodiscard]]
    static SafetyCommand emergencyBrake(
        const ResolutionInput& input
    ) noexcept;

    [[nodiscard]]
    static SafetyCommand reduceSpeed(
        const ResolutionInput& input
    ) noexcept;

    [[nodiscard]]
    static SafetyCommand holdAtSignal(
        const ResolutionInput& input
    ) noexcept;

    [[nodiscard]]
    static SafetyCommand noAction(
        const ResolutionInput& input
    ) noexcept;
};

} // namespace tcas::safety