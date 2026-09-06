#include "safety/ResolutionEngine.hpp"

#include <algorithm>
#include <limits>

namespace tcas::safety
{

SafetyCommand ResolutionEngine::resolve(
    const ResolutionInput& input) const noexcept
{
    if (input.risk.isCritical())
    {
        return emergencyBrake(input);
    }

    if (!input.brakingFeasible)
    {
        return emergencyBrake(input);
    }

    if (!input.priorityGranted)
    {
        if (input.risk.level == RiskLevel::High)
        {
            return holdAtSignal(input);
        }

        if (input.risk.level == RiskLevel::Medium)
        {
            return reduceSpeed(input);
        }

        if (input.risk.level == RiskLevel::Low)
        {
            return holdAtSignal(input);
        }
    }

    if (input.risk.level == RiskLevel::High)
    {
        return reduceSpeed(input);
    }

    if (input.risk.level == RiskLevel::Medium)
    {
        return reduceSpeed(input);
    }

    return noAction(input);
}

SafetyCommand ResolutionEngine::emergencyBrake(
    const ResolutionInput& input) noexcept
{
    SafetyCommand command;

    command.type = SafetyCommandType::EmergencyBrake;
    command.trainId = input.train.id();
    command.targetSpeed = 0.0;
    command.issuedAt = input.conflict.firstConflictTime;
    command.riskScore = input.risk.score;

    return command;
}

SafetyCommand ResolutionEngine::reduceSpeed(
    const ResolutionInput& input) noexcept
{
    SafetyCommand command;

    command.type = SafetyCommandType::ReduceSpeed;
    command.trainId = input.train.id();
    command.issuedAt = input.conflict.firstConflictTime;
    command.riskScore = input.risk.score;

    const double currentSpeed =
        std::max(0.0, input.train.velocity());

    command.targetSpeed = currentSpeed * 0.5;

    return command;
}

SafetyCommand ResolutionEngine::holdAtSignal(
    const ResolutionInput& input) noexcept
{
    SafetyCommand command;

    command.type = SafetyCommandType::HoldAtSignal;
    command.trainId = input.train.id();
    command.targetSpeed = 0.0;
    command.issuedAt = input.conflict.firstConflictTime;
    command.riskScore = input.risk.score;

    return command;
}

SafetyCommand ResolutionEngine::noAction(
    const ResolutionInput& input) noexcept
{
    SafetyCommand command;

    command.type = SafetyCommandType::NoAction;
    command.trainId = input.train.id();
    command.targetSpeed = input.train.velocity();
    command.issuedAt = input.conflict.firstConflictTime;
    command.riskScore = input.risk.score;

    return command;
}

} // namespace tcas::safety