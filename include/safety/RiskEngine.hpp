#pragma once

#include "common/Types.hpp"
#include "conflict/Conflict.hpp"

namespace tcas::safety
{

enum class RiskLevel
{
    Low,
    Medium,
    High,
    Critical
};

struct RiskInput
{
    // Time from the current simulation time until the predicted conflict.
    TimeSeconds timeToCollision{ 0.0 };

    // Positive value represents closing speed.
    SpeedMetersPerSecond relativeVelocity{ 0.0 };

    // Distance required to stop using the applicable braking model.
    DistanceMeters brakingDistance{ 0.0 };

    // Remaining clearance after accounting for braking distance.
    // Positive = remaining safety clearance.
    // Zero/negative = insufficient clearance.
    DistanceMeters safetyMargin{ 0.0 };

    conflict::ConflictType conflictType{
        conflict::ConflictType::RearEnd
    };

    // Mass of the train for which this risk assessment is being made.
    double trainMass{ 0.0 };

    // [0, 1], where 1 = fully trusted.
    double sensorConfidence{ 1.0 };

    // [0, 1], where 1 = fully trusted.
    double communicationConfidence{ 1.0 };
};

struct RiskAssessment
{
    double score{ 0.0 };

    RiskLevel level{ RiskLevel::Low };

    TimeSeconds timeToCollision{ 0.0 };

    DistanceMeters brakingDistance{ 0.0 };

    DistanceMeters safetyMargin{ 0.0 };

    [[nodiscard]]
    bool isActionRequired() const noexcept;

    [[nodiscard]]
    bool isCritical() const noexcept;
};

class RiskEngine
{
public:
    [[nodiscard]]
    static RiskAssessment assess(
        const RiskInput& input
    ) noexcept;

    [[nodiscard]]
    static RiskLevel classify(double score) noexcept;

private:
    [[nodiscard]]
    static double calculateTtcRisk(
        TimeSeconds ttc
    ) noexcept;

    [[nodiscard]]
    static double calculateRelativeVelocityRisk(
        SpeedMetersPerSecond relativeVelocity
    ) noexcept;

    [[nodiscard]]
    static double calculateBrakingRisk(
        DistanceMeters brakingDistance,
        DistanceMeters safetyMargin
    ) noexcept;

    [[nodiscard]]
    static double calculateConflictTypeRisk(
        conflict::ConflictType type
    ) noexcept;

    [[nodiscard]]
    static double calculateMassRisk(
        double mass
    ) noexcept;

    [[nodiscard]]
    static double calculateSensorRisk(
        double sensorConfidence
    ) noexcept;

    [[nodiscard]]
    static double calculateCommunicationRisk(
        double communicationConfidence
    ) noexcept;
};

} // namespace tcas::safety