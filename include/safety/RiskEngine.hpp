#pragma once

#include "common/Types.hpp"
#include "conflict/Conflict.hpp"
#include "sensor/SensorData.hpp"
#include "train/Train.hpp"

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
    TimeSeconds timeToCollision{ 0.0 };

    SpeedMetersPerSecond relativeVelocity{ 0.0 };

    DistanceMeters brakingDistance{ 0.0 };

    DistanceMeters safetyMargin{ 0.0 };

    conflict::ConflictType conflictType{
        conflict::ConflictType::RearEnd
    };

    double trainMass{ 0.0 };

    double sensorConfidence{ 1.0 };

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
    RiskAssessment assess(const RiskInput& input) const noexcept;

    [[nodiscard]]
    static RiskLevel classify(double score) noexcept;

private:
    [[nodiscard]]
    static double calculateTtcRisk(TimeSeconds ttc) noexcept;

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
    static double calculateMassRisk(double mass) noexcept;

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