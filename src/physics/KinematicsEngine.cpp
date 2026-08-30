#include "physics/KinematicsEngine.hpp"

#include <algorithm>
#include <cmath>

namespace tcas::physics
{

DistanceMeters KinematicsEngine::updatePosition(
    const DistanceMeters position,
    const SpeedMetersPerSecond velocity,
    const AccelerationMetersPerSecondSquared acceleration,
    const TimeSeconds dt
) noexcept
{
    if (dt <= 0.0)
    {
        return position;
    }

    return position + velocity * dt + 0.5 * acceleration * dt * dt;
}

SpeedMetersPerSecond KinematicsEngine::updateVelocity(
    const SpeedMetersPerSecond velocity,
    const AccelerationMetersPerSecondSquared acceleration,
    const TimeSeconds dt,
    const SpeedMetersPerSecond maximumSpeed
) noexcept
{
    if (dt <= 0.0)
    {
        return velocity;
    }

    const double updated = velocity + acceleration * dt;
    return std::clamp(updated, 0.0, maximumSpeed);
}

AccelerationMetersPerSecondSquared KinematicsEngine::effectiveDeceleration(
    const AccelerationMetersPerSecondSquared nominalDeceleration,
    const double gradient
) noexcept
{
    // Positive gradient (uphill): gravity component assists braking.
    // Negative gradient (downhill): gravity component opposes braking.
    const double gradientCorrection = kGravity * gradient;
    const double effective = nominalDeceleration + gradientCorrection;

    // Clamp to minimum to prevent runaway (zero or negative deceleration).
    return std::max(effective, kMinDeceleration);
}

DistanceMeters KinematicsEngine::brakingDistance(
    const SpeedMetersPerSecond velocity,
    const AccelerationMetersPerSecondSquared effectiveDecel
) noexcept
{
    if (velocity <= 0.0)
    {
        return 0.0;
    }

    const double decel = std::max(effectiveDecel, kMinDeceleration);
    return (velocity * velocity) / (2.0 * decel);
}

DistanceMeters KinematicsEngine::brakingDistance(
    const SpeedMetersPerSecond velocity,
    const AccelerationMetersPerSecondSquared nominalDeceleration,
    const double gradient
) noexcept
{
    const double aEff = effectiveDeceleration(nominalDeceleration, gradient);
    return brakingDistance(velocity, aEff);
}

DistanceMeters KinematicsEngine::reactionDistance(
    const SpeedMetersPerSecond velocity,
    const TimeSeconds reactionTime
) noexcept
{
    if (velocity <= 0.0 || reactionTime <= 0.0)
    {
        return 0.0;
    }

    return velocity * reactionTime;
}

DistanceMeters KinematicsEngine::safeDistance(
    const SpeedMetersPerSecond velocity,
    const AccelerationMetersPerSecondSquared nominalDeceleration,
    const double gradient,
    const TimeSeconds reactionTime,
    const DistanceMeters safetyMargin
) noexcept
{
    const double dReaction = reactionDistance(velocity, reactionTime);
    const double dBraking  = brakingDistance(velocity, nominalDeceleration, gradient);
    const double margin    = std::max(safetyMargin, 0.0);

    return dReaction + dBraking + margin;
}

DistanceMeters KinematicsEngine::emergencyStoppingDistance(
    const SpeedMetersPerSecond velocity,
    const AccelerationMetersPerSecondSquared emergencyDeceleration,
    const double gradient
) noexcept
{
    // Emergency braking: no reaction time component.
    const double aEff = effectiveDeceleration(emergencyDeceleration, gradient);
    return brakingDistance(velocity, aEff);
}

SpeedMetersPerSecond KinematicsEngine::speedAfterDistance(
    const SpeedMetersPerSecond initialVelocity,
    const AccelerationMetersPerSecondSquared acceleration,
    const DistanceMeters distance
) noexcept
{
    if (distance <= 0.0)
    {
        return std::max(initialVelocity, 0.0);
    }

    const double v2 = initialVelocity * initialVelocity
                    + 2.0 * acceleration * distance;

    if (v2 <= 0.0)
    {
        return 0.0;
    }

    return std::sqrt(v2);
}

} // namespace tcas::physics
