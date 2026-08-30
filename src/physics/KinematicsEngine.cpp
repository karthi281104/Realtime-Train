#include "physics/KinematicsEngine.hpp"

#include <algorithm>
#include <cmath>

namespace tcas::physics
{

double KinematicsEngine::updatePosition(
    double position,
    double velocity,
    double acceleration,
    double dt
) noexcept
{
    if (dt <= 0.0)
    {
        return position;
    }

    if (velocity < 0.0)
    {
        velocity = 0.0;
    }

    // No motion.
    if (velocity == 0.0)
    {
        return position;
    }

    // If the train is decelerating, check whether it stops
    // during this timestep.
    if (acceleration < 0.0)
    {
        const double stoppingTime = -velocity / acceleration;

        if (stoppingTime <= dt)
        {
            // Train reaches zero velocity before the timestep ends.
            // Move only until it stops.
            return position
                + velocity * stoppingTime
                + 0.5 * acceleration * stoppingTime * stoppingTime;
        }
    }

    // Normal kinematic update.
    return position
        + velocity * dt
        + 0.5 * acceleration * dt * dt;
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

double KinematicsEngine::brakingDistance(
    double velocity,
    double deceleration,
    double gradient
) noexcept
{
    if (velocity <= 0.0)
    {
        return 0.0;
    }

    const double effective =
        effectiveDeceleration(deceleration, gradient);

    if (effective <= 0.0)
    {
        return 0.0;
    }

    return (velocity * velocity) / (2.0 * effective);
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

double KinematicsEngine::speedAfterDistance(
    double initialVelocity,
    double acceleration,
    double distance
) noexcept
{
    // Negative initial velocity is invalid.
    if (initialVelocity < 0.0)
    {
        return 0.0;
    }

    // No distance travelled -> velocity unchanged.
    if (distance <= 0.0)
    {
        return initialVelocity;
    }

    // v² = u² + 2as
    const double velocitySquared =
        initialVelocity * initialVelocity
        + 2.0 * acceleration * distance;

    // The train has decelerated to zero before
    // travelling the requested distance.
    if (velocitySquared <= 0.0)
    {
        return 0.0;
    }

    return std::sqrt(velocitySquared);
}

} // namespace tcas::physics
