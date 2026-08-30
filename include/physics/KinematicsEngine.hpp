#pragma once

#include "common/Types.hpp"

namespace tcas::physics
{

// Pure kinematics computation engine for train motion.
//
// All methods are static — this class has no instance state and
// no side effects. It operates purely on the provided parameters.
//
// Gradient convention: positive = uphill, negative = downhill.
// Gradient is expressed as a dimensionless ratio (e.g. 0.01 = 1%).
// This matches the Track::gradient() return value.
//
// Standard gravity: g = 9.80665 m/s^2 (ISO 80000-3).
//
// Reaction time default: 1.5 s (typical train driver reaction time).
// Safety margin default: 50.0 m (minimum clearance buffer).
//
// Thread safety: all methods are pure functions — fully thread-safe.
class KinematicsEngine
{
public:
    KinematicsEngine() = delete;

    // ============================================================
    // Constants
    // ============================================================

    static constexpr double kGravity        = 9.80665;  // m/s^2
    static constexpr double kDefaultReactionTime = 1.5; // seconds
    static constexpr double kDefaultSafetyMargin = 50.0; // metres
    static constexpr double kMinDeceleration = 0.01;    // m/s^2 floor

    // ============================================================
    // Position and Velocity
    // ============================================================

    // Compute updated position using kinematic equation:
    //   x = x0 + v0 * dt + 0.5 * a * dt^2
    // Returns x0 unchanged when dt <= 0.
    [[nodiscard]]
    static DistanceMeters updatePosition(
        DistanceMeters position,
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared acceleration,
        TimeSeconds dt
    ) noexcept;

    // Compute updated velocity, clamped to [0, maximumSpeed]:
    //   v = v0 + a * dt
    // Returns velocity unchanged when dt <= 0.
    [[nodiscard]]
    static SpeedMetersPerSecond updateVelocity(
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared acceleration,
        TimeSeconds dt,
        SpeedMetersPerSecond maximumSpeed
    ) noexcept;

    // ============================================================
    // Gradient-Corrected Effective Deceleration
    // ============================================================

    // Compute the effective deceleration accounting for track gradient.
    //
    // Positive gradient (uphill): gravity assists braking — effective
    //   deceleration is higher than nominal.
    // Negative gradient (downhill): gravity opposes braking — effective
    //   deceleration is lower than nominal.
    //
    //   a_eff = nominalDeceleration + g * gradient
    //
    // Result is clamped to [kMinDeceleration, ...] to prevent negative
    // or zero deceleration (runaway condition is guarded).
    [[nodiscard]]
    static AccelerationMetersPerSecondSquared effectiveDeceleration(
        AccelerationMetersPerSecondSquared nominalDeceleration,
        double gradient
    ) noexcept;

    // ============================================================
    // Braking Distance
    // ============================================================

    // Minimum distance required to stop from the given velocity
    // using the given effective (gradient-corrected) deceleration:
    //   d = v^2 / (2 * a_eff)
    //
    // Returns 0 when velocity <= 0.
    [[nodiscard]]
    static DistanceMeters brakingDistance(
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared effectiveDecel
    ) noexcept;

    // Braking distance using nominal deceleration and gradient correction.
    // Convenience overload that calls effectiveDeceleration() internally.
    [[nodiscard]]
    static DistanceMeters brakingDistance(
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared nominalDeceleration,
        double gradient
    ) noexcept;

    // ============================================================
    // Reaction Distance
    // ============================================================

    // Distance covered during driver reaction time before braking begins:
    //   d_r = velocity * reactionTime
    //
    // Returns 0 when velocity <= 0 or reactionTime <= 0.
    [[nodiscard]]
    static DistanceMeters reactionDistance(
        SpeedMetersPerSecond velocity,
        TimeSeconds reactionTime = kDefaultReactionTime
    ) noexcept;

    // ============================================================
    // Safe Distance
    // ============================================================

    // Total safe stopping distance:
    //   d_safe = d_reaction + d_braking + safetyMargin
    //
    // This is the minimum distance a train travelling at the given
    // velocity must maintain ahead of a stationary obstacle to
    // guarantee a safe stop under service braking with gradient
    // correction and driver reaction time.
    [[nodiscard]]
    static DistanceMeters safeDistance(
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared nominalDeceleration,
        double gradient,
        TimeSeconds reactionTime = kDefaultReactionTime,
        DistanceMeters safetyMargin = kDefaultSafetyMargin
    ) noexcept;

    // ============================================================
    // Emergency Stopping Distance
    // ============================================================

    // Minimum distance to stop using emergency braking (no reaction time):
    //   d_emergency = v^2 / (2 * a_emergency_eff)
    [[nodiscard]]
    static DistanceMeters emergencyStoppingDistance(
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared emergencyDeceleration,
        double gradient
    ) noexcept;

    // ============================================================
    // Speed-Distance Relationship
    // ============================================================

    // Compute the speed a train would have after travelling distance d
    // from an initial speed v0, under constant acceleration a:
    //   v^2 = v0^2 + 2 * a * d
    //
    // Returns 0 when the result would be imaginary (train stops before d).
    [[nodiscard]]
    static SpeedMetersPerSecond speedAfterDistance(
        SpeedMetersPerSecond initialVelocity,
        AccelerationMetersPerSecondSquared acceleration,
        DistanceMeters distance
    ) noexcept;
};

} // namespace tcas::physics
