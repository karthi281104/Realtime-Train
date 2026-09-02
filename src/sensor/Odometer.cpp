#include "sensor/Odometer.hpp"

#include <algorithm>
#include <cmath>

namespace tcas::sensor
{

Odometer::Odometer(const SensorNoiseConfig& config)
    : config_(config),
      accumulatedDrift_(0.0),
      isFaulty_(false)
{
}

OdometerMeasurement Odometer::measure(
    const DistanceMeters truePosition,
    const SpeedMetersPerSecond trueVelocity,
    const AccelerationMetersPerSecondSquared trueAcceleration,
    const TimeSeconds dt,
    const SimTimeTick timestamp
)
{
    if (isFaulty_)
    {
        return OdometerMeasurement{
            .rawPosition     = truePosition + 9999.0, // Erroneous reading
            .rawVelocity     = 0.0,
            .rawAcceleration = 0.0,
            .timestamp       = timestamp,
            .isValid         = false
        };
    }

    if (dt > 0.0 && trueVelocity > 0.0)
    {
        // Drift accumulates proportionally to velocity and time elapsed
        const double driftDelta = trueVelocity * config_.driftRatePerSecond * dt;
        accumulatedDrift_ += driftDelta;
    }

    const double measuredPos = truePosition + accumulatedDrift_;
    const double measuredVel = trueVelocity;
    const double measuredAcc = trueAcceleration;

    return OdometerMeasurement{
        .rawPosition     = std::max(0.0, measuredPos),
        .rawVelocity     = std::max(0.0, measuredVel),
        .rawAcceleration = measuredAcc,
        .timestamp       = timestamp,
        .isValid         = true
    };
}

void Odometer::calibrate(const DistanceMeters exactPosition) noexcept
{
    (void)exactPosition;
    // Balise reading resets accumulated wheel slip drift back to zero
    accumulatedDrift_ = 0.0;
}

void Odometer::setFaulty(const bool faulty) noexcept
{
    isFaulty_ = faulty;
}

bool Odometer::isFaulty() const noexcept
{
    return isFaulty_;
}

DistanceMeters Odometer::accumulatedDrift() const noexcept
{
    return accumulatedDrift_;
}

void Odometer::reset() noexcept
{
    accumulatedDrift_ = 0.0;
    isFaulty_ = false;
}

} // namespace tcas::sensor
