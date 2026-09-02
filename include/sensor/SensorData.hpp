#pragma once

#include "common/Types.hpp"

#include <cstdint>

namespace tcas::sensor
{

// Raw sensor measurement from tachometer/odometer/IMU
struct OdometerMeasurement
{
    DistanceMeters                    rawPosition{ 0.0 };
    SpeedMetersPerSecond              rawVelocity{ 0.0 };
    AccelerationMetersPerSecondSquared rawAcceleration{ 0.0 };
    SimTimeTick                       timestamp{ 0 };
    bool                              isValid{ true };
};

// Fixed track balise / transponder providing absolute position anchor
struct BaliseTransponder
{
    std::uint32_t   baliseId{ 0 };
    TrackId         trackId{ 0 };
    DistanceMeters  exactPosition{ 0.0 };
};

// Filtered / estimated state of a train with uncertainty metrics
struct EstimatedState
{
    DistanceMeters                    position{ 0.0 };
    SpeedMetersPerSecond              velocity{ 0.0 };
    AccelerationMetersPerSecondSquared acceleration{ 0.0 };
    double                            positionUncertainty{ 1.0 }; // std dev (m)
    double                            velocityUncertainty{ 0.5 }; // std dev (m/s)
    SimTimeTick                       timestamp{ 0 };
    bool                              isDegraded{ false };
};

// Configuration for Kalman Filter and Sensor Noise Model
struct SensorNoiseConfig
{
    double processNoisePos{ 0.1 };       // Q_pos (m^2/s)
    double processNoiseVel{ 0.2 };       // Q_vel ((m/s)^2/s)
    double processNoiseAcc{ 0.5 };       // Q_acc ((m/s^2)^2/s)

    double measurementNoisePos{ 2.0 };   // R_pos (m^2)
    double measurementNoiseVel{ 0.5 };   // R_vel ((m/s)^2)
    double measurementNoiseBalise{ 0.01 }; // R_balise (m^2)

    double driftRatePerSecond{ 0.01 };   // 1% wheel slip drift
    double outlierGateSigma{ 3.5 };      // 3.5-sigma innovation rejection threshold
};

} // namespace tcas::sensor
