#include "sensor/StateEstimator.hpp"

#include <algorithm>
#include <cmath>

namespace tcas::sensor
{

StateEstimator::StateEstimator(
    const SensorNoiseConfig& config,
    const DistanceMeters initialPosition,
    const SpeedMetersPerSecond initialVelocity
)
    : config_(config),
      x_{ initialPosition, initialVelocity, 0.0 },
      P_{ 1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0 },
      timestamp_(0),
      isDegraded_(false),
      consecutiveOutliers_(0)
{
}

void StateEstimator::predict(const TimeSeconds dt, const SimTimeTick timestamp) noexcept
{
    timestamp_ = timestamp;

    if (dt <= 0.0)
    {
        return;
    }

    const double dt2 = 0.5 * dt * dt;

    // State transition:
    // p_new = p + v*dt + 0.5*a*dt^2
    // v_new = v + a*dt
    // a_new = a
    const double pNew = x_[0] + x_[1] * dt + x_[2] * dt2;
    const double vNew = x_[1] + x_[2] * dt;
    const double aNew = x_[2];

    x_[0] = std::max(0.0, pNew);
    x_[1] = std::max(0.0, vNew);
    x_[2] = aNew;

    // F matrix = [1, dt, dt2]
    //            [0,  1,  dt]
    //            [0,  0,   1]
    // Compute P_new = F * P * F^T + Q * dt
    const double p00 = P_[0], p01 = P_[1], p02 = P_[2];
    const double p10 = P_[3], p11 = P_[4], p12 = P_[5];
    const double p20 = P_[6], p21 = P_[7], p22 = P_[8];

    // FP = F * P
    const double fp00 = p00 + dt * p10 + dt2 * p20;
    const double fp01 = p01 + dt * p11 + dt2 * p21;
    const double fp02 = p02 + dt * p12 + dt2 * p22;

    const double fp10 = p10 + dt * p20;
    const double fp11 = p11 + dt * p21;
    const double fp12 = p12 + dt * p22;

    const double fp20 = p20;
    const double fp21 = p21;
    const double fp22 = p22;

    // (FP) * F^T
    P_[0] = fp00 + dt * fp01 + dt2 * fp02 + config_.processNoisePos * dt;
    P_[1] = fp01 + dt * fp02;
    P_[2] = fp02;

    P_[3] = fp10 + dt * fp11 + dt2 * fp12;
    P_[4] = fp11 + dt * fp12 + config_.processNoiseVel * dt;
    P_[5] = fp12;

    P_[6] = fp20 + dt * fp21 + dt2 * fp22;
    P_[7] = fp21 + dt * fp22;
    P_[8] = fp22 + config_.processNoiseAcc * dt;
}

bool StateEstimator::updateOdometry(const OdometerMeasurement& measurement) noexcept
{
    if (!measurement.isValid)
    {
        isDegraded_ = true;
        return false;
    }

    // Measurement residual (innovation)
    const double residualPos = measurement.rawPosition - x_[0];

    // Innovation covariance: S_pos = P[0][0] + R_pos, S_vel = P[1][1] + R_vel
    const double sPos = P_[0] + config_.measurementNoisePos;
    const double sVel = P_[4] + config_.measurementNoiseVel;

    if (sPos <= 0.0 || sVel <= 0.0)
    {
        return false;
    }

    // Statistical outlier rejection check (normalized innovation squared)
    const double normResidualPos = (residualPos * residualPos) / sPos;
    const double gateThreshold = config_.outlierGateSigma * config_.outlierGateSigma;

    if (normResidualPos > gateThreshold)
    {
        ++consecutiveOutliers_;
        if (consecutiveOutliers_ >= 3)
        {
            isDegraded_ = true;
        }
        return false;
    }

    consecutiveOutliers_ = 0;
    isDegraded_ = false;

    // Kalman gains for position measurement
    const double kPos0 = P_[0] / sPos;
    const double kPos1 = P_[3] / sPos;
    const double kPos2 = P_[6] / sPos;

    // Update with position
    x_[0] += kPos0 * residualPos;
    x_[1] += kPos1 * residualPos;
    x_[2] += kPos2 * residualPos;

    // Covariance update (Joseph form or standard (I - K*H)*P)
    P_[0] = std::max(1e-6, (1.0 - kPos0) * P_[0]);
    P_[4] = std::max(1e-6, P_[4] - kPos1 * P_[1]);
    P_[8] = std::max(1e-6, P_[8] - kPos2 * P_[2]);

    // Kalman gains for velocity measurement
    const double kVel0 = P_[1] / sVel;
    const double kVel1 = P_[4] / sVel;
    const double kVel2 = P_[7] / sVel;

    const double updatedResidualVel = measurement.rawVelocity - x_[1];
    x_[0] += kVel0 * updatedResidualVel;
    x_[1] += kVel1 * updatedResidualVel;
    x_[2] += kVel2 * updatedResidualVel;

    P_[4] = std::max(1e-6, (1.0 - kVel1) * P_[4]);

    // Ensure state non-negativity for position and velocity
    x_[0] = std::max(0.0, x_[0]);
    x_[1] = std::max(0.0, x_[1]);

    return true;
}

bool StateEstimator::updateBalise(const BaliseTransponder& balise) noexcept
{
    // Absolute position anchor update with very low measurement noise
    const double residual = balise.exactPosition - x_[0];
    const double s = P_[0] + config_.measurementNoiseBalise;

    if (s <= 0.0)
    {
        return false;
    }

    const double k0 = P_[0] / s;
    const double k1 = P_[3] / s;
    const double k2 = P_[6] / s;

    x_[0] += k0 * residual;
    x_[1] += k1 * residual;
    x_[2] += k2 * residual;

    // Drastically lowers position uncertainty
    P_[0] = std::max(1e-6, (1.0 - k0) * P_[0]);
    P_[4] = std::max(1e-6, P_[4] - k1 * P_[1]);

    x_[0] = std::max(0.0, x_[0]);
    x_[1] = std::max(0.0, x_[1]);

    isDegraded_ = false;
    consecutiveOutliers_ = 0;

    return true;
}

EstimatedState StateEstimator::estimatedState() const noexcept
{
    return EstimatedState{
        .position            = x_[0],
        .velocity            = x_[1],
        .acceleration        = x_[2],
        .positionUncertainty = std::sqrt(std::max(0.0, P_[0])),
        .velocityUncertainty = std::sqrt(std::max(0.0, P_[4])),
        .timestamp           = timestamp_,
        .isDegraded          = isDegraded_
    };
}

DistanceMeters StateEstimator::position() const noexcept
{
    return x_[0];
}

SpeedMetersPerSecond StateEstimator::velocity() const noexcept
{
    return x_[1];
}

AccelerationMetersPerSecondSquared StateEstimator::acceleration() const noexcept
{
    return x_[2];
}

double StateEstimator::positionUncertainty() const noexcept
{
    return std::sqrt(std::max(0.0, P_[0]));
}

double StateEstimator::velocityUncertainty() const noexcept
{
    return std::sqrt(std::max(0.0, P_[4]));
}

bool StateEstimator::isDegraded() const noexcept
{
    return isDegraded_;
}

void StateEstimator::reset(
    const DistanceMeters initialPosition,
    const SpeedMetersPerSecond initialVelocity
) noexcept
{
    x_ = { initialPosition, initialVelocity, 0.0 };
    P_ = { 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0 };
    timestamp_ = 0;
    isDegraded_ = false;
    consecutiveOutliers_ = 0;
}

} // namespace tcas::sensor
