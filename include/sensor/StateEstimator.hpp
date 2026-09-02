#pragma once

#include <array>

#include "common/Types.hpp"
#include "sensor/SensorData.hpp"

namespace tcas::sensor {

// 1D Kalman Filter State Estimator for Train Position, Velocity, and
// Acceleration.
//
// Fuses high-frequency noisy/drifting odometry with discrete absolute track
// balise fixes. Features outlier rejection gating and covariance uncertainty
// tracking.
class StateEstimator {
 public:
  explicit StateEstimator(const SensorNoiseConfig& config = {},
                          DistanceMeters initialPosition = 0.0,
                          SpeedMetersPerSecond initialVelocity = 0.0);

  // Predict step: advances state estimation using kinematic transition matrix
  void predict(TimeSeconds dt, SimTimeTick timestamp) noexcept;

  // Measurement update step from wheel tachometer / odometer
  bool updateOdometry(const OdometerMeasurement& measurement) noexcept;

  // Measurement update step from absolute track balise transponder
  bool updateBalise(const BaliseTransponder& balise) noexcept;

  // Current filtered state estimate
  [[nodiscard]]
  EstimatedState estimatedState() const noexcept;

  [[nodiscard]]
  DistanceMeters position() const noexcept;

  [[nodiscard]]
  SpeedMetersPerSecond velocity() const noexcept;

  [[nodiscard]]
  AccelerationMetersPerSecondSquared acceleration() const noexcept;

  [[nodiscard]]
  double positionUncertainty() const noexcept;

  [[nodiscard]]
  double velocityUncertainty() const noexcept;

  [[nodiscard]]
  bool isDegraded() const noexcept;

  // Reset estimator with given initial conditions
  void reset(DistanceMeters initialPosition = 0.0,
             SpeedMetersPerSecond initialVelocity = 0.0) noexcept;

 private:
  SensorNoiseConfig config_;

  // State vector: [0]=position, [1]=velocity, [2]=acceleration
  std::array<double, 3> x_{0.0, 0.0, 0.0};

  // Covariance matrix P (3x3 flattened row-major)
  std::array<double, 9> P_{};

  SimTimeTick timestamp_{0};
  bool isDegraded_{false};
  std::size_t consecutiveOutliers_{0};
};

}  // namespace tcas::sensor
