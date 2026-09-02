#pragma once

#include "common/Types.hpp"
#include "sensor/SensorData.hpp"

namespace tcas::sensor {

// Simulates on-board train wheel tachometer / odometer.
//
// Models sensor drift (accumulated error from wheel slip/slide)
// and allows calibration / reset upon passing physical track balises.
class Odometer {
 public:
  explicit Odometer(const SensorNoiseConfig& config = {});

  // Generate a sensor measurement from ground-truth kinematics
  [[nodiscard]]
  OdometerMeasurement measure(
      DistanceMeters truePosition, SpeedMetersPerSecond trueVelocity,
      AccelerationMetersPerSecondSquared trueAcceleration, TimeSeconds dt,
      SimTimeTick timestamp);

  // Calibrate odometer against an absolute position beacon (balise)
  void calibrate(DistanceMeters exactPosition) noexcept;

  // Inject / clear sensor hardware fault state
  void setFaulty(bool faulty) noexcept;

  [[nodiscard]]
  bool isFaulty() const noexcept;

  [[nodiscard]]
  DistanceMeters accumulatedDrift() const noexcept;

  // Reset odometer state to zero
  void reset() noexcept;

 private:
  SensorNoiseConfig config_;
  DistanceMeters accumulatedDrift_{0.0};
  bool isFaulty_{false};
};

}  // namespace tcas::sensor
