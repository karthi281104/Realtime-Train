#pragma once

#include "common/Types.hpp"

namespace tcas::train {

class Train {
 public:
  Train(TrainId id, double mass, double maximumSpeed, double serviceBraking,
        double emergencyBraking);

  virtual ~Train() = default;

  [[nodiscard]]
  TrainId id() const noexcept;

  [[nodiscard]]
  virtual TrainType type() const noexcept = 0;

  [[nodiscard]]
  double mass() const noexcept;

  [[nodiscard]]
  double maximumSpeed() const noexcept;

  [[nodiscard]]
  double serviceBraking() const noexcept;

  [[nodiscard]]
  double emergencyBraking() const noexcept;

  [[nodiscard]]
  double position() const noexcept;

  [[nodiscard]]
  double velocity() const noexcept;

  [[nodiscard]]
  double acceleration() const noexcept;

  [[nodiscard]]
  TrainState state() const noexcept;

  void setPosition(double position);

  void setVelocity(double velocity);

  void setAcceleration(double acceleration);

  void setState(TrainState state) noexcept;

 private:
  TrainId id_;

  double mass_;
  double maximumSpeed_;
  double serviceBraking_;
  double emergencyBraking_;

  double position_;
  double velocity_;
  double acceleration_;

  TrainState state_;
};

}  // namespace tcas::train