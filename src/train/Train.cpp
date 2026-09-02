#include "train/Train.hpp"

#include <stdexcept>

namespace tcas::train {

Train::Train(TrainId id, double mass, double maximumSpeed,
             double serviceBraking, double emergencyBraking)
    : id_(id),
      mass_(mass),
      maximumSpeed_(maximumSpeed),
      serviceBraking_(serviceBraking),
      emergencyBraking_(emergencyBraking),
      position_(0.0),
      velocity_(0.0),
      acceleration_(0.0),
      state_(TrainState::Idle) {
  if (mass <= 0.0) {
    throw std::invalid_argument("Train mass must be greater than zero");
  }

  if (maximumSpeed < 0.0) {
    throw std::invalid_argument("Train maximum speed cannot be negative");
  }

  if (serviceBraking < 0.0) {
    throw std::invalid_argument("Service braking cannot be negative");
  }

  if (emergencyBraking < 0.0) {
    throw std::invalid_argument("Emergency braking cannot be negative");
  }

  if (emergencyBraking < serviceBraking) {
    throw std::invalid_argument(
        "Emergency braking must be greater than or equal to service braking");
  }
}

TrainId Train::id() const noexcept { return id_; }

double Train::mass() const noexcept { return mass_; }

double Train::maximumSpeed() const noexcept { return maximumSpeed_; }

double Train::serviceBraking() const noexcept { return serviceBraking_; }

double Train::emergencyBraking() const noexcept { return emergencyBraking_; }

double Train::position() const noexcept { return position_; }

double Train::velocity() const noexcept { return velocity_; }

double Train::acceleration() const noexcept { return acceleration_; }

TrainState Train::state() const noexcept { return state_; }

void Train::setPosition(double position) {
  if (position < 0.0) {
    throw std::invalid_argument("Train position cannot be negative");
  }

  position_ = position;
}

void Train::setVelocity(double velocity) {
  if (velocity < 0.0) {
    throw std::invalid_argument("Train velocity cannot be negative");
  }

  if (velocity > maximumSpeed_) {
    throw std::invalid_argument("Train velocity cannot exceed maximum speed");
  }

  velocity_ = velocity;
}

void Train::setAcceleration(double acceleration) {
  acceleration_ = acceleration;
}

void Train::setState(TrainState state) noexcept { state_ = state; }

}  // namespace tcas::train