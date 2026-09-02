#include "train/PassengerTrain.hpp"

namespace tcas::train {

PassengerTrain::PassengerTrain(TrainId id, double mass, double maximumSpeed,
                               double serviceBraking, double emergencyBraking)
    : Train(id, mass, maximumSpeed, serviceBraking, emergencyBraking) {}

TrainType PassengerTrain::type() const noexcept { return TrainType::Passenger; }

}  // namespace tcas::train