#include "train/FreightTrain.hpp"

namespace tcas::train {

FreightTrain::FreightTrain(TrainId id, double mass, double maximumSpeed,
                           double serviceBraking, double emergencyBraking)
    : Train(id, mass, maximumSpeed, serviceBraking, emergencyBraking) {}

TrainType FreightTrain::type() const noexcept { return TrainType::Freight; }

}  // namespace tcas::train