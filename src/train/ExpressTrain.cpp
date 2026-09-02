#include "train/ExpressTrain.hpp"

namespace tcas::train {

ExpressTrain::ExpressTrain(TrainId id, double mass, double maximumSpeed,
                           double serviceBraking, double emergencyBraking)
    : Train(id, mass, maximumSpeed, serviceBraking, emergencyBraking) {}

TrainType ExpressTrain::type() const noexcept { return TrainType::Express; }

}  // namespace tcas::train