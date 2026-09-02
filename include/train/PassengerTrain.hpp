#pragma once

#include "train/Train.hpp"

namespace tcas::train {

class PassengerTrain final : public Train {
 public:
  PassengerTrain(TrainId id, double mass, double maximumSpeed,
                 double serviceBraking, double emergencyBraking);

  [[nodiscard]]
  TrainType type() const noexcept override;
};

}  // namespace tcas::train