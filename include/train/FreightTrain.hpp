#pragma once

#include "train/Train.hpp"

namespace tcas::train {

class FreightTrain final : public Train {
 public:
  FreightTrain(TrainId id, double mass, double maximumSpeed,
               double serviceBraking, double emergencyBraking);

  [[nodiscard]]
  TrainType type() const noexcept override;
};

}  // namespace tcas::train