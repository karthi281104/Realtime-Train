#pragma once

#include "train/Train.hpp"

namespace tcas::train
{

class ExpressTrain final : public Train
{
public:
    ExpressTrain(
        TrainId id,
        double mass,
        double maximumSpeed,
        double serviceBraking,
        double emergencyBraking
    );

    [[nodiscard]]
    TrainType type() const noexcept override;
};

} // namespace tcas::train