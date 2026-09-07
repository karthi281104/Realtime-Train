#pragma once

#include "common/Types.hpp"
#include "train/Train.hpp"

namespace tcas::safety
{

struct PriorityAssessment
{
    int priority{ 0 };

    TrainType trainType{ TrainType::Freight };

    [[nodiscard]]
    bool higherThan(
        const PriorityAssessment& other
    ) const noexcept;
};

class PriorityEngine
{
public:
    [[nodiscard]]
    PriorityAssessment assess(
        const train::Train& train
    ) const noexcept;

    [[nodiscard]]
    static int basePriority(TrainType type) noexcept;
};

} // namespace tcas::safety