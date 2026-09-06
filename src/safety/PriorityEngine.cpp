#include "safety/PriorityEngine.hpp"

namespace tcas::safety
{

bool PriorityAssessment::higherThan(
    const PriorityAssessment& other) const noexcept
{
    return priority > other.priority;
}

PriorityAssessment PriorityEngine::assess(
    const train::Train& train) const noexcept
{
    PriorityAssessment result;

    result.trainType = train.type();
    result.priority = basePriority(result.trainType);

    return result;
}

int PriorityEngine::basePriority(TrainType type) noexcept
{
    switch (type)
    {
    case TrainType::Express:
        return 3;

    case TrainType::Passenger:
        return 2;

    case TrainType::Freight:
        return 1;
    }

    return 0;
}

} // namespace tcas::safety