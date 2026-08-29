#include "train/TrainManager.hpp"

namespace tcas::train
{

bool TrainManager::addTrain(
    std::unique_ptr<Train> train
)
{
    if (!train)
    {
        return false;
    }

    const TrainId id = train->id();

    const auto [iterator, inserted] =
        trains_.emplace(id, std::move(train));

    return inserted;
}

bool TrainManager::removeTrain(TrainId id)
{
    return trains_.erase(id) > 0;
}

Train* TrainManager::getTrain(TrainId id) noexcept
{
    const auto iterator = trains_.find(id);

    if (iterator == trains_.end())
    {
        return nullptr;
    }

    return iterator->second.get();
}

const Train* TrainManager::getTrain(
    TrainId id
) const noexcept
{
    const auto iterator = trains_.find(id);

    if (iterator == trains_.end())
    {
        return nullptr;
    }

    return iterator->second.get();
}

bool TrainManager::contains(TrainId id) const noexcept
{
    return trains_.contains(id);
}

std::size_t TrainManager::trainCount() const noexcept
{
    return trains_.size();
}

void TrainManager::clear() noexcept
{
    trains_.clear();
}

} // namespace tcas::train