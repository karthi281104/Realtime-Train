#pragma once

#include "common/Types.hpp"
#include "conflict/ConflictZone.hpp"

namespace tcas::conflict
{

struct ResourceReservation
{
    TrainId trainId{ 0 };
    ConflictZone zone{};
    TimeSeconds startTime{ 0.0 };
    TimeSeconds endTime{ 0.0 };
    ResourceState state{ ResourceState::Free };
};

} // namespace tcas::conflict
