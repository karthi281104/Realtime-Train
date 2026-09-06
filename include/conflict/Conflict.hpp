#pragma once

#include "common/Types.hpp"
#include "infrastructure/Node.hpp"

namespace tcas::conflict
{

enum class ConflictType
{
    RearEnd,
    HeadOn,
    Junction,
    Platform
};

struct Conflict
{
    TrainId trainA{ 0 };
    TrainId trainB{ 0 };
    ConflictType type{ ConflictType::RearEnd };

    TrackId trackId{ 0 };
    NodeId resourceNodeId{ 0 };

    TimeSeconds firstConflictTime{ 0.0 };
    TimeSeconds lastConflictTime{ 0.0 };

    DistanceMeters minimumSeparation{ 0.0 };

    [[nodiscard]]
    bool involves(TrainId trainId) const noexcept
    {
        return trainA == trainId || trainB == trainId;
    }
};

struct ConflictDetectionConfig
{
    DistanceMeters minimumTrackSeparation{ 50.0 };
    TimeSeconds resourceTimeSeparation{ 5.0 };
    TimeSeconds resourceClearanceTime{ 2.0 };
};

} // namespace tcas::conflict
