#pragma once

#include "common/Types.hpp"

namespace tcas::conflict
{

enum class ConflictZoneType
{
    Junction,
    Platform,
    TrackSection
};

enum class ResourceState
{
    Free,
    Reserved,
    Occupied,
    Released
};

struct ConflictZone
{
    ConflictZoneType type{ ConflictZoneType::TrackSection };
    NodeId nodeId{ 0 };
    TrackId trackId{ 0 };
};

} // namespace tcas::conflict
