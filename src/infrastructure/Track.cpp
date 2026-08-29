#include "infrastructure/Track.hpp"

namespace tcas::infrastructure
{

Track::Track(
    TrackId id,
    NodeId source,
    NodeId destination,
    DistanceMeters length,
    SpeedMetersPerSecond speedLimit,
    double gradient
)
    : id_(id),
      source_(source),
      destination_(destination),
      length_(length),
      speedLimit_(speedLimit),
      gradient_(gradient)
{
}

TrackId Track::id() const noexcept
{
    return id_;
}

NodeId Track::source() const noexcept
{
    return source_;
}

NodeId Track::destination() const noexcept
{
    return destination_;
}

DistanceMeters Track::length() const noexcept
{
    return length_;
}

SpeedMetersPerSecond Track::speedLimit() const noexcept
{
    return speedLimit_;
}

double Track::gradient() const noexcept
{
    return gradient_;
}

} // namespace tcas::infrastructure