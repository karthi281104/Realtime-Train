#include "infrastructure/Track.hpp"

#include <stdexcept>

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
    if (length_ <= 0.0)
    {
        throw std::invalid_argument("Track length must be positive");
    }

    if (speedLimit_ < 0.0)
    {
        throw std::invalid_argument("Track speed limit cannot be negative");
    }
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