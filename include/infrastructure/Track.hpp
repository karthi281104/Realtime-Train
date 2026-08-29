#pragma once

#include "common/Types.hpp"

namespace tcas::infrastructure
{

class Track
{
public:
    Track(
        TrackId id,
        NodeId source,
        NodeId destination,
        DistanceMeters length,
        SpeedMetersPerSecond speedLimit,
        double gradient
    );

    [[nodiscard]]
    TrackId id() const noexcept;

    [[nodiscard]]
    NodeId source() const noexcept;

    [[nodiscard]]
    NodeId destination() const noexcept;

    [[nodiscard]]
    DistanceMeters length() const noexcept;

    [[nodiscard]]
    SpeedMetersPerSecond speedLimit() const noexcept;

    [[nodiscard]]
    double gradient() const noexcept;

private:
    TrackId id_;
    NodeId source_;
    NodeId destination_;
    DistanceMeters length_;
    SpeedMetersPerSecond speedLimit_;
    double gradient_;
};

} // namespace tcas::infrastructure