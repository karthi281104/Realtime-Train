#pragma once

#include "common/Types.hpp"

namespace tcas::prediction
{

// Represents a predicted future state of a train at a specific timestamp.
//
// Position is measured from the beginning of the current track.
// When a prediction crosses a track boundary, position is reset relative
// to the beginning of the newly occupied track.
//
// Uncertainty represents one-sigma position uncertainty in metres.
struct FutureState
{
    TimeSeconds timestamp{ 0.0 };

    TrackId trackId{ 0 };

    DistanceMeters position{ 0.0 };

    SpeedMetersPerSecond velocity{ 0.0 };

    AccelerationMetersPerSecondSquared acceleration{ 0.0 };

    DistanceMeters uncertainty{ 0.0 };
};

} // namespace tcas::prediction