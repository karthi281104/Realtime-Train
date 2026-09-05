#pragma once

#include "common/Types.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "navigation/RouteResult.hpp"
#include "prediction/FutureState.hpp"
#include "train/Train.hpp"

#include <array>
#include <vector>

namespace tcas::prediction
{

// Predicts the future trajectory of a train over its planned route.
//
// The prediction engine:
//   - predicts position at requested future horizons,
//   - follows the ordered route across multiple tracks,
//   - respects track speed limits,
//   - accounts for track gradients,
//   - uses the existing KinematicsEngine,
//   - stops at the end of the route,
//   - propagates position uncertainty.
//
// Position supplied by the train is interpreted as metres from the beginning
// of currentTrackId.
class PredictionEngine
{
public:
    PredictionEngine() = default;

    // Standard prediction horizons required by the TCAS specification.
    [[nodiscard]]
    static std::vector<FutureState> predictStandardHorizon(
        const train::Train& train,
        const infrastructure::RailwayNetwork& network,
        const navigation::RouteResult& route,
        TrackId currentTrackId,
        DistanceMeters initialUncertainty = 0.0
    );

    // Predict the train at arbitrary future horizons.
    //
    // Horizons must be finite and non-negative.
    //
    // currentTrackId must occur in route.tracks.
    [[nodiscard]]
    static std::vector<FutureState> predict(
        const train::Train& train,
        const infrastructure::RailwayNetwork& network,
        const navigation::RouteResult& route,
        TrackId currentTrackId,
        const std::vector<TimeSeconds>& horizons,
        DistanceMeters initialUncertainty = 0.0
    );

private:
    struct PredictionPoint
    {
        TrackId trackId{ 0 };
        DistanceMeters position{ 0.0 };
        SpeedMetersPerSecond velocity{ 0.0 };
        AccelerationMetersPerSecondSquared acceleration{ 0.0 };
        DistanceMeters uncertainty{ 0.0 };
    };

    [[nodiscard]]
    static PredictionPoint predictAtTime(
        const train::Train& train,
        const infrastructure::RailwayNetwork& network,
        const navigation::RouteResult& route,
        std::size_t currentRouteIndex,
        DistanceMeters initialPosition,
        DistanceMeters initialUncertainty,
        TimeSeconds horizon
    );

    [[nodiscard]]
    static void validateRouteConnectivity(
        const infrastructure::RailwayNetwork& network,
        const navigation::RouteResult& route
    );

    [[nodiscard]]
    static DistanceMeters propagateUncertainty(
        DistanceMeters initialUncertainty,
        TimeSeconds elapsed
    );

    [[nodiscard]]
    static TimeSeconds calculateTimeToBoundary(
        DistanceMeters distanceToBoundary,
        SpeedMetersPerSecond velocity,
        AccelerationMetersPerSecondSquared acceleration
    ) noexcept;

    [[nodiscard]]
    static bool isFiniteNonNegative(double value) noexcept;
};

} // namespace tcas::prediction