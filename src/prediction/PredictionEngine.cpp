#include "prediction/PredictionEngine.hpp"

#include "physics/KinematicsEngine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tcas::prediction
{

namespace
{

constexpr TimeSeconds kUncertaintyGrowthRate = 0.5;

// Find the index of currentTrackId in the ordered route.
[[nodiscard]]
std::size_t findCurrentTrackIndex(
    const navigation::RouteResult& route,
    TrackId currentTrackId
)
{
    const auto iterator = std::find(
        route.tracks.begin(),
        route.tracks.end(),
        currentTrackId
    );

    if (iterator == route.tracks.end())
    {
        throw std::invalid_argument(
            "Current track is not present in the route"
        );
    }

    return static_cast<std::size_t>(
        std::distance(route.tracks.begin(), iterator)
    );
}

} // namespace

bool PredictionEngine::isFiniteNonNegative(double value) noexcept
{
    return std::isfinite(value) && value >= 0.0;
}

void PredictionEngine::validateRouteConnectivity(
    const infrastructure::RailwayNetwork& network,
    const navigation::RouteResult& route
)
{
    for (std::size_t i = 0; i + 1 < route.tracks.size(); ++i)
    {
        const auto* currentTrack = network.getTrack(route.tracks[i]);
        const auto* nextTrack = network.getTrack(route.tracks[i + 1]);

        if (currentTrack == nullptr || nextTrack == nullptr)
        {
            throw std::invalid_argument(
                "Route contains a track that does not exist in railway network"
            );
        }

        if (currentTrack->destination() != nextTrack->source())
        {
            throw std::invalid_argument(
                "Route contains disconnected tracks"
            );
        }
    }
}

TimeSeconds PredictionEngine::calculateTimeToBoundary(
    DistanceMeters distanceToBoundary,
    SpeedMetersPerSecond velocity,
    AccelerationMetersPerSecondSquared acceleration
) noexcept
{
    if (distanceToBoundary <= 0.0)
    {
        return 0.0;
    }

    if (velocity <= 0.0 && acceleration <= 0.0)
    {
        return std::numeric_limits<double>::infinity();
    }

    if (std::abs(acceleration) < 1e-9)
    {
        return velocity > 0.0
                   ? distanceToBoundary / velocity
                   : std::numeric_limits<double>::infinity();
    }

    const double discriminant =
        velocity * velocity + 2.0 * acceleration * distanceToBoundary;

    if (discriminant <= 0.0)
    {
        return velocity > 0.0
                   ? distanceToBoundary / velocity
                   : std::numeric_limits<double>::infinity();
    }

    const double sqrtDisc = std::sqrt(discriminant);
    const double root1 = (-velocity + sqrtDisc) / acceleration;
    const double root2 = (-velocity - sqrtDisc) / acceleration;

    double candidate = std::numeric_limits<double>::infinity();
    if (root1 > 0.0)
    {
        candidate = std::min(candidate, root1);
    }
    if (root2 > 0.0)
    {
        candidate = std::min(candidate, root2);
    }

    if (std::isfinite(candidate))
    {
        return candidate;
    }

    return velocity > 0.0
               ? distanceToBoundary / velocity
               : std::numeric_limits<double>::infinity();
}

std::vector<FutureState> PredictionEngine::predictStandardHorizon(
    const train::Train& train,
    const infrastructure::RailwayNetwork& network,
    const navigation::RouteResult& route,
    TrackId currentTrackId,
    DistanceMeters initialUncertainty
)
{
    constexpr std::array<TimeSeconds, 5> kHorizons{
        5.0,
        10.0,
        20.0,
        30.0,
        60.0
    };

    return predict(
        train,
        network,
        route,
        currentTrackId,
        std::vector<TimeSeconds>(
            kHorizons.begin(),
            kHorizons.end()
        ),
        initialUncertainty
    );
}

std::vector<FutureState> PredictionEngine::predict(
    const train::Train& train,
    const infrastructure::RailwayNetwork& network,
    const navigation::RouteResult& route,
    TrackId currentTrackId,
    const std::vector<TimeSeconds>& horizons,
    DistanceMeters initialUncertainty
)
{
    if (!isFiniteNonNegative(initialUncertainty))
    {
        throw std::invalid_argument(
            "Initial prediction uncertainty must be finite and non-negative"
        );
    }

    if (!route.success || route.tracks.empty())
    {
        throw std::invalid_argument(
            "Prediction requires a valid non-empty route"
        );
    }

    validateRouteConnectivity(network, route);

    const std::size_t currentRouteIndex =
        findCurrentTrackIndex(route, currentTrackId);

    const infrastructure::Track* currentTrack =
        network.getTrack(currentTrackId);

    if (currentTrack == nullptr)
    {
        throw std::invalid_argument(
            "Current track does not exist in railway network"
        );
    }

    if (!isFiniteNonNegative(train.position()))
    {
        throw std::invalid_argument(
            "Train position must be finite and non-negative"
        );
    }

    if (!isFiniteNonNegative(train.velocity()))
    {
        throw std::invalid_argument(
            "Train velocity must be finite and non-negative"
        );
    }

    if (!std::isfinite(train.acceleration()))
    {
        throw std::invalid_argument(
            "Train acceleration must be finite"
        );
    }

    if (train.position() > currentTrack->length())
    {
        throw std::invalid_argument(
            "Train position exceeds current track length"
        );
    }

    for (const TimeSeconds horizon : horizons)
    {
        if (!isFiniteNonNegative(horizon))
        {
            throw std::invalid_argument(
                "Prediction horizons must be finite and non-negative"
            );
        }
    }

    // Sort horizons chronologically for predictable downstream processing
    std::vector<TimeSeconds> sortedHorizons = horizons;
    std::sort(sortedHorizons.begin(), sortedHorizons.end());

    std::vector<FutureState> predictions;
    predictions.reserve(sortedHorizons.size());

    for (const TimeSeconds horizon : sortedHorizons)
    {
        if (horizon == 0.0)
        {
            predictions.push_back(
                FutureState{
                    0.0,
                    currentTrackId,
                    train.position(),
                    train.velocity(),
                    train.acceleration(),
                    initialUncertainty
                }
            );
            continue;
        }

        const PredictionPoint point = predictAtTime(
            train,
            network,
            route,
            currentRouteIndex,
            train.position(),
            initialUncertainty,
            horizon
        );

        predictions.push_back(
            FutureState{
                horizon,
                point.trackId,
                point.position,
                point.velocity,
                point.acceleration,
                point.uncertainty
            }
        );
    }

    return predictions;
}

PredictionEngine::PredictionPoint PredictionEngine::predictAtTime(
    const train::Train& train,
    const infrastructure::RailwayNetwork& network,
    const navigation::RouteResult& route,
    std::size_t currentRouteIndex,
    DistanceMeters initialPosition,
    DistanceMeters initialUncertainty,
    TimeSeconds horizon
)
{
    TrackId currentTrackId = route.tracks[currentRouteIndex];

    const infrastructure::Track* track =
        network.getTrack(currentTrackId);

    if (track == nullptr)
    {
        throw std::invalid_argument(
            "Route contains a track that does not exist"
        );
    }

    DistanceMeters position = initialPosition;
    SpeedMetersPerSecond velocity = train.velocity();
    AccelerationMetersPerSecondSquared acceleration =
        train.acceleration();

    TimeSeconds remainingTime = horizon;
    TimeSeconds elapsedTime = 0.0;

    std::size_t routeIndex = currentRouteIndex;

    while (remainingTime > 0.0)
    {
        currentTrackId = route.tracks[routeIndex];
        track = network.getTrack(currentTrackId);

        if (track == nullptr)
        {
            throw std::invalid_argument(
                "Route contains a missing track"
            );
        }

        // Handle explicit track boundary transition edge case
        if (position >= track->length())
        {
            if (routeIndex + 1U >= route.tracks.size())
            {
                velocity = 0.0;
                acceleration = 0.0;
                position = track->length();
                break;
            }

            ++routeIndex;
            position = 0.0;
            continue;
        }

        const DistanceMeters remainingDistance =
            std::max(0.0, track->length() - position);

        // Stopped condition check
        if (velocity <= 0.0 && acceleration <= 0.0)
        {
            break;
        }

        // Apply track speed limit
        const SpeedMetersPerSecond trackSpeedLimit =
            std::min(train.maximumSpeed(), track->speedLimit());
        velocity = std::min(velocity, trackSpeedLimit);

        // Check next track speed limit for required service braking before boundary
        double targetNextLimit = trackSpeedLimit;
        if (routeIndex + 1U < route.tracks.size())
        {
            const auto* nextTrack = network.getTrack(route.tracks[routeIndex + 1U]);
            if (nextTrack != nullptr)
            {
                targetNextLimit = std::min(train.maximumSpeed(), nextTrack->speedLimit());
            }
        }

        const double effectiveServiceDecel =
            physics::KinematicsEngine::effectiveDeceleration(
                train.serviceBraking(),
                track->gradient()
            );

        AccelerationMetersPerSecondSquared effectiveAcceleration = acceleration;

        // If approaching a lower speed limit track, check if braking is required
        if (velocity > targetNextLimit && effectiveServiceDecel > 0.0)
        {
            const double requiredBrakingDist =
                (velocity * velocity - targetNextLimit * targetNextLimit) /
                (2.0 * effectiveServiceDecel);

            if (remainingDistance <= requiredBrakingDist)
            {
                // Start service braking to meet the lower speed limit at the boundary
                effectiveAcceleration = -effectiveServiceDecel;
            }
            else
            {
                // Apply gradient effect to longitudinal motion
                const double gradientAcceleration =
                    -physics::KinematicsEngine::kGravity * track->gradient();
                effectiveAcceleration = acceleration + gradientAcceleration;
            }
        }
        else
        {
            // Apply gradient effect to longitudinal motion
            const double gradientAcceleration =
                -physics::KinematicsEngine::kGravity * track->gradient();
            effectiveAcceleration = acceleration + gradientAcceleration;
        }

        const SpeedMetersPerSecond predictedVelocity =
            physics::KinematicsEngine::updateVelocity(
                velocity,
                effectiveAcceleration,
                remainingTime,
                trackSpeedLimit
            );

        const DistanceMeters predictedPosition =
            physics::KinematicsEngine::updatePosition(
                position,
                velocity,
                effectiveAcceleration,
                remainingTime
            );

        // Predicted position remains within current track
        if (predictedPosition < track->length())
        {
            position = std::max(0.0, predictedPosition);
            velocity = std::min(predictedVelocity, trackSpeedLimit);
            acceleration = effectiveAcceleration;

            elapsedTime += remainingTime;
            remainingTime = 0.0;
            break;
        }

        // Reaches end of track during remaining time interval
        TimeSeconds timeToBoundary = calculateTimeToBoundary(
            remainingDistance,
            velocity,
            effectiveAcceleration
        );

        timeToBoundary = std::clamp(timeToBoundary, 0.0, remainingTime);

        position = track->length();

        velocity = physics::KinematicsEngine::updateVelocity(
            velocity,
            effectiveAcceleration,
            timeToBoundary,
            trackSpeedLimit
        );

        acceleration = effectiveAcceleration;

        elapsedTime += timeToBoundary;
        remainingTime -= timeToBoundary;

        // Check if route completed
        if (routeIndex + 1U >= route.tracks.size())
        {
            velocity = 0.0;
            acceleration = 0.0;
            remainingTime = 0.0;
            break;
        }

        // Move to next track
        ++routeIndex;
        currentTrackId = route.tracks[routeIndex];
        track = network.getTrack(currentTrackId);

        if (track == nullptr)
        {
            throw std::invalid_argument("Next route track does not exist");
        }

        position = 0.0;
        velocity = std::min(velocity, std::min(train.maximumSpeed(), track->speedLimit()));

        if (remainingTime <= 0.0)
        {
            break;
        }
    }

    const DistanceMeters uncertainty =
        propagateUncertainty(
            initialUncertainty,
            elapsedTime
        );

    return PredictionPoint{
        currentTrackId,
        position,
        velocity,
        acceleration,
        uncertainty
    };
}

DistanceMeters PredictionEngine::propagateUncertainty(
    DistanceMeters initialUncertainty,
    TimeSeconds elapsed
)
{
    // Deterministic uncertainty growth prediction model for the PoC.
    //
    // sigma_position(t) = sigma_initial + growth_rate * t
    //
    // Module 6 remains the authoritative state estimator.
    // Module 8 propagates uncertainty over future prediction horizons.
    return initialUncertainty + kUncertaintyGrowthRate * elapsed;
}

} // namespace tcas::prediction