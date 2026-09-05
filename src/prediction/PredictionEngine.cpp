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

    std::vector<FutureState> predictions;
    predictions.reserve(horizons.size());

    for (const TimeSeconds horizon : horizons)
    {
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
        track = network.getTrack(
            route.tracks[routeIndex]
        );

        if (track == nullptr)
        {
            throw std::invalid_argument(
                "Route contains a missing track"
            );
        }

        const DistanceMeters remainingDistance =
            std::max(0.0, track->length() - position);

        // No physical movement is possible when the train is stopped.
        if (velocity <= 0.0 && acceleration <= 0.0)
        {
            break;
        }

        // Respect the speed limit of the current track.
        velocity = std::min(
            velocity,
            track->speedLimit()
        );

        // Positive gradient is uphill.
        // Negative gradient is downhill.
        //
        // For prediction, use the train's acceleration while also
        // applying the gradient effect to the longitudinal motion.
        const double gradientAcceleration =
            -physics::KinematicsEngine::kGravity *
            track->gradient();

        const AccelerationMetersPerSecondSquared effectiveAcceleration =
            acceleration + gradientAcceleration;

        const SpeedMetersPerSecond predictedVelocity =
            physics::KinematicsEngine::updateVelocity(
                velocity,
                effectiveAcceleration,
                remainingTime,
                std::min(
                    train.maximumSpeed(),
                    track->speedLimit()
                )
            );

        const DistanceMeters predictedPosition =
            physics::KinematicsEngine::updatePosition(
                position,
                velocity,
                effectiveAcceleration,
                remainingTime
            );

        // The predicted position remains inside the current track.
        if (predictedPosition < track->length())
        {
            position = std::max(
                0.0,
                predictedPosition
            );

            velocity = std::min(
                predictedVelocity,
                track->speedLimit()
            );

            acceleration = effectiveAcceleration;

            elapsedTime += remainingTime;
            remainingTime = 0.0;
            break;
        }

        // The train reaches the end of this track during the
        // remaining prediction interval.
        const DistanceMeters distanceToBoundary =
            remainingDistance;

        TimeSeconds timeToBoundary = 0.0;

        if (velocity > 0.0)
        {
            if (std::abs(effectiveAcceleration) < 1e-9)
            {
                timeToBoundary =
                    distanceToBoundary / velocity;
            }
            else
            {
                const double discriminant =
                    velocity * velocity +
                    2.0 *
                    effectiveAcceleration *
                    distanceToBoundary;

                if (discriminant <= 0.0)
                {
                    timeToBoundary =
                        distanceToBoundary / velocity;
                }
                else
                {
                    const double sqrtDiscriminant =
                        std::sqrt(discriminant);

                    const double root1 =
                        (-velocity + sqrtDiscriminant) /
                        effectiveAcceleration;

                    const double root2 =
                        (-velocity - sqrtDiscriminant) /
                        effectiveAcceleration;

                    const double candidate1 =
                        root1 > 0.0
                            ? root1
                            : std::numeric_limits<double>::infinity();

                    const double candidate2 =
                        root2 > 0.0
                            ? root2
                            : std::numeric_limits<double>::infinity();

                    timeToBoundary =
                        std::min(candidate1, candidate2);

                    if (!std::isfinite(timeToBoundary))
                    {
                        timeToBoundary =
                            distanceToBoundary / velocity;
                    }
                }
            }
        }

        timeToBoundary = std::clamp(
            timeToBoundary,
            0.0,
            remainingTime
        );

        position = track->length();

        velocity =
            physics::KinematicsEngine::updateVelocity(
                velocity,
                effectiveAcceleration,
                timeToBoundary,
                std::min(
                    train.maximumSpeed(),
                    track->speedLimit()
                )
            );

        acceleration = effectiveAcceleration;

        elapsedTime += timeToBoundary;
        remainingTime -= timeToBoundary;

        // Route finished.
        if (routeIndex + 1U >= route.tracks.size())
        {
            velocity = 0.0;
            acceleration = 0.0;
            remainingTime = 0.0;
            break;
        }

        // Move to the next route track.
        ++routeIndex;

        currentTrackId =
            route.tracks[routeIndex];

        track = network.getTrack(currentTrackId);

        if (track == nullptr)
        {
            throw std::invalid_argument(
                "Next route track does not exist"
            );
        }

        position = 0.0;

        // New track speed limit applies immediately.
        velocity = std::min(
            velocity,
            std::min(
                train.maximumSpeed(),
                track->speedLimit()
            )
        );

        // If there is no time remaining, the state is exactly
        // at the beginning of the next track.
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
    // Simple deterministic uncertainty growth model for the PoC.
    //
    // sigma_position(t) =
    //     sigma_initial + growth_rate * t
    //
    // Module 6 remains the authoritative state estimator.
    // Module 8 propagates the uncertainty available at prediction start.
    return initialUncertainty +
           kUncertaintyGrowthRate * elapsed;
}

} // namespace tcas::prediction