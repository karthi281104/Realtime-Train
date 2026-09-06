#include "conflict/ConflictDetector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tcas::conflict
{
namespace
{

struct Sample
{
    TimeSeconds time{ 0.0 };
    DistanceMeters position{ 0.0 };
    DistanceMeters uncertainty{ 0.0 };
};

Sample interpolate(
    const prediction::FutureState& first,
    const prediction::FutureState& second,
    TimeSeconds time
)
{
    const TimeSeconds duration = second.timestamp - first.timestamp;
    if (duration <= 0.0)
    {
        return { time, first.position, first.uncertainty };
    }

    const double fraction =
        std::clamp((time - first.timestamp) / duration, 0.0, 1.0);

    return {
        time,
        first.position + fraction * (second.position - first.position),
        first.uncertainty + fraction * (second.uncertainty - first.uncertainty)
    };
}

bool validTrajectory(const std::vector<prediction::FutureState>& trajectory)
{
    if (trajectory.empty())
    {
        return false;
    }

    for (std::size_t index = 1; index < trajectory.size(); ++index)
    {
        if (trajectory[index].timestamp < trajectory[index - 1].timestamp)
        {
            return false;
        }
    }
    return true;
}

} // namespace

ConflictDetector::ConflictDetector(ConflictDetectionConfig config)
    : config_(config)
{
    if (!std::isfinite(config_.minimumTrackSeparation) ||
        config_.minimumTrackSeparation < 0.0 ||
        !std::isfinite(config_.resourceTimeSeparation) ||
        config_.resourceTimeSeparation < 0.0 ||
        !std::isfinite(config_.resourceClearanceTime) ||
        config_.resourceClearanceTime < 0.0)
    {
        throw std::invalid_argument("Invalid conflict detection configuration");
    }
}

std::vector<Conflict> ConflictDetector::detect(
    TrainId trainA,
    const std::vector<prediction::FutureState>& trajectoryA,
    TrainId trainB,
    const std::vector<prediction::FutureState>& trajectoryB
) const
{
    if (trainA == trainB)
    {
        throw std::invalid_argument("Conflict detection requires two different trains");
    }

    if (!validTrajectory(trajectoryA) || !validTrajectory(trajectoryB))
    {
        throw std::invalid_argument("Trajectories must be non-empty and chronological");
    }

    std::vector<Conflict> conflicts;

    // Each prediction sample represents the state at that instant. For the
    // interval to the next sample, linear interpolation supplies a continuous
    // approximation. A track transition is conservatively treated as occupying
    // both adjacent tracks for that sample interval so a short-lived conflict
    // cannot be hidden by sparse prediction horizons.
    for (std::size_t i = 0; i < trajectoryA.size(); ++i)
    {
        const auto& a0 = trajectoryA[i];
        const auto& a1 = (i + 1 < trajectoryA.size()) ? trajectoryA[i + 1] : a0;

        for (std::size_t j = 0; j < trajectoryB.size(); ++j)
        {
            const auto& b0 = trajectoryB[j];
            const auto& b1 = (j + 1 < trajectoryB.size()) ? trajectoryB[j + 1] : b0;

            if (a0.trackId != b0.trackId ||
                a1.trackId != b1.trackId ||
                a0.trackId != a1.trackId ||
                b0.trackId != b1.trackId)
            {
                continue;
            }

            DistanceMeters minimumSeparation =
                std::numeric_limits<DistanceMeters>::infinity();
            TimeSeconds firstTime = 0.0;
            TimeSeconds lastTime = 0.0;

            if (!hasTemporalConflict(
                    a0, a1, b0, b1, minimumSeparation, firstTime, lastTime))
            {
                continue;
            }

            const ConflictType type = classifySameTrack(a0, b0);
            const Conflict candidate{
                trainA,
                trainB,
                type,
                a0.trackId,
                0,
                firstTime,
                lastTime,
                minimumSeparation
            };

            const bool duplicate = std::any_of(
                conflicts.begin(), conflicts.end(),
                [&](const Conflict& existing)
                {
                    return existing.trackId == candidate.trackId &&
                           existing.type == candidate.type &&
                           std::abs(existing.firstConflictTime - candidate.firstConflictTime) < 1e-9;
                });

            if (!duplicate)
            {
                conflicts.push_back(candidate);
            }
        }
    }

    return conflicts;
}

ConflictType ConflictDetector::classifySameTrack(
    const prediction::FutureState& a,
    const prediction::FutureState& b
) const noexcept
{
    // Relative velocity sign determines whether the trains are closing. For
    // opposite-direction motion this is a head-on conflict; otherwise it is
    // treated as rear-end. Zero relative velocity is conservatively rear-end.
    const double relativeVelocity = a.velocity - b.velocity;
    const double relativePosition = a.position - b.position;

    if (relativeVelocity * relativePosition < 0.0)
    {
        return ConflictType::RearEnd;
    }

    if (relativeVelocity != 0.0)
    {
        return ConflictType::HeadOn;
    }

    return ConflictType::RearEnd;
}

bool ConflictDetector::hasTemporalConflict(
    const prediction::FutureState& a0,
    const prediction::FutureState& a1,
    const prediction::FutureState& b0,
    const prediction::FutureState& b1,
    DistanceMeters& minimumSeparation,
    TimeSeconds& firstTime,
    TimeSeconds& lastTime
) const noexcept
{
    const TimeSeconds start = std::max(a0.timestamp, b0.timestamp);
    const TimeSeconds end = std::min(a1.timestamp, b1.timestamp);

    if (end < start)
    {
        return false;
    }

    const TimeSeconds duration = end - start;
    constexpr int kSteps = 20;

    bool conflict = false;
    firstTime = 0.0;
    lastTime = 0.0;
    minimumSeparation = std::numeric_limits<DistanceMeters>::infinity();

    for (int step = 0; step <= kSteps; ++step)
    {
        const TimeSeconds time =
            start + duration * static_cast<double>(step) / static_cast<double>(kSteps);
        const Sample a = interpolate(a0, a1, time);
        const Sample b = interpolate(b0, b1, time);
        const DistanceMeters separation = std::abs(a.position - b.position);
        const DistanceMeters uncertaintyMargin =
            config_.minimumTrackSeparation + a.uncertainty + b.uncertainty;

        minimumSeparation = std::min(minimumSeparation, separation);

        if (separation <= uncertaintyMargin)
        {
            if (!conflict)
            {
                firstTime = time;
                conflict = true;
            }
            lastTime = time;
        }
    }

    return conflict;
}

} // namespace tcas::conflict
