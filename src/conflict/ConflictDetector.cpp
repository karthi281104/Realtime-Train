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
    TimeSeconds time)
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
        if (!std::isfinite(trajectory[index].timestamp) ||
            !std::isfinite(trajectory[index].position) ||
            !std::isfinite(trajectory[index].velocity) ||
            !std::isfinite(trajectory[index].uncertainty) ||
            trajectory[index].timestamp < trajectory[index - 1].timestamp)
        {
            return false;
        }
    }
    return std::isfinite(trajectory.front().timestamp) &&
           std::isfinite(trajectory.front().position) &&
           std::isfinite(trajectory.front().velocity) &&
           std::isfinite(trajectory.front().uncertainty);
}

struct NodeEvent
{
    NodeId nodeId{ 0 };
    TimeSeconds time{ 0.0 };
    DistanceMeters uncertainty{ 0.0 };
};

std::vector<NodeEvent> extractNodeEvents(
    const std::vector<prediction::FutureState>& trajectory,
    const infrastructure::RailwayNetwork& network)
{
    std::vector<NodeEvent> events;

    for (std::size_t index = 1; index < trajectory.size(); ++index)
    {
        const auto& previous = trajectory[index - 1];
        const auto& current = trajectory[index];
        if (previous.trackId == current.trackId)
        {
            continue;
        }

        const auto* previousTrack = network.getTrack(previous.trackId);
        const auto* currentTrack = network.getTrack(current.trackId);
        if (previousTrack == nullptr || currentTrack == nullptr ||
            previousTrack->destination() != currentTrack->source())
        {
            continue;
        }

        events.push_back({
            previousTrack->destination(),
            current.timestamp,
            current.uncertainty
        });
    }

    return events;
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
    const std::vector<prediction::FutureState>& trajectoryB,
    const infrastructure::RailwayNetwork& network) const
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

    for (std::size_t i = 0; i + 1 < trajectoryA.size(); ++i)
    {
        for (std::size_t j = 0; j + 1 < trajectoryB.size(); ++j)
        {
            const auto& a0 = trajectoryA[i];
            const auto& a1 = trajectoryA[i + 1];
            const auto& b0 = trajectoryB[j];
            const auto& b1 = trajectoryB[j + 1];

            // Only compare a common directed track during an interval. This
            // avoids inventing conflicts between unrelated track IDs.
            if (a0.trackId != a1.trackId || a0.trackId != b0.trackId ||
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

            const Conflict candidate{
                trainA,
                trainB,
                classifySameTrack(a0, b0),
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

    const auto eventsA = extractNodeEvents(trajectoryA, network);
    const auto eventsB = extractNodeEvents(trajectoryB, network);

    for (const auto& eventA : eventsA)
    {
        const auto* node = network.getNode(eventA.nodeId);
        if (node == nullptr ||
            (node->type() != infrastructure::NodeType::Junction &&
             node->type() != infrastructure::NodeType::Platform))
        {
            continue;
        }

        for (const auto& eventB : eventsB)
        {
            if (eventA.nodeId != eventB.nodeId ||
                std::abs(eventA.time - eventB.time) > config_.resourceTimeSeparation)
            {
                continue;
            }

            const ConflictType type =
                node->type() == infrastructure::NodeType::Junction
                    ? ConflictType::Junction
                    : ConflictType::Platform;

            conflicts.push_back({
                trainA,
                trainB,
                type,
                0,
                eventA.nodeId,
                std::min(eventA.time, eventB.time),
                std::max(eventA.time, eventB.time) + config_.resourceClearanceTime,
                0.0
            });
        }
    }

    return conflicts;
}

ConflictType ConflictDetector::classifySameTrack(
    const prediction::FutureState& a,
    const prediction::FutureState& b) const noexcept
{
    // FutureState contains a signed velocity quantity. Opposite signs mean
    // the trains are travelling in opposite directions on the same track.
    if (a.velocity * b.velocity < 0.0)
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
    TimeSeconds& lastTime) const noexcept
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
    minimumSeparation = std::numeric_limits<DistanceMeters>::infinity();

    for (int step = 0; step <= kSteps; ++step)
    {
        const TimeSeconds time =
            start + duration * static_cast<double>(step) / static_cast<double>(kSteps);
        const Sample a = interpolate(a0, a1, time);
        const Sample b = interpolate(b0, b1, time);
        const DistanceMeters separation = std::abs(a.position - b.position);
        const DistanceMeters protectedDistance =
            config_.minimumTrackSeparation + a.uncertainty + b.uncertainty;

        minimumSeparation = std::min(minimumSeparation, separation);
        if (separation <= protectedDistance)
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

bool ConflictDetector::hasNodeConflict(
    TrainId,
    const std::vector<prediction::FutureState>&,
    TrainId,
    const std::vector<prediction::FutureState>&,
    const infrastructure::RailwayNetwork&,
    std::vector<Conflict>&) const
{
    return false;
}

} // namespace tcas::conflict
