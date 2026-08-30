#include "navigation/RouteNavigator.hpp"

#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace tcas::navigation
{

RouteResult RouteNavigator::findRoute(
    const infrastructure::RailwayNetwork& network,
    const NodeId origin,
    const NodeId destination
)
{
    // ---- Validation ------------------------------------------------

    if (origin == destination)
    {
        return RouteResult{
            .success       = false,
            .tracks        = {},
            .totalDistance = 0.0,
            .reason        = RouteResult::FailReason::SameNode
        };
    }

    if (network.getNode(origin) == nullptr ||
        network.getNode(destination) == nullptr)
    {
        return RouteResult{
            .success       = false,
            .tracks        = {},
            .totalDistance = 0.0,
            .reason        = RouteResult::FailReason::NodeNotFound
        };
    }

    // ---- Dijkstra --------------------------------------------------

    // dist[nodeId] = shortest known distance from origin
    std::unordered_map<NodeId, double> dist;
    // prev[nodeId] = TrackId used to reach this node on shortest path
    std::unordered_map<NodeId, TrackId> prev;
    // prevNode[nodeId] = node preceding this node on shortest path
    std::unordered_map<NodeId, NodeId> prevNode;

    constexpr double kInfinity = std::numeric_limits<double>::infinity();

    // Initialise all nodes to infinity
    for (const auto& [nodeId, node] : network.getAllNodes())
    {
        dist[nodeId] = kInfinity;
    }

    dist[origin] = 0.0;

    // Min-heap: (distance, nodeId)
    using Entry = std::pair<double, NodeId>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    pq.emplace(0.0, origin);

    while (!pq.empty())
    {
        const auto [currentDist, currentNode] = pq.top();
        pq.pop();

        // Skip stale entries
        if (currentDist > dist[currentNode])
        {
            continue;
        }

        if (currentNode == destination)
        {
            break;
        }

        // Relax outgoing edges
        for (const TrackId trackId : network.getOutgoingTracks(currentNode))
        {
            const auto* track = network.getTrack(trackId);

            if (track == nullptr)
            {
                continue;
            }

            const double candidate = dist[currentNode] + track->length();

            if (candidate < dist[track->destination()])
            {
                dist[track->destination()] = candidate;
                prev[track->destination()]     = trackId;
                prevNode[track->destination()] = currentNode;
                pq.emplace(candidate, track->destination());
            }
        }
    }

    // ---- Path reconstruction ---------------------------------------

    if (dist[destination] == kInfinity)
    {
        return RouteResult{
            .success       = false,
            .tracks        = {},
            .totalDistance = 0.0,
            .reason        = RouteResult::FailReason::NoPathExists
        };
    }

    // Walk back from destination to origin via prevNode map
    std::vector<TrackId> reversedTracks;
    NodeId cursor = destination;

    while (cursor != origin)
    {
        reversedTracks.push_back(prev.at(cursor));
        cursor = prevNode.at(cursor);
    }

    // Reverse to get origin-to-destination order
    std::vector<TrackId> orderedTracks(
        reversedTracks.rbegin(),
        reversedTracks.rend()
    );

    return RouteResult{
        .success       = true,
        .tracks        = std::move(orderedTracks),
        .totalDistance = dist[destination],
        .reason        = RouteResult::FailReason::None
    };
}

} // namespace tcas::navigation
