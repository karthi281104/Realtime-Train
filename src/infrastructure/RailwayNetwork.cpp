#include "infrastructure/RailwayNetwork.hpp"

#include <queue>
#include <unordered_set>

namespace tcas::infrastructure
{

bool RailwayNetwork::addNode(const Node& node)
{
    const auto [iterator, inserted] =
        nodes_.emplace(node.id(), node);

    if (!inserted)
    {
        return false;
    }

    adjacency_.try_emplace(node.id());

    return true;
}

bool RailwayNetwork::addTrack(const Track& track)
{
    if (tracks_.contains(track.id()))
    {
        return false;
    }

    if (!nodes_.contains(track.source()) ||
        !nodes_.contains(track.destination()))
    {
        return false;
    }

    if (track.length() <= 0.0 ||
        track.speedLimit() < 0.0)
    {
        return false;
    }

    tracks_.emplace(track.id(), track);

    adjacency_[track.source()].push_back(
        track.destination()
    );

    return true;
}

const Node* RailwayNetwork::getNode(NodeId id) const noexcept
{
    const auto iterator = nodes_.find(id);

    if (iterator == nodes_.end())
    {
        return nullptr;
    }

    return &iterator->second;
}

const Track* RailwayNetwork::getTrack(TrackId id) const noexcept
{
    const auto iterator = tracks_.find(id);

    if (iterator == tracks_.end())
    {
        return nullptr;
    }

    return &iterator->second;
}

std::size_t RailwayNetwork::nodeCount() const noexcept
{
    return nodes_.size();
}

std::size_t RailwayNetwork::trackCount() const noexcept
{
    return tracks_.size();
}

bool RailwayNetwork::isConnected() const
{
    if (nodes_.empty())
    {
        return true;
    }

    for (const auto& [startNode, _] : nodes_)
    {
        std::queue<NodeId> queue;
        std::unordered_set<NodeId> visited;

        queue.push(startNode);
        visited.insert(startNode);

        while (!queue.empty())
        {
            const NodeId current = queue.front();
            queue.pop();

            const auto adjacencyIterator =
                adjacency_.find(current);

            if (adjacencyIterator == adjacency_.end())
            {
                continue;
            }

            for (const NodeId neighbour :
                 adjacencyIterator->second)
            {
                if (!visited.contains(neighbour))
                {
                    visited.insert(neighbour);
                    queue.push(neighbour);
                }
            }
        }

        if (visited.size() == nodes_.size())
        {
            return true;
        }
    }

    return false;
}

bool RailwayNetwork::hasCycle() const
{
    std::unordered_map<NodeId, bool> visited;
    std::unordered_map<NodeId, bool> recursionStack;

    for (const auto& [nodeId, node] : nodes_)
    {
        if (!visited[nodeId])
        {
            if (hasCycleFrom(
                    nodeId,
                    visited,
                    recursionStack))
            {
                return true;
            }
        }
    }

    return false;
}

bool RailwayNetwork::hasCycleFrom(
    NodeId node,
    std::unordered_map<NodeId, bool>& visited,
    std::unordered_map<NodeId, bool>& recursionStack
) const
{
    visited[node] = true;
    recursionStack[node] = true;

    const auto adjacencyIterator =
        adjacency_.find(node);

    if (adjacencyIterator != adjacency_.end())
    {
        for (const NodeId neighbour :
             adjacencyIterator->second)
        {
            if (!visited[neighbour])
            {
                if (hasCycleFrom(
                        neighbour,
                        visited,
                        recursionStack))
                {
                    return true;
                }
            }
            else if (recursionStack[neighbour])
            {
                return true;
            }
        }
    }

    recursionStack[node] = false;

    return false;
}

} // namespace tcas::infrastructure