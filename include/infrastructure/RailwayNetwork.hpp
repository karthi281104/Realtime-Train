#pragma once

#include "infrastructure/Node.hpp"
#include "infrastructure/Track.hpp"

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace tcas::infrastructure
{

class RailwayNetwork
{
public:
    RailwayNetwork() = default;

    bool addNode(const Node& node);

    bool addTrack(const Track& track);

    [[nodiscard]]
    const Node* getNode(NodeId id) const noexcept;

    [[nodiscard]]
    const Track* getTrack(TrackId id) const noexcept;

    [[nodiscard]]
    std::size_t nodeCount() const noexcept;

    [[nodiscard]]
    std::size_t trackCount() const noexcept;

    [[nodiscard]]
    bool isConnected() const;

    [[nodiscard]]
    bool hasCycle() const;

private:
    [[nodiscard]]
    bool hasCycleFrom(
        NodeId node,
        std::unordered_map<NodeId, bool>& visited,
        std::unordered_map<NodeId, bool>& recursionStack
    ) const;

private:
    std::unordered_map<NodeId, Node> nodes_;
    std::unordered_map<TrackId, Track> tracks_;

    std::unordered_map<NodeId, std::vector<NodeId>> adjacency_;
};

} // namespace tcas::infrastructure