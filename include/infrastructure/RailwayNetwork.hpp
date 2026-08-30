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

    // Returns track IDs of all outgoing tracks from the given node.
    // Returns an empty vector if the node has no outgoing tracks or
    // does not exist.
    [[nodiscard]]
    std::vector<TrackId> getOutgoingTracks(NodeId nodeId) const;

    // Returns a const-reference to the internal track map.
    // Used by Dijkstra to enumerate all edges.
    [[nodiscard]]
    const std::unordered_map<TrackId, Track>& getAllTracks() const noexcept;

    // Returns a const-reference to the internal node map.
    [[nodiscard]]
    const std::unordered_map<NodeId, Node>& getAllNodes() const noexcept;

private:
    [[nodiscard]]
    bool hasCycleFrom(
        NodeId node,
        std::unordered_map<NodeId, bool>& visited,
        std::unordered_map<NodeId, bool>& recursionStack
    ) const;

private:
    std::unordered_map<NodeId, Node>   nodes_;
    std::unordered_map<TrackId, Track> tracks_;

    // adjacency_[nodeId] = list of destination NodeIds
    std::unordered_map<NodeId, std::vector<NodeId>> adjacency_;

    // outgoing_[nodeId] = list of TrackIds leaving that node
    std::unordered_map<NodeId, std::vector<TrackId>> outgoing_;
};

} // namespace tcas::infrastructure