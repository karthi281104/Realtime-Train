#include "infrastructure/RailwayNetwork.hpp"

#include <gtest/gtest.h>

using namespace tcas;
using namespace tcas::infrastructure;

namespace
{

RailwayNetwork createSimpleNetwork()
{
    RailwayNetwork network;

    network.addNode(
        Node(1, "A", NodeType::Station)
    );

    network.addNode(
        Node(2, "B", NodeType::Station)
    );

    network.addNode(
        Node(3, "C", NodeType::Station)
    );

    network.addTrack(
        Track(101, 1, 2, 1000.0, 30.0, 0.0)
    );

    network.addTrack(
        Track(102, 2, 3, 1000.0, 30.0, 0.0)
    );

    return network;
}

} // namespace

TEST(RailwayNetworkTest, StartsEmpty)
{
    RailwayNetwork network;

    EXPECT_EQ(network.nodeCount(), 0);
    EXPECT_EQ(network.trackCount(), 0);
}

TEST(RailwayNetworkTest, AddsNode)
{
    RailwayNetwork network;

    EXPECT_TRUE(
        network.addNode(
            Node(1, "A", NodeType::Station)
        )
    );

    EXPECT_EQ(network.nodeCount(), 1);
}

TEST(RailwayNetworkTest, RejectsDuplicateNode)
{
    RailwayNetwork network;

    EXPECT_TRUE(
        network.addNode(
            Node(1, "A", NodeType::Station)
        )
    );

    EXPECT_FALSE(
        network.addNode(
            Node(1, "Duplicate", NodeType::Station)
        )
    );

    EXPECT_EQ(network.nodeCount(), 1);
}

TEST(RailwayNetworkTest, AddsValidTrack)
{
    RailwayNetwork network;

    network.addNode(
        Node(1, "A", NodeType::Station)
    );

    network.addNode(
        Node(2, "B", NodeType::Station)
    );

    EXPECT_TRUE(
        network.addTrack(
            Track(
                100,
                1,
                2,
                1000.0,
                30.0,
                0.0
            )
        )
    );

    EXPECT_EQ(network.trackCount(), 1);
}

TEST(RailwayNetworkTest, RejectsTrackWithMissingSource)
{
    RailwayNetwork network;

    network.addNode(
        Node(2, "B", NodeType::Station)
    );

    EXPECT_FALSE(
        network.addTrack(
            Track(
                100,
                1,
                2,
                1000.0,
                30.0,
                0.0
            )
        )
    );
}

TEST(RailwayNetworkTest, RejectsTrackWithMissingDestination)
{
    RailwayNetwork network;

    network.addNode(
        Node(1, "A", NodeType::Station)
    );

    EXPECT_FALSE(
        network.addTrack(
            Track(
                100,
                1,
                2,
                1000.0,
                30.0,
                0.0
            )
        )
    );
}

TEST(RailwayNetworkTest, RejectsDuplicateTrack)
{
    RailwayNetwork network;

    network.addNode(
        Node(1, "A", NodeType::Station)
    );

    network.addNode(
        Node(2, "B", NodeType::Station)
    );

    Track track(
        100,
        1,
        2,
        1000.0,
        30.0,
        0.0
    );

    EXPECT_TRUE(network.addTrack(track));
    EXPECT_FALSE(network.addTrack(track));

    EXPECT_EQ(network.trackCount(), 1);
}

TEST(RailwayNetworkTest, FindsNode)
{
    RailwayNetwork network;

    network.addNode(
        Node(1, "A", NodeType::Station)
    );

    const Node* node = network.getNode(1);

    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->name(), "A");
}

TEST(RailwayNetworkTest, MissingNodeReturnsNull)
{
    RailwayNetwork network;

    EXPECT_EQ(network.getNode(999), nullptr);
}

TEST(RailwayNetworkTest, FindsTrack)
{
    RailwayNetwork network;

    network.addNode(
        Node(1, "A", NodeType::Station)
    );

    network.addNode(
        Node(2, "B", NodeType::Station)
    );

    network.addTrack(
        Track(100, 1, 2, 1000.0, 30.0, 0.0)
    );

    const Track* track = network.getTrack(100);

    ASSERT_NE(track, nullptr);
    EXPECT_EQ(track->source(), 1);
    EXPECT_EQ(track->destination(), 2);
}

TEST(RailwayNetworkTest, EmptyNetworkIsConnected)
{
    RailwayNetwork network;

    EXPECT_TRUE(network.isConnected());
}

TEST(RailwayNetworkTest, SingleNodeNetworkIsConnected)
{
    RailwayNetwork network;

    network.addNode(
        Node(1, "A", NodeType::Station)
    );

    EXPECT_TRUE(network.isConnected());
}

TEST(RailwayNetworkTest, BFSDetectsConnectedNetwork)
{
    RailwayNetwork network = createSimpleNetwork();

    EXPECT_TRUE(network.isConnected());
}

TEST(RailwayNetworkTest, BFSDetectsDisconnectedNetwork)
{
    RailwayNetwork network;

    network.addNode(
        Node(1, "A", NodeType::Station)
    );

    network.addNode(
        Node(2, "B", NodeType::Station)
    );

    network.addNode(
        Node(3, "C", NodeType::Station)
    );

    network.addTrack(
        Track(100, 1, 2, 1000.0, 30.0, 0.0)
    );

    EXPECT_FALSE(network.isConnected());
}

TEST(RailwayNetworkTest, DFSDetectsNoCycle)
{
    RailwayNetwork network = createSimpleNetwork();

    EXPECT_FALSE(network.hasCycle());
}

TEST(RailwayNetworkTest, DFSDetectsCycle)
{
    RailwayNetwork network;

    network.addNode(
        Node(1, "A", NodeType::Junction)
    );

    network.addNode(
        Node(2, "B", NodeType::Junction)
    );

    network.addNode(
        Node(3, "C", NodeType::Junction)
    );

    network.addTrack(
        Track(101, 1, 2, 1000.0, 30.0, 0.0)
    );

    network.addTrack(
        Track(102, 2, 3, 1000.0, 30.0, 0.0)
    );

    network.addTrack(
        Track(103, 3, 1, 1000.0, 30.0, 0.0)
    );

    EXPECT_TRUE(network.hasCycle());
}