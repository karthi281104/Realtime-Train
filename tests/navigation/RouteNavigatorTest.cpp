#include "navigation/RouteNavigator.hpp"
#include "infrastructure/Node.hpp"
#include "infrastructure/Track.hpp"
#include "infrastructure/RailwayNetwork.hpp"

#include <gtest/gtest.h>

using namespace tcas;
using namespace tcas::infrastructure;
using namespace tcas::navigation;

namespace
{

// Helper: build a simple linear network: N1 --(T1)--> N2 --(T2)--> N3
//
//   N1 ---T1(1000m)---> N2 ---T2(500m)---> N3
//
RailwayNetwork makeLinearNetwork()
{
    RailwayNetwork net;
    net.addNode(Node(1, "A", NodeType::Station));
    net.addNode(Node(2, "B", NodeType::Station));
    net.addNode(Node(3, "C", NodeType::Station));
    net.addTrack(Track(101, 1, 2, 1000.0, 30.0, 0.0));
    net.addTrack(Track(102, 2, 3,  500.0, 30.0, 0.0));
    return net;
}

// Diamond network: two parallel paths N1->N4
//
//   N1 --T1(100)--> N2 --T3(200)--> N4
//   N1 --T2(600)--> N3 --T4(100)--> N4
//
// Shortest: N1->N2->N4 (300m)
//
RailwayNetwork makeDiamondNetwork()
{
    RailwayNetwork net;
    net.addNode(Node(1, "A", NodeType::Station));
    net.addNode(Node(2, "B", NodeType::Junction));
    net.addNode(Node(3, "C", NodeType::Junction));
    net.addNode(Node(4, "D", NodeType::Station));
    net.addTrack(Track(101, 1, 2, 100.0, 30.0, 0.0));
    net.addTrack(Track(102, 1, 3, 600.0, 30.0, 0.0));
    net.addTrack(Track(103, 2, 4, 200.0, 30.0, 0.0));
    net.addTrack(Track(104, 3, 4, 100.0, 30.0, 0.0));
    return net;
}

} // namespace

// ============================================================
// Basic routing
// ============================================================

TEST(RouteNavigatorTest, LinearRouteOneHop)
{
    const RailwayNetwork net = makeLinearNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 2);

    EXPECT_TRUE(result.success);
    ASSERT_EQ(result.tracks.size(), 1u);
    EXPECT_EQ(result.tracks[0], 101u);
    EXPECT_DOUBLE_EQ(result.totalDistance, 1000.0);
    EXPECT_EQ(result.reason, RouteResult::FailReason::None);
}

TEST(RouteNavigatorTest, LinearRouteTwoHops)
{
    const RailwayNetwork net = makeLinearNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 3);

    EXPECT_TRUE(result.success);
    ASSERT_EQ(result.tracks.size(), 2u);
    EXPECT_EQ(result.tracks[0], 101u);
    EXPECT_EQ(result.tracks[1], 102u);
    EXPECT_DOUBLE_EQ(result.totalDistance, 1500.0);
}

TEST(RouteNavigatorTest, DiamondPicksShortestPath)
{
    const RailwayNetwork net = makeDiamondNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 4);

    EXPECT_TRUE(result.success);
    // Shortest: T101 + T103 = 300m, not T102 + T104 = 700m
    EXPECT_DOUBLE_EQ(result.totalDistance, 300.0);
    ASSERT_EQ(result.tracks.size(), 2u);
    EXPECT_EQ(result.tracks[0], 101u);
    EXPECT_EQ(result.tracks[1], 103u);
}

TEST(RouteNavigatorTest, SecondHopDirectRoute)
{
    const RailwayNetwork net = makeLinearNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 2, 3);

    EXPECT_TRUE(result.success);
    ASSERT_EQ(result.tracks.size(), 1u);
    EXPECT_EQ(result.tracks[0], 102u);
    EXPECT_DOUBLE_EQ(result.totalDistance, 500.0);
}

// ============================================================
// Failure cases
// ============================================================

TEST(RouteNavigatorTest, SameNodeFails)
{
    const RailwayNetwork net = makeLinearNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 1);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, RouteResult::FailReason::SameNode);
    EXPECT_TRUE(result.tracks.empty());
    EXPECT_DOUBLE_EQ(result.totalDistance, 0.0);
}

TEST(RouteNavigatorTest, OriginNotFoundFails)
{
    const RailwayNetwork net = makeLinearNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 999, 3);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, RouteResult::FailReason::NodeNotFound);
}

TEST(RouteNavigatorTest, DestinationNotFoundFails)
{
    const RailwayNetwork net = makeLinearNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 999);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, RouteResult::FailReason::NodeNotFound);
}

TEST(RouteNavigatorTest, BothNodesNotFoundFails)
{
    const RailwayNetwork net = makeLinearNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 888, 999);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, RouteResult::FailReason::NodeNotFound);
}

TEST(RouteNavigatorTest, DirectedGraphNoReverseRoute)
{
    // Linear is one-directional: no route from N3 back to N1
    const RailwayNetwork net = makeLinearNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 3, 1);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, RouteResult::FailReason::NoPathExists);
}

TEST(RouteNavigatorTest, DisconnectedGraphFails)
{
    RailwayNetwork net;
    net.addNode(Node(1, "A", NodeType::Station));
    net.addNode(Node(2, "B", NodeType::Station));
    // No track between them
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 2);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, RouteResult::FailReason::NoPathExists);
}

// ============================================================
// Edge cases
// ============================================================

TEST(RouteNavigatorTest, SingleTrackNetwork)
{
    RailwayNetwork net;
    net.addNode(Node(1, "A", NodeType::Station));
    net.addNode(Node(2, "B", NodeType::Station));
    net.addTrack(Track(1, 1, 2, 250.0, 20.0, 0.0));
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 2);

    EXPECT_TRUE(result.success);
    ASSERT_EQ(result.tracks.size(), 1u);
    EXPECT_DOUBLE_EQ(result.totalDistance, 250.0);
}

TEST(RouteNavigatorTest, LongChainFiveNodes)
{
    RailwayNetwork net;
    for (NodeId i = 1; i <= 5; ++i)
    {
        net.addNode(Node(i, "N" + std::to_string(i), NodeType::Station));
    }
    for (TrackId i = 1; i <= 4; ++i)
    {
        net.addTrack(Track(i * 100, i, i + 1, 300.0, 25.0, 0.0));
    }
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 5);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.tracks.size(), 4u);
    EXPECT_DOUBLE_EQ(result.totalDistance, 1200.0);
}

TEST(RouteNavigatorTest, BidirectionalNetworkRoundTrip)
{
    // Add forward and reverse tracks
    RailwayNetwork net;
    net.addNode(Node(1, "A", NodeType::Station));
    net.addNode(Node(2, "B", NodeType::Station));
    net.addTrack(Track(101, 1, 2, 500.0, 30.0, 0.0));
    net.addTrack(Track(102, 2, 1, 500.0, 30.0, 0.0));
    RouteNavigator nav;

    const RouteResult fwd = nav.findRoute(net, 1, 2);
    const RouteResult rev = nav.findRoute(net, 2, 1);

    EXPECT_TRUE(fwd.success);
    EXPECT_TRUE(rev.success);
    EXPECT_DOUBLE_EQ(fwd.totalDistance, 500.0);
    EXPECT_DOUBLE_EQ(rev.totalDistance, 500.0);
    EXPECT_EQ(fwd.tracks[0], 101u);
    EXPECT_EQ(rev.tracks[0], 102u);
}

TEST(RouteNavigatorTest, ParallelTracksPicksShorter)
{
    // Two direct tracks N1->N2, one short, one long
    RailwayNetwork net;
    net.addNode(Node(1, "A", NodeType::Station));
    net.addNode(Node(2, "B", NodeType::Station));
    net.addTrack(Track(101, 1, 2, 1000.0, 30.0, 0.0));
    net.addTrack(Track(102, 1, 2,  200.0, 30.0, 0.0));
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 2);

    EXPECT_TRUE(result.success);
    // Dijkstra picks shortest: T102 = 200m
    EXPECT_DOUBLE_EQ(result.totalDistance, 200.0);
    ASSERT_EQ(result.tracks.size(), 1u);
    EXPECT_EQ(result.tracks[0], 102u);
}

TEST(RouteNavigatorTest, EmptyTracksResultIsOrdered)
{
    // Verify tracks vector is always origin-to-destination order
    RailwayNetwork net;
    net.addNode(Node(1, "A", NodeType::Station));
    net.addNode(Node(2, "B", NodeType::Junction));
    net.addNode(Node(3, "C", NodeType::Station));
    net.addTrack(Track(201, 1, 2, 100.0, 20.0, 0.0));
    net.addTrack(Track(202, 2, 3,  50.0, 20.0, 0.0));
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 3);

    ASSERT_EQ(result.tracks.size(), 2u);
    // First track must start at node 1, second must start at node 2
    const auto* t1 = net.getTrack(result.tracks[0]);
    const auto* t2 = net.getTrack(result.tracks[1]);
    ASSERT_NE(t1, nullptr);
    ASSERT_NE(t2, nullptr);
    EXPECT_EQ(t1->source(), 1u);
    EXPECT_EQ(t2->source(), 2u);
}

TEST(RouteNavigatorTest, TotalDistanceIsAccurate)
{
    RailwayNetwork net;
    net.addNode(Node(1, "A", NodeType::Station));
    net.addNode(Node(2, "B", NodeType::Station));
    net.addNode(Node(3, "C", NodeType::Station));
    net.addTrack(Track(101, 1, 2,  123.4, 30.0, 0.0));
    net.addTrack(Track(102, 2, 3,  456.7, 30.0, 0.0));
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 3);

    EXPECT_TRUE(result.success);
    EXPECT_NEAR(result.totalDistance, 580.1, 1e-9);
}
