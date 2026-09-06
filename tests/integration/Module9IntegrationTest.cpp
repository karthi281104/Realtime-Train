#include "conflict/ConflictDetector.hpp"
#include "conflict/ResourceReservationManager.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "navigation/RouteNavigator.hpp"
#include "prediction/FutureState.hpp"

#include <gtest/gtest.h>

namespace tcas::integration
{
namespace
{

using infrastructure::Node;
using infrastructure::NodeType;
using infrastructure::RailwayNetwork;
using infrastructure::Track;
using conflict::ConflictDetector;
using conflict::ConflictType;
using conflict::ConflictZone;
using conflict::ConflictZoneType;
using conflict::ResourceReservationManager;
using prediction::FutureState;

RailwayNetwork makeJunctionNetwork()
{
    RailwayNetwork network;
    network.addNode(Node(1, "Express Origin", NodeType::Generic));
    network.addNode(Node(2, "J1", NodeType::Junction));
    network.addNode(Node(3, "Express Destination", NodeType::Generic));
    network.addNode(Node(4, "Freight Origin", NodeType::Generic));
    network.addNode(Node(5, "Freight Destination", NodeType::Generic));

    network.addTrack(Track(101, 1, 2, 1000.0, 30.0, 0.0));
    network.addTrack(Track(102, 2, 3, 1000.0, 30.0, 0.0));
    network.addTrack(Track(103, 4, 2, 1000.0, 30.0, 0.0));
    network.addTrack(Track(104, 2, 5, 1000.0, 30.0, 0.0));
    return network;
}

std::vector<FutureState> trajectory(
    TrackId firstTrack,
    TrackId secondTrack,
    TimeSeconds junctionTime)
{
    return {
        { 0.0, firstTrack, 0.0, 20.0, 0.0, 1.0 },
        { junctionTime, secondTrack, 0.0, 20.0, 0.0, 1.0 },
        { junctionTime + 10.0, secondTrack, 200.0, 20.0, 0.0, 1.0 }
    };
}

TEST(Module9Integration, RoutePredictionConflictAndReservation)
{
    const RailwayNetwork network = makeJunctionNetwork();

    const auto expressRoute = navigation::RouteNavigator::findRoute(network, 1, 3);
    const auto freightRoute = navigation::RouteNavigator::findRoute(network, 4, 5);

    ASSERT_TRUE(expressRoute.success);
    ASSERT_TRUE(freightRoute.success);
    ASSERT_EQ(expressRoute.tracks.front(), 101u);
    ASSERT_EQ(freightRoute.tracks.front(), 103u);

    const auto expressTrajectory = trajectory(101, 102, 20.0);
    const auto freightTrajectory = trajectory(103, 104, 21.0);

    ConflictDetector detector({ 50.0, 5.0, 2.0 });
    const auto conflicts = detector.detect(
        1, expressTrajectory, 2, freightTrajectory, network);

    ASSERT_EQ(conflicts.size(), 1u);
    EXPECT_EQ(conflicts.front().type, ConflictType::Junction);
    EXPECT_EQ(conflicts.front().resourceNodeId, 2u);
    EXPECT_LE(conflicts.front().firstConflictTime, 20.0);
    EXPECT_GE(conflicts.front().lastConflictTime, 23.0);

    ResourceReservationManager reservations;
    const ConflictZone j1{ ConflictZoneType::Junction, 2, 0 };

    EXPECT_TRUE(reservations.request(1, j1, 20.0, 24.0));
    EXPECT_FALSE(reservations.request(2, j1, 21.0, 25.0));
    EXPECT_TRUE(reservations.release(1, j1));
    reservations.clearReleased();
    EXPECT_TRUE(reservations.request(2, j1, 21.0, 25.0));
}

TEST(Module9Integration, SameJunctionAtDifferentTimesIsSafe)
{
    const RailwayNetwork network = makeJunctionNetwork();
    const auto expressTrajectory = trajectory(101, 102, 20.0);
    const auto freightTrajectory = trajectory(103, 104, 40.0);

    ConflictDetector detector({ 50.0, 5.0, 2.0 });
    const auto conflicts = detector.detect(
        1, expressTrajectory, 2, freightTrajectory, network);

    EXPECT_TRUE(conflicts.empty());
}

} // namespace
} // namespace tcas::integration
