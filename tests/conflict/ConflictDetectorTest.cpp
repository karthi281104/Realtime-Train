#include "conflict/ConflictDetector.hpp"

#include "infrastructure/Node.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "infrastructure/Track.hpp"

#include <gtest/gtest.h>

namespace tcas::conflict
{
namespace
{

infrastructure::RailwayNetwork makeNetwork()
{
    infrastructure::RailwayNetwork network;
    EXPECT_TRUE(network.addNode({1, "A", infrastructure::NodeType::Generic}));
    EXPECT_TRUE(network.addNode({2, "J1", infrastructure::NodeType::Junction}));
    EXPECT_TRUE(network.addNode({3, "B", infrastructure::NodeType::Generic}));
    EXPECT_TRUE(network.addNode({4, "P1", infrastructure::NodeType::Platform}));
    EXPECT_TRUE(network.addNode({5, "C", infrastructure::NodeType::Generic}));
    EXPECT_TRUE(network.addTrack({101, 1, 2, 1000.0, 50.0, 0.0}));
    EXPECT_TRUE(network.addTrack({102, 2, 3, 1000.0, 50.0, 0.0}));
    EXPECT_TRUE(network.addTrack({103, 1, 4, 500.0, 30.0, 0.0}));
    EXPECT_TRUE(network.addTrack({104, 4, 5, 500.0, 30.0, 0.0}));
    return network;
}

prediction::FutureState state(
    TimeSeconds time,
    TrackId track,
    DistanceMeters position,
    SpeedMetersPerSecond velocity,
    DistanceMeters uncertainty = 0.0)
{
    return {time, track, position, velocity, 0.0, uncertainty};
}

} // namespace

TEST(ConflictDetectorTest, DetectsRearEndConflict)
{
    const auto network = makeNetwork();
    ConflictDetector detector({10.0, 5.0, 2.0});

    const auto conflicts = detector.detect(
        1,
        {state(0.0, 101, 100.0, 20.0), state(10.0, 101, 300.0, 20.0)},
        2,
        {state(0.0, 101, 140.0, 10.0), state(10.0, 101, 240.0, 10.0)},
        network);

    ASSERT_EQ(conflicts.size(), 1U);
    EXPECT_EQ(conflicts[0].type, ConflictType::RearEnd);
    EXPECT_EQ(conflicts[0].trackId, 101U);
    EXPECT_LE(conflicts[0].minimumSeparation, 10.0);
}

TEST(ConflictDetectorTest, DetectsHeadOnConflictWhenVelocityIsOpposite)
{
    const auto network = makeNetwork();
    ConflictDetector detector({10.0, 5.0, 2.0});

    const auto conflicts = detector.detect(
        1,
        {state(0.0, 101, 100.0, 20.0), state(10.0, 101, 300.0, 20.0)},
        2,
        {state(0.0, 101, 300.0, -20.0), state(10.0, 101, 100.0, -20.0)},
        network);

    ASSERT_EQ(conflicts.size(), 1U);
    EXPECT_EQ(conflicts[0].type, ConflictType::HeadOn);
}

TEST(ConflictDetectorTest, DetectsJunctionConflictFromCommonNode)
{
    const auto network = makeNetwork();
    ConflictDetector detector({10.0, 5.0, 2.0});

    const auto conflicts = detector.detect(
        1,
        {state(0.0, 101, 900.0, 20.0), state(5.0, 102, 0.0, 20.0)},
        2,
        {state(0.0, 103, 350.0, 30.0), state(5.0, 104, 0.0, 30.0)},
        network);

    ASSERT_EQ(conflicts.size(), 1U);
    EXPECT_EQ(conflicts[0].type, ConflictType::Junction);
    EXPECT_EQ(conflicts[0].resourceNodeId, 2U);
}

TEST(ConflictDetectorTest, DetectsPlatformConflictFromCommonNode)
{
    const auto network = makeNetwork();
    ConflictDetector detector({10.0, 5.0, 2.0});

    const auto conflicts = detector.detect(
        1,
        {state(0.0, 103, 400.0, 20.0), state(5.0, 104, 0.0, 20.0)},
        2,
        {state(0.0, 103, 300.0, 20.0), state(5.0, 104, 0.0, 20.0)},
        network);

    ASSERT_EQ(conflicts.size(), 1U);
    EXPECT_EQ(conflicts[0].type, ConflictType::Platform);
    EXPECT_EQ(conflicts[0].resourceNodeId, 4U);
}

TEST(ConflictDetectorTest, DoesNotDetectDifferentTracks)
{
    const auto network = makeNetwork();
    ConflictDetector detector;

    const auto conflicts = detector.detect(
        1,
        {state(0.0, 101, 100.0, 10.0), state(10.0, 101, 200.0, 10.0)},
        2,
        {state(0.0, 103, 100.0, 10.0), state(10.0, 103, 200.0, 10.0)},
        network);

    EXPECT_TRUE(conflicts.empty());
}

TEST(ConflictDetectorTest, AccountsForPredictionUncertainty)
{
    const auto network = makeNetwork();
    ConflictDetector detector({10.0, 5.0, 2.0});

    const auto conflicts = detector.detect(
        1,
        {state(0.0, 101, 100.0, 10.0, 10.0), state(10.0, 101, 200.0, 10.0, 10.0)},
        2,
        {state(0.0, 101, 125.0, 10.0, 10.0), state(10.0, 101, 225.0, 10.0, 10.0)},
        network);

    ASSERT_EQ(conflicts.size(), 1U);
}

TEST(ConflictDetectorTest, RejectsSameTrain)
{
    const auto network = makeNetwork();
    ConflictDetector detector;

    EXPECT_THROW(
        detector.detect(
            1,
            {state(0.0, 101, 0.0, 10.0)},
            1,
            {state(0.0, 101, 0.0, 10.0)},
            network),
        std::invalid_argument);
}

TEST(ResourceReservationManagerTest, GrantsNonOverlappingReservations)
{
    ResourceReservationManager manager;
    const ConflictZone zone{ConflictZoneType::Junction, 2, 0};

    EXPECT_TRUE(manager.request(1, zone, 10.0, 20.0));
    EXPECT_TRUE(manager.request(2, zone, 20.0, 30.0));
}

TEST(ResourceReservationManagerTest, RejectsOverlappingReservation)
{
    ResourceReservationManager manager;
    const ConflictZone zone{ConflictZoneType::Junction, 2, 0};

    EXPECT_TRUE(manager.request(1, zone, 10.0, 20.0));
    EXPECT_FALSE(manager.request(2, zone, 19.0, 30.0));
    EXPECT_TRUE(manager.request(1, zone, 19.0, 30.0));
}

TEST(ResourceReservationManagerTest, ReleasesReservation)
{
    ResourceReservationManager manager;
    const ConflictZone zone{ConflictZoneType::Junction, 2, 0};

    ASSERT_TRUE(manager.request(1, zone, 10.0, 20.0));
    EXPECT_TRUE(manager.release(1, zone));
    EXPECT_TRUE(manager.request(2, zone, 10.0, 20.0));
}

} // namespace tcas::conflict
