#include "physics/KinematicsEngine.hpp"
#include "navigation/RouteNavigator.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "infrastructure/Node.hpp"
#include "infrastructure/Track.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "simulation/SimClock.hpp"

#include <gtest/gtest.h>
#include <numeric>

using namespace tcas;
using namespace tcas::physics;
using namespace tcas::navigation;
using namespace tcas::infrastructure;
using namespace tcas::train;
using namespace tcas::simulation;

namespace
{

// Standard test railway:
//
//   A --(T101: 2000m, 0% grade)---> B
//   B --(T102: 1000m, +2% grade)--> C  (uphill)
//   B --(T103: 3000m, -1% grade)--> D  (downhill, longer)
//
RailwayNetwork makeTestNetwork()
{
    RailwayNetwork net;
    net.addNode(Node(1, "A", NodeType::Station));
    net.addNode(Node(2, "B", NodeType::Junction));
    net.addNode(Node(3, "C", NodeType::Station));
    net.addNode(Node(4, "D", NodeType::Station));
    net.addTrack(Track(101, 1, 2, 2000.0, 33.3, 0.00));
    net.addTrack(Track(102, 2, 3, 1000.0, 25.0, 0.02));
    net.addTrack(Track(103, 2, 4, 3000.0, 20.0, -0.01));
    return net;
}

} // namespace

// ============================================================
// Integration: physics over a routed path
// ============================================================

TEST(PhysicsNavigationIntegrationTest, RouteExistsAndPhysicsIsPositive)
{
    const RailwayNetwork net = makeTestNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 3);

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.totalDistance, 0.0);
}

TEST(PhysicsNavigationIntegrationTest, SafeDistanceOverRouteTrack)
{
    const RailwayNetwork net = makeTestNetwork();
    RouteNavigator nav;

    // Find route A -> C (through B uphill)
    const RouteResult result = nav.findRoute(net, 1, 3);
    ASSERT_TRUE(result.success);

    // Use an express train at 80 km/h (22.2 m/s)
    const SpeedMetersPerSecond v = 22.2;
    const AccelerationMetersPerSecondSquared braking = 0.8;

    // For each track in route, compute safe distance
    for (const TrackId tid : result.tracks)
    {
        const auto* track = net.getTrack(tid);
        ASSERT_NE(track, nullptr);

        const double safe = KinematicsEngine::safeDistance(
            v, braking, track->gradient()
        );

        EXPECT_GT(safe, 0.0);
    }
}

TEST(PhysicsNavigationIntegrationTest, UphillRouteSafeDistanceShorterThanFlat)
{
    const SpeedMetersPerSecond v      = 25.0;
    const double braking               = 0.8;
    const double kDefaultReaction      = KinematicsEngine::kDefaultReactionTime;
    const double kDefaultMargin        = KinematicsEngine::kDefaultSafetyMargin;

    const double safeFlat   = KinematicsEngine::safeDistance(v, braking, 0.00, kDefaultReaction, kDefaultMargin);
    const double safeUphill = KinematicsEngine::safeDistance(v, braking, 0.02, kDefaultReaction, kDefaultMargin);

    EXPECT_LT(safeUphill, safeFlat);
}

TEST(PhysicsNavigationIntegrationTest, DownhillRouteSafeDistanceLongerThanFlat)
{
    const SpeedMetersPerSecond v = 25.0;
    const double braking          = 0.8;

    const double safeFlat     = KinematicsEngine::safeDistance(v, braking, 0.00);
    const double safeDownhill = KinematicsEngine::safeDistance(v, braking, -0.01);

    EXPECT_GT(safeDownhill, safeFlat);
}

TEST(PhysicsNavigationIntegrationTest, RouteTotalDistanceMatchesTrackLengthSum)
{
    const RailwayNetwork net = makeTestNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 3);
    ASSERT_TRUE(result.success);

    double manualSum = 0.0;
    for (const TrackId tid : result.tracks)
    {
        const auto* track = net.getTrack(tid);
        ASSERT_NE(track, nullptr);
        manualSum += track->length();
    }

    EXPECT_DOUBLE_EQ(result.totalDistance, manualSum);
}

TEST(PhysicsNavigationIntegrationTest, DijkstraPicksShorterOfTwoRoutes)
{
    // A->C (3000m) vs A->D (5000m) — Dijkstra picks A->C
    const RailwayNetwork net = makeTestNetwork();
    RouteNavigator nav;

    const RouteResult toC = nav.findRoute(net, 1, 3);
    const RouteResult toD = nav.findRoute(net, 1, 4);

    EXPECT_TRUE(toC.success);
    EXPECT_TRUE(toD.success);
    EXPECT_LT(toC.totalDistance, toD.totalDistance);
}

TEST(PhysicsNavigationIntegrationTest, FreightTrainBrakingDistanceLongerDueToEmergency)
{
    // FreightTrain has lower emergency braking => longer stopping distance
    FreightTrain freight(1, 80000.0, 22.0, 0.5, 0.9);
    ExpressTrain express(2, 40000.0, 55.0, 0.8, 1.2);

    const double v = 20.0;
    const double grad = 0.0;

    const double freightStop  = KinematicsEngine::emergencyStoppingDistance(
        v, freight.emergencyBraking(), grad
    );
    const double expressStop  = KinematicsEngine::emergencyStoppingDistance(
        v, express.emergencyBraking(), grad
    );

    EXPECT_GT(freightStop, expressStop);
}

TEST(PhysicsNavigationIntegrationTest, SimClockDtMatchesPhysicsUpdate)
{
    SimClock clock(0.020);  // 20 ms

    const SpeedMetersPerSecond   v0  = 20.0;
    const AccelerationMetersPerSecondSquared a = 1.0;
    const SpeedMetersPerSecond   vMax = 50.0;

    clock.tick();

    const double dt = clock.dt();
    const double v1 = KinematicsEngine::updateVelocity(v0, a, dt, vMax);

    // v1 = 20 + 1 * 0.02 = 20.02
    EXPECT_NEAR(v1, 20.02, 1e-9);
}

TEST(PhysicsNavigationIntegrationTest, PositionAfterNTicksMatchesAnalytic)
{
    SimClock clock(0.020);

    const SpeedMetersPerSecond v  = 20.0;    // constant (a=0)
    const double               a  = 0.0;
    double                     x  = 0.0;
    const int                  N  = 100;

    for (int i = 0; i < N; ++i)
    {
        clock.tick();
        x = KinematicsEngine::updatePosition(x, v, a, clock.dt());
    }

    // Expected: 20 * 100 * 0.02 = 40.0 m
    EXPECT_NEAR(x, 40.0, 1e-9);
}

TEST(PhysicsNavigationIntegrationTest, RouteTracksConnectCorrectly)
{
    const RailwayNetwork net = makeTestNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 3);
    ASSERT_TRUE(result.success);
    ASSERT_GE(result.tracks.size(), 2u);

    // Each successive track must connect: prev.destination == next.source
    for (std::size_t i = 0; i + 1 < result.tracks.size(); ++i)
    {
        const auto* prev = net.getTrack(result.tracks[i]);
        const auto* next = net.getTrack(result.tracks[i + 1]);
        ASSERT_NE(prev, nullptr);
        ASSERT_NE(next, nullptr);
        EXPECT_EQ(prev->destination(), next->source());
    }
}

TEST(PhysicsNavigationIntegrationTest, SafeDistanceExceedsTrackLengthWarning)
{
    // If safe stopping distance > track length, train cannot stop in time
    // (informational integration test — no crash expected)
    RailwayNetwork net;
    net.addNode(Node(1, "A", NodeType::Station));
    net.addNode(Node(2, "B", NodeType::Station));
    net.addTrack(Track(1, 1, 2, 100.0, 30.0, 0.0));  // short track

    RouteNavigator nav;
    const RouteResult result = nav.findRoute(net, 1, 2);
    ASSERT_TRUE(result.success);

    const auto* track = net.getTrack(result.tracks[0]);
    const double safeStop = KinematicsEngine::safeDistance(
        30.0, 0.8, track->gradient()
    );

    // In this test the safe distance will exceed the track length
    EXPECT_GT(safeStop, track->length());
}

TEST(PhysicsNavigationIntegrationTest, SpeedAfterRouteFirstTrack)
{
    // Compute expected speed after travelling 2000m under service deceleration
    const double v0     = 30.0;
    const double a      = -0.8;   // braking
    const double dist   = 2000.0;

    const double vFinal = KinematicsEngine::speedAfterDistance(v0, a, dist);

    // v^2 = 900 + 2*(-0.8)*2000 = 900 - 3200 < 0 => stopped
    EXPECT_DOUBLE_EQ(vFinal, 0.0);
}

TEST(PhysicsNavigationIntegrationTest, EmergencyStoppingOnRouteTrack)
{
    const RailwayNetwork net = makeTestNetwork();
    RouteNavigator nav;

    const RouteResult result = nav.findRoute(net, 1, 4);  // A -> D (downhill)
    ASSERT_TRUE(result.success);

    const auto* firstTrack = net.getTrack(result.tracks[0]);
    ASSERT_NE(firstTrack, nullptr);

    // Emergency braking from 33 m/s on first track
    const double emergencyDist = KinematicsEngine::emergencyStoppingDistance(
        33.0, 1.2, firstTrack->gradient()
    );

    // Must be a positive finite distance
    EXPECT_GT(emergencyDist, 0.0);
    EXPECT_LT(emergencyDist, 1e6);
}

TEST(PhysicsNavigationIntegrationTest, RouteNavigatorIsStateless)
{
    // Call findRoute multiple times — should return identical results
    const RailwayNetwork net = makeTestNetwork();
    RouteNavigator nav;

    const RouteResult r1 = nav.findRoute(net, 1, 3);
    const RouteResult r2 = nav.findRoute(net, 1, 3);

    EXPECT_EQ(r1.success, r2.success);
    EXPECT_EQ(r1.totalDistance, r2.totalDistance);
    EXPECT_EQ(r1.tracks.size(), r2.tracks.size());
}
