#include "prediction/PredictionEngine.hpp"

#include "infrastructure/Node.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "infrastructure/Track.hpp"
#include "navigation/RouteResult.hpp"
#include "train/ExpressTrain.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace tcas::prediction
{
namespace
{

infrastructure::RailwayNetwork makeSimpleNetwork()
{
    infrastructure::RailwayNetwork network;

    EXPECT_TRUE(
        network.addNode(
            infrastructure::Node(1, "A", infrastructure::NodeType::Generic)
        )
    );

    EXPECT_TRUE(
        network.addNode(
            infrastructure::Node(2, "B", infrastructure::NodeType::Generic)
        )
    );

    EXPECT_TRUE(
        network.addNode(
            infrastructure::Node(3, "C", infrastructure::NodeType::Generic)
        )
    );

    EXPECT_TRUE(
        network.addTrack(
            infrastructure::Track(
                101,
                1,
                2,
                1000.0,
                50.0,
                0.0
            )
        )
    );

    EXPECT_TRUE(
        network.addTrack(
            infrastructure::Track(
                102,
                2,
                3,
                1000.0,
                30.0,
                0.0
            )
        )
    );

    return network;
}

navigation::RouteResult makeSimpleRoute()
{
    navigation::RouteResult route;

    route.success = true;
    route.tracks = { 101, 102 };
    route.totalDistance = 2000.0;
    route.reason =
        navigation::RouteResult::FailReason::None;

    return route;
}

} // namespace

TEST(PredictionEngineTest, PredictsStandardHorizons)
{
    const auto network = makeSimpleNetwork();
    const auto route = makeSimpleRoute();

    train::ExpressTrain train(
        1,
        100000.0,
        60.0,
        0.8,
        1.2
    );

    train.setPosition(0.0);
    train.setVelocity(10.0);
    train.setAcceleration(0.0);

    const auto predictions =
        PredictionEngine::predictStandardHorizon(
            train,
            network,
            route,
            101
        );

    ASSERT_EQ(predictions.size(), 5U);

    EXPECT_DOUBLE_EQ(
        predictions[0].timestamp,
        5.0
    );

    EXPECT_DOUBLE_EQ(
        predictions[1].timestamp,
        10.0
    );

    EXPECT_DOUBLE_EQ(
        predictions[2].timestamp,
        20.0
    );

    EXPECT_DOUBLE_EQ(
        predictions[3].timestamp,
        30.0
    );

    EXPECT_DOUBLE_EQ(
        predictions[4].timestamp,
        60.0
    );
}

TEST(PredictionEngineTest, PredictsConstantVelocity)
{
    const auto network = makeSimpleNetwork();
    const auto route = makeSimpleRoute();

    train::ExpressTrain train(
        1,
        100000.0,
        60.0,
        0.8,
        1.2
    );

    train.setPosition(100.0);
    train.setVelocity(10.0);
    train.setAcceleration(0.0);

    const auto predictions =
        PredictionEngine::predict(
            train,
            network,
            route,
            101,
            { 5.0 }
        );

    ASSERT_EQ(predictions.size(), 1U);

    EXPECT_EQ(
        predictions[0].trackId,
        101U
    );

    EXPECT_NEAR(
        predictions[0].position,
        150.0,
        1e-9
    );

    EXPECT_NEAR(
        predictions[0].velocity,
        10.0,
        1e-9
    );
}

TEST(PredictionEngineTest, PredictsAccelerationFromRest)
{
    const auto network = makeSimpleNetwork();
    const auto route = makeSimpleRoute();

    train::ExpressTrain train(
        1,
        100000.0,
        60.0,
        0.8,
        1.2
    );

    train.setPosition(0.0);
    train.setVelocity(0.0);
    train.setAcceleration(1.0);

    const auto predictions =
        PredictionEngine::predict(
            train,
            network,
            route,
            101,
            { 1.0 }
        );

    ASSERT_EQ(predictions.size(), 1U);

    EXPECT_NEAR(
        predictions[0].position,
        0.5,
        1e-9
    );

    EXPECT_NEAR(
        predictions[0].velocity,
        1.0,
        1e-9
    );
}

TEST(PredictionEngineTest, RespectsTrackSpeedLimit)
{
    const auto network = makeSimpleNetwork();
    const auto route = makeSimpleRoute();

    train::ExpressTrain train(
        1,
        100000.0,
        100.0,
        0.8,
        1.2
    );

    train.setPosition(0.0);
    train.setVelocity(100.0);
    train.setAcceleration(0.0);

    const auto predictions =
        PredictionEngine::predict(
            train,
            network,
            route,
            101,
            { 1.0 }
        );

    ASSERT_EQ(predictions.size(), 1U);

    EXPECT_LE(
        predictions[0].velocity,
        50.0
    );
}

TEST(PredictionEngineTest, CrossesIntoNextTrack)
{
    const auto network = makeSimpleNetwork();
    const auto route = makeSimpleRoute();

    train::ExpressTrain train(
        1,
        100000.0,
        60.0,
        0.8,
        1.2
    );

    train.setPosition(900.0);
    train.setVelocity(20.0);
    train.setAcceleration(0.0);

    const auto predictions =
        PredictionEngine::predict(
            train,
            network,
            route,
            101,
            { 10.0 }
        );

    ASSERT_EQ(predictions.size(), 1U);

    EXPECT_EQ(
        predictions[0].trackId,
        102U
    );

    EXPECT_GT(
        predictions[0].position,
        0.0
    );
}

TEST(PredictionEngineTest, HandlesRouteTermination)
{
    const auto network = makeSimpleNetwork();

    navigation::RouteResult route;

    route.success = true;
    route.tracks = { 101 };
    route.totalDistance = 1000.0;

    train::ExpressTrain train(
        1,
        100000.0,
        60.0,
        0.8,
        1.2
    );

    train.setPosition(900.0);
    train.setVelocity(20.0);
    train.setAcceleration(0.0);

    const auto predictions =
        PredictionEngine::predict(
            train,
            network,
            route,
            101,
            { 20.0 }
        );

    ASSERT_EQ(predictions.size(), 1U);

    EXPECT_EQ(
        predictions[0].trackId,
        101U
    );

    EXPECT_DOUBLE_EQ(
        predictions[0].position,
        1000.0
    );

    EXPECT_DOUBLE_EQ(
        predictions[0].velocity,
        0.0
    );
}

TEST(PredictionEngineTest, UncertaintyIncreasesWithTime)
{
    const auto network = makeSimpleNetwork();
    const auto route = makeSimpleRoute();

    train::ExpressTrain train(
        1,
        100000.0,
        60.0,
        0.8,
        1.2
    );

    train.setPosition(0.0);
    train.setVelocity(10.0);
    train.setAcceleration(0.0);

    const auto predictions =
        PredictionEngine::predict(
            train,
            network,
            route,
            101,
            { 5.0, 10.0, 20.0 },
            2.0
        );

    ASSERT_EQ(predictions.size(), 3U);

    EXPECT_LT(
        predictions[0].uncertainty,
        predictions[1].uncertainty
    );

    EXPECT_LT(
        predictions[1].uncertainty,
        predictions[2].uncertainty
    );
}

TEST(PredictionEngineTest, RejectsInvalidRoute)
{
    const auto network = makeSimpleNetwork();

    navigation::RouteResult route;

    route.success = false;

    train::ExpressTrain train(
        1,
        100000.0,
        60.0,
        0.8,
        1.2
    );

    EXPECT_THROW(
        (void)PredictionEngine::predict(
            train,
            network,
            route,
            101,
            { 5.0 }
        ),
        std::invalid_argument
    );
}

TEST(PredictionEngineTest, RejectsUnknownCurrentTrack)
{
    const auto network = makeSimpleNetwork();
    const auto route = makeSimpleRoute();

    train::ExpressTrain train(
        1,
        100000.0,
        60.0,
        0.8,
        1.2
    );

    EXPECT_THROW(
        (void)PredictionEngine::predict(
            train,
            network,
            route,
            999,
            { 5.0 }
        ),
        std::invalid_argument
    );
}

TEST(PredictionEngineTest, ZeroVelocityWithoutAccelerationRemainsStopped)
{
    const auto network = makeSimpleNetwork();
    const auto route = makeSimpleRoute();

    train::ExpressTrain train(
        1,
        100000.0,
        60.0,
        0.8,
        1.2
    );

    train.setPosition(100.0);
    train.setVelocity(0.0);
    train.setAcceleration(0.0);

    const auto predictions =
        PredictionEngine::predict(
            train,
            network,
            route,
            101,
            { 5.0 }
        );

    ASSERT_EQ(predictions.size(), 1U);

    EXPECT_DOUBLE_EQ(
        predictions[0].position,
        100.0
    );

    EXPECT_DOUBLE_EQ(
        predictions[0].velocity,
        0.0
    );
}

} // namespace tcas::prediction
