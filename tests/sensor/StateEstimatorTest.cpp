#include "sensor/StateEstimator.hpp"

#include <gtest/gtest.h>

using namespace tcas;
using namespace tcas::sensor;

// ============================================================
// StateEstimator Tests
// ============================================================

TEST(StateEstimatorTest, InitialStateIsAccurate)
{
    StateEstimator estimator({}, 100.0, 25.0);

    EXPECT_DOUBLE_EQ(estimator.position(), 100.0);
    EXPECT_DOUBLE_EQ(estimator.velocity(), 25.0);
    EXPECT_DOUBLE_EQ(estimator.acceleration(), 0.0);
    EXPECT_FALSE(estimator.isDegraded());
    EXPECT_GT(estimator.positionUncertainty(), 0.0);
    EXPECT_GT(estimator.velocityUncertainty(), 0.0);
}

TEST(StateEstimatorTest, PredictAdvancesPositionWithConstantVelocity)
{
    StateEstimator estimator({}, 0.0, 20.0);

    // After 1 second at 20 m/s: pos = 20 m, vel = 20 m/s
    estimator.predict(1.0, 50);

    EXPECT_NEAR(estimator.position(), 20.0, 1e-6);
    EXPECT_NEAR(estimator.velocity(), 20.0, 1e-6);
    EXPECT_EQ(estimator.estimatedState().timestamp, 50u);
}

TEST(StateEstimatorTest, PredictZeroDtLeavesStateUnchanged)
{
    StateEstimator estimator({}, 50.0, 15.0);
    estimator.predict(0.0, 1);

    EXPECT_DOUBLE_EQ(estimator.position(), 50.0);
    EXPECT_DOUBLE_EQ(estimator.velocity(), 15.0);
}

TEST(StateEstimatorTest, PredictGrowsCovarianceOverTime)
{
    StateEstimator estimator({}, 0.0, 10.0);
    const double initialUncertainty = estimator.positionUncertainty();

    // After 10 predictions with no update, uncertainty should grow
    for (int i = 0; i < 10; ++i)
    {
        estimator.predict(0.1, static_cast<SimTimeTick>(i));
    }

    EXPECT_GT(estimator.positionUncertainty(), initialUncertainty);
}

TEST(StateEstimatorTest, UpdateOdometryFusesMeasurements)
{
    StateEstimator estimator({}, 10.0, 5.0);

    // Initial uncertainty
    const double initialPosUncertainty = estimator.positionUncertainty();

    OdometerMeasurement meas{
        .rawPosition     = 10.8,
        .rawVelocity     = 5.2,
        .rawAcceleration = 0.0,
        .timestamp       = 1,
        .isValid         = true
    };

    const bool updated = estimator.updateOdometry(meas);
    EXPECT_TRUE(updated);
    EXPECT_GT(estimator.position(), 10.0);
    EXPECT_LT(estimator.positionUncertainty(), initialPosUncertainty);
}

TEST(StateEstimatorTest, UpdateOdometryReducesUncertainty)
{
    StateEstimator estimator({}, 0.0, 20.0);

    // Grow uncertainty via predictions
    for (int i = 0; i < 5; ++i)
    {
        estimator.predict(0.1, static_cast<SimTimeTick>(i));
    }

    const double uncertaintyBeforeUpdate = estimator.positionUncertainty();

    OdometerMeasurement meas{
        .rawPosition     = estimator.position(),
        .rawVelocity     = 20.0,
        .rawAcceleration = 0.0,
        .timestamp       = 5,
        .isValid         = true
    };

    estimator.updateOdometry(meas);
    EXPECT_LT(estimator.positionUncertainty(), uncertaintyBeforeUpdate);
}

TEST(StateEstimatorTest, BaliseUpdateReducesPositionUncertaintySignificantly)
{
    StateEstimator estimator({}, 1000.0, 30.0);

    // Predict several steps to grow covariance uncertainty
    for (int i = 0; i < 20; ++i)
    {
        estimator.predict(0.1, static_cast<SimTimeTick>(i));
    }

    const double highUncertainty = estimator.positionUncertainty();

    // Pass absolute physical balise at exact position 1060.0
    BaliseTransponder balise{
        .baliseId      = 42,
        .trackId       = 101,
        .exactPosition = 1060.0
    };

    const bool updated = estimator.updateBalise(balise);
    EXPECT_TRUE(updated);
    EXPECT_NEAR(estimator.position(), 1060.0, 0.1);
    EXPECT_LT(estimator.positionUncertainty(), highUncertainty);
}

TEST(StateEstimatorTest, BaliseUpdateClearsOutlierCountAndDegradedState)
{
    StateEstimator estimator({}, 100.0, 20.0);

    // Push 3 invalid measurements to trigger degradation
    OdometerMeasurement invalid{
        .rawPosition = 0.0, .rawVelocity = 0.0, .rawAcceleration = 0.0,
        .timestamp = 1, .isValid = false
    };
    estimator.updateOdometry(invalid);
    EXPECT_TRUE(estimator.isDegraded());

    // Balise fix should clear degraded state
    BaliseTransponder balise{ .baliseId = 1, .trackId = 101, .exactPosition = 100.0 };
    estimator.updateBalise(balise);
    EXPECT_FALSE(estimator.isDegraded());
}

TEST(StateEstimatorTest, RejectsStatisticalOutliers)
{
    StateEstimator estimator({}, 100.0, 20.0);

    // Impossible sensor jump (e.g. 50,000 m away)
    OdometerMeasurement spike{
        .rawPosition     = 50000.0,
        .rawVelocity     = 20.0,
        .rawAcceleration = 0.0,
        .timestamp       = 10,
        .isValid         = true
    };

    const bool accepted = estimator.updateOdometry(spike);
    EXPECT_FALSE(accepted);
    // Estimated position should remain near 100.0, not jumping to 50000.0
    EXPECT_NEAR(estimator.position(), 100.0, 5.0);
}

TEST(StateEstimatorTest, DegradesAfterInvalidMeasurement)
{
    StateEstimator estimator({}, 50.0, 10.0);

    OdometerMeasurement invalidMeas{
        .rawPosition     = 0.0,
        .rawVelocity     = 0.0,
        .rawAcceleration = 0.0,
        .timestamp       = 1,
        .isValid         = false
    };

    EXPECT_FALSE(estimator.updateOdometry(invalidMeas));
    EXPECT_TRUE(estimator.isDegraded());
}

TEST(StateEstimatorTest, DegradesAfterConsecutiveOutliers)
{
    StateEstimator estimator({}, 50.0, 10.0);

    OdometerMeasurement spike{
        .rawPosition = 99999.0, .rawVelocity = 10.0, .rawAcceleration = 0.0,
        .timestamp = 1, .isValid = true
    };

    // Need >= 3 consecutive outliers to trigger degradation
    estimator.updateOdometry(spike);
    estimator.updateOdometry(spike);
    EXPECT_FALSE(estimator.isDegraded()); // 2 outliers: not yet

    estimator.updateOdometry(spike);
    EXPECT_TRUE(estimator.isDegraded());  // 3 outliers: now degraded
}

TEST(StateEstimatorTest, ResetRestoresState)
{
    StateEstimator estimator({}, 200.0, 30.0);
    estimator.predict(2.0, 100);

    estimator.reset(0.0, 0.0);

    EXPECT_DOUBLE_EQ(estimator.position(), 0.0);
    EXPECT_DOUBLE_EQ(estimator.velocity(), 0.0);
    EXPECT_FALSE(estimator.isDegraded());
}

TEST(StateEstimatorTest, EstimatedStateReflectsTimestamp)
{
    StateEstimator estimator({}, 0.0, 5.0);
    estimator.predict(0.5, 42);

    const auto state = estimator.estimatedState();
    EXPECT_EQ(state.timestamp, 42u);
}
