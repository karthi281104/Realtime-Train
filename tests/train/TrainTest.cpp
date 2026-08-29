#include "train/Train.hpp"

#include <gtest/gtest.h>

using namespace tcas;
using namespace tcas::train;

namespace
{

class TestTrain final : public Train
{
public:
    TestTrain(
        TrainId id,
        double mass,
        double maximumSpeed,
        double serviceBraking,
        double emergencyBraking
    )
        : Train(
            id,
            mass,
            maximumSpeed,
            serviceBraking,
            emergencyBraking
        )
    {
    }

    TrainType type() const noexcept override
    {
        return TrainType::Passenger;
    }
};

}

TEST(TrainTest, CreatesValidTrain)
{
    TestTrain train(
        1,
        50000.0,
        40.0,
        0.8,
        1.2
    );

    EXPECT_EQ(train.id(), 1);
    EXPECT_DOUBLE_EQ(train.mass(), 50000.0);
    EXPECT_DOUBLE_EQ(train.maximumSpeed(), 40.0);
    EXPECT_DOUBLE_EQ(train.serviceBraking(), 0.8);
    EXPECT_DOUBLE_EQ(train.emergencyBraking(), 1.2);
}

TEST(TrainTest, InitialStateIsIdle)
{
    TestTrain train(
        1,
        50000.0,
        40.0,
        0.8,
        1.2
    );

    EXPECT_EQ(train.state(), TrainState::Idle);
}

TEST(TrainTest, InitialMotionValuesAreZero)
{
    TestTrain train(
        1,
        50000.0,
        40.0,
        0.8,
        1.2
    );

    EXPECT_DOUBLE_EQ(train.position(), 0.0);
    EXPECT_DOUBLE_EQ(train.velocity(), 0.0);
    EXPECT_DOUBLE_EQ(train.acceleration(), 0.0);
}

TEST(TrainTest, RejectsZeroMass)
{
    EXPECT_THROW(
        TestTrain(1, 0.0, 40.0, 0.8, 1.2),
        std::invalid_argument
    );
}

TEST(TrainTest, RejectsNegativeMass)
{
    EXPECT_THROW(
        TestTrain(1, -100.0, 40.0, 0.8, 1.2),
        std::invalid_argument
    );
}

TEST(TrainTest, RejectsNegativeMaximumSpeed)
{
    EXPECT_THROW(
        TestTrain(1, 50000.0, -1.0, 0.8, 1.2),
        std::invalid_argument
    );
}

TEST(TrainTest, RejectsNegativeServiceBraking)
{
    EXPECT_THROW(
        TestTrain(1, 50000.0, 40.0, -0.8, 1.2),
        std::invalid_argument
    );
}

TEST(TrainTest, RejectsNegativeEmergencyBraking)
{
    EXPECT_THROW(
        TestTrain(1, 50000.0, 40.0, 0.8, -1.2),
        std::invalid_argument
    );
}

TEST(TrainTest, RejectsEmergencyBrakingLowerThanServiceBraking)
{
    EXPECT_THROW(
        TestTrain(1, 50000.0, 40.0, 1.2, 0.8),
        std::invalid_argument
    );
}

TEST(TrainTest, UpdatesPosition)
{
    TestTrain train(
        1,
        50000.0,
        40.0,
        0.8,
        1.2
    );

    train.setPosition(250.0);

    EXPECT_DOUBLE_EQ(train.position(), 250.0);
}

TEST(TrainTest, RejectsNegativePosition)
{
    TestTrain train(
        1,
        50000.0,
        40.0,
        0.8,
        1.2
    );

    EXPECT_THROW(
        train.setPosition(-1.0),
        std::invalid_argument
    );
}

TEST(TrainTest, UpdatesVelocity)
{
    TestTrain train(
        1,
        50000.0,
        40.0,
        0.8,
        1.2
    );

    train.setVelocity(25.0);

    EXPECT_DOUBLE_EQ(train.velocity(), 25.0);
}

TEST(TrainTest, RejectsVelocityAboveMaximum)
{
    TestTrain train(
        1,
        50000.0,
        40.0,
        0.8,
        1.2
    );

    EXPECT_THROW(
        train.setVelocity(50.0),
        std::invalid_argument
    );
}

TEST(TrainTest, UpdatesTrainState)
{
    TestTrain train(
        1,
        50000.0,
        40.0,
        0.8,
        1.2
    );

    train.setState(TrainState::Running);

    EXPECT_EQ(train.state(), TrainState::Running);
}