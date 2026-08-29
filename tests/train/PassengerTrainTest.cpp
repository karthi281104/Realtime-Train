#include "train/PassengerTrain.hpp"

#include <gtest/gtest.h>

using namespace tcas;
using namespace tcas::train;

TEST(PassengerTrainTest, HasPassengerType)
{
    PassengerTrain train(
        201,
        60000.0,
        35.0,
        0.7,
        1.1
    );

    EXPECT_EQ(train.type(), TrainType::Passenger);
}

TEST(PassengerTrainTest, SupportsPolymorphism)
{
    PassengerTrain train(
        201,
        60000.0,
        35.0,
        0.7,
        1.1
    );

    Train* base = &train;

    EXPECT_EQ(base->type(), TrainType::Passenger);
}