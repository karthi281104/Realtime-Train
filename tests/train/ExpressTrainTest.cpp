#include "train/ExpressTrain.hpp"

#include <gtest/gtest.h>

using namespace tcas;
using namespace tcas::train;

TEST(ExpressTrainTest, HasExpressType)
{
    ExpressTrain train(
        101,
        50000.0,
        45.0,
        0.8,
        1.2
    );

    EXPECT_EQ(train.type(), TrainType::Express);
}

TEST(ExpressTrainTest, SupportsPolymorphism)
{
    ExpressTrain train(
        101,
        50000.0,
        45.0,
        0.8,
        1.2
    );

    Train* base = &train;

    EXPECT_EQ(base->type(), TrainType::Express);
    EXPECT_EQ(base->id(), 101);
}