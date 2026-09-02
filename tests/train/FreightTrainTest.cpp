#include <gtest/gtest.h>

#include "train/FreightTrain.hpp"

using namespace tcas;
using namespace tcas::train;

TEST(FreightTrainTest, HasFreightType) {
  FreightTrain train(301, 500000.0, 25.0, 0.5, 0.8);

  EXPECT_EQ(train.type(), TrainType::Freight);
}

TEST(FreightTrainTest, SupportsPolymorphism) {
  FreightTrain train(301, 500000.0, 25.0, 0.5, 0.8);

  const Train* base = &train;

  EXPECT_EQ(base->type(), TrainType::Freight);
}
