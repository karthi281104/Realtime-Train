#include "safety/PriorityEngine.hpp"

#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"

#include <gtest/gtest.h>

namespace tcas::safety
{
namespace
{

train::ExpressTrain express()
{
    return {1, 45000.0, 45.0, 0.9, 1.4};
}

train::PassengerTrain passenger()
{
    return {2, 60000.0, 33.3, 0.8, 1.2};
}

train::FreightTrain freight()
{
    return {3, 120000.0, 22.2, 0.5, 0.8};
}

} // namespace

TEST(PriorityEngineTest, ExpressHasHighestPriority)
{
    EXPECT_EQ(PriorityEngine{}.assess(express()).priority, 3);
}

TEST(PriorityEngineTest, PassengerHasMediumPriority)
{
    EXPECT_EQ(PriorityEngine{}.assess(passenger()).priority, 2);
}

TEST(PriorityEngineTest, FreightHasLowestPriority)
{
    EXPECT_EQ(PriorityEngine{}.assess(freight()).priority, 1);
}

} // namespace tcas::safety
