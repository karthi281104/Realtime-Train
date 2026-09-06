#include "safety/ConflictPriorityQueue.hpp"

#include <gtest/gtest.h>

namespace tcas::safety
{
namespace
{

conflict::Conflict conflict(
    TrainId trainA,
    TrainId trainB,
    TimeSeconds firstTime)
{
    return {trainA, trainB, conflict::ConflictType::Junction, 0, 2,
            firstTime, firstTime + 2.0, 0.0};
}

RiskAssessment risk(double score)
{
    RiskAssessment assessment;
    assessment.score = score;
    assessment.level = RiskEngine::classify(score);
    return assessment;
}

} // namespace

TEST(ConflictPriorityQueueTest, HighestRiskComesFirst)
{
    ConflictPriorityQueue queue;
    queue.push(conflict(1, 2, 10.0), risk(40.0));
    queue.push(conflict(3, 4, 10.0), risk(80.0));

    EXPECT_EQ(queue.top().conflict.trainA, 3U);
}

TEST(ConflictPriorityQueueTest, EqualRiskUsesEarliestConflict)
{
    ConflictPriorityQueue queue;
    queue.push(conflict(1, 2, 20.0), risk(50.0));
    queue.push(conflict(3, 4, 10.0), risk(50.0));

    EXPECT_EQ(queue.top().conflict.firstConflictTime, 10.0);
}

TEST(ConflictPriorityQueueTest, EqualValuesUseDeterministicTrainId)
{
    ConflictPriorityQueue queue;
    queue.push(conflict(5, 6, 10.0), risk(50.0));
    queue.push(conflict(2, 3, 10.0), risk(50.0));

    EXPECT_EQ(queue.top().conflict.trainA, 2U);
}

} // namespace tcas::safety
