#include "orchestrator/CommandQueue.hpp"

#include <gtest/gtest.h>

namespace tcas::orchestrator
{

TEST(CommandQueueTest, PushPopAndSize)
{
    CommandQueue queue;
    const safety::SafetyCommand command{
        safety::SafetyCommandType::ReduceSpeed,
        7,
        5.0,
        1.0,
        20.0};

    queue.push(command);
    ASSERT_EQ(queue.size(), 1U);

    safety::SafetyCommand result;
    ASSERT_TRUE(queue.tryPop(&result));
    EXPECT_EQ(result.trainId, command.trainId);
    EXPECT_EQ(result.type, command.type);
    EXPECT_DOUBLE_EQ(result.targetSpeed, command.targetSpeed);
    EXPECT_EQ(queue.size(), 0U);
}

TEST(CommandQueueTest, NullOutputAndEmptyQueueAreSafe)
{
    CommandQueue queue;

    EXPECT_FALSE(queue.tryPop(nullptr));
    EXPECT_FALSE(queue.tryPop(new safety::SafetyCommand{}));
    queue.clear();
    EXPECT_EQ(queue.size(), 0U);
}

TEST(CommandQueueTest, PreservesFifoOrder)
{
    CommandQueue queue;
    queue.push({safety::SafetyCommandType::HoldAtSignal, 1, 0.0, 0.0, 1.0});
    queue.push({safety::SafetyCommandType::EmergencyBrake, 2, 0.0, 0.0, 2.0});

    safety::SafetyCommand first;
    safety::SafetyCommand second;
    ASSERT_TRUE(queue.tryPop(&first));
    ASSERT_TRUE(queue.tryPop(&second));
    EXPECT_EQ(first.trainId, 1U);
    EXPECT_EQ(second.trainId, 2U);
}

} // namespace tcas::orchestrator
