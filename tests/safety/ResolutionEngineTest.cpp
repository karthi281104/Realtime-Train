#include "safety/ResolutionEngine.hpp"

#include "train/FreightTrain.hpp"

#include <gtest/gtest.h>

namespace tcas::safety
{
namespace
{

train::FreightTrain makeTrain()
{
    train::FreightTrain train(3, 120000.0, 22.2, 0.5, 0.8);
    train.setVelocity(20.0);
    return train;
}

conflict::Conflict makeConflict()
{
    return {1, 3, conflict::ConflictType::Junction, 0, 2,
            5.0, 7.0, 0.0};
}

RiskAssessment makeRisk(RiskLevel level, double score)
{
    RiskAssessment risk;
    risk.level = level;
    risk.score = score;
    return risk;
}

SafetyCommand resolve(
    const RiskAssessment& risk,
    bool priorityGranted = false,
    bool brakingFeasible = true)
{
    const auto train = makeTrain();
    return ResolutionEngine{}.resolve({
        train, makeConflict(), risk, priorityGranted, brakingFeasible,
        1000.0, 100.0});
}

} // namespace

TEST(ResolutionEngineTest, CriticalRiskProducesEmergencyBrake)
{
    const auto command = resolve(makeRisk(RiskLevel::Critical, 90.0));

    EXPECT_EQ(command.type, SafetyCommandType::EmergencyBrake);
    EXPECT_TRUE(command.isEmergency());
}

TEST(ResolutionEngineTest, MediumRiskProducesSpeedReduction)
{
    const auto command = resolve(makeRisk(RiskLevel::Medium, 50.0), true);

    EXPECT_EQ(command.type, SafetyCommandType::ReduceSpeed);
    EXPECT_DOUBLE_EQ(command.targetSpeed, 10.0);
}

TEST(ResolutionEngineTest, HighRiskNonPriorityTrainHolds)
{
    const auto command = resolve(makeRisk(RiskLevel::High, 70.0));

    EXPECT_EQ(command.type, SafetyCommandType::HoldAtSignal);
    EXPECT_EQ(command.targetSpeed, 0.0);
}

TEST(ResolutionEngineTest, SafeLowRiskProducesNoAction)
{
    const auto command = resolve(makeRisk(RiskLevel::Low, 10.0), true);

    EXPECT_EQ(command.type, SafetyCommandType::NoAction);
    EXPECT_DOUBLE_EQ(command.targetSpeed, 20.0);
}

} // namespace tcas::safety
