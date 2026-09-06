#include "demo/Module10Demo.hpp"

#include "communication/CommunicationChannel.hpp"
#include "communication/Message.hpp"
#include "conflict/ConflictDetector.hpp"
#include "navigation/RouteNavigator.hpp"
#include "prediction/PredictionEngine.hpp"
#include "safety/ConflictPriorityQueue.hpp"
#include "safety/PriorityEngine.hpp"
#include "safety/ResolutionEngine.hpp"
#include "safety/RiskEngine.hpp"
#include "sensor/Odometer.hpp"
#include "sensor/StateEstimator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace tcas::demo
{
namespace
{

const char* riskLevelName(
    safety::RiskLevel level)
{
    switch (level)
    {
    case safety::RiskLevel::Low:
        return "LOW";

    case safety::RiskLevel::Medium:
        return "MEDIUM";

    case safety::RiskLevel::High:
        return "HIGH";

    case safety::RiskLevel::Critical:
        return "CRITICAL";
    }

    return "UNKNOWN";
}

const char* commandName(
    safety::SafetyCommandType type)
{
    switch (type)
    {
    case safety::SafetyCommandType::NoAction:
        return "NO_ACTION";

    case safety::SafetyCommandType::ReduceSpeed:
        return "REDUCE_SPEED";

    case safety::SafetyCommandType::HoldAtSignal:
        return "HOLD_AT_SIGNAL";

    case safety::SafetyCommandType::EmergencyBrake:
        return "EMERGENCY_BRAKE";
    }

    return "UNKNOWN";
}

} // namespace

void runModule10Demo(
    const infrastructure::RailwayNetwork& network,
    train::TrainManager& trainManager)
{
    std::cout << "\n";
    std::cout << "====================================================================\n";
    std::cout << "       MODULE 10: RISK + PRIORITY + PREDICTIVE RESOLUTION\n";
    std::cout << "====================================================================\n";

    auto* express =
        trainManager.getTrain(1);

    auto* freight =
        trainManager.getTrain(3);

    if (express == nullptr
        || freight == nullptr)
    {
        std::cout
            << "RESULT: Express #1 or Freight #3 is missing\n";

        return;
    }

    // ------------------------------------------------------------
    // Stage the trains for the junction conflict scenario.
    // ------------------------------------------------------------

    express->setPosition(1400.0);
    express->setVelocity(20.0);
    express->setAcceleration(0.0);

    freight->setPosition(1400.0);
    freight->setVelocity(20.0);
    freight->setAcceleration(0.0);

    // ------------------------------------------------------------
    // MODULE 5: Route generation
    // ------------------------------------------------------------

    const auto expressRoute =
        navigation::RouteNavigator::findRoute(
            network,
            1,
            3);

    const auto freightRoute =
        navigation::RouteNavigator::findRoute(
            network,
            6,
            7);

    if (!expressRoute.success
        || !freightRoute.success)
    {
        std::cout
            << "RESULT: route generation failed\n";

        return;
    }

    // ------------------------------------------------------------
    // MODULE 6: Sensor confidence
    // ------------------------------------------------------------

    sensor::SensorNoiseConfig noiseConfig;

    noiseConfig.measurementNoisePos = 2.0;
    noiseConfig.measurementNoiseVel = 0.5;
    noiseConfig.measurementNoiseBalise = 0.01;

    sensor::Odometer expressOdometer(
        noiseConfig);

    sensor::StateEstimator expressEstimator(
        noiseConfig,
        express->position(),
        express->velocity());

    const auto measurement =
        expressOdometer.measure(
            express->position(),
            express->velocity(),
            express->acceleration(),
            0.1,
            0);

    expressEstimator.predict(
        0.1,
        0);

    expressEstimator.updateOdometry(
        measurement);

    const auto estimatedState =
        expressEstimator.estimatedState();

    express->setPosition(
        estimatedState.position);

    // ------------------------------------------------------------
    // MODULE 7: Communication health
    // ------------------------------------------------------------

    communication::ChannelConfig channelConfig;

    channelConfig.latencyTicks = 1;
    channelConfig.maxRangeMeters = 5000.0;

    communication::CommunicationChannel channel(
        channelConfig);

    channel.registerEntity(1);
    channel.registerEntity(3);

    channel.sendMessage(
        communication::Message::makeHeartbeat(
            9001,
            3,
            0),
        0.0,
        0.0);

    channel.step(1);

    const bool communicationHealthy =
        channel.totalDelivered() > 0;

    // ------------------------------------------------------------
    // MODULE 8: Predictive position engine
    // ------------------------------------------------------------

    const auto expressPrediction =
        prediction::PredictionEngine::predictStandardHorizon(
            *express,
            network,
            expressRoute,
            101,
            estimatedState.positionUncertainty);

    const auto freightPrediction =
        prediction::PredictionEngine::predictStandardHorizon(
            *freight,
            network,
            freightRoute,
            105,
            1.0);

    // ------------------------------------------------------------
    // MODULE 9: Conflict detection
    // ------------------------------------------------------------

    conflict::ConflictDetector detector;

    const auto conflicts =
        detector.detect(
            express->id(),
            expressPrediction,
            freight->id(),
            freightPrediction,
            network);

    std::cout << std::fixed
              << std::setprecision(2);

    std::cout
        << "\n[INPUT]\n";

    std::cout
        << "Express route status : "
        << (expressRoute.success
                ? "SUCCESS"
                : "FAILED")
        << '\n';

    std::cout
        << "Freight route status : "
        << (freightRoute.success
                ? "SUCCESS"
                : "FAILED")
        << '\n';

    std::cout
        << "Express prediction points : "
        << expressPrediction.size()
        << '\n';

    std::cout
        << "Freight prediction points : "
        << freightPrediction.size()
        << '\n';

    std::cout
        << "Communication health : "
        << (communicationHealthy
                ? "HEALTHY"
                : "FAILED")
        << '\n';

    std::cout
        << "Predicted conflicts : "
        << conflicts.size()
        << '\n';

    if (conflicts.empty())
    {
        std::cout
            << "\nRESULT: no predictive conflict detected\n";

        return;
    }

    // ------------------------------------------------------------
    // MODULE 10: Risk assessment
    // ------------------------------------------------------------

    safety::RiskEngine riskEngine;

    safety::PriorityEngine priorityEngine;

    safety::ResolutionEngine resolutionEngine;

    safety::ConflictPriorityQueue conflictQueue;

    constexpr TimeSeconds currentTime = 0.0;

    for (const auto& detectedConflict : conflicts)
    {
        const auto* trainA =
            trainManager.getTrain(
                detectedConflict.trainA);

        const auto* trainB =
            trainManager.getTrain(
                detectedConflict.trainB);

        if (trainA == nullptr
            || trainB == nullptr)
        {
            continue;
        }

        const TimeSeconds ttc =
            std::max(
                0.0,
                detectedConflict.firstConflictTime
                    - currentTime);

        const double relativeVelocity =
            std::abs(
                trainA->velocity()
                - trainB->velocity());

        // Evaluate risk for the train that may need
        // to yield. This will later be determined
        // from priority.
        const auto priorityA =
            priorityEngine.assess(*trainA);

        const auto priorityB =
            priorityEngine.assess(*trainB);

        const bool trainAHasPriority =
            priorityA.higherThan(priorityB)
            || (
                priorityA.priority
                    == priorityB.priority
                && trainA->id() < trainB->id()
            );

        const auto* yieldingTrain =
            trainAHasPriority
                ? trainB
                : trainA;

        const double emergencyDeceleration =
            yieldingTrain->emergencyBraking();

        const double speed =
            std::max(
                0.0,
                yieldingTrain->velocity());

        const double brakingDistance =
            emergencyDeceleration > 0.0
                ? (speed * speed)
                    / (2.0 * emergencyDeceleration)
                : 0.0;

        // The conflict detector's minimum separation
        // represents the required protected separation.
        const double safetyMargin =
            detectedConflict.minimumSeparation
            - brakingDistance;

        safety::RiskInput riskInput;

        riskInput.timeToCollision = ttc;

        riskInput.relativeVelocity =
            relativeVelocity;

        riskInput.brakingDistance =
            brakingDistance;

        riskInput.safetyMargin =
            safetyMargin;

        riskInput.conflictType =
            detectedConflict.type;

        riskInput.trainMass =
            yieldingTrain->mass();

        riskInput.sensorConfidence =
            estimatedState.isDegraded
                ? 0.0
                : 1.0;

        riskInput.communicationConfidence =
            communicationHealthy
                ? 1.0
                : 0.0;

        const auto risk =
            riskEngine.assess(riskInput);

        conflictQueue.push(
            detectedConflict,
            risk);
    }

    if (conflictQueue.empty())
    {
        std::cout
            << "RESULT: conflicts could not be assessed\n";

        return;
    }

    // ------------------------------------------------------------
    // Process highest-priority safety conflict.
    // ------------------------------------------------------------

    while (!conflictQueue.empty())
    {
        const auto prioritized =
            conflictQueue.top();

        conflictQueue.pop();

        const auto* trainA =
            trainManager.getTrain(
                prioritized.conflict.trainA);

        const auto* trainB =
            trainManager.getTrain(
                prioritized.conflict.trainB);

        if (trainA == nullptr
            || trainB == nullptr)
        {
            continue;
        }

        const auto priorityA =
            priorityEngine.assess(*trainA);

        const auto priorityB =
            priorityEngine.assess(*trainB);

        const bool trainAHasPriority =
            priorityA.higherThan(priorityB)
            || (
                priorityA.priority
                    == priorityB.priority
                && trainA->id() < trainB->id()
            );

        const auto* priorityTrain =
            trainAHasPriority
                ? trainA
                : trainB;

        const auto* yieldingTrain =
            trainAHasPriority
                ? trainB
                : trainA;

        // --------------------------------------------------------
        // Braking feasibility
        // --------------------------------------------------------

        const double speed =
            std::max(
                0.0,
                yieldingTrain->velocity());

        const double emergencyDeceleration =
            yieldingTrain->emergencyBraking();

        const double requiredBrakingDistance =
            emergencyDeceleration > 0.0
                ? (speed * speed)
                    / (2.0 * emergencyDeceleration)
                : 0.0;

        // Demonstration of the available distance to the
        // protected conflict zone.
        const double availableDistance =
            1000.0;

        const bool brakingFeasible =
            std::isfinite(requiredBrakingDistance)
            && requiredBrakingDistance >= 0.0
            && availableDistance
                >= requiredBrakingDistance
                    + std::max(
                        0.0,
                        prioritized.risk.safetyMargin);

        // --------------------------------------------------------
        // MODULE 10: Resolution
        // --------------------------------------------------------

        const safety::ResolutionInput resolutionInput{
            *yieldingTrain,
            prioritized.conflict,
            prioritized.risk,

            // The priority train gets the protected movement
            // authority. The yielding train does not.
            false,

            brakingFeasible,

            availableDistance,

            requiredBrakingDistance
        };

        const auto command =
            resolutionEngine.resolve(
                resolutionInput);

        // --------------------------------------------------------
        // HMI output
        // --------------------------------------------------------

        std::cout << "\n";
        std::cout
            << "--------------------------------------------------------------------\n";

        std::cout
            << "MODULE 10 SAFETY DECISION\n";

        std::cout
            << "--------------------------------------------------------------------\n";

        std::cout
            << "Conflict type          : "
            << static_cast<int>(
                prioritized.conflict.type)
            << '\n';

        std::cout
            << "Conflict time          : "
            << prioritized.conflict.firstConflictTime
            << " s\n";

        std::cout
            << "Risk score             : "
            << prioritized.risk.score
            << '\n';

        std::cout
            << "Risk level             : "
            << riskLevelName(
                prioritized.risk.level)
            << '\n';

        std::cout
            << "Train with priority    : Train #"
            << priorityTrain->id()
            << '\n';

        std::cout
            << "Yielding train         : Train #"
            << yieldingTrain->id()
            << '\n';

        std::cout
            << "Required brake dist.   : "
            << requiredBrakingDistance
            << " m\n";

        std::cout
            << "Available distance     : "
            << availableDistance
            << " m\n";

        std::cout
            << "Braking feasible       : "
            << (brakingFeasible
                    ? "YES"
                    : "NO")
            << '\n';

        std::cout
            << "Resolution command     : "
            << commandName(command.type)
            << '\n';

        std::cout
            << "Command target         : Train #"
            << command.trainId
            << '\n';

        std::cout
            << "Target speed           : "
            << command.targetSpeed
            << " m/s\n";

        if (command.isEmergency())
        {
            std::cout
                << "SAFETY STATUS          : EMERGENCY BRAKING\n";
        }
        else
        {
            std::cout
                << "SAFETY STATUS          : CONFLICT RESOLVED\n";
        }

        std::cout
            << "--------------------------------------------------------------------\n";

        std::cout
            << "RESULT: predictive conflict detected and "
               "safety resolution generated\n";
    }
}

} // namespace tcas::demo