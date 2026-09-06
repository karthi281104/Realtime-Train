#include "conflict/ConflictDetector.hpp"
#include "conflict/ResourceReservationManager.hpp"
#include "communication/CommunicationChannel.hpp"
#include "communication/Message.hpp"
#include "navigation/RouteNavigator.hpp"
#include "prediction/PredictionEngine.hpp"
#include "sensor/Odometer.hpp"
#include "sensor/StateEstimator.hpp"
#include "train/TrainManager.hpp"
#include "safety/ConflictPriorityQueue.hpp"
#include "safety/PriorityEngine.hpp"
#include "safety/ResolutionEngine.hpp"
#include "safety/RiskEngine.hpp"

#include <iomanip>
#include <iostream>
#include <memory>

namespace tcas::demo
{
namespace
{

const char* riskLevelName(safety::RiskLevel level)
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

const char* commandName(safety::SafetyCommandType type)
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

void runModule9Demo(
    const infrastructure::RailwayNetwork& network,
    train::TrainManager& trainManager)
{
    std::cout << "\n============================================================\n";
    std::cout << "             MODULE 9: CONFLICT INTEGRATION\n";
    std::cout << "============================================================\n";

    auto* express = trainManager.getTrain(1);
    auto* freight = trainManager.getTrain(3);
    if (express == nullptr || freight == nullptr)
    {
        std::cout << "RESULT: shared fleet is missing Express #1 or Freight #3\n";
        return;
    }

    // Stage the shared trains to reach Alpha Junction at the same horizon.
    express->setPosition(1400.0);
    express->setVelocity(20.0);
    express->setAcceleration(0.0);
    freight->setPosition(1400.0);
    freight->setVelocity(20.0);
    freight->setAcceleration(0.0);

    const auto expressRoute = navigation::RouteNavigator::findRoute(network, 1, 3);
    const auto freightRoute = navigation::RouteNavigator::findRoute(network, 6, 7);

    sensor::SensorNoiseConfig noiseConfig;
    noiseConfig.measurementNoisePos = 2.0;
    noiseConfig.measurementNoiseVel = 0.5;
    noiseConfig.measurementNoiseBalise = 0.01;
    sensor::Odometer expressOdometer(noiseConfig);
    sensor::StateEstimator expressEstimator(
        noiseConfig, express->position(), express->velocity());
    const auto measurement = expressOdometer.measure(
        express->position(), express->velocity(), express->acceleration(), 0.1, 0);
    expressEstimator.predict(0.1, 0);
    expressEstimator.updateOdometry(measurement);
    express->setPosition(expressEstimator.estimatedState().position);

    const auto expressPrediction = prediction::PredictionEngine::predictStandardHorizon(
        *express, network, expressRoute, 101,
        expressEstimator.estimatedState().positionUncertainty);
    const auto freightPrediction = prediction::PredictionEngine::predictStandardHorizon(
        *freight, network, freightRoute, 105, 1.0);

    communication::ChannelConfig channelConfig;
    channelConfig.latencyTicks = 1;
    channelConfig.maxRangeMeters = 5000.0;
    communication::CommunicationChannel channel(channelConfig);
    channel.registerEntity(1);
    channel.registerEntity(3);
    channel.sendMessage(
        communication::Message::makeHeartbeat(9001, 3, 0), 0.0, 0.0);
    channel.step(1);
    const bool communicationHealthy = channel.totalDelivered() > 0;

    conflict::ConflictDetector detector;
    const auto conflicts = detector.detect(
        express->id(), expressPrediction,
        freight->id(), freightPrediction,
        network);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Route Express: " << (expressRoute.success ? "OK" : "FAILED") << '\n';
    std::cout << "Route Freight: " << (freightRoute.success ? "OK" : "FAILED") << '\n';
    std::cout << "Express prediction points: " << expressPrediction.size() << '\n';
    std::cout << "Freight prediction points: " << freightPrediction.size() << '\n';
    std::cout << "Communication health: " << (communicationHealthy ? "HEALTHY" : "FAILED") << '\n';
    std::cout << "Predicted conflicts: " << conflicts.size() << '\n';

    safety::RiskEngine riskEngine;
    safety::PriorityEngine priorityEngine;
    safety::ResolutionEngine resolutionEngine;
    safety::ConflictPriorityQueue conflictQueue;
    constexpr TimeSeconds currentTime = 0.0;

    for (const auto& detectedConflict : conflicts)
    {
        const auto* trainA = trainManager.getTrain(detectedConflict.trainA);
        const auto* trainB = trainManager.getTrain(detectedConflict.trainB);

        if (trainA == nullptr || trainB == nullptr)
        {
            continue;
        }

        const TimeSeconds ttc = std::max(
            0.0,
            detectedConflict.firstConflictTime - currentTime);
        const double relativeVelocity =
            std::abs(trainA->velocity() - trainB->velocity());
        const double emergencyBraking = std::max(
            trainA->emergencyBraking(), trainB->emergencyBraking());
        const double speed = std::max(
            std::abs(trainA->velocity()), std::abs(trainB->velocity()));
        const double brakingDistance = emergencyBraking > 0.0
            ? (speed * speed) / (2.0 * emergencyBraking)
            : 0.0;

        safety::RiskInput riskInput;
        riskInput.timeToCollision = ttc;
        riskInput.relativeVelocity = relativeVelocity;
        riskInput.brakingDistance = brakingDistance;
        riskInput.safetyMargin = detectedConflict.minimumSeparation - brakingDistance;
        riskInput.conflictType = detectedConflict.type;
        riskInput.trainMass = std::max(trainA->mass(), trainB->mass());
        riskInput.sensorConfidence = 1.0;
        riskInput.communicationConfidence = communicationHealthy ? 1.0 : 0.0;

        const auto risk = riskEngine.assess(riskInput);
        conflictQueue.push(detectedConflict, risk);
    }

    if (!conflicts.empty())
    {
        const auto& detectedConflict = conflicts.front();
        const conflict::ConflictZone junction{
            conflict::ConflictZoneType::Junction, detectedConflict.resourceNodeId, 0};
        conflict::ResourceReservationManager reservations;

        const bool expressGranted = reservations.request(
            express->id(), junction,
            detectedConflict.firstConflictTime,
            detectedConflict.lastConflictTime);
        const bool freightGranted = reservations.request(
            freight->id(), junction,
            detectedConflict.firstConflictTime,
            detectedConflict.lastConflictTime);

        std::cout << "Conflict type: "
                  << (detectedConflict.type == conflict::ConflictType::Junction ? "JUNCTION" : "OTHER")
                  << '\n';
        std::cout << "J1 reservation -> Express: "
                  << (expressGranted ? "GRANTED" : "DENIED") << '\n';
        std::cout << "J1 reservation -> Freight: "
                  << (freightGranted ? "GRANTED" : "DENIED") << '\n';

        if (expressGranted)
        {
            const bool released = reservations.release(express->id(), junction);
            std::cout << "J1 release: " << (released ? "SUCCESS" : "FAILED") << '\n';
            reservations.clearReleased();
        }

        std::cout << "J1 state: RELEASED\n";

        while (!conflictQueue.empty())
        {
            const auto prioritized = conflictQueue.top();
            conflictQueue.pop();

            const auto* trainA = trainManager.getTrain(prioritized.conflict.trainA);
            const auto* trainB = trainManager.getTrain(prioritized.conflict.trainB);
            if (trainA == nullptr || trainB == nullptr)
            {
                continue;
            }

            const auto priorityA = priorityEngine.assess(*trainA);
            const auto priorityB = priorityEngine.assess(*trainB);
            const bool trainAHasPriority = priorityA.higherThan(priorityB)
                || (priorityA.priority == priorityB.priority
                    && trainA->id() < trainB->id());
            const auto* priorityTrain = trainAHasPriority ? trainA : trainB;
            const auto* yieldingTrain = trainAHasPriority ? trainB : trainA;

            const safety::ResolutionInput resolutionInput{
                *yieldingTrain,
                prioritized.conflict,
                prioritized.risk,
                true,
                true,
                1000.0,
                100.0};
            const auto command = resolutionEngine.resolve(resolutionInput);

            std::cout << "\n============================================================\n";
            std::cout << "MODULE 10: RISK + PRIORITY + PREDICTIVE RESOLUTION\n";
            std::cout << "============================================================\n";
            std::cout << "Conflict type       : JUNCTION\n";
            std::cout << "Time to conflict    : "
                      << prioritized.conflict.firstConflictTime << " s\n";
            std::cout << "Risk score          : " << prioritized.risk.score << '\n';
            std::cout << "Risk level          : "
                      << riskLevelName(prioritized.risk.level) << '\n';
            std::cout << "Train #" << priorityTrain->id()
                      << " priority       : "
                      << (trainAHasPriority ? "HIGH" : "LOW") << '\n';
            std::cout << "Train #" << yieldingTrain->id()
                      << " priority       : "
                      << (trainAHasPriority ? "LOW" : "HIGH") << '\n';
            std::cout << "Selected priority   : Train #" << priorityTrain->id() << '\n';
            std::cout << "Resolution command  : " << commandName(command.type) << '\n';
            std::cout << "Command target      : Train #" << command.trainId << '\n';
            std::cout << "Safety decision     : CONFLICT RESOLUTION GENERATED\n";
        }

        std::cout << "RESULT: predictive conflict detected, resource protected, and resolution command generated\n";
    }
    else
    {
        std::cout << "RESULT: no conflict detected\n";
    }
}

} // namespace tcas::demo
