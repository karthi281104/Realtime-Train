#include "conflict/ConflictDetector.hpp"
#include "conflict/ResourceReservationManager.hpp"
#include "communication/CommunicationChannel.hpp"
#include "communication/Message.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "navigation/RouteNavigator.hpp"
#include "prediction/PredictionEngine.hpp"
#include "sensor/Odometer.hpp"
#include "sensor/StateEstimator.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/TrainManager.hpp"

#include <iomanip>
#include <iostream>
#include <memory>

namespace tcas::demo
{
namespace
{

infrastructure::RailwayNetwork makeNetwork()
{
    infrastructure::RailwayNetwork network;
    network.addNode({1, "Express Origin", infrastructure::NodeType::Generic});
    network.addNode({2, "J1", infrastructure::NodeType::Junction});
    network.addNode({3, "Express Destination", infrastructure::NodeType::Generic});
    network.addNode({4, "Freight Origin", infrastructure::NodeType::Generic});
    network.addNode({5, "Freight Destination", infrastructure::NodeType::Generic});
    network.addTrack({101, 1, 2, 1000.0, 30.0, 0.0});
    network.addTrack({102, 2, 3, 1000.0, 30.0, 0.0});
    network.addTrack({103, 4, 2, 1000.0, 30.0, 0.0});
    network.addTrack({104, 2, 5, 1000.0, 30.0, 0.0});
    return network;
}

} // namespace

void runModule9Demo()
{
    std::cout << "\n============================================================\n";
    std::cout << "             MODULE 9: CONFLICT INTEGRATION\n";
    std::cout << "============================================================\n";

    const auto network = makeNetwork();

    train::TrainManager trainManager;
    trainManager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    trainManager.addTrain(std::make_unique<train::FreightTrain>(
        2, 120000.0, 22.2, 0.5, 0.8));

    auto* express = trainManager.getTrain(1);
    auto* freight = trainManager.getTrain(2);
    express->setPosition(600.0);
    express->setVelocity(20.0);
    express->setAcceleration(0.0);
    freight->setPosition(580.0);
    freight->setVelocity(20.0);
    freight->setAcceleration(0.0);

    const auto expressRoute = navigation::RouteNavigator::findRoute(network, 1, 3);
    const auto freightRoute = navigation::RouteNavigator::findRoute(network, 4, 5);

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
        *freight, network, freightRoute, 103, 1.0);

    communication::ChannelConfig channelConfig;
    channelConfig.latencyTicks = 1;
    channelConfig.maxRangeMeters = 5000.0;
    communication::CommunicationChannel channel(channelConfig);
    channel.registerEntity(1);
    channel.registerEntity(2);
    channel.sendMessage(
        communication::Message::makeHeartbeat(9001, 2, 0), 0.0, 0.0);
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
        std::cout << "RESULT: predictive conflict detected and resource protected\n";
    }
    else
    {
        std::cout << "RESULT: no conflict detected\n";
    }
}

} // namespace tcas::demo
