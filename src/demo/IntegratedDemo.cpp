#include "demo/IntegratedDemo.hpp"
#include "demo/Module9Demo.hpp"

#include "common/Types.hpp"
#include "infrastructure/Node.hpp"
#include "infrastructure/Track.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "train/ExpressTrain.hpp"
#include "train/PassengerTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/TrainManager.hpp"
#include "simulation/SimClock.hpp"
#include "simulation/SimulationConfig.hpp"
#include "simulation/SimulationTimer.hpp"
#include "physics/KinematicsEngine.hpp"
#include "navigation/RouteNavigator.hpp"
#include "sensor/Odometer.hpp"
#include "sensor/StateEstimator.hpp"
#include "communication/CommunicationChannel.hpp"
#include "communication/Message.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace tcas::demo
{

void runIntegratedDemo()
{
    using namespace tcas::infrastructure;
    using namespace tcas::train;
    using namespace tcas::simulation;
    using namespace tcas::physics;
    using namespace tcas::navigation;
    using namespace tcas::sensor;
    using namespace tcas::communication;

    std::cout << "\n";
    std::cout << "========================================================================\n";
    std::cout << "        TCAS INTEGRATED SYSTEM DEMO (MODULES 1 TO 7 ACTIVE)\n";
    std::cout << "========================================================================\n";

    RailwayNetwork network;
    network.addNode(Node(1, "Central Station", NodeType::Station));
    network.addNode(Node(2, "Alpha Junction", NodeType::Junction));
    network.addNode(Node(3, "Beta Junction", NodeType::Junction));
    network.addNode(Node(4, "North Terminal", NodeType::Station));
    network.addNode(Node(5, "South Harbor", NodeType::Station));
    network.addTrack(Track(101, 1, 2, 2000.0, 35.0, 0.000));
    network.addTrack(Track(102, 2, 3, 1500.0, 30.0, 0.020));
    network.addTrack(Track(103, 3, 4, 2500.0, 40.0, -0.015));
    network.addTrack(Track(104, 2, 5, 3000.0, 25.0, 0.010));

    std::cout << "Nodes added  : " << network.nodeCount() << '\n';
    std::cout << "Tracks added : " << network.trackCount() << '\n';
    std::cout << "Graph status : Weakly connected = "
              << (network.isWeaklyConnected() ? "YES" : "NO")
              << ", Cycles = " << (network.hasCycle() ? "YES" : "NO") << '\n';

    TrainManager trainManager;
    trainManager.addTrain(std::make_unique<ExpressTrain>(1, 45000.0, 45.0, 0.9, 1.4));
    trainManager.addTrain(std::make_unique<PassengerTrain>(2, 60000.0, 33.3, 0.8, 1.2));
    trainManager.addTrain(std::make_unique<FreightTrain>(3, 120000.0, 22.2, 0.5, 0.8));

    const RouteResult route1 = RouteNavigator::findRoute(network, 1, 4);
    const RouteResult route2 = RouteNavigator::findRoute(network, 1, 5);
    std::cout << "Route #1 Central -> North: " << (route1.success ? "SUCCESS" : "FAILED") << '\n';
    std::cout << "Route #2 Central -> South: " << (route2.success ? "SUCCESS" : "FAILED") << '\n';

    SimulationConfig simConfig;
    SimClock simClock(static_cast<TimeSeconds>(simConfig.physicsPeriodMs) / 1000.0);
    SimulationTimer safetyTimer(simConfig.safetyPeriodMs, simConfig.physicsPeriodMs);
    SimulationTimer hmiTimer(simConfig.hmiPeriodMs, simConfig.physicsPeriodMs);
    (void)safetyTimer;

    auto* expressTrain = trainManager.getTrain(1);
    expressTrain->setState(TrainState::Running);
    expressTrain->setVelocity(15.0);

    constexpr SimTimeTick kSimulationSteps = 30;
    constexpr double targetAcceleration = 0.6;
    for (SimTimeTick step = 0; step < kSimulationSteps; ++step)
    {
        const auto tick = simClock.tickCount();
        const auto dt = simClock.dt();
        const double pos = expressTrain->position();
        TrackId currentTrackId = pos < 2000.0 ? 101u : (pos < 3500.0 ? 102u : 103u);
        const double currentGradient = network.getTrack(currentTrackId)->gradient();

        expressTrain->setVelocity(KinematicsEngine::updateVelocity(
            expressTrain->velocity(), targetAcceleration, dt, expressTrain->maximumSpeed()));
        expressTrain->setPosition(KinematicsEngine::updatePosition(
            expressTrain->position(), expressTrain->velocity(), targetAcceleration, dt));

        if (hmiTimer.shouldFire(tick))
        {
            std::cout << std::fixed << std::setprecision(2)
                      << "[SIM] t=" << simClock.elapsed()
                      << "s train=1 track=" << currentTrackId
                      << " pos=" << expressTrain->position()
                      << " vel=" << expressTrain->velocity()
                      << " gradient=" << currentGradient * 100.0 << "%\n";
        }
        simClock.tick();
    }

    SensorNoiseConfig noiseConfig;
    noiseConfig.driftRatePerSecond = 0.01;
    noiseConfig.measurementNoisePos = 2.0;
    noiseConfig.measurementNoiseVel = 0.5;
    noiseConfig.measurementNoiseBalise = 0.01;
    Odometer odometer(noiseConfig);
    StateEstimator estimator(noiseConfig, 0.0, 15.0);
    double truePos = 0.0;
    constexpr double trueVel = 15.0;
    constexpr double trueDt = 0.1;
    for (int step = 0; step < 15; ++step)
    {
        truePos += trueVel * trueDt;
        const auto measurement = odometer.measure(
            truePos, trueVel, 0.0, trueDt, static_cast<SimTimeTick>(step));
        estimator.predict(trueDt, static_cast<SimTimeTick>(step));
        estimator.updateOdometry(measurement);
        if (step == 10)
        {
            estimator.updateBalise({.baliseId = 1, .trackId = 101, .exactPosition = truePos});
        }
    }
    std::cout << "[SENSOR] Estimated position=" << estimator.estimatedState().position
              << "m, uncertainty=" << estimator.estimatedState().positionUncertainty << "m\n";

    ChannelConfig channelCfg;
    channelCfg.latencyTicks = 2;
    channelCfg.maxRangeMeters = 5000.0;
    CommunicationChannel channel(channelCfg);
    channel.registerEntity(1);
    channel.registerEntity(2);
    channel.registerEntity(3);
    channel.sendMessage(Message::makeHeartbeat(1001, 1, 0), 0.0, 0.0);
    channel.step(1);
    channel.step(2);
    std::cout << "[COMM] Sent=" << channel.totalSent()
              << " Delivered=" << channel.totalDelivered()
              << " Dropped=" << channel.totalDropped() << '\n';

    std::cout << "\n[MODULE 9] PREDICTIVE CONFLICT + JUNCTION RESERVATION\n";
    std::cout << "------------------------------------------------------------------------\n";
    runModule9Demo();

    std::cout << "========================================================================\n\n";
}

} // namespace tcas::demo
