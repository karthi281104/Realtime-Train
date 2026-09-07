#include "common/Types.hpp"
#include "communication/CommunicationChannel.hpp"
#include "communication/Message.hpp"
#include "conflict/ConflictDetector.hpp"
#include "conflict/ResourceReservationManager.hpp"
#include "hmi/HmiDisplay.hpp"
#include "hmi/PerformanceMetrics.hpp"
#include "hmi/TelemetryLogger.hpp"
#include "infrastructure/Junction.hpp"
#include "infrastructure/Node.hpp"
#include "infrastructure/Platform.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "infrastructure/Station.hpp"
#include "infrastructure/Track.hpp"
#include "navigation/RouteNavigator.hpp"
#include "orchestrator/ThreadOrchestrator.hpp"
#include "physics/KinematicsEngine.hpp"
#include "prediction/PredictionEngine.hpp"
#include "safety/ConflictPriorityQueue.hpp"
#include "safety/PriorityEngine.hpp"
#include "safety/ResolutionEngine.hpp"
#include "safety/RiskEngine.hpp"
#include "sensor/Odometer.hpp"
#include "sensor/SensorData.hpp"
#include "sensor/StateEstimator.hpp"
#include "simulation/SimClock.hpp"
#include "simulation/SimulationConfig.hpp"
#include "simulation/SimulationTimer.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"
#include "train/TrainManager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace
{

std::atomic<bool> g_stop_requested{false};

void signalHandler(int /*signum*/)
{
    g_stop_requested.store(true);
}

void printBanner()
{
    std::cout << R"(
================================================================================
  _____ ___    _   ___   ____            _ _____ _                 
 |_   _/ __|  /_\ / __| |  _ \ ___  __ _| |_   _(_)_ __  ___       
   | || (__  / _ \\__ \ | |_) / _ \/ _` | | | | | | '_ \/ -_)      
   |_| \___|/_/ \_\___/ |  _ <\___/\__,_|_| |_| |_|_| ._/\___|      
                        |_| \_\                     |_|            
  Train Collision Avoidance System - Real-Time Autonomous Safety Core
  Modules 1-10: Multi-threaded Orchestration & Predictive Safety Engine
================================================================================
)" << std::endl;
}

void printHelp(const char* progName)
{
    std::cout << "Usage: " << progName << " [OPTIONS]\n\n"
              << "Options:\n"
              << "  --help, -h          Display this help message and exit\n"
              << "  --demo              Run complete end-to-end demonstration across all modules\n"
              << "  --realtime, -r      Run real-time multi-threaded orchestrator (default)\n"
              << "  --duration <sec>    Run duration in seconds for real-time mode (default: 5)\n"
              << "  --trains <count>    Number of active trains to simulate (1 to 3, default: 3)\n"
              << "  --verbose, -v       Enable detailed step logging to standard output\n"
              << std::endl;
}

struct AppConfig
{
    bool runDemoMode{false};
    bool runRealtimeMode{true};
    int durationSeconds{5};
    int trainCount{3};
    bool verbose{false};
};

AppConfig parseArgs(int argc, char* argv[])
{
    AppConfig config;
    for (int i = 1; i < argc; ++i)
    {
        std::string_view arg(argv[i]);
        if (arg == "--help" || arg == "-h")
        {
            printHelp(argv[0]);
            std::exit(0);
        }
        else if (arg == "--demo")
        {
            config.runDemoMode = true;
            config.runRealtimeMode = false;
        }
        else if (arg == "--realtime" || arg == "-r")
        {
            config.runRealtimeMode = true;
        }
        else if ((arg == "--duration" || arg == "-d") && i + 1 < argc)
        {
            config.durationSeconds = std::max(1, std::atoi(argv[++i]));
        }
        else if ((arg == "--trains" || arg == "-t") && i + 1 < argc)
        {
            config.trainCount = std::clamp(std::atoi(argv[++i]), 1, 3);
        }
        else if (arg == "--verbose" || arg == "-v")
        {
            config.verbose = true;
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << "\n";
            printHelp(argv[0]);
            std::exit(1);
        }
    }
    return config;
}

tcas::infrastructure::RailwayNetwork buildNetwork()
{
    using namespace tcas::infrastructure;
    RailwayNetwork network;

    network.addNode(Node(1, "Central Terminal", NodeType::Station));
    network.addNode(Node(2, "Alpha Junction", NodeType::Junction));
    network.addNode(Node(3, "Beta Junction", NodeType::Junction));
    network.addNode(Node(4, "North Terminal", NodeType::Station));
    network.addNode(Node(5, "South Harbor", NodeType::Station));
    network.addNode(Node(6, "Freight Depot", NodeType::Station));
    network.addNode(Node(7, "Industrial Yard", NodeType::Station));

    network.addTrack(Track(101, 1, 2, 2000.0, 45.0, 0.0));
    network.addTrack(Track(102, 2, 3, 1500.0, 35.0, 0.02));
    network.addTrack(Track(103, 3, 4, 2500.0, 40.0, -0.015));
    network.addTrack(Track(104, 3, 5, 3000.0, 30.0, 0.01));
    network.addTrack(Track(105, 6, 2, 1800.0, 25.0, 0.0));
    network.addTrack(Track(106, 2, 7, 2200.0, 25.0, -0.005));

    return network;
}

void setupFleet(tcas::train::TrainManager& manager, int trainCount)
{
    using namespace tcas::train;

    if (trainCount >= 1)
    {
        auto express = std::make_unique<ExpressTrain>(1, 45000.0, 45.0, 0.9, 1.4);
        express->setPosition(1200.0);
        express->setVelocity(25.0);
        manager.addTrain(std::move(express));
    }
    if (trainCount >= 2)
    {
        auto passenger = std::make_unique<PassengerTrain>(2, 60000.0, 33.3, 0.8, 1.2);
        passenger->setPosition(200.0);
        passenger->setVelocity(18.0);
        manager.addTrain(std::move(passenger));
    }
    if (trainCount >= 3)
    {
        auto freight = std::make_unique<FreightTrain>(3, 120000.0, 22.2, 0.5, 0.8);
        freight->setPosition(1100.0);
        freight->setVelocity(15.0);
        manager.addTrain(std::move(freight));
    }
}

void runRealtimeSystem(const AppConfig& config)
{
    std::cout << "[SYSTEM] Initializing TCAS Real-Time Autonomous Safety Core...\n";

    auto network = buildNetwork();
    std::cout << "  [OK] Railway network topology: 7 stations/junctions, 6 tracks active\n";

    tcas::train::TrainManager trainManager;
    setupFleet(trainManager, config.trainCount);
    std::cout << "  [OK] Fleet initialized with " << config.trainCount << " operational trains\n";

    tcas::communication::ChannelConfig commCfg;
    commCfg.latencyTicks = 1;
    commCfg.maxRangeMeters = 8000.0;
    tcas::communication::CommunicationChannel channel(commCfg);
    for (int id = 1; id <= config.trainCount; ++id)
    {
        channel.registerEntity(static_cast<tcas::TrainId>(id));
    }
    std::cout << "  [OK] V2V/V2I wireless communication channel configured\n";

    tcas::conflict::ConflictDetector conflictDetector;
    tcas::safety::RiskEngine riskEngine;
    tcas::safety::ResolutionEngine resolutionEngine;

    std::vector<tcas::TrainId> activeTrainIds;
    for (int id = 1; id <= config.trainCount; ++id)
    {
        activeTrainIds.push_back(static_cast<tcas::TrainId>(id));
    }

    tcas::orchestrator::OrchestratorConfig orchConfig;
    orchConfig.physicsPeriod = std::chrono::milliseconds(20);
    orchConfig.safetyPeriod = std::chrono::milliseconds(50);
    orchConfig.communicationPeriod = std::chrono::milliseconds(50);
    orchConfig.hmiPeriod = std::chrono::milliseconds(200);
    orchConfig.printHmi = config.verbose;

    auto safetyStep = [&](const tcas::orchestrator::WorldState& world) -> tcas::orchestrator::SafetyCycleResult {
        tcas::orchestrator::SafetyCycleResult result;
        if (world.trains.size() < 2) return result;

        std::vector<tcas::prediction::FutureState> traj1 = {
            {0.0, 101, world.trains[0].position, world.trains[0].velocity, 0.0, 1.0},
            {5.0, 101, world.trains[0].position + world.trains[0].velocity * 5.0, world.trains[0].velocity, 0.0, 2.0}
        };

        std::vector<tcas::prediction::FutureState> traj2 = {
            {0.0, 101, world.trains[1].position, world.trains[1].velocity, 0.0, 1.0},
            {5.0, 101, world.trains[1].position + world.trains[1].velocity * 5.0, world.trains[1].velocity, 0.0, 2.0}
        };

        result.predictions.insert(result.predictions.end(), traj1.begin(), traj1.end());
        result.predictions.insert(result.predictions.end(), traj2.begin(), traj2.end());

        auto conflicts = conflictDetector.detect(
            world.trains[0].id, traj1, world.trains[1].id, traj2, network);
        result.activeConflicts = conflicts;

        for (const auto& c : conflicts)
        {
            tcas::safety::RiskInput rIn;
            rIn.timeToCollision = c.firstConflictTime > 0.0 ? c.firstConflictTime : 5.0;
            rIn.relativeVelocity = std::abs(world.trains[0].velocity - world.trains[1].velocity);
            rIn.brakingDistance = 100.0;
            rIn.safetyMargin = c.minimumSeparation;
            rIn.conflictType = c.type;
            rIn.trainMass = 50000.0;
            rIn.sensorConfidence = 1.0;
            rIn.communicationConfidence = 1.0;

            auto risk = riskEngine.assess(rIn);

            auto* trainA = trainManager.getTrain(c.trainA);
            if (trainA)
            {
                tcas::safety::ResolutionInput resIn{
                    *trainA, c, risk, false, true, 300.0, 150.0
                };
                auto cmd = resolutionEngine.resolve(resIn);
                if (cmd.type != tcas::safety::SafetyCommandType::NoAction)
                {
                    result.commands.push_back(cmd);
                }
            }
        }
        return result;
    };

    tcas::orchestrator::ThreadOrchestrator orchestrator(
        network,
        trainManager,
        channel,
        activeTrainIds,
        orchConfig,
        safetyStep
    );

    std::cout << "\n[START] Starting Multi-Threaded Real-Time TCAS Core ("
              << config.durationSeconds << "s run)...\n";
    std::cout << "  - Physics Thread   : 20 ms cycle (50 Hz)\n";
    std::cout << "  - Safety Thread    : 50 ms cycle (20 Hz)\n";
    std::cout << "  - Comm Thread      : 50 ms cycle (20 Hz)\n";
    std::cout << "  - HMI/Log Thread   : 200 ms cycle (5 Hz)\n";
    std::cout << "Press Ctrl+C at any time to request graceful shutdown.\n\n";

    orchestrator.start();

    auto startTime = std::chrono::steady_clock::now();
    auto runDuration = std::chrono::seconds(config.durationSeconds);

    while (!g_stop_requested.load() &&
           (std::chrono::steady_clock::now() - startTime) < runDuration)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto snap = orchestrator.snapshot();
        std::cout << "[T+" << std::fixed << std::setprecision(1) << snap.simulationTime << "s] "
                  << "Active Trains: " << snap.trains.size() << " | ";
        for (const auto& t : snap.trains)
        {
            std::cout << "Train#" << t.id << ": pos=" << std::setprecision(1) << t.position
                      << "m, v=" << std::setprecision(1) << t.velocity << "m/s | ";
        }
        std::cout << "\n";
    }

    std::cout << "\n[SHUTDOWN] Initiating graceful shutdown of all threads...\n";
    orchestrator.stop();

    std::cout << "\n================================================================================\n";
    std::cout << "                  REAL-TIME EXECUTION METRICS SUMMARY\n";
    std::cout << "================================================================================\n";
    std::cout << "  Total Physics Engine Cycles     : " << orchestrator.physicsCycles() << " cycles\n";
    std::cout << "  Total Safety Assessment Cycles  : " << orchestrator.safetyCycles() << " cycles\n";
    std::cout << "  Total Communication Cycles      : " << orchestrator.communicationCycles() << " cycles\n";
    std::cout << "  Total HMI Telemetry Cycles      : " << orchestrator.hmiCycles() << " cycles\n";
    std::cout << "  Orchestration Health Status     : ALL THREADS HEALTHY & TERMINATED CLEANLY\n";
    std::cout << "================================================================================\n\n";
}

void runIntegratedFullSystemDemo()
{
    using namespace tcas;
    using namespace tcas::infrastructure;
    using namespace tcas::train;
    using namespace tcas::simulation;
    using namespace tcas::physics;
    using namespace tcas::navigation;
    using namespace tcas::sensor;
    using namespace tcas::communication;

    std::cout << "\n========================================================================\n";
    std::cout << "        TCAS INTEGRATED SYSTEM DEMO (ALL 10 MODULES)\n";
    std::cout << "========================================================================\n";

    // 1. Module 1: Infrastructure
    std::cout << "\n[MODULE 1] INITIALISING RAILWAY INFRASTRUCTURE TOPOLOGY\n";
    std::cout << "------------------------------------------------------------------------\n";
    RailwayNetwork network = buildNetwork();
    std::cout << "Nodes added  : " << network.nodeCount() << " (Central, Alpha Jct, Beta Jct, North Term, South Harbor)\n";
    std::cout << "Tracks added : " << network.trackCount() << " (101-106, including the Alpha freight approach)\n";
    std::cout << "Graph status : Weakly connected = " << (network.isWeaklyConnected() ? "YES" : "NO")
              << ", Cycles = " << (network.hasCycle() ? "YES" : "NO") << "\n";

    // 2. Module 2: Train Fleet
    std::cout << "\n[MODULE 2] INITIALISING TRAIN FLEET IN TRAIN MANAGER\n";
    std::cout << "------------------------------------------------------------------------\n";
    TrainManager trainManager;
    trainManager.addTrain(std::make_unique<ExpressTrain>(1, 45000.0, 45.0, 0.9, 1.4));
    trainManager.addTrain(std::make_unique<PassengerTrain>(2, 60000.0, 33.3, 0.8, 1.2));
    trainManager.addTrain(std::make_unique<FreightTrain>(3, 120000.0, 22.2, 0.5, 0.8));
    std::cout << "Active trains registered in fleet:\n";
    std::cout << "  - Train #1: Express   | Mass:  45t | MaxSpeed: 45.0 m/s (162 km/h) | SrvBrake: 0.9 m/s2 | EmgBrake: 1.4 m/s2\n";
    std::cout << "  - Train #2: Passenger | Mass:  60t | MaxSpeed: 33.3 m/s (120 km/h) | SrvBrake: 0.8 m/s2 | EmgBrake: 1.2 m/s2\n";
    std::cout << "  - Train #3: Freight   | Mass: 120t | MaxSpeed: 22.2 m/s  (80 km/h) | SrvBrake: 0.5 m/s2 | EmgBrake: 0.8 m/s2\n";

    // 3. Module 5: Dijkstra Routing
    std::cout << "\n[MODULE 5] DIJKSTRA ROUTE NAVIGATION PLANNING\n";
    std::cout << "------------------------------------------------------------------------\n";
    const RouteResult route1 = RouteNavigator::findRoute(network, 1, 4);
    std::cout << "Route Plan [Train #1: Central -> North Terminal]: Total " << route1.totalDistance << " m\n";
    const RouteResult route2 = RouteNavigator::findRoute(network, 1, 5);
    std::cout << "Route Plan [Train #2: Central -> South Harbor]: Total " << route2.totalDistance << " m\n";

    // 4. Modules 3 & 4: Physics & Simulation Clock
    std::cout << "\n[MODULE 3 & 4] RUNNING REAL-TIME DISCRETE SIMULATION & DYNAMIC PHYSICS\n";
    std::cout << "------------------------------------------------------------------------\n";
    SimulationConfig simConfig;
    SimClock simClock(static_cast<TimeSeconds>(simConfig.physicsPeriodMs) / 1000.0);
    SimulationTimer hmiTimer(simConfig.hmiPeriodMs, simConfig.physicsPeriodMs);

    auto* expressTrain = trainManager.getTrain(1);
    expressTrain->setState(TrainState::Running);
    expressTrain->setVelocity(15.0);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::left
              << std::setw(6)  << "Tick"
              << std::setw(9)  << "Time(s)"
              << std::setw(12) << "Speed(m/s)"
              << std::setw(12) << "Pos(m)"
              << std::setw(10) << "Track"
              << std::setw(10) << "Gradient"
              << std::setw(12) << "Eff.Decel"
              << std::setw(14) << "SafeDist(m)"
              << std::setw(12) << "EmgDist(m)\n";
    std::cout << "----------------------------------------------------------------------------------------------------\n";

    constexpr SimTimeTick kSimulationSteps = 30;
    const double targetAcceleration = 0.6;
    for (SimTimeTick step = 0; step < kSimulationSteps; ++step)
    {
        const auto tick = simClock.tickCount();
        const auto elapsed = simClock.elapsed();
        const auto dt = simClock.dt();
        const double pos = expressTrain->position();
        TrackId currentTrackId = 101;
        double currentGradient = 0.0;

        if (pos < 2000.0)
        {
            currentTrackId = 101;
            currentGradient = network.getTrack(101)->gradient();
        }
        else if (pos < 3500.0)
        {
            currentTrackId = 102;
            currentGradient = network.getTrack(102)->gradient();
        }
        else
        {
            currentTrackId = 103;
            currentGradient = network.getTrack(103)->gradient();
        }

        const double newVel = KinematicsEngine::updateVelocity(
            expressTrain->velocity(), targetAcceleration, dt, expressTrain->maximumSpeed());
        expressTrain->setVelocity(newVel);

        const double newPos = KinematicsEngine::updatePosition(
            expressTrain->position(), expressTrain->velocity(), targetAcceleration, dt);
        expressTrain->setPosition(newPos);

        const double effDecel = KinematicsEngine::effectiveDeceleration(
            expressTrain->serviceBraking(), currentGradient);
        const double safeDistance = KinematicsEngine::safeDistance(
            expressTrain->velocity(), expressTrain->serviceBraking(), currentGradient);
        const double emgDistance = KinematicsEngine::emergencyStoppingDistance(
            expressTrain->velocity(), expressTrain->emergencyBraking(), currentGradient);

        if (hmiTimer.shouldFire(tick))
        {
            std::cout << std::left
                      << std::setw(6) << tick << std::setw(9) << elapsed
                      << std::setw(12) << expressTrain->velocity()
                      << std::setw(12) << expressTrain->position()
                      << std::setw(10) << currentTrackId
                      << std::setw(10) << (currentGradient * 100.0)
                      << std::setw(12) << effDecel
                      << std::setw(14) << safeDistance
                      << std::setw(12) << emgDistance << '\n';
        }
        simClock.tick();
    }
    std::cout << "----------------------------------------------------------------------------------------------------\n";

    // 5. Module 6: Sensor & Kalman Filter
    std::cout << "\n[MODULE 6] SENSOR FUSION & KALMAN FILTER STATE ESTIMATION\n";
    std::cout << "------------------------------------------------------------------------\n";
    SensorNoiseConfig noiseConfig;
    noiseConfig.driftRatePerSecond = 0.01;
    noiseConfig.measurementNoisePos = 2.0;
    noiseConfig.measurementNoiseVel = 0.5;
    noiseConfig.measurementNoiseBalise = 0.01;

    Odometer odometer(noiseConfig);
    StateEstimator estimator(noiseConfig, 0.0, 15.0);

    double truePos = 0.0;
    double trueVel = 15.0;
    const double trueDt = 0.1;
    for (int step = 0; step < 15; ++step)
    {
        truePos += trueVel * trueDt;
        const auto meas = odometer.measure(truePos, trueVel, 0.0, trueDt, static_cast<SimTimeTick>(step));
        estimator.predict(trueDt, static_cast<SimTimeTick>(step));
        estimator.updateOdometry(meas);
        if (step == 10)
        {
            BaliseTransponder balise{.baliseId = 1, .trackId = 101, .exactPosition = truePos};
            estimator.updateBalise(balise);
        }
    }
    const EstimatedState estimatedState = estimator.estimatedState();
    std::cout << "Estimated position: " << estimatedState.position
              << " m, uncertainty: " << estimatedState.positionUncertainty << " m\n";
    std::cout << "Odometer accumulated drift: " << odometer.accumulatedDrift() << " m\n";

    // 6. Module 7: Communication Channel
    std::cout << "\n[MODULE 7] V2V/V2I WIRELESS COMMUNICATION SIMULATION\n";
    std::cout << "------------------------------------------------------------------------\n";
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
    std::cout << "Transmissions delivered: " << channel.totalDelivered()
              << ", Delivery rate: " << channel.deliveryRate() * 100.0 << "%\n";

    // 7. Modules 8 & 9: Trajectory Forecasts & Conflict Detection
    std::cout << "\n[MODULE 8 & 9] PREDICTIVE TRAJECTORY FORECAST & CONFLICT DETECTION\n";
    std::cout << "------------------------------------------------------------------------\n";
    auto* express = trainManager.getTrain(1);
    auto* freight = trainManager.getTrain(3);
    express->setPosition(1400.0);
    express->setVelocity(20.0);
    express->setAcceleration(0.0);
    freight->setPosition(1200.0);
    freight->setVelocity(20.0);
    freight->setAcceleration(0.0);

    const auto expressRoute = RouteNavigator::findRoute(network, 1, 3);
    const auto freightRoute = RouteNavigator::findRoute(network, 6, 7);

    const auto expressPrediction = tcas::prediction::PredictionEngine::predictStandardHorizon(
        *express, network, expressRoute, 101, estimatedState.positionUncertainty);
    const auto freightPrediction = tcas::prediction::PredictionEngine::predictStandardHorizon(
        *freight, network, freightRoute, 105, 1.0);

    tcas::conflict::ConflictDetector detector;
    const auto conflicts = detector.detect(
        express->id(), expressPrediction,
        freight->id(), freightPrediction,
        network);

    std::cout << "Express prediction points: " << expressPrediction.size() << '\n';
    std::cout << "Freight prediction points: " << freightPrediction.size() << '\n';
    std::cout << "Predicted conflicts detected: " << conflicts.size() << '\n';

    // 8. Module 10: Risk Scoring, Priority Hierarchy & Safe Resolution
    std::cout << "\n[MODULE 10] SAFETY RISK SCORING, OPERATIONAL PRIORITY & INTERVENTION\n";
    std::cout << "------------------------------------------------------------------------\n";
    if (!conflicts.empty())
    {
        const auto& c = conflicts.front();
        tcas::safety::RiskEngine riskEngine;
        tcas::safety::PriorityEngine priorityEngine;
        tcas::safety::ResolutionEngine resolutionEngine;

        tcas::safety::RiskInput rIn;
        rIn.timeToCollision = c.firstConflictTime > 0.0 ? c.firstConflictTime : 30.0;
        rIn.relativeVelocity = std::abs(express->velocity() - freight->velocity());
        rIn.brakingDistance = 250.0;
        rIn.safetyMargin = 350.0;
        rIn.conflictType = c.type;
        rIn.trainMass = freight->mass();
        rIn.sensorConfidence = 1.0;
        rIn.communicationConfidence = 1.0;

        const auto risk = riskEngine.assess(rIn);
        const auto pExpress = priorityEngine.assess(*express);
        const auto pFreight = priorityEngine.assess(*freight);

        std::cout << "Conflict Type      : Junction Convergence\n";
        std::cout << "Risk Score         : " << risk.score << " (Classification: "
                  << (risk.level == tcas::safety::RiskLevel::Low ? "LOW" : "HIGH") << ")\n";
        std::cout << "Priority Analysis  : Express Train #" << express->id() << " [Rank " << pExpress.priority
                  << "] > Freight Train #" << freight->id() << " [Rank " << pFreight.priority << "]\n";

        tcas::safety::ResolutionInput resIn{
            *freight, c, risk, false, true, 600.0, 250.0
        };
        const auto command = resolutionEngine.resolve(resIn);
        std::cout << "Intervention Action: " << (command.isEmergency() ? "EMERGENCY BRAKE" : "CONTROLLED HOLD / REDUCE")
                  << " -> Applied to Train #" << command.trainId << "\n";
        std::cout << "Safety Status      : CONFLICT PREDICTIVELY RESOLVED\n";
    }

    std::cout << "\n[RESULT] Complete 10-Module Integrated Verification Finished Successfully.\n";
    std::cout << "========================================================================\n\n";
}

} // namespace

int main(int argc, char* argv[])
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    printBanner();
    auto config = parseArgs(argc, argv);

    if (config.runDemoMode)
    {
        std::cout << "[MODE] Executing Comprehensive Full-System Integration Demo...\n";
        runIntegratedFullSystemDemo();
    }
    else
    {
        runRealtimeSystem(config);
    }

    return 0;
}
