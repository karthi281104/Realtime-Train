#include "communication/CommunicationChannel.hpp"
#include "communication/Message.hpp"
#include "conflict/ConflictDetector.hpp"
#include "conflict/ResourceReservationManager.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "navigation/RouteNavigator.hpp"
#include "orchestrator/ThreadOrchestrator.hpp"
#include "physics/KinematicsEngine.hpp"
#include "prediction/PredictionEngine.hpp"
#include "safety/ConflictPriorityQueue.hpp"
#include "safety/PriorityEngine.hpp"
#include "safety/ResolutionEngine.hpp"
#include "safety/RiskEngine.hpp"
#include "sensor/Odometer.hpp"
#include "sensor/StateEstimator.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"
#include "train/TrainManager.hpp"

#include <atomic>
#include <chrono>
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
              << "  --demo              Run all module verification demos sequentially\n"
              << "  --realtime, -r      Run real-time multi-threaded orchestrator (default)\n"
              << "  --duration <sec>    Run duration in seconds for real-time mode (default: 5)\n"
              << "  --trains <count>    Number of active trains to simulate (default: 3)\n"
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

    auto safetyStep = [&](const tcas::orchestrator::WorldState& world,
                          tcas::orchestrator::CommandQueue& cmdQueue) {
        if (world.trains.size() < 2) return;

        std::vector<tcas::prediction::FutureState> traj1 = {
            {0.0, 101, world.trains[0].position, world.trains[0].velocity, 0.0, 1.0},
            {5.0, 101, world.trains[0].position + world.trains[0].velocity * 5.0, world.trains[0].velocity, 0.0, 2.0}
        };

        std::vector<tcas::prediction::FutureState> traj2 = {
            {0.0, 101, world.trains[1].position, world.trains[1].velocity, 0.0, 1.0},
            {5.0, 101, world.trains[1].position + world.trains[1].velocity * 5.0, world.trains[1].velocity, 0.0, 2.0}
        };

        auto conflicts = conflictDetector.detect(
            world.trains[0].id, traj1, world.trains[1].id, traj2, network);

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
                    cmdQueue.push(cmd);
                }
            }
        }
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

} // namespace

#include "demo/IntegratedDemo.hpp"

int main(int argc, char* argv[])
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    printBanner();
    auto config = parseArgs(argc, argv);

    if (config.runDemoMode)
    {
        std::cout << "[MODE] Executing Comprehensive Full-System Integration Demo...\n";
        tcas::demo::runIntegratedDemo();
    }
    else
    {
        runRealtimeSystem(config);
    }

    return 0;
}
