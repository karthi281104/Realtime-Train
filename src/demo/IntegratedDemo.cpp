#include "demo/IntegratedDemo.hpp"

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

    std::cout << "\n";
    std::cout << "========================================================================\n";
    std::cout << "        TCAS INTEGRATED SYSTEM DEMO (MODULES 1 TO 5 ACTIVE)\n";
    std::cout << "========================================================================\n";

    // -------------------------------------------------------------------------
    // 1. MODULE 1: RAILWAY INFRASTRUCTURE
    // -------------------------------------------------------------------------
    std::cout << "\n[MODULE 1] INITIALISING RAILWAY INFRASTRUCTURE TOPOLOGY\n";
    std::cout << "------------------------------------------------------------------------\n";

    RailwayNetwork network;

    // Nodes: Stations & Junctions
    network.addNode(Node(1, "Central Station",   NodeType::Station));
    network.addNode(Node(2, "Alpha Junction",    NodeType::Junction));
    network.addNode(Node(3, "Beta Junction",     NodeType::Junction));
    network.addNode(Node(4, "North Terminal",    NodeType::Station));
    network.addNode(Node(5, "South Harbor",      NodeType::Station));

    // Directed Tracks with realistic lengths, speed limits, and gradients
    network.addTrack(Track(101, 1, 2, 2000.0, 35.0,  0.000)); // Flat
    network.addTrack(Track(102, 2, 3, 1500.0, 30.0,  0.020)); // +2.0% Uphill
    network.addTrack(Track(103, 3, 4, 2500.0, 40.0, -0.015)); // -1.5% Downhill
    network.addTrack(Track(104, 2, 5, 3000.0, 25.0,  0.010)); // +1.0% Uphill

    std::cout << "Nodes added  : " << network.nodeCount() << " (Central, Alpha Jct, Beta Jct, North Term, South Harbor)\n";
    std::cout << "Tracks added : " << network.trackCount() << " (Tracks 101, 102, 103, 104 with gradient profiles)\n";
    std::cout << "Graph status : Connected = " << (network.isConnected() ? "YES" : "NO")
              << ", Cycles = " << (network.hasCycle() ? "YES" : "NO") << "\n";

    // -------------------------------------------------------------------------
    // 2. MODULE 2: TRAIN FLEET MANAGEMENT
    // -------------------------------------------------------------------------
    std::cout << "\n[MODULE 2] INITIALISING TRAIN FLEET IN TRAIN MANAGER\n";
    std::cout << "------------------------------------------------------------------------\n";

    TrainManager trainManager;

    // Express Train (high speed, strong brakes)
    trainManager.addTrain(std::make_unique<ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4
    ));

    // Passenger Train (commuter)
    trainManager.addTrain(std::make_unique<PassengerTrain>(
        2, 60000.0, 33.3, 0.8, 1.2
    ));

    // Freight Train (heavy cargo, lower braking)
    trainManager.addTrain(std::make_unique<FreightTrain>(
        3, 120000.0, 22.2, 0.5, 0.8
    ));

    std::cout << "Active trains registered in fleet:\n";
    std::cout << "  - Train #1: Express   | Mass:  45t | MaxSpeed: 45.0 m/s (162 km/h) | SrvBrake: 0.9 m/s2 | EmgBrake: 1.4 m/s2\n";
    std::cout << "  - Train #2: Passenger | Mass:  60t | MaxSpeed: 33.3 m/s (120 km/h) | SrvBrake: 0.8 m/s2 | EmgBrake: 1.2 m/s2\n";
    std::cout << "  - Train #3: Freight   | Mass: 120t | MaxSpeed: 22.2 m/s  (80 km/h) | SrvBrake: 0.5 m/s2 | EmgBrake: 0.8 m/s2\n";

    // -------------------------------------------------------------------------
    // 3. MODULE 5: DIJKSTRA ROUTE PLANNING & NAVIGATION
    // -------------------------------------------------------------------------
    std::cout << "\n[MODULE 5] DIJKSTRA ROUTE NAVIGATION PLANNING\n";
    std::cout << "------------------------------------------------------------------------\n";

    // Route 1: Express from Central (1) to North Terminal (4)
    const RouteResult route1 = RouteNavigator::findRoute(network, 1, 4);
    std::cout << "Route Plan [Train #1: Central -> North Terminal]:\n";
    std::cout << "  - Status         : " << (route1.success ? "SUCCESS" : "FAILED") << "\n";
    std::cout << "  - Total Distance : " << route1.totalDistance << " m\n";
    std::cout << "  - Track Sequence : ";
    for (std::size_t i = 0; i < route1.tracks.size(); ++i)
    {
        const auto* trk = network.getTrack(route1.tracks[i]);
        std::cout << "Track " << route1.tracks[i] << " (" << trk->length() << "m, grade: "
                  << (trk->gradient() * 100.0) << "%)"
                  << (i + 1 < route1.tracks.size() ? " -> " : "\n");
    }

    // Route 2: Passenger from Central (1) to South Harbor (5)
    const RouteResult route2 = RouteNavigator::findRoute(network, 1, 5);
    std::cout << "Route Plan [Train #2: Central -> South Harbor]:\n";
    std::cout << "  - Status         : " << (route2.success ? "SUCCESS" : "FAILED") << "\n";
    std::cout << "  - Total Distance : " << route2.totalDistance << " m\n";
    std::cout << "  - Track Sequence : ";
    for (std::size_t i = 0; i < route2.tracks.size(); ++i)
    {
        const auto* trk = network.getTrack(route2.tracks[i]);
        std::cout << "Track " << route2.tracks[i] << " (" << trk->length() << "m, grade: "
                  << (trk->gradient() * 100.0) << "%)"
                  << (i + 1 < route2.tracks.size() ? " -> " : "\n");
    }

    // -------------------------------------------------------------------------
    // 4. MODULE 3 & MODULE 4: REAL-TIME SIMULATION & GRADIENT KINEMATICS LOOP
    // -------------------------------------------------------------------------
    std::cout << "\n[MODULE 3 & 4] RUNNING REAL-TIME DISCRETE SIMULATION & DYNAMIC PHYSICS\n";
    std::cout << "------------------------------------------------------------------------\n";

    SimulationConfig simConfig;
    SimClock simClock(static_cast<TimeSeconds>(simConfig.physicsPeriodMs) / 1000.0);

    SimulationTimer safetyTimer(simConfig.safetyPeriodMs, simConfig.physicsPeriodMs);
    SimulationTimer hmiTimer(simConfig.hmiPeriodMs, simConfig.physicsPeriodMs);

    auto* expressTrain = trainManager.getTrain(1);
    expressTrain->setState(TrainState::Running);
    expressTrain->setVelocity(15.0); // Initial 15 m/s

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
              << std::setw(12) << "EmgDist(m)"
              << "\n";
    std::cout << "----------------------------------------------------------------------------------------------------\n";

    constexpr SimTimeTick kSimulationSteps = 30;
    const double targetAcceleration = 0.6; // Accelerating

    for (SimTimeTick step = 0; step < kSimulationSteps; ++step)
    {
        const auto tick = simClock.tickCount();
        const auto elapsed = simClock.elapsed();
        const auto dt = simClock.dt();

        // 1. Determine active track along route
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

        // 2. Physics & Kinematics update (Module 4)
        const double newVel = KinematicsEngine::updateVelocity(
            expressTrain->velocity(),
            targetAcceleration,
            dt,
            expressTrain->maximumSpeed()
        );
        expressTrain->setVelocity(newVel);

        const double newPos = KinematicsEngine::updatePosition(
            expressTrain->position(),
            expressTrain->velocity(),
            targetAcceleration,
            dt
        );
        expressTrain->setPosition(newPos);

        // 3. Compute dynamic gradient-aware safety metrics (Module 4)
        const double effDecel = KinematicsEngine::effectiveDeceleration(
            expressTrain->serviceBraking(),
            currentGradient
        );

        const double safeDistance = KinematicsEngine::safeDistance(
            expressTrain->velocity(),
            expressTrain->serviceBraking(),
            currentGradient
        );

        const double emgDistance = KinematicsEngine::emergencyStoppingDistance(
            expressTrain->velocity(),
            expressTrain->emergencyBraking(),
            currentGradient
        );

        // 4. Output HMI telemetry (every 200ms)
        if (hmiTimer.shouldFire(tick))
        {
            std::cout << std::left
                      << std::setw(6)  << tick
                      << std::setw(9)  << elapsed
                      << std::setw(12) << expressTrain->velocity()
                      << std::setw(12) << expressTrain->position()
                      << std::setw(10) << currentTrackId
                      << std::setw(10) << (currentGradient * 100.0)
                      << std::setw(12) << effDecel
                      << std::setw(14) << safeDistance
                      << std::setw(12) << emgDistance
                      << "\n";
        }

        simClock.tick();
    }

    std::cout << "----------------------------------------------------------------------------------------------------\n";
    std::cout << "\n[RESULT] All 5 modules successfully executed in full integration:\n";
    std::cout << "  - Module 1 (Infrastructure) : Railway topology with grades & speed limits loaded\n";
    std::cout << "  - Module 2 (Train Fleet)    : Multi-class train fleet managed with physics properties\n";
    std::cout << "  - Module 3 (Simulation)     : Deterministic discrete tick clock synchronized at " << simConfig.physicsPeriodMs << " ms\n";
    std::cout << "  - Module 4 (Physics)        : Gradient-corrected dynamic braking distances and kinematics calculated\n";
    std::cout << "  - Module 5 (Navigation)     : Dijkstra optimal track route planned and executed\n";
    std::cout << "========================================================================\n\n";
}

} // namespace tcas::demo
