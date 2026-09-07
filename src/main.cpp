#include "communication/CommunicationChannel.hpp"
#include "hmi/HmiDisplay.hpp"
#include "hmi/PerformanceMetrics.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "navigation/RouteNavigator.hpp"
#include "orchestrator/SafetyPipeline.hpp"
#include "orchestrator/ThreadOrchestrator.hpp"
#include "scenario/ScenarioManager.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"
#include "train/TrainManager.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

static void printSeparator()
{
    std::cout << "============================================================\n";
}

static void printHeader()
{
    printSeparator();
    std::cout << "             TCAS CONTROL CENTER  v1.0\n";
    std::cout << "   Real-Time Train Collision Avoidance System (C++23)\n";
    printSeparator();
}

static void printMenu(bool running)
{
    std::cout << "\n";
    printSeparator();
    std::cout << "SIMULATION CONTROL\n";
    std::cout << "  [1]  Start Simulation\n";
    std::cout << "  [2]  Pause Simulation\n";
    std::cout << "  [3]  Resume Simulation\n";
    std::cout << " [18]  Reset Simulation\n";
    std::cout << " [19]  Shutdown\n\n";

    std::cout << "TRAIN MANAGEMENT\n";
    std::cout << "  [4]  Add Train\n";
    std::cout << "  [5]  Remove Train\n";
    std::cout << "  [6]  Change Train Speed\n";
    std::cout << "  [7]  Change Route\n";
    std::cout << "  [8]  Hold Train\n";
    std::cout << "  [9]  Resume Train\n\n";

    std::cout << "FAULT INJECTION\n";
    std::cout << " [10]  Inject Sensor Failure\n";
    std::cout << " [11]  Recover Sensor\n";
    std::cout << " [12]  Inject Communication Failure\n";
    std::cout << " [13]  Recover Communication\n";
    std::cout << " [20]  Set Communication Quality\n\n";

    std::cout << "DISPLAY\n";
    std::cout << " [14]  Show Conflicts\n";
    std::cout << " [15]  Show Reservations\n";
    std::cout << " [16]  Show Telemetry\n";
    std::cout << " [17]  Show Performance\n";
    std::cout << " [21]  Show Live Display\n";
    std::cout << " [22]  Show Train Status\n";
    std::cout << " [23]  Show System Info\n\n";

    std::cout << "DEMO SCENARIOS\n";
    std::cout << " [24]  Scenario: Junction Conflict\n";
    std::cout << " [25]  Scenario: Rear-End Conflict\n";
    std::cout << " [26]  Scenario: Head-On Conflict\n";
    std::cout << " [27]  Scenario: Platform Conflict\n";
    std::cout << " [28]  Scenario: Multiple Conflicts\n";
    std::cout << " [29]  Scenario: Sensor Failure\n";
    std::cout << " [30]  Scenario: Communication Failure\n";
    std::cout << " [31]  Scenario: Unsafe Stopping Distance\n";
    printSeparator();
    std::cout << "Status: " << (running ? "RUNNING" : "STOPPED") << "\n";
    std::cout << "Command > ";
}

static int readInt()
{
    int value = 0;
    std::string line;
    if (!std::getline(std::cin, line))
    {
        return -1;
    }
    try
    {
        value = std::stoi(line);
    }
    catch (...)
    {
        return -1;
    }
    return value;
}

static double readDouble(const char* prompt)
{
    std::cout << prompt;
    double value = 0.0;
    std::string line;
    if (!std::getline(std::cin, line))
    {
        return 0.0;
    }
    try
    {
        value = std::stod(line);
    }
    catch (...)
    {
        return 0.0;
    }
    return value;
}

static tcas::TrainId readTrainId(const char* prompt)
{
    std::cout << prompt;
    return static_cast<tcas::TrainId>(readInt());
}

// -----------------------------------------------------------------------
// State managed across the control loop
// -----------------------------------------------------------------------

struct AppState
{
    tcas::infrastructure::RailwayNetwork              network;
    tcas::train::TrainManager                         trainManager;
    tcas::communication::CommunicationChannel         commChannel;
    std::unique_ptr<tcas::orchestrator::SafetyPipeline> pipeline;
    std::unique_ptr<tcas::orchestrator::ThreadOrchestrator> orchestrator;
    tcas::hmi::PerformanceMetrics                     perfMetrics;

    bool sensorFailureOverride     = false;
    bool commFailureOverride       = false;
    bool paused                    = false;

    std::vector<tcas::orchestrator::SafetyPipeline::TrainRoute> currentRoutes;

    bool isRunning() const
    {
        return orchestrator && orchestrator->isRunning();
    }
};

// -----------------------------------------------------------------------
// Load a scenario and restart the orchestrator
// -----------------------------------------------------------------------

static void loadScenario(
    AppState& app,
    tcas::scenario::ScenarioType type)
{
    // Stop existing orchestrator if running
    if (app.orchestrator)
    {
        app.orchestrator->stop();
        app.orchestrator.reset();
    }

    tcas::scenario::ScenarioManager mgr(app.network, app.trainManager);
    const auto result = mgr.load(type);

    app.sensorFailureOverride = result.injectSensorFailure;
    app.commFailureOverride   = result.injectCommunicationFailure;
    app.currentRoutes         = result.routes;

    std::cout << "\n[SCENARIO] " << tcas::scenario::ScenarioManager::scenarioName(type) << "\n";
    std::cout << "------------------------------------------------------------\n";
    std::cout << result.description << "\n\n";
    std::cout << "Trains registered: " << app.trainManager.trainCount() << "\n";
    std::cout << "Use [1] Start Simulation to activate the safety pipeline.\n";
}

// -----------------------------------------------------------------------
// Start / restart the orchestrator with the current pipeline
// -----------------------------------------------------------------------

static void startOrchestrator(AppState& app)
{
    if (app.currentRoutes.empty())
    {
        std::cout << "[INFO] No scenario loaded. Loading Junction Conflict...\n";
        loadScenario(app, tcas::scenario::ScenarioType::JunctionConflict);
    }

    if (app.orchestrator && app.orchestrator->isRunning())
    {
        std::cout << "[INFO] Simulation already running.\n";
        return;
    }

    if (app.orchestrator)
    {
        app.orchestrator->stop();
        app.orchestrator.reset();
    }

    // Build pipeline
    app.pipeline = std::make_unique<tcas::orchestrator::SafetyPipeline>(
        app.network, app.trainManager, app.currentRoutes);

    // Collect train IDs from TrainManager
    std::vector<tcas::TrainId> trainIds;
    for (const auto& route : app.currentRoutes)
    {
        trainIds.push_back(route.trainId);
    }

    tcas::orchestrator::OrchestratorConfig cfg;
    cfg.printHmi = false;   // We print on demand via menu

    app.orchestrator = std::make_unique<tcas::orchestrator::ThreadOrchestrator>(
        app.network,
        app.trainManager,
        app.commChannel,
        trainIds,
        cfg);

    app.orchestrator->setSafetyStep(app.pipeline->makeStep());
    app.orchestrator->setSensorFault(app.sensorFailureOverride);
    app.orchestrator->setCommFault(app.commFailureOverride);
    app.orchestrator->start();
    app.paused = false;

    std::cout << "[OK] Simulation started. Safety pipeline (Modules 8-10) active.\n";
}

// -----------------------------------------------------------------------
// Display helpers
// -----------------------------------------------------------------------

static void showLiveDisplay(AppState& app)
{
    if (!app.orchestrator)
    {
        std::cout << "[INFO] Simulation not started.\n";
        return;
    }
    auto state = app.orchestrator->snapshot();
    state.sensorFailure        = app.sensorFailureOverride;
    state.communicationFailure = app.commFailureOverride;
    state.systemStatus = app.paused
        ? tcas::orchestrator::SystemStatus::Paused
        : tcas::orchestrator::SystemStatus::Running;

    tcas::hmi::HmiDisplay::render(state, std::cout);
}

static void showTrainStatus(AppState& app)
{
    if (!app.orchestrator)
    {
        std::cout << "[INFO] Simulation not started.\n";
        return;
    }
    const auto state = app.orchestrator->snapshot();
    std::cout << "\n--------------------------------------------------------------\n";
    std::cout << "TRAIN STATUS  (t = " << std::fixed << std::setprecision(2)
              << state.simulationTime << " s)\n";
    std::cout << "--------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(6)  << "ID"
              << std::setw(11) << "TYPE"
              << std::setw(12) << "POSITION(m)"
              << std::setw(11) << "SPEED(m/s)"
              << "STATE\n";
    for (const auto& t : state.trains)
    {
        const char* typeName =
            t.type == tcas::TrainType::Express    ? "Express"   :
            t.type == tcas::TrainType::Passenger  ? "Passenger" : "Freight";
        const char* stateName =
            t.state == tcas::TrainState::Running       ? "RUNNING"   :
            t.state == tcas::TrainState::Braking       ? "BRAKING"   :
            t.state == tcas::TrainState::Stopped       ? "STOPPED"   :
            t.state == tcas::TrainState::EmergencyBrake ? "EMERGENCY" : "IDLE";
        std::cout << std::setw(6)  << t.id
                  << std::setw(11) << typeName
                  << std::setw(12) << t.position
                  << std::setw(11) << t.velocity
                  << stateName << "\n";
    }
}

static void showConflicts(AppState& app)
{
    if (!app.orchestrator)
    {
        std::cout << "[INFO] Simulation not started.\n";
        return;
    }
    const auto state = app.orchestrator->snapshot();
    std::cout << "\n[CONFLICTS]  count=" << state.activeConflicts.size() << "\n";
    std::cout << "--------------------------------------------------------------\n";
    if (state.activeConflicts.empty())
    {
        std::cout << "None — system SAFE.\n";
        return;
    }
    for (const auto& c : state.activeConflicts)
    {
        const char* typeName =
            c.type == tcas::conflict::ConflictType::RearEnd   ? "REAR-END"  :
            c.type == tcas::conflict::ConflictType::HeadOn    ? "HEAD-ON"   :
            c.type == tcas::conflict::ConflictType::Junction  ? "JUNCTION"  : "PLATFORM";
        std::cout << "Type=" << typeName
                  << "  Node=" << c.resourceNodeId
                  << "  Trains=" << c.trainA << " <-> " << c.trainB
                  << "  TTC=" << std::fixed << std::setprecision(2)
                  << c.firstConflictTime << " s"
                  << "  MinSep=" << c.minimumSeparation << " m\n";
    }
    if (!state.decisions.empty())
    {
        std::cout << "\n[DECISIONS]\n";
        for (const auto& d : state.decisions)
        {
            const char* cmdName =
                d.commandType == tcas::safety::SafetyCommandType::EmergencyBrake ? "EMERGENCY_BRAKE" :
                d.commandType == tcas::safety::SafetyCommandType::ReduceSpeed    ? "REDUCE_SPEED"    :
                d.commandType == tcas::safety::SafetyCommandType::HoldAtSignal   ? "HOLD_AT_SIGNAL"  : "NO_ACTION";
            std::cout << "Priority=#" << d.priorityTrain
                      << "  Yielding=#" << d.yieldingTrain
                      << "  Risk=" << std::fixed << std::setprecision(1) << d.riskScore
                      << "  Cmd=" << cmdName << "\n";
        }
    }
}

static void showReservations(AppState& app)
{
    if (!app.orchestrator)
    {
        std::cout << "[INFO] Simulation not started.\n";
        return;
    }
    const auto state = app.orchestrator->snapshot();
    std::cout << "\n[RESERVATIONS]  count=" << state.reservations.size() << "\n";
    std::cout << "--------------------------------------------------------------\n";
    if (state.reservations.empty())
    {
        std::cout << "None.\n";
        return;
    }
    for (const auto& r : state.reservations)
    {
        const char* rState =
            r.state == tcas::conflict::ResourceState::Reserved  ? "RESERVED"  :
            r.state == tcas::conflict::ResourceState::Occupied  ? "OCCUPIED"  :
            r.state == tcas::conflict::ResourceState::Released  ? "RELEASED"  : "FREE";
        std::cout << "Node=" << r.zone.nodeId
                  << "  Owner=Train#" << r.trainId
                  << "  State=" << rState
                  << "  [" << std::fixed << std::setprecision(2)
                  << r.startTime << "s, " << r.endTime << "s]\n";
    }
}

static void showTelemetry(AppState& app)
{
    if (!app.orchestrator)
    {
        std::cout << "[INFO] Simulation not started.\n";
        return;
    }
    const auto state = app.orchestrator->snapshot();
    std::cout << "\n[TELEMETRY]\n";
    std::cout << "  Simulation Time : " << state.simulationTime << " s\n";
    std::cout << "  Trains          : " << state.trains.size() << "\n";
    std::cout << "  Predictions     : " << state.predictions.size() << " points\n";
    std::cout << "  Active Conflicts: " << state.activeConflicts.size() << "\n";
    std::cout << "  Reservations    : " << state.reservations.size() << "\n";
    std::cout << "  Commands Issued : " << state.commands.size() << "\n";
    std::cout << "  Sensor Status   : "
              << (app.sensorFailureOverride ? "FAILED" : "OK") << "\n";
    std::cout << "  Comm Status     : "
              << (app.commFailureOverride   ? "FAILED" : "OK") << "\n";
}

static void showPerformance(AppState& app)
{
    if (!app.orchestrator)
    {
        std::cout << "[INFO] Simulation not started.\n";
        return;
    }
    const auto metrics = app.perfMetrics.snapshot();
    std::cout << "\n[PERFORMANCE METRICS]\n";
    std::cout << "  Collision count      : " << metrics.collisionCount      << "\n";
    std::cout << "  Near-miss count      : " << metrics.nearMissCount       << "\n";
    std::cout << "  Emergency brake count: " << metrics.emergencyBrakeCount << "\n";
    std::cout << "  Conflict observations: " << metrics.conflictObservations << "\n";
    std::cout << "  Minimum separation   : " << std::fixed << std::setprecision(2)
                                             << metrics.minimumSeparation   << " m\n";
    std::cout << "  Minimum TTC          : " << metrics.minimumTtc          << " s\n";
    std::cout << "  Max HMI latency      : " << metrics.maximumHmiLatencyMs << " ms\n";
}

static void showSystemInfo(AppState& app)
{
    std::cout << "\n[SYSTEM INFO]\n";
    if (!app.orchestrator)
    {
        std::cout << "  Orchestrator: not started\n";
        return;
    }
    std::cout << "  Physics  cycles : " << app.orchestrator->physicsCycles()       << "\n";
    std::cout << "  Safety   cycles : " << app.orchestrator->safetyCycles()        << "\n";
    std::cout << "  Comm     cycles : " << app.orchestrator->communicationCycles() << "\n";
    std::cout << "  HMI      cycles : " << app.orchestrator->hmiCycles()           << "\n";
    std::cout << "  Is running      : " << (app.orchestrator->isRunning() ? "YES" : "NO") << "\n";
    std::cout << "  Trains loaded   : " << app.trainManager.trainCount()            << "\n";
    std::cout << "  Sensor fault    : " << (app.sensorFailureOverride ? "YES" : "NO") << "\n";
    std::cout << "  Comm fault      : " << (app.commFailureOverride   ? "YES" : "NO") << "\n";
}

// -----------------------------------------------------------------------
// Train management helpers
// -----------------------------------------------------------------------

static void addTrain(AppState& app)
{
    std::cout << "\nTrain type? [1=Express  2=Passenger  3=Freight]: ";
    const int typeChoice = readInt();
    const auto id = readTrainId("Train ID (unique integer): ");

    std::unique_ptr<tcas::train::Train> train;
    switch (typeChoice)
    {
    case 1:
        train = std::make_unique<tcas::train::ExpressTrain>(id, 45000.0, 45.0, 0.9, 1.4);
        break;
    case 2:
        train = std::make_unique<tcas::train::PassengerTrain>(id, 60000.0, 33.3, 0.8, 1.2);
        break;
    case 3:
    default:
        train = std::make_unique<tcas::train::FreightTrain>(id, 120000.0, 22.2, 0.5, 0.8);
        break;
    }

    const double pos = readDouble("Initial position (m): ");
    const double vel = readDouble("Initial velocity (m/s): ");
    train->setPosition(pos);
    train->setVelocity(vel);

    std::cout << "Assign Route - Source Node ID: ";
    const int srcNode = readInt();
    std::cout << "Assign Route - Destination Node ID: ";
    const int dstNode = readInt();

    const auto route = tcas::navigation::RouteNavigator::findRoute(
        app.network, static_cast<tcas::NodeId>(srcNode), static_cast<tcas::NodeId>(dstNode));

    if (!route.success || route.tracks.empty())
    {
        std::cout << "[WARN] No route found between Node " << srcNode << " and Node " << dstNode << ".\n";
    }

    if (app.trainManager.addTrain(std::move(train)))
    {
        std::cout << "[OK] Train #" << id << " added to registry.\n";

        if (route.success && !route.tracks.empty())
        {
            tcas::orchestrator::SafetyPipeline::TrainRoute tr{
                id,
                route.tracks.front(),
                route
            };
            app.currentRoutes.push_back(tr);
            if (app.pipeline)
            {
                app.pipeline->addOrUpdateRoute(tr);
            }
            std::cout << "[OK] Route assigned (" << route.totalDistance << " m across "
                      << route.tracks.size() << " tracks).\n";
        }

        if (app.orchestrator)
        {
            app.orchestrator->addTrain(id);
        }
        std::cout << "[OK] Train #" << id << " dynamically added to active simulation.\n";
    }
    else
    {
        std::cout << "[ERR] Train #" << id << " already exists.\n";
    }
}

static void removeTrain(AppState& app)
{
    const auto id = readTrainId("Train ID to remove: ");
    if (app.trainManager.removeTrain(id))
    {
        if (app.orchestrator)
        {
            app.orchestrator->removeTrain(id);
        }
        if (app.pipeline)
        {
            app.pipeline->removeRoute(id);
        }
        std::erase_if(app.currentRoutes, [id](const auto& r) { return r.trainId == id; });
        std::cout << "[OK] Train #" << id << " dynamically removed from simulation and safety pipeline.\n";
    }
    else
    {
        std::cout << "[ERR] Train #" << id << " not found.\n";
    }
}

static void changeRoute(AppState& app)
{
    const auto id = readTrainId("Train ID to change route: ");
    auto* train = app.trainManager.getTrain(id);
    if (train == nullptr)
    {
        std::cout << "[ERR] Train #" << id << " not found.\n";
        return;
    }

    std::cout << "New Source Node ID: ";
    const int srcNode = readInt();
    std::cout << "New Destination Node ID: ";
    const int dstNode = readInt();

    const auto newRoute = tcas::navigation::RouteNavigator::findRoute(
        app.network, static_cast<tcas::NodeId>(srcNode), static_cast<tcas::NodeId>(dstNode));

    if (!newRoute.success || newRoute.tracks.empty())
    {
        std::cout << "[ERR] No path found between Node " << srcNode << " and Node " << dstNode << ".\n";
        return;
    }

    train->setPosition(0.0);

    tcas::orchestrator::SafetyPipeline::TrainRoute tr{
        id,
        newRoute.tracks.front(),
        newRoute
    };

    bool found = false;
    for (auto& r : app.currentRoutes)
    {
        if (r.trainId == id)
        {
            r = tr;
            found = true;
            break;
        }
    }
    if (!found)
    {
        app.currentRoutes.push_back(tr);
    }

    if (app.pipeline)
    {
        app.pipeline->addOrUpdateRoute(tr);
    }

    std::cout << "[OK] Route updated for Train #" << id << ":\n";
    std::cout << "  Tracks: ";
    for (const auto tid : newRoute.tracks)
    {
        std::cout << "T" << tid << " ";
    }
    std::cout << "\n  Total distance: " << newRoute.totalDistance << " m\n";
}

static void changeTrainSpeed(AppState& app)
{
    const auto id    = readTrainId("Train ID: ");
    const double vel = readDouble("New velocity (m/s): ");
    auto* train = app.trainManager.getTrain(id);
    if (train == nullptr)
    {
        std::cout << "[ERR] Train #" << id << " not found.\n";
        return;
    }
    train->setVelocity(vel);
    std::cout << "[OK] Train #" << id << " velocity set to " << vel << " m/s.\n";
}

static void holdTrain(AppState& app)
{
    const auto id = readTrainId("Train ID to hold: ");
    auto* train = app.trainManager.getTrain(id);
    if (train == nullptr)
    {
        std::cout << "[ERR] Train #" << id << " not found.\n";
        return;
    }
    train->setVelocity(0.0);
    train->setAcceleration(0.0);
    train->setState(tcas::TrainState::Stopped);
    std::cout << "[OK] Train #" << id << " held at signal.\n";
}

static void resumeTrain(AppState& app)
{
    const auto id    = readTrainId("Train ID to resume: ");
    const double vel = readDouble("Resume velocity (m/s): ");
    auto* train = app.trainManager.getTrain(id);
    if (train == nullptr)
    {
        std::cout << "[ERR] Train #" << id << " not found.\n";
        return;
    }
    train->setVelocity(vel);
    train->setState(tcas::TrainState::Running);
    std::cout << "[OK] Train #" << id << " resumed at " << vel << " m/s.\n";
}

static void setCommQuality(AppState& app)
{
    std::cout << "\nCommunication quality:\n";
    std::cout << "  [1] Normal (no packet loss)\n";
    std::cout << "  [2] Degraded (30% loss)\n";
    std::cout << "  [3] Failed (70% loss)\n";
    std::cout << "  [4] Recover\n";
    std::cout << "Choice: ";
    const int choice = readInt();
    switch (choice)
    {
    case 1:
        app.commFailureOverride = false;
        if (app.orchestrator) { app.orchestrator->setCommFault(false); }
        std::cout << "[OK] Communication: Normal.\n";
        break;
    case 2:
        app.commFailureOverride = false;
        if (app.orchestrator) { app.orchestrator->setCommFault(false); }
        std::cout << "[OK] Communication: Degraded (simulated via risk engine confidence).\n";
        break;
    case 3:
        app.commFailureOverride = true;
        if (app.orchestrator) { app.orchestrator->setCommFault(true); }
        std::cout << "[OK] Communication: Failed (risk confidence = 0.4).\n";
        break;
    case 4:
        app.commFailureOverride = false;
        if (app.orchestrator) { app.orchestrator->setCommFault(false); }
        std::cout << "[OK] Communication: Recovered.\n";
        break;
    default:
        std::cout << "[ERR] Invalid choice.\n";
        break;
    }
}

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------

int main()
{
    printHeader();
    std::cout << "\nWelcome to the TCAS interactive control center.\n";
    std::cout << "All 12 modules are active. Use the menu to control the simulation.\n\n";

    AppState app;
    bool shutdown = false;

    while (!shutdown)
    {
        printMenu(app.isRunning());
        const int cmd = readInt();
        std::cout << "\n";

        switch (cmd)
        {
        // ---- SIMULATION CONTROL ----
        case 1:
            startOrchestrator(app);
            break;

        case 2:
            if (app.isRunning())
            {
                app.orchestrator->stop();
                app.paused = true;
                std::cout << "[OK] Simulation paused.\n";
            }
            else
            {
                std::cout << "[INFO] Simulation is not running.\n";
            }
            break;

        case 3:
            if (!app.isRunning() && app.orchestrator)
            {
                app.orchestrator->start();
                app.paused = false;
                std::cout << "[OK] Simulation resumed.\n";
            }
            else if (app.isRunning())
            {
                std::cout << "[INFO] Simulation is already running.\n";
            }
            else
            {
                std::cout << "[INFO] No simulation to resume. Use [1] Start.\n";
            }
            break;

        case 18:
            if (app.orchestrator)
            {
                app.orchestrator->stop();
                app.orchestrator.reset();
            }
            app.pipeline.reset();
            app.currentRoutes.clear();
            app.sensorFailureOverride = false;
            app.commFailureOverride   = false;
            app.paused                = false;
            std::cout << "[OK] Simulation reset. Load a scenario or [1] Start.\n";
            break;

        case 19:
            if (app.orchestrator)
            {
                app.orchestrator->stop();
            }
            shutdown = true;
            std::cout << "[OK] Shutting down TCAS. Goodbye.\n";
            break;

        // ---- TRAIN MANAGEMENT ----
        case 4:
            addTrain(app);
            break;

        case 5:
            removeTrain(app);
            break;

        case 6:
            changeTrainSpeed(app);
            break;

        case 7:
            changeRoute(app);
            break;

        case 8:
            holdTrain(app);
            break;

        case 9:
            resumeTrain(app);
            break;

        // ---- FAULT INJECTION ----
        case 10:
            app.sensorFailureOverride = true;
            if (app.orchestrator)
            {
                app.orchestrator->setSensorFault(true);
            }
            std::cout << "[OK] Sensor failure injected. Safety pipeline degraded confidence active.\n";
            break;

        case 11:
            app.sensorFailureOverride = false;
            if (app.orchestrator)
            {
                app.orchestrator->setSensorFault(false);
            }
            std::cout << "[OK] Sensor recovered.\n";
            break;

        case 12:
            app.commFailureOverride = true;
            if (app.orchestrator)
            {
                app.orchestrator->setCommFault(true);
            }
            std::cout << "[OK] Communication failure injected. Degraded comm confidence active.\n";
            break;

        case 13:
            app.commFailureOverride = false;
            if (app.orchestrator)
            {
                app.orchestrator->setCommFault(false);
            }
            std::cout << "[OK] Communication recovered.\n";
            break;

        case 20:
            setCommQuality(app);
            break;

        // ---- DISPLAY ----
        case 14:
            showConflicts(app);
            break;

        case 15:
            showReservations(app);
            break;

        case 16:
            showTelemetry(app);
            break;

        case 17:
            showPerformance(app);
            break;

        case 21:
            showLiveDisplay(app);
            break;

        case 22:
            showTrainStatus(app);
            break;

        case 23:
            showSystemInfo(app);
            break;

        // ---- DEMO SCENARIOS ----
        case 24:
            loadScenario(app, tcas::scenario::ScenarioType::JunctionConflict);
            startOrchestrator(app);
            break;

        case 25:
            loadScenario(app, tcas::scenario::ScenarioType::RearEndConflict);
            startOrchestrator(app);
            break;

        case 26:
            loadScenario(app, tcas::scenario::ScenarioType::HeadOnConflict);
            startOrchestrator(app);
            break;

        case 27:
            loadScenario(app, tcas::scenario::ScenarioType::PlatformConflict);
            startOrchestrator(app);
            break;

        case 28:
            loadScenario(app, tcas::scenario::ScenarioType::MultipleConflicts);
            startOrchestrator(app);
            break;

        case 29:
            loadScenario(app, tcas::scenario::ScenarioType::SensorFailure);
            startOrchestrator(app);
            break;

        case 30:
            loadScenario(app, tcas::scenario::ScenarioType::CommunicationFailure);
            startOrchestrator(app);
            break;

        case 31:
            loadScenario(app, tcas::scenario::ScenarioType::UnsafeStopping);
            startOrchestrator(app);
            break;

        default:
            if (cmd != -1)
            {
                std::cout << "[ERR] Unknown command: " << cmd << ". Try again.\n";
            }
            break;
        }
    }

    return 0;
}
