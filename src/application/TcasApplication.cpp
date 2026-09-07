#include "application/TcasApplication.hpp"

#include "hmi/HmiDisplay.hpp"
#include "navigation/RouteNavigator.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace tcas::app
{

namespace
{

void printSeparator()
{
    std::cout << "==============================================================\n";
}

int readInt()
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

double readDouble(const char* prompt)
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

TrainId readTrainId(const char* prompt)
{
    std::cout << prompt;
    return static_cast<TrainId>(readInt());
}

} // namespace

TcasApplication::TcasApplication()
{
    // Pre-load default Junction conflict scenario for immediate readiness
    loadScenario(scenario::ScenarioType::JunctionConflict);
}

TcasApplication::~TcasApplication()
{
    if (orchestrator_)
    {
        orchestrator_->stop();
    }
}

int TcasApplication::run()
{
    printHeader();
    std::cout << "\nWelcome to the TCAS Control Center.\n"
              << "Type a command number or choose a demo scenario.\n\n";

    while (!shutdown_)
    {
        printDashboard();
        printMenu();
        const int cmd = readInt();
        std::cout << "\n";
        handleCommand(cmd);
    }

    return 0;
}

void TcasApplication::printHeader() const
{
    printSeparator();
    std::cout << "                 TCAS CONTROL CENTER  v2.0\n"
              << "   Real-Time Train Collision Avoidance System (C++23)\n";
    printSeparator();
}

void TcasApplication::printDashboard()
{
    if (orchestrator_)
    {
        // Authoritative WorldState observation only — no UI mutation!
        const auto state = orchestrator_->snapshot();
        hmi::HmiDisplay::render(state, std::cout);
    }
    else
    {
        printSeparator();
        std::cout << "SYSTEM STATUS : READY (Simulation not yet active)\n";
        printSeparator();
    }
}

void TcasApplication::printMenu() const
{
    std::cout << "\nOPERATOR ACTIONS:\n"
              << "  [1] Start    [2] Pause    [3] Resume   [18] Reset   [19] Shutdown\n"
              << "  [4] Add Train             [5] Remove Train         [6] Change Speed\n"
              << "  [7] Change Route          [8] Hold Train           [9] Resume Train\n"
              << " [10] Inject Sensor Fault  [11] Recover Sensor\n"
              << " [12] Inject Comm Fault    [13] Recover Comm        [20] Comm Quality\n"
              << " [14] Show Conflicts       [15] Show Reservations   [16] Show Telemetry\n"
              << " [17] Show Performance     [22] Show Train Status   [23] Show System Info\n"
              << "DEMO SCENARIOS:\n"
              << " [24] Junction Conflict    [25] Rear-End Conflict   [26] Head-On Conflict\n"
              << " [27] Platform Conflict    [28] Multiple Conflicts  [29] Sensor Failure\n"
              << " [30] Comm Failure         [31] Unsafe Stopping Distance\n";
    printSeparator();
    std::cout << "Command > ";
}

void TcasApplication::handleCommand(int cmd)
{
    switch (cmd)
    {
    case 1:  startSimulation(); break;
    case 2:  pauseSimulation(); break;
    case 3:  resumeSimulation(); break;
    case 4:  addTrainInteractive(); break;
    case 5:  removeTrainInteractive(); break;
    case 6:  changeSpeedInteractive(); break;
    case 7:  changeRouteInteractive(); break;
    case 8:  holdTrainInteractive(); break;
    case 9:  resumeTrainInteractive(); break;
    case 10: injectSensorFaultInteractive(); break;
    case 11: recoverSensorInteractive(); break;
    case 12: injectCommFaultInteractive(); break;
    case 13: recoverCommInteractive(); break;
    case 14:
        if (orchestrator_)
        {
            const auto st = orchestrator_->snapshot();
            std::cout << "\n[ACTIVE CONFLICTS count=" << st.activeConflicts.size() << "]\n";
            for (const auto& c : st.activeConflicts)
            {
                std::cout << "  Conflict: Trains #" << c.trainA << " <-> #" << c.trainB
                          << " at Node " << c.resourceNodeId
                          << " | TTC: " << std::fixed << std::setprecision(2) << c.firstConflictTime << " s"
                          << " | MinSep: " << c.minimumSeparation << " m\n";
            }
        }
        break;
    case 15:
        if (orchestrator_)
        {
            const auto st = orchestrator_->snapshot();
            std::cout << "\n[RESERVATIONS count=" << st.reservations.size() << "]\n";
            for (const auto& r : st.reservations)
            {
                std::cout << "  Zone Node " << r.zone.nodeId
                          << " -> Train #" << r.trainId
                          << " [" << std::fixed << std::setprecision(2) << r.startTime << "s - " << r.endTime << "s]\n";
            }
        }
        break;
    case 16: showTelemetry(); break;
    case 17: showPerformance(); break;
    case 18: resetSimulation(); break;
    case 19:
        if (orchestrator_) { orchestrator_->stop(); }
        shutdown_ = true;
        std::cout << "[OK] TCAS Application shut down cleanly. Goodbye.\n";
        break;
    case 20: setCommQualityInteractive(); break;
    case 21: printDashboard(); break;
    case 22: showTrainStatus(); break;
    case 23: showSystemInfo(); break;

    // Scenarios
    case 24: loadScenario(scenario::ScenarioType::JunctionConflict); startSimulation(); break;
    case 25: loadScenario(scenario::ScenarioType::RearEndConflict); startSimulation(); break;
    case 26: loadScenario(scenario::ScenarioType::HeadOnConflict); startSimulation(); break;
    case 27: loadScenario(scenario::ScenarioType::PlatformConflict); startSimulation(); break;
    case 28: loadScenario(scenario::ScenarioType::MultipleConflicts); startSimulation(); break;
    case 29: loadScenario(scenario::ScenarioType::SensorFailure); startSimulation(); break;
    case 30: loadScenario(scenario::ScenarioType::CommunicationFailure); startSimulation(); break;
    case 31: loadScenario(scenario::ScenarioType::UnsafeStopping); startSimulation(); break;

    default:
        if (cmd != -1)
        {
            std::cout << "[ERR] Unknown command " << cmd << ". Enter a valid menu option.\n";
        }
        break;
    }
}

void TcasApplication::startSimulation()
{
    if (currentRoutes_.empty())
    {
        loadScenario(scenario::ScenarioType::JunctionConflict);
    }

    if (orchestrator_ && orchestrator_->isRunning())
    {
        if (orchestrator_->isPaused())
        {
            resumeSimulation();
        }
        else
        {
            std::cout << "[INFO] Simulation already running.\n";
        }
        return;
    }

    if (orchestrator_)
    {
        orchestrator_->stop();
        orchestrator_.reset();
    }

    pipeline_ = std::make_unique<orchestrator::SafetyPipeline>(
        network_, trainManager_, currentRoutes_);

    std::vector<TrainId> trainIds;
    for (const auto& route : currentRoutes_)
    {
        trainIds.push_back(route.trainId);
    }

    orchestrator::OrchestratorConfig cfg;
    cfg.printHmi = false; // We render on-demand in the dashboard

    orchestrator_ = std::make_unique<orchestrator::ThreadOrchestrator>(
        network_, trainManager_, commChannel_, trainIds, cfg);

    for (const auto& r : currentRoutes_)
    {
        orchestrator_->setTrainRoute(r.trainId, r.currentTrackId, r.route);
    }

    orchestrator_->setSafetyStep(pipeline_->makeStep());
    orchestrator_->start();

    std::cout << "[OK] Simulation started. Real-time safety pipeline active.\n";
}

void TcasApplication::pauseSimulation()
{
    if (orchestrator_ && orchestrator_->isRunning())
    {
        orchestrator_->pause();
        std::cout << "[OK] Simulation PAUSED. Physics and safety paused; dashboard remains active.\n";
    }
    else
    {
        std::cout << "[INFO] Simulation is not running.\n";
    }
}

void TcasApplication::resumeSimulation()
{
    if (orchestrator_ && orchestrator_->isPaused())
    {
        orchestrator_->resume();
        std::cout << "[OK] Simulation RESUMED.\n";
    }
    else if (orchestrator_ && orchestrator_->isRunning())
    {
        std::cout << "[INFO] Simulation is already running.\n";
    }
    else
    {
        startSimulation();
    }
}

void TcasApplication::resetSimulation()
{
    if (orchestrator_)
    {
        orchestrator_->stop();
        orchestrator_.reset();
    }
    pipeline_.reset();
    currentRoutes_.clear();
    std::cout << "[OK] Simulation reset complete. Load a scenario or [1] Start.\n";
}

void TcasApplication::addTrainInteractive()
{
    std::cout << "\nTrain type? [1=Express  2=Passenger  3=Freight]: ";
    const int typeChoice = readInt();
    const auto id = readTrainId("Train ID (unique integer): ");

    std::unique_ptr<train::Train> train;
    switch (typeChoice)
    {
    case 1:
        train = std::make_unique<train::ExpressTrain>(id, 45000.0, 45.0, 0.9, 1.4);
        break;
    case 2:
        train = std::make_unique<train::PassengerTrain>(id, 60000.0, 33.3, 0.8, 1.2);
        break;
    case 3:
    default:
        train = std::make_unique<train::FreightTrain>(id, 120000.0, 22.2, 0.5, 0.8);
        break;
    }

    const double pos = readDouble("Initial position (m): ");
    const double vel = readDouble("Initial velocity (m/s): ");
    train->setPosition(pos);
    train->setVelocity(vel);

    std::cout << "Route Source Node ID: ";
    const int srcNode = readInt();
    std::cout << "Route Destination Node ID: ";
    const int dstNode = readInt();

    const auto route = navigation::RouteNavigator::findRoute(
        network_, static_cast<NodeId>(srcNode), static_cast<NodeId>(dstNode));

    if (!route.success || route.tracks.empty())
    {
        std::cout << "[WARN] No route found between Node " << srcNode << " and Node " << dstNode << ".\n";
    }

    if (trainManager_.addTrain(std::move(train)))
    {
        std::cout << "[OK] Train #" << id << " added to fleet registry.\n";
        if (route.success && !route.tracks.empty())
        {
            orchestrator::SafetyPipeline::TrainRoute tr{
                id,
                route.tracks.front(),
                route
            };
            currentRoutes_.push_back(tr);
            if (pipeline_)
            {
                pipeline_->addOrUpdateRoute(tr);
            }
            if (orchestrator_)
            {
                orchestrator_->setTrainRoute(id, route.tracks.front(), route);
            }
            std::cout << "[OK] Route assigned (" << route.totalDistance << " m).\n";
        }

        if (orchestrator_)
        {
            orchestrator_->postCommand({orchestrator::UserCommandType::AddTrain, id});
        }
        std::cout << "[OK] Train #" << id << " registered in simulation.\n";
    }
    else
    {
        std::cout << "[ERR] Train #" << id << " already exists.\n";
    }
}

void TcasApplication::removeTrainInteractive()
{
    const auto id = readTrainId("Train ID to remove: ");
    if (trainManager_.removeTrain(id))
    {
        if (orchestrator_)
        {
            orchestrator_->postCommand({orchestrator::UserCommandType::RemoveTrain, id});
        }
        if (pipeline_)
        {
            pipeline_->removeRoute(id);
        }
        std::erase_if(currentRoutes_, [id](const auto& r) { return r.trainId == id; });
        std::cout << "[OK] Train #" << id << " removed from simulation and safety pipeline.\n";
    }
    else
    {
        std::cout << "[ERR] Train #" << id << " not found.\n";
    }
}

void TcasApplication::changeSpeedInteractive()
{
    const auto id = readTrainId("Train ID: ");
    const double vel = readDouble("New velocity (m/s): ");

    if (orchestrator_)
    {
        // Thread-safe dispatch via UserCommandQueue
        orchestrator_->postCommand({orchestrator::UserCommandType::SetSpeed, id, vel});
        std::cout << "[OK] Speed change command posted for Train #" << id << " -> " << vel << " m/s.\n";
    }
    else
    {
        auto* train = trainManager_.getTrain(id);
        if (train)
        {
            train->setVelocity(vel);
            std::cout << "[OK] Velocity updated.\n";
        }
        else
        {
            std::cout << "[ERR] Train #" << id << " not found.\n";
        }
    }
}

void TcasApplication::changeRouteInteractive()
{
    const auto id = readTrainId("Train ID to change route: ");
    auto* train = trainManager_.getTrain(id);
    if (train == nullptr)
    {
        std::cout << "[ERR] Train #" << id << " not found.\n";
        return;
    }

    std::cout << "New Source Node ID: ";
    const int srcNode = readInt();
    std::cout << "New Destination Node ID: ";
    const int dstNode = readInt();

    const auto newRoute = navigation::RouteNavigator::findRoute(
        network_, static_cast<NodeId>(srcNode), static_cast<NodeId>(dstNode));

    if (!newRoute.success || newRoute.tracks.empty())
    {
        std::cout << "[ERR] No path found between Node " << srcNode << " and Node " << dstNode << ".\n";
        return;
    }

    train->setPosition(0.0);

    orchestrator::SafetyPipeline::TrainRoute tr{
        id,
        newRoute.tracks.front(),
        newRoute
    };

    bool found = false;
    for (auto& r : currentRoutes_)
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
        currentRoutes_.push_back(tr);
    }

    if (pipeline_)
    {
        pipeline_->addOrUpdateRoute(tr);
    }

    if (orchestrator_)
    {
        orchestrator::UserRouteSpec spec{ id, newRoute.tracks.front(), newRoute };
        orchestrator_->postCommand({
            orchestrator::UserCommandType::ChangeRoute,
            id,
            0.0,
            "",
            spec
        });
    }

    std::cout << "[OK] Route updated for Train #" << id << " (" << newRoute.totalDistance << " m).\n";
}

void TcasApplication::holdTrainInteractive()
{
    const auto id = readTrainId("Train ID to hold: ");
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::HoldTrain, id});
        std::cout << "[OK] Hold command posted for Train #" << id << ".\n";
    }
    else
    {
        auto* t = trainManager_.getTrain(id);
        if (t) { t->setVelocity(0.0); t->setState(TrainState::Stopped); }
    }
}

void TcasApplication::resumeTrainInteractive()
{
    const auto id = readTrainId("Train ID to resume: ");
    const double vel = readDouble("Resume velocity (m/s): ");
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::ResumeTrain, id, vel});
        std::cout << "[OK] Resume command posted for Train #" << id << " -> " << vel << " m/s.\n";
    }
    else
    {
        auto* t = trainManager_.getTrain(id);
        if (t) { t->setVelocity(vel); t->setState(TrainState::Running); }
    }
}

void TcasApplication::injectSensorFaultInteractive()
{
    std::cout << "Train ID to fail sensor (0 for all trains): ";
    const auto id = static_cast<TrainId>(readInt());
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::InjectSensorFailure, id});
    }
    std::cout << "[OK] Sensor fault injected for " << (id == 0 ? "all trains" : ("Train #" + std::to_string(id))) << ".\n";
}

void TcasApplication::recoverSensorInteractive()
{
    std::cout << "Train ID to recover sensor (0 for all trains): ";
    const auto id = static_cast<TrainId>(readInt());
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::RecoverSensor, id});
    }
    std::cout << "[OK] Sensor recovered for " << (id == 0 ? "all trains" : ("Train #" + std::to_string(id))) << ".\n";
}

void TcasApplication::injectCommFaultInteractive()
{
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::InjectCommFailure});
    }
    std::cout << "[OK] Communication failure injected.\n";
}

void TcasApplication::recoverCommInteractive()
{
    if (orchestrator_)
    {
        orchestrator_->postCommand({orchestrator::UserCommandType::RecoverComm});
    }
    std::cout << "[OK] Communication failure cleared.\n";
}

void TcasApplication::setCommQualityInteractive()
{
    std::cout << "\nCommunication quality:\n"
              << "  [1] Normal (clean)\n"
              << "  [2] Degraded (30% drop rate)\n"
              << "  [3] Failed (70% drop rate)\n"
              << "  [4] Recover\n"
              << "Choice: ";
    const int c = readInt();
    if (c == 3)
    {
        injectCommFaultInteractive();
    }
    else if (c == 4 || c == 1)
    {
        recoverCommInteractive();
    }
    else
    {
        std::cout << "[OK] Setting applied.\n";
    }
}

void TcasApplication::showTelemetry()
{
    if (!orchestrator_)
    {
        std::cout << "[INFO] Simulation not active.\n";
        return;
    }
    const auto st = orchestrator_->snapshot();
    std::cout << "\n[TELEMETRY STREAM]\n"
              << "  Simulation Time : " << st.simulationTime << " s\n"
              << "  System Status   : " << static_cast<int>(st.systemStatus) << "\n"
              << "  Trains Active   : " << st.trains.size() << "\n"
              << "  Trajectory Pts  : " << st.predictions.size() << "\n"
              << "  Active Conflicts: " << st.activeConflicts.size() << "\n"
              << "  Reservations    : " << st.reservations.size() << "\n"
              << "  Commands        : " << st.commands.size() << "\n"
              << "  Sensor Link     : " << (st.sensorFailure ? "DEGRADED" : "OK") << "\n"
              << "  Comm Link       : " << (st.communicationFailure ? "DEGRADED" : "OK") << "\n";
}

void TcasApplication::showPerformance()
{
    if (!orchestrator_)
    {
        std::cout << "[INFO] Simulation not active.\n";
        return;
    }
    const auto m = perfMetrics_.snapshot();
    std::cout << "\n[SAFETY PERFORMANCE METRICS]\n"
              << "  Collisions observed   : " << m.collisionCount << "\n"
              << "  Near-misses observed  : " << m.nearMissCount << "\n"
              << "  Emergency brake ops   : " << m.emergencyBrakeCount << "\n"
              << "  Conflict observations : " << m.conflictObservations << "\n"
              << "  Minimum separation    : " << std::fixed << std::setprecision(2) << m.minimumSeparation << " m\n"
              << "  Minimum TTC           : " << m.minimumTtc << " s\n"
              << "  Max HMI loop latency  : " << m.maximumHmiLatencyMs << " ms\n";
}

void TcasApplication::showTrainStatus()
{
    if (!orchestrator_)
    {
        std::cout << "[INFO] Simulation not active.\n";
        return;
    }
    const auto st = orchestrator_->snapshot();
    std::cout << "\n[FLEET STATUS  t = " << std::fixed << std::setprecision(2) << st.simulationTime << " s]\n"
              << "ID       TYPE       TRACK       POSITION    SPEED     STATE      SENSORS\n";
    for (const auto& t : st.trains)
    {
        const std::string trackStr = (t.trackId != 0) ? ("T" + std::to_string(t.trackId)) : "-";
        std::cout << std::setw(8) << t.id << ' '
                  << std::setw(10) << (t.type == TrainType::Express ? "Express" : (t.type == TrainType::Passenger ? "Passenger" : "Freight")) << ' '
                  << std::setw(10) << trackStr << ' '
                  << std::setw(10) << t.position << " m  "
                  << std::setw(8) << t.velocity << " m/s  "
                  << std::setw(10) << (t.state == TrainState::Running ? "RUNNING" : (t.state == TrainState::Braking ? "BRAKING" : (t.state == TrainState::EmergencyBrake ? "EMERGENCY" : "STOPPED"))) << ' '
                  << (t.sensorFailure ? "FAULT" : "OK") << '\n';
    }
}

void TcasApplication::showSystemInfo()
{
    std::cout << "\n[SYSTEM DIAGNOSTICS]\n";
    if (orchestrator_)
    {
        std::cout << "  Physics cycles  : " << orchestrator_->physicsCycles() << "\n"
                  << "  Safety cycles   : " << orchestrator_->safetyCycles() << "\n"
                  << "  Comm cycles     : " << orchestrator_->communicationCycles() << "\n"
                  << "  HMI cycles      : " << orchestrator_->hmiCycles() << "\n"
                  << "  Running state   : " << (orchestrator_->isRunning() ? "ACTIVE" : "STOPPED") << "\n"
                  << "  Paused state    : " << (orchestrator_->isPaused() ? "PAUSED" : "RUNNING") << "\n";
    }
    else
    {
        std::cout << "  Orchestrator not initialized.\n";
    }
}

void TcasApplication::loadScenario(scenario::ScenarioType type)
{
    if (orchestrator_)
    {
        orchestrator_->stop();
        orchestrator_.reset();
    }

    scenario::ScenarioManager mgr(network_, trainManager_);
    const auto result = mgr.load(type);

    currentRoutes_ = result.routes;

    std::cout << "\n[SCENARIO LOADED] " << scenario::ScenarioManager::scenarioName(type) << "\n"
              << "--------------------------------------------------------------\n"
              << result.description << "\n"
              << "Trains in fleet: " << trainManager_.trainCount() << "\n";
}

} // namespace tcas::app
