#pragma once

#include "communication/CommunicationChannel.hpp"
#include "hmi/PerformanceMetrics.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "orchestrator/SafetyPipeline.hpp"
#include "orchestrator/ThreadOrchestrator.hpp"
#include "scenario/ScenarioManager.hpp"
#include "train/TrainManager.hpp"

#include <memory>
#include <vector>

namespace tcas::app
{

class TcasApplication
{
public:
    TcasApplication();
    ~TcasApplication();

    TcasApplication(const TcasApplication&) = delete;
    TcasApplication& operator=(const TcasApplication&) = delete;

    int run();

private:
    void printHeader() const;
    void printDashboard();
    void printMenu() const;
    void handleCommand(int cmd);

    void startSimulation();
    void pauseSimulation();
    void resumeSimulation();
    void resetSimulation();

    void addTrainInteractive();
    void removeTrainInteractive();
    void changeSpeedInteractive();
    void changeRouteInteractive();
    void holdTrainInteractive();
    void resumeTrainInteractive();

    void injectSensorFaultInteractive();
    void recoverSensorInteractive();
    void injectCommFaultInteractive();
    void recoverCommInteractive();
    void setCommQualityInteractive();

    void showTelemetry();
    void showPerformance();
    void showTrainStatus();
    void showSystemInfo();

    void loadScenario(scenario::ScenarioType type);

    infrastructure::RailwayNetwork network_;
    train::TrainManager trainManager_;
    communication::CommunicationChannel commChannel_;
    std::unique_ptr<orchestrator::SafetyPipeline> pipeline_;
    std::unique_ptr<orchestrator::ThreadOrchestrator> orchestrator_;
    hmi::PerformanceMetrics perfMetrics_;
    std::vector<orchestrator::SafetyPipeline::TrainRoute> currentRoutes_;
    bool shutdown_{ false };
};

} // namespace tcas::app
