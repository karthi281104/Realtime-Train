#pragma once

#include "infrastructure/RailwayNetwork.hpp"
#include "orchestrator/SafetyPipeline.hpp"
#include "train/TrainManager.hpp"

#include <string>
#include <vector>

namespace tcas::scenario
{

enum class ScenarioType
{
    JunctionConflict,       // Express + Freight converging on Alpha Junction
    RearEndConflict,        // Two trains closing on the same track
    HeadOnConflict,         // Two trains approaching head-on
    PlatformConflict,       // Two trains arriving at the same platform node
    MultipleConflicts,      // Three trains, two simultaneous conflicts
    SensorFailure,          // Sensor degraded on a train
    CommunicationFailure,   // High packet-loss on comm channel
    UnsafeStopping          // Train cannot stop before conflict zone
};

struct ScenarioResult
{
    // Route table ready to pass to SafetyPipeline
    std::vector<orchestrator::SafetyPipeline::TrainRoute> routes;

    // Human-readable description printed to the terminal
    std::string description;

    // True if the scenario injects a sensor failure into WorldState
    bool injectSensorFailure{ false };

    // True if the scenario injects a communication failure into WorldState
    bool injectCommunicationFailure{ false };
};

// Builds and mutates an existing RailwayNetwork + TrainManager to create
// a specific conflict scenario for the interactive TCAS demo.
//
// Both network and trainManager must be valid and are expected to be the
// live objects owned by the interactive main demo. The ScenarioManager
// resets their content each time load() is called.
class ScenarioManager
{
public:
    ScenarioManager(
        infrastructure::RailwayNetwork& network,
        train::TrainManager& trainManager);

    // Clears the existing network and fleet, then sets up the requested
    // scenario. Returns the scenario result with routes and description.
    [[nodiscard]]
    ScenarioResult load(ScenarioType scenario);

    // Returns a human-readable name for a scenario type.
    [[nodiscard]]
    static const char* scenarioName(ScenarioType scenario) noexcept;

private:
    infrastructure::RailwayNetwork& network_;
    train::TrainManager& trainManager_;

    void buildBaseNetwork();

    ScenarioResult loadJunctionConflict();
    ScenarioResult loadRearEndConflict();
    ScenarioResult loadHeadOnConflict();
    ScenarioResult loadPlatformConflict();
    ScenarioResult loadMultipleConflicts();
    ScenarioResult loadSensorFailure();
    ScenarioResult loadCommunicationFailure();
    ScenarioResult loadUnsafeStopping();
};

} // namespace tcas::scenario
