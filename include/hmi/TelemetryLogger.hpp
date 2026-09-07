#pragma once

#include "orchestrator/WorldState.hpp"

#include <filesystem>
#include <fstream>
#include <mutex>

namespace tcas::hmi
{

class TelemetryLogger
{
public:
    explicit TelemetryLogger(
        std::filesystem::path directory = "logs");
    ~TelemetryLogger();

    TelemetryLogger(const TelemetryLogger&) = delete;
    TelemetryLogger& operator=(const TelemetryLogger&) = delete;

    void logSnapshot(const orchestrator::WorldState& state);
    void logConflicts(const orchestrator::WorldState& state);
    void logEvent(double timestamp, const std::string& eventType, const std::string& details);

private:
    std::mutex mutex_;
    std::ofstream telemetryFile_;
    std::ofstream conflictFile_;
    std::ofstream eventsFile_;
};

} // namespace tcas::hmi
