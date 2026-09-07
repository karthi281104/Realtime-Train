#include "hmi/TelemetryLogger.hpp"

#include <filesystem>

namespace tcas::hmi
{

TelemetryLogger::TelemetryLogger(std::filesystem::path directory)
{
    std::filesystem::create_directories(directory);
    telemetryFile_.open(directory / "train_telemetry.csv", std::ios::app);
    conflictFile_.open(directory / "conflict_reports.log", std::ios::app);

    if (telemetryFile_.tellp() == std::streampos(0))
    {
        telemetryFile_ << "timestamp,train_id,train_type,track_id,position,velocity,acceleration,state,sensor_state,communication_state,command\n";
    }
}

TelemetryLogger::~TelemetryLogger()
{
    std::lock_guard lock(mutex_);
    telemetryFile_.flush();
    conflictFile_.flush();
}

void TelemetryLogger::logSnapshot(const orchestrator::WorldState& state)
{
    std::lock_guard lock(mutex_);
    for (const auto& train : state.trains)
    {
        telemetryFile_ << state.simulationTime << ','
                       << train.id << ','
                       << static_cast<int>(train.type) << ','
                       << train.trackId << ','
                       << train.position << ','
                       << train.velocity << ','
                       << train.acceleration << ','
                       << static_cast<int>(train.state) << ','
                       << (state.sensorFailure ? "FAILED" : "OK") << ','
                       << (state.communicationFailure ? "FAILED" : "OK") << ','
                       << (state.commands.empty() ? "NONE" : "ACTIVE") << '\n';
    }
}

void TelemetryLogger::logConflicts(const orchestrator::WorldState& state)
{
    std::lock_guard lock(mutex_);
    for (const auto& conflict : state.activeConflicts)
    {
        conflictFile_ << "timestamp=" << state.simulationTime
                      << "\ntrain_a=" << conflict.trainA
                      << "\ntrain_b=" << conflict.trainB
                      << "\nconflict_type=" << static_cast<int>(conflict.type)
                      << "\nlocation=" << conflict.resourceNodeId
                      << "\nttc=" << conflict.firstConflictTime
                      << "\nminimum_separation=" << conflict.minimumSeparation
                      << "\n\n";
    }
}

} // namespace tcas::hmi
