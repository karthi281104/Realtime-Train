#include "hmi/TelemetryLogger.hpp"

#include <filesystem>

namespace tcas::hmi
{

TelemetryLogger::TelemetryLogger(std::filesystem::path directory)
{
    std::filesystem::create_directories(directory);
    telemetryFile_.open(directory / "train_telemetry.csv", std::ios::app);
    conflictFile_.open(directory / "conflict_reports.log", std::ios::app);
    eventsFile_.open(directory / "events.log", std::ios::app);

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
    eventsFile_.flush();
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
                       << (train.sensorFailure ? "FAILED" : "OK") << ','
                       << (state.communicationFailure ? "FAILED" : "OK") << ','
                       << (state.commands.empty() ? "NONE" : "ACTIVE") << '\n';
    }

    // Log discrete safety events to events.log
    for (const auto& cmd : state.commands)
    {
        if (cmd.type != safety::SafetyCommandType::NoAction)
        {
            eventsFile_ << '[' << state.simulationTime << "s] SAFETY_COMMAND: Train #"
                        << cmd.trainId << " command=" << static_cast<int>(cmd.type)
                        << " targetSpeed=" << cmd.targetSpeed << '\n';
        }
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

        eventsFile_ << '[' << state.simulationTime << "s] CONFLICT_DETECTED: Trains #"
                    << conflict.trainA << " <-> #" << conflict.trainB
                    << " at node " << conflict.resourceNodeId
                    << " TTC=" << conflict.firstConflictTime << "s\n";
    }
}

void TelemetryLogger::logEvent(double timestamp, const std::string& eventType, const std::string& details)
{
    std::lock_guard lock(mutex_);
    if (eventsFile_.is_open())
    {
        eventsFile_ << '[' << timestamp << "s] " << eventType << ": " << details << '\n';
        eventsFile_.flush();
    }
}

} // namespace tcas::hmi
