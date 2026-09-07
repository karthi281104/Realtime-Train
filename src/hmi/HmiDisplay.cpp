#include "hmi/HmiDisplay.hpp"

#include "conflict/ConflictZone.hpp"

#include <iomanip>
#include <ostream>
#include <sstream>

namespace tcas::hmi
{
namespace
{

const char* trainTypeName(const TrainType type)
{
    switch (type)
    {
    case TrainType::Express: return "Express";
    case TrainType::Passenger: return "Passenger";
    case TrainType::Freight: return "Freight";
    }
    return "Unknown";
}

const char* trainStateName(const TrainState state)
{
    switch (state)
    {
    case TrainState::Idle: return "IDLE";
    case TrainState::Running: return "RUNNING";
    case TrainState::Braking: return "BRAKING";
    case TrainState::Stopped: return "STOPPED";
    case TrainState::EmergencyBrake: return "EMERGENCY";
    }
    return "UNKNOWN";
}

const char* conflictTypeName(const conflict::ConflictType type)
{
    switch (type)
    {
    case conflict::ConflictType::RearEnd: return "REAR-END";
    case conflict::ConflictType::HeadOn: return "HEAD-ON";
    case conflict::ConflictType::Junction: return "JUNCTION";
    case conflict::ConflictType::Platform: return "PLATFORM";
    }
    return "UNKNOWN";
}

const char* commandName(const safety::SafetyCommandType type)
{
    switch (type)
    {
    case safety::SafetyCommandType::NoAction: return "NO ACTION";
    case safety::SafetyCommandType::ReduceSpeed: return "REDUCE SPEED";
    case safety::SafetyCommandType::HoldAtSignal: return "HOLD AT SIGNAL";
    case safety::SafetyCommandType::EmergencyBrake: return "EMERGENCY BRAKE";
    }
    return "UNKNOWN";
}

} // namespace

std::string HmiDisplay::format(const orchestrator::WorldState& state)
{
    std::ostringstream output;
    output << std::fixed << std::setprecision(2);
    output << "==============================================================\n";
    output << "                    TCAS CONTROL CENTER\n";
    output << "==============================================================\n\n";
    output << "SYSTEM STATUS : RUNNING\n";
    output << "SIMULATION    : " << state.simulationTime << " s\n";
    output << "COMMUNICATION : " << (state.communicationFailure ? "FAILED" : "OK") << '\n';
    output << "SENSORS       : " << (state.sensorFailure ? "FAILED" : "OK") << "\n\n";

    output << "--------------------------------------------------------------\n";
    output << "TRAIN STATUS\n";
    output << "--------------------------------------------------------------\n";
    output << "ID       TYPE       TRACK       POSITION    SPEED     STATE\n";
    for (const auto& train : state.trains)
    {
        output << std::setw(8) << train.id << ' '
               << std::setw(10) << trainTypeName(train.type) << ' '
               << std::setw(10) << train.trackId << ' '
               << std::setw(10) << train.position << " m  "
               << std::setw(8) << train.velocity << " m/s  "
               << trainStateName(train.state) << '\n';
    }

    output << "\n--------------------------------------------------------------\n";
    output << "ACTIVE CONFLICTS\n";
    output << "--------------------------------------------------------------\n";
    if (state.activeConflicts.empty())
    {
        output << "None\n";
    }
    for (const auto& conflict : state.activeConflicts)
    {
        output << "Type: " << conflictTypeName(conflict.type)
               << "  Node: " << conflict.resourceNodeId
               << "  Trains: " << conflict.trainA << " <-> " << conflict.trainB
               << "  TTC: " << conflict.firstConflictTime << " s\n";
    }

    output << "\n--------------------------------------------------------------\n";
    output << "JUNCTION RESERVATIONS\n";
    output << "--------------------------------------------------------------\n";
    if (state.reservations.empty())
    {
        output << "None\n";
    }
    for (const auto& reservation : state.reservations)
    {
        output << "Node " << reservation.zone.nodeId
               << " : owner Train #" << reservation.trainId
               << "  state=" << static_cast<int>(reservation.state) << '\n';
    }

    output << "\n--------------------------------------------------------------\n";
    output << "COMMANDS\n";
    output << "--------------------------------------------------------------\n";
    if (state.commands.empty())
    {
        output << "None\n";
    }
    for (const auto& command : state.commands)
    {
        output << "Train #" << command.trainId << " : "
               << commandName(command.type)
               << " -> " << command.targetSpeed << " m/s\n";
    }

    output << "\nSYSTEM STATE : "
           << (state.activeConflicts.empty() ? "SAFE" : "CONFLICT ACTIVE")
           << "\n==============================================================\n";
    return output.str();
}

void HmiDisplay::render(
    const orchestrator::WorldState& state,
    std::ostream& output)
{
    output << format(state);
}

} // namespace tcas::hmi
