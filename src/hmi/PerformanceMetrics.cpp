#include "hmi/PerformanceMetrics.hpp"

#include <algorithm>
#include <limits>

namespace tcas::hmi
{

void PerformanceMetrics::observe(
    const orchestrator::WorldState& state,
    const std::chrono::steady_clock::duration hmiDuration)
{
    std::lock_guard lock(mutex_);
    metrics_.conflictObservations += state.activeConflicts.size();
    metrics_.emergencyBrakeCount += static_cast<std::size_t>(std::count_if(
        state.commands.begin(),
        state.commands.end(),
        [](const auto& command) { return command.isEmergency(); }));

    for (const auto& conflict : state.activeConflicts)
    {
        if (metrics_.minimumSeparation == 0.0)
        {
            metrics_.minimumSeparation = conflict.minimumSeparation;
        }
        else
        {
            metrics_.minimumSeparation = std::min(
                metrics_.minimumSeparation,
                conflict.minimumSeparation);
        }

        if (metrics_.minimumTtc == 0.0)
        {
            metrics_.minimumTtc = conflict.firstConflictTime;
        }
        else
        {
            metrics_.minimumTtc = std::min(
                metrics_.minimumTtc,
                conflict.firstConflictTime);
        }
    }

    const double durationMs =
        std::chrono::duration<double, std::milli>(hmiDuration).count();
    metrics_.maximumHmiLatencyMs = std::max(
        metrics_.maximumHmiLatencyMs,
        durationMs);
}

PerformanceSnapshot PerformanceMetrics::snapshot() const
{
    std::lock_guard lock(mutex_);
    return metrics_;
}

} // namespace tcas::hmi
