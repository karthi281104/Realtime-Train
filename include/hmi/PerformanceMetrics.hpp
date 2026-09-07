#pragma once

#include "orchestrator/WorldState.hpp"

#include <chrono>
#include <cstddef>
#include <mutex>

namespace tcas::hmi
{

struct PerformanceSnapshot
{
    std::size_t collisionCount{ 0 };
    std::size_t nearMissCount{ 0 };
    std::size_t emergencyBrakeCount{ 0 };
    std::size_t conflictObservations{ 0 };
    double minimumSeparation{ 0.0 };
    double minimumTtc{ 0.0 };
    double maximumHmiLatencyMs{ 0.0 };
};

class PerformanceMetrics
{
public:
    void observe(
        const orchestrator::WorldState& state,
        std::chrono::steady_clock::duration hmiDuration);

    [[nodiscard]]
    PerformanceSnapshot snapshot() const;

private:
    mutable std::mutex mutex_;
    PerformanceSnapshot metrics_;
};

} // namespace tcas::hmi
