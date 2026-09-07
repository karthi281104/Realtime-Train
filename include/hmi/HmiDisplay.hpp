#pragma once

#include "orchestrator/WorldState.hpp"

#include <iosfwd>
#include <string>

namespace tcas::hmi
{

class HmiDisplay
{
public:
    [[nodiscard]]
    static std::string format(const orchestrator::WorldState& state);

    static void render(
        const orchestrator::WorldState& state,
        std::ostream& output);
};

} // namespace tcas::hmi
