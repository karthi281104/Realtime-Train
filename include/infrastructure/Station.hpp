#pragma once

#include "common/Types.hpp"

#include <string>

namespace tcas::infrastructure
{

class Station
{
public:
    Station(
        NodeId nodeId,
        std::string name
    );

    [[nodiscard]]
    NodeId nodeId() const noexcept;

    [[nodiscard]]
    const std::string& name() const noexcept;

private:
    NodeId nodeId_;
    std::string name_;
};

} // namespace tcas::infrastructure