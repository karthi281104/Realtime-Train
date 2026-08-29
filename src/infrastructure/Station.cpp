#include "infrastructure/Station.hpp"

#include <utility>

namespace tcas::infrastructure
{

Station::Station(
    NodeId nodeId,
    std::string name
)
    : nodeId_(nodeId),
      name_(std::move(name))
{
}

NodeId Station::nodeId() const noexcept
{
    return nodeId_;
}

const std::string& Station::name() const noexcept
{
    return name_;
}

} // namespace tcas::infrastructure