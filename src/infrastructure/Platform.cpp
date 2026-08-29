#include "infrastructure/Platform.hpp"

#include <utility>

namespace tcas::infrastructure
{

Platform::Platform(
    PlatformId id,
    NodeId stationNodeId,
    std::string name
)
    : id_(id),
      stationNodeId_(stationNodeId),
      name_(std::move(name))
{
}

PlatformId Platform::id() const noexcept
{
    return id_;
}

NodeId Platform::stationNodeId() const noexcept
{
    return stationNodeId_;
}

const std::string& Platform::name() const noexcept
{
    return name_;
}

} // namespace tcas::infrastructure