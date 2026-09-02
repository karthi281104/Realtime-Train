#pragma once

#include <string>

#include "common/Types.hpp"

namespace tcas::infrastructure {

class Platform {
 public:
  Platform(PlatformId id, NodeId stationNodeId, std::string name);

  [[nodiscard]]
  PlatformId id() const noexcept;

  [[nodiscard]]
  NodeId stationNodeId() const noexcept;

  [[nodiscard]]
  const std::string& name() const noexcept;

 private:
  PlatformId id_;
  NodeId stationNodeId_;
  std::string name_;
};

}  // namespace tcas::infrastructure