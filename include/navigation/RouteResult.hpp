#pragma once

#include <vector>

#include "common/Types.hpp"

namespace tcas::navigation {

// Result returned by RouteNavigator::findRoute().
//
// When success == true:
//   - tracks contains the ordered sequence of TrackId values
//     representing the path from origin to destination.
//   - totalDistance is the sum of track lengths along the route (metres).
//
// When success == false:
//   - tracks is empty.
//   - totalDistance is 0.0.
//   - reason indicates why the route could not be found.
struct RouteResult {
  enum class FailReason {
    None,          // success == true
    SameNode,      // origin == destination
    NodeNotFound,  // origin or destination does not exist in network
    NoPathExists   // graph is disconnected or destination unreachable
  };

  bool success{false};
  std::vector<TrackId> tracks{};
  DistanceMeters totalDistance{0.0};
  FailReason reason{FailReason::None};
};

}  // namespace tcas::navigation
