#pragma once

#include "infrastructure/RailwayNetwork.hpp"
#include "navigation/RouteResult.hpp"

namespace tcas::navigation {

// Dijkstra-based route planner operating on a RailwayNetwork.
//
// Edge weight = track length in metres (shortest physical route).
// Time complexity: O((V + E) log V) using a binary min-heap.
//
// RouteNavigator is stateless — it holds no network reference.
// Pass the network into each findRoute() call.
//
// Thread safety: findRoute() is a const member function operating
// on const inputs — safe to call concurrently.
class RouteNavigator {
 public:
  RouteNavigator() = default;

  // Find the shortest route (by track length) from origin to destination
  // in the given railway network.
  //
  // Returns a RouteResult with success == true and the ordered TrackId
  // sequence when a path exists. Returns success == false with the
  // appropriate FailReason otherwise.
  [[nodiscard]]
  static RouteResult findRoute(const infrastructure::RailwayNetwork& network,
                               NodeId origin, NodeId destination);
};

}  // namespace tcas::navigation
