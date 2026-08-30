# Module 5 — Route & Navigation

## 1. Architectural Role

Module 5 is the route planning and graph navigation engine of TCAS.

It executes Dijkstra’s shortest-path algorithm over the directed `RailwayNetwork` graph, returning an ordered sequence of `TrackId`s and the total path distance in metres.

```text
RailwayNetwork (Graph) + Origin Node + Destination Node
                         │
                         ▼
                  RouteNavigator (Dijkstra)
                         │
                         ▼
                    RouteResult
           ├── success (bool)
           ├── tracks (std::vector<TrackId>)
           ├── totalDistance (DistanceMeters)
           └── reason (FailReason)
```

---

## 2. Public API

### `tcas::navigation::RouteNavigator`

| Method | Description |
|---|---|
| `static RouteResult findRoute(network, origin, destination)` | Finds shortest route using Dijkstra min-heap |

### `tcas::navigation::RouteResult`

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | True if path exists from origin to destination |
| `tracks` | `std::vector<TrackId>` | Ordered sequence of tracks along route |
| `totalDistance` | `DistanceMeters` | Total length of route in metres |
| `reason` | `FailReason` | `None`, `SameNode`, `NodeNotFound`, or `NoPathExists` |
