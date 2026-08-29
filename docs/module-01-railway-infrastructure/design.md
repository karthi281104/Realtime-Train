# Module 1 — Design Specification

## 1. Architectural Role

Module 1 is the infrastructure layer of TCAS.

It provides a static representation of the railway topology upon which train routes, movement, prediction, and safety decisions will later operate.

```text
Railway Infrastructure
          │
          ▼
       Graph Model
          │
    ┌─────┴─────┐
    ▼           ▼
   BFS         DFS
Connectivity   Cycles
```

---

## 2. Object Model

```text
Node
├── id
├── name
└── type

Track
├── id
├── source
├── destination
├── length
├── speedLimit
└── gradient

Station
├── nodeId
└── name

Junction
└── id

Platform
├── id
├── stationNodeId
└── name

RailwayNetwork
├── nodes
├── tracks
└── adjacency
```

---

## 3. Graph Representation

The railway network is represented using an adjacency list.

Example:

```text
A → B
A → C
B → D
C → D
```

Adjacency list:

```text
A → B, C
B → D
C → D
D → []
```

This representation is efficient for railway networks because later route and conflict algorithms need to inspect outgoing track connections.

---

## 4. Directed Graph Decision

The graph is directed.

A track:

```text
A → B
```

does not automatically imply:

```text
B → A
```

This allows the model to represent directional railway operation accurately.

Bidirectional operation can later be represented using separate directed track relationships.

---

## 5. Data Ownership

`RailwayNetwork` owns the infrastructure graph.

The network stores nodes and tracks by identifier.

This provides centralized infrastructure lookup for later modules.

---

## 6. Validation Rules

### Node

```text
ID must be unique.
```

### Track

```text
Track ID must be unique.
Source node must exist.
Destination node must exist.
Length must be > 0.
Speed limit must be >= 0.
```

---

## 7. BFS Design

BFS uses:

```text
std::queue
std::unordered_set
```

Process:

```text
Select start node
       ↓
Add to queue
       ↓
Mark visited
       ↓
Remove node from queue
       ↓
Visit unvisited neighbours
       ↓
Repeat
```

The traversal terminates when the queue becomes empty.

---

## 8. DFS Design

DFS uses:

```text
visited
recursionStack
```

A node in the recursion stack represents a node currently being explored.

If DFS reaches another node already in that stack, a cycle exists.

---

## 9. Future Safety Integration

Junctions and platforms are represented now because later safety logic will require them.

For example:

```text
Train A ──► J1 ──► Platform P1
Train B ──► J1 ──► Platform P1
```

Module 1 provides:

```text
J1
P1
Track relationships
```

Later modules will calculate:

```text
Train ETA
Conflict
Priority
Reservation
Braking
```

---

## 10. Design Principle

Module 1 contains infrastructure knowledge only.

It should not contain train-specific safety decisions.

This separation prevents the infrastructure layer from becoming tightly coupled to the collision-avoidance engine.