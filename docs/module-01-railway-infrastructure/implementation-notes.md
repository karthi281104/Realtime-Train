# Module 1 — Implementation Notes

## 1. Implementation Language

The module is implemented in:

```text
C++23
```

---

## 2. Containers

The implementation uses:

```cpp
std::unordered_map
```

for identifier-based node and track storage.

Adjacency is stored using:

```cpp
std::unordered_map<NodeId, std::vector<NodeId>>
```

---

## 3. Const Correctness

Read-only lookup functions use `const`:

```cpp
const Node* getNode(NodeId id) const noexcept;
const Track* getTrack(TrackId id) const noexcept;
```

This prevents accidental modification during lookup.

---

## 4. Error Handling

Insertion operations return `bool`.

Example:

```cpp
if (network.addNode(node))
{
    // Successfully added
}
```

A `false` return indicates rejection.

---

## 5. Memory Model

`RailwayNetwork` stores infrastructure objects directly inside its containers.

Lookups return pointers to the stored objects.

No manual `new`/`delete` operations are required.

This avoids unnecessary manual memory management.

---

## 6. BFS Implementation

BFS is implemented using an explicit queue.

The visited set prevents repeated processing of nodes.

The algorithm counts visited nodes and compares this count with the total number of nodes.

---

## 7. DFS Implementation

DFS is implemented recursively.

Two states are maintained:

```text
visited
recursionStack
```

The recursion stack is cleared as recursion unwinds.

---

## 8. Validation

Track validation occurs before insertion.

The implementation verifies:

```text
Duplicate track
Missing source
Missing destination
Invalid length
Invalid speed limit
```

---

## 9. Current Scope

The current implementation intentionally avoids:

- Route optimization
- Train state
- Dynamic occupancy
- Train prediction
- Collision detection
- Safety commands
- Thread synchronization

Those will be introduced by subsequent modules.