# Module 9 — Conflict Detection & Resource Reservation

## 1. Architectural Role

Module 9 is the predictive conflict detection and spatial-temporal resource coordination engine of TCAS.

It receives predicted future trajectories from Module 8 and cross-evaluates them across the railway topology graph from Module 1. It identifies impending spatial conflicts well before physical proximity violations occur, allowing the system to schedule reservations for shared critical infrastructure (junctions, switches, and single-line tracks).

```text
       Future Trajectories (Module 8) + Network Graph (Module 1)
                                   │
                                   ▼
                         ConflictDetector
           ├── Same-Track Separation & Direction Analysis
           ├── Temporal Interval Overlap Evaluation
           ├── Junction Convergence Time Window Check
           └── Platform Headway Clearance Validation
                                   │
                                   ▼
                     Detected Conflicts & Zones
                                   │
                                   ▼
                   ResourceReservationManager
           └── Atomic Interval Locking for Shared Nodes
```

---

## 2. Conflict Classifications

| Conflict Type | Physical Scenario | Detection Criteria |
|---|---|---|
| **`RearEnd`** | Leading train overtaken by trailing train on the same track | Identical track, same direction of travel, spatial separation $< d_{\text{min}}$ within lookahead interval |
| **`HeadOn`** | Opposing trains travelling toward each other on the same track segment | Identical track, opposing velocity vectors ($v_1 \cdot v_2 < 0$) |
| **`Junction`** | Trains approaching an intersecting junction switch from different converging tracks | Overlapping estimated occupancy windows $[t_{\text{in}}, t_{\text{out}}]$ at the switch node |
| **`Platform`** | Simultaneous scheduled arrival at the same station platform | Platform dwell time interval conflict with insufficient clearance buffer |

---

## 3. Public API

### `tcas::conflict::ConflictDetector`

| Method | Description |
|---|---|
| `detect(trainA, trajA, trainB, trajB, network)` | Evaluates pairwise trajectories and returns all spatial and junction conflicts. |
| `classifySameTrack(stateA, stateB)` | Determines whether a co-linear track conflict is `HeadOn` or `RearEnd`. |

### `tcas::conflict::ResourceReservationManager`

| Method | Description |
|---|---|
| `request(trainId, zone, startTime, endTime)` | Attempts an exclusive atomic reservation for a `ConflictZone` over a time interval. |
| `release(trainId, zone)` | Manually frees an active reservation when a train clears a zone. |
| `isReserved(zone, time)` | Checks if a given physical resource is occupied or locked at timestamp $t$. |
| `clear()` / `clearReleased()` | Flushes expired or cancelled reservations from the active allocation table. |

### Data Structures

```cpp
struct Conflict
{
    TrainId trainA{ 0 };
    TrainId trainB{ 0 };
    ConflictType type{ ConflictType::RearEnd };
    TrackId trackId{ 0 };
    NodeId resourceNodeId{ 0 };
    TimeSeconds firstConflictTime{ 0.0 };
    TimeSeconds lastConflictTime{ 0.0 };
    DistanceMeters minimumSeparation{ 0.0 };
};
```

---

## 4. Verification & Testing

- **`tests/conflict/ConflictDetectorTest.cpp`**:
  - `DetectsSameTrackRearEndConflict`
  - `DetectsHeadOnConflict`
  - `DetectsJunctionConvergenceConflict`
  - `IgnoresSafeSeparationIntervals`
  - `GrantsExclusiveReservation`
  - `RejectsConflictingIntervalReservation`
  - `FreesResourceUponRelease`
