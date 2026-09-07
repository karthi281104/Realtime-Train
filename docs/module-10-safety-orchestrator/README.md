# Module 10 — Safety, Risk, Resolution & Real-Time Orchestrator

## 1. Architectural Role

Module 10 forms the autonomous decision-making and real-time execution brain of TCAS.

It receives detected conflicts from Module 9, calculates mathematical risk levels, applies operational and passenger priority rules, generates intervention commands, and orchestrates the multi-threaded execution loops across the entire system.

```text
       Conflict Alerts (Module 9)
                   │
                   ▼
              RiskEngine
       (Quantitative Scoring & Classification: Low, Med, High, Critical)
                   │
                   ▼
             PriorityEngine
       (Express > Passenger > Freight Rule Validation)
                   │
                   ▼
            ResolutionEngine
       ├── NoAction
       ├── ReduceSpeed (Target Ceiling)
       ├── HoldAtSignal (Controlled Halt)
       └── EmergencyBrake (Maximum Retardation)
                   │
                   ▼
             CommandQueue
       (Thread-Safe Batch Dispatching)
                   │
                   ▼
          ThreadOrchestrator
       ├── Physics Loop        (50 Hz / 20 ms)
       ├── Safety Loop         (20 Hz / 50 ms)
       ├── Communication Loop  (20 Hz / 50 ms)
       └── HMI Telemetry Loop  ( 5 Hz / 200 ms)
```

---

## 2. Decision Logic & Engines

### 2.1 `RiskEngine`
Computes an aggregate safety risk score based on:
- **Time to Collision (TTC)**: Exponential penalty as $t \to 0$.
- **Closing Velocity**: Higher kinetic energy incurs higher severity.
- **Braking Distance vs. Safety Margin**: Checks if stopping distance exceeds available headway.
- **Sensor / Communication Confidence**: Degrades risk assessment safety margins under lossy channel conditions.

Classification tiers:
- **`Low`**: No immediate action needed.
- **`Medium`**: Monitor situation; advisory speed reduction.
- **`High`**: Command train to hold at next signal or enforce service deceleration.
- **`Critical`**: Immediate fail-safe emergency braking.

### 2.2 `PriorityEngine`
Determines which train must yield when both cannot proceed simultaneously:
1. **Safety First**: Physical feasibility always supersedes operational rank.
2. **Train Type Ranking**: `ExpressTrain` (3) > `PassengerTrain` (2) > `FreightTrain` (1).
3. **Headway Margin**: The train with the shorter stopping distance or lower payload is commanded to decelerate.

### 2.3 `ResolutionEngine`
Maps the risk assessment and priority judgment into a concrete `SafetyCommand`:
- `NoAction`: Safe separation maintained.
- `ReduceSpeed`: Commands target speed $v_{\text{target}} \le v_{\text{limit}}$.
- `HoldAtSignal`: Halts train at the boundary of a reserved switch or occupied track.
- `EmergencyBrake`: Engages maximum emergency deceleration.

---

## 3. Real-Time Concurrency Architecture

The `ThreadOrchestrator` manages four concurrent worker threads:

| Thread | Frequency | Period | Synchronization | Responsibilities |
|---|---|---|---|---|
| **Physics** | 50 Hz | 20 ms | Exclusive write lock on `worldMutex_` | Numerical integration of kinematics, position updates, command consumption |
| **Safety** | 20 Hz | 50 ms | Shared read lock on `worldMutex_` | Trajectory prediction, conflict detection, risk scoring, command generation |
| **Communication**| 20 Hz | 50 ms | Atomic channel step | Message transmission, packet latency handling, mailbox distribution |
| **HMI / Telemetry**| 5 Hz | 200 ms | Shared read lock | Status snapshots, dashboard display, performance cycle metrics |

### Concurrency Guarantees:
- **Batched Command Draining**: `CommandQueue::popAll()` drains pending commands in a single lock acquisition, eliminating mutex thrashing.
- **Non-blocking Read Snapshots**: `snapshot()` leverages `std::shared_lock<std::shared_mutex>`, permitting simultaneous inspection by HMI and Safety threads without blocking the 50 Hz physics integration.
- **Deterministic Shutdown**: Worker threads wake immediately upon shutdown signals via `std::condition_variable` and atomic flags, avoiding thread leaks or hangs.

---

## 4. Verification & Testing

- **`tests/safety/RiskEngineTest.cpp`**: TTC risk curve, mass scaling, sensor confidence degradation.
- **`tests/safety/PriorityEngineTest.cpp`**: Multi-class priority hierarchy and tie-breaking.
- **`tests/safety/ResolutionEngineTest.cpp`**: Emergency braking on critical risk, controlled speed reduction.
- **`tests/safety/SafetyIntegrationTest.cpp`**: End-to-end conflict $\to$ risk $\to$ command generation pipeline.
- **`tests/orchestrator/CommandQueueTest.cpp`**: Multi-producer thread safety and batch draining.
- **`tests/orchestrator/ThreadOrchestratorTest.cpp`**: Concurrent cycle verification, snapshot validity, and clean multi-thread termination.
