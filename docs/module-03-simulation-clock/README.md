# Module 3 — Simulation & Clock

## 1. Module Overview

The Simulation & Clock module provides the **deterministic time backbone** of the TCAS system.

All TCAS modules that need to know the current time query the simulation clock rather than any real-time wall-clock source. This ensures the system is fully reproducible, testable, and deterministic.

---

## 2. Objectives

1. Provide a discrete-step simulation clock (`SimClock`) driven by explicit `tick()` calls.
2. Provide a configuration value type (`SimulationConfig`) holding all timing parameters.
3. Provide a periodic timer helper (`SimulationTimer`) to tell subsystems when to fire.
4. Ensure all time-related code is fully testable without threading or real-time dependencies.
5. Establish the time foundation required by Module 4 (Physics) and all later modules.

---

## 3. Module Responsibilities

Module 3 is responsible for:

- Simulation time tracking
- Tick-based clock advancement
- Timing parameter configuration
- Periodic subsystem fire scheduling

Module 3 is NOT responsible for:

- Real-time scheduling (Module 11 — Thread Orchestrator)
- Train physics (Module 4)
- Any I/O or sensor data (Module 6)

---

## 4. Classes

### 4.1 SimClock

Discrete-step deterministic simulation clock.

| Method | Description |
|---|---|
| `SimClock(TimeSeconds dt)` | Construct with given time step (default 0.020 s) |
| `void tick()` | Advance by one time step |
| `void reset()` | Reset to t=0, tick=0 |
| `TimeSeconds elapsed()` | Current simulation time in seconds |
| `SimTimeTick tickCount()` | Number of ticks since construction or reset |
| `TimeSeconds dt()` | Fixed time step |

### 4.2 SimulationConfig

Plain aggregate value type holding timing parameters.

| Field | Default | Description |
|---|---|---|
| `physicsPeriodMs` | 20 | Physics update interval (ms) |
| `safetyPeriodMs` | 100 | Safety check interval (ms) |
| `communicationPeriodMs` | 100 | Communication update interval (ms) |
| `hmiPeriodMs` | 200 | HMI telemetry interval (ms) |
| `predictionHorizonSeconds` | 60.0 | Prediction look-ahead window (s) |

### 4.3 SimulationTimer

Periodic fire helper for subsystem scheduling.

| Method | Description |
|---|---|
| `SimulationTimer(periodMs, physicsPeriodMs)` | Construct with period and physics tick size |
| `bool shouldFire(SimTimeTick)` | True when tick is a multiple of intervalTicks |
| `SimTimeTick intervalTicks()` | Computed interval in ticks |

---

## 5. Design Decisions

### No Wall-Clock Dependency

`SimClock` uses only `double` arithmetic internally. It has zero dependency on `std::chrono`, `time()`, or any system call. This is intentional — the system clock must be deterministic for testing.

### Tick-Driven Advancement

Time advances only when `tick()` is called explicitly. There is no background thread or automatic advancement. Module 11 will call `tick()` in its main loop.

### SimulationTimer Floor Division

If `periodMs` is not an exact multiple of `physicsPeriodMs`, the interval is rounded down. For example, 110 ms / 20 ms = 5.5 → 5 ticks (100 ms effective period). This is documented in the header.

---

## 6. Module Status

**STATUS: COMPLETE — BASELINE IMPLEMENTATION**
