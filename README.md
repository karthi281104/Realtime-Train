# Train Collision Avoidance System (TCAS)

A C++23 real-time train collision-avoidance system, developed as twelve
incremental modules. Modules 1–5 are implemented and covered by unit and
cross-module integration tests.

## Implemented modules

1. Railway Infrastructure — directed nodes, tracks, and network topology
2. Train Management — train types and fleet lifecycle management
3. Simulation & Clock — deterministic simulation time and periodic timers
4. Physics & Braking — kinematics and gradient-aware stopping distances
5. Route & Navigation — Dijkstra shortest-path routing over the rail network

Module 6 (Sensor & State Estimation) is the next planned module.

## Layout

- `include/<domain>/`: public module interfaces
- `src/<domain>/`: implementation for the matching module
- `tests/<domain>/`: unit tests, plus `tests/integration/` for module seams
- `src/demo/` and `include/demo/`: executable demonstrations, separate from
  core-domain code
- `data/`: example network, train, and simulation configuration data
- `docs/module-*/`: module design and verification notes

## Build and test

Use CMake 3.20+ and a C++23-capable compiler:

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

`scripts/run_all_tests.ps1` performs the same configure, build, and test
sequence. GoogleTest is retrieved through CMake FetchContent with normal TLS
certificate verification enabled.
