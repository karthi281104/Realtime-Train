# TCAS Architecture

## Project

Real-Time Train Collision Avoidance System
with Predictive Conflict Resolution

## Technology

- C++23
- CMake
- GoogleTest

## Modules

1. Railway Infrastructure
2. Train Management
3. Simulation & Clock
4. Physics & Braking
5. Route & Navigation
6. Sensor & State Estimation
7. Communication Simulation
8. Predictive Position Engine
9. Conflict Detection, Conflict Zones & Resource Reservation
10. Risk, Priority & Predictive Resolution
11. Thread Orchestrator
12. HMI & Telemetry

## Development Strategy

Modules will be implemented sequentially.

Each module must have:

- Implementation
- Unit tests
- Edge-case tests
- Integration tests where applicable

A module is considered complete only when its GoogleTest suite passes.