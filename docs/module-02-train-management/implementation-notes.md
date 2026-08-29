# Module 2 — Implementation Notes

## 1. Implementation Overview

Module 2 implements the **Train Entity Manager** for the Real-Time Train Collision Avoidance System.

The implementation uses C++23 object-oriented programming principles to represent different train types through a common abstract base class.

The implementation consists of:

- Abstract `Train` base class.
- `ExpressTrain` derived class.
- `PassengerTrain` derived class.
- `FreightTrain` derived class.
- `TrainManager` for train lifecycle management.
- Common train-related types in `common/Types.hpp`.
- GoogleTest-based unit tests.

---

## 2. Implementation Location

### Header Files

```text
include/
└── train/
    ├── Train.hpp
    ├── ExpressTrain.hpp
    ├── PassengerTrain.hpp
    ├── FreightTrain.hpp
    └── TrainManager.hpp
```

### Source Files

```text
src/
└── train/
    ├── Train.cpp
    ├── ExpressTrain.cpp
    ├── PassengerTrain.cpp
    ├── FreightTrain.cpp
    └── TrainManager.cpp
```

### Test Files

```text
tests/
└── train/
    ├── TrainTest.cpp
    ├── ExpressTrainTest.cpp
    ├── PassengerTrainTest.cpp
    ├── FreightTrainTest.cpp
    └── TrainManagerTest.cpp
```

---

## 3. Common Type Implementation

Shared types are maintained in:

```text
include/common/Types.hpp
```

The file contains common identifiers and enumerations used across the system.

Examples:

```text
TrainId
TrainType
TrainState
DistanceMeters
SpeedMetersPerSecond
AccelerationMetersPerSecondSquared
```

The purpose of this arrangement is to prevent multiple modules from defining duplicate versions of the same domain type.

---

## 4. Train Base Class Implementation

The `Train` class is implemented as an abstract base class.

It contains the common properties required by all train types.

### Main properties

```text
TrainId
mass
maximumSpeed
serviceBraking
emergencyBraking
position
velocity
acceleration
state
```

The constructor initializes a newly created train in the `Idle` state.

Initial motion values are:

```text
position     = 0
velocity     = 0
acceleration = 0
state        = Idle
```

---

## 5. Train Type Polymorphism

The train type is exposed through a pure virtual function:

```text
virtual TrainType type() const noexcept = 0;
```

Each derived class implements this function.

Conceptually:

```text
ExpressTrain
    -> TrainType::Express

PassengerTrain
    -> TrainType::Passenger

FreightTrain
    -> TrainType::Freight
```

This allows the rest of the TCAS system to process trains through the common `Train` interface.

---

## 6. Express Train Implementation

`ExpressTrain` derives from `Train`.

Its primary responsibility at this stage is to identify itself as:

```text
TrainType::Express
```

The class uses the base class for common physical and operational properties.

Train-specific braking parameters are supplied during construction and can later be refined as the physics model becomes more detailed.

---

## 7. Passenger Train Implementation

`PassengerTrain` derives from `Train`.

It identifies itself as:

```text
TrainType::Passenger
```

Common train properties are maintained by the base class.

This provides a consistent interface for future physics and safety calculations.

---

## 8. Freight Train Implementation

`FreightTrain` derives from `Train`.

It identifies itself as:

```text
TrainType::Freight
```

Freight trains can have different physical characteristics, particularly mass and braking capability.

These properties are already represented in the base class so that future modules can use them without changing the basic architecture.

---

## 9. Parameter Validation

Validation is performed during train construction and state updates.

### Mass

A train must have:

```text
mass > 0
```

Invalid values result in an exception.

### Maximum Speed

The maximum speed cannot be negative.

```text
maximumSpeed >= 0
```

### Service Braking

Service braking capability cannot be negative.

```text
serviceBraking >= 0
```

### Emergency Braking

Emergency braking capability cannot be negative and must not be lower than service braking.

```text
emergencyBraking >= serviceBraking
```

### Position

Train position cannot be negative.

```text
position >= 0
```

### Velocity

Velocity must satisfy:

```text
0 <= velocity <= maximumSpeed
```

These checks prevent invalid physical states from entering the simulation.

---

## 10. Train State Implementation

The train operational state is represented using:

```text
TrainState
```

Supported states include:

```text
Idle
Running
Braking
Stopped
EmergencyBrake
```

The current implementation provides controlled state updates.

Future modules will use these states to represent the result of safety decisions.

For example:

```text
Conflict detected
       |
       v
Safety Engine
       |
       v
TrainState::Braking
```

or:

```text
Critical conflict
       |
       v
TrainState::EmergencyBrake
```

---

## 11. TrainManager Implementation

`TrainManager` provides centralized train lifecycle management.

Trains are stored using:

```text
std::unordered_map<TrainId, std::unique_ptr<Train>>
```

This provides:

- Unique ownership.
- Automatic destruction.
- Efficient lookup by ID.
- Polymorphic storage.
- No manual memory deallocation.

---

## 12. Adding a Train

The `addTrain()` operation performs validation before storing a train.

The logical flow is:

```text
addTrain()
    |
    v
Check null pointer
    |
    +---- null ----> reject
    |
    v
Get Train ID
    |
    v
Check existing ID
    |
    +---- duplicate ----> reject
    |
    v
Store unique_ptr
    |
    v
Success
```

Duplicate train IDs are rejected to maintain a unique identity for every train.

---

## 13. Finding a Train

The manager provides train lookup by `TrainId`.

Conceptually:

```text
getTrain(id)
      |
      v
Search unordered_map
      |
   +--+--+
   |     |
 found  missing
   |     |
   v     v
 Train   nullptr
```

Returning `nullptr` for a missing train avoids throwing an exception for normal lookup failure.

---

## 14. Removing a Train

The manager supports removal by train ID.

The operation returns whether the train was successfully removed.

Example:

```text
removeTrain(1001)
```

If the train exists:

```text
1001 -> removed
```

If it does not exist:

```text
1001 -> false
```

---

## 15. Memory Management

The implementation uses RAII through:

```text
std::unique_ptr
```

The `TrainManager` owns the train objects.

When a train is removed from the manager, its `unique_ptr` is destroyed and the train object is automatically released.

When the manager is cleared, all managed trains are automatically destroyed.

No explicit:

```text
delete
```

is required.

---

## 16. CMake Integration

The Module 2 source files were added to the `tcas_core` library.

The core library now contains:

```text
Infrastructure Sources
        +
Train Sources
        |
        v
    tcas_core
```

The test executable contains both Module 1 and Module 2 tests.

GoogleTest is integrated using CMake `FetchContent`.

---

## 17. Unit Testing Implementation

GoogleTest is used for all Module 2 unit tests.

Tests are separated according to responsibility:

```text
TrainTest
    |
    +-- Base train behavior
    +-- Validation
    +-- State
    +-- Motion properties

ExpressTrainTest
    |
    +-- Express classification
    +-- Polymorphism

PassengerTrainTest
    |
    +-- Passenger classification
    +-- Polymorphism

FreightTrainTest
    |
    +-- Freight classification
    +-- Polymorphism

TrainManagerTest
    |
    +-- Add
    +-- Find
    +-- Remove
    +-- Contains
    +-- Clear
    +-- Duplicate protection
```

---

## 18. Verification Procedure

The implementation is verified using three stages.

### Stage 1 — Configure

```powershell
cmake -S . -B build
```

### Stage 2 — Build

```powershell
cmake --build build --config Debug
```

### Stage 3 — Test

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

---

## 19. Final Implementation Result

The final implementation produced:

```text
Module 1 tests: 17/17 PASS
Module 2 tests: 31/31 PASS
--------------------------------
Total:          48/48 PASS
Failures:       0
```

Build artifacts successfully generated:

```text
tcas.exe
tcas_core.lib
tcas_tests.exe
```

Therefore, Module 2 is successfully implemented and integrated without regression to Module 1.

---

## 20. Scope Boundary

The following functionality is intentionally not implemented in Module 2.

### Not part of Module 2

- Train movement simulation.
- Kinematic calculations.
- Braking distance calculations.
- Collision detection.
- Time-to-conflict calculations.
- Route planning.
- Junction conflict resolution.
- Predictive position projection.
- Communication latency.
- Sensor failure simulation.
- Multi-threaded simulation.

These responsibilities will be implemented by subsequent modules.

---

## 21. Future Integration

Module 2 provides the train model required by the next stages of the TCAS system.

The expected dependency flow is:

```text
Module 1
Railway Infrastructure
        |
        v
Module 2
Train Entity Manager
        |
        v
Module 3
Simulation Clock
        |
        v
Module 4
Kinematics Engine
        |
        v
Module 5
Braking & Safe Separation
        |
        v
Predictive Safety Modules
```

The train's position, velocity, acceleration, mass, braking capability, type, and state will become inputs to the predictive collision-avoidance algorithms.

---

# Implementation Status

**Module 2 — Train Entity Manager: COMPLETE**

```text
Implementation:     COMPLETE
Unit Tests:         31/31 PASS
Regression Tests:   17/17 PASS
Total Tests:        48/48 PASS
Build:              PASS
Integration:        PASS
```