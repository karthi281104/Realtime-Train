# Module 2 — Design Document

## 1. Design Approach

Module 2 follows an object-oriented design based on abstraction, inheritance, polymorphism, encapsulation, and RAII.

The objective is to provide a common representation for all trains while allowing individual train categories to be identified and extended independently.

---

## 2. Class Architecture

```text
                         +----------------+
                         |     Train      |
                         |  <<abstract>>  |
                         +----------------+
                         | TrainId        |
                         | mass           |
                         | maxSpeed       |
                         | serviceBrake   |
                         | emergencyBrake |
                         | position       |
                         | velocity       |
                         | acceleration   |
                         | state          |
                         +----------------+
                                  |
              +-------------------+-------------------+
              |                   |                   |
              v                   v                   v
     +----------------+  +----------------+  +----------------+
     |  ExpressTrain  |  | PassengerTrain |  |  FreightTrain  |
     +----------------+  +----------------+  +----------------+
     | type()         |  | type()         |  | type()         |
     +----------------+  +----------------+  +----------------+
```

---

## 3. Train Abstraction

`Train` is an abstract base class.

The class contains common train properties and provides common accessors and state management.

The following method is polymorphic:

```text
virtual TrainType type() const noexcept = 0;
```

This allows the safety system to work with:

```text
Train*
unique_ptr<Train>
```

without knowing the concrete derived class.

---

## 4. Encapsulation

Train properties are private.

External modules interact with the train through controlled methods such as:

```text
id()
mass()
maximumSpeed()
serviceBraking()
emergencyBraking()
position()
velocity()
acceleration()
state()
```

State changes are performed through controlled setters.

This prevents uncontrolled modification of train data.

---

## 5. Validation Rules

The following constraints are enforced.

### Mass

```text
mass > 0
```

### Maximum speed

```text
maximumSpeed >= 0
```

### Service braking

```text
serviceBraking >= 0
```

### Emergency braking

```text
emergencyBraking >= 0
```

and:

```text
emergencyBraking >= serviceBraking
```

### Position

```text
position >= 0
```

### Velocity

```text
0 <= velocity <= maximumSpeed
```

These constraints protect the simulation from physically invalid states.

---

## 6. TrainManager Design

The manager maintains a mapping:

```text
TrainId -> unique_ptr<Train>
```

Conceptually:

```text
unordered_map<TrainId, unique_ptr<Train>>
```

This provides efficient train lookup by ID.

---

## 7. Train Lifecycle

```text
Create Train
     |
     v
Validate Parameters
     |
     v
TrainManager::addTrain()
     |
     v
Check ID
     |
     +---- duplicate ---> Reject
     |
     v
Store Train
     |
     v
Simulation
     |
     v
removeTrain()
     |
     v
Automatic destruction
```

---

## 8. Polymorphism

The manager does not need separate containers for:

```text
ExpressTrain
PassengerTrain
FreightTrain
```

Instead:

```text
unique_ptr<Train>
```

can store any derived train.

Example conceptual structure:

```text
TrainManager
    |
    +-- 1001 -> ExpressTrain
    +-- 1002 -> PassengerTrain
    +-- 1003 -> FreightTrain
```

This simplifies future safety-engine processing.

---

## 9. Future Extension

The design allows future train-specific behavior to be added without changing the basic manager architecture.

Potential future extensions include:

- Train-specific braking curves.
- Maximum acceleration.
- Maximum jerk.
- Train length.
- Number of coaches.
- Cargo mass.
- Passenger capacity.
- Sensor state.
- Communication state.
- Route information.

These should be added only when required by later modules.

---

## 10. Design Constraints

Module 2 deliberately does not implement:

- Physics calculations.
- Collision detection.
- Route planning.
- Prediction.
- Communication simulation.
- Sensor simulation.
- Conflict resolution.

Those responsibilities belong to later modules.

This separation keeps Module 2 focused on train representation and lifecycle management.