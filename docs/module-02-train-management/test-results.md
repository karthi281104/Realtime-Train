# Module 2 — Test Plan

## 1. Testing Objective

The objective of Module 2 testing is to verify that train entities are correctly represented, validated, classified, and managed.

GoogleTest is used as the unit testing framework.

---

## 2. Test Categories

### Train Base Class

Tests verify:

- Valid train creation.
- Initial state.
- Initial motion values.
- Mass validation.
- Maximum speed validation.
- Service braking validation.
- Emergency braking validation.
- Position updates.
- Velocity updates.
- Train state updates.

### Derived Train Classes

Tests verify:

- Express train classification.
- Passenger train classification.
- Freight train classification.
- Polymorphic access through the base class.

### TrainManager

Tests verify:

- Empty manager.
- Adding trains.
- Finding trains.
- Missing train lookup.
- Duplicate train rejection.
- Null train rejection.
- Multiple train types.
- Removing trains.
- Removing missing trains.
- Contains operation.
- Clearing all trains.

---

## 3. Test Results

### Train Tests

```text
TrainTest.CreatesValidTrain                              PASS
TrainTest.InitialStateIsIdle                            PASS
TrainTest.InitialMotionValuesAreZero                    PASS
TrainTest.RejectsZeroMass                               PASS
TrainTest.RejectsNegativeMass                           PASS
TrainTest.RejectsNegativeMaximumSpeed                   PASS
TrainTest.RejectsNegativeServiceBraking                 PASS
TrainTest.RejectsNegativeEmergencyBraking               PASS
TrainTest.RejectsEmergencyBrakingLowerThanServiceBraking PASS
TrainTest.UpdatesPosition                               PASS
TrainTest.RejectsNegativePosition                       PASS
TrainTest.UpdatesVelocity                               PASS
TrainTest.RejectsVelocityAboveMaximum                   PASS
TrainTest.UpdatesTrainState                             PASS
```

### Derived Train Tests

```text
ExpressTrainTest.HasExpressType                         PASS
ExpressTrainTest.SupportsPolymorphism                   PASS

PassengerTrainTest.HasPassengerType                     PASS
PassengerTrainTest.SupportsPolymorphism                 PASS

FreightTrainTest.HasFreightType                         PASS
FreightTrainTest.SupportsPolymorphism                   PASS
```

### TrainManager Tests

```text
TrainManagerTest.StartsEmpty                            PASS
TrainManagerTest.AddsTrain                              PASS
TrainManagerTest.FindsTrain                             PASS
TrainManagerTest.MissingTrainReturnsNull                PASS
TrainManagerTest.RejectsDuplicateTrainId                PASS
TrainManagerTest.RejectsNullTrain                       PASS
TrainManagerTest.SupportsMultipleTrainTypes             PASS
TrainManagerTest.RemovesTrain                           PASS
TrainManagerTest.RemovingMissingTrainReturnsFalse       PASS
TrainManagerTest.ContainsTrain                          PASS
TrainManagerTest.ClearRemovesAllTrains                  PASS
```

---

## 4. Regression Testing

Module 1 tests were executed after Module 2 integration.

Module 1:

```text
17/17 passed
```

Module 2:

```text
31/31 passed
```

Total:

```text
48/48 passed
```

This confirms that Module 2 did not break the existing infrastructure implementation.

---

## 5. Test Command

Configure:

```powershell
cmake -S . -B build
```

Build:

```powershell
cmake --build build --config Debug
```

Execute tests:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

---

## 6. Acceptance Criteria

Module 2 passes acceptance when:

- All train construction tests pass.
- All validation tests pass.
- All derived-class tests pass.
- Polymorphism tests pass.
- TrainManager lifecycle tests pass.
- No memory-management errors are observed.
- Module 1 regression tests remain successful.
- CMake build completes successfully.

Current result:

```text
PASS
48/48 tests passed
0 failures
```