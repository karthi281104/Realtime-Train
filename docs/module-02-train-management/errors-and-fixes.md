# Module 2 — Errors and Fixes

## Issue 001 — Duplicate TrainType and TrainState Definitions

### Symptom

Compilation failed with:

```text
error C2872: 'TrainType': ambiguous symbol
```

The compiler reported two definitions:

```text
tcas::TrainType
tcas::train::TrainType
```

A similar conflict occurred with:

```text
TrainState
```

### Root Cause

`TrainType` and `TrainState` had been defined both in the common type layer and inside the train module.

This resulted in two different types with the same name.

### Resolution

The common domain types were centralized in:

```text
include/common/Types.hpp
```

The train module now includes the common type definitions instead of redefining them.

### Design Rule Added

Shared domain types must have a single authoritative definition.

Module-specific headers must reuse common types rather than redefine them.

---

# Issue 002 — Accidental Train Class Definition in Types.hpp

### Symptom

Compilation produced:

```text
error C2011:
'tcas::train::Train': class type redefinition
```

The compiler indicated that `Train` was declared in both:

```text
include/common/Types.hpp
include/train/Train.hpp
```

### Root Cause

During correction of the previous type conflict, the common types header was incorrectly modified to contain the complete `Train` class.

This caused the class to be defined twice.

### Resolution

The `Train` class was removed from:

```text
include/common/Types.hpp
```

The complete class definition remains exclusively in:

```text
include/train/Train.hpp
```

`Types.hpp` now contains only shared IDs, enums, and physical type aliases.

---

# Issue 003 — Missing Module 1 Common Types

### Symptom

After correcting the train class duplication, Module 1 compilation produced errors such as:

```text
DistanceMeters : identifier not found
JunctionId     : identifier not found
PlatformId     : identifier not found
```

### Root Cause

Existing Module 1 code depended on common types that had been accidentally removed while modifying `Types.hpp`.

### Resolution

The common type layer was restored to contain the infrastructure and train-related shared types.

Examples:

```text
NodeId
TrackId
StationId
JunctionId
PlatformId
TrainId
DistanceMeters
SpeedMetersPerSecond
AccelerationMetersPerSecondSquared
TrainType
TrainState
```

### Verification

After restoration:

```text
Module 1 tests: 17/17 PASS
Module 2 tests: 31/31 PASS
Total: 48/48 PASS
```

---

# Issue 004 — Regression Verification

### Problem

Adding a new module can unintentionally break previously completed modules.

### Verification Strategy

After fixing Module 2 integration, the complete test suite was executed rather than testing Module 2 alone.

Command:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

### Result

```text
48/48 tests passed
0 failures
```

This confirms that Module 2 integration did not introduce a currently detectable regression into Module 1.

---

# Lessons Learned

## 1. Common types must have one source of truth

Shared enums, identifiers, and value types belong in:

```text
include/common/Types.hpp
```

---

## 2. Module ownership must remain clear

```text
common/
    Shared types

infrastructure/
    Railway infrastructure classes

train/
    Train classes and train management
```

---

## 3. Fix the first compiler error first

Large compiler error lists can contain many cascading errors.

The recommended debugging procedure is:

```text
First meaningful compiler error
          |
          v
Identify root cause
          |
          v
Fix root cause
          |
          v
Clean rebuild
          |
          v
Run tests
          |
          v
Investigate remaining errors
```

---

## 4. Always run regression tests

A module is not considered complete simply because its own code compiles.

All previously completed module tests must continue to pass.

---

## Final Module 2 Status

```text
Build:             PASS
Module 1 tests:    17/17 PASS
Module 2 tests:    31/31 PASS
Total tests:       48/48 PASS
Failures:          0
```

**Module 2 is complete.**