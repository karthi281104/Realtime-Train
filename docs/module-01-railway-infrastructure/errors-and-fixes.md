# Module 1 — Errors and Fixes

## Issue #001 — Duplicate GoogleTest Target

### Problem

During the initial Module 1 CMake integration, configuration failed with:

```text
CMake Error:
add_executable cannot create target "tcas_tests"
because another target with the same name already exists.
```

### Root Cause

The CMake configuration defined the `tcas_tests` executable twice.

One definition contained the Module 1 tests:

```text
NodeTest.cpp
TrackTest.cpp
RailwayNetworkTest.cpp
```

A second definition still referenced the original boilerplate:

```text
tests/test_main.cpp
```

CMake does not allow two executable targets with the same name in the same project.

### Fix

The duplicate test target was removed.

The project now defines exactly one:

```cmake
add_executable(tcas_tests
    tests/infrastructure/NodeTest.cpp
    tests/infrastructure/TrackTest.cpp
    tests/infrastructure/RailwayNetworkTest.cpp
)
```

The target is linked against:

```text
tcas_core
GTest::gtest
GTest::gtest_main
```

### Verification

CMake configuration completed successfully after the correction.

---

## Issue #002 — GoogleTest PDB Build Failure

### Problem

During an earlier parallel build, GoogleTest failed while compiling `gtest_main.cc` with:

```text
error C1090:
PDB API call failed, error code '3'
```

### Environment

```text
Windows
Visual Studio 2022 Build Tools
MSVC 19.42
CMake 4.4
GoogleTest 1.17.0
```

### Investigation

The TCAS application itself successfully compiled, and GoogleTest's other libraries were also being built.

The failure occurred specifically while generating the debug information/PDB for `gtest_main`.

### Fix / Resolution

The build directory was removed and the project was configured again from a clean state.

The build was then performed using the explicit Visual Studio Debug configuration:

```powershell
cmake --build build --config Debug
```

CTest was executed using:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

The subsequent clean build successfully produced:

```text
gtest.lib
gtest_main.lib
tcas_core.lib
tcas.exe
tcas_tests.exe
```

### Verification

GoogleTest executed successfully and all Module 1 tests passed.

---

## Issue #003 — CTest Reported No Tests

### Problem

CTest initially reported:

```text
No tests were found!!!
```

### Root Cause

The GoogleTest test executable had not successfully completed its build because of the earlier PDB failure.

Additionally, the Visual Studio multi-configuration generator requires the correct configuration to be specified when running CTest.

### Fix

The test executable was successfully rebuilt and CTest was run with:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

### Verification

CTest successfully discovered and executed all Module 1 tests.

Final result:

```text
17/17 tests passed
100% tests passed
```

---

## Status

All known Module 1 build and test integration issues are resolved.