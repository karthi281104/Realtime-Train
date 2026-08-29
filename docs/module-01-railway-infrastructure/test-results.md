# Module 1 — Test Results

## 1. Test Framework

The TCAS project uses:

```text
GoogleTest
```

as its standard unit-testing framework.

CTest is used as the test execution and discovery layer.

---

## 2. Build Configuration

```text
Compiler:
Microsoft Visual C++ 19.42.34435.0

Build System:
CMake

C++ Standard:
C++23

Test Framework:
GoogleTest 1.17.0

Configuration:
Debug
```

---

## 3. Test Command

The Module 1 test suite was executed using:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

---

## 4. Test Result

```text
17/17 tests passed

100% tests passed out of 17

Total Test time:
0.40 sec
```

---

## 5. Test Cases

### Node Tests

```text
NodeTest.StoresNodeInformation
NodeTest.SupportsJunctionType
NodeTest.SupportsPlatformType
```

Result:

```text
PASS
```

### Track Tests

```text
TrackTest.StoresTrackInformation
```

Result:

```text
PASS
```

### Railway Network Tests

```text
RailwayNetworkTest.StartsEmpty
RailwayNetworkTest.AddsNode
RailwayNetworkTest.RejectsDuplicateNode
RailwayNetworkTest.AddsValidTrack
RailwayNetworkTest.RejectsTrackWithMissingSource
RailwayNetworkTest.RejectsTrackWithMissingDestination
RailwayNetworkTest.RejectsDuplicateTrack
RailwayNetworkTest.FindsNode
RailwayNetworkTest.MissingNodeReturnsNull
RailwayNetworkTest.FindsTrack
RailwayNetworkTest.EmptyNetworkIsConnected
RailwayNetworkTest.SingleNodeNetworkIsConnected
RailwayNetworkTest.BFSDetectsConnectedNetwork
RailwayNetworkTest.BFSDetectsDisconnectedNetwork
RailwayNetworkTest.DFSDetectsNoCycle
RailwayNetworkTest.DFSDetectsCycle
```

Result:

```text
PASS
```

---

## 6. Final Test Summary

| Category | Tests | Passed | Failed |
|---|---:|---:|---:|
| Node | 3 | 3 | 0 |
| Track | 1 | 1 | 0 |
| RailwayNetwork | 13 | 13 | 0 |
| **Total** | **17** | **17** | **0** |

---

## 7. Module Status

```text
BUILD: PASS
TEST DISCOVERY: PASS
UNIT TESTS: PASS
EDGE CASE TESTS: PASS

17/17 PASS
```

Module 1 satisfies its current completion criteria.

---

## 8. Next Module

The next module is:

```text
Module 2 — Train Management
```

Module 2 will introduce the train entity model while consuming the infrastructure foundation established by Module 1.