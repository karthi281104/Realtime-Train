cmake_minimum_required(VERSION 3.20)

project(TCAS
    VERSION 0.1.0
    DESCRIPTION "Real-Time Train Collision Avoidance System with Predictive Conflict Resolution"
    LANGUAGES CXX
)

# ============================================================
# C++ Standard
# ============================================================

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================
# Compiler Warnings
# ============================================================

if(MSVC)

    add_compile_options(
        /W4
        /permissive-
    )

else()

    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
    )

endif()

# ============================================================
# TCAS Core Library
# ============================================================

add_library(tcas_core

    # Module 1 - Railway Infrastructure
    src/infrastructure/Node.cpp
    src/infrastructure/Track.cpp
    src/infrastructure/Station.cpp
    src/infrastructure/Junction.cpp
    src/infrastructure/Platform.cpp
    src/infrastructure/RailwayNetwork.cpp

    # Module 2 - Train Management
    src/train/Train.cpp
    src/train/ExpressTrain.cpp
    src/train/PassengerTrain.cpp
    src/train/FreightTrain.cpp
    src/train/TrainManager.cpp

    # Module 3 - Simulation & Clock
    src/simulation/SimClock.cpp
    src/simulation/SimulationTimer.cpp

    # Module 4 - Physics & Braking
    src/physics/KinematicsEngine.cpp

    # Module 5 - Route & Navigation
    src/navigation/RouteNavigator.cpp

    # Module 6 - Sensor & State Estimation
    src/sensor/Odometer.cpp
    src/sensor/StateEstimator.cpp

    # Module 7 - Communication Simulation
    src/communication/Message.cpp
    src/communication/CommunicationChannel.cpp
)

target_include_directories(tcas_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

# ============================================================
# Main Application
# ============================================================

add_executable(tcas
    src/main.cpp
    src/demo/FoundationDemo.cpp
    src/demo/SimulationDemo.cpp
    src/demo/IntegratedDemo.cpp
)

target_link_libraries(tcas
    PRIVATE
        tcas_core
)

target_include_directories(tcas
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# ============================================================
# GoogleTest
# ============================================================

include(FetchContent)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)

FetchContent_MakeAvailable(googletest)

# ============================================================
# Testing
# ============================================================

enable_testing()

# ============================================================
# TCAS GoogleTest Executable
# ============================================================

add_executable(tcas_tests

    # Module 1 Tests
    tests/infrastructure/NodeTest.cpp
    tests/infrastructure/TrackTest.cpp
    tests/infrastructure/RailwayNetworkTest.cpp

    # Module 2 Tests - Train Management
    tests/train/TrainTest.cpp
    tests/train/ExpressTrainTest.cpp
    tests/train/PassengerTrainTest.cpp
    tests/train/FreightTrainTest.cpp
    tests/train/TrainManagerTest.cpp

    # Module 3 Tests - Simulation & Clock
    tests/simulation/SimClockTest.cpp
    tests/simulation/SimulationConfigTest.cpp
    tests/simulation/SimulationTimerTest.cpp

    # Module 4 Tests - Physics & Braking
    tests/physics/KinematicsEngineTest.cpp

    # Module 5 Tests - Route & Navigation
    tests/navigation/RouteNavigatorTest.cpp

    # Integration Tests - Modules 4 & 5 with 1-3
    tests/integration/PhysicsNavigationIntegrationTest.cpp

    # Module 6 Tests - Sensor & State Estimation
    tests/sensor/OdometerTest.cpp
    tests/sensor/StateEstimatorTest.cpp

    # Module 7 Tests - Communication Simulation
    tests/communication/MessageTest.cpp
    tests/communication/CommunicationChannelTest.cpp
)

target_include_directories(tcas_tests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(tcas_tests
    PRIVATE
        tcas_core
        GTest::gtest
        GTest::gtest_main
)

# ============================================================
# GoogleTest Test Discovery
# ============================================================

include(GoogleTest)

gtest_discover_tests(tcas_tests)