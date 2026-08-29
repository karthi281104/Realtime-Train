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
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
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