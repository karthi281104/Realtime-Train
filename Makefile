.PHONY: all configure build run test test-physics test-route test-sensor test-comm test-integration test-verbose check clean help

BUILD_DIR = build
CMAKE = cmake
CTEST = ctest
CPPCHECK = cppcheck

ifeq ($(OS),Windows_NT)
    CMAKE_GENERATOR = -G "MinGW Makefiles"
    RUN_EXE = .\$(BUILD_DIR)\tcas.exe
    TEST_EXE = .\$(BUILD_DIR)\tcas_tests.exe
    CLEAN_CMD = powershell -Command "if (Test-Path $(BUILD_DIR)) { Remove-Item -Recurse -Force $(BUILD_DIR) }; Write-Host 'Build directory cleaned.'"
else
    CMAKE_GENERATOR =
    RUN_EXE = ./$(BUILD_DIR)/tcas
    TEST_EXE = ./$(BUILD_DIR)/tcas_tests
    CLEAN_CMD = rm -rf $(BUILD_DIR) && echo "Build directory cleaned."
endif

all: build

configure:
	@$(CMAKE) -S . -B $(BUILD_DIR) $(CMAKE_GENERATOR) -DCMAKE_BUILD_TYPE=RelWithDebInfo

build: configure
	@$(CMAKE) --build $(BUILD_DIR) --parallel

run: build
	@$(RUN_EXE)

test: build
	@$(TEST_EXE)

test-verbose: build
	@$(CTEST) --test-dir $(BUILD_DIR) --output-on-failure -V

test-physics: build
	@$(TEST_EXE) --gtest_filter="KinematicsEngineTest.*"

test-route: build
	@$(TEST_EXE) --gtest_filter="RouteNavigatorTest.*"

test-sensor: build
	@$(TEST_EXE) --gtest_filter="OdometerTest.*:StateEstimatorTest.*"

test-comm: build
	@$(TEST_EXE) --gtest_filter="MessageTest.*:CommunicationChannelTest.*"

test-integration: build
	@$(TEST_EXE) --gtest_filter="PhysicsNavigationIntegrationTest.*"

check:
	@$(CPPCHECK) --enable=warning,style,performance,portability --suppress=missingIncludeSystem --suppress=unusedFunction --std=c++20 -I include src tests

clean:
	@$(CLEAN_CMD)

help:
	@echo ============================================================
	@echo                    TCAS BUILD SHORTCUTS
	@echo ============================================================
	@echo   make build             - Configure and compile the project
	@echo   make run               - Build and launch the interactive demo
	@echo   make test              - Run all unit and integration tests
	@echo   make test-physics      - Run Module 4 Physics tests
	@echo   make test-route        - Run Module 5 Navigation tests
	@echo   make test-sensor       - Run Module 6 Sensor & Estimation tests
	@echo   make test-comm         - Run Module 7 Communication tests
	@echo   make test-integration  - Run cross-module integration tests
	@echo   make test-verbose      - Run all tests with full CTest verbosity
	@echo   make check             - Run Cppcheck static analysis
	@echo   make clean             - Remove build directory
	@echo ============================================================
