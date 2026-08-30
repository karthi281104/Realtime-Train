.PHONY: all configure build run test test-physics test-route test-integration check clean help

BUILD_DIR = build
CMAKE = cmake
CTEST = ctest
CPPCHECK = cppcheck

all: build

configure:
	@$(CMAKE) -S . -B $(BUILD_DIR) -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=RelWithDebInfo

build: configure
	@$(CMAKE) --build $(BUILD_DIR) --parallel

run: build
	@.\$(BUILD_DIR)\tcas.exe

test: build
	@.\$(BUILD_DIR)\tcas_tests.exe

test-verbose: build
	@$(CTEST) --test-dir $(BUILD_DIR) --output-on-failure -V

test-physics: build
	@.\$(BUILD_DIR)\tcas_tests.exe --gtest_filter="KinematicsEngineTest.*"

test-route: build
	@.\$(BUILD_DIR)\tcas_tests.exe --gtest_filter="RouteNavigatorTest.*"

test-integration: build
	@.\$(BUILD_DIR)\tcas_tests.exe --gtest_filter="PhysicsNavigationIntegrationTest.*"

check:
	@$(CPPCHECK) --enable=warning,style,performance,portability --suppress=missingIncludeSystem --suppress=unusedFunction --std=c++20 -I include src tests

clean:
	@powershell -Command "if (Test-Path $(BUILD_DIR)) { Remove-Item -Recurse -Force $(BUILD_DIR) }; Write-Host 'Build directory cleaned.'"

help:
	@echo ============================================================
	@echo                    TCAS BUILD SHORTCUTS
	@echo ============================================================
	@echo   make build             - Configure and compile the project
	@echo   make run               - Build and launch the interactive demo
	@echo   make test              - Run the complete automated test suite
	@echo   make test-verbose      - Run all tests with full CTest verbosity
	@echo   make test-physics      - Run Module 4 Physics tests
	@echo   make test-route        - Run Module 5 Navigation tests
	@echo   make test-integration  - Run cross-module integration tests
	@echo   make check             - Run Cppcheck static analysis
	@echo   make clean             - Remove build directory
	@echo ============================================================
