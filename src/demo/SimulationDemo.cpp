#include "demo/SimulationDemo.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "simulation/SimClock.hpp"
#include "simulation/SimulationConfig.hpp"
#include "simulation/SimulationTimer.hpp"

namespace tcas::demo {

void runSimulationDemo() {
  using namespace tcas::simulation;

  std::cout << '\n';
  std::cout << "============================================================\n";
  std::cout << "              TCAS MODULE 3 DEMO\n";
  std::cout << "          SIMULATION & DETERMINISTIC CLOCK\n";
  std::cout << "============================================================\n";

  // --------------------------------------------------------
  // 1. Simulation configuration
  // --------------------------------------------------------

  SimulationConfig config;

  std::cout << "\n[1] SIMULATION CONFIGURATION\n";
  std::cout << "------------------------------------------------------------\n";

  std::cout << "Physics period        : " << config.physicsPeriodMs << " ms\n";

  std::cout << "Safety period         : " << config.safetyPeriodMs << " ms\n";

  std::cout << "Communication period  : " << config.communicationPeriodMs
            << " ms\n";

  std::cout << "HMI period            : " << config.hmiPeriodMs << " ms\n";

  std::cout << "Prediction horizon    : " << config.predictionHorizonSeconds
            << " s\n";

  // --------------------------------------------------------
  // 2. Create deterministic simulation clock
  // --------------------------------------------------------

  SimClock clock(static_cast<TimeSeconds>(config.physicsPeriodMs) / 1000.0);

  std::cout << "\n[2] SIMULATION CLOCK\n";
  std::cout << "------------------------------------------------------------\n";

  std::cout << std::fixed << std::setprecision(3);

  std::cout << "Clock dt              : " << clock.dt() << " s\n";

  std::cout << "Initial simulation time: " << clock.elapsed() << " s\n";

  std::cout << "Initial tick count    : " << clock.tickCount() << '\n';

  // --------------------------------------------------------
  // 3. Create subsystem timers
  // --------------------------------------------------------

  SimulationTimer safetyTimer(config.safetyPeriodMs, config.physicsPeriodMs);

  SimulationTimer communicationTimer(config.communicationPeriodMs,
                                     config.physicsPeriodMs);

  SimulationTimer hmiTimer(config.hmiPeriodMs, config.physicsPeriodMs);

  std::cout << "\n[3] TIMER CONFIGURATION\n";
  std::cout << "------------------------------------------------------------\n";

  std::cout << "Safety timer interval        : " << safetyTimer.intervalTicks()
            << " ticks\n";

  std::cout << "Communication timer interval : "
            << communicationTimer.intervalTicks() << " ticks\n";

  std::cout << "HMI timer interval           : " << hmiTimer.intervalTicks()
            << " ticks\n";

  // --------------------------------------------------------
  // 4. Run deterministic simulation
  // --------------------------------------------------------

  constexpr SimTimeTick totalTicks = 25;

  std::cout << "\n[4] RUNNING SIMULATION\n";
  std::cout << "------------------------------------------------------------\n";

  std::cout << std::left << std::setw(8) << "Tick" << std::setw(12) << "Time(s)"
            << std::setw(12) << "Physics" << std::setw(12) << "Safety"
            << std::setw(18) << "Communication" << std::setw(12) << "HMI"
            << '\n';

  std::cout << "------------------------------------------------------------\n";

  for (SimTimeTick i = 0; i < totalTicks; ++i) {
    const auto tick = clock.tickCount();

    const bool safetyFire = safetyTimer.shouldFire(tick);

    const bool communicationFire = communicationTimer.shouldFire(tick);

    const bool hmiFire = hmiTimer.shouldFire(tick);

    std::cout << std::left << std::setw(8) << tick

              << std::setw(12) << clock.elapsed()

              << std::setw(12) << "RUN"

              << std::setw(12) << (safetyFire ? "FIRE" : "-")

              << std::setw(18) << (communicationFire ? "FIRE" : "-")

              << std::setw(12) << (hmiFire ? "FIRE" : "-")

              << '\n';

    // Advance deterministic simulation time.
    clock.tick();

    // Small delay ONLY for visual demonstration.
    //
    // IMPORTANT:
    // The simulation itself does NOT depend on this delay.
    // SimClock advances only through tick().
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config.physicsPeriodMs));
  }

  // --------------------------------------------------------
  // 5. Final state
  // --------------------------------------------------------

  std::cout << "\n[5] FINAL CLOCK STATE\n";
  std::cout << "------------------------------------------------------------\n";

  std::cout << "Final tick count     : " << clock.tickCount() << '\n';

  std::cout << "Final simulation time : " << clock.elapsed() << " s\n";

  // --------------------------------------------------------
  // 6. Reset demonstration
  // --------------------------------------------------------

  std::cout << "\n[6] CLOCK RESET\n";
  std::cout << "------------------------------------------------------------\n";

  clock.reset();

  std::cout << "After reset tick     : " << clock.tickCount() << '\n';

  std::cout << "After reset time     : " << clock.elapsed() << " s\n";

  std::cout
      << "\n============================================================\n";
  std::cout << "             MODULE 3 DEMO COMPLETE\n";
  std::cout << "============================================================\n";
}

}  // namespace tcas::demo