#include <iostream>

#include "demo/FoundationDemo.hpp"
#include "demo/IntegratedDemo.hpp"
#include "demo/SimulationDemo.hpp"

int main() {
  std::cout << "\n";
  std::cout << "============================================================\n";
  std::cout << "              TCAS REAL-TIME TRAIN SYSTEM\n";
  std::cout << "============================================================\n";

  // Run the full end-to-end integration demo of all implemented modules (1 to
  // 5)
  tcas::demo::runIntegratedDemo();

  return 0;
}
