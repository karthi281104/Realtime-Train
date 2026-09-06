#include "demo/IntegratedDemo.hpp"
#include "demo/FoundationDemo.hpp"
#include "demo/SimulationDemo.hpp"

#include <iostream>

int main()
{
    std::cout << "\n";
    std::cout << "============================================================\n";
    std::cout << "              TCAS REAL-TIME TRAIN SYSTEM\n";
    std::cout << "============================================================\n";

    // Run the existing Modules 1-7 integrated demonstration.
    tcas::demo::runIntegratedDemo();

    return 0;
}
