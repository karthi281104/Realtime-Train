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

    // Run the full end-to-end integration demo of all implemented modules (1 to 7)
    tcas::demo::runIntegratedDemo();

    return 0;
}
