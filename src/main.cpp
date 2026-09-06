#include "demo/IntegratedDemo.hpp"
#include "demo/FoundationDemo.hpp"
#include "demo/SimulationDemo.hpp"
#include "demo/Module9Demo.hpp"

#include <iostream>

int main()
{
    std::cout << "\n";
    std::cout << "============================================================\n";
    std::cout << "              TCAS REAL-TIME TRAIN SYSTEM\n";
    std::cout << "============================================================\n";

    // Run the existing Modules 1-7 integrated demonstration.
    tcas::demo::runIntegratedDemo();

    // Run Module 9 against the existing infrastructure/navigation/prediction
    // data flow. Module 10 will consume these conflict results next.
    tcas::demo::runModule9Demo();

    return 0;
}
