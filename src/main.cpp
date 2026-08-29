#include "demo/FoundationDemo.hpp"
#include "demo/SimulationDemo.hpp"

#include <iostream>

int main()
{
    std::cout << "\n";
    std::cout << "============================================================\n";
    std::cout << "              TCAS REAL-TIME TRAIN SYSTEM\n";
    std::cout << "============================================================\n";

    std::cout << "\nSelect demo:\n";
    std::cout << "1. Module 1 + Module 2 Foundation Demo\n";
    std::cout << "2. Module 3 Simulation Demo\n";
    std::cout << "3. Run both demos\n";
    std::cout << "\nEnter choice: ";

    int choice{};
    std::cin >> choice;

    switch (choice)
    {
    case 1:
        tcas::demo::runFoundationDemo();
        break;

    case 2:
        tcas::demo::runSimulationDemo();
        break;

    case 3:
        tcas::demo::runFoundationDemo();
        tcas::demo::runSimulationDemo();
        break;

    default:
        std::cout << "\nInvalid choice.\n";
        return 1;
    }

    return 0;
}