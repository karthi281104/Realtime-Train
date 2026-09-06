#include "conflict/ConflictDetector.hpp"
#include "conflict/ResourceReservationManager.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "navigation/RouteNavigator.hpp"
#include "prediction/FutureState.hpp"

#include <iomanip>
#include <iostream>

namespace tcas::demo
{
namespace
{

infrastructure::RailwayNetwork makeNetwork()
{
    infrastructure::RailwayNetwork network;
    network.addNode({1, "Express Origin", infrastructure::NodeType::Generic});
    network.addNode({2, "J1", infrastructure::NodeType::Junction});
    network.addNode({3, "Express Destination", infrastructure::NodeType::Generic});
    network.addNode({4, "Freight Origin", infrastructure::NodeType::Generic});
    network.addNode({5, "Freight Destination", infrastructure::NodeType::Generic});
    network.addTrack({101, 1, 2, 1000.0, 30.0, 0.0});
    network.addTrack({102, 2, 3, 1000.0, 30.0, 0.0});
    network.addTrack({103, 4, 2, 1000.0, 30.0, 0.0});
    network.addTrack({104, 2, 5, 1000.0, 30.0, 0.0});
    return network;
}

std::vector<prediction::FutureState> makeTrajectory(
    TrackId approach,
    TrackId departure,
    TimeSeconds junctionTime)
{
    return {
        {0.0, approach, 0.0, 20.0, 0.0, 1.0},
        {junctionTime, departure, 0.0, 20.0, 0.0, 1.0},
        {junctionTime + 10.0, departure, 200.0, 20.0, 0.0, 1.0}
    };
}

} // namespace

void runModule9Demo()
{
    std::cout << "\n============================================================\n";
    std::cout << "             MODULE 9: CONFLICT INTEGRATION\n";
    std::cout << "============================================================\n";

    const auto network = makeNetwork();
    const auto expressRoute = navigation::RouteNavigator::findRoute(network, 1, 3);
    const auto freightRoute = navigation::RouteNavigator::findRoute(network, 4, 5);

    const auto expressPrediction = makeTrajectory(101, 102, 20.0);
    const auto freightPrediction = makeTrajectory(103, 104, 21.0);

    conflict::ConflictDetector detector;
    const auto conflicts = detector.detect(
        1, expressPrediction, 2, freightPrediction, network);

    conflict::ResourceReservationManager reservations;
    const conflict::ConflictZone junction{
        conflict::ConflictZoneType::Junction, 2, 0};

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Express route: " << (expressRoute.success ? "OK" : "FAILED") << '\n';
    std::cout << "Freight route: " << (freightRoute.success ? "OK" : "FAILED") << '\n';
    std::cout << "Predicted conflicts: " << conflicts.size() << '\n';

    if (!conflicts.empty())
    {
        const auto& conflict = conflicts.front();
        std::cout << "Conflict: Junction J1, trains "
                  << conflict.trainA << " and " << conflict.trainB << '\n';
        std::cout << "Conflict window: [" << conflict.firstConflictTime
                  << ", " << conflict.lastConflictTime << ") s\n";

        const bool expressGranted = reservations.request(
            1, junction, conflict.firstConflictTime, conflict.lastConflictTime);
        const bool freightGranted = reservations.request(
            2, junction, conflict.firstConflictTime, conflict.lastConflictTime);

        std::cout << "J1 reservation -> Express: "
                  << (expressGranted ? "GRANTED" : "DENIED") << '\n';
        std::cout << "J1 reservation -> Freight: "
                  << (freightGranted ? "GRANTED" : "DENIED") << '\n';

        if (expressGranted)
        {
            reservations.release(1, junction);
            reservations.clearReleased();
        }

        std::cout << "J1 state: RELEASED\n";
        std::cout << "RESULT: collision avoided by predictive reservation\n";
    }
    else
    {
        std::cout << "RESULT: no conflict detected\n";
    }
}

} // namespace tcas::demo
