#pragma once

#include "infrastructure/RailwayNetwork.hpp"
#include "train/TrainManager.hpp"

namespace tcas::demo
{

// Runs Module 9 using the infrastructure and fleet created by the integrated
// system demo.
void runModule9Demo(
	const infrastructure::RailwayNetwork& network,
	train::TrainManager& trainManager
);

} // namespace tcas::demo
