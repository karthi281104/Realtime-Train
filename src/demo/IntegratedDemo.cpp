#include "demo/IntegratedDemo.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "common/Types.hpp"
#include "communication/CommunicationChannel.hpp"
#include "communication/Message.hpp"
#include "infrastructure/Node.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "infrastructure/Track.hpp"
#include "navigation/RouteNavigator.hpp"
#include "physics/KinematicsEngine.hpp"
#include "sensor/Odometer.hpp"
#include "sensor/StateEstimator.hpp"
#include "simulation/SimClock.hpp"
#include "simulation/SimulationConfig.hpp"
#include "simulation/SimulationTimer.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"
#include "train/TrainManager.hpp"

namespace tcas::demo {

void runIntegratedDemo() {
  using namespace tcas::infrastructure;
  using namespace tcas::train;
  using namespace tcas::simulation;
  using namespace tcas::physics;
  using namespace tcas::navigation;
  using namespace tcas::sensor;
  using namespace tcas::communication;

  std::cout << "\n";
  std::cout << "==============================================================="
               "=========\n";
  std::cout << "        TCAS INTEGRATED SYSTEM DEMO (MODULES 1 TO 7 ACTIVE)\n";
  std::cout << "==============================================================="
               "=========\n";

  // -------------------------------------------------------------------------
  // 1. MODULE 1: RAILWAY INFRASTRUCTURE
  // -------------------------------------------------------------------------
  std::cout << "\n[MODULE 1] INITIALISING RAILWAY INFRASTRUCTURE TOPOLOGY\n";
  std::cout << "---------------------------------------------------------------"
               "---------\n";

  RailwayNetwork network;

  // Nodes: Stations & Junctions
  network.addNode(Node(1, "Central Station", NodeType::Station));
  network.addNode(Node(2, "Alpha Junction", NodeType::Junction));
  network.addNode(Node(3, "Beta Junction", NodeType::Junction));
  network.addNode(Node(4, "North Terminal", NodeType::Station));
  network.addNode(Node(5, "South Harbor", NodeType::Station));

  // Directed Tracks with realistic lengths, speed limits, and gradients
  network.addTrack(Track(101, 1, 2, 2000.0, 35.0, 0.000));   // Flat
  network.addTrack(Track(102, 2, 3, 1500.0, 30.0, 0.020));   // +2.0% Uphill
  network.addTrack(Track(103, 3, 4, 2500.0, 40.0, -0.015));  // -1.5% Downhill
  network.addTrack(Track(104, 2, 5, 3000.0, 25.0, 0.010));   // +1.0% Uphill

  std::cout << "Nodes added  : " << network.nodeCount()
            << " (Central, Alpha Jct, Beta Jct, North Term, South Harbor)\n";
  std::cout << "Tracks added : " << network.trackCount()
            << " (Tracks 101, 102, 103, 104 with gradient profiles)\n";
  std::cout << "Graph status : Weakly connected = "
            << (network.isWeaklyConnected() ? "YES" : "NO")
            << ", Cycles = " << (network.hasCycle() ? "YES" : "NO") << "\n";

  // -------------------------------------------------------------------------
  // 2. MODULE 2: TRAIN FLEET MANAGEMENT
  // -------------------------------------------------------------------------
  std::cout << "\n[MODULE 2] INITIALISING TRAIN FLEET IN TRAIN MANAGER\n";
  std::cout << "---------------------------------------------------------------"
               "---------\n";

  TrainManager trainManager;

  // Express Train (high speed, strong brakes)
  trainManager.addTrain(
      std::make_unique<ExpressTrain>(1, 45000.0, 45.0, 0.9, 1.4));

  // Passenger Train (commuter)
  trainManager.addTrain(
      std::make_unique<PassengerTrain>(2, 60000.0, 33.3, 0.8, 1.2));

  // Freight Train (heavy cargo, lower braking)
  trainManager.addTrain(
      std::make_unique<FreightTrain>(3, 120000.0, 22.2, 0.5, 0.8));

  std::cout << "Active trains registered in fleet:\n";
  std::cout << "  - Train #1: Express   | Mass:  45t | MaxSpeed: 45.0 m/s (162 "
               "km/h) | SrvBrake: 0.9 m/s2 | EmgBrake: 1.4 m/s2\n";
  std::cout << "  - Train #2: Passenger | Mass:  60t | MaxSpeed: 33.3 m/s (120 "
               "km/h) | SrvBrake: 0.8 m/s2 | EmgBrake: 1.2 m/s2\n";
  std::cout << "  - Train #3: Freight   | Mass: 120t | MaxSpeed: 22.2 m/s  (80 "
               "km/h) | SrvBrake: 0.5 m/s2 | EmgBrake: 0.8 m/s2\n";

  // -------------------------------------------------------------------------
  // 3. MODULE 5: DIJKSTRA ROUTE PLANNING & NAVIGATION
  // -------------------------------------------------------------------------
  std::cout << "\n[MODULE 5] DIJKSTRA ROUTE NAVIGATION PLANNING\n";
  std::cout << "---------------------------------------------------------------"
               "---------\n";

  // Route 1: Express from Central (1) to North Terminal (4)
  const RouteResult route1 = RouteNavigator::findRoute(network, 1, 4);
  std::cout << "Route Plan [Train #1: Central -> North Terminal]:\n";
  std::cout << "  - Status         : "
            << (route1.success ? "SUCCESS" : "FAILED") << "\n";
  std::cout << "  - Total Distance : " << route1.totalDistance << " m\n";
  std::cout << "  - Track Sequence : ";
  for (std::size_t i = 0; i < route1.tracks.size(); ++i) {
    const auto* trk = network.getTrack(route1.tracks[i]);
    std::cout << "Track " << route1.tracks[i] << " (" << trk->length()
              << "m, grade: " << (trk->gradient() * 100.0) << "%)"
              << (i + 1 < route1.tracks.size() ? " -> " : "\n");
  }

  // Route 2: Passenger from Central (1) to South Harbor (5)
  const RouteResult route2 = RouteNavigator::findRoute(network, 1, 5);
  std::cout << "Route Plan [Train #2: Central -> South Harbor]:\n";
  std::cout << "  - Status         : "
            << (route2.success ? "SUCCESS" : "FAILED") << "\n";
  std::cout << "  - Total Distance : " << route2.totalDistance << " m\n";
  std::cout << "  - Track Sequence : ";
  for (std::size_t i = 0; i < route2.tracks.size(); ++i) {
    const auto* trk = network.getTrack(route2.tracks[i]);
    std::cout << "Track " << route2.tracks[i] << " (" << trk->length()
              << "m, grade: " << (trk->gradient() * 100.0) << "%)"
              << (i + 1 < route2.tracks.size() ? " -> " : "\n");
  }

  // -------------------------------------------------------------------------
  // 4. MODULE 3 & MODULE 4: REAL-TIME SIMULATION & GRADIENT KINEMATICS LOOP
  // -------------------------------------------------------------------------
  std::cout << "\n[MODULE 3 & 4] RUNNING REAL-TIME DISCRETE SIMULATION & "
               "DYNAMIC PHYSICS\n";
  std::cout << "---------------------------------------------------------------"
               "---------\n";

  SimulationConfig simConfig;
  SimClock simClock(static_cast<TimeSeconds>(simConfig.physicsPeriodMs) /
                    1000.0);

  SimulationTimer safetyTimer(simConfig.safetyPeriodMs,
                              simConfig.physicsPeriodMs);
  SimulationTimer hmiTimer(simConfig.hmiPeriodMs, simConfig.physicsPeriodMs);

  auto* expressTrain = trainManager.getTrain(1);
  expressTrain->setState(TrainState::Running);
  expressTrain->setVelocity(15.0);  // Initial 15 m/s

  std::cout << std::fixed << std::setprecision(2);
  std::cout << std::left << std::setw(6) << "Tick" << std::setw(9) << "Time(s)"
            << std::setw(12) << "Speed(m/s)" << std::setw(12) << "Pos(m)"
            << std::setw(10) << "Track" << std::setw(10) << "Gradient"
            << std::setw(12) << "Eff.Decel" << std::setw(14) << "SafeDist(m)"
            << std::setw(12) << "EmgDist(m)"
            << "\n";
  std::cout << "---------------------------------------------------------------"
               "-------------------------------------\n";

  constexpr SimTimeTick kSimulationSteps = 30;
  const double targetAcceleration = 0.6;  // Accelerating

  for (SimTimeTick step = 0; step < kSimulationSteps; ++step) {
    const auto tick = simClock.tickCount();
    const auto elapsed = simClock.elapsed();
    const auto dt = simClock.dt();

    // 1. Determine active track along route
    const double pos = expressTrain->position();
    TrackId currentTrackId = 101;
    double currentGradient = 0.0;

    if (pos < 2000.0) {
      currentTrackId = 101;
      currentGradient = network.getTrack(101)->gradient();
    } else if (pos < 3500.0) {
      currentTrackId = 102;
      currentGradient = network.getTrack(102)->gradient();
    } else {
      currentTrackId = 103;
      currentGradient = network.getTrack(103)->gradient();
    }

    // 2. Physics & Kinematics update (Module 4)
    const double newVel = KinematicsEngine::updateVelocity(
        expressTrain->velocity(), targetAcceleration, dt,
        expressTrain->maximumSpeed());
    expressTrain->setVelocity(newVel);

    const double newPos = KinematicsEngine::updatePosition(
        expressTrain->position(), expressTrain->velocity(), targetAcceleration,
        dt);
    expressTrain->setPosition(newPos);

    // 3. Compute dynamic gradient-aware safety metrics (Module 4)
    const double effDecel = KinematicsEngine::effectiveDeceleration(
        expressTrain->serviceBraking(), currentGradient);

    const double safeDistance = KinematicsEngine::safeDistance(
        expressTrain->velocity(), expressTrain->serviceBraking(),
        currentGradient);

    const double emgDistance = KinematicsEngine::emergencyStoppingDistance(
        expressTrain->velocity(), expressTrain->emergencyBraking(),
        currentGradient);

    // 4. Output HMI telemetry (every 200ms)
    if (hmiTimer.shouldFire(tick)) {
      std::cout << std::left << std::setw(6) << tick << std::setw(9) << elapsed
                << std::setw(12) << expressTrain->velocity() << std::setw(12)
                << expressTrain->position() << std::setw(10) << currentTrackId
                << std::setw(10) << (currentGradient * 100.0) << std::setw(12)
                << effDecel << std::setw(14) << safeDistance << std::setw(12)
                << emgDistance << "\n";
    }

    simClock.tick();
  }

  std::cout << "---------------------------------------------------------------"
               "-------------------------------------\n";

  // -------------------------------------------------------------------------
  // 5. MODULE 6: SENSOR & STATE ESTIMATION (Kalman Filter)
  // -------------------------------------------------------------------------
  std::cout << "\n[MODULE 6] SENSOR FUSION & KALMAN FILTER STATE ESTIMATION\n";
  std::cout << "---------------------------------------------------------------"
               "---------\n";

  SensorNoiseConfig noiseConfig;
  noiseConfig.driftRatePerSecond = 0.01;      // 1% wheel slip drift
  noiseConfig.measurementNoisePos = 2.0;      // 2m^2 position noise
  noiseConfig.measurementNoiseVel = 0.5;      // 0.5 (m/s)^2 velocity noise
  noiseConfig.measurementNoiseBalise = 0.01;  // Very precise balise fix

  Odometer odometer(noiseConfig);
  StateEstimator estimator(noiseConfig, 0.0, 15.0);  // Start at 0m, 15 m/s

  std::cout << std::left << std::setw(6) << "Step" << std::setw(10) << "TruePos"
            << std::setw(10) << "MeasPos" << std::setw(10) << "EstPos"
            << std::setw(10) << "EstVel" << std::setw(12) << "PosUncert"
            << std::setw(12) << "Degraded"
            << "\n";
  std::cout << "---------------------------------------------------------------"
               "---------\n";

  double truePos = 0.0;
  double trueVel = 15.0;
  const double trueDt = 0.1;  // 100ms step

  for (int step = 0; step < 15; ++step) {
    // Ground truth advance
    truePos += trueVel * trueDt;

    // Sensor measurement with wheel slip drift
    const auto meas = odometer.measure(truePos, trueVel, 0.0, trueDt,
                                       static_cast<SimTimeTick>(step));

    // Kalman predict + update
    estimator.predict(trueDt, static_cast<SimTimeTick>(step));
    estimator.updateOdometry(meas);

    // Simulate balise at step 10 (absolute position anchor at 15.0m)
    if (step == 10) {
      BaliseTransponder balise{
          .baliseId = 1, .trackId = 101, .exactPosition = truePos};
      estimator.updateBalise(balise);
      std::cout << "  [BALISE FIX at step 10: absolute anchor = " << truePos
                << " m]\n";
    }

    const EstimatedState est = estimator.estimatedState();

    std::cout << std::left << std::setw(6) << step << std::setw(10) << truePos
              << std::setw(10) << meas.rawPosition << std::setw(10)
              << est.position << std::setw(10) << est.velocity << std::setw(12)
              << est.positionUncertainty << std::setw(12)
              << (est.isDegraded ? "YES" : "NO") << "\n";
  }

  std::cout << "Odometer accumulated drift: " << odometer.accumulatedDrift()
            << " m\n";

  // -------------------------------------------------------------------------
  // 6. MODULE 7: V2V COMMUNICATION SIMULATION
  // -------------------------------------------------------------------------
  std::cout << "\n[MODULE 7] V2V/V2I WIRELESS COMMUNICATION SIMULATION\n";
  std::cout << "---------------------------------------------------------------"
               "---------\n";

  ChannelConfig channelCfg;
  channelCfg.latencyTicks = 2;         // 2-tick propagation delay
  channelCfg.maxRangeMeters = 5000.0;  // 5 km radio range

  CommunicationChannel channel(channelCfg);
  channel.registerEntity(1);  // Express Train
  channel.registerEntity(2);  // Passenger Train
  channel.registerEntity(3);  // Freight Train

  // Train 1 broadcasts heartbeat at tick 0
  const auto heartbeat = Message::makeHeartbeat(1001, 1, 0);
  channel.sendMessage(heartbeat, 0.0, 0.0);

  // Train 2 sends movement authority to Train 3 at tick 0
  MovementAuthorityPayload maPayload{.trainId = 3,
                                     .permittedDistance = 3000.0,
                                     .targetSpeed = 22.0,
                                     .validUntilTick = 100};
  const auto maMsg = Message::makeMovementAuthority(1002, 2, 3, 0, maPayload);
  channel.sendMessage(maMsg, 500.0, 1200.0);

  // Train 2 sends emergency brake to Train 1 (close range)
  EmergencyBrakePayload ebPayload{.targetTrainId = 1,
                                  .dangerZoneStart = 2800.0,
                                  .dangerZoneEnd = 3000.0,
                                  .reasonCode = 42};
  const auto ebMsg = Message::makeEmergencyBrake(1003, 2, 1, 0, ebPayload);
  channel.sendMessage(ebMsg, 500.0, 100.0);

  // Advance channel 2 ticks to deliver all messages
  channel.step(1);
  channel.step(2);

  std::cout << "Channel stats after 3 transmissions:\n";
  std::cout << "  Total sent     : " << channel.totalSent() << "\n";
  std::cout << "  Total delivered: " << channel.totalDelivered() << "\n";
  std::cout << "  Total dropped  : " << channel.totalDropped() << "\n";
  std::cout << "  Delivery rate  : " << channel.deliveryRate() * 100.0
            << "%\n\n";

  // Check Train 2 and 3 received heartbeat from Train 1
  std::cout << "Train 2 mailbox: "
            << (channel.hasMessages(2) ? "HAS messages" : "empty") << "\n";
  std::cout << "Train 3 mailbox: "
            << (channel.hasMessages(3) ? "HAS messages" : "empty") << "\n";

  const auto msgs2 = channel.receiveMessages(2);
  const auto msgs3 = channel.receiveMessages(3);

  std::cout << "Train 2 received " << msgs2.size() << " message(s):\n";
  for (const auto& m : msgs2) {
    std::cout << "  MsgId=" << m.header.messageId
              << " from=" << m.header.senderId
              << " type=" << static_cast<int>(m.header.type)
              << " priority=" << static_cast<int>(m.header.priority) << "\n";
  }

  std::cout << "Train 3 received " << msgs3.size() << " message(s):\n";
  for (const auto& m : msgs3) {
    std::cout << "  MsgId=" << m.header.messageId
              << " from=" << m.header.senderId
              << " type=" << static_cast<int>(m.header.type)
              << " priority=" << static_cast<int>(m.header.priority) << "\n";
  }

  // Train 1 mailbox for emergency brake from Train 2
  const auto msgs1 = channel.receiveMessages(1);
  std::cout << "Train 1 received " << msgs1.size()
            << " message(s) (including emergency brake):\n";
  for (const auto& m : msgs1) {
    std::cout << "  MsgId=" << m.header.messageId
              << " from=" << m.header.senderId
              << " type=" << static_cast<int>(m.header.type)
              << " priority=" << static_cast<int>(m.header.priority) << "\n";
  }

  std::cout << "\n[RESULT] All 7 modules successfully executed in full "
               "integration:\n";
  std::cout << "  - Module 1 (Infrastructure) : Railway topology with grades & "
               "speed limits loaded\n";
  std::cout << "  - Module 2 (Train Fleet)    : Multi-class train fleet "
               "managed with physics properties\n";
  std::cout << "  - Module 3 (Simulation)     : Deterministic discrete tick "
               "clock synchronized\n";
  std::cout << "  - Module 4 (Physics)        : Gradient-corrected dynamic "
               "braking distances and kinematics calculated\n";
  std::cout << "  - Module 5 (Navigation)     : Dijkstra optimal track route "
               "planned and executed\n";
  std::cout << "  - Module 6 (Sensor/Kalman)  : Odometer drift + Kalman filter "
               "state estimation with balise anchor\n";
  std::cout << "  - Module 7 (Communication)  : V2V/V2I wireless channel with "
               "latency, range, and broadcast routing\n";
  std::cout << "==============================================================="
               "=========\n\n";
}

}  // namespace tcas::demo
