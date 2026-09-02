#include <gtest/gtest.h>

#include "communication/Message.hpp"

using namespace tcas;
using namespace tcas::communication;

TEST(MessageTest, CreatesTelemetryMessage) {
  TelemetryPayload data{.trainId = 1,
                        .position = 1250.0,
                        .velocity = 28.5,
                        .state = TrainState::Running,
                        .activeTrackId = 102};

  const auto msg = Message::makeTelemetry(1001, 1, 99, 50, data);

  EXPECT_EQ(msg.header.messageId, 1001u);
  EXPECT_EQ(msg.header.senderId, 1u);
  EXPECT_EQ(msg.header.recipientId, 99u);
  EXPECT_EQ(msg.header.sentTick, 50u);
  EXPECT_EQ(msg.header.type, MessageType::Telemetry);
  EXPECT_EQ(msg.header.priority, MessagePriority::Normal);

  ASSERT_TRUE(std::holds_alternative<TelemetryPayload>(msg.payload));
  const auto& payload = std::get<TelemetryPayload>(msg.payload);
  EXPECT_EQ(payload.trainId, 1u);
  EXPECT_DOUBLE_EQ(payload.position, 1250.0);
  EXPECT_DOUBLE_EQ(payload.velocity, 28.5);
  EXPECT_EQ(payload.state, TrainState::Running);
  EXPECT_EQ(payload.activeTrackId, 102u);
}

TEST(MessageTest, CreatesMovementAuthorityMessage) {
  MovementAuthorityPayload data{.trainId = 2,
                                .permittedDistance = 5000.0,
                                .targetSpeed = 33.3,
                                .validUntilTick = 500};

  const auto msg = Message::makeMovementAuthority(1002, 0, 2, 10, data);

  EXPECT_EQ(msg.header.type, MessageType::MovementAuthority);
  EXPECT_EQ(msg.header.priority, MessagePriority::High);

  ASSERT_TRUE(std::holds_alternative<MovementAuthorityPayload>(msg.payload));
  const auto& payload = std::get<MovementAuthorityPayload>(msg.payload);
  EXPECT_DOUBLE_EQ(payload.permittedDistance, 5000.0);
  EXPECT_EQ(payload.validUntilTick, 500u);
}

TEST(MessageTest, CreatesEmergencyBrakeMessageWithEmergencyPriority) {
  EmergencyBrakePayload data{.targetTrainId = 1,
                             .dangerZoneStart = 3000.0,
                             .dangerZoneEnd = 3500.0,
                             .reasonCode = 99};

  const auto msg = Message::makeEmergencyBrake(1003, 0, 1, 20, data);

  EXPECT_EQ(msg.header.type, MessageType::EmergencyBrake);
  EXPECT_EQ(msg.header.priority, MessagePriority::Emergency);

  ASSERT_TRUE(std::holds_alternative<EmergencyBrakePayload>(msg.payload));
}

TEST(MessageTest, CreatesHeartbeatBroadcastMessage) {
  const auto msg = Message::makeHeartbeat(1004, 5, 100);

  EXPECT_EQ(msg.header.type, MessageType::Heartbeat);
  EXPECT_EQ(msg.header.recipientId, kBroadcastRecipientId);
  EXPECT_EQ(msg.header.priority, MessagePriority::Low);

  ASSERT_TRUE(std::holds_alternative<HeartbeatPayload>(msg.payload));
  const auto& payload = std::get<HeartbeatPayload>(msg.payload);
  EXPECT_EQ(payload.senderId, 5u);
  EXPECT_TRUE(payload.isHealthy);
}
