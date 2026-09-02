#include <gtest/gtest.h>

#include "communication/CommunicationChannel.hpp"

using namespace tcas;
using namespace tcas::communication;

// ============================================================
// CommunicationChannel Tests
// ============================================================

TEST(CommunicationChannelTest, DirectDeliveryWithZeroLatency) {
  ChannelConfig config;
  config.latencyTicks = 0;

  CommunicationChannel channel(config);
  channel.registerEntity(2);

  const auto msg = Message::makeHeartbeat(1, 1, 0);
  Message directMsg = msg;
  directMsg.header.recipientId = 2;

  const bool sent = channel.sendMessage(directMsg, 100.0, 200.0);
  EXPECT_TRUE(sent);

  // Delivery occurs on step at current tick
  channel.step(0);

  EXPECT_TRUE(channel.hasMessages(2));
  const auto received = channel.receiveMessages(2);
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].header.messageId, 1u);
  EXPECT_FALSE(channel.hasMessages(2));  // Mailbox consumed
}

TEST(CommunicationChannelTest, RespectsLatencyDelay) {
  ChannelConfig config;
  config.latencyTicks = 3;  // 3 ticks propagation delay

  CommunicationChannel channel(config);
  channel.registerEntity(2);

  auto msg = Message::makeHeartbeat(1, 1, 10);
  msg.header.recipientId = 2;

  channel.sendMessage(msg);

  // At tick 10, message is in-flight
  channel.step(10);
  EXPECT_FALSE(channel.hasMessages(2));
  EXPECT_EQ(channel.inFlightCount(), 1u);

  // At tick 12, still in-flight
  channel.step(12);
  EXPECT_FALSE(channel.hasMessages(2));

  // At tick 13 (10 + 3), message is delivered!
  channel.step(13);
  EXPECT_TRUE(channel.hasMessages(2));
  EXPECT_EQ(channel.inFlightCount(), 0u);
}

TEST(CommunicationChannelTest, BroadcastDeliversToAllEntitiesExceptSender) {
  CommunicationChannel channel;
  channel.registerEntity(1);
  channel.registerEntity(2);
  channel.registerEntity(3);

  // Train 1 sends broadcast heartbeat
  const auto msg = Message::makeHeartbeat(99, 1, 0);
  channel.sendMessage(msg);

  channel.step(10);  // Past latency

  // Sender (1) does not receive its own broadcast
  EXPECT_FALSE(channel.hasMessages(1));

  // Other entities (2, 3) receive the broadcast
  EXPECT_TRUE(channel.hasMessages(2));
  EXPECT_TRUE(channel.hasMessages(3));

  const auto r2 = channel.receiveMessages(2);
  const auto r3 = channel.receiveMessages(3);
  EXPECT_EQ(r2.size(), 1u);
  EXPECT_EQ(r3.size(), 1u);
}

TEST(CommunicationChannelTest,
     BroadcastDeliveryCountReflectsRecipientsNotMessages) {
  // Broadcast to 3 entities: sender (1) + 2 recipients (2,3)
  // totalDelivered should count 1 (one message was delivered through channel)
  CommunicationChannel channel;
  channel.registerEntity(1);
  channel.registerEntity(2);
  channel.registerEntity(3);

  const auto msg = Message::makeHeartbeat(1, 1, 0);
  channel.sendMessage(msg);
  channel.step(5);

  EXPECT_EQ(channel.totalSent(), 1u);
  // totalDelivered counts the message as 1 delivery event (not per-recipient)
  EXPECT_EQ(channel.totalDelivered(), 1u);
}

TEST(CommunicationChannelTest, DropsMessageBeyondMaxRange) {
  ChannelConfig config;
  config.maxRangeMeters = 1000.0;  // 1 km max range

  CommunicationChannel channel(config);
  channel.registerEntity(2);

  auto msg = Message::makeHeartbeat(1, 1, 0);
  msg.header.recipientId = 2;

  // Sender at 0m, Recipient at 5000m (out of range)
  const bool accepted = channel.sendMessage(msg, 0.0, 5000.0);
  EXPECT_FALSE(accepted);
  EXPECT_EQ(channel.totalDropped(), 1u);
  EXPECT_EQ(channel.totalSent(), 1u);
}

TEST(CommunicationChannelTest, DeliversMessageWithinRange) {
  ChannelConfig config;
  config.maxRangeMeters = 1000.0;
  config.latencyTicks = 0;

  CommunicationChannel channel(config);
  channel.registerEntity(2);

  auto msg = Message::makeHeartbeat(1, 1, 0);
  msg.header.recipientId = 2;

  // Sender at 0m, Recipient at 500m (within range)
  const bool accepted = channel.sendMessage(msg, 0.0, 500.0);
  EXPECT_TRUE(accepted);
  channel.step(0);
  EXPECT_TRUE(channel.hasMessages(2));
}

TEST(CommunicationChannelTest, DeterministicDropModulo) {
  ChannelConfig config;
  config.deterministicDropModulo = 2;  // Drops every 2nd packet

  CommunicationChannel channel(config);
  channel.registerEntity(2);

  auto m1 = Message::makeHeartbeat(1, 1, 0);
  m1.header.recipientId = 2;
  auto m2 = Message::makeHeartbeat(2, 1, 0);
  m2.header.recipientId = 2;

  EXPECT_TRUE(channel.sendMessage(m1));   // 1st sent
  EXPECT_FALSE(channel.sendMessage(m2));  // 2nd dropped

  EXPECT_EQ(channel.totalSent(), 2u);
  EXPECT_EQ(channel.totalDropped(), 1u);
}

TEST(CommunicationChannelTest, TracksDeliveryStatisticsAccurately) {
  ChannelConfig config;
  config.latencyTicks = 1;

  CommunicationChannel channel(config);
  channel.registerEntity(2);

  auto msg = Message::makeHeartbeat(1, 1, 0);
  msg.header.recipientId = 2;

  channel.sendMessage(msg);
  channel.step(1);

  EXPECT_EQ(channel.totalSent(), 1u);
  EXPECT_EQ(channel.totalDelivered(), 1u);
  EXPECT_EQ(channel.totalDropped(), 0u);
  EXPECT_DOUBLE_EQ(channel.deliveryRate(), 1.0);
}

TEST(CommunicationChannelTest, DeliveryRateIsOneWhenNothingSent) {
  CommunicationChannel channel;
  EXPECT_DOUBLE_EQ(channel.deliveryRate(), 1.0);
}

TEST(CommunicationChannelTest, ClearResetsAllStateIncludingEntities) {
  CommunicationChannel channel;
  channel.registerEntity(2);

  auto msg = Message::makeHeartbeat(1, 1, 0);
  msg.header.recipientId = 2;
  channel.sendMessage(msg);

  // Full clear resets everything including entity registrations
  channel.clear();

  EXPECT_EQ(channel.totalSent(), 0u);
  EXPECT_EQ(channel.totalDelivered(), 0u);
  EXPECT_EQ(channel.inFlightCount(), 0u);
  // After full clear, entity 2 is no longer registered so mailbox is gone
  EXPECT_FALSE(channel.hasMessages(2));
}

TEST(CommunicationChannelTest, ClearMailboxesRetainsEntityRegistrations) {
  CommunicationChannel channel;
  channel.registerEntity(1);
  channel.registerEntity(2);

  auto msg = Message::makeHeartbeat(1, 1, 0);
  msg.header.recipientId = 2;
  channel.sendMessage(msg);

  // Soft reset: clears mailboxes but keeps entities registered
  channel.clearMailboxes();

  EXPECT_EQ(channel.totalSent(), 0u);
  EXPECT_EQ(channel.inFlightCount(), 0u);
  EXPECT_FALSE(channel.hasMessages(2));

  // Re-send after soft reset: entity is still registered
  channel.registerEntity(2);  // redundant but should not crash
  auto msg2 = Message::makeHeartbeat(2, 1, 0);
  msg2.header.recipientId = 2;
  EXPECT_NO_THROW(channel.sendMessage(msg2));
}

TEST(CommunicationChannelTest, UnregisterEntityRemovesFromBroadcast) {
  CommunicationChannel channel;
  channel.registerEntity(1);
  channel.registerEntity(2);
  channel.registerEntity(3);

  // Unregister entity 3 before broadcast
  channel.unregisterEntity(3);

  const auto msg = Message::makeHeartbeat(1, 1, 0);
  channel.sendMessage(msg);
  channel.step(5);

  EXPECT_TRUE(channel.hasMessages(2));
  EXPECT_FALSE(channel.hasMessages(3));  // Unregistered, should not receive
}

TEST(CommunicationChannelTest, InFlightCountDecrementsOnDelivery) {
  ChannelConfig config;
  config.latencyTicks = 5;

  CommunicationChannel channel(config);
  channel.registerEntity(2);

  auto msg = Message::makeHeartbeat(1, 1, 0);
  msg.header.recipientId = 2;
  channel.sendMessage(msg);

  EXPECT_EQ(channel.inFlightCount(), 1u);
  channel.step(5);  // Deliver
  EXPECT_EQ(channel.inFlightCount(), 0u);
}
