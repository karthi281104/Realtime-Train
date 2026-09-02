#include "communication/Message.hpp"

namespace tcas::communication {

Message Message::makeTelemetry(const std::uint64_t msgId,
                               const std::uint32_t senderId,
                               const std::uint32_t recipientId,
                               const SimTimeTick tick,
                               const TelemetryPayload& data,
                               const MessagePriority priority) noexcept {
  return Message{.header = {.messageId = msgId,
                            .senderId = senderId,
                            .recipientId = recipientId,
                            .sentTick = tick,
                            .deliveryTick = tick,
                            .priority = priority,
                            .type = MessageType::Telemetry},
                 .payload = data};
}

Message Message::makeMovementAuthority(
    const std::uint64_t msgId, const std::uint32_t senderId,
    const std::uint32_t recipientId, const SimTimeTick tick,
    const MovementAuthorityPayload& data,
    const MessagePriority priority) noexcept {
  return Message{.header = {.messageId = msgId,
                            .senderId = senderId,
                            .recipientId = recipientId,
                            .sentTick = tick,
                            .deliveryTick = tick,
                            .priority = priority,
                            .type = MessageType::MovementAuthority},
                 .payload = data};
}

Message Message::makeEmergencyBrake(
    const std::uint64_t msgId, const std::uint32_t senderId,
    const std::uint32_t recipientId, const SimTimeTick tick,
    const EmergencyBrakePayload& data) noexcept {
  return Message{.header = {.messageId = msgId,
                            .senderId = senderId,
                            .recipientId = recipientId,
                            .sentTick = tick,
                            .deliveryTick = tick,
                            .priority = MessagePriority::Emergency,
                            .type = MessageType::EmergencyBrake},
                 .payload = data};
}

Message Message::makeHeartbeat(const std::uint64_t msgId,
                               const std::uint32_t senderId,
                               const SimTimeTick tick) noexcept {
  return Message{.header = {.messageId = msgId,
                            .senderId = senderId,
                            .recipientId = kBroadcastRecipientId,
                            .sentTick = tick,
                            .deliveryTick = tick,
                            .priority = MessagePriority::Low,
                            .type = MessageType::Heartbeat},
                 .payload = HeartbeatPayload{
                     .senderId = senderId, .tick = tick, .isHealthy = true}};
}

}  // namespace tcas::communication
