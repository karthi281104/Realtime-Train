#pragma once

#include "common/Types.hpp"

#include <cstdint>
#include <string>
#include <variant>

namespace tcas::communication
{

inline constexpr std::uint32_t kBroadcastRecipientId = 0xFFFFFFFF;

enum class MessageType
{
    Heartbeat,
    Telemetry,
    MovementAuthority,
    EmergencyBrake,
    ReservationRequest,
    ReservationResponse
};

enum class MessagePriority : std::uint8_t
{
    Low       = 0,
    Normal    = 1,
    High      = 2,
    Emergency = 3
};

struct MessageHeader
{
    std::uint64_t   messageId{ 0 };
    std::uint32_t   senderId{ 0 };
    std::uint32_t   recipientId{ kBroadcastRecipientId };
    SimTimeTick     sentTick{ 0 };
    SimTimeTick     deliveryTick{ 0 };
    MessagePriority priority{ MessagePriority::Normal };
    MessageType     type{ MessageType::Heartbeat };
};

struct TelemetryPayload
{
    TrainId        trainId{ 0 };
    DistanceMeters position{ 0.0 };
    SpeedMetersPerSecond velocity{ 0.0 };
    TrainState     state{ TrainState::Idle };
    TrackId        activeTrackId{ 0 };
};

struct MovementAuthorityPayload
{
    TrainId              trainId{ 0 };
    DistanceMeters       permittedDistance{ 0.0 };
    SpeedMetersPerSecond targetSpeed{ 0.0 };
    SimTimeTick          validUntilTick{ 0 };
};

struct EmergencyBrakePayload
{
    TrainId        targetTrainId{ 0 };
    DistanceMeters dangerZoneStart{ 0.0 };
    DistanceMeters dangerZoneEnd{ 0.0 };
    std::uint32_t  reasonCode{ 0 };
};

struct ReservationPayload
{
    TrainId       trainId{ 0 };
    TrackId       trackId{ 0 };
    SimTimeTick   startTick{ 0 };
    SimTimeTick   endTick{ 0 };
    bool          isGranted{ false };
};

struct HeartbeatPayload
{
    std::uint32_t senderId{ 0 };
    SimTimeTick   tick{ 0 };
    bool          isHealthy{ true };
};

using MessagePayload = std::variant<
    std::monostate,
    TelemetryPayload,
    MovementAuthorityPayload,
    EmergencyBrakePayload,
    ReservationPayload,
    HeartbeatPayload
>;

struct Message
{
    MessageHeader  header{};
    MessagePayload payload{};

    [[nodiscard]]
    static Message makeTelemetry(
        std::uint64_t msgId,
        std::uint32_t senderId,
        std::uint32_t recipientId,
        SimTimeTick tick,
        const TelemetryPayload& data,
        MessagePriority priority = MessagePriority::Normal
    ) noexcept;

    [[nodiscard]]
    static Message makeMovementAuthority(
        std::uint64_t msgId,
        std::uint32_t senderId,
        std::uint32_t recipientId,
        SimTimeTick tick,
        const MovementAuthorityPayload& data,
        MessagePriority priority = MessagePriority::High
    ) noexcept;

    [[nodiscard]]
    static Message makeEmergencyBrake(
        std::uint64_t msgId,
        std::uint32_t senderId,
        std::uint32_t recipientId,
        SimTimeTick tick,
        const EmergencyBrakePayload& data
    ) noexcept;

    [[nodiscard]]
    static Message makeHeartbeat(
        std::uint64_t msgId,
        std::uint32_t senderId,
        SimTimeTick tick
    ) noexcept;
};

} // namespace tcas::communication
