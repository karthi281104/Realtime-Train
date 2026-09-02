# Module 7 — Communication Simulation

## 1. Architectural Role

Module 7 is the wireless vehicle-to-vehicle (V2V) and vehicle-to-infrastructure (V2I) communication simulation engine of TCAS.

It models realistic wireless network behavior (e.g. LTE-R / GSM-R / DSRC), including transmission latency, packet drop rates, wireless range attenuation, priority queuing, and broadcast / unicast mailbox routing.

```text
Train Telemetry / Movement Authority / Emergency Brake Commands
                                │
                                ▼
                      CommunicationChannel
             ├── Configurable Latency & Delay Propagation
             ├── Physical Distance & Range Filtering
             ├── Packet Loss & Drop Modulo Simulation
             └── Priority Mailbox Queue
                                │
                                ▼
                         Delivered Messages
             (Unicast to target TrainId or Broadcast to fleet)
```

---

## 2. Public API

### `tcas::communication::CommunicationChannel`

| Method | Description |
|---|---|
| `registerEntity(id)` / `unregisterEntity(id)` | Registers active train or infrastructure entity for broadcast distribution |
| `sendMessage(msg, senderPos, recipientPos)` | Transmits message; validates wireless range and packet loss |
| `step(currentTick)` | Advances channel time, moving delivered messages from in-flight to recipient mailboxes |
| `hasMessages(recipientId)` | Checks if recipient has pending delivered messages |
| `receiveMessages(recipientId)` | Retrieves and clears arrived messages for recipient |
| `deliveryRate()` / `totalSent()` / `totalDropped()` | Returns real-time transmission diagnostics and metrics |

### `tcas::communication::Message`

| Message Type | Factory Helper | Default Priority |
|---|---|---|
| `Telemetry` | `Message::makeTelemetry(...)` | `Normal` |
| `MovementAuthority` | `Message::makeMovementAuthority(...)` | `High` |
| `EmergencyBrake` | `Message::makeEmergencyBrake(...)` | `Emergency` |
| `Heartbeat` | `Message::makeHeartbeat(...)` | `Low` (Broadcast) |
