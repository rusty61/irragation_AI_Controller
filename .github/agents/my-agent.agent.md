---
# Fill in the fields below to create a basic custom agent for your repository.
# The Copilot CLI can be used for local testing: https://gh.io/customagents/cli
# To make this agent available, merge this file into the default repository branch.
# For format details, see: https://gh.io/customagents/config

name:
description:
---

# My Agent

Describe what your agent does here...[AgriNet_RF24_Agent_Brief.md](https://github.com/user-attachments/files/23844674/AgriNet_RF24_Agent_Brief.md)
# AgriNet RF24 Communications – Agent Implementation Brief

## 0. Goal

Build a robust, low-level NRF24 communications stack for a multi-node irrigation system (AgriNet):

- Bottom: **UNO actuator nodes** (valves/actuators, safety logic).
- Middle: **ESP32 cluster nodes** (field hubs).
- Top: **Arduino UNO_Q MMC** (Qualcomm+STM32 hybrid) as **central master** and AI brain.

You (the agent) are responsible for **RF24 transport + packet protocol only** – *no AI logic, no irrigation decision logic, no cloud integration*.

The outcome should be a clean, reusable RF24 comms layer that the project owner can plug into their existing irrigation logic on each platform.

---

## 1. Hardware Roles & Network Topology

### 1.1 Node Types

1. **UNO Actuator Node (NRF24 Slave)**  
   - MCU: Arduino UNO (ATmega328P).  
   - Radio: NRF24L01(+), some with PA+LNA.  
   - Responsibilities:
     - Drive 1–N actuators (relays / H-bridges / linear actuators).
     - Read local sensors (limit switches, possibly soil sensor & rail/battery voltage).
     - Implement **hard safety**: max runtime, interlock, emergency close on comms loss.
   - RF Role: **Child/slave** to a single ESP32 cluster node.  
   - No AI, no global context – this is purely "muscle + safety".

2. **ESP32 Cluster Node (NRF Master to UNOs, NRF Child to UNO_Q)**  
   - MCU: ESP32.  
   - Radio: NRF24L01(+).  
   - Responsibilities:
     - Act as **local master** for 1–N UNO actuator nodes in a physical cluster/paddock.
     - Aggregate telemetry from its UNO children.
     - Send **aggregated telemetry** upwards to the UNO_Q MMC.
     - Receive **high-level schedules/commands** from UNO_Q and translate those into concrete commands for its UNO nodes.
     - Maintain **local fallback schedule** if UNO_Q is offline.
   - RF Role:
     - **Master** for its own UNO children.
     - **Child** of the UNO_Q MMC in the global tier.

3. **UNO_Q MMC (Global RF24 Master)**  
   - Board: Arduino UNO Q hybrid (Qualcomm QRB2210 + STM32U585).  
   - Radio: NRF24L01(+).  
   - Responsibilities:
     - **Global RF24 master** for all ESP32 cluster nodes.
     - Terminates all RF24 traffic from ESP32 nodes.
     - For this brief: expose incoming RF24 messages to its host (Linux side) and accept outbound messages from the host to send to clusters.
     - Higher-level AI and cloud logic are outside this brief but must be easy to connect to (e.g., via UART or similar).

### 1.2 Network Topology

Two-tier RF24 tree:

```text
             UNO_Q MMC (RF24 master for clusters)
                       ▲   ▲   ▲
                       │   │   │
               ESP32 A  ESP32 B  ESP32 C   (cluster nodes)
                ▲  ▲       ▲  ▲       ▲  ▲
               /    \     /    \     /    \
            UNO…   UNO… UNO…  UNO… UNO…  UNO… (actuator nodes)
```

- Each **UNO** binds to exactly **one ESP32**.
- Each **ESP32** binds to exactly **one UNO_Q**.

The transport between each tier is NRF24. No Wi-Fi is used for these RF24 links.

---

## 2. RF24 Settings & General Constraints

### 2.1 Base RF24 Settings

Use the TMRh20 RF24 library (C++). Baseline defaults:

- Frequency channel: `76` (2.476 GHz) – configurable constant.
- Data rate: `250 kbps` (for range and robustness).
- Auto-ack: **enabled**.
- Retries: enabled with sane defaults (e.g., delay `5` × 250 µs, count `15`).
- CRC: 2-byte.
- Dynamic payloads: **enabled**.

### 2.2 Design Constraints

- Prioritise **reliability and determinism** over raw throughput.
- Expected scale:
  - UNO_Q: up to ~16 cluster nodes (ESP32).
  - Each ESP32: up to ~8 UNO actuator nodes.
- The protocol must allow:
  - Detection of lost commands.
  - Detection of stale telemetry and missing nodes.
  - Basic RF health visibility (timeouts / retry failures).

---

## 3. Addressing & Pipes

You must implement a clear RF24 addressing scheme that is:

- Collision-free.
- Human-debuggable.
- Scalable to the expected node counts.

### 3.1 Address Concept

We use 5-byte addresses, derived from logical IDs.

#### Global tier (UNO_Q ↔ ESP32):

- UNO_Q listens as master on a **base address**, e.g. `"HQ000"`.
- Each ESP32 cluster has a unique address, e.g.:
  - Cluster 1: `"HQA01"`
  - Cluster 2: `"HQA02"`
  - ...
- UNO_Q must be able to:
  - Receive telemetry from all clusters, and
  - Send targeted commands/schedules to a specific cluster.

#### Local tier (ESP32 ↔ UNO):

- Each ESP32 defines a local base namespace.
- UNO actuator nodes under a cluster have unique addresses derived from:
  - `clusterId` (1..N)
  - `nodeLocalId` (1..N under that cluster)
- Example pattern (can be refined by you but must be systematic):
  - `"N1A01"`, `"N1A02"`, ... for nodes under cluster `1`.

### 3.2 Helper Functions

The project exposes address helper functions for you to implement:

```cpp
void agri_getClusterAddress(uint8_t clusterId, uint8_t address[6]);
void agri_getUnoNodeAddress(uint8_t clusterId, uint8_t nodeLocalId, uint8_t address[6]);
```

You must ensure these mapping functions:

- Map consistently from logical IDs → 5-byte RF24 addresses.
- Are documented so future developers can understand the mapping.

---

## 4. Packet Design & Protocol

You own the **binary packet layout and RF24 state machines**, within the provided struct scaffolding.

Each packet must start with a common header: `AgriPacketHeader`, followed by a payload struct appropriate for the message type.

### 4.1 Common Header

```cpp
struct AgriPacketHeader {
  uint8_t   magic;      // constant, e.g. 0xA5
  uint8_t   msgType;    // AgriMsgType enum
  uint8_t   srcRole;    // AgriRole enum
  uint8_t   dstRole;    // AgriRole enum
  uint8_t   srcId;      // node or cluster ID
  uint8_t   dstId;      // node or cluster ID
  uint16_t  seq;        // sequence number (per-sender)
};
```

- `magic` is a fixed byte (0xA5) used to quickly discard garbage.
- `msgType` indicates which payload struct follows.
- `srcRole`/`dstRole` differentiate UNO/ESP32/UNO_Q.
- `seq` lets us detect duplicates and track order.

All packing/alignment issues are your responsibility; you may use `#pragma pack` or manual serialization, but the logical fields and types must be preserved.

### 4.2 Message Type Enums

`AgriMsgType` (non-exhaustive, defined in code):

- UNO ↔ ESP32 (local tier):
  - `UNO_STATUS`
  - `UNO_TELEMETRY`
  - `UNO_ACK_CMD`
  - `UNO_FAULT`
  - `UNO_CMD_ACTUATOR`
  - `UNO_CMD_CONFIG`
  - `UNO_PING`
- ESP32 ↔ UNO_Q (global tier):
  - `CLUSTER_TELEMETRY`
  - `CLUSTER_STATUS`
  - `CLUSTER_FAULT`
  - `CLUSTER_HEARTBEAT`
  - `CLUSTER_SET_SCHED`
  - `CLUSTER_CMD_ZONE`
  - `CLUSTER_CONFIG`
  - `CLUSTER_PING`

### 4.3 UNO ↔ ESP32 Payloads

#### Downlink – Actuator command (`UNO_CMD_ACTUATOR`)

```cpp
struct AgriUnoActuatorCommand {
  uint8_t   zoneLocalId;   // 1..N for this cluster
  uint8_t   action;        // AgriActAction: STOP/OPEN/CLOSE
  uint32_t  runMs;         // requested run duration in ms
  uint16_t  cmdSeq;        // command sequence token
};
```

#### Uplink – Status/telemetry (`UNO_STATUS` / `UNO_TELEMETRY`)

```cpp
struct AgriUnoStatus {
  uint8_t   zoneLocalId;
  uint16_t  statusFlags;   // OPEN/CLOSED/MOVING/FAULT/ENDSTOP (bitmask)
  uint16_t  current_mA;    // actuator current, if available
  uint16_t  soilRaw;       // raw ADC soil value (optional)
  uint8_t   soilPct;       // 0..100 scaled soil moisture (optional)
  uint16_t  batt_mV;       // local battery/rail voltage (optional)
  uint16_t  lastCmdSeq;    // echoes last command seq processed
};
```

**Requirements:**

- Every `UNO_CMD_ACTUATOR` must ultimately produce an `UNO_ACK_CMD` or a `UNO_STATUS` reflecting success/failure of the action.
- UNO must implement:
  - Max runtime safety.
  - Actuator direction interlock.
  - Failsafe behaviour on comms loss (auto-close or safe state).

You must ensure the RF24 layer makes it trivial for the higher level on ESP32 to know:

- Whether a command was delivered (write success with ACK).
- Whether the command was executed (status + `lastCmdSeq` matches).

### 4.4 ESP32 ↔ UNO_Q Payloads

#### Uplink – Cluster telemetry (`CLUSTER_TELEMETRY`)

```cpp
#define AGRI_MAX_ZONES_PER_CLUSTER 8

struct AgriZoneSnapshot {
  uint8_t   zoneId;        // global or per-cluster
  uint8_t   zoneState;     // AgriZoneState: IDLE/IRRIGATING/FAULT
  uint8_t   soilPct;
  uint16_t  lastRun_s;     // last irrigation duration in seconds
};

struct AgriClusterTelemetry {
  uint8_t   clusterId;
  uint16_t  seq;
  uint16_t  batt_mV;
  uint16_t  panel_mV;
  uint16_t  pumpCurrent_mA;
  uint8_t   numZones;
  AgriZoneSnapshot zones[AGRI_MAX_ZONES_PER_CLUSTER];
};
```

#### Downlink – Cluster schedule (`CLUSTER_SET_SCHED`)

```cpp
#define AGRI_MAX_SCHEDULE_ENTRIES 8

struct AgriZoneSchedule {
  uint8_t   zoneId;
  uint8_t   mode;          // 0=OFF,1=AUTO,2=FORCE_ON,3=FORCE_OFF
  uint32_t  startEpoch_s;  // start time (epoch seconds or defined relative)
  uint32_t  duration_s;
  uint8_t   priority;      // 0..2
};

struct AgriClusterSchedule {
  uint8_t   clusterId;
  uint16_t  schedSeq;      // schedule version
  uint8_t   numEntries;
  AgriZoneSchedule entries[AGRI_MAX_SCHEDULE_ENTRIES];
};
```

**Requirements:**

- Schedules must be versioned (`schedSeq`).
- ESP32 must acknowledge new schedules and:
  - Store the latest one.
  - Reject older versions or at least ignore them safely.
- Higher-level logic will produce the actual schedules; you only handle:
  - Transport and parsing.
  - Passing them to the ESP32’s local scheduler logic (which is not part of this brief).

---

## 5. Reliability, State Machines & Timing

You must implement RF24 logic that:

1. Uses **auto-ack** and configured retries.
2. Provides blocking or timeout-aware wrappers for send/receive:
   - `AgriResult agri_rf24_send(...)`
   - `AgriResult agri_rf24_recv(...)`

### 5.1 Sequence Handling

- Each sender (UNO / ESP32 / UNO_Q) maintains a local `seq` counter.
- Receivers can detect duplicate packets via `seq` and ignore them.
- Commands include a `cmdSeq` token that can be echoed back to confirm the action.

### 5.2 Telemetry & Heartbeats

- UNO nodes:
  - Send status/telemetry periodically (e.g. every 2–10 seconds) and on state changes.
- ESP32 clusters:
  - Aggregate data and send `CLUSTER_TELEMETRY` frames to UNO_Q regularly.
  - Send `CLUSTER_HEARTBEAT` frames even if no state changed (to detect silent nodes).

### 5.3 Loss Handling

- If an ESP32 loses RF contact with a UNO:
  - Mark that node as offline.
  - Report the offline state in `CLUSTER_STATUS` or `CLUSTER_TELEMETRY` to UNO_Q.
- If UNO_Q loses an ESP32 (no heartbeat/telemetry):
  - UNO_Q flags that cluster as offline.
- UNO and ESP32 must still follow local safety rules and fallback schedules if higher-tier nodes disappear.

You must provide **clear, documented state machines** for:

- UNO RF behaviour (receive commands, send status, handle timeouts).
- ESP32 RF behaviour (both local and global tiers).
- UNO_Q RF behaviour (cluster polling / listening, schedule broadcasting).

---

## 6. Error Handling & Diagnostics

Define and implement `AgriResult` codes, e.g.:

```cpp
enum class AgriResult : uint8_t {
  OK          = 0,
  TIMEOUT     = 1,
  RF_FAIL     = 2,
  BAD_PACKET  = 3,
  DUPLICATE   = 4
};
```

You must:

- Return sensible `AgriResult` values from init / send / receive helpers.
- Implement **optional debug logging** guarded by `#define RF24_DEBUG`:
  - Print when:
    - RF24 init fails.
    - A packet is dropped as invalid.
    - A timeout occurs waiting for available data.
    - Retries or writes fail.

Debug logs must be concise but sufficient for troubleshooting.

---

## 7. Security (Minimal but Non-Brain-Dead)

This is a farm RF link, not a bank, but we do want basic protection from random interference.

Minimum requirements:

- Use non-trivial addresses, not RF24 defaults.
- Include a `magic` byte in the header and drop packets with incorrect `magic`.
- Reserve room for future lightweight signing / token in the header (not implemented now).

---

## 8. Public API Expectations per Platform

You must provide clean APIs for each node type using the shared header `agri_rf24_common.h` and the stubs supplied.

### 8.1 UNO Actuator Node

Expectations (example functions to support in the sketch):

- Initialization:
  - `AgriResult agri_rf24_init_common(RF24 &radio, AgriRole::UNO_NODE);`
- In the main loop:
  - Non-blocking polling of RF for commands.
  - Periodic sending of `AgriUnoStatus`.

You will fill in the `TODO` sections in `firmware/uno_node/uno_node.ino` to:

- Configure RX/TX pipes correctly.
- Parse incoming `UNO_CMD_ACTUATOR` and apply them to the actuator logic.
- Build and send `UNO_STATUS` frames periodically and on state changes.

### 8.2 ESP32 Cluster Node

Expectations:

- Initialization via `agri_rf24_init_common` with `AgriRole::ESP32_CLUSTER`.
- Support for both:
  - Listening to UNO nodes (local tier).
  - Listening/sending to UNO_Q (global tier).

You will:

- Implement logic to distinguish packets from UNO vs UNO_Q based on addresses and/or roles in the header.
- Implement handler functions, e.g.:
  - `onUnoStatusReceived(...)`
  - `onClusterScheduleReceived(...)`
- Periodically send `AgriClusterTelemetry` to UNO_Q using the provided structs.

### 8.3 UNO_Q MMC

Expectations:

- Initialization via `agri_rf24_init_common` with `AgriRole::UNOQ_MMC`.
- Listen for cluster telemetry from ESP32 nodes.
- Forward telemetry to the Linux host (QRB2210) via a simple IPC (e.g. UART) – the IPC details are **out of scope**, but your code must be structured so adding this is easy.
- Accept schedule data (for now we can fake it in the sketch) and send `AgriClusterSchedule` messages down to clusters.

You will fill in `firmware/uno_q_mmc/uno_q_mmc.ino` so that:

- All clusters in `CLUSTER_IDS[]` are addressed correctly.
- Incoming `CLUSTER_TELEMETRY` is parsed and passed to a stub callback.
- Outgoing `CLUSTER_SET_SCHED` is sent with proper headers and sequences.

---

## 9. Repo Layout & Supplied Stubs

The project owner provides this template layout:

```text
agrinet_rf24_comms/
├─ README.md                  # This brief
├─ lib/
│  └─ agri_rf24/
│     ├─ agri_rf24_common.h   # Shared enums, structs, helpers
│     └─ agri_rf24_common.cpp # Shared RF24 init + basic send/recv + address helpers
└─ firmware/
   ├─ uno_node/
   │  └─ uno_node.ino         # UNO actuator node RF24 stub
   ├─ esp32_cluster/
   │  └─ esp32_cluster.ino    # ESP32 cluster node RF24 stub
   └─ uno_q_mmc/
      └─ uno_q_mmc.ino        # UNO_Q master RF24 stub
```

You must:

- Implement the RF24 parts inside this structure.
- Keep all public APIs and struct definitions intact unless changes are agreed explicitly.

---

## 10. Testing Requirements

You must provide evidence (code + brief notes) that the following scenarios work:

1. **UNO ↔ ESP32, single node**
   - Command open/close sent from ESP32 to UNO.
   - UNO sends status back reflecting actuator behaviour.
   - RF loss simulated (e.g., remove power on UNO) and system reacts with safe behaviour.

2. **ESP32 ↔ UNO_Q, single cluster**
   - ESP32 sends periodic `CLUSTER_TELEMETRY`.
   - UNO_Q sends `CLUSTER_SET_SCHED` and ESP32 accepts it.

3. **Multiple UNOs under one ESP32**
   - Two or more UNO nodes operating simultaneously.
   - No mix-up of node IDs or addresses.
   - Commands/telemetry correctly routed per node.

4. **Multiple ESP32 clusters**
   - At least two ESP32 clusters active.
   - UNO_Q can distinguish and address each.
   - Telemetry from both is received and decoded.

5. **Loss/recovery scenarios**
   - UNO node disappears mid-operation; ESP32 marks it offline.
   - ESP32 disappears; UNO_Q marks cluster offline.
   - Nodes recover and rejoin properly when power is restored.

---

## 11. Deliverables

You must deliver:

1. **Source code**:
   - Completed implementations in:
     - `lib/agri_rf24/agri_rf24_common.cpp` (address helpers, send/recv, any glue).
     - `firmware/uno_node/uno_node.ino` (RF24 side completed).
     - `firmware/esp32_cluster/esp32_cluster.ino` (RF24 side completed).
     - `firmware/uno_q_mmc/uno_q_mmc.ino` (RF24 side completed).
2. **Documentation**:
   - Any additional notes needed to explain:
     - Final address scheme.
     - Any deviations from this brief.
     - How to run your test sketches.
3. **Build & test instructions**:
   - Arduino/PlatformIO board definitions used.
   - Any special libraries and their versions (RF24, etc.).
   - Steps to compile and flash each firmware target.
4. **Known limitations**:
   - Explicitly list anything not handled (e.g., no encryption, assumed max node counts).
   - Any edge cases not covered.

This brief is the canonical specification for your work on the RF24 layer.  
Do not modify public struct definitions or high-level architecture without prior agreement.
