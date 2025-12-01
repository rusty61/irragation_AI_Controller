# AgriNet RF24 Communications Stack

A robust, low-level NRF24L01+ communications stack for a multi-node irrigation system with a two-tier RF24 tree topology.

## Network Topology

```
             UNO_Q MMC (RF24 master for clusters)
                       ▲   ▲   ▲
                       │   │   │
               ESP32 A  ESP32 B  ESP32 C   (cluster nodes)
                ▲  ▲       ▲  ▲       ▲  ▲
               /    \     /    \     /    \
            UNO…   UNO… UNO…  UNO… UNO…  UNO… (actuator nodes)
```

### Node Tiers

| Tier | Hardware | Role | Count |
|------|----------|------|-------|
| Top | Arduino UNO_Q MMC | Central master, system coordination | 1 |
| Middle | ESP32 | Cluster nodes, field hubs | Up to 16 |
| Bottom | Arduino UNO | Actuator nodes, valves, sensors | Up to 8 per cluster |

## Directory Structure

```
agrinet_rf24_comms/
├─ README.md                          # This file
├─ lib/
│  └─ agri_rf24/
│     ├─ agri_rf24_common.h           # Shared enums, structs, helpers
│     └─ agri_rf24_common.cpp         # RF24 init, send/recv, address helpers
└─ firmware/
   ├─ uno_node/
   │  └─ uno_node.ino                 # UNO actuator node firmware
   ├─ esp32_cluster/
   │  └─ esp32_cluster.ino            # ESP32 cluster node firmware
   └─ uno_q_mmc/
      └─ uno_q_mmc.ino                # UNO_Q master firmware
```

## RF24 Settings

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency Channel | 76 | 2.476 GHz |
| Data Rate | 250 kbps | Best range/reliability |
| PA Level | HIGH | Adjustable per environment |
| Auto-Ack | Enabled | Reliable delivery |
| Retry Delay | 5 × 250 µs = 1.25 ms | |
| Retry Count | 15 | Maximum retries |
| CRC | 2-byte (16-bit) | Error detection |
| Dynamic Payloads | Enabled | Variable packet sizes |
| Address Width | 5 bytes | Standard |

## Addressing Scheme

### Global Tier (UNO_Q ↔ ESP32)

The master communicates with cluster nodes using hierarchical addresses:

| Node | Address | Description |
|------|---------|-------------|
| UNO_Q Master | `"HQ000"` | Central master base address |
| Cluster 1 | `"HQA01"` | First cluster node |
| Cluster 2 | `"HQA02"` | Second cluster node |
| ... | ... | |
| Cluster 16 | `"HQA16"` | Sixteenth cluster node |

### Local Tier (ESP32 ↔ UNO)

Cluster nodes communicate with their UNO actuator nodes:

| Node | Address | Description |
|------|---------|-------------|
| Cluster 1, Node 1 | `"N1A01"` | First node in cluster 1 |
| Cluster 1, Node 2 | `"N1A02"` | Second node in cluster 1 |
| Cluster 2, Node 1 | `"N2A01"` | First node in cluster 2 |
| Cluster 10, Node 5 | `"NAA05"` | Fifth node in cluster 10 (hex) |

**Note:** Cluster IDs 10-16 use hexadecimal characters (A-G) in the address.

### Address Helper Functions

```cpp
// Get master address ("HQ000")
void agri_getMasterAddress(uint8_t address[6]);

// Get cluster address (e.g., "HQA01" for cluster 1)
void agri_getClusterAddress(uint8_t clusterId, uint8_t address[6]);

// Get UNO node address (e.g., "N1A02" for cluster 1, node 2)
void agri_getUnoNodeAddress(uint8_t clusterId, uint8_t nodeLocalId, uint8_t address[6]);
```

## Packet Protocol

### Common Header (8 bytes)

All packets start with an `AgriPacketHeader`:

```cpp
struct AgriPacketHeader {
  uint8_t   magic;      // 0xA5 - magic byte for validation
  uint8_t   msgType;    // Message type enum
  uint8_t   srcRole;    // Source role (UNO_NODE/ESP32_CLUSTER/UNO_Q_MMC)
  uint8_t   dstRole;    // Destination role
  uint8_t   srcId;      // Source node/cluster ID
  uint8_t   dstId;      // Destination node/cluster ID
  uint16_t  seq;        // Sequence number for duplicate detection
};
```

### Message Types

#### UNO ↔ ESP32 Messages

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| `UNO_STATUS` | 0x10 | UNO → ESP32 | Status report |
| `UNO_TELEMETRY` | 0x11 | UNO → ESP32 | Telemetry data |
| `UNO_ACK_CMD` | 0x12 | UNO → ESP32 | Command acknowledgment |
| `UNO_FAULT` | 0x13 | UNO → ESP32 | Fault report |
| `UNO_CMD_ACTUATOR` | 0x20 | ESP32 → UNO | Actuator command |
| `UNO_CMD_CONFIG` | 0x21 | ESP32 → UNO | Configuration update |
| `UNO_PING` | 0x22 | ESP32 → UNO | Ping/keepalive |

#### ESP32 ↔ UNO_Q Messages

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| `CLUSTER_TELEMETRY` | 0x30 | ESP32 → UNO_Q | Aggregated telemetry |
| `CLUSTER_STATUS` | 0x31 | ESP32 → UNO_Q | Cluster status |
| `CLUSTER_FAULT` | 0x32 | ESP32 → UNO_Q | Cluster fault report |
| `CLUSTER_HEARTBEAT` | 0x33 | ESP32 → UNO_Q | Heartbeat |
| `CLUSTER_SET_SCHED` | 0x40 | UNO_Q → ESP32 | Set schedule |
| `CLUSTER_CMD_ZONE` | 0x41 | UNO_Q → ESP32 | Zone command |
| `CLUSTER_CONFIG` | 0x42 | UNO_Q → ESP32 | Configuration |
| `CLUSTER_PING` | 0x43 | UNO_Q → ESP32 | Ping/keepalive |

## Payload Structures

### UNO Actuator Command

```cpp
struct AgriUnoActuatorCommand {
  uint8_t   zoneLocalId;    // Local zone identifier
  uint8_t   action;         // STOP(0) / OPEN(1) / CLOSE(2)
  uint32_t  runMs;          // Runtime in milliseconds
  uint16_t  cmdSeq;         // Command sequence number
};
```

### UNO Status

```cpp
struct AgriUnoStatus {
  uint8_t   zoneLocalId;    // Local zone identifier
  uint16_t  statusFlags;    // Status bitmask (see below)
  uint16_t  current_mA;     // Motor current in mA
  uint16_t  soilRaw;        // Raw ADC soil sensor reading
  uint8_t   soilPct;        // Soil moisture percentage (0-100)
  uint16_t  batt_mV;        // Battery voltage in mV
  uint16_t  lastCmdSeq;     // Last processed command sequence
};
```

**Status Flags:**

| Flag | Value | Description |
|------|-------|-------------|
| `STATUS_FLAG_OPEN` | 0x0001 | Valve is open |
| `STATUS_FLAG_CLOSED` | 0x0002 | Valve is closed |
| `STATUS_FLAG_MOVING` | 0x0004 | Valve is moving |
| `STATUS_FLAG_FAULT` | 0x0008 | Fault condition |
| `STATUS_FLAG_ENDSTOP` | 0x0010 | Endstop reached |
| `STATUS_FLAG_OVERCURR` | 0x0020 | Overcurrent detected |
| `STATUS_FLAG_TIMEOUT` | 0x0040 | Operation timeout |
| `STATUS_FLAG_COMMS_OK` | 0x0080 | Communications OK |

### Cluster Telemetry

```cpp
struct AgriZoneSnapshot {
  uint8_t   zoneId;         // Zone identifier
  uint8_t   zoneState;      // IDLE(0) / IRRIGATING(1) / FAULT(2)
  uint8_t   soilPct;        // Soil moisture percentage
  uint16_t  lastRun_s;      // Seconds since last irrigation
};

struct AgriClusterTelemetry {
  uint8_t   clusterId;      // Cluster identifier
  uint16_t  seq;            // Sequence number
  uint16_t  batt_mV;        // Battery voltage in mV
  uint16_t  panel_mV;       // Solar panel voltage in mV
  uint16_t  pumpCurrent_mA; // Pump current in mA
  uint8_t   numZones;       // Number of active zones
  AgriZoneSnapshot zones[8]; // Zone snapshots
};
```

### Cluster Schedule

```cpp
struct AgriZoneSchedule {
  uint8_t   zoneId;         // Zone identifier
  uint8_t   mode;           // OFF(0) / AUTO(1) / FORCE_ON(2) / FORCE_OFF(3)
  uint32_t  startEpoch_s;   // Start time (Unix epoch)
  uint32_t  duration_s;     // Duration in seconds
  uint8_t   priority;       // Schedule priority
};

struct AgriClusterSchedule {
  uint8_t   clusterId;      // Cluster identifier
  uint16_t  schedSeq;       // Schedule sequence number
  uint8_t   numEntries;     // Number of schedule entries
  AgriZoneSchedule entries[8]; // Schedule entries
};
```

## API Functions

### Initialization

```cpp
AgriResult agri_rf24_init_common(RF24 &radio, AgriRole role);
```

Initializes the RF24 radio with AgriNet settings for the specified role.

### Send/Receive

```cpp
AgriResult agri_rf24_send(RF24 &radio, const uint8_t *destAddr, 
                          const void *packet, uint8_t len);

AgriResult agri_rf24_recv(RF24 &radio, void *packet, uint8_t *len, 
                          uint16_t timeout_ms);
```

### Packet Helpers

```cpp
void agri_initPacketHeader(AgriPacketHeader *header, uint8_t msgType,
                           AgriRole srcRole, AgriRole dstRole,
                           uint8_t srcId, uint8_t dstId, uint16_t seq);

bool agri_validatePacketHeader(const AgriPacketHeader *header);
```

## Result Codes

```cpp
enum class AgriResult : uint8_t {
  OK          = 0,   // Operation successful
  TIMEOUT     = 1,   // Operation timed out
  RF_FAIL     = 2,   // RF24 hardware failure
  BAD_PACKET  = 3,   // Invalid packet (bad magic, CRC, etc.)
  DUPLICATE   = 4    // Duplicate packet received
};
```

## Safety Features

### UNO Actuator Nodes

1. **Maximum Runtime Cutoff**: Actuator automatically stops after configured maximum runtime (default 30 seconds)
2. **Direction Interlock**: Brief delay between direction changes prevents motor damage
3. **Failsafe on Comms Loss**: Auto-closes valve if no communication for 60 seconds
4. **Overcurrent Protection**: Stops actuator if current exceeds threshold

### ESP32 Cluster Nodes

1. **Node Offline Detection**: Marks UNO nodes offline after 30 seconds without contact
2. **Offline Status Reporting**: Includes offline node status in telemetry
3. **Local Fallback Schedule**: Maintains and executes local schedule if master connection lost

## Build Instructions

### Arduino IDE

1. Install the RF24 library by TMRh20:
   - Open Arduino IDE
   - Go to **Sketch → Include Library → Manage Libraries**
   - Search for "RF24" and install "RF24 by TMRh20"

2. Copy library files:
   - Copy `lib/agri_rf24/` folder to your Arduino libraries folder:
     - Windows: `Documents\Arduino\libraries\agri_rf24\`
     - macOS: `~/Documents/Arduino/libraries/agri_rf24/`
     - Linux: `~/Arduino/libraries/agri_rf24/`

3. Open the appropriate firmware:
   - For UNO nodes: `firmware/uno_node/uno_node.ino`
   - For ESP32 clusters: `firmware/esp32_cluster/esp32_cluster.ino`
   - For UNO_Q master: `firmware/uno_q_mmc/uno_q_mmc.ino`

4. Configure node settings at the top of each file:
   - `MY_CLUSTER_ID` - Cluster identifier
   - `MY_NODE_ID` - Node identifier (for UNO nodes)
   - Pin assignments

5. Select the appropriate board and upload.

### PlatformIO

1. Create a new PlatformIO project or add to existing

2. Add the RF24 library to `platformio.ini`:
   ```ini
   lib_deps = 
       nRF24/RF24@^1.4.8
   ```

3. Copy library files to `lib/agri_rf24/`

4. Configure firmware files in `src/`

5. Build and upload:
   ```bash
   pio run -t upload
   ```

## Required Libraries

| Library | Version | Source |
|---------|---------|--------|
| RF24 by TMRh20 | ≥ 1.4.0 | Arduino Library Manager / PlatformIO |

## Hardware Configuration

### Pin Connections

#### Arduino UNO / UNO_Q

| RF24 Pin | Arduino Pin |
|----------|-------------|
| VCC | 3.3V |
| GND | GND |
| CE | 9 |
| CSN | 10 |
| SCK | 13 |
| MOSI | 11 |
| MISO | 12 |

#### ESP32

| RF24 Pin | ESP32 Pin |
|----------|-----------|
| VCC | 3.3V |
| GND | GND |
| CE | 4 |
| CSN | 5 |
| SCK | 18 |
| MOSI | 23 |
| MISO | 19 |

## Debug Logging

Enable debug output by defining `RF24_DEBUG` before including the header:

```cpp
#define RF24_DEBUG
#include "agri_rf24_common.h"
```

Debug output includes:
- RF24 initialization status
- Send/receive operations
- Invalid packet drops
- Timeout events

## Test Instructions

### Basic Communication Test

1. Flash one device as UNO_Q master
2. Flash one device as ESP32 cluster (cluster ID 1)
3. Flash one device as UNO node (cluster ID 1, node ID 1)

4. Open serial monitors for all devices at 115200 baud

5. Verify:
   - Master receives heartbeats from cluster
   - Cluster receives status from UNO node
   - Master shows "ONLINE" status for cluster

### Command Test (via Master Serial)

```
STATUS                    # Show system status
PING 1                    # Ping cluster 1
ZONE 1 1 1 10000         # Open zone 1 in cluster 1 for 10 seconds
ZONE 1 1 2 5000          # Close zone 1 in cluster 1
ZONE 1 1 0               # Stop zone 1 in cluster 1
```

## Known Limitations

1. **Timing Synchronization**: Schedules require accurate time on each device. Consider adding RTC or NTP synchronization for production use.

2. **Single RF24 Module**: Each device has one RF24 module. Clusters must time-multiplex between uplink (to master) and downlink (to nodes).

3. **Packet Size**: Maximum payload is 32 bytes. Large schedules are sent in chunks.

4. **Range**: 250 kbps setting provides best range but may need external antenna or power amplifier for long distances.

5. **Collision Handling**: Relies on RF24 auto-retry. High-density deployments may need additional collision avoidance.

6. **Security**: No encryption implemented. Add AES encryption for production in unsecured environments.

## Scale Limits

| Metric | Maximum |
|--------|---------|
| Clusters per master | 16 |
| UNO nodes per cluster | 8 |
| Total UNO nodes | 128 |
| Zones per cluster | 8 |
| Schedule entries per cluster | 8 |

## License

This project is provided as-is for educational and development purposes.

## Contributing

Contributions are welcome. Please ensure:
- Code follows existing style
- New features include documentation
- Safety-critical changes are thoroughly tested
