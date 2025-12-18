
 // agrinet_rf24_comms/firmware/esp32_cluster_full/esp32_cluster_full.ino

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

/**
 * ESP32 Cluster Node – RF24 RX + TX
 *
 * - UNO actuator link:
 *   * Listens for UNO_STATUS from UNO on pipe "CL001"
 *   * Sends UNO_CMD_ACTUATOR to UNO on pipe "UN001"
 *   * Serial commands:
 *       'o' -> OPEN  5000 ms
 *       'c' -> CLOSE 5000 ms
 *       's' -> STOP  (runMs=0)
 *
 * - UNO_Q (MMC) link:
 *   * Uses AgriMsgType::CLUSTER_* messages
 *   * RX: CLUSTER_SET_SCHED from UNO_Q
 *   * TX: CLUSTER_TELEMETRY to UNO_Q via agri_rf24_sendTo()
 *
 * COMMS HEARTBEATS:
 * - UNO actuator online/offline remains based on UNO_STATUS only.
 * - UNO_Q online/offline based on:
 *     * Any RX from UNO_Q (cluster messages), OR
 *     * Any successful TX to UNO_Q (agri_rf24_sendTo == OK).
 */

// -----------------------------
// HW CONFIG - ESP32
// -----------------------------

static const uint8_t PIN_RF24_CE  = 4;
static const uint8_t PIN_RF24_CSN = 5;

// RF24 addresses – UNO actuator link (must match UNO side)
static const byte ADDR_UP[6]   = "CL001";   // UNO -> ESP32 (status)
static const byte ADDR_DOWN[6] = "UN001";   // ESP32 -> UNO (commands)

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// IDs
static const uint8_t CLUSTER_ID    = 1;  // this ESP32 "cluster"
static const uint8_t UNO_NODE_ID   = 1;  // target UNO actuator node
static const uint8_t ZONE_LOCAL_ID = 1;  // zone on that UNO

// -----------------------------
// LOCAL ZONE STATE (UNO ACTUATOR)
// -----------------------------

struct LocalZoneState {
  uint8_t  zoneId;         // local zone id
  uint8_t  zoneState;      // 0=IDLE, 1=IRRIGATING, 2=FAULT (not used yet)
  uint8_t  soilPct;        // 0-100
  uint32_t lastRunStartMs; // not used yet
  uint32_t lastSeenMs;     // last time we received any UNO status
  bool     online;         // derived from UNO status only
};

static LocalZoneState g_zone1;

static const uint32_t UNO_OFFLINE_TIMEOUT_MS = 30000;

// Outgoing sequence for packets to UNO
static uint16_t g_seq    = 0;
// Command sequence mirrored into UNO status lastCmdSeq
static uint16_t g_cmdSeq = 0;

// -----------------------------
// UNO_Q (MMC) COMMS STATE
// -----------------------------

// RF24 addresses for UNO_Q link (provided by agri_get* helpers)
static uint8_t g_addrMaster[6];   // UNO_Q MMC address
static uint8_t g_addrCluster[6];  // this cluster's address for UNO_Q to target

// COMMS HEARTBEAT – updated on RX and TX-ACK with UNO_Q
static uint32_t g_lastCommMs_unoQ     = 0;
static bool     g_unoQOnline          = false;
static const uint32_t UNOQ_TIMEOUT_MS = 60000UL;   // 60 s

// -----------------------------
// FORWARD DECLS
// -----------------------------

void handleRx();  // shared RX for UNO and UNO_Q
void handleUnoStatus(const AgriPacketHeader &hdr,
                     const AgriUnoStatus &status);
void handleClusterSchedule(const AgriPacketHeader &hdr,
                           const AgriClusterSchedule &sched);

void handleSerialCommands();
void sendActuatorCommand(uint8_t action, uint32_t runMs);

// Telemetry up to UNO_Q (stub, to be called when you have real telemetry)
bool sendClusterTelemetryToMaster(const AgriClusterTelemetry &tel);

// UNO_Q heartbeat helpers
static inline void unoQHeartbeatRx();
static inline void unoQHeartbeatTxAck();

// -----------------------------
// SETUP / LOOP
// -----------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n[ESP32_CLUSTER] Booting (UNO + UNO_Q RF24)"));

  g_zone1.zoneId         = 1;
  g_zone1.zoneState      = 0;
  g_zone1.soilPct        = 0;
  g_zone1.lastRunStartMs = 0;
  g_zone1.lastSeenMs     = 0;
  g_zone1.online         = false;

  g_unoQOnline      = false;
  g_lastCommMs_unoQ = millis();   // treat as "recent" at boot

  if (!radio.begin()) {
    Serial.println(F("[ESP32_CLUSTER] RF24 begin FAILED"));
  } else {
    Serial.println(F("[ESP32_CLUSTER] RF24 begin OK"));
  }

  // Match UNO RF24 settings
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_MAX); // Boost power so ACKs reach UNO_Q
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 15);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);

  // Configure addresses for UNO_Q link from helpers
  agri_getMasterAddress(g_addrMaster);
  agri_getClusterAddress(CLUSTER_ID, g_addrCluster);

  Serial.print(F("[ESP32] Cluster Addr: "));
  for(int i=0; i<5; i++) { Serial.print(g_addrCluster[i], HEX); Serial.print(" "); }
  Serial.println();

  // UNO actuator uplink + downlink
  radio.openReadingPipe(1, ADDR_UP);        // status from UNO
  radio.openWritingPipe(ADDR_DOWN);         // commands to UNO

  // UNO_Q link: Use Pipe 0 for full 5-byte address uniqueness
  // (Pipes 2-5 must share prefix with Pipe 1, which we don't match)
  radio.openReadingPipe(0, g_addrCluster);  

  radio.startListening();

  Serial.println(F("[ESP32_CLUSTER] Listening on pipe 1 for UNO \"CL001\""));
  Serial.println(F("[ESP32_CLUSTER] Listening on pipe 0 for UNO_Q cluster address"));
  Serial.println(F("[ESP32_CLUSTER] Serial commands: o=open 5s, c=close 5s, s=stop"));
}

void loop() {
  // RX: handle incoming packets from UNO actuator and UNO_Q MMC
  handleRx();

  // Serial: send actuator commands to UNO
  handleSerialCommands();

  uint32_t now = millis();

  // UNO actuator offline detection (unchanged semantics)
  if (g_zone1.online && (now - g_zone1.lastSeenMs > UNO_OFFLINE_TIMEOUT_MS)) {
    g_zone1.online = false;
    Serial.println(F("[ESP32_CLUSTER] UNO marked OFFLINE (no status)"));
  }

  // UNO_Q MMC offline detection (combined RX/TX-ACK heartbeat)
  if (g_unoQOnline && (now - g_lastCommMs_unoQ > UNOQ_TIMEOUT_MS)) {
    g_unoQOnline = false;
    Serial.println(F("[ESP32_CLUSTER] UNO_Q marked OFFLINE (no RX/TX-ACK)"));
  }
}

// -----------------------------
// RX HANDLER (UNO + UNO_Q)
// -----------------------------

void handleRx() {
  uint8_t pipe;
  while (radio.available(&pipe)) {
    uint8_t buffer[AGRI_RF24_PAYLOAD_MAX];
    uint8_t size = radio.getDynamicPayloadSize();
    if (size > sizeof(buffer)) {
      size = sizeof(buffer);
    }
    radio.read(buffer, size);

    Serial.print(F("[ESP32_CLUSTER] RX len="));
    Serial.print(size);
    Serial.print(F(" on pipe "));
    Serial.println(pipe);

    if (size < sizeof(AgriPacketHeader)) {
      Serial.println(F("[ESP32_CLUSTER] RX too short for header"));
      continue;
    }

    AgriPacketHeader *hdr = reinterpret_cast<AgriPacketHeader*>(buffer);

    if (hdr->magic != AGRI_MAGIC_BYTE) {
      Serial.print(F("[ESP32_CLUSTER] BAD magic: 0x"));
      Serial.println(hdr->magic, HEX);
      continue;
    }

    uint8_t *payload    = buffer + sizeof(AgriPacketHeader);
    uint8_t  payloadLen = size - sizeof(AgriPacketHeader);

    AgriMsgType msgType = static_cast<AgriMsgType>(hdr->msgType);
    AgriRole    srcRole = static_cast<AgriRole>(hdr->srcRole);
    AgriRole    dstRole = static_cast<AgriRole>(hdr->dstRole);

    switch (msgType) {
      case AgriMsgType::UNO_STATUS: {
        Serial.print(F("[ESP32_CLUSTER] payloadLen="));
        Serial.print(payloadLen);
        Serial.print(F(" expected="));
        Serial.println(sizeof(AgriUnoStatus));

        if (payloadLen < sizeof(AgriUnoStatus)) {
          Serial.println(F("[ESP32_CLUSTER] UNO_STATUS len too short"));
          break;
        }
        const AgriUnoStatus *st =
          reinterpret_cast<const AgriUnoStatus*>(payload);
        handleUnoStatus(*hdr, *st);
        break;
      }

      case AgriMsgType::CLUSTER_SET_SCHED: {
        // This should be sent by UNO_Q (MMC) to this cluster
        Serial.print(F("[ESP32_CLUSTER] CLUSTER_SET_SCHED payloadLen="));
        Serial.print(payloadLen);
        Serial.print(F(" expected>="));
        Serial.println(sizeof(AgriClusterSchedule));

        if (payloadLen < sizeof(AgriClusterSchedule)) {
          Serial.println(F("[ESP32_CLUSTER] CLUSTER_SET_SCHED len too short"));
          break;
        }

        // Heartbeat if it actually came from UNO_Q
        if (srcRole == AgriRole::UNOQ_MMC && dstRole == AgriRole::ESP32_CLUSTER) {
          unoQHeartbeatRx();
        }

        const AgriClusterSchedule *sched =
          reinterpret_cast<const AgriClusterSchedule*>(payload);
        handleClusterSchedule(*hdr, *sched);
        break;
      }

      // -----------------------------------------------------------------
      // NEW: Handle Cluster Zone Command from MMC
      // -----------------------------------------------------------------
      case AgriMsgType::CLUSTER_CMD_ZONE: {
        if (payloadLen < sizeof(AgriClusterZoneCmd)) {
          Serial.println(F("[ESP32_CLUSTER] CLUSTER_CMD_ZONE too short"));
          break;
        }

        // Heartbeat if it actually came from UNO_Q
        if (srcRole == AgriRole::UNOQ_MMC) {
          unoQHeartbeatRx();
        }

        const AgriClusterZoneCmd *cmd = reinterpret_cast<const AgriClusterZoneCmd*>(payload);
        
        Serial.print(F("[ESP32_CLUSTER] RX MMC CMD: Zone="));
        Serial.print(cmd->zoneId);
        Serial.print(F(" Mode="));
        Serial.print(cmd->mode);
        Serial.print(F(" Duration="));
        Serial.println(cmd->duration_s);

        // TRANSLATE TO ACTUATOR COMMAND (for Zone 1)
        // MMC Mode: 0=OFF, 1=AUTO, 2=FORCE_ON, 3=FORCE_OFF
        // Actuators: 1=OPEN, 2=CLOSE, 0=STOP
        
        uint8_t action = 0; // default stop
        uint32_t runMs = 0;

        if (cmd->mode == 1 || cmd->mode == 2) {
          // AUTO or FORCE_ON -> OPEN
          action = 1; // OPEN
          runMs = cmd->duration_s * 1000UL;
        } else if (cmd->mode == 0 || cmd->mode == 3) {
          // OFF or FORCE_OFF -> CLOSE
          action = 2; // CLOSE
          runMs = 30000UL; // Default 30s close time
        }

        if (action != 0) {
          Serial.println(F("   -> Forwarding to UNO Actuator..."));
          sendActuatorCommand(action, runMs);
        }
        break;
      }

      default:
        Serial.print(F("[ESP32_CLUSTER] Unknown msgType 0x"));
        Serial.println(static_cast<uint8_t>(msgType), HEX);
        break;
    }
  }
}

// -----------------------------
// HANDLE UNO STATUS (UNO ACTUATOR)
// -----------------------------

void handleUnoStatus(const AgriPacketHeader &hdr,
                     const AgriUnoStatus &status) {
  uint32_t now = millis();
  g_zone1.lastSeenMs = now;

  if (!g_zone1.online) {
    g_zone1.online = true;
    Serial.println(F("[ESP32_CLUSTER] UNO marked ONLINE (status RX)"));
  }

  g_zone1.soilPct = status.soilPct;

  Serial.print(F("[ESP32_CLUSTER] UNO_STATUS srcId="));
  Serial.print(hdr.srcId);
  Serial.print(F(" zone="));
  Serial.print(status.zoneLocalId);
  Serial.print(F(" soil="));
  Serial.print(status.soilPct);
  Serial.print(F("% batt="));
  Serial.print(status.batt_mV);
  Serial.print(F("mV current="));
  Serial.print(status.current_mA);
  Serial.print(F("mA flags=0x"));
  Serial.print(status.statusFlags, HEX);
  Serial.print(F(" lastCmdSeq="));
  Serial.println(status.lastCmdSeq);
}

// -----------------------------
// HANDLE CLUSTER SCHEDULE FROM UNO_Q
// -----------------------------

void handleClusterSchedule(const AgriPacketHeader &hdr,
                           const AgriClusterSchedule &sched) {
  Serial.print(F("[ESP32_CLUSTER] SCHED from srcRole="));
  Serial.print(hdr.srcRole);
  Serial.print(F(" srcId="));
  Serial.print(hdr.srcId);
  Serial.print(F(" clusterId="));
  Serial.print(sched.clusterId);
  Serial.print(F(" schedSeq="));
  Serial.print(sched.schedSeq);
  Serial.print(F(" numEntries="));
  Serial.println(sched.numEntries);

  for (uint8_t i = 0; i < sched.numEntries && i < AGRI_MAX_SCHEDULE_ENTRIES; ++i) {
    const AgriZoneSchedule &z = sched.entries[i];
    Serial.print(F("  [entry "));
    Serial.print(i);
    Serial.print(F("] zone="));
    Serial.print(z.zoneId);
    Serial.print(F(" mode="));
    Serial.print(z.mode);
    Serial.print(F(" start="));
    Serial.print(z.startEpoch_s);
    Serial.print(F(" dur="));
    Serial.print(z.duration_s);
    Serial.print(F(" priority="));
    Serial.println(z.priority);
  }

  // TODO: apply schedule to local zones / store in RAM
}

// -----------------------------
// SERIAL → ACTUATOR COMMANDS (UNO ACTUATOR)
// -----------------------------

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'o' || c == 'O') {
      Serial.println(F("[ESP32_CLUSTER] Serial: OPEN 5s"));
      sendActuatorCommand(static_cast<uint8_t>(AgriActAction::OPEN), 5000UL);
    } else if (c == 'c' || c == 'C') {
      Serial.println(F("[ESP32_CLUSTER] Serial: CLOSE 5s"));
      sendActuatorCommand(static_cast<uint8_t>(AgriActAction::CLOSE), 5000UL);
    } else if (c == 's' || c == 'S') {
      Serial.println(F("[ESP32_CLUSTER] Serial: STOP"));
      sendActuatorCommand(static_cast<uint8_t>(AgriActAction::STOP), 0UL);
    } else {
      Serial.print(F("[ESP32_CLUSTER] Unknown serial cmd: "));
      Serial.println(c);
    }
  }
}

// -----------------------------
// SEND ACTUATOR COMMAND TO UNO
// -----------------------------

void sendActuatorCommand(uint8_t action, uint32_t runMs) {
  struct {
    AgriPacketHeader       hdr;
    AgriUnoActuatorCommand cmd;
  } pkt;

  // Build header
  pkt.hdr.magic   = AGRI_MAGIC_BYTE;
  pkt.hdr.msgType = static_cast<uint8_t>(AgriMsgType::UNO_CMD_ACTUATOR);
  pkt.hdr.srcRole = static_cast<uint8_t>(AgriRole::ESP32_CLUSTER);
  pkt.hdr.dstRole = static_cast<uint8_t>(AgriRole::UNO_NODE);
  pkt.hdr.srcId   = CLUSTER_ID;
  pkt.hdr.dstId   = UNO_NODE_ID;
  pkt.hdr.seq     = ++g_seq;

  // Build command
  pkt.cmd.zoneLocalId = ZONE_LOCAL_ID;
  pkt.cmd.action      = action;
  pkt.cmd.runMs       = runMs;
  pkt.cmd.cmdSeq      = ++g_cmdSeq;

  uint8_t len = sizeof(pkt);

  Serial.print(F("[ESP32_CLUSTER] sendActuatorCommand len="));
  Serial.print(len);
  Serial.print(F(" action="));
  Serial.print(action);
  Serial.print(F(" runMs="));
  Serial.print(runMs);
  Serial.print(F(" cmdSeq="));
  Serial.println(pkt.cmd.cmdSeq);

  // TX burst: ensure writing pipe is UNO address, send, resume listening
  radio.stopListening();
  radio.openWritingPipe(ADDR_DOWN);  // keep UNO link correct even if UNO_Q changed the pipe
  
  bool ok = false;
  // Simple software retry (5 attempts, slightly longer backoff)
  for (int i=0; i<5; i++) {
    if (radio.write(&pkt, len)) {
      ok = true;
      break;
    }
    delay(10); // 10ms backoff
  }

  // RESTORE Pipe 0 for UNO_Q (because openWritingPipe overwrites Pipe 0 RX addr)
  radio.openReadingPipe(0, g_addrCluster);
  
  radio.startListening();

  if (!ok) {
    Serial.println(F("[ESP32_CLUSTER] sendActuatorCommand RF FAIL"));
  } else {
    Serial.println(F("[ESP32_CLUSTER] sendActuatorCommand OK"));
  }
}

// -----------------------------
// SEND CLUSTER TELEMETRY TO UNO_Q (MMC)
// -----------------------------

bool sendClusterTelemetryToMaster(const AgriClusterTelemetry &tel) {
  // Build into a raw buffer to avoid packed-struct / reference issues.
  uint8_t buf[AGRI_RF24_PAYLOAD_MAX];

  const uint8_t headerSize   = sizeof(AgriPacketHeader);
  const uint8_t payloadSize  = sizeof(AgriClusterTelemetry);
  const uint8_t totalSize    = headerSize + payloadSize;

  if (totalSize > AGRI_RF24_PAYLOAD_MAX) {
    Serial.println(F("[ESP32_CLUSTER] CLUSTER_TELEMETRY too large for RF24 payload"));
    return false;
  }

  AgriPacketHeader *hdr           = reinterpret_cast<AgriPacketHeader*>(buf);
  AgriClusterTelemetry *payload   =
    reinterpret_cast<AgriClusterTelemetry*>(buf + headerSize);

  agri_buildHeader(
    *hdr,
    AgriMsgType::CLUSTER_TELEMETRY,
    AgriRole::ESP32_CLUSTER,
    AgriRole::UNOQ_MMC,
    CLUSTER_ID,   // srcId
    0             // dstId (MMC id)
  );

  // Plain struct copy into payload
  *payload = tel;

  AgriResult res = agri_rf24_sendTo(
    radio,
    g_addrMaster,
    buf,
    totalSize
  );

  if (res == AgriResult::OK) {
    unoQHeartbeatTxAck();
    Serial.println(F("[ESP32_CLUSTER] CLUSTER_TELEMETRY sent OK"));
    return true;
  } else {
    Serial.print(F("[ESP32_CLUSTER] CLUSTER_TELEMETRY send FAIL res="));
    Serial.println(static_cast<uint8_t>(res));
    return false;
  }
}

// -----------------------------
// UNO_Q HEARTBEAT HELPERS
// -----------------------------

static inline void unoQHeartbeatRx() {
  uint32_t now = millis();
  g_lastCommMs_unoQ = now;
  if (!g_unoQOnline) {
    g_unoQOnline = true;
    Serial.println(F("[ESP32_CLUSTER] UNO_Q marked ONLINE (RX)"));
  }
}

static inline void unoQHeartbeatTxAck() {
  uint32_t now = millis();
  g_lastCommMs_unoQ = now;
  if (!g_unoQOnline) {
    g_unoQOnline = true;
    Serial.println(F("[ESP32_CLUSTER] UNO_Q marked ONLINE (TX-ACK)"));
  }
}
