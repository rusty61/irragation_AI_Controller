##include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

// -----------------------------
// HW CONFIG - ADJUST
// RF24 pins for ESP32 (per README)
// -----------------------------

static const uint8_t PIN_RF24_CE  = 4;
static const uint8_t PIN_RF24_CSN = 5;

// This cluster's ID
static const uint8_t CLUSTER_ID = 1;

// UNO nodes under this cluster
// Adjust to match your actual actuator nodes
static const uint8_t UNO_NODE_IDS[] = {1, 2};  // example: 2 nodes
static const uint8_t NUM_UNO_NODES  = sizeof(UNO_NODE_IDS) / sizeof(UNO_NODE_IDS[0]);

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// -----------------------------
// TIMING CONSTANTS
// -----------------------------

static const uint32_t TELEMETRY_PERIOD_MS     = 5000;   // uplink to UNO_Q
static const uint32_t UNO_OFFLINE_TIMEOUT_MS  = 30000;  // mark UNO offline if no status for 30s

uint32_t g_lastTelemetryMs = 0;

// -----------------------------
// LOCAL STATE PER UNO NODE
// -----------------------------

struct LocalZoneState {
  uint8_t  zoneId;         // matches UNO_NODE_IDS[i]
  uint8_t  zoneState;      // 0=IDLE, 1=IRRIGATING, 2=FAULT
  uint8_t  soilPct;        // 0-100
  uint32_t lastRunStartMs; // when we last saw it "moving"
  uint32_t lastSeenMs;     // last time we received any status
  bool     online;         // derived from lastSeenMs
};

static LocalZoneState g_zones[NUM_UNO_NODES];

// -----------------------------
// FORWARD DECLS
// -----------------------------

void initLocalZones();
void handleRx();
void handleUnoStatus(const AgriPacketHeader &hdr,
                     const AgriUnoStatus &status,
                     uint8_t payloadLen);
void handleMasterPackets(const AgriPacketHeader &hdr,
                         const uint8_t *payload,
                         uint8_t payloadLen);
void updateOnlineFlags(uint32_t now);
void sendClusterTelemetry();

// -----------------------------
// SETUP / LOOP
// -----------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n[ESP32_CLUSTER] Booting"));

  initLocalZones();

  AgriResult res = agri_rf24_init_common(
    radio,
    AgriRole::ESP32_CLUSTER,
    CLUSTER_ID,
    0
  );

  if (res != AgriResult::OK) {
    Serial.println(F("[ESP32_CLUSTER] RF24 init failed"));
  } else {
    Serial.println(F("[ESP32_CLUSTER] RF24 init OK"));
  }

  // TODO:
  //  - additional pipes for UNO nodes vs UNO_Q, if you want multi-pipe layout.
  //  - more detailed per-UNO state machines.
}

void loop() {
  uint32_t now = millis();

  // Handle all incoming packets (from UNO nodes and UNO_Q)
  handleRx();

  // Update online/offline flags based on lastSeen timestamps
  updateOnlineFlags(now);

  // Periodic telemetry up to UNO_Q
  if (now - g_lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    sendClusterTelemetry();
    g_lastTelemetryMs = now;
  }

  // TODO:
  //  - resend commands with retry windows
  //  - apply local schedules if master link is down
}

// -----------------------------
// LOCAL ZONE INIT
// -----------------------------

void initLocalZones() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < NUM_UNO_NODES; ++i) {
    g_zones[i].zoneId         = UNO_NODE_IDS[i];
    g_zones[i].zoneState      = 0;  // IDLE
    g_zones[i].soilPct        = 0;
    g_zones[i].lastRunStartMs = 0;
    g_zones[i].lastSeenMs     = 0;
    g_zones[i].online         = false;
  }
  (void)now; // quiet unused warning if not used
}

// -----------------------------
// RX DISPATCH
// -----------------------------

void handleRx() {
  while (radio.available()) {
    uint8_t buffer[AGRI_RF24_PAYLOAD_MAX];
    uint8_t size = radio.getDynamicPayloadSize();
    radio.read(buffer, size);

    if (size < sizeof(AgriPacketHeader)) {
      Serial.println(F("[ESP32_CLUSTER] RX too short"));
      continue;
    }

    AgriPacketHeader *hdr = reinterpret_cast<AgriPacketHeader*>(buffer);
    if (hdr->magic != AGRI_MAGIC_BYTE) {
      Serial.println(F("[ESP32_CLUSTER] BAD magic"));
      continue;
    }

    uint8_t *payload    = buffer + sizeof(AgriPacketHeader);
    uint8_t  payloadLen = size - sizeof(AgriPacketHeader);

    AgriRole srcRole = static_cast<AgriRole>(hdr->srcRole);
    AgriMsgType msgType = static_cast<AgriMsgType>(hdr->msgType);

    if (srcRole == AgriRole::UNO_NODE) {
      // UNO → ESP32 (status/telemetry/ack/fault)
      if (msgType == AgriMsgType::UNO_STATUS) {
        if (payloadLen < sizeof(AgriUnoStatus)) {
          Serial.println(F("[ESP32_CLUSTER] UNO_STATUS len too short"));
          continue;
        }
        const AgriUnoStatus *st =
          reinterpret_cast<const AgriUnoStatus*>(payload);
        handleUnoStatus(*hdr, *st, payloadLen);
      } else {
        // For now, just log other UNO message types
        Serial.print(F("[ESP32_CLUSTER] From UNO msgType=0x"));
        Serial.println(hdr->msgType, HEX);
      }
    } else if (srcRole == AgriRole::UNOQ_MMC) {
      // UNO_Q → ESP32 (schedule / zone cmds / ping / config)
      handleMasterPackets(*hdr, payload, payloadLen);
    } else {
      Serial.print(F("[ESP32_CLUSTER] Packet from unknown srcRole="));
      Serial.println(static_cast<uint8_t>(srcRole));
    }
  }
}

// -----------------------------
// HANDLE UNO STATUS
// -----------------------------

static int8_t findZoneIndex(uint8_t zoneLocalId) {
  for (uint8_t i = 0; i < NUM_UNO_NODES; ++i) {
    if (g_zones[i].zoneId == zoneLocalId) {
      return i;
    }
  }
  return -1;
}

void handleUnoStatus(const AgriPacketHeader &hdr,
                     const AgriUnoStatus &status,
                     uint8_t /*payloadLen*/) {
  uint32_t now = millis();
  int8_t idx = findZoneIndex(status.zoneLocalId);
  if (idx < 0) {
    Serial.print(F("[ESP32_CLUSTER] UNO_STATUS from unknown zoneLocalId="));
    Serial.println(status.zoneLocalId);
    return;
  }

  LocalZoneState &z = g_zones[idx];

  z.lastSeenMs = now;
  z.online     = true;
  z.soilPct    = status.soilPct;

  // Derive zoneState from flags:
  //  - FAULT, then zoneState=2
  //  - MOVING, then zoneState=1
  //  - else IDLE (0)
  if (status.statusFlags & STATUS_FLAG_FAULT) {
    z.zoneState = 2; // FAULT
  } else if (status.statusFlags & STATUS_FLAG_MOVING) {
    if (z.zoneState != 1) {
      // just transitioned into IRRIGATING
      z.lastRunStartMs = now;
    }
    z.zoneState = 1; // IRRIGATING
  } else {
    z.zoneState = 0; // IDLE
  }

  Serial.print(F("[ESP32_CLUSTER] UNO_STATUS cluster="));
  Serial.print(hdr.srcId);
  Serial.print(F(" zone="));
  Serial.print(status.zoneLocalId);
  Serial.print(F(" soil="));
  Serial.print(status.soilPct);
  Serial.print(F("% flags=0x"));
  Serial.println(status.statusFlags, HEX);
}

// -----------------------------
// HANDLE MASTER PACKETS (UNO_Q)
// -----------------------------

void handleMasterPackets(const AgriPacketHeader &hdr,
                         const uint8_t *payload,
                         uint8_t payloadLen) {
  (void)payload;
  (void)payloadLen;

  AgriMsgType msgType = static_cast<AgriMsgType>(hdr->msgType);

  switch (msgType) {
    case AgriMsgType::CLUSTER_SET_SCHED:
      // Full AgriClusterSchedule payload – for now, just log.
      Serial.println(F("[ESP32_CLUSTER] CLUSTER_SET_SCHED received (stub)"));
      break;

    case AgriMsgType::CLUSTER_CMD_ZONE:
      // Future: decode a per-zone command and send UNO_CMD_ACTUATOR to target UNO.
      Serial.println(F("[ESP32_CLUSTER] CLUSTER_CMD_ZONE received (stub)"));
      break;

    case AgriMsgType::CLUSTER_PING:
      Serial.println(F("[ESP32_CLUSTER] CLUSTER_PING received"));
      // Could respond with CLUSTER_HEARTBEAT here if desired.
      break;

    default:
      Serial.print(F("[ESP32_CLUSTER] From UNO_Q msgType=0x"));
      Serial.println(hdr->msgType, HEX);
      break;
  }
}

// -----------------------------
// ONLINE/OFFLINE FLAG UPDATE
// -----------------------------

void updateOnlineFlags(uint32_t now) {
  for (uint8_t i = 0; i < NUM_UNO_NODES; ++i) {
    LocalZoneState &z = g_zones[i];
    if (!z.online) continue;

    if (now - z.lastSeenMs > UNO_OFFLINE_TIMEOUT_MS) {
      z.online = false;
      z.zoneState = 2; // treat as FAULT for telemetry purposes
      Serial.print(F("[ESP32_CLUSTER] Zone "));
      Serial.print(z.zoneId);
      Serial.println(F(" marked OFFLINE"));
    }
  }
}

// -----------------------------
// TELEMETRY TO UNO_Q
// -----------------------------

void sendClusterTelemetry() {
  uint8_t destAddr[6];
  // UNO_Q master address is "HQ000"
  agri_getMasterAddress(destAddr);

  struct __attribute__((packed)) {
    AgriPacketHeader     hdr;
    AgriClusterTelemetry tel;
  } pkt;

  agri_buildHeader(
    pkt.hdr,
    AgriMsgType::CLUSTER_TELEMETRY,
    AgriRole::ESP32_CLUSTER,
    AgriRole::UNOQ_MMC,
    CLUSTER_ID,
    0   // MMC id
  );

  pkt.tel.clusterId = CLUSTER_ID;
  pkt.tel.seq       = pkt.hdr.seq;

  // For now these are placeholders – wire to real ADC/Modbus later.
  pkt.tel.batt_mV        = 0;
  pkt.tel.panel_mV       = 0;
  pkt.tel.pumpCurrent_mA = 0;

  pkt.tel.numZones = NUM_UNO_NODES;

  uint32_t now = millis();

  for (uint8_t i = 0; i < NUM_UNO_NODES; ++i) {
    LocalZoneState &z = g_zones[i];
    AgriZoneSnapshot &snap = pkt.tel.zones[i];

    snap.zoneId  = z.zoneId;
    snap.zoneState = z.zoneState;
    snap.soilPct = z.soilPct;

    if (z.lastRunStartMs == 0 || z.zoneState == 0) {
      // never run or currently idle – approximate "time since last irrigation"
      if (z.lastRunStartMs == 0) {
        snap.lastRun_s = 0;
      } else {
        snap.lastRun_s = (uint16_t)((now - z.lastRunStartMs) / 1000UL);
      }
    } else {
      // currently irrigating – time since this run started
      snap.lastRun_s = (uint16_t)((now - z.lastRunStartMs) / 1000UL);
    }
  }

  AgriResult res = agri_rf24_sendTo(
    radio,
    destAddr,
    &pkt,
    sizeof(pkt.hdr) + sizeof(AgriClusterTelemetry)
  );

  if (res != AgriResult::OK) {
    Serial.println(F("[ESP32_CLUSTER] sendClusterTelemetry RF fail"));
  } else {
    Serial.print(F("[ESP32_CLUSTER] sent telemetry; zones="));
    Serial.println(pkt.tel.numZones);
  }
}


