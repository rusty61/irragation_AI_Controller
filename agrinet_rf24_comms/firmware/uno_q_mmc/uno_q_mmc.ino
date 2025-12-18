 #include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

#include <Arduino_RouterBridge.h>
#define CONSOLE Monitor

// -----------------------------
// HW CONFIG - UNO_Q RF24 pins
// -----------------------------
static const uint8_t PIN_RF24_CE  = 9;
static const uint8_t PIN_RF24_CSN = 10;

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// -----------------------------
// UNO_Q <-> ESP32 addresses (from helpers)
// -----------------------------
static uint8_t g_addrMaster[6];   // UNO_Q listens here (ESP32 sends telemetry to this)
static uint8_t g_addrCluster[6];  // UNO_Q sends control to this (ESP32 listens on pipe 2)

// -----------------------------
// COMMS heartbeat (RX/TX-ACK)
// -----------------------------
static uint32_t g_lastCommMs_cluster = 0;
static bool     g_clusterOnline      = false;
static const uint32_t CLUSTER_TIMEOUT_MS = 60000UL;

// -----------------------------
// TEST TIMER
// -----------------------------
static const uint32_t SEND_CMD_EVERY_MS = 15000UL;

// -----------------------------
// SMALL CONTROL PAYLOAD (FITS NRF24)
// -----------------------------
// NOTE: Struct definition moved to agri_rf24_common.h

static uint16_t g_cmdSeq = 0;

// -----------------------------
// FORWARD DECLS
// -----------------------------
static void handleRx();
static void handleClusterTelemetry(const AgriPacketHeader &hdr, const AgriClusterTelemetry &tel);

static bool sendFakeZoneCmd(uint8_t clusterId);

static inline void clusterHeartbeatRx();
static inline void clusterHeartbeatTxAck();

// -----------------------------
// SETUP / LOOP
// -----------------------------
void setup() {
  CONSOLE.begin();
  delay(300);

  CONSOLE.println();
  CONSOLE.println(F("[UNO_Q_MMC] Booting (Master/MMC)"));

  AgriResult res = agri_rf24_init_common(
    radio,
    AgriRole::UNOQ_MMC,
    0,
    0
  );

  if (res != AgriResult::OK) {
    CONSOLE.println(F("[UNO_Q_MMC] RF24 init FAILED"));
  } else {
    CONSOLE.println(F("[UNO_Q_MMC] RF24 init OK"));
  }

  // FORCE SETTINGS TO MATCH ESP32 EXACTLY
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setRetries(5, 15);     // 15*250us = 4ms delay, 5 retries

  // Addresses must match ESP32 expectations:
  // ESP32 sends telemetry to Master address
  // ESP32 receives MMC commands on Cluster address (pipe 2)
  agri_getMasterAddress(g_addrMaster);
  agri_getClusterAddress(1, g_addrCluster); // clusterId = 1 (match ESP32 CLUSTER_ID)

  CONSOLE.print(F("[UNO_Q] Cluster Addr: "));
  for(int i=0; i<5; i++) { CONSOLE.print(g_addrCluster[i], HEX); CONSOLE.print(" "); }
  CONSOLE.println();

  // Listen for ESP32->UNO_Q telemetry on Master address
  radio.openReadingPipe(1, g_addrMaster);
  radio.startListening();

  CONSOLE.println(F("[UNO_Q_MMC] Listening on MASTER addr for telemetry (pipe 1)"));
  CONSOLE.println(F("[UNO_Q_MMC] Will TX commands to cluster addr (pipe 2 on ESP32)"));

  g_lastCommMs_cluster = millis();
  g_clusterOnline      = false;
}

void loop() {
  handleRx();

  static uint32_t lastCmdMs = 0;
  uint32_t now = millis();
  if (now - lastCmdMs >= SEND_CMD_EVERY_MS) {
    sendFakeZoneCmd(1);
    lastCmdMs = now;
  }

  if (g_clusterOnline && (now - g_lastCommMs_cluster > CLUSTER_TIMEOUT_MS)) {
    g_clusterOnline = false;
    CONSOLE.println(F("[UNO_Q_MMC] Cluster marked OFFLINE (no RX/TX-ACK)"));
  }
}

// -----------------------------
// RX HANDLING (ESP32 -> UNO_Q)
// -----------------------------
static void handleRx() {
  while (radio.available()) {
    uint8_t buf[AGRI_RF24_PAYLOAD_MAX];
    uint8_t len = radio.getDynamicPayloadSize();
    if (len > sizeof(buf)) len = sizeof(buf);

    radio.read(buf, len);

    if (len < sizeof(AgriPacketHeader)) {
      CONSOLE.println(F("[UNO_Q_MMC] RX too short"));
      continue;
    }

    const AgriPacketHeader *hdr = reinterpret_cast<const AgriPacketHeader*>(buf);
    if (hdr->magic != AGRI_MAGIC_BYTE) {
      CONSOLE.println(F("[UNO_Q_MMC] RX BAD magic"));
      continue;
    }

    const uint8_t *payload = buf + sizeof(AgriPacketHeader);
    const uint8_t payloadLen = len - sizeof(AgriPacketHeader);
    const AgriMsgType msgType = static_cast<AgriMsgType>(hdr->msgType);

    switch (msgType) {
      case AgriMsgType::CLUSTER_TELEMETRY: {
        if (payloadLen < sizeof(AgriClusterTelemetry)) {
          CONSOLE.println(F("[UNO_Q_MMC] CLUSTER_TELEMETRY too short"));
          break;
        }
        clusterHeartbeatRx();

        const AgriClusterTelemetry *tel =
          reinterpret_cast<const AgriClusterTelemetry*>(payload);
        handleClusterTelemetry(*hdr, *tel);
        break;
      }

      default:
        CONSOLE.print(F("[UNO_Q_MMC] RX msgType=0x"));
        CONSOLE.println(static_cast<uint8_t>(msgType), HEX);
        break;
    }
  }
}

// -----------------------------
// TELEMETRY HANDOFF
// -----------------------------
static void handleClusterTelemetry(const AgriPacketHeader &hdr, const AgriClusterTelemetry &tel) {
  CONSOLE.print(F("[UNO_Q_MMC] TELEMETRY from cluster "));
  CONSOLE.print(tel.clusterId);
  CONSOLE.print(F(" seq="));
  CONSOLE.print(tel.seq);
  CONSOLE.print(F(" batt="));
  CONSOLE.print(tel.batt_mV);
  CONSOLE.print(F(" panel="));
  CONSOLE.print(tel.panel_mV);
  CONSOLE.print(F(" pump="));
  CONSOLE.print(tel.pumpCurrent_mA);
  CONSOLE.print(F(" zones="));
  CONSOLE.println(tel.numZones);
}

// -----------------------------
// SEND SMALL CMD (UNO_Q -> ESP32) FITS NRF24
// -----------------------------
static bool sendFakeZoneCmd(uint8_t clusterId) {
  // Packet: header + AgriClusterZoneCmd (tiny)
  uint8_t buf[AGRI_RF24_PAYLOAD_MAX];

  const uint8_t headerSize  = sizeof(AgriPacketHeader);
  const uint8_t payloadSize = sizeof(AgriClusterZoneCmd);
  const uint8_t totalSize   = headerSize + payloadSize;

  if (totalSize > AGRI_RF24_PAYLOAD_MAX) {
    CONSOLE.println(F("[UNO_Q_MMC] ZoneCmd too large (should never happen)"));
    return false;
  }

  // Build header in local then memcpy (avoid packed bind issues)
  AgriPacketHeader hdr;
  agri_buildHeader(
    hdr,
    AgriMsgType::CLUSTER_CMD_ZONE,
    AgriRole::UNOQ_MMC,
    AgriRole::ESP32_CLUSTER,
    0,           // MMC id
    clusterId
  );

  memcpy(buf, &hdr, headerSize);

  AgriClusterZoneCmd cmd;
  cmd.clusterId   = clusterId;
  cmd.zoneId      = 1;
  cmd.mode        = 1;          // AUTO (test)
  cmd.cmdSeq      = ++g_cmdSeq;
  cmd.duration_s  = 600;        // 10 min

  memcpy(buf + headerSize, &cmd, payloadSize);

  radio.stopListening();
  AgriResult res = agri_rf24_sendTo(radio, g_addrCluster, buf, totalSize);
  radio.startListening();

  if (res == AgriResult::OK) {
    clusterHeartbeatTxAck();
    CONSOLE.print(F("[UNO_Q_MMC] Sent CLUSTER_CMD_ZONE to cluster "));
    CONSOLE.print(clusterId);
    CONSOLE.print(F(" cmdSeq="));
    CONSOLE.print(cmd.cmdSeq);
    CONSOLE.print(F(" (len="));
    CONSOLE.print(totalSize);
    CONSOLE.println(F(")"));
    return true;
  } else {
    CONSOLE.print(F("[UNO_Q_MMC] ZoneCmd TX FAIL res="));
    CONSOLE.println(static_cast<uint8_t>(res));
    return false;
  }
}

// -----------------------------
// HEARTBEAT HELPERS (RX/TX-ACK)
// -----------------------------
static inline void clusterHeartbeatRx() {
  uint32_t now = millis();
  g_lastCommMs_cluster = now;
  if (!g_clusterOnline) {
    g_clusterOnline = true;
    CONSOLE.println(F("[UNO_Q_MMC] Cluster marked ONLINE (RX)"));
  }
}

static inline void clusterHeartbeatTxAck() {
  uint32_t now = millis();
  g_lastCommMs_cluster = now;
  if (!g_clusterOnline) {
    g_clusterOnline = true;
    CONSOLE.println(F("[UNO_Q_MMC] Cluster marked ONLINE (TX-ACK)"));
  }
}
