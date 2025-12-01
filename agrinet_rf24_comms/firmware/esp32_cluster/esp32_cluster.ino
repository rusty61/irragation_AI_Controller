#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

// -----------------------------
// HW CONFIG - ADJUST
// -----------------------------

static const uint8_t PIN_RF24_CE  = 22;
static const uint8_t PIN_RF24_CSN = 5;

static const uint8_t CLUSTER_ID = 1;

// UNO nodes under this cluster
static const uint8_t UNO_NODE_IDS[] = {1, 2};  // example: 2 nodes
static const uint8_t NUM_UNO_NODES  = sizeof(UNO_NODE_IDS) / sizeof(UNO_NODE_IDS[0]);

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// Timing
static const uint32_t TELEMETRY_PERIOD_MS = 5000;
uint32_t g_lastTelemetryMs = 0;

// -----------------------------
// FORWARD DECLS
// -----------------------------

void handleLocalUnoPackets();
void handleMasterPackets();
void sendClusterTelemetry();

// -----------------------------
// SETUP / LOOP
// -----------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n[ESP32_CLUSTER] Booting"));

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
  //  - set up local state for each UNO node (online/offline, last seq, etc).
}

void loop() {
  // For now, treat all packets identically and inspect headers
  if (radio.available()) {
    uint8_t buffer[AGRI_RF24_PAYLOAD_MAX];
    uint8_t size = radio.getDynamicPayloadSize();
    radio.read(buffer, size);

    if (size < sizeof(AgriPacketHeader)) {
      Serial.println(F("[ESP32_CLUSTER] RX too short"));
    } else {
      AgriPacketHeader *hdr = reinterpret_cast<AgriPacketHeader*>(buffer);
      if (hdr->magic != AGRI_MAGIC_BYTE) {
        Serial.println(F("[ESP32_CLUSTER] BAD magic"));
      } else {
        // route based on srcRole
        AgriRole srcRole = static_cast<AgriRole>(hdr->srcRole);
        if (srcRole == AgriRole::UNO_NODE) {
          // from a local UNO
          // In a full implementation, pass to onUnoStatusReceived/Telemetry
          Serial.println(F("[ESP32_CLUSTER] Packet from UNO_NODE"));
        } else if (srcRole == AgriRole::UNOQ_MMC) {
          // from MMC
          Serial.println(F("[ESP32_CLUSTER] Packet from UNO_Q MMC"));
        }
      }
    }
  }

  uint32_t now = millis();
  if (now - g_lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    sendClusterTelemetry();
    g_lastTelemetryMs = now;
  }

  // TODO:
  //  - poll per-UNO state machines
  //  - resend commands with retry windows
}

// -----------------------------
// TELEMETRY TO UNO_Q
// -----------------------------

void sendClusterTelemetry() {
  uint8_t destAddr[6];
  // UNO_Q as global master
  // for now, UNO_Q address is hardcoded helper
  agri_getClusterAddress(0, destAddr); // or define a dedicated helper if you prefer

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

  pkt.tel.clusterId       = CLUSTER_ID;
  pkt.tel.seq             = pkt.hdr.seq;
  pkt.tel.batt_mV         = 0; // TODO: measure from ADC/Modbus
  pkt.tel.panel_mV        = 0; // TODO
  pkt.tel.pumpCurrent_mA  = 0; // TODO
  pkt.tel.numZones        = 0; // TODO: fill from UNO children

  // TODO: fill zones[] with aggregated data from UNO nodes

  AgriResult res = agri_rf24_sendTo(
    radio,
    destAddr,
    &pkt,
    sizeof(pkt.hdr) + sizeof(AgriClusterTelemetry)
  );

  if (res != AgriResult::OK) {
    Serial.println(F("[ESP32_CLUSTER] sendClusterTelemetry RF fail"));
  } else {
    Serial.println(F("[ESP32_CLUSTER] sent telemetry"));
  }
}

