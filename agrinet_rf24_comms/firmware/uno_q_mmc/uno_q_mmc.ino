#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

// -----------------------------
// HW CONFIG - ADJUST
// -----------------------------

static const uint8_t PIN_RF24_CE  = 7;
static const uint8_t PIN_RF24_CSN = 8;

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// Example: clusters we expect
static const uint8_t CLUSTER_IDS[] = {1, 2};
static const uint8_t NUM_CLUSTERS  = sizeof(CLUSTER_IDS) / sizeof(CLUSTER_IDS[0]);

// -----------------------------
// FORWARD DECLS
// -----------------------------

void handleClusterPackets();
void onClusterTelemetry(const AgriClusterTelemetry &tel);
void sendFakeSchedule(uint8_t clusterId);

// -----------------------------
// SETUP / LOOP
// -----------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n[UNO_Q_MMC] Booting"));

  AgriResult res = agri_rf24_init_common(
    radio,
    AgriRole::UNOQ_MMC,
    0,
    0
  );

  if (res != AgriResult::OK) {
    Serial.println(F("[UNO_Q_MMC] RF24 init failed"));
  } else {
    Serial.println(F("[UNO_Q_MMC] RF24 init OK"));
  }

  // TODO: open additional pipes if you want per-cluster pipes rather than a single address
}

void loop() {
  handleClusterPackets();

  // Example: send a fake schedule periodically to cluster 1 for testing
  static uint32_t lastSchedMs = 0;
  uint32_t now = millis();
  if (now - lastSchedMs > 15000) {
    sendFakeSchedule(1);
    lastSchedMs = now;
  }

  // TODO: IPC with Linux side to forward telemetry / receive real schedules
}

// -----------------------------
// RX HANDLING
// -----------------------------

void handleClusterPackets() {
  while (radio.available()) {
    uint8_t buffer[AGRI_RF24_PAYLOAD_MAX];
    uint8_t size = radio.getDynamicPayloadSize();
    radio.read(buffer, size);

    if (size < sizeof(AgriPacketHeader)) {
      Serial.println(F("[UNO_Q_MMC] RX too short"));
      continue;
    }

    AgriPacketHeader *hdr = reinterpret_cast<AgriPacketHeader*>(buffer);
    if (hdr->magic != AGRI_MAGIC_BYTE) {
      Serial.println(F("[UNO_Q_MMC] BAD magic"));
      continue;
    }

    AgriMsgType msgType = static_cast<AgriMsgType>(hdr->msgType);

    switch (msgType) {
      case AgriMsgType::CLUSTER_TELEMETRY: {
        if (size < sizeof(AgriPacketHeader) + sizeof(AgriClusterTelemetry)) {
          Serial.println(F("[UNO_Q_MMC] Bad TELEMETRY len"));
          break;
        }
        AgriClusterTelemetry *tel =
          reinterpret_cast<AgriClusterTelemetry*>(buffer + sizeof(AgriPacketHeader));
        onClusterTelemetry(*tel);
        break;
      }

      default:
        // TODO: handle STATUS/FAULT/HEARTBEAT etc.
        break;
    }
  }
}

// -----------------------------
// TELEMETRY HANDOFF
// -----------------------------

void onClusterTelemetry(const AgriClusterTelemetry &tel) {
  Serial.print(F("[UNO_Q_MMC] Telemetry from cluster "));
  Serial.print(tel.clusterId);
  Serial.print(F(" batt="));
  Serial.print(tel.batt_mV);
  Serial.print(F(" panel="));
  Serial.print(tel.panel_mV);
  Serial.print(F(" pump="));
  Serial.print(tel.pumpCurrent_mA);
  Serial.print(F(" zones="));
  Serial.println(tel.numZones);

  // TODO: forward to Linux (QRB2210) over UART/SPI/etc
}

// -----------------------------
// SCHEDULE TX (TEST STUB)
// -----------------------------

void sendFakeSchedule(uint8_t clusterId) {
  uint8_t destAddr[6];
  agri_getClusterAddress(clusterId, destAddr);

  struct __attribute__((packed)) {
    AgriPacketHeader     hdr;
    AgriClusterSchedule  sched;
  } pkt;

  agri_buildHeader(
    pkt.hdr,
    AgriMsgType::CLUSTER_SET_SCHED,
    AgriRole::UNOQ_MMC,
    AgriRole::ESP32_CLUSTER,
    0,           // MMC id
    clusterId
  );

  pkt.sched.clusterId = clusterId;
  pkt.sched.schedSeq  = pkt.hdr.seq;   // simple: tie seq to schedule version
  pkt.sched.numEntries = 1;

  pkt.sched.entries[0].zoneId       = 1;
  pkt.sched.entries[0].mode         = 1;         // AUTO
  pkt.sched.entries[0].startEpoch_s = 0;         // relative / placeholder
  pkt.sched.entries[0].duration_s   = 600;       // 10 mins
  pkt.sched.entries[0].priority     = 0;

  AgriResult res = agri_rf24_sendTo(
    radio,
    destAddr,
    &pkt,
    sizeof(pkt.hdr) + sizeof(AgriClusterSchedule)
  );

  if (res != AgriResult::OK) {
    Serial.println(F("[UNO_Q_MMC] sendFakeSchedule RF fail"));
  } else {
    Serial.print(F("[UNO_Q_MMC] Sent fake schedule to cluster "));
    Serial.println(clusterId);
  }
}

