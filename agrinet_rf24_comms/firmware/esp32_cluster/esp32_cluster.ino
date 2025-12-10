// agrinet_rf24_comms/firmware/esp32_cluster_full/esp32_cluster_full.ino

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

/**
 * ESP32 Cluster Node – RF24 RX + TX
 *
 * - Listens for UNO_STATUS from UNO on pipe "CL001"
 * - Sends UNO_CMD_ACTUATOR to UNO on pipe "UN001"
 * - Serial commands:
 *   'o' -> OPEN  5000 ms
 *   'c' -> CLOSE 5000 ms
 *   's' -> STOP  (runMs=0)
 */

// -----------------------------
// HW CONFIG - ESP32
// -----------------------------

static const uint8_t PIN_RF24_CE  = 4;
static const uint8_t PIN_RF24_CSN = 5;

// RF24 addresses – must match UNO side
static const byte ADDR_UP[6]   = "CL001";   // UNO -> ESP32 (status)
static const byte ADDR_DOWN[6] = "UN001";   // ESP32 -> UNO (commands)

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// IDs
static const uint8_t CLUSTER_ID   = 1;   // this ESP32 "cluster"
static const uint8_t UNO_NODE_ID  = 1;   // target UNO node
static const uint8_t ZONE_LOCAL_ID = 1;  // zone on that UNO

// -----------------------------
// LOCAL ZONE STATE
// -----------------------------

struct LocalZoneState {
  uint8_t  zoneId;         // local zone id
  uint8_t  zoneState;      // 0=IDLE, 1=IRRIGATING, 2=FAULT (not used yet)
  uint8_t  soilPct;        // 0-100
  uint32_t lastRunStartMs; // not used yet
  uint32_t lastSeenMs;     // last time we received any status
  bool     online;         // derived from lastSeenMs
};

static LocalZoneState g_zone1;

static const uint32_t UNO_OFFLINE_TIMEOUT_MS = 30000;

// Outgoing sequence for packets
static uint16_t g_seq      = 0;
// Command sequence mirrored into UNO status lastCmdSeq
static uint16_t g_cmdSeq   = 0;

// -----------------------------
// FORWARD DECLS
// -----------------------------

void handleRx();
void handleUnoStatus(const AgriPacketHeader &hdr,
                     const AgriUnoStatus &status);
void handleSerialCommands();
void sendActuatorCommand(uint8_t action, uint32_t runMs);

// -----------------------------
// SETUP / LOOP
// -----------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n[ESP32_CLUSTER] Booting (FULL RF24 RX+TX)"));

  g_zone1.zoneId         = 1;
  g_zone1.zoneState      = 0;
  g_zone1.soilPct        = 0;
  g_zone1.lastRunStartMs = 0;
  g_zone1.lastSeenMs     = 0;
  g_zone1.online         = false;

  if (!radio.begin()) {
    Serial.println(F("[ESP32_CLUSTER] RF24 begin FAILED"));
  } else {
    Serial.println(F("[ESP32_CLUSTER] RF24 begin OK"));
  }

  // Match UNO RF24 settings
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 15);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);

  // Uplink + downlink
  radio.openReadingPipe(1, ADDR_UP);    // status from UNO
  radio.openWritingPipe(ADDR_DOWN);     // commands to UNO
  radio.startListening();

  Serial.println(F("[ESP32_CLUSTER] Listening on pipe \"CL001\""));
  Serial.println(F("[ESP32_CLUSTER] Serial commands: o=open 5s, c=close 5s, s=stop"));
}

void loop() {
  // RX: handle incoming status packets
  handleRx();

  // Serial: send actuator commands
  handleSerialCommands();

  // Offline detection
  uint32_t now = millis();
  if (g_zone1.online && (now - g_zone1.lastSeenMs > UNO_OFFLINE_TIMEOUT_MS)) {
    g_zone1.online = false;
    Serial.println(F("[ESP32_CLUSTER] UNO marked OFFLINE (no status)"));
  }
}

// -----------------------------
// RX HANDLER
// -----------------------------

void handleRx() {
  while (radio.available()) {
    uint8_t buffer[AGRI_RF24_PAYLOAD_MAX];
    uint8_t size = radio.getDynamicPayloadSize();
    if (size > sizeof(buffer)) {
      size = sizeof(buffer);
    }
    radio.read(buffer, size);

    Serial.print(F("[ESP32_CLUSTER] RX len="));
    Serial.println(size);

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

    if (msgType == AgriMsgType::UNO_STATUS) {
      Serial.print(F("[ESP32_CLUSTER] payloadLen="));
      Serial.print(payloadLen);
      Serial.print(F(" expected="));
      Serial.println(sizeof(AgriUnoStatus));

      if (payloadLen < sizeof(AgriUnoStatus)) {
        Serial.println(F("[ESP32_CLUSTER] UNO_STATUS len too short"));
        continue;
      }
      const AgriUnoStatus *st =
        reinterpret_cast<const AgriUnoStatus*>(payload);
      handleUnoStatus(*hdr, *st);
    } else {
      Serial.print(F("[ESP32_CLUSTER] Unknown msgType 0x"));
      Serial.println(static_cast<uint8_t>(msgType), HEX);
    }
  }
}

// -----------------------------
// HANDLE UNO STATUS
// -----------------------------

void handleUnoStatus(const AgriPacketHeader &hdr,
                     const AgriUnoStatus &status) {
  uint32_t now = millis();
  g_zone1.lastSeenMs = now;
  g_zone1.online     = true;
  g_zone1.soilPct    = status.soilPct;

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
// SERIAL → ACTUATOR COMMANDS
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
// SEND ACTUATOR COMMAND
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

  // TX burst: stop listening, send, resume listening
  radio.stopListening();
  bool ok = radio.write(&pkt, len);
  radio.startListening();

  if (!ok) {
    Serial.println(F("[ESP32_CLUSTER] sendActuatorCommand RF FAIL"));
  } else {
    Serial.println(F("[ESP32_CLUSTER] sendActuatorCommand OK"));
  }
}
