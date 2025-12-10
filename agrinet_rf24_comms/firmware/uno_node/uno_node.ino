// agrinet_rf24_comms/firmware/uno_node_full/uno_node_full.ino

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

/**
 * UNO Actuator Node – RF24 + Actuator State Machine
 *
 * - Listens for UNO_CMD_ACTUATOR from ESP32 on pipe "UN001"
 * - Sends UNO_STATUS periodically to ESP32 on pipe "CL001"
 * - Drives a single actuator (OPEN/CLOSE) with:
 *   * per-command runMs (capped by MAX_RUNTIME_MS)
 *   * direction interlock
 *   * overcurrent cutoff
 *   * comms-timeout failsafe -> CLOSE
 */

// -----------------------------
// HW CONFIG – PER NODE
// -----------------------------

// RF24 pins on Arduino UNO
static const uint8_t PIN_RF24_CE  = 9;
static const uint8_t PIN_RF24_CSN = 10;

// Actuator + sensors – adjust to your actual wiring
static const uint8_t PIN_ACTUATOR_OPEN  = 4;
static const uint8_t PIN_ACTUATOR_CLOSE = 5;
static const uint8_t PIN_CURRENT_SENSE  = A0;
static const uint8_t PIN_SOIL_SENSOR    = A1;
static const uint8_t PIN_BATT_SENSE     = A2;

// Node identity
static const uint8_t MY_CLUSTER_ID = 1;   // which ESP32 cluster we talk to
static const uint8_t MY_NODE_ID    = 1;   // local node ID within that cluster
static const uint8_t ZONE_LOCAL_ID = 1;   // zone ID (usually same as node)

// RF24 addresses
//  - Uplink (UNO -> ESP32) uses "CL001"
//  - Downlink (ESP32 -> UNO) uses "UN001"
static const byte ADDR_UP[6]   = "CL001";  // status to ESP32
static const byte ADDR_DOWN[6] = "UN001";  // commands from ESP32

// -----------------------------
// SAFETY CONSTANTS
// -----------------------------

static const uint32_t MAX_RUNTIME_MS        = 30000UL;   // absolute cap (30 s)
static const uint32_t DIRECTION_DELAY_MS    = 100UL;     // between reverse moves
static const uint32_t COMMS_TIMEOUT_MS      = 60000UL;   // 60 s
static const uint16_t OVERCURRENT_THRESHOLD = 2000;      // mA (adjust to real sensor)

static const uint32_t STATUS_PERIOD_MS      = 3000UL;    // 3 s

// -----------------------------
// RF24 INSTANCE
// -----------------------------

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// -----------------------------
// STATE
// -----------------------------
// Actuator action is encoded as AgriActAction (0=STOP,1=OPEN,2=CLOSE)
static uint8_t  g_currentAction    = static_cast<uint8_t>(AgriActAction::STOP);
static uint8_t  g_pendingAction    = static_cast<uint8_t>(AgriActAction::STOP);

static uint32_t g_actuatorStartMs  = 0;
static uint32_t g_interlockUntilMs = 0;

// Target runtime for the *current command* (bounded by MAX_RUNTIME_MS)
static uint32_t g_targetRunMs      = MAX_RUNTIME_MS;

static uint32_t g_lastStatusMs     = 0;
static uint32_t g_lastCommMs       = 0;

static uint16_t g_lastCmdSeq       = 0;
static uint16_t g_statusFlags      = 0;     // we only reliably use MOVING + FAULT

// Outgoing sequence number
static uint16_t g_seq              = 0;

// -----------------------------
// FORWARD DECLS
// -----------------------------

void handleIncomingPackets();
void processActuatorCommand(const AgriUnoActuatorCommand &cmd);

void requestActuatorAction(uint8_t newAction);
void applyActuatorOutputs(uint8_t action);
void updateActuatorState();
void checkSafetyConditions();

uint16_t readCurrentmA();
uint8_t  readSoilPct();
uint16_t readBattmV();

void sendUnoStatus();

// -----------------------------
// SETUP
// -----------------------------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n[UNO_NODE] Booting (FULL RF24 + ACTUATOR)"));
  Serial.print(F("[UNO_NODE] Cluster="));
  Serial.print(MY_CLUSTER_ID);
  Serial.print(F(" Node="));
  Serial.println(MY_NODE_ID);

  pinMode(PIN_ACTUATOR_OPEN, OUTPUT);
  pinMode(PIN_ACTUATOR_CLOSE, OUTPUT);
  digitalWrite(PIN_ACTUATOR_OPEN, LOW);
  digitalWrite(PIN_ACTUATOR_CLOSE, LOW);

  if (!radio.begin()) {
    Serial.println(F("[UNO_NODE] RF24 begin FAILED"));
  } else {
    Serial.println(F("[UNO_NODE] RF24 begin OK"));
  }

  // RF24 config – must match ESP32
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 15);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);

  // Uplink + downlink
  radio.openWritingPipe(ADDR_UP);
  radio.openReadingPipe(1, ADDR_DOWN);
  radio.startListening();

  uint32_t now = millis();
  g_lastCommMs   = now;
  g_lastStatusMs = now;
}

// -----------------------------
// MAIN LOOP
// -----------------------------

void loop() {
  uint32_t now = millis();

  // Handle RF24 RX (commands from ESP32)
  handleIncomingPackets();

  // Drive actuator state machine (non-blocking)
  updateActuatorState();

  // Safety (comms timeout -> failsafe close)
  checkSafetyConditions();

  // Periodic status up to cluster
  if (now - g_lastStatusMs >= STATUS_PERIOD_MS) {
    sendUnoStatus();
    g_lastStatusMs = now;
  }
}

// -----------------------------
// RF PACKET HANDLING
// -----------------------------

void handleIncomingPackets() {
  while (radio.available()) {
    uint8_t buffer[AGRI_RF24_PAYLOAD_MAX];
    uint8_t size = radio.getDynamicPayloadSize();
    if (size > sizeof(buffer)) {
      size = sizeof(buffer);
    }
    radio.read(buffer, size);

    if (size < sizeof(AgriPacketHeader)) {
      Serial.println(F("[UNO_NODE] RX too short"));
      continue;
    }

    AgriPacketHeader *hdr = reinterpret_cast<AgriPacketHeader*>(buffer);

    if (hdr->magic != AGRI_MAGIC_BYTE) {
      Serial.println(F("[UNO_NODE] BAD magic, dropping"));
      continue;
    }

    // Any valid RX counts as comms activity
    g_lastCommMs = millis();

    AgriMsgType msgType = static_cast<AgriMsgType>(hdr->msgType);

    switch (msgType) {
      case AgriMsgType::UNO_CMD_ACTUATOR: {
        if (size < sizeof(AgriPacketHeader) + sizeof(AgriUnoActuatorCommand)) {
          Serial.println(F("[UNO_NODE] CMD len bad"));
          break;
        }
        AgriUnoActuatorCommand *cmd =
          reinterpret_cast<AgriUnoActuatorCommand*>(buffer + sizeof(AgriPacketHeader));
        processActuatorCommand(*cmd);
        break;
      }

      case AgriMsgType::UNO_PING: {
        Serial.println(F("[UNO_NODE] PING -> sending status"));
        sendUnoStatus();
        break;
      }

      default:
        Serial.print(F("[UNO_NODE] Unknown msgType 0x"));
        Serial.println(static_cast<uint8_t>(msgType), HEX);
        break;
    }
  }
}

// -----------------------------
// ACTUATOR COMMAND PROCESSING
// -----------------------------

void processActuatorCommand(const AgriUnoActuatorCommand &cmd) {
  Serial.print(F("[UNO_NODE] CMD zone="));
  Serial.print(cmd.zoneLocalId);
  Serial.print(F(" action="));
  Serial.print(cmd.action);
  Serial.print(F(" runMs="));
  Serial.println(cmd.runMs);

  g_lastCmdSeq = cmd.cmdSeq;

  uint8_t action = cmd.action;

  if (action > static_cast<uint8_t>(AgriActAction::CLOSE)) {
    Serial.println(F("[UNO_NODE] Unknown action, ignoring"));
    return;
  }

  // Honour runMs, but cap it for safety.
  uint32_t requested = cmd.runMs;
  if (requested == 0) {
    g_targetRunMs = MAX_RUNTIME_MS;
  } else if (requested > MAX_RUNTIME_MS) {
    g_targetRunMs = MAX_RUNTIME_MS;
  } else {
    g_targetRunMs = requested;
  }

  // Clear FAULT bit on new command (we’re giving it another go)
  g_statusFlags &= ~STATUS_FLAG_FAULT;

  requestActuatorAction(action);
}

// -----------------------------
// ACTUATOR CONTROL STATE MACHINE
// -----------------------------

void requestActuatorAction(uint8_t newAction) {
  uint8_t stop   = static_cast<uint8_t>(AgriActAction::STOP);
  uint8_t openA  = static_cast<uint8_t>(AgriActAction::OPEN);
  uint8_t closeA = static_cast<uint8_t>(AgriActAction::CLOSE);

  if (newAction == stop) {
    // Immediate stop, keep last position unknown
    applyActuatorOutputs(stop);
    g_currentAction    = stop;
    g_pendingAction    = stop;
    g_actuatorStartMs  = 0;
    g_interlockUntilMs = 0;
    g_statusFlags &= ~STATUS_FLAG_MOVING;
    Serial.println(F("[UNO_NODE] STOP requested"));
    return;
  }

  // Direction change → brief interlock
  if ((g_currentAction == openA && newAction == closeA) ||
      (g_currentAction == closeA && newAction == openA)) {

    Serial.println(F("[UNO_NODE] Direction change with interlock"));
    applyActuatorOutputs(stop);
    g_currentAction    = stop;
    g_pendingAction    = newAction;
    g_interlockUntilMs = millis() + DIRECTION_DELAY_MS;
    g_statusFlags &= ~STATUS_FLAG_MOVING;
    return;
  }

  // From STOP or same direction: just schedule
  g_pendingAction    = newAction;
  g_interlockUntilMs = millis();  // allow immediate start in updateActuatorState()
}

void applyActuatorOutputs(uint8_t action) {
  uint8_t stop   = static_cast<uint8_t>(AgriActAction::STOP);
  uint8_t openA  = static_cast<uint8_t>(AgriActAction::OPEN);
  uint8_t closeA = static_cast<uint8_t>(AgriActAction::CLOSE);

  if (action == stop) {
    digitalWrite(PIN_ACTUATOR_OPEN, LOW);
    digitalWrite(PIN_ACTUATOR_CLOSE, LOW);
    Serial.println(F("[UNO_NODE] Actuator outputs: STOP"));
  } else if (action == openA) {
    digitalWrite(PIN_ACTUATOR_CLOSE, LOW);
    digitalWrite(PIN_ACTUATOR_OPEN, HIGH);
    Serial.println(F("[UNO_NODE] Actuator outputs: OPEN"));
  } else if (action == closeA) {
    digitalWrite(PIN_ACTUATOR_OPEN, LOW);
    digitalWrite(PIN_ACTUATOR_CLOSE, HIGH);
    Serial.println(F("[UNO_NODE] Actuator outputs: CLOSE"));
  }
}

void updateActuatorState() {
  uint32_t now = millis();
  uint8_t stop   = static_cast<uint8_t>(AgriActAction::STOP);

  // Handle pending actions after interlock
  if (g_pendingAction != stop &&
      now >= g_interlockUntilMs &&
      g_currentAction == stop) {

    // Start new move
    g_currentAction    = g_pendingAction;
    g_pendingAction    = stop;
    g_actuatorStartMs  = now;
    g_statusFlags |= STATUS_FLAG_MOVING;
    applyActuatorOutputs(g_currentAction);
  }

  // If we’re moving, enforce per-command runtime + max runtime + overcurrent
  if (g_currentAction != stop) {
    uint32_t elapsed = now - g_actuatorStartMs;

    // 1) Normal completion: runMs reached
    if (elapsed >= g_targetRunMs) {
      Serial.println(F("[UNO_NODE] Command runMs reached -> STOP (normal completion)"));

      g_statusFlags &= ~STATUS_FLAG_MOVING;

      applyActuatorOutputs(stop);
      g_currentAction    = stop;
      g_actuatorStartMs  = 0;
      return;
    }

    // 2) Hard safety cap (absolute MAX_RUNTIME ceiling)
    if (elapsed >= MAX_RUNTIME_MS) {
      Serial.println(F("[UNO_NODE] MAX_RUNTIME reached -> STOP (timeout)"));
      g_statusFlags |= STATUS_FLAG_FAULT;
      g_statusFlags &= ~STATUS_FLAG_MOVING;
      applyActuatorOutputs(stop);
      g_currentAction    = stop;
      g_actuatorStartMs  = 0;
      return;
    }

    // 3) Overcurrent protection
    uint16_t currentmA = readCurrentmA();
    if (currentmA > OVERCURRENT_THRESHOLD) {
      Serial.println(F("[UNO_NODE] OVERCURRENT -> STOP + FAULT"));
      g_statusFlags |= STATUS_FLAG_FAULT;
      g_statusFlags &= ~STATUS_FLAG_MOVING;
      applyActuatorOutputs(stop);
      g_currentAction    = stop;
      g_actuatorStartMs  = 0;
      return;
    }
  }
}

// -----------------------------
// SAFETY – COMMS TIMEOUT FAILSAFE
// -----------------------------

void checkSafetyConditions() {
  uint32_t now = millis();

  // If we haven't seen *any* RF activity (RX or TX-ACK) for too long → failsafe CLOSE
  if (now - g_lastCommMs >= COMMS_TIMEOUT_MS) {
    Serial.println(F("[UNO_NODE] COMMS TIMEOUT -> failsafe CLOSE"));
    g_lastCommMs = now;  // avoid spamming

    uint8_t closeA = static_cast<uint8_t>(AgriActAction::CLOSE);
    uint8_t stop   = static_cast<uint8_t>(AgriActAction::STOP);

    // Only request if not already closing or pending
    if (g_currentAction == stop &&
        g_pendingAction == stop) {
      g_targetRunMs = MAX_RUNTIME_MS;  // full stroke attempt
      requestActuatorAction(closeA);
    }
  }
}

// -----------------------------
// SENSOR READINGS
// -----------------------------

uint16_t readCurrentmA() {
  int raw = analogRead(PIN_CURRENT_SENSE);
  if (raw < 0) raw = 0;
  if (raw > 1023) raw = 1023;
  // TODO: calibrate properly; placeholder 5 mA/ADC step
  return static_cast<uint16_t>(raw * 5);
}

uint8_t readSoilPct() {
  int raw = analogRead(PIN_SOIL_SENSOR);
  if (raw < 0) raw = 0;
  if (raw > 1023) raw = 1023;
  long pct = map(raw, 0, 1023, 0, 100);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return static_cast<uint8_t>(pct);
}

uint16_t readBattmV() {
  int raw = analogRead(PIN_BATT_SENSE);
  if (raw < 0) raw = 0;
  if (raw > 1023) raw = 1023;
  // Example: 2:1 divider, 5 V reference. Adjust to your real resistors.
  uint32_t mv = (static_cast<uint32_t>(raw) * 5000UL * 2UL) / 1023UL;
  return static_cast<uint16_t>(mv);
}

// -----------------------------
// STATUS UPSTREAM TO CLUSTER
// -----------------------------

void sendUnoStatus() {
  struct {
    AgriPacketHeader hdr;
    AgriUnoStatus    status;
  } pkt;

  // Build header manually
  pkt.hdr.magic   = AGRI_MAGIC_BYTE;
  pkt.hdr.msgType = static_cast<uint8_t>(AgriMsgType::UNO_STATUS);
  pkt.hdr.srcRole = static_cast<uint8_t>(AgriRole::UNO_NODE);
  pkt.hdr.dstRole = static_cast<uint8_t>(AgriRole::ESP32_CLUSTER);
  pkt.hdr.srcId   = MY_NODE_ID;
  pkt.hdr.dstId   = MY_CLUSTER_ID;
  pkt.hdr.seq     = ++g_seq;

  // Fill status
  pkt.status.zoneLocalId = ZONE_LOCAL_ID;
  pkt.status.statusFlags = g_statusFlags;
  pkt.status.current_mA  = readCurrentmA();
  pkt.status.soilRaw     = analogRead(PIN_SOIL_SENSOR);
  pkt.status.soilPct     = readSoilPct();
  pkt.status.batt_mV     = readBattmV();
  pkt.status.lastCmdSeq  = g_lastCmdSeq;

  uint8_t len = sizeof(pkt);

  Serial.print(F("[UNO_NODE] sendUnoStatus len="));
  Serial.print(len);
  Serial.print(F(" hdr="));
  Serial.print(sizeof(AgriPacketHeader));
  Serial.print(F(" status="));
  Serial.println(sizeof(AgriUnoStatus));

  // TX burst: stop listening, send, resume listening
  radio.stopListening();
  bool ok = radio.write(&pkt, len);
  radio.startListening();

  if (!ok) {
    Serial.println(F("[UNO_NODE] sendUnoStatus RF FAIL"));
  } else {
    // *** KEY FIX: successful TX counts as comms activity ***
    g_lastCommMs = millis();
    Serial.println(F("[UNO_NODE] sendUnoStatus OK"));
  }
}
