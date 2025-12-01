/**
 * @file uno_node.ino
 * @brief UNO Actuator Node Firmware for AgriNet RF24 Communications
 * 
 * This firmware implements a UNO actuator node (bottom tier) that:
 * - Receives commands from ESP32 cluster nodes
 * - Controls valves/actuators with safety features
 * - Sends status and telemetry to ESP32 cluster
 */

#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

// -----------------------------
// HW CONFIG - ADJUST PER NODE
// -----------------------------

// RF24 pins for Arduino UNO (per README)
static const uint8_t PIN_RF24_CE  = 9;
static const uint8_t PIN_RF24_CSN = 10;

// Node configuration - SET THESE FOR EACH NODE
static const uint8_t MY_CLUSTER_ID = 1;   // Which cluster this node belongs to
static const uint8_t MY_NODE_ID    = 1;   // This node's local ID within the cluster

// Actuator pins (example - adjust for your hardware)
static const uint8_t PIN_ACTUATOR_OPEN  = 4;
static const uint8_t PIN_ACTUATOR_CLOSE = 5;
static const uint8_t PIN_CURRENT_SENSE  = A0;
static const uint8_t PIN_SOIL_SENSOR    = A1;
static const uint8_t PIN_BATT_SENSE     = A2;

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// -----------------------------
// SAFETY CONSTANTS (per README)
// -----------------------------

static const uint32_t MAX_RUNTIME_MS        = 30000;   // Max actuator runtime (30 seconds)
static const uint32_t DIRECTION_DELAY_MS    = 100;     // Delay between direction changes
static const uint32_t COMMS_TIMEOUT_MS      = 60000;   // Failsafe timeout (60 seconds)
static const uint16_t OVERCURRENT_THRESHOLD = 2000;    // Overcurrent threshold in mA

// Timing
static const uint32_t STATUS_PERIOD_MS = 3000;

// -----------------------------
// STATE VARIABLES
// -----------------------------

uint32_t g_lastStatusMs         = 0;
uint32_t g_lastCommMs           = 0;
uint32_t g_actuatorStartMs      = 0;
uint16_t g_lastCmdSeq           = 0;
uint16_t g_statusFlags          = STATUS_FLAG_CLOSED | STATUS_FLAG_COMMS_OK;
uint8_t  g_currentAction        = 0;  // AgriActAction: 0=STOP, 1=OPEN, 2=CLOSE

// State machine for direction interlock and failsafe
enum class ActuatorState : uint8_t {
  IDLE,
  INTERLOCK_WAIT,    // Waiting for direction change delay
  FAILSAFE_CLOSING,  // Failsafe close in progress
};
ActuatorState g_actuatorState   = ActuatorState::IDLE;
uint32_t g_stateStartMs         = 0;
uint8_t  g_pendingAction        = 0;  // Action to execute after interlock

// -----------------------------
// FORWARD DECLARATIONS
// -----------------------------

void handleIncomingPackets();
void processActuatorCommand(const AgriUnoActuatorCommand &cmd);
void sendUnoStatus();
void updateActuatorState();
void updateStateMachine();
void stopActuator();
void openActuator();
void closeActuator();
void requestOpenActuator();
void requestCloseActuator();
uint16_t readCurrentmA();
uint8_t  readSoilPct();
uint16_t readBattmV();
void checkSafetyConditions();

// -----------------------------
// SETUP
// -----------------------------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n[UNO_NODE] Booting"));
  Serial.print(F("[UNO_NODE] Cluster="));
  Serial.print(MY_CLUSTER_ID);
  Serial.print(F(" Node="));
  Serial.println(MY_NODE_ID);

  // Initialize actuator pins
  pinMode(PIN_ACTUATOR_OPEN, OUTPUT);
  pinMode(PIN_ACTUATOR_CLOSE, OUTPUT);
  digitalWrite(PIN_ACTUATOR_OPEN, LOW);
  digitalWrite(PIN_ACTUATOR_CLOSE, LOW);

  // Initialize RF24
  AgriResult res = agri_rf24_init_common(
    radio,
    AgriRole::UNO_NODE,
    MY_NODE_ID,
    MY_CLUSTER_ID
  );

  if (res != AgriResult::OK) {
    Serial.println(F("[UNO_NODE] RF24 init failed"));
    g_statusFlags |= STATUS_FLAG_FAULT;
  } else {
    Serial.println(F("[UNO_NODE] RF24 init OK"));
  }

  g_lastCommMs = millis();
}

// -----------------------------
// MAIN LOOP
// -----------------------------

void loop() {
  uint32_t now = millis();

  // Handle incoming packets from ESP32 cluster
  handleIncomingPackets();

  // Update state machine (non-blocking interlocks and failsafe)
  updateStateMachine();

  // Update actuator state and safety checks
  updateActuatorState();
  checkSafetyConditions();

  // Periodic status reporting
  if (now - g_lastStatusMs >= STATUS_PERIOD_MS) {
    sendUnoStatus();
    g_lastStatusMs = now;
  }
}

// -----------------------------
// PACKET HANDLING
// -----------------------------

void handleIncomingPackets() {
  if (!radio.available()) {
    return;
  }

  uint8_t buffer[AGRI_RF24_PAYLOAD_MAX];
  uint8_t size = radio.getDynamicPayloadSize();
  radio.read(buffer, size);

  if (size < sizeof(AgriPacketHeader)) {
    Serial.println(F("[UNO_NODE] RX too short"));
    return;
  }

  AgriPacketHeader *hdr = reinterpret_cast<AgriPacketHeader*>(buffer);
  
  if (hdr->magic != AGRI_MAGIC_BYTE) {
    Serial.println(F("[UNO_NODE] BAD magic"));
    return;
  }

  // Update communication timestamp
  g_lastCommMs = millis();
  g_statusFlags |= STATUS_FLAG_COMMS_OK;

  AgriMsgType msgType = static_cast<AgriMsgType>(hdr->msgType);

  switch (msgType) {
    case AgriMsgType::UNO_CMD_ACTUATOR: {
      if (size < sizeof(AgriPacketHeader) + sizeof(AgriUnoActuatorCommand)) {
        Serial.println(F("[UNO_NODE] Bad CMD len"));
        break;
      }
      AgriUnoActuatorCommand *cmd = 
        reinterpret_cast<AgriUnoActuatorCommand*>(buffer + sizeof(AgriPacketHeader));
      processActuatorCommand(*cmd);
      break;
    }

    case AgriMsgType::UNO_PING: {
      Serial.println(F("[UNO_NODE] PING received"));
      // Reply with status
      sendUnoStatus();
      break;
    }

    case AgriMsgType::UNO_CMD_CONFIG: {
      Serial.println(F("[UNO_NODE] CONFIG received (not implemented)"));
      break;
    }

    default:
      Serial.print(F("[UNO_NODE] Unknown msgType: 0x"));
      Serial.println(hdr->msgType, HEX);
      break;
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

  switch (cmd.action) {
    case static_cast<uint8_t>(AgriActAction::ACT_STOP):
      stopActuator();
      g_actuatorState = ActuatorState::IDLE;
      break;

    case static_cast<uint8_t>(AgriActAction::ACT_OPEN):
      requestOpenActuator();
      break;

    case static_cast<uint8_t>(AgriActAction::ACT_CLOSE):
      requestCloseActuator();
      break;

    default:
      Serial.println(F("[UNO_NODE] Unknown action"));
      break;
  }
}

// -----------------------------
// NON-BLOCKING STATE MACHINE
// -----------------------------

void updateStateMachine() {
  uint32_t now = millis();

  switch (g_actuatorState) {
    case ActuatorState::INTERLOCK_WAIT:
      // Wait for direction interlock delay before executing pending action
      if ((now - g_stateStartMs) >= DIRECTION_DELAY_MS) {
        if (g_pendingAction == static_cast<uint8_t>(AgriActAction::ACT_OPEN)) {
          openActuator();
        } else if (g_pendingAction == static_cast<uint8_t>(AgriActAction::ACT_CLOSE)) {
          closeActuator();
        }
        g_actuatorState = ActuatorState::IDLE;
      }
      break;

    case ActuatorState::FAILSAFE_CLOSING:
      // Failsafe close for a brief period then stop
      if ((now - g_stateStartMs) >= 1000) {
        stopActuator();
        g_actuatorState = ActuatorState::IDLE;
        Serial.println(F("[UNO_NODE] Failsafe complete"));
      }
      break;

    case ActuatorState::IDLE:
    default:
      // Nothing to do
      break;
  }
}

void requestOpenActuator() {
  // Direction interlock: if closing, stop first and wait
  if (g_currentAction == static_cast<uint8_t>(AgriActAction::ACT_CLOSE)) {
    stopActuator();
    g_pendingAction = static_cast<uint8_t>(AgriActAction::ACT_OPEN);
    g_actuatorState = ActuatorState::INTERLOCK_WAIT;
    g_stateStartMs = millis();
    Serial.println(F("[UNO_NODE] Direction interlock - waiting to open"));
  } else {
    openActuator();
  }
}

void requestCloseActuator() {
  // Direction interlock: if opening, stop first and wait
  if (g_currentAction == static_cast<uint8_t>(AgriActAction::ACT_OPEN)) {
    stopActuator();
    g_pendingAction = static_cast<uint8_t>(AgriActAction::ACT_CLOSE);
    g_actuatorState = ActuatorState::INTERLOCK_WAIT;
    g_stateStartMs = millis();
    Serial.println(F("[UNO_NODE] Direction interlock - waiting to close"));
  } else {
    closeActuator();
  }
}

// -----------------------------
// ACTUATOR CONTROL
// -----------------------------

void stopActuator() {
  digitalWrite(PIN_ACTUATOR_OPEN, LOW);
  digitalWrite(PIN_ACTUATOR_CLOSE, LOW);
  g_currentAction = static_cast<uint8_t>(AgriActAction::ACT_STOP);
  g_actuatorStartMs = 0;
  
  g_statusFlags &= ~STATUS_FLAG_MOVING;
  Serial.println(F("[UNO_NODE] Actuator STOPPED"));
}

void openActuator() {
  digitalWrite(PIN_ACTUATOR_CLOSE, LOW);
  digitalWrite(PIN_ACTUATOR_OPEN, HIGH);
  g_currentAction = static_cast<uint8_t>(AgriActAction::ACT_OPEN);
  g_actuatorStartMs = millis();
  
  g_statusFlags |= STATUS_FLAG_MOVING;
  g_statusFlags &= ~(STATUS_FLAG_OPEN | STATUS_FLAG_CLOSED);
  Serial.println(F("[UNO_NODE] Actuator OPENING"));
}

void closeActuator() {
  digitalWrite(PIN_ACTUATOR_OPEN, LOW);
  digitalWrite(PIN_ACTUATOR_CLOSE, HIGH);
  g_currentAction = static_cast<uint8_t>(AgriActAction::ACT_CLOSE);
  g_actuatorStartMs = millis();
  
  g_statusFlags |= STATUS_FLAG_MOVING;
  g_statusFlags &= ~(STATUS_FLAG_OPEN | STATUS_FLAG_CLOSED);
  Serial.println(F("[UNO_NODE] Actuator CLOSING"));
}

// -----------------------------
// SAFETY CHECKS
// -----------------------------

void updateActuatorState() {
  if (g_currentAction == static_cast<uint8_t>(AgriActAction::ACT_STOP)) {
    return;
  }

  uint32_t now = millis();

  // Maximum runtime cutoff
  if ((now - g_actuatorStartMs) >= MAX_RUNTIME_MS) {
    Serial.println(F("[UNO_NODE] MAX RUNTIME - auto stop"));
    g_statusFlags |= STATUS_FLAG_TIMEOUT;
    stopActuator();
    return;
  }

  // Overcurrent protection
  uint16_t currentmA = readCurrentmA();
  if (currentmA > OVERCURRENT_THRESHOLD) {
    Serial.println(F("[UNO_NODE] OVERCURRENT - auto stop"));
    g_statusFlags |= STATUS_FLAG_OVERCURR | STATUS_FLAG_FAULT;
    stopActuator();
    return;
  }
}

void checkSafetyConditions() {
  uint32_t now = millis();

  // Failsafe on communication loss (only if not already in failsafe state)
  if ((now - g_lastCommMs) >= COMMS_TIMEOUT_MS) {
    g_statusFlags &= ~STATUS_FLAG_COMMS_OK;
    
    // Auto-close valve on comms loss for safety (non-blocking)
    if (g_currentAction != static_cast<uint8_t>(AgriActAction::ACT_STOP) &&
        g_actuatorState != ActuatorState::FAILSAFE_CLOSING) {
      Serial.println(F("[UNO_NODE] COMMS TIMEOUT - failsafe close"));
      closeActuator();
      g_actuatorState = ActuatorState::FAILSAFE_CLOSING;
      g_stateStartMs = millis();
    }
    
    // Reset comms timer to avoid repeated failsafe actions
    g_lastCommMs = now;
  }
}

// -----------------------------
// SENSOR READINGS
// -----------------------------

uint16_t readCurrentmA() {
  // Placeholder: convert ADC to mA based on current sensor
  int raw = analogRead(PIN_CURRENT_SENSE);
  // Example: 0.1V per amp, 5V/1024 = 4.88mV per step
  // Adjust calibration for actual sensor
  return (uint16_t)(raw * 5);  // Simplified conversion
}

uint8_t readSoilPct() {
  int raw = analogRead(PIN_SOIL_SENSOR);
  // Map 0-1023 to 0-100%
  return (uint8_t)map(raw, 0, 1023, 0, 100);
}

uint16_t readBattmV() {
  int raw = analogRead(PIN_BATT_SENSE);
  // Example: voltage divider with 2:1 ratio, 5V reference
  // Adjust for actual circuit
  return (uint16_t)((raw * 5000L * 2) / 1024);
}

// -----------------------------
// STATUS REPORTING
// -----------------------------

void sendUnoStatus() {
  uint8_t destAddr[6];
  agri_getClusterAddress(MY_CLUSTER_ID, destAddr);

  struct __attribute__((packed)) {
    AgriPacketHeader hdr;
    AgriUnoStatus    status;
  } pkt;

  agri_buildHeader(
    pkt.hdr,
    AgriMsgType::UNO_STATUS,
    AgriRole::UNO_NODE,
    AgriRole::ESP32_CLUSTER,
    MY_NODE_ID,
    MY_CLUSTER_ID
  );

  pkt.status.zoneLocalId = MY_NODE_ID;
  pkt.status.statusFlags = g_statusFlags;
  pkt.status.current_mA  = readCurrentmA();
  pkt.status.soilRaw     = analogRead(PIN_SOIL_SENSOR);
  pkt.status.soilPct     = readSoilPct();
  pkt.status.batt_mV     = readBattmV();
  pkt.status.lastCmdSeq  = g_lastCmdSeq;

  AgriResult res = agri_rf24_sendTo(
    radio,
    destAddr,
    &pkt,
    sizeof(pkt)
  );

  if (res != AgriResult::OK) {
    Serial.println(F("[UNO_NODE] sendStatus RF fail"));
  } else {
    Serial.println(F("[UNO_NODE] sent status"));
  }
}


