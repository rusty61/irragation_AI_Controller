// agrinet_rf24_comms/firmware/uno_node_full/uno_node_full.ino

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

// -----------------------------
// HW CONFIG – PER NODE
// -----------------------------

// RF24 pins on Arduino UNO
static const uint8_t PIN_RF24_CE  = 9;
static const uint8_t PIN_RF24_CSN = 10;

// Actuator + sensors
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
static const byte ADDR_UP[6]   = "CL001";  // status to ESP32
static const byte ADDR_DOWN[6] = "UN001";  // commands from ESP32

// -----------------------------
// SAFETY CONSTANTS
// -----------------------------

static const uint32_t MAX_RUNTIME_MS        = 600000UL;  // 10 minutes
static const uint32_t DIRECTION_DELAY_MS    = 100UL;     // between reverse moves
static const uint32_t COMMS_TIMEOUT_MS      = 60000UL;   // 60 s
static const uint16_t OVERCURRENT_THRESHOLD = 2000;      // mA 

static const uint32_t STATUS_PERIOD_MS      = 3000UL;    // 3 s

// -----------------------------
// RF24 INSTANCE
// -----------------------------

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// -----------------------------
// STATE
// -----------------------------
static uint8_t  g_currentAction    = static_cast<uint8_t>(AgriActAction::STOP);
static uint8_t  g_pendingAction    = static_cast<uint8_t>(AgriActAction::STOP);

static uint32_t g_actuatorStartMs  = 0;
static uint32_t g_interlockUntilMs = 0;

static uint32_t g_targetRunMs      = MAX_RUNTIME_MS;

static uint32_t g_lastStatusMs     = 0;
static uint32_t g_lastCommMs       = 0;

static uint16_t g_lastCmdSeq       = 0;
static uint16_t g_statusFlags      = 0;     

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
  
  // Start Relays OFF (Check logic: HIGH=Off or LOW=Off)
  digitalWrite(PIN_ACTUATOR_OPEN, LOW);
  digitalWrite(PIN_ACTUATOR_CLOSE, LOW);

  if (!radio.begin()) {
    Serial.println(F("[UNO_NODE] RF24 begin FAILED"));
    while(1);
  }
  Serial.println(F("[UNO_NODE] RF24 begin OK"));

  radio.setChannel(76);
  radio.setPALevel(RF24_PA_LOW); // Low power for desk testing
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 15);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);

  // Uplink + downlink
  // We use Pipe 0 for listening to avoid Pipe 1/2 prefix issues
  radio.openWritingPipe(ADDR_UP);
  radio.openReadingPipe(0, ADDR_DOWN); 
  
  radio.startListening();

  Serial.print(F("[UNO_NODE] Listening on: "));
  Serial.write((char*)ADDR_DOWN);
  Serial.println();

  uint32_t now = millis();
  g_lastCommMs   = now;
  g_lastStatusMs = now;
}

// -----------------------------
// MAIN LOOP
// -----------------------------

void loop() {
  uint32_t now = millis();

  // Handle RF24 RX
  handleIncomingPackets();

  // Drive actuator
  updateActuatorState();

  // Safety
  checkSafetyConditions();

  // Periodic status
  if (now - g_lastStatusMs >= STATUS_PERIOD_MS) {
    sendUnoStatus();
    g_lastStatusMs = now;
  }
}

// -----------------------------
// RF PACKET HANDLING
// -----------------------------

void handleIncomingPackets() {
  uint8_t pipe;
  while (radio.available(&pipe)) {
    uint8_t buffer[AGRI_RF24_PAYLOAD_MAX];
    uint8_t size = radio.getDynamicPayloadSize();
    if (size > sizeof(buffer)) size = sizeof(buffer);
    radio.read(buffer, size);

    Serial.print(F("[UNO_NODE] RX len="));
    Serial.println(size);

    if (size < sizeof(AgriPacketHeader)) {
       Serial.println(F("[UNO_NODE] RX too short"));
       continue;
    }

    AgriPacketHeader *hdr = reinterpret_cast<AgriPacketHeader*>(buffer);
    if (hdr->magic != AGRI_MAGIC_BYTE) {
       Serial.println(F("[UNO_NODE] RX BAD Magic"));
       continue;
    }

    g_lastCommMs = millis();

    AgriMsgType msgType = static_cast<AgriMsgType>(hdr->msgType);

    if (msgType == AgriMsgType::UNO_CMD_ACTUATOR) {
        if (size < sizeof(AgriPacketHeader) + sizeof(AgriUnoActuatorCommand)) {
          Serial.println(F("[UNO_NODE] CMD too short"));
        } else {
          AgriUnoActuatorCommand *cmd =
            reinterpret_cast<AgriUnoActuatorCommand*>(buffer + sizeof(AgriPacketHeader));
          processActuatorCommand(*cmd);
        }
    } else if (msgType == AgriMsgType::UNO_PING) {
        Serial.println(F("[UNO_NODE] PING?"));
        sendUnoStatus();
    }
  }
}

// -----------------------------
// ACTUATOR COMMAND PROCESSING
// -----------------------------

void processActuatorCommand(const AgriUnoActuatorCommand &cmd) {
  Serial.print(F("[UNO_NODE] PROCESS CMD zone="));
  Serial.print(cmd.zoneLocalId);
  Serial.print(F(" action="));
  Serial.println(cmd.action);

  g_lastCmdSeq = cmd.cmdSeq;
  uint8_t action = cmd.action;

  uint32_t requested = cmd.runMs;
  if (requested == 0 || requested > MAX_RUNTIME_MS) {
    g_targetRunMs = MAX_RUNTIME_MS;
  } else {
    g_targetRunMs = requested;
  }

  g_statusFlags &= ~STATUS_FLAG_FAULT;
  requestActuatorAction(action);
}

// -----------------------------
// ACTUATOR LOGIC
// -----------------------------

void requestActuatorAction(uint8_t newAction) {
  uint8_t stop   = static_cast<uint8_t>(AgriActAction::STOP);
  uint8_t openA  = static_cast<uint8_t>(AgriActAction::OPEN);
  uint8_t closeA = static_cast<uint8_t>(AgriActAction::CLOSE);

  if (newAction == stop) {
    applyActuatorOutputs(stop);
    g_currentAction = stop;
    g_pendingAction = stop;
    g_actuatorStartMs = 0;
    g_statusFlags &= ~STATUS_FLAG_MOVING;
    Serial.println(F("[UNO_NODE] STOP request"));
    return;
  }

  // Interlock
  if ((g_currentAction == openA && newAction == closeA) ||
      (g_currentAction == closeA && newAction == openA)) {
    Serial.println(F("[UNO_NODE] INTERLOCK reverse"));
    applyActuatorOutputs(stop);
    g_currentAction = stop;
    g_pendingAction = newAction;
    g_interlockUntilMs = millis() + DIRECTION_DELAY_MS;
    g_statusFlags &= ~STATUS_FLAG_MOVING;
    return;
  }

  g_pendingAction = newAction;
  g_interlockUntilMs = millis();
}

void applyActuatorOutputs(uint8_t action) {
  uint8_t stop   = static_cast<uint8_t>(AgriActAction::STOP);
  uint8_t openA  = static_cast<uint8_t>(AgriActAction::OPEN);
  uint8_t closeA = static_cast<uint8_t>(AgriActAction::CLOSE);

  // LOGIC: HIGH=ON, LOW=OFF (Adjust if relay inverted)
  if (action == stop) {
    digitalWrite(PIN_ACTUATOR_OPEN, LOW);
    digitalWrite(PIN_ACTUATOR_CLOSE, LOW);
    Serial.println(F("[UNO_NODE] Relay: STOP"));
  } else if (action == openA) {
    digitalWrite(PIN_ACTUATOR_CLOSE, LOW);
    digitalWrite(PIN_ACTUATOR_OPEN, HIGH);
    Serial.println(F("[UNO_NODE] Relay: OPEN"));
  } else if (action == closeA) {
    digitalWrite(PIN_ACTUATOR_OPEN, LOW);
    digitalWrite(PIN_ACTUATOR_CLOSE, HIGH);
    Serial.println(F("[UNO_NODE] Relay: CLOSE"));
  }
}

void updateActuatorState() {
  uint32_t now = millis();
  uint8_t stop = static_cast<uint8_t>(AgriActAction::STOP);

  if (g_pendingAction != stop && now >= g_interlockUntilMs && g_currentAction == stop) {
    g_currentAction   = g_pendingAction;
    g_pendingAction   = stop;
    g_actuatorStartMs = now;
    g_statusFlags |= STATUS_FLAG_MOVING;
    applyActuatorOutputs(g_currentAction);
  }

  if (g_currentAction != stop) {
    if ((now - g_actuatorStartMs) >= g_targetRunMs) {
      Serial.println(F("[UNO_NODE] Run done -> Stop"));
      g_statusFlags &= ~STATUS_FLAG_MOVING;
      applyActuatorOutputs(stop);
      g_currentAction = stop;
    }
  }
}

void checkSafetyConditions() {
  uint32_t now = millis();
  if (now - g_lastCommMs >= COMMS_TIMEOUT_MS) {
     if (g_currentAction != static_cast<uint8_t>(AgriActAction::CLOSE)) {
         Serial.println(F("[UNO_NODE] Comms Timeout -> CLOSE"));
         requestActuatorAction(static_cast<uint8_t>(AgriActAction::CLOSE));
         g_lastCommMs = now; 
     }
  }
}

// -----------------------------
// SENSORS & STATUS
// -----------------------------
uint16_t readCurrentmA() { return 0; }
uint8_t readSoilPct() { return 0; }
uint16_t readBattmV() { return 0; }

void sendUnoStatus() {
  struct { AgriPacketHeader hdr; AgriUnoStatus status; } pkt;

  pkt.hdr.magic   = AGRI_MAGIC_BYTE;
  pkt.hdr.srcRole = static_cast<uint8_t>(AgriRole::UNO_NODE);
  pkt.hdr.msgType = static_cast<uint8_t>(AgriMsgType::UNO_STATUS);
  pkt.hdr.dstRole = static_cast<uint8_t>(AgriRole::ESP32_CLUSTER);
  pkt.hdr.srcId   = MY_NODE_ID;
  pkt.hdr.dstId   = MY_CLUSTER_ID;
  pkt.hdr.seq     = ++g_seq;
  
  pkt.status.zoneLocalId = ZONE_LOCAL_ID;
  pkt.status.statusFlags = g_statusFlags;
  pkt.status.current_mA  = readCurrentmA();
  pkt.status.lastCmdSeq  = g_lastCmdSeq;

  // IMPORTANT: Restore Pipe 0 for Reading after Write
  radio.stopListening();
  bool ok = radio.write(&pkt, sizeof(pkt));
  
  // RESTORE PIPE 0 RX ADDRESS (UN001)
  radio.openReadingPipe(0, ADDR_DOWN);
  
  radio.startListening();

  if (ok) {
     // Serial.println(F("[UNO_NODE] Status sent OK"));
     g_lastCommMs = millis();
  } else {
     Serial.println(F("[UNO_NODE] Status TX Fail"));
  }
}
