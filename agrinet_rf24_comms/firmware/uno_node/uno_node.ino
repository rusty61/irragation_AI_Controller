// agrinet_rf24_comms/firmware/uno_node_full/uno_node_full.ino

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

// -----------------------------
// HW CONFIG – PER NODE
// -----------------------------
static const uint8_t PIN_RF24_CE  = 9;
static const uint8_t PIN_RF24_CSN = 10;
static const uint8_t PIN_ACTUATOR_OPEN  = 4;
static const uint8_t PIN_ACTUATOR_CLOSE = 5;
static const uint8_t PIN_CURRENT_SENSE  = A0;
static const uint8_t PIN_SOIL_SENSOR    = A1; // Deprecated
static const uint8_t PIN_BATT_SENSE     = A2;

static const uint8_t MY_CLUSTER_ID = 1;   
static const uint8_t MY_NODE_ID    = 1;   
static const uint8_t ZONE_LOCAL_ID = 1;   

static const byte ADDR_UP[6]   = "CL001";  
static const byte ADDR_DOWN[6] = "UN001";  

// -----------------------------
// SAFETY CONSTANTS
// -----------------------------
static const uint32_t MAX_RUNTIME_MS        = 600000UL;  
static const uint32_t DIRECTION_DELAY_MS    = 100UL;     
static const uint32_t COMMS_TIMEOUT_MS      = 60000UL;   
static const uint16_t OVERCURRENT_THRESHOLD = 2000;      
static const uint32_t STATUS_PERIOD_MS      = 3000UL;    

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// STATE
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

// FORWARD DECLS
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

// SETUP
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n[UNO_NODE] Booting (FULL RF24 + ACTUATOR)"));
  Serial.print(F("[UNO_NODE] Cluster=")); Serial.println(MY_CLUSTER_ID);

  pinMode(PIN_ACTUATOR_OPEN, OUTPUT);
  pinMode(PIN_ACTUATOR_CLOSE, OUTPUT);
  digitalWrite(PIN_ACTUATOR_OPEN, LOW);
  digitalWrite(PIN_ACTUATOR_CLOSE, LOW);

  if (!radio.begin()) {
    Serial.println(F("[UNO_NODE] RF24 begin FAILED"));
    while(1);
  }
  Serial.println(F("[UNO_NODE] RF24 begin OK"));

  radio.setChannel(AGRI_RF24_CHANNEL);
  radio.setDataRate(AGRI_RF24_DATARATE);
  radio.setPALevel(AGRI_RF24_PA_LEVEL);
  radio.setRetries(5, 15);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);

  radio.openWritingPipe(ADDR_UP);
  radio.openReadingPipe(0, ADDR_DOWN); 
  radio.startListening();

  Serial.println(F("[UNO_NODE] Listening..."));
  g_lastCommMs = millis();
}

void loop() {
  uint32_t now = millis();
  handleIncomingPackets();
  updateActuatorState();
  checkSafetyConditions();

  if (now - g_lastStatusMs >= STATUS_PERIOD_MS) {
    sendUnoStatus();
    g_lastStatusMs = now;
  }
}

void handleIncomingPackets() {
  uint8_t pipe;
  while (radio.available(&pipe)) {
    uint8_t buffer[AGRI_RF24_PAYLOAD_MAX];
    uint8_t size = radio.getDynamicPayloadSize();
    if (size > sizeof(buffer)) size = sizeof(buffer);
    radio.read(buffer, size);

    if (size < sizeof(AgriPacketHeader)) continue;

    AgriPacketHeader *hdr = reinterpret_cast<AgriPacketHeader*>(buffer);
    if (hdr->magic != AGRI_MAGIC_BYTE) continue;

    g_lastCommMs = millis();
    AgriMsgType msgType = static_cast<AgriMsgType>(hdr->msgType);

    if (msgType == AgriMsgType::UNO_CMD_ACTUATOR) {
        if (size >= sizeof(AgriPacketHeader) + sizeof(AgriUnoActuatorCommand)) {
          AgriUnoActuatorCommand *cmd = reinterpret_cast<AgriUnoActuatorCommand*>(buffer + sizeof(AgriPacketHeader));
          processActuatorCommand(*cmd);
        }
    } else if (msgType == AgriMsgType::UNO_PING) {
        sendUnoStatus();
    }
  }
}

void processActuatorCommand(const AgriUnoActuatorCommand &cmd) {
  Serial.print(F("[UNO_NODE] CMD Action=")); Serial.println(cmd.action);
  g_lastCmdSeq = cmd.cmdSeq;
  
  uint32_t requested = cmd.runMs;
  g_targetRunMs = (requested == 0 || requested > MAX_RUNTIME_MS) ? MAX_RUNTIME_MS : requested;

  g_statusFlags &= ~STATUS_FLAG_FAULT;
  requestActuatorAction(cmd.action);
}

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
    return;
  }

  if ((g_currentAction == openA && newAction == closeA) || (g_currentAction == closeA && newAction == openA)) {
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

  if (action == stop) {
    digitalWrite(PIN_ACTUATOR_OPEN, LOW);
    digitalWrite(PIN_ACTUATOR_CLOSE, LOW);
  } else if (action == openA) {
    digitalWrite(PIN_ACTUATOR_CLOSE, LOW);
    digitalWrite(PIN_ACTUATOR_OPEN, HIGH);
  } else if (action == closeA) {
    digitalWrite(PIN_ACTUATOR_OPEN, LOW);
    digitalWrite(PIN_ACTUATOR_CLOSE, HIGH);
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
         requestActuatorAction(static_cast<uint8_t>(AgriActAction::CLOSE));
         g_lastCommMs = now; 
     }
  }
}

// -----------------------------
// SENSORS
// -----------------------------
uint16_t readCurrentmA() { return 0; }

uint8_t readSoilPct() { 
  return 0; // HARDCODED ZERO - Sensor Hub handles this now
}

uint16_t readBattmV() { 
  // Map A2 (0-1023) to 0-5000mV
  long val = analogRead(PIN_BATT_SENSE);
  return (uint16_t)((val * 5000) / 1023);
}

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
  pkt.status.soilPct     = readSoilPct();
  pkt.status.soilRaw     = 0; 
  pkt.status.batt_mV     = readBattmV();
  pkt.status.lastCmdSeq  = g_lastCmdSeq;

  radio.stopListening();
  radio.openWritingPipe(ADDR_UP); // Restore correct TX pipe just in case
  radio.write(&pkt, sizeof(pkt));
  radio.openReadingPipe(0, ADDR_DOWN);
  radio.startListening();
}
