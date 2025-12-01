/**
 * @file uno_node.ino
 * @brief AgriNet UNO Actuator Node - RF24 Implementation
 * 
 * Bottom tier actuator node for the AgriNet irrigation system.
 * Handles valve/actuator control with safety logic.
 * 
 * Safety Features:
 *   - Maximum runtime safety cutoff
 *   - Actuator direction interlock
 *   - Failsafe on comms loss (auto-close)
 * 
 * Hardware:
 *   - Arduino UNO
 *   - NRF24L01+ module (CE=9, CSN=10)
 *   - Actuator driver (configurable pins)
 *   - Soil moisture sensor (A0)
 *   - Current sensor (A1)
 *   - Battery voltage divider (A2)
 */

#include <SPI.h>
#include <RF24.h>
#include "../../lib/agri_rf24/agri_rf24_common.h"

/*============================================================================
 * Configuration
 *==========================================================================*/

/** RF24 CE pin */
#define RF24_CE_PIN         9

/** RF24 CSN pin */
#define RF24_CSN_PIN        10

/** This node's cluster ID (1-16) */
#define MY_CLUSTER_ID       1

/** This node's local ID within cluster (1-8) */
#define MY_NODE_ID          1

/** Zone/actuator ID this node controls */
#define MY_ZONE_ID          1

/** Actuator control pins */
#define PIN_ACTUATOR_OPEN   5
#define PIN_ACTUATOR_CLOSE  6
#define PIN_ACTUATOR_EN     7

/** Sensor pins */
#define PIN_SOIL_SENSOR     A0
#define PIN_CURRENT_SENSOR  A1
#define PIN_BATT_VOLTAGE    A2

/** Maximum actuator runtime (ms) - safety cutoff */
#define MAX_ACTUATOR_RUNTIME_MS     30000

/** Comms timeout for failsafe (ms) */
#define COMMS_TIMEOUT_MS            60000

/** Status report interval (ms) */
#define STATUS_INTERVAL_MS          5000

/** Telemetry report interval (ms) */
#define TELEMETRY_INTERVAL_MS       10000

/*============================================================================
 * Globals
 *==========================================================================*/

// RF24 instance
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

// Addresses
uint8_t myAddress[6];
uint8_t clusterAddress[6];

// Sequence numbers
uint16_t txSeq = 0;
uint16_t lastRxSeq = 0;

// Actuator state
volatile bool actuatorMoving = false;
volatile uint8_t actuatorAction = static_cast<uint8_t>(AgriActuatorAction::ACTION_STOP);
volatile unsigned long actuatorStartTime = 0;
volatile uint32_t actuatorRuntime = 0;
uint16_t lastCmdSeq = 0;

// Status flags
uint16_t statusFlags = STATUS_FLAG_CLOSED;

// Timing
unsigned long lastCommsTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastTelemetryTime = 0;

// Sensor readings
uint16_t currentReading_mA = 0;
uint16_t soilRaw = 0;
uint8_t soilPct = 0;
uint16_t battVoltage_mV = 0;

/*============================================================================
 * Safety Functions
 *==========================================================================*/

/**
 * @brief Stop all actuator movement immediately
 */
void stopActuator()
{
    digitalWrite(PIN_ACTUATOR_OPEN, LOW);
    digitalWrite(PIN_ACTUATOR_CLOSE, LOW);
    digitalWrite(PIN_ACTUATOR_EN, LOW);
    actuatorMoving = false;
    actuatorAction = static_cast<uint8_t>(AgriActuatorAction::ACTION_STOP);
    statusFlags &= ~STATUS_FLAG_MOVING;
}

/**
 * @brief Execute failsafe - close valve and safe state
 */
void executeFailsafe()
{
    AGRI_DEBUG_PRINTLN(F("[SAFE] Executing failsafe - closing valve"));
    
    stopActuator();
    
    // Initiate close sequence
    digitalWrite(PIN_ACTUATOR_EN, HIGH);
    digitalWrite(PIN_ACTUATOR_CLOSE, HIGH);
    actuatorMoving = true;
    actuatorAction = static_cast<uint8_t>(AgriActuatorAction::ACTION_CLOSE);
    actuatorStartTime = millis();
    actuatorRuntime = MAX_ACTUATOR_RUNTIME_MS; // Full close
    statusFlags |= STATUS_FLAG_MOVING;
    statusFlags |= STATUS_FLAG_TIMEOUT;
    statusFlags &= ~STATUS_FLAG_COMMS_OK;
}

/**
 * @brief Check and enforce safety limits
 */
void checkSafetyLimits()
{
    unsigned long now = millis();
    
    // Check actuator runtime limit
    if (actuatorMoving) {
        if ((now - actuatorStartTime) >= actuatorRuntime || 
            (now - actuatorStartTime) >= MAX_ACTUATOR_RUNTIME_MS) {
            AGRI_DEBUG_PRINTLN(F("[SAFE] Runtime limit reached"));
            stopActuator();
            
            // Update status based on last action
            if (actuatorAction == static_cast<uint8_t>(AgriActuatorAction::ACTION_OPEN)) {
                statusFlags |= STATUS_FLAG_OPEN;
                statusFlags &= ~STATUS_FLAG_CLOSED;
            } else if (actuatorAction == static_cast<uint8_t>(AgriActuatorAction::ACTION_CLOSE)) {
                statusFlags |= STATUS_FLAG_CLOSED;
                statusFlags &= ~STATUS_FLAG_OPEN;
            }
            statusFlags |= STATUS_FLAG_ENDSTOP;
        }
    }
    
    // Check communications timeout
    if ((now - lastCommsTime) > COMMS_TIMEOUT_MS) {
        executeFailsafe();
        lastCommsTime = now; // Prevent repeated failsafe
    }
    
    // Check current limit (overcurrent protection)
    if (currentReading_mA > 2000) { // 2A limit
        AGRI_DEBUG_PRINTLN(F("[SAFE] Overcurrent detected"));
        stopActuator();
        statusFlags |= STATUS_FLAG_FAULT | STATUS_FLAG_OVERCURR;
    }
}

/*============================================================================
 * Actuator Control
 *==========================================================================*/

/**
 * @brief Process actuator command
 * @param cmd Actuator command structure
 */
void processActuatorCommand(const AgriUnoActuatorCommand &cmd)
{
    // Check if command is for this zone
    if (cmd.zoneLocalId != MY_ZONE_ID) {
        AGRI_DEBUG_PRINTLN(F("[CMD] Wrong zone ID"));
        return;
    }
    
    // Check for duplicate command
    if (cmd.cmdSeq == lastCmdSeq) {
        AGRI_DEBUG_PRINTLN(F("[CMD] Duplicate command"));
        return;
    }
    
    lastCmdSeq = cmd.cmdSeq;
    
    AGRI_DEBUG_PRINT(F("[CMD] Action: "));
    AGRI_DEBUG_PRINTLN(cmd.action);
    
    // Stop current movement first (direction interlock)
    // Brief delay is intentional to prevent motor H-bridge shoot-through
    // This 50ms delay is acceptable as command processing is not time-critical
    stopActuator();
    delay(50);
    
    switch (static_cast<AgriActuatorAction>(cmd.action)) {
        case AgriActuatorAction::ACTION_OPEN:
            // Only open if not already fully open
            if (!(statusFlags & STATUS_FLAG_OPEN)) {
                digitalWrite(PIN_ACTUATOR_EN, HIGH);
                digitalWrite(PIN_ACTUATOR_OPEN, HIGH);
                actuatorMoving = true;
                actuatorAction = cmd.action;
                actuatorStartTime = millis();
                actuatorRuntime = min(cmd.runMs, (uint32_t)MAX_ACTUATOR_RUNTIME_MS);
                statusFlags |= STATUS_FLAG_MOVING;
                statusFlags &= ~(STATUS_FLAG_OPEN | STATUS_FLAG_CLOSED | STATUS_FLAG_ENDSTOP);
            }
            break;
            
        case AgriActuatorAction::ACTION_CLOSE:
            // Only close if not already fully closed
            if (!(statusFlags & STATUS_FLAG_CLOSED)) {
                digitalWrite(PIN_ACTUATOR_EN, HIGH);
                digitalWrite(PIN_ACTUATOR_CLOSE, HIGH);
                actuatorMoving = true;
                actuatorAction = cmd.action;
                actuatorStartTime = millis();
                actuatorRuntime = min(cmd.runMs, (uint32_t)MAX_ACTUATOR_RUNTIME_MS);
                statusFlags |= STATUS_FLAG_MOVING;
                statusFlags &= ~(STATUS_FLAG_OPEN | STATUS_FLAG_CLOSED | STATUS_FLAG_ENDSTOP);
            }
            break;
            
        case AgriActuatorAction::ACTION_STOP:
        default:
            // Already stopped above
            break;
    }
}

/*============================================================================
 * Sensor Reading
 *==========================================================================*/

/**
 * @brief Read all sensors
 */
void readSensors()
{
    // Read soil moisture
    soilRaw = analogRead(PIN_SOIL_SENSOR);
    soilPct = map(soilRaw, 0, 1023, 0, 100);
    
    // Read current sensor (assuming ACS712 5A module or similar)
    // Center point is 512 (2.5V at 5V reference for 10-bit ADC)
    // Adjust scaling factor based on sensor sensitivity (e.g., ACS712-5A = 185mV/A)
    int rawCurrent = analogRead(PIN_CURRENT_SENSOR);
    currentReading_mA = (uint16_t)(abs(rawCurrent - 512) * 26); // ~26mA per ADC step for 5A module
    
    // Read battery voltage (assuming voltage divider)
    int rawBatt = analogRead(PIN_BATT_VOLTAGE);
    // Convert to mV (adjust for your voltage divider ratio)
    battVoltage_mV = rawBatt * 5000UL / 1023 * 2; // 2:1 divider assumed
}

/*============================================================================
 * Communication Functions
 *==========================================================================*/

/**
 * @brief Send status to cluster
 */
void sendStatus()
{
    AgriPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    agri_initPacketHeader(&packet.header,
                          static_cast<uint8_t>(AgriMsgTypeUno::UNO_STATUS),
                          AgriRole::ROLE_UNO_NODE,
                          AgriRole::ROLE_ESP32_CLUSTER,
                          MY_NODE_ID, MY_CLUSTER_ID, txSeq++);
    
    packet.payload.unoStatus.zoneLocalId = MY_ZONE_ID;
    packet.payload.unoStatus.statusFlags = statusFlags;
    packet.payload.unoStatus.current_mA = currentReading_mA;
    packet.payload.unoStatus.soilRaw = soilRaw;
    packet.payload.unoStatus.soilPct = soilPct;
    packet.payload.unoStatus.batt_mV = battVoltage_mV;
    packet.payload.unoStatus.lastCmdSeq = lastCmdSeq;
    
    AgriResult result = agri_rf24_send(radio, clusterAddress, &packet,
                                        sizeof(AgriPacketHeader) + sizeof(AgriUnoStatus));
    
    if (result == AgriResult::OK) {
        AGRI_DEBUG_PRINTLN(F("[TX] Status sent"));
    } else {
        AGRI_DEBUG_PRINTLN(F("[TX] Status send failed"));
    }
}

/**
 * @brief Send command acknowledgment
 * @param cmdSeq Command sequence being acknowledged
 */
void sendCmdAck(uint16_t cmdSeq)
{
    AgriPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    agri_initPacketHeader(&packet.header,
                          static_cast<uint8_t>(AgriMsgTypeUno::UNO_ACK_CMD),
                          AgriRole::ROLE_UNO_NODE,
                          AgriRole::ROLE_ESP32_CLUSTER,
                          MY_NODE_ID, MY_CLUSTER_ID, txSeq++);
    
    // Use status payload for ACK with cmdSeq
    packet.payload.unoStatus.zoneLocalId = MY_ZONE_ID;
    packet.payload.unoStatus.lastCmdSeq = cmdSeq;
    packet.payload.unoStatus.statusFlags = statusFlags;
    
    agri_rf24_send(radio, clusterAddress, &packet,
                   sizeof(AgriPacketHeader) + sizeof(AgriUnoStatus));
}

/**
 * @brief Process received packets
 */
void processRxPackets()
{
    AgriPacket packet;
    uint8_t len = sizeof(packet);
    
    AgriResult result = agri_rf24_recv(radio, &packet, &len, 0);
    
    if (result != AgriResult::OK) {
        return; // No packet or error
    }
    
    // Validate header
    if (!agri_validatePacketHeader(&packet.header)) {
        AGRI_DEBUG_PRINTLN(F("[RX] Invalid header"));
        return;
    }
    
    // Check if packet is for us
    if (packet.header.dstId != MY_NODE_ID) {
        return;
    }
    
    // Duplicate detection
    if (packet.header.seq == lastRxSeq) {
        AGRI_DEBUG_PRINTLN(F("[RX] Duplicate packet"));
        return;
    }
    lastRxSeq = packet.header.seq;
    
    // Update comms timestamp
    lastCommsTime = millis();
    statusFlags |= STATUS_FLAG_COMMS_OK;
    
    // Process based on message type
    switch (static_cast<AgriMsgTypeUno>(packet.header.msgType)) {
        case AgriMsgTypeUno::UNO_CMD_ACTUATOR:
            AGRI_DEBUG_PRINTLN(F("[RX] Actuator command"));
            processActuatorCommand(packet.payload.actuatorCmd);
            sendCmdAck(packet.payload.actuatorCmd.cmdSeq);
            break;
            
        case AgriMsgTypeUno::UNO_PING:
            AGRI_DEBUG_PRINTLN(F("[RX] Ping received"));
            sendStatus();
            break;
            
        case AgriMsgTypeUno::UNO_CMD_CONFIG:
            AGRI_DEBUG_PRINTLN(F("[RX] Config command (not implemented)"));
            break;
            
        default:
            AGRI_DEBUG_PRINT(F("[RX] Unknown msg type: "));
            AGRI_DEBUG_PRINTHEX(packet.header.msgType);
            AGRI_DEBUG_PRINTLN("");
            break;
    }
}

/*============================================================================
 * Setup and Loop
 *==========================================================================*/

void setup()
{
    // Initialize serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Wait up to 3s for serial
    
    AGRI_DEBUG_PRINTLN(F("\n=== AgriNet UNO Node ==="));
    AGRI_DEBUG_PRINT(F("Cluster: ")); AGRI_DEBUG_PRINTLN(MY_CLUSTER_ID);
    AGRI_DEBUG_PRINT(F("Node: ")); AGRI_DEBUG_PRINTLN(MY_NODE_ID);
    
    // Configure actuator pins
    pinMode(PIN_ACTUATOR_OPEN, OUTPUT);
    pinMode(PIN_ACTUATOR_CLOSE, OUTPUT);
    pinMode(PIN_ACTUATOR_EN, OUTPUT);
    stopActuator(); // Ensure safe state
    
    // Configure sensor pins
    pinMode(PIN_SOIL_SENSOR, INPUT);
    pinMode(PIN_CURRENT_SENSOR, INPUT);
    pinMode(PIN_BATT_VOLTAGE, INPUT);
    
    // Initialize SPI
    SPI.begin();
    
    // Initialize RF24
    AgriResult result = agri_rf24_init_common(radio, AgriRole::ROLE_UNO_NODE);
    if (result != AgriResult::OK) {
        AGRI_DEBUG_PRINTLN(F("[ERR] RF24 init failed!"));
        // Flash LED or indicate error
        while (1) {
            delay(1000);
        }
    }
    
    // Set up addresses
    agri_getUnoNodeAddress(MY_CLUSTER_ID, MY_NODE_ID, myAddress);
    agri_getClusterAddress(MY_CLUSTER_ID, clusterAddress);
    
    AGRI_DEBUG_PRINT(F("My addr: ")); AGRI_DEBUG_PRINTLN((char*)myAddress);
    AGRI_DEBUG_PRINT(F("Cluster addr: ")); AGRI_DEBUG_PRINTLN((char*)clusterAddress);
    
    // Open reading pipe on our address
    radio.openReadingPipe(1, myAddress);
    radio.startListening();
    
    // Initialize timing
    lastCommsTime = millis();
    lastStatusTime = millis();
    lastTelemetryTime = millis();
    
    AGRI_DEBUG_PRINTLN(F("[OK] Node initialized"));
}

void loop()
{
    unsigned long now = millis();
    
    // Read sensors
    readSensors();
    
    // Check safety limits
    checkSafetyLimits();
    
    // Process incoming packets
    processRxPackets();
    
    // Send periodic status
    if ((now - lastStatusTime) >= STATUS_INTERVAL_MS) {
        sendStatus();
        lastStatusTime = now;
    }
    
    // Small delay to prevent watchdog issues
    delay(10);
}
