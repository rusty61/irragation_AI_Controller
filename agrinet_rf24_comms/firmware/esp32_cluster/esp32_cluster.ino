/**
 * @file esp32_cluster.ino
 * @brief AgriNet ESP32 Cluster Node - RF24 Implementation
 * 
 * Middle tier cluster node for the AgriNet irrigation system.
 * Acts as field hub managing multiple UNO actuator nodes.
 * 
 * Features:
 *   - Manages up to 8 UNO actuator nodes
 *   - Aggregates telemetry for UNO_Q master
 *   - Maintains local fallback schedule
 *   - Tracks node online/offline status
 * 
 * Hardware:
 *   - ESP32
 *   - NRF24L01+ module (CE=4, CSN=5)
 *   - Optional: Local sensors, display
 */

#include <SPI.h>
#include <RF24.h>
#include "../../lib/agri_rf24/agri_rf24_common.h"

/*============================================================================
 * Configuration
 *==========================================================================*/

/** RF24 CE pin */
#define RF24_CE_PIN         4

/** RF24 CSN pin */
#define RF24_CSN_PIN        5

/** This cluster's ID (1-16) */
#define MY_CLUSTER_ID       1

/** Number of UNO nodes in this cluster */
#define NUM_NODES           2

/** Node timeout for marking offline (ms) */
#define NODE_TIMEOUT_MS     30000

/** Heartbeat interval to master (ms) */
#define HEARTBEAT_INTERVAL_MS   10000

/** Telemetry interval to master (ms) */
#define TELEMETRY_INTERVAL_MS   15000

/** Status poll interval for nodes (ms) */
#define NODE_POLL_INTERVAL_MS   5000

/** Sensor pins */
#define PIN_BATT_VOLTAGE    34
#define PIN_PANEL_VOLTAGE   35
#define PIN_PUMP_CURRENT    32

/*============================================================================
 * Types
 *==========================================================================*/

/**
 * @brief Node tracking structure
 */
struct NodeInfo {
    uint8_t nodeId;
    uint8_t address[6];
    bool online;
    unsigned long lastContact;
    AgriUnoStatus lastStatus;
};

/*============================================================================
 * Globals
 *==========================================================================*/

// RF24 instance
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

// Addresses
uint8_t myAddress[6];
uint8_t masterAddress[6];

// Node tracking
NodeInfo nodes[AGRI_MAX_NODES_PER_CLUSTER];
uint8_t numNodes = NUM_NODES;

// Zone snapshots for telemetry
AgriZoneSnapshot zoneSnapshots[AGRI_MAX_ZONES_PER_CLUSTER];

// Local schedule (fallback)
AgriClusterSchedule localSchedule;
bool hasLocalSchedule = false;

// Sequence numbers
uint16_t txSeq = 0;
uint16_t lastRxSeq = 0;
uint16_t cmdSeq = 0;

// Timing
unsigned long lastHeartbeatTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastNodePollTime = 0;

// Sensor readings
uint16_t battVoltage_mV = 0;
uint16_t panelVoltage_mV = 0;
uint16_t pumpCurrent_mA = 0;

/*============================================================================
 * Node Management
 *==========================================================================*/

/**
 * @brief Initialize node tracking
 */
void initNodes()
{
    for (uint8_t i = 0; i < AGRI_MAX_NODES_PER_CLUSTER; i++) {
        if (i < numNodes) {
            nodes[i].nodeId = i + 1;
            agri_getUnoNodeAddress(MY_CLUSTER_ID, i + 1, nodes[i].address);
            nodes[i].online = false;
            nodes[i].lastContact = 0;
            memset(&nodes[i].lastStatus, 0, sizeof(AgriUnoStatus));
        } else {
            nodes[i].nodeId = 0;
        }
    }
}

/**
 * @brief Update node status based on received data
 * @param nodeId Node identifier
 * @param status Status data from node
 */
void updateNodeStatus(uint8_t nodeId, const AgriUnoStatus &status)
{
    for (uint8_t i = 0; i < numNodes; i++) {
        if (nodes[i].nodeId == nodeId) {
            nodes[i].online = true;
            nodes[i].lastContact = millis();
            memcpy(&nodes[i].lastStatus, &status, sizeof(AgriUnoStatus));
            
            AGRI_DEBUG_PRINT(F("[NODE] Updated node "));
            AGRI_DEBUG_PRINTLN(nodeId);
            return;
        }
    }
}

/**
 * @brief Check node timeouts and mark offline
 */
void checkNodeTimeouts()
{
    unsigned long now = millis();
    
    for (uint8_t i = 0; i < numNodes; i++) {
        if (nodes[i].nodeId != 0 && nodes[i].online) {
            if ((now - nodes[i].lastContact) > NODE_TIMEOUT_MS) {
                AGRI_DEBUG_PRINT(F("[NODE] Node "));
                AGRI_DEBUG_PRINT(nodes[i].nodeId);
                AGRI_DEBUG_PRINTLN(F(" marked offline"));
                nodes[i].online = false;
            }
        }
    }
}

/**
 * @brief Build zone snapshots from node data
 */
void buildZoneSnapshots()
{
    for (uint8_t i = 0; i < AGRI_MAX_ZONES_PER_CLUSTER; i++) {
        if (i < numNodes && nodes[i].nodeId != 0) {
            zoneSnapshots[i].zoneId = nodes[i].lastStatus.zoneLocalId;
            zoneSnapshots[i].soilPct = nodes[i].lastStatus.soilPct;
            
            // Determine zone state from status flags
            if (!nodes[i].online) {
                zoneSnapshots[i].zoneState = static_cast<uint8_t>(AgriZoneState::ZONE_FAULT);
            } else if (nodes[i].lastStatus.statusFlags & STATUS_FLAG_FAULT) {
                zoneSnapshots[i].zoneState = static_cast<uint8_t>(AgriZoneState::ZONE_FAULT);
            } else if (nodes[i].lastStatus.statusFlags & STATUS_FLAG_OPEN) {
                zoneSnapshots[i].zoneState = static_cast<uint8_t>(AgriZoneState::ZONE_IRRIGATING);
            } else {
                zoneSnapshots[i].zoneState = static_cast<uint8_t>(AgriZoneState::ZONE_IDLE);
            }
            
            // lastRun_s would need actual tracking
            zoneSnapshots[i].lastRun_s = 0;
        } else {
            memset(&zoneSnapshots[i], 0, sizeof(AgriZoneSnapshot));
        }
    }
}

/*============================================================================
 * Sensor Reading
 *==========================================================================*/

/**
 * @brief Read cluster sensors
 */
void readSensors()
{
    // Read battery voltage (adjust for your voltage divider)
    int rawBatt = analogRead(PIN_BATT_VOLTAGE);
    battVoltage_mV = rawBatt * 3300 / 4095 * 4; // 4:1 divider for 12V
    
    // Read solar panel voltage
    int rawPanel = analogRead(PIN_PANEL_VOLTAGE);
    panelVoltage_mV = rawPanel * 3300 / 4095 * 6; // 6:1 divider for 18V
    
    // Read pump current (assuming ACS712 or similar current sensor)
    // Center point is 2048 (1.65V at 3.3V reference for 12-bit ADC)
    // Adjust scaling factor based on sensor sensitivity
    int rawCurrent = analogRead(PIN_PUMP_CURRENT);
    pumpCurrent_mA = (uint16_t)(abs(rawCurrent - 2048) * 8); // ~8mA per ADC step (adjust for sensor)
}

/*============================================================================
 * Communication Functions - To Master (UNO_Q)
 *==========================================================================*/

/**
 * @brief Send telemetry to master
 */
void sendTelemetryToMaster()
{
    buildZoneSnapshots();
    
    AgriPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    agri_initPacketHeader(&packet.header,
                          static_cast<uint8_t>(AgriMsgTypeCluster::CLUSTER_TELEMETRY),
                          AgriRole::ROLE_ESP32_CLUSTER,
                          AgriRole::ROLE_UNO_Q_MMC,
                          MY_CLUSTER_ID, 0, txSeq++);
    
    packet.payload.clusterTelem.clusterId = MY_CLUSTER_ID;
    packet.payload.clusterTelem.seq = txSeq;
    packet.payload.clusterTelem.batt_mV = battVoltage_mV;
    packet.payload.clusterTelem.panel_mV = panelVoltage_mV;
    packet.payload.clusterTelem.pumpCurrent_mA = pumpCurrent_mA;
    packet.payload.clusterTelem.numZones = numNodes;
    memcpy(packet.payload.clusterTelem.zones, zoneSnapshots, 
           sizeof(AgriZoneSnapshot) * AGRI_MAX_ZONES_PER_CLUSTER);
    
    AgriResult result = agri_rf24_send(radio, masterAddress, &packet,
                                        sizeof(AgriPacketHeader) + sizeof(AgriClusterTelemetry));
    
    if (result == AgriResult::OK) {
        AGRI_DEBUG_PRINTLN(F("[TX->M] Telemetry sent"));
    } else {
        AGRI_DEBUG_PRINTLN(F("[TX->M] Telemetry send failed"));
    }
}

/**
 * @brief Send heartbeat to master
 */
void sendHeartbeatToMaster()
{
    AgriPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    agri_initPacketHeader(&packet.header,
                          static_cast<uint8_t>(AgriMsgTypeCluster::CLUSTER_HEARTBEAT),
                          AgriRole::ROLE_ESP32_CLUSTER,
                          AgriRole::ROLE_UNO_Q_MMC,
                          MY_CLUSTER_ID, 0, txSeq++);
    
    AgriResult result = agri_rf24_send(radio, masterAddress, &packet,
                                        sizeof(AgriPacketHeader));
    
    if (result == AgriResult::OK) {
        AGRI_DEBUG_PRINTLN(F("[TX->M] Heartbeat sent"));
    }
}

/**
 * @brief Send cluster status to master
 */
void sendStatusToMaster()
{
    AgriPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    agri_initPacketHeader(&packet.header,
                          static_cast<uint8_t>(AgriMsgTypeCluster::CLUSTER_STATUS),
                          AgriRole::ROLE_ESP32_CLUSTER,
                          AgriRole::ROLE_UNO_Q_MMC,
                          MY_CLUSTER_ID, 0, txSeq++);
    
    // Include basic telemetry in status
    packet.payload.clusterTelem.clusterId = MY_CLUSTER_ID;
    packet.payload.clusterTelem.batt_mV = battVoltage_mV;
    packet.payload.clusterTelem.numZones = numNodes;
    
    agri_rf24_send(radio, masterAddress, &packet,
                   sizeof(AgriPacketHeader) + sizeof(AgriClusterTelemetry));
}

/*============================================================================
 * Communication Functions - To Nodes (UNO)
 *==========================================================================*/

/**
 * @brief Send actuator command to a node
 * @param nodeId Target node ID
 * @param zoneId Zone to control
 * @param action Action to perform
 * @param runMs Runtime in milliseconds
 */
void sendActuatorCommand(uint8_t nodeId, uint8_t zoneId, 
                         AgriActuatorAction action, uint32_t runMs)
{
    // Find node
    NodeInfo *node = nullptr;
    for (uint8_t i = 0; i < numNodes; i++) {
        if (nodes[i].nodeId == nodeId) {
            node = &nodes[i];
            break;
        }
    }
    
    if (node == nullptr) {
        AGRI_DEBUG_PRINTLN(F("[TX->N] Node not found"));
        return;
    }
    
    AgriPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    agri_initPacketHeader(&packet.header,
                          static_cast<uint8_t>(AgriMsgTypeUno::UNO_CMD_ACTUATOR),
                          AgriRole::ROLE_ESP32_CLUSTER,
                          AgriRole::ROLE_UNO_NODE,
                          MY_CLUSTER_ID, nodeId, txSeq++);
    
    packet.payload.actuatorCmd.zoneLocalId = zoneId;
    packet.payload.actuatorCmd.action = static_cast<uint8_t>(action);
    packet.payload.actuatorCmd.runMs = runMs;
    packet.payload.actuatorCmd.cmdSeq = ++cmdSeq;
    
    AgriResult result = agri_rf24_send(radio, node->address, &packet,
                                        sizeof(AgriPacketHeader) + sizeof(AgriUnoActuatorCommand));
    
    if (result == AgriResult::OK) {
        AGRI_DEBUG_PRINT(F("[TX->N] Cmd sent to node "));
        AGRI_DEBUG_PRINTLN(nodeId);
    }
}

/**
 * @brief Send ping to a node
 * @param nodeId Target node ID
 */
void pingNode(uint8_t nodeId)
{
    NodeInfo *node = nullptr;
    for (uint8_t i = 0; i < numNodes; i++) {
        if (nodes[i].nodeId == nodeId) {
            node = &nodes[i];
            break;
        }
    }
    
    if (node == nullptr) return;
    
    AgriPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    agri_initPacketHeader(&packet.header,
                          static_cast<uint8_t>(AgriMsgTypeUno::UNO_PING),
                          AgriRole::ROLE_ESP32_CLUSTER,
                          AgriRole::ROLE_UNO_NODE,
                          MY_CLUSTER_ID, nodeId, txSeq++);
    
    agri_rf24_send(radio, node->address, &packet, sizeof(AgriPacketHeader));
}

/**
 * @brief Poll all nodes for status
 * @note Brief delay between pings allows RF24 auto-retry to complete
 *       and prevents channel congestion. This function is called
 *       infrequently (every NODE_POLL_INTERVAL_MS) so impact is minimal.
 */
void pollAllNodes()
{
    for (uint8_t i = 0; i < numNodes; i++) {
        if (nodes[i].nodeId != 0) {
            pingNode(nodes[i].nodeId);
            delay(50); // Allow RF24 auto-retry cycle to complete
        }
    }
}

/*============================================================================
 * Packet Processing
 *==========================================================================*/

/**
 * @brief Process received packets
 */
void processRxPackets()
{
    AgriPacket packet;
    uint8_t len = sizeof(packet);
    
    AgriResult result = agri_rf24_recv(radio, &packet, &len, 0);
    
    if (result != AgriResult::OK) {
        return;
    }
    
    if (!agri_validatePacketHeader(&packet.header)) {
        AGRI_DEBUG_PRINTLN(F("[RX] Invalid header"));
        return;
    }
    
    // Duplicate detection
    if (packet.header.seq == lastRxSeq) {
        AGRI_DEBUG_PRINTLN(F("[RX] Duplicate"));
        return;
    }
    lastRxSeq = packet.header.seq;
    
    // Handle based on source role
    if (packet.header.srcRole == static_cast<uint8_t>(AgriRole::ROLE_UNO_NODE)) {
        // Message from UNO node
        processNodeMessage(packet);
    } else if (packet.header.srcRole == static_cast<uint8_t>(AgriRole::ROLE_UNO_Q_MMC)) {
        // Message from master
        processMasterMessage(packet);
    }
}

/**
 * @brief Process message from UNO node
 * @param packet Received packet
 */
void processNodeMessage(const AgriPacket &packet)
{
    uint8_t nodeId = packet.header.srcId;
    
    switch (static_cast<AgriMsgTypeUno>(packet.header.msgType)) {
        case AgriMsgTypeUno::UNO_STATUS:
            AGRI_DEBUG_PRINT(F("[RX<-N] Status from "));
            AGRI_DEBUG_PRINTLN(nodeId);
            updateNodeStatus(nodeId, packet.payload.unoStatus);
            break;
            
        case AgriMsgTypeUno::UNO_ACK_CMD:
            AGRI_DEBUG_PRINT(F("[RX<-N] ACK from "));
            AGRI_DEBUG_PRINTLN(nodeId);
            updateNodeStatus(nodeId, packet.payload.unoStatus);
            break;
            
        case AgriMsgTypeUno::UNO_TELEMETRY:
            AGRI_DEBUG_PRINT(F("[RX<-N] Telemetry from "));
            AGRI_DEBUG_PRINTLN(nodeId);
            updateNodeStatus(nodeId, packet.payload.unoStatus);
            break;
            
        case AgriMsgTypeUno::UNO_FAULT:
            AGRI_DEBUG_PRINT(F("[RX<-N] FAULT from "));
            AGRI_DEBUG_PRINTLN(nodeId);
            updateNodeStatus(nodeId, packet.payload.unoStatus);
            // TODO: Report fault to master
            break;
            
        default:
            AGRI_DEBUG_PRINTLN(F("[RX<-N] Unknown message"));
            break;
    }
}

/**
 * @brief Process message from master
 * @param packet Received packet
 */
void processMasterMessage(const AgriPacket &packet)
{
    switch (static_cast<AgriMsgTypeCluster>(packet.header.msgType)) {
        case AgriMsgTypeCluster::CLUSTER_PING:
            AGRI_DEBUG_PRINTLN(F("[RX<-M] Ping"));
            sendStatusToMaster();
            break;
            
        case AgriMsgTypeCluster::CLUSTER_SET_SCHED:
            AGRI_DEBUG_PRINTLN(F("[RX<-M] Schedule update"));
            memcpy(&localSchedule, &packet.payload.clusterSched, 
                   sizeof(AgriClusterSchedule));
            hasLocalSchedule = true;
            break;
            
        case AgriMsgTypeCluster::CLUSTER_CMD_ZONE: {
            AGRI_DEBUG_PRINTLN(F("[RX<-M] Zone command"));
            // Extract command and forward to appropriate node
            // Note: In this implementation, zoneLocalId maps directly to nodeId
            // Modify this mapping if your zone-to-node relationship differs
            const AgriUnoActuatorCommand &cmd = packet.payload.actuatorCmd;
            uint8_t targetNodeId = cmd.zoneLocalId; // Assumes 1:1 zone-to-node mapping
            sendActuatorCommand(targetNodeId, cmd.zoneLocalId,
                               static_cast<AgriActuatorAction>(cmd.action),
                               cmd.runMs);
            break;
        }
            
        case AgriMsgTypeCluster::CLUSTER_CONFIG:
            AGRI_DEBUG_PRINTLN(F("[RX<-M] Config (not implemented)"));
            break;
            
        default:
            AGRI_DEBUG_PRINTLN(F("[RX<-M] Unknown message"));
            break;
    }
}

/*============================================================================
 * Local Schedule Processing
 *==========================================================================*/

/**
 * @brief Process local fallback schedule
 */
void processLocalSchedule()
{
    if (!hasLocalSchedule) return;
    
    // Get current time (would need RTC or NTP)
    // For now, this is a placeholder
    uint32_t currentTime = millis() / 1000; // Simplified
    
    for (uint8_t i = 0; i < localSchedule.numEntries; i++) {
        AgriZoneSchedule &entry = localSchedule.entries[i];
        
        // Check if schedule entry should be active
        if (entry.mode == static_cast<uint8_t>(AgriScheduleMode::MODE_AUTO)) {
            // Auto mode - check time windows
            // This would need proper time handling
        } else if (entry.mode == static_cast<uint8_t>(AgriScheduleMode::MODE_FORCE_ON)) {
            sendActuatorCommand(entry.zoneId, entry.zoneId,
                               AgriActuatorAction::ACTION_OPEN,
                               entry.duration_s * 1000);
        } else if (entry.mode == static_cast<uint8_t>(AgriScheduleMode::MODE_FORCE_OFF)) {
            sendActuatorCommand(entry.zoneId, entry.zoneId,
                               AgriActuatorAction::ACTION_CLOSE, 0);
        }
    }
}

/*============================================================================
 * Setup and Loop
 *==========================================================================*/

void setup()
{
    // Initialize serial
    Serial.begin(115200);
    delay(1000);
    
    AGRI_DEBUG_PRINTLN(F("\n=== AgriNet ESP32 Cluster ==="));
    AGRI_DEBUG_PRINT(F("Cluster ID: ")); AGRI_DEBUG_PRINTLN(MY_CLUSTER_ID);
    AGRI_DEBUG_PRINT(F("Nodes: ")); AGRI_DEBUG_PRINTLN(NUM_NODES);
    
    // Configure sensor pins
    pinMode(PIN_BATT_VOLTAGE, INPUT);
    pinMode(PIN_PANEL_VOLTAGE, INPUT);
    pinMode(PIN_PUMP_CURRENT, INPUT);
    
    // Initialize SPI (ESP32 default pins)
    SPI.begin();
    
    // Initialize RF24
    AgriResult result = agri_rf24_init_common(radio, AgriRole::ROLE_ESP32_CLUSTER);
    if (result != AgriResult::OK) {
        AGRI_DEBUG_PRINTLN(F("[ERR] RF24 init failed!"));
        while (1) {
            delay(1000);
        }
    }
    
    // Set up addresses
    agri_getClusterAddress(MY_CLUSTER_ID, myAddress);
    agri_getMasterAddress(masterAddress);
    
    AGRI_DEBUG_PRINT(F("My addr: ")); AGRI_DEBUG_PRINTLN((char*)myAddress);
    AGRI_DEBUG_PRINT(F("Master addr: ")); AGRI_DEBUG_PRINTLN((char*)masterAddress);
    
    // Open reading pipe on our address
    radio.openReadingPipe(1, myAddress);
    radio.startListening();
    
    // Initialize nodes
    initNodes();
    
    // Initialize timing
    lastHeartbeatTime = millis();
    lastTelemetryTime = millis();
    lastNodePollTime = millis();
    
    AGRI_DEBUG_PRINTLN(F("[OK] Cluster initialized"));
}

void loop()
{
    unsigned long now = millis();
    
    // Read local sensors
    readSensors();
    
    // Check node timeouts
    checkNodeTimeouts();
    
    // Process incoming packets
    processRxPackets();
    
    // Poll nodes periodically
    if ((now - lastNodePollTime) >= NODE_POLL_INTERVAL_MS) {
        pollAllNodes();
        lastNodePollTime = now;
    }
    
    // Send heartbeat to master
    if ((now - lastHeartbeatTime) >= HEARTBEAT_INTERVAL_MS) {
        sendHeartbeatToMaster();
        lastHeartbeatTime = now;
    }
    
    // Send telemetry to master
    if ((now - lastTelemetryTime) >= TELEMETRY_INTERVAL_MS) {
        sendTelemetryToMaster();
        lastTelemetryTime = now;
    }
    
    // Process local schedule (fallback)
    // processLocalSchedule(); // Enable when time handling is implemented
    
    // Yield for ESP32 tasks
    delay(10);
}
