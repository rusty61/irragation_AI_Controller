/**
 * @file uno_q_mmc.ino
 * @brief AgriNet UNO_Q MMC Master Node - RF24 Implementation
 * 
 * Top tier central master for the AgriNet irrigation system.
 * Manages all ESP32 cluster nodes and coordinates system-wide operations.
 * 
 * Features:
 *   - Manages up to 16 ESP32 cluster nodes
 *   - System-wide scheduling and coordination
 *   - Telemetry aggregation and logging
 *   - Cluster health monitoring
 * 
 * Hardware:
 *   - Arduino UNO_Q (Qualcomm+STM32 hybrid)
 *   - NRF24L01+ module (CE=9, CSN=10)
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

/** Number of clusters to manage */
#define NUM_CLUSTERS        2

/** Cluster timeout for marking offline (ms) */
#define CLUSTER_TIMEOUT_MS  60000

/** Poll interval for clusters (ms) */
#define CLUSTER_POLL_INTERVAL_MS    30000

/** Schedule broadcast interval (ms) */
#define SCHEDULE_BROADCAST_INTERVAL_MS  120000

/*============================================================================
 * Types
 *==========================================================================*/

/**
 * @brief Cluster tracking structure
 */
struct ClusterInfo {
    uint8_t clusterId;
    uint8_t address[6];
    bool online;
    unsigned long lastContact;
    AgriClusterTelemetry lastTelemetry;
};

/*============================================================================
 * Globals
 *==========================================================================*/

// RF24 instance
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

// Master address
uint8_t myAddress[6];

// Cluster tracking
ClusterInfo clusters[AGRI_MAX_CLUSTERS];
uint8_t numClusters = NUM_CLUSTERS;

// Master schedule (to broadcast to clusters)
AgriClusterSchedule masterSchedules[AGRI_MAX_CLUSTERS];

// Sequence numbers
uint16_t txSeq = 0;
uint16_t lastRxSeq = 0;
uint16_t cmdSeq = 0;

// Timing
unsigned long lastClusterPollTime = 0;
unsigned long lastScheduleBroadcastTime = 0;

/*============================================================================
 * Cluster Management
 *==========================================================================*/

/**
 * @brief Initialize cluster tracking
 */
void initClusters()
{
    for (uint8_t i = 0; i < AGRI_MAX_CLUSTERS; i++) {
        if (i < numClusters) {
            clusters[i].clusterId = i + 1;
            agri_getClusterAddress(i + 1, clusters[i].address);
            clusters[i].online = false;
            clusters[i].lastContact = 0;
            memset(&clusters[i].lastTelemetry, 0, sizeof(AgriClusterTelemetry));
        } else {
            clusters[i].clusterId = 0;
        }
        
        // Initialize empty schedules
        memset(&masterSchedules[i], 0, sizeof(AgriClusterSchedule));
        masterSchedules[i].clusterId = i + 1;
    }
}

/**
 * @brief Update cluster status from telemetry
 * @param clusterId Cluster identifier
 * @param telemetry Telemetry data
 */
void updateClusterTelemetry(uint8_t clusterId, const AgriClusterTelemetry &telemetry)
{
    for (uint8_t i = 0; i < numClusters; i++) {
        if (clusters[i].clusterId == clusterId) {
            clusters[i].online = true;
            clusters[i].lastContact = millis();
            memcpy(&clusters[i].lastTelemetry, &telemetry, sizeof(AgriClusterTelemetry));
            
            AGRI_DEBUG_PRINT(F("[CLUSTER] Updated cluster "));
            AGRI_DEBUG_PRINTLN(clusterId);
            return;
        }
    }
}

/**
 * @brief Mark cluster as online from heartbeat
 * @param clusterId Cluster identifier
 */
void markClusterOnline(uint8_t clusterId)
{
    for (uint8_t i = 0; i < numClusters; i++) {
        if (clusters[i].clusterId == clusterId) {
            clusters[i].online = true;
            clusters[i].lastContact = millis();
            return;
        }
    }
}

/**
 * @brief Check cluster timeouts
 */
void checkClusterTimeouts()
{
    unsigned long now = millis();
    
    for (uint8_t i = 0; i < numClusters; i++) {
        if (clusters[i].clusterId != 0 && clusters[i].online) {
            if ((now - clusters[i].lastContact) > CLUSTER_TIMEOUT_MS) {
                AGRI_DEBUG_PRINT(F("[CLUSTER] Cluster "));
                AGRI_DEBUG_PRINT(clusters[i].clusterId);
                AGRI_DEBUG_PRINTLN(F(" marked offline"));
                clusters[i].online = false;
            }
        }
    }
}

/**
 * @brief Get cluster info by ID
 * @param clusterId Cluster identifier
 * @return Pointer to ClusterInfo or nullptr
 */
ClusterInfo* getCluster(uint8_t clusterId)
{
    for (uint8_t i = 0; i < numClusters; i++) {
        if (clusters[i].clusterId == clusterId) {
            return &clusters[i];
        }
    }
    return nullptr;
}

/*============================================================================
 * Communication Functions - To Clusters
 *==========================================================================*/

/**
 * @brief Send ping to a cluster
 * @param clusterId Target cluster ID
 */
void pingCluster(uint8_t clusterId)
{
    ClusterInfo *cluster = getCluster(clusterId);
    if (cluster == nullptr) return;
    
    AgriPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    agri_initPacketHeader(&packet.header,
                          static_cast<uint8_t>(AgriMsgTypeCluster::CLUSTER_PING),
                          AgriRole::ROLE_UNO_Q_MMC,
                          AgriRole::ROLE_ESP32_CLUSTER,
                          0, clusterId, txSeq++);
    
    AgriResult result = agri_rf24_send(radio, cluster->address, &packet,
                                        sizeof(AgriPacketHeader));
    
    if (result == AgriResult::OK) {
        AGRI_DEBUG_PRINT(F("[TX->C] Ping sent to cluster "));
        AGRI_DEBUG_PRINTLN(clusterId);
    }
}

/**
 * @brief Poll all clusters
 */
void pollAllClusters()
{
    for (uint8_t i = 0; i < numClusters; i++) {
        if (clusters[i].clusterId != 0) {
            pingCluster(clusters[i].clusterId);
            delay(100); // Delay between pings
        }
    }
}

/**
 * @brief Send schedule to a cluster
 * @param clusterId Target cluster ID
 * @param schedule Schedule data
 */
void sendScheduleToCluster(uint8_t clusterId, const AgriClusterSchedule &schedule)
{
    ClusterInfo *cluster = getCluster(clusterId);
    if (cluster == nullptr) return;
    
    AgriPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    agri_initPacketHeader(&packet.header,
                          static_cast<uint8_t>(AgriMsgTypeCluster::CLUSTER_SET_SCHED),
                          AgriRole::ROLE_UNO_Q_MMC,
                          AgriRole::ROLE_ESP32_CLUSTER,
                          0, clusterId, txSeq++);
    
    memcpy(&packet.payload.clusterSched, &schedule, sizeof(AgriClusterSchedule));
    
    AgriResult result = agri_rf24_send(radio, cluster->address, &packet,
                                        sizeof(AgriPacketHeader) + sizeof(AgriClusterSchedule));
    
    if (result == AgriResult::OK) {
        AGRI_DEBUG_PRINT(F("[TX->C] Schedule sent to cluster "));
        AGRI_DEBUG_PRINTLN(clusterId);
    }
}

/**
 * @brief Broadcast schedules to all clusters
 */
void broadcastSchedules()
{
    for (uint8_t i = 0; i < numClusters; i++) {
        if (clusters[i].clusterId != 0 && clusters[i].online) {
            sendScheduleToCluster(clusters[i].clusterId, masterSchedules[i]);
            delay(100);
        }
    }
}

/**
 * @brief Send zone command to a cluster
 * @param clusterId Target cluster ID
 * @param zoneId Zone to control
 * @param action Action to perform
 * @param runMs Runtime in milliseconds
 */
void sendZoneCommand(uint8_t clusterId, uint8_t zoneId,
                     AgriActuatorAction action, uint32_t runMs)
{
    ClusterInfo *cluster = getCluster(clusterId);
    if (cluster == nullptr) {
        AGRI_DEBUG_PRINTLN(F("[TX->C] Cluster not found"));
        return;
    }
    
    AgriPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    agri_initPacketHeader(&packet.header,
                          static_cast<uint8_t>(AgriMsgTypeCluster::CLUSTER_CMD_ZONE),
                          AgriRole::ROLE_UNO_Q_MMC,
                          AgriRole::ROLE_ESP32_CLUSTER,
                          0, clusterId, txSeq++);
    
    packet.payload.actuatorCmd.zoneLocalId = zoneId;
    packet.payload.actuatorCmd.action = static_cast<uint8_t>(action);
    packet.payload.actuatorCmd.runMs = runMs;
    packet.payload.actuatorCmd.cmdSeq = ++cmdSeq;
    
    AgriResult result = agri_rf24_send(radio, cluster->address, &packet,
                                        sizeof(AgriPacketHeader) + sizeof(AgriUnoActuatorCommand));
    
    if (result == AgriResult::OK) {
        AGRI_DEBUG_PRINT(F("[TX->C] Zone cmd sent to cluster "));
        AGRI_DEBUG_PRINTLN(clusterId);
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
    
    // Only process messages from clusters
    if (packet.header.srcRole != static_cast<uint8_t>(AgriRole::ROLE_ESP32_CLUSTER)) {
        AGRI_DEBUG_PRINTLN(F("[RX] Not from cluster"));
        return;
    }
    
    processClusterMessage(packet);
}

/**
 * @brief Process message from cluster
 * @param packet Received packet
 */
void processClusterMessage(const AgriPacket &packet)
{
    uint8_t clusterId = packet.header.srcId;
    
    switch (static_cast<AgriMsgTypeCluster>(packet.header.msgType)) {
        case AgriMsgTypeCluster::CLUSTER_TELEMETRY:
            AGRI_DEBUG_PRINT(F("[RX<-C] Telemetry from cluster "));
            AGRI_DEBUG_PRINTLN(clusterId);
            updateClusterTelemetry(clusterId, packet.payload.clusterTelem);
            logTelemetry(clusterId, packet.payload.clusterTelem);
            break;
            
        case AgriMsgTypeCluster::CLUSTER_STATUS:
            AGRI_DEBUG_PRINT(F("[RX<-C] Status from cluster "));
            AGRI_DEBUG_PRINTLN(clusterId);
            updateClusterTelemetry(clusterId, packet.payload.clusterTelem);
            break;
            
        case AgriMsgTypeCluster::CLUSTER_HEARTBEAT:
            AGRI_DEBUG_PRINT(F("[RX<-C] Heartbeat from cluster "));
            AGRI_DEBUG_PRINTLN(clusterId);
            markClusterOnline(clusterId);
            break;
            
        case AgriMsgTypeCluster::CLUSTER_FAULT:
            AGRI_DEBUG_PRINT(F("[RX<-C] FAULT from cluster "));
            AGRI_DEBUG_PRINTLN(clusterId);
            handleClusterFault(clusterId, packet);
            break;
            
        default:
            AGRI_DEBUG_PRINTLN(F("[RX<-C] Unknown message"));
            break;
    }
}

/**
 * @brief Handle cluster fault report
 * @param clusterId Cluster identifier
 * @param packet Fault packet
 */
void handleClusterFault(uint8_t clusterId, const AgriPacket &packet)
{
    // Log fault
    AGRI_DEBUG_PRINT(F("[FAULT] Cluster "));
    AGRI_DEBUG_PRINT(clusterId);
    AGRI_DEBUG_PRINTLN(F(" reported fault"));
    
    // Could trigger alerts, logging, or corrective actions here
}

/**
 * @brief Log telemetry data
 * @param clusterId Cluster identifier
 * @param telemetry Telemetry data
 */
void logTelemetry(uint8_t clusterId, const AgriClusterTelemetry &telemetry)
{
    Serial.print(F("TELEM,"));
    Serial.print(millis());
    Serial.print(F(",C"));
    Serial.print(clusterId);
    Serial.print(F(",BATT="));
    Serial.print(telemetry.batt_mV);
    Serial.print(F(",PANEL="));
    Serial.print(telemetry.panel_mV);
    Serial.print(F(",PUMP="));
    Serial.print(telemetry.pumpCurrent_mA);
    Serial.print(F(",ZONES="));
    Serial.print(telemetry.numZones);
    
    for (uint8_t i = 0; i < telemetry.numZones && i < AGRI_MAX_ZONES_PER_CLUSTER; i++) {
        Serial.print(F(",Z"));
        Serial.print(telemetry.zones[i].zoneId);
        Serial.print(F("="));
        Serial.print(telemetry.zones[i].soilPct);
        Serial.print(F("%,S"));
        Serial.print(telemetry.zones[i].zoneState);
    }
    Serial.println();
}

/*============================================================================
 * System Status
 *==========================================================================*/

/**
 * @brief Print system status
 */
void printSystemStatus()
{
    Serial.println(F("\n=== System Status ==="));
    
    uint8_t onlineCount = 0;
    for (uint8_t i = 0; i < numClusters; i++) {
        if (clusters[i].clusterId != 0) {
            Serial.print(F("Cluster "));
            Serial.print(clusters[i].clusterId);
            Serial.print(F(": "));
            
            if (clusters[i].online) {
                Serial.print(F("ONLINE, "));
                Serial.print(clusters[i].lastTelemetry.numZones);
                Serial.print(F(" zones, "));
                Serial.print(clusters[i].lastTelemetry.batt_mV);
                Serial.println(F("mV"));
                onlineCount++;
            } else {
                Serial.println(F("OFFLINE"));
            }
        }
    }
    
    Serial.print(F("Total: "));
    Serial.print(onlineCount);
    Serial.print(F("/"));
    Serial.print(numClusters);
    Serial.println(F(" clusters online"));
    Serial.println();
}

/*============================================================================
 * Serial Command Interface
 *==========================================================================*/

/**
 * @brief Process serial commands
 */
void processSerialCommands()
{
    if (!Serial.available()) return;
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("STATUS")) {
        printSystemStatus();
    }
    else if (cmd.startsWith("PING")) {
        // PING <clusterId>
        int clusterId = cmd.substring(5).toInt();
        if (clusterId > 0 && clusterId <= numClusters) {
            pingCluster(clusterId);
        }
    }
    else if (cmd.startsWith("ZONE")) {
        // ZONE <clusterId> <zoneId> <action> <runMs>
        // action: 0=STOP, 1=OPEN, 2=CLOSE
        int params[4];
        int idx = 0;
        int pos = 5;
        while (idx < 4 && pos < cmd.length()) {
            int space = cmd.indexOf(' ', pos);
            if (space == -1) space = cmd.length();
            params[idx++] = cmd.substring(pos, space).toInt();
            pos = space + 1;
        }
        
        if (idx >= 3) {
            sendZoneCommand(params[0], params[1], 
                           static_cast<AgriActuatorAction>(params[2]),
                           idx >= 4 ? params[3] : 10000);
        }
    }
    else if (cmd.startsWith("HELP")) {
        Serial.println(F("Commands:"));
        Serial.println(F("  STATUS - Show system status"));
        Serial.println(F("  PING <clusterId> - Ping a cluster"));
        Serial.println(F("  ZONE <clusterId> <zoneId> <action> [runMs]"));
        Serial.println(F("    action: 0=STOP, 1=OPEN, 2=CLOSE"));
    }
}

/*============================================================================
 * Setup and Loop
 *==========================================================================*/

void setup()
{
    // Initialize serial
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println(F("\n========================================"));
    Serial.println(F("      AgriNet UNO_Q MMC Master"));
    Serial.println(F("========================================"));
    Serial.print(F("Managing ")); Serial.print(NUM_CLUSTERS); Serial.println(F(" clusters"));
    
    // Initialize SPI
    SPI.begin();
    
    // Initialize RF24
    AgriResult result = agri_rf24_init_common(radio, AgriRole::ROLE_UNO_Q_MMC);
    if (result != AgriResult::OK) {
        Serial.println(F("[ERR] RF24 init failed!"));
        while (1) {
            delay(1000);
        }
    }
    
    // Set up master address
    agri_getMasterAddress(myAddress);
    Serial.print(F("Master addr: ")); Serial.println((char*)myAddress);
    
    // Open reading pipe on master address
    radio.openReadingPipe(1, myAddress);
    radio.startListening();
    
    // Initialize clusters
    initClusters();
    
    // Print cluster addresses
    Serial.println(F("Cluster addresses:"));
    for (uint8_t i = 0; i < numClusters; i++) {
        Serial.print(F("  C")); Serial.print(clusters[i].clusterId);
        Serial.print(F(": ")); Serial.println((char*)clusters[i].address);
    }
    
    // Initialize timing
    lastClusterPollTime = millis();
    lastScheduleBroadcastTime = millis();
    
    Serial.println(F("\n[OK] Master initialized"));
    Serial.println(F("Type HELP for commands\n"));
}

void loop()
{
    unsigned long now = millis();
    
    // Check cluster timeouts
    checkClusterTimeouts();
    
    // Process incoming packets
    processRxPackets();
    
    // Process serial commands
    processSerialCommands();
    
    // Poll clusters periodically
    if ((now - lastClusterPollTime) >= CLUSTER_POLL_INTERVAL_MS) {
        pollAllClusters();
        lastClusterPollTime = now;
    }
    
    // Broadcast schedules periodically
    if ((now - lastScheduleBroadcastTime) >= SCHEDULE_BROADCAST_INTERVAL_MS) {
        broadcastSchedules();
        lastScheduleBroadcastTime = now;
    }
    
    // Small delay
    delay(10);
}
