/**
 * @file agri_rf24_common.h
 * @brief AgriNet RF24 Communications Stack - Shared definitions
 * 
 * Common enums, structs, and helper declarations for the AgriNet
 * multi-node irrigation system RF24 communications.
 * 
 * Network Topology:
 *   - Bottom tier: UNO actuator nodes (valves/actuators, safety logic)
 *   - Middle tier: ESP32 cluster nodes (field hubs)
 *   - Top tier: Arduino UNO_Q MMC (Qualcomm+STM32 hybrid) as central master
 */

#ifndef AGRI_RF24_COMMON_H
#define AGRI_RF24_COMMON_H

#include <stdint.h>
#include <RF24.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Configuration Constants
 *==========================================================================*/

/** RF24 frequency channel (2.476 GHz) */
#define AGRI_RF24_CHANNEL       76

/** RF24 data rate (250 kbps) */
#define AGRI_RF24_DATA_RATE     RF24_250KBPS

/** RF24 PA level (high) */
#define AGRI_RF24_PA_LEVEL      RF24_PA_HIGH

/** RF24 CRC length (2 bytes) */
#define AGRI_RF24_CRC_LENGTH    RF24_CRC_16

/** RF24 retry delay (5 * 250µs = 1.25ms) */
#define AGRI_RF24_RETRY_DELAY   5

/** RF24 retry count (maximum 15 retries) */
#define AGRI_RF24_RETRY_COUNT   15

/** RF24 address length (5 bytes) */
#define AGRI_RF24_ADDR_LEN      5

/** Magic byte for packet validation */
#define AGRI_MAGIC_BYTE         0xA5

/** Maximum zones per cluster */
#define AGRI_MAX_ZONES_PER_CLUSTER    8

/** Maximum schedule entries */
#define AGRI_MAX_SCHEDULE_ENTRIES     8

/** Maximum clusters in network */
#define AGRI_MAX_CLUSTERS             16

/** Maximum nodes per cluster */
#define AGRI_MAX_NODES_PER_CLUSTER    8

/** Receive timeout in milliseconds */
#define AGRI_RX_TIMEOUT_MS            50

/*============================================================================
 * Enumerations
 *==========================================================================*/

/**
 * @brief Result codes for RF24 operations
 */
enum class AgriResult : uint8_t {
    OK          = 0,    ///< Operation successful
    TIMEOUT     = 1,    ///< Operation timed out
    RF_FAIL     = 2,    ///< RF24 hardware failure
    BAD_PACKET  = 3,    ///< Invalid packet (bad magic, CRC, etc.)
    DUPLICATE   = 4     ///< Duplicate packet received
};

/**
 * @brief Node roles in the network
 */
enum class AgriRole : uint8_t {
    ROLE_UNO_NODE     = 0,  ///< UNO actuator node (bottom tier)
    ROLE_ESP32_CLUSTER= 1,  ///< ESP32 cluster node (middle tier)
    ROLE_UNO_Q_MMC    = 2   ///< UNO_Q master node (top tier)
};

/**
 * @brief Message types for UNO ↔ ESP32 communication
 */
enum class AgriMsgTypeUno : uint8_t {
    UNO_STATUS          = 0x10,  ///< UNO status report
    UNO_TELEMETRY       = 0x11,  ///< UNO telemetry data
    UNO_ACK_CMD         = 0x12,  ///< UNO command acknowledgment
    UNO_FAULT           = 0x13,  ///< UNO fault report
    UNO_CMD_ACTUATOR    = 0x20,  ///< Command to actuator
    UNO_CMD_CONFIG      = 0x21,  ///< Configuration command
    UNO_PING            = 0x22   ///< Ping/keepalive
};

/**
 * @brief Message types for ESP32 ↔ UNO_Q communication
 */
enum class AgriMsgTypeCluster : uint8_t {
    CLUSTER_TELEMETRY   = 0x30,  ///< Cluster telemetry data
    CLUSTER_STATUS      = 0x31,  ///< Cluster status report
    CLUSTER_FAULT       = 0x32,  ///< Cluster fault report
    CLUSTER_HEARTBEAT   = 0x33,  ///< Cluster heartbeat
    CLUSTER_SET_SCHED   = 0x40,  ///< Set cluster schedule
    CLUSTER_CMD_ZONE    = 0x41,  ///< Zone command
    CLUSTER_CONFIG      = 0x42,  ///< Cluster configuration
    CLUSTER_PING        = 0x43   ///< Ping/keepalive
};

/**
 * @brief Actuator actions
 */
enum class AgriActuatorAction : uint8_t {
    ACTION_STOP     = 0,    ///< Stop actuator
    ACTION_OPEN     = 1,    ///< Open valve
    ACTION_CLOSE    = 2     ///< Close valve
};

/**
 * @brief Zone states
 */
enum class AgriZoneState : uint8_t {
    ZONE_IDLE       = 0,    ///< Zone is idle
    ZONE_IRRIGATING = 1,    ///< Zone is irrigating
    ZONE_FAULT      = 2     ///< Zone is in fault state
};

/**
 * @brief Zone schedule modes
 */
enum class AgriScheduleMode : uint8_t {
    MODE_OFF        = 0,    ///< Zone off
    MODE_AUTO       = 1,    ///< Automatic scheduling
    MODE_FORCE_ON   = 2,    ///< Force zone on
    MODE_FORCE_OFF  = 3     ///< Force zone off
};

/**
 * @brief UNO status flags (bitmask)
 */
enum AgriUnoStatusFlags : uint16_t {
    STATUS_FLAG_OPEN      = 0x0001,  ///< Valve is open
    STATUS_FLAG_CLOSED    = 0x0002,  ///< Valve is closed
    STATUS_FLAG_MOVING    = 0x0004,  ///< Valve is moving
    STATUS_FLAG_FAULT     = 0x0008,  ///< Fault condition
    STATUS_FLAG_ENDSTOP   = 0x0010,  ///< Endstop reached
    STATUS_FLAG_OVERCURR  = 0x0020,  ///< Overcurrent detected
    STATUS_FLAG_TIMEOUT   = 0x0040,  ///< Operation timeout
    STATUS_FLAG_COMMS_OK  = 0x0080   ///< Communications OK
};

/*============================================================================
 * Packet Structures
 *==========================================================================*/

#pragma pack(push, 1)

/**
 * @brief Common packet header (8 bytes)
 */
struct AgriPacketHeader {
    uint8_t   magic;      ///< Magic byte (0xA5)
    uint8_t   msgType;    ///< Message type (AgriMsgType enum)
    uint8_t   srcRole;    ///< Source role (AgriRole enum)
    uint8_t   dstRole;    ///< Destination role (AgriRole enum)
    uint8_t   srcId;      ///< Source node or cluster ID
    uint8_t   dstId;      ///< Destination node or cluster ID
    uint16_t  seq;        ///< Sequence number
};

/**
 * @brief UNO Actuator Command payload
 */
struct AgriUnoActuatorCommand {
    uint8_t   zoneLocalId;  ///< Local zone identifier
    uint8_t   action;       ///< Action (STOP/OPEN/CLOSE)
    uint32_t  runMs;        ///< Runtime in milliseconds
    uint16_t  cmdSeq;       ///< Command sequence number
};

/**
 * @brief UNO Status payload
 */
struct AgriUnoStatus {
    uint8_t   zoneLocalId;  ///< Local zone identifier
    uint16_t  statusFlags;  ///< Status flags bitmask
    uint16_t  current_mA;   ///< Current draw in mA
    uint16_t  soilRaw;      ///< Raw soil sensor reading
    uint8_t   soilPct;      ///< Soil moisture percentage
    uint16_t  batt_mV;      ///< Battery voltage in mV
    uint16_t  lastCmdSeq;   ///< Last acknowledged command sequence
};

/**
 * @brief Zone snapshot for cluster telemetry
 */
struct AgriZoneSnapshot {
    uint8_t   zoneId;       ///< Zone identifier
    uint8_t   zoneState;    ///< Zone state (IDLE/IRRIGATING/FAULT)
    uint8_t   soilPct;      ///< Soil moisture percentage
    uint16_t  lastRun_s;    ///< Seconds since last run
};

/**
 * @brief Cluster Telemetry payload
 */
struct AgriClusterTelemetry {
    uint8_t   clusterId;        ///< Cluster identifier
    uint16_t  seq;              ///< Sequence number
    uint16_t  batt_mV;          ///< Battery voltage in mV
    uint16_t  panel_mV;         ///< Solar panel voltage in mV
    uint16_t  pumpCurrent_mA;   ///< Pump current in mA
    uint8_t   numZones;         ///< Number of active zones
    AgriZoneSnapshot zones[AGRI_MAX_ZONES_PER_CLUSTER];
};

/**
 * @brief Zone schedule entry
 */
struct AgriZoneSchedule {
    uint8_t   zoneId;           ///< Zone identifier
    uint8_t   mode;             ///< Schedule mode (OFF/AUTO/FORCE_ON/FORCE_OFF)
    uint32_t  startEpoch_s;     ///< Start time (Unix epoch seconds)
    uint32_t  duration_s;       ///< Duration in seconds
    uint8_t   priority;         ///< Schedule priority
};

/**
 * @brief Cluster Schedule payload
 */
struct AgriClusterSchedule {
    uint8_t   clusterId;        ///< Cluster identifier
    uint16_t  schedSeq;         ///< Schedule sequence number
    uint8_t   numEntries;       ///< Number of schedule entries
    AgriZoneSchedule entries[AGRI_MAX_SCHEDULE_ENTRIES];
};

/**
 * @brief Generic packet structure
 */
struct AgriPacket {
    AgriPacketHeader header;
    union {
        AgriUnoActuatorCommand actuatorCmd;
        AgriUnoStatus          unoStatus;
        AgriClusterTelemetry   clusterTelem;
        AgriClusterSchedule    clusterSched;
        uint8_t                raw[24];  // Raw payload buffer
    } payload;
};

#pragma pack(pop)

/*============================================================================
 * Address Helper Functions
 *==========================================================================*/

/**
 * @brief Get the base address for UNO_Q master
 * @param[out] address 6-byte buffer for the address (5 chars + null)
 */
void agri_getMasterAddress(uint8_t address[6]);

/**
 * @brief Get the address for a cluster node
 * @param clusterId Cluster identifier (1-16)
 * @param[out] address 6-byte buffer for the address (5 chars + null)
 */
void agri_getClusterAddress(uint8_t clusterId, uint8_t address[6]);

/**
 * @brief Get the address for a UNO node
 * @param clusterId Parent cluster identifier (1-16)
 * @param nodeLocalId Local node identifier within cluster (1-8)
 * @param[out] address 6-byte buffer for the address (5 chars + null)
 */
void agri_getUnoNodeAddress(uint8_t clusterId, uint8_t nodeLocalId, uint8_t address[6]);

/*============================================================================
 * RF24 Core Functions
 *==========================================================================*/

/**
 * @brief Initialize RF24 radio with AgriNet settings
 * @param radio Reference to RF24 instance
 * @param role Node role (determines address configuration)
 * @return AgriResult::OK on success, error code otherwise
 */
AgriResult agri_rf24_init_common(RF24 &radio, AgriRole role);

/**
 * @brief Send a packet via RF24
 * @param radio Reference to RF24 instance
 * @param destAddr Destination address (5 bytes)
 * @param packet Pointer to packet data
 * @param len Packet length in bytes
 * @return AgriResult::OK on success, error code otherwise
 */
AgriResult agri_rf24_send(RF24 &radio, const uint8_t *destAddr, 
                          const void *packet, uint8_t len);

/**
 * @brief Receive a packet via RF24 (non-blocking)
 * @param radio Reference to RF24 instance
 * @param[out] packet Buffer for received packet
 * @param[in,out] len Input: buffer size, Output: received length
 * @param timeout_ms Timeout in milliseconds (0 = no wait)
 * @return AgriResult::OK on success, TIMEOUT if no packet, error otherwise
 */
AgriResult agri_rf24_recv(RF24 &radio, void *packet, uint8_t *len, 
                          uint16_t timeout_ms);

/*============================================================================
 * Packet Helper Functions
 *==========================================================================*/

/**
 * @brief Initialize a packet header
 * @param header Pointer to header structure
 * @param msgType Message type
 * @param srcRole Source role
 * @param dstRole Destination role
 * @param srcId Source ID
 * @param dstId Destination ID
 * @param seq Sequence number
 */
void agri_initPacketHeader(AgriPacketHeader *header, uint8_t msgType,
                           AgriRole srcRole, AgriRole dstRole,
                           uint8_t srcId, uint8_t dstId, uint16_t seq);

/**
 * @brief Validate a packet header
 * @param header Pointer to header structure
 * @return true if valid, false otherwise
 */
bool agri_validatePacketHeader(const AgriPacketHeader *header);

/*============================================================================
 * Debug Logging
 *==========================================================================*/

#ifdef RF24_DEBUG
    #define AGRI_DEBUG_PRINT(msg)       Serial.print(msg)
    #define AGRI_DEBUG_PRINTLN(msg)     Serial.println(msg)
    #define AGRI_DEBUG_PRINTHEX(val)    Serial.print(val, HEX)
#else
    #define AGRI_DEBUG_PRINT(msg)
    #define AGRI_DEBUG_PRINTLN(msg)
    #define AGRI_DEBUG_PRINTHEX(val)
#endif

#ifdef __cplusplus
}
#endif

#endif // AGRI_RF24_COMMON_H
