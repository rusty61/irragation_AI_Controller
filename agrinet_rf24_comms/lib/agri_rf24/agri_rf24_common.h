#pragma once

#include <Arduino.h>
#include <RF24.h>

// =============================
// RF24 GLOBAL CONFIG
// =============================

static const uint8_t  AGRI_RF24_CHANNEL       = 76;      // 2.476 GHz
static const uint8_t  AGRI_RF24_RETRY_DELAY   = 5;       // * 250us
static const uint8_t  AGRI_RF24_RETRY_COUNT   = 15;
static const uint8_t  AGRI_RF24_PAYLOAD_MAX   = 32;
static const uint8_t  AGRI_RF24_PIPE_MAIN     = 1;

static const uint8_t  AGRI_MAGIC_BYTE         = 0xA5;

// =============================
// ROLES
// =============================

enum class AgriRole : uint8_t {
  UNKNOWN       = 0,
  UNO_NODE      = 1,
  ESP32_CLUSTER = 2,
  UNOQ_MMC      = 3,
};

// =============================
// MESSAGE TYPES (per README)
// =============================

enum class AgriMsgType : uint8_t {
  // UNO <-> ESP32 (UNO → ESP32: 0x10-0x1F)
  UNO_STATUS        = 0x10,
  UNO_TELEMETRY     = 0x11,
  UNO_ACK_CMD       = 0x12,
  UNO_FAULT         = 0x13,
  // ESP32 → UNO: 0x20-0x2F
  UNO_CMD_ACTUATOR  = 0x20,
  UNO_CMD_CONFIG    = 0x21,
  UNO_PING          = 0x22,

  // ESP32 <-> UNO_Q (ESP32 → UNO_Q: 0x30-0x3F)
  CLUSTER_TELEMETRY = 0x30,
  CLUSTER_STATUS    = 0x31,
  CLUSTER_FAULT     = 0x32,
  CLUSTER_HEARTBEAT = 0x33,
  // UNO_Q → ESP32: 0x40-0x4F
  CLUSTER_SET_SCHED = 0x40,
  CLUSTER_CMD_ZONE  = 0x41,
  CLUSTER_CONFIG    = 0x42,
  CLUSTER_PING      = 0x43,
};

// =============================
// ACTUATOR + ZONE STATES
// =============================

enum class AgriActAction : uint8_t {
  ACT_STOP  = 0,
  ACT_OPEN  = 1,
  ACT_CLOSE = 2,
};

enum class AgriZoneState : uint8_t {
  ZONE_IDLE        = 0,
  ZONE_IRRIGATING  = 1,
  ZONE_FAULT       = 2,
};

// =============================
// RESULT CODES
// =============================

enum class AgriResult : uint8_t {
  OK         = 0,
  TIMEOUT    = 1,
  RF_FAIL    = 2,
  BAD_PACKET = 3,
  DUPLICATE  = 4,
};

// =============================
// UNO STATUS FLAGS (bitmask per README)
// =============================

static const uint16_t STATUS_FLAG_OPEN     = 0x0001;  // Valve is open
static const uint16_t STATUS_FLAG_CLOSED   = 0x0002;  // Valve is closed
static const uint16_t STATUS_FLAG_MOVING   = 0x0004;  // Valve is moving
static const uint16_t STATUS_FLAG_FAULT    = 0x0008;  // Fault condition
static const uint16_t STATUS_FLAG_ENDSTOP  = 0x0010;  // Endstop reached
static const uint16_t STATUS_FLAG_OVERCURR = 0x0020;  // Overcurrent detected
static const uint16_t STATUS_FLAG_TIMEOUT  = 0x0040;  // Operation timeout
static const uint16_t STATUS_FLAG_COMMS_OK = 0x0080;  // Communications OK

// =============================
// COMMON HEADER
// =============================

struct AgriPacketHeader {
  uint8_t   magic;      // constant, AGRI_MAGIC_BYTE
  uint8_t   msgType;    // AgriMsgType
  uint8_t   srcRole;    // AgriRole
  uint8_t   dstRole;    // AgriRole
  uint8_t   srcId;      // node or cluster ID
  uint8_t   dstId;      // node or cluster ID
  uint16_t  seq;        // sequence number (per-sender)
};

// =============================
// UNO <-> ESP32 PAYLOADS
// =============================

struct AgriUnoActuatorCommand {
  uint8_t   zoneLocalId;   // 1..N for this cluster
  uint8_t   action;        // AgriActAction
  uint32_t  runMs;         // requested run duration in ms
  uint16_t  cmdSeq;        // command sequence token
};

struct AgriUnoStatus {
  uint8_t   zoneLocalId;
  uint16_t  statusFlags;   // OPEN/CLOSED/MOVING/FAULT/ENDSTOP (bitmask)
  uint16_t  current_mA;    // actuator current, if available
  uint16_t  soilRaw;       // raw ADC soil value (optional)
  uint8_t   soilPct;       // 0..100 scaled soil moisture (optional)
  uint16_t  batt_mV;       // local battery/rail voltage (optional)
  uint16_t  lastCmdSeq;    // echoes last command seq processed
};

// =============================
// ESP32 <-> UNO_Q PAYLOADS
// =============================

#define AGRI_MAX_ZONES_PER_CLUSTER     8
#define AGRI_MAX_SCHEDULE_ENTRIES      8

struct AgriZoneSnapshot {
  uint8_t   zoneId;        // global or per-cluster
  uint8_t   zoneState;     // AgriZoneState
  uint8_t   soilPct;
  uint16_t  lastRun_s;     // last irrigation duration in seconds
};

struct AgriClusterTelemetry {
  uint8_t   clusterId;
  uint16_t  seq;
  uint16_t  batt_mV;
  uint16_t  panel_mV;
  uint16_t  pumpCurrent_mA;
  uint8_t   numZones;
  AgriZoneSnapshot zones[AGRI_MAX_ZONES_PER_CLUSTER];
};

struct AgriZoneSchedule {
  uint8_t   zoneId;
  uint8_t   mode;          // 0=OFF,1=AUTO,2=FORCE_ON,3=FORCE_OFF
  uint32_t  startEpoch_s;  // start time (epoch seconds or defined relative)
  uint32_t  duration_s;
  uint8_t   priority;      // 0..2
};

struct AgriClusterSchedule {
  uint8_t   clusterId;
  uint16_t  schedSeq;      // schedule version
  uint8_t   numEntries;
  AgriZoneSchedule entries[AGRI_MAX_SCHEDULE_ENTRIES];
};

// =============================
// ADDRESS HELPERS
// =============================

// =============================
// ADDRESS HELPERS
// =============================

// Get master address ("HQ000")
void agri_getMasterAddress(uint8_t address[6]);

// Get cluster address (e.g., "HQA01".."HQA16")
void agri_getClusterAddress(uint8_t clusterId, uint8_t address[6]);

// Get UNO node address (e.g. "N1A02", "NAA05" for cluster 10, node 5)
void agri_getUnoNodeAddress(uint8_t clusterId,
                            uint8_t nodeLocalId,
                            uint8_t address[6]);

// =============================
// INIT + SEND/RECV API
// =============================

/**
 * Common init for all roles.
 *
 * role:
 *   - UNO_NODE      => thisId = nodeLocalId, extraId = clusterId
 *   - ESP32_CLUSTER => thisId = clusterId,   extraId ignored
 *   - UNOQ_MMC      => IDs ignored
 */
AgriResult agri_rf24_init_common(
  RF24      &radio,
  AgriRole   role,
  uint8_t    thisId,
  uint8_t    extraId /* cluster or unused */
);

/**
 * Build a packet header with auto-incremented seq for this sender.
 */
void agri_buildHeader(
  AgriPacketHeader &hdr,
  AgriMsgType       type,
  AgriRole          srcRole,
  AgriRole          dstRole,
  uint8_t           srcId,
  uint8_t           dstId
);

/**
 * Send a raw packet to destAddr (5-byte RF24 address).
 * Returns OK on success (ACKed) or RF_FAIL/TIMEOUT.
 */
AgriResult agri_rf24_sendTo(
  RF24            &radio,
  const uint8_t    destAddr[6],
  const void      *payload,
  uint8_t          len
);

/**
 * Blocking receive with timeout. Reads up to maxLen bytes into buffer.
 * len in/out: on entry max buffer size; on exit actual bytes read.
 */
AgriResult agri_rf24_recvBlocking(
  RF24    &radio,
  void    *buffer,
  uint8_t *len,
  uint16_t timeoutMs
);

// =============================
// DEBUG SUPPORT
// =============================

#ifdef RF24_DEBUG
void agri_debugPrintAddr(const char *label, const uint8_t addr[6]);
#else
inline void agri_debugPrintAddr(const char *, const uint8_t[6]) {}
#endif
