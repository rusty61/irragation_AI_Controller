#ifndef AGRI_RF24_COMMON_H
#define AGRI_RF24_COMMON_H

#include <Arduino.h>
#include <RF24.h>

// ---------------------------------
// RF24 / packet constants
// ---------------------------------

// NRF24 max payload
#define AGRI_RF24_PAYLOAD_MAX 32

// Magic byte to identify valid AgriNet packets
#define AGRI_MAGIC_BYTE 0xA5

// ---------------------------------
// Status flag bits used by UNO -> ESP32
// ---------------------------------

static const uint16_t STATUS_FLAG_MOVING    = 0x0001;
static const uint16_t STATUS_FLAG_FAULT     = 0x0002;
static const uint16_t STATUS_FLAG_OPEN      = 0x0004;
static const uint16_t STATUS_FLAG_CLOSED    = 0x0008;
static const uint16_t STATUS_FLAG_TIMEOUT   = 0x0010;
static const uint16_t STATUS_FLAG_ENDSTOP   = 0x0020;
static const uint16_t STATUS_FLAG_OVERCURR  = 0x0040;
static const uint16_t STATUS_FLAG_COMMS_OK  = 0x0080;

// ---------------------------------
// Enums
// ---------------------------------

enum class AgriRole : uint8_t {
  UNO_NODE      = 0,
  ESP32_CLUSTER = 1,
  UNOQ_MMC      = 2
};

enum class AgriMsgType : uint8_t {
  // UNO <-> ESP32
  UNO_STATUS        = 0x01,
  UNO_TELEMETRY     = 0x02,
  UNO_ACK_CMD       = 0x03,
  UNO_FAULT         = 0x04,

  UNO_CMD_ACTUATOR  = 0x10,
  UNO_CMD_CONFIG    = 0x11,
  UNO_PING          = 0x12,

  // ESP32 <-> UNO_Q
  CLUSTER_TELEMETRY = 0x20,
  CLUSTER_STATUS    = 0x21,
  CLUSTER_FAULT     = 0x22,
  CLUSTER_HEARTBEAT = 0x23,

  CLUSTER_SET_SCHED = 0x30,
  CLUSTER_CMD_ZONE  = 0x31,
  CLUSTER_CONFIG    = 0x32,
  CLUSTER_PING      = 0x33
};

enum class AgriResult : uint8_t {
  OK          = 0,
  TIMEOUT     = 1,
  RF_FAIL     = 2,
  BAD_PACKET  = 3,
  DUPLICATE   = 4
};

enum class AgriZoneState : uint8_t {
  IDLE        = 0,
  IRRIGATING  = 1,
  FAULT       = 2
};

enum class AgriActAction : uint8_t {
  STOP        = 0,
  OPEN        = 1,
  CLOSE       = 2
};

// ---------------------------------
// Packet structs
// ---------------------------------

struct AgriPacketHeader {
  uint8_t   magic;
  uint8_t   msgType;
  uint8_t   srcRole;
  uint8_t   dstRole;
  uint8_t   srcId;
  uint8_t   dstId;
  uint16_t  seq;
};

struct AgriUnoActuatorCommand {
  uint8_t   zoneLocalId;
  uint8_t   action;    // AgriActAction
  uint32_t  runMs;
  uint16_t  cmdSeq;
};

// *** WIRE STRUCT – MUST MATCH ON UNO + ESP32 ***
struct __attribute__((packed)) AgriUnoStatus {
  uint8_t   zoneLocalId;
  uint16_t  statusFlags;
  uint16_t  current_mA;
  uint16_t  soilRaw;
  uint8_t   soilPct;
  uint16_t  batt_mV;
  uint16_t  lastCmdSeq;
};

struct AgriZoneSnapshot {
  uint8_t   zoneId;
  uint8_t   zoneState;    // AgriZoneState
  uint8_t   soilPct;
  uint16_t  lastRun_s;
};

// NOTE: NRF24 payload is max 32 bytes
// header(8) + telemetry(...) must be <= 32.
// With 2 zones: 8 + 20 = 28 bytes → OK.
#define AGRI_MAX_ZONES_PER_CLUSTER 2
#define AGRI_MAX_SCHEDULE_ENTRIES  8

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
  uint32_t  startEpoch_s;
  uint32_t  duration_s;
  uint8_t   priority;
};

struct AgriClusterSchedule {
  uint8_t   clusterId;
  uint16_t  schedSeq;
  uint8_t   numEntries;
  AgriZoneSchedule entries[AGRI_MAX_SCHEDULE_ENTRIES];
};

// ---------------------------------
// Address helpers
// ---------------------------------

void agri_getMasterAddress(uint8_t address[6]);
void agri_getClusterAddress(uint8_t clusterId, uint8_t address[6]);
void agri_getUnoNodeAddress(uint8_t clusterId, uint8_t nodeLocalId, uint8_t address[6]);

// ---------------------------------
// RF24 helpers
// ---------------------------------

AgriResult agri_rf24_init_common(
  RF24 &radio,
  AgriRole role,
  uint8_t clusterId,
  uint8_t nodeId
);

AgriResult agri_rf24_sendTo(
  RF24 &radio,
  const uint8_t destAddr[6],
  const void *buf,
  uint8_t len
);

AgriResult agri_rf24_recv(
  RF24 &radio,
  void *buf,
  uint8_t &len,
  uint16_t timeoutMs
);

// Build header in-place
void agri_buildHeader(
  AgriPacketHeader &hdr,
  AgriMsgType msgType,
  AgriRole srcRole,
  AgriRole dstRole,
  uint8_t srcId,
  uint8_t dstId
);

#endif // AGRI_RF24_COMMON_H
