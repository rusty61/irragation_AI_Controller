#include "agri_rf24_common.h"

#ifdef RF24_DEBUG
  #define DBG_PRINT(x)    Serial.print(x)
  #define DBG_PRINTLN(x)  Serial.println(x)
#else
  #define DBG_PRINT(x)    do {} while (0)
  #define DBG_PRINTLN(x)  do {} while (0)
#endif


// Map clusterId 1..16 to the character used in addresses:
//  1..9   -> '1'..'9'
// 10..16  -> 'A'..'G'
// Anything outside 1..16 returns 'X' as a debug marker.
static char agri_cluster_char(uint8_t clusterId) {
    if (clusterId >= 1 && clusterId <= 9) {
        return static_cast<char>('0' + clusterId);
    }
    if (clusterId >= 10 && clusterId <= 16) {
        return static_cast<char>('A' + (clusterId - 10)); // 10->'A', 11->'B', ... 16->'G'
    }
    return 'X';
}

// =============================
// INTERNAL STATE
// =============================

static uint16_t g_seqCounter = 1;

// =============================
// ADDRESS HELPERS
// =============================

void agri_getMasterAddress(uint8_t address[6]) {
  // "HQ000" - master base address
  address[0] = 'H';
  address[1] = 'Q';
  address[2] = '0';
  address[3] = '0';
  address[4] = '0';
  address[5] = '\0';
}

static void agri_getUnoQAddress(uint8_t address[6]) {
  // Alias for internal use
  agri_getMasterAddress(address);
}

void agri_getClusterAddress(uint8_t clusterId, uint8_t address[6]) {
    // Global Tier (UNO_Q ↔ ESP32)
    //
    // README examples:
    //   Cluster 1  -> "HQA01"
    //   Cluster 2  -> "HQA02"
    //   ...
    //   Cluster 16 -> "HQA16"
    //
    // Pattern: "HQA" + 2-digit decimal clusterId (zero-padded)

    uint8_t tens = (clusterId / 10) % 10;    // 0..1 for 1..16
    uint8_t ones = clusterId % 10;          // 0..9

    address[0] = 'H';
    address[1] = 'Q';
    address[2] = 'A';
    address[3] = static_cast<uint8_t>('0' + tens);
    address[4] = static_cast<uint8_t>('0' + ones);
    address[5] = '\0';
}


void agri_getUnoNodeAddress(uint8_t clusterId,
                            uint8_t nodeLocalId,
                            uint8_t address[6]) {
    // Local Tier (ESP32 ↔ UNO)
    //
    // README examples:
    //   Cluster 1, Node 1  -> "N1A01"
    //   Cluster 1, Node 2  -> "N1A02"
    //   Cluster 2, Node 1  -> "N2A01"
    //   Cluster 10, Node 5 -> "NAA05"  (cluster 10 -> 'A', node 5 -> "05")
    //
    // Pattern: 'N' + clusterChar + 'A' + 2-digit decimal nodeLocalId

    char clusterChar = agri_cluster_char(clusterId);

    uint8_t nodeTens = (nodeLocalId / 10) % 10;   // 0..9
    uint8_t nodeOnes = nodeLocalId % 10;          // 0..9

    address[0] = 'N';
    address[1] = static_cast<uint8_t>(clusterChar);
    address[2] = 'A';
    address[3] = static_cast<uint8_t>('0' + nodeTens);
    address[4] = static_cast<uint8_t>('0' + nodeOnes);
    address[5] = '\0';
}


// =============================
// HEADER BUILDING
// =============================

void agri_buildHeader(
  AgriPacketHeader &hdr,
  AgriMsgType       type,
  AgriRole          srcRole,
  AgriRole          dstRole,
  uint8_t           srcId,
  uint8_t           dstId
) {
  hdr.magic  = AGRI_MAGIC_BYTE;
  hdr.msgType = static_cast<uint8_t>(type);
  hdr.srcRole = static_cast<uint8_t>(srcRole);
  hdr.dstRole = static_cast<uint8_t>(dstRole);
  hdr.srcId   = srcId;
  hdr.dstId   = dstId;
  hdr.seq     = g_seqCounter++;
}

// =============================
// INIT
// =============================

AgriResult agri_rf24_init_common(
  RF24    &radio,
  AgriRole role,
  uint8_t  thisId,
  uint8_t  extraId
) {
  if (!radio.begin()) {
    DBG_PRINTLN(F("[RF24] begin() failed"));
    return AgriResult::RF_FAIL;
  }

  radio.setChannel(AGRI_RF24_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(AGRI_RF24_RETRY_DELAY, AGRI_RF24_RETRY_COUNT);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
radio.setPALevel(RF24_PA_HIGH); 
  radio.setAddressWidth(5);  

  uint8_t rxAddr[6];

  switch (role) {
    case AgriRole::UNO_NODE:
      // thisId = nodeLocalId, extraId = clusterId
      agri_getUnoNodeAddress(extraId, thisId, rxAddr);
      break;
    case AgriRole::ESP32_CLUSTER:
      // thisId = clusterId
      agri_getClusterAddress(thisId, rxAddr);
      break;
    case AgriRole::UNOQ_MMC:
      agri_getUnoQAddress(rxAddr);
      break;
    default:
      DBG_PRINTLN(F("[RF24] Unknown role in init"));
      return AgriResult::BAD_PACKET;
  }

  radio.openReadingPipe(AGRI_RF24_PIPE_MAIN, rxAddr);
  radio.startListening();

#ifdef RF24_DEBUG
  agri_debugPrintAddr("[RF24] RX addr", rxAddr);
#endif

  return AgriResult::OK;
}

// =============================
// SEND / RECV
// =============================

AgriResult agri_rf24_sendTo(
  RF24            &radio,
  const uint8_t    destAddr[6],
  const void      *payload,
  uint8_t          len
) {
  if (len > AGRI_RF24_PAYLOAD_MAX) {
    DBG_PRINTLN(F("[RF24] Payload too large"));
    return AgriResult::BAD_PACKET;
  }

  radio.stopListening();
  radio.openWritingPipe(destAddr);

#ifdef RF24_DEBUG
  agri_debugPrintAddr("[RF24] TX addr", destAddr);
#endif

  bool ok = radio.write(payload, len);

  radio.startListening();

  if (!ok) {
    DBG_PRINTLN(F("[RF24] write() failed"));
    return AgriResult::RF_FAIL;
  }

  return AgriResult::OK;
}

AgriResult agri_rf24_recvBlocking(
  RF24    &radio,
  void    *buffer,
  uint8_t *len,
  uint16_t timeoutMs
) {
  uint32_t start = millis();

  while (!radio.available()) {
    if ((millis() - start) >= timeoutMs) {
      return AgriResult::TIMEOUT;
    }
    // yield lightly
    delay(1);
  }

  uint8_t pipe;
  uint8_t payloadSize = radio.getDynamicPayloadSize();
  if (payloadSize > *len) {
    // Truncate but still read the data to clear FIFO
    DBG_PRINTLN(F("[RF24] RX payload truncated"));
    payloadSize = *len;
  }

  radio.read(buffer, payloadSize);
  *len = payloadSize;

  // Basic magic filter if it's a packet with header
  AgriPacketHeader *hdr = reinterpret_cast<AgriPacketHeader*>(buffer);
  if (payloadSize >= sizeof(AgriPacketHeader)) {
    if (hdr->magic != AGRI_MAGIC_BYTE) {
      DBG_PRINTLN(F("[RF24] BAD magic, dropping"));
      return AgriResult::BAD_PACKET;
    }
  }

  return AgriResult::OK;
}

// =============================
// DEBUG HELPERS
// =============================

#ifdef RF24_DEBUG
void agri_debugPrintAddr(const char *label, const uint8_t addr[6]) {
  DBG_PRINT(label);
  DBG_PRINT(F(": "));
  for (int i = 0; i < 5; ++i) {
    DBG_PRINT((char)addr[i]);
  }
  DBG_PRINTLN("");
}
#endif


