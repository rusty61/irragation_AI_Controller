#include "agri_rf24_common.h"

#ifdef RF24_DEBUG
  #define DBG_PRINT(x)    Serial.print(x)
  #define DBG_PRINTLN(x)  Serial.println(x)
#else
  #define DBG_PRINT(x)    do {} while (0)
  #define DBG_PRINTLN(x)  do {} while (0)
#endif

// =============================
// INTERNAL STATE
// =============================

static uint16_t g_seqCounter = 1;

static void agri_getUnoQAddress(uint8_t address[6]) {
  // "HQ000"
  address[0] = 'H';
  address[1] = 'Q';
  address[2] = '0';
  address[3] = '0';
  address[4] = '0';
  address[5] = '\0';
}

// =============================
// ADDRESS HELPERS
// =============================

void agri_getClusterAddress(uint8_t clusterId, uint8_t address[6]) {
  // "HQA01".."HQA16"
  uint8_t tens = (clusterId / 10) % 10;
  uint8_t ones = clusterId % 10;

  address[0] = 'H';
  address[1] = 'Q';
  address[2] = 'A';
  address[3] = '0' + tens;
  address[4] = '0' + ones;
  address[5] = '\0';
}

void agri_getUnoNodeAddress(uint8_t clusterId, uint8_t nodeLocalId, uint8_t address[6]) {
  // "N1A01" style: N [clusterId digit] A [tens] [ones]
  uint8_t tens = (nodeLocalId / 10) % 10;
  uint8_t ones = nodeLocalId % 10;

  address[0] = 'N';
  address[1] = '0' + (clusterId % 10);
  address[2] = 'A';
  address[3] = '0' + tens;
  address[4] = '0' + ones;
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
  radio.setPALevel(RF24_PA_MAX);

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


