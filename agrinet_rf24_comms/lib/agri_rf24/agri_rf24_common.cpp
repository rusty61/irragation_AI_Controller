/**
 * @file agri_rf24_common.cpp
 * @brief AgriNet RF24 Communications Stack - Implementation
 * 
 * Shared RF24 initialization, send/receive functions, and address helpers
 * for the AgriNet multi-node irrigation system.
 */

#include "agri_rf24_common.h"
#include <string.h>

/*============================================================================
 * Address Constants
 *==========================================================================*/

/** Base address for UNO_Q master: "HQ000" */
static const uint8_t MASTER_BASE_ADDR[6] = "HQ000";

/** Base prefix for cluster addresses: "HQA" */
static const char CLUSTER_PREFIX[] = "HQA";

/** Base prefix for UNO node addresses: "N" */
static const char NODE_PREFIX[] = "N";

/*============================================================================
 * Address Helper Functions
 *==========================================================================*/

void agri_getMasterAddress(uint8_t address[6])
{
    memcpy(address, MASTER_BASE_ADDR, 6);
}

void agri_getClusterAddress(uint8_t clusterId, uint8_t address[6])
{
    // Cluster addresses: "HQA01", "HQA02", ..., "HQA16"
    // Clamp cluster ID to valid range (1-16)
    if (clusterId < 1) clusterId = 1;
    if (clusterId > AGRI_MAX_CLUSTERS) clusterId = AGRI_MAX_CLUSTERS;
    
    address[0] = 'H';
    address[1] = 'Q';
    address[2] = 'A';
    address[3] = '0' + (clusterId / 10);
    address[4] = '0' + (clusterId % 10);
    address[5] = '\0';
}

void agri_getUnoNodeAddress(uint8_t clusterId, uint8_t nodeLocalId, uint8_t address[6])
{
    // Node addresses: "N1A01", "N1A02", ..., "N16A08"
    // Pattern: N<clusterId>A<nodeId>
    // Clamp IDs to valid range
    if (clusterId < 1) clusterId = 1;
    if (clusterId > AGRI_MAX_CLUSTERS) clusterId = AGRI_MAX_CLUSTERS;
    if (nodeLocalId < 1) nodeLocalId = 1;
    if (nodeLocalId > AGRI_MAX_NODES_PER_CLUSTER) nodeLocalId = AGRI_MAX_NODES_PER_CLUSTER;
    
    // For clusters > 9, use hex digit (A-F for 10-15, G for 16)
    char clusterChar;
    if (clusterId <= 9) {
        clusterChar = '0' + clusterId;
    } else {
        clusterChar = 'A' + (clusterId - 10);
    }
    
    address[0] = 'N';
    address[1] = clusterChar;
    address[2] = 'A';
    address[3] = '0' + (nodeLocalId / 10);
    address[4] = '0' + (nodeLocalId % 10);
    address[5] = '\0';
}

/*============================================================================
 * RF24 Core Functions
 *==========================================================================*/

AgriResult agri_rf24_init_common(RF24 &radio, AgriRole role)
{
    // Initialize RF24
    if (!radio.begin()) {
        AGRI_DEBUG_PRINTLN(F("[AGRI] RF24 init failed - hardware error"));
        return AgriResult::RF_FAIL;
    }
    
    // Configure RF24 settings as per specification
    radio.setChannel(AGRI_RF24_CHANNEL);
    radio.setDataRate(AGRI_RF24_DATA_RATE);
    radio.setPALevel(AGRI_RF24_PA_LEVEL);
    radio.setCRCLength(AGRI_RF24_CRC_LENGTH);
    radio.setRetries(AGRI_RF24_RETRY_DELAY, AGRI_RF24_RETRY_COUNT);
    
    // Enable auto-acknowledgment
    radio.setAutoAck(true);
    
    // Enable dynamic payloads
    radio.enableDynamicPayloads();
    
    AGRI_DEBUG_PRINT(F("[AGRI] RF24 initialized, role: "));
    AGRI_DEBUG_PRINTLN((uint8_t)role);
    
    return AgriResult::OK;
}

AgriResult agri_rf24_send(RF24 &radio, const uint8_t *destAddr, 
                          const void *packet, uint8_t len)
{
    if (destAddr == nullptr || packet == nullptr || len == 0) {
        AGRI_DEBUG_PRINTLN(F("[AGRI] Send failed - invalid parameters"));
        return AgriResult::BAD_PACKET;
    }
    
    // Open writing pipe to destination
    radio.stopListening();
    radio.openWritingPipe(destAddr);
    
    AGRI_DEBUG_PRINT(F("[AGRI] Sending to: "));
    AGRI_DEBUG_PRINTLN((const char*)destAddr);
    
    // Attempt to send with auto-retry
    bool success = radio.write(packet, len);
    
    // Return to listening mode
    radio.startListening();
    
    if (!success) {
        AGRI_DEBUG_PRINTLN(F("[AGRI] Send failed - write error"));
        return AgriResult::RF_FAIL;
    }
    
    AGRI_DEBUG_PRINTLN(F("[AGRI] Send successful"));
    return AgriResult::OK;
}

AgriResult agri_rf24_recv(RF24 &radio, void *packet, uint8_t *len, 
                          uint16_t timeout_ms)
{
    if (packet == nullptr || len == nullptr || *len == 0) {
        AGRI_DEBUG_PRINTLN(F("[AGRI] Recv failed - invalid parameters"));
        return AgriResult::BAD_PACKET;
    }
    
    radio.startListening();
    
    unsigned long startTime = millis();
    uint8_t pipeNum;
    
    // Wait for packet or timeout
    while (!radio.available(&pipeNum)) {
        if (timeout_ms > 0 && (millis() - startTime) >= timeout_ms) {
            return AgriResult::TIMEOUT;
        }
        // Small delay to prevent busy-waiting
        if (timeout_ms > 0) {
            delay(1);
        } else {
            // No wait mode - return immediately if no packet
            return AgriResult::TIMEOUT;
        }
    }
    
    // Get payload size
    uint8_t payloadSize = radio.getDynamicPayloadSize();
    
    if (payloadSize > *len) {
        // Buffer too small, flush the packet
        radio.flush_rx();
        AGRI_DEBUG_PRINTLN(F("[AGRI] Recv failed - buffer too small"));
        return AgriResult::BAD_PACKET;
    }
    
    // Read the payload
    radio.read(packet, payloadSize);
    *len = payloadSize;
    
    // Validate magic byte if this is a proper AgriNet packet
    const AgriPacketHeader *header = (const AgriPacketHeader *)packet;
    if (payloadSize >= sizeof(AgriPacketHeader) && header->magic != AGRI_MAGIC_BYTE) {
        AGRI_DEBUG_PRINTLN(F("[AGRI] Recv failed - bad magic byte"));
        return AgriResult::BAD_PACKET;
    }
    
    AGRI_DEBUG_PRINT(F("[AGRI] Received "));
    AGRI_DEBUG_PRINT(payloadSize);
    AGRI_DEBUG_PRINTLN(F(" bytes"));
    
    return AgriResult::OK;
}

/*============================================================================
 * Packet Helper Functions
 *==========================================================================*/

void agri_initPacketHeader(AgriPacketHeader *header, uint8_t msgType,
                           AgriRole srcRole, AgriRole dstRole,
                           uint8_t srcId, uint8_t dstId, uint16_t seq)
{
    if (header == nullptr) return;
    
    header->magic   = AGRI_MAGIC_BYTE;
    header->msgType = msgType;
    header->srcRole = static_cast<uint8_t>(srcRole);
    header->dstRole = static_cast<uint8_t>(dstRole);
    header->srcId   = srcId;
    header->dstId   = dstId;
    header->seq     = seq;
}

bool agri_validatePacketHeader(const AgriPacketHeader *header)
{
    if (header == nullptr) {
        return false;
    }
    
    // Check magic byte
    if (header->magic != AGRI_MAGIC_BYTE) {
        AGRI_DEBUG_PRINTLN(F("[AGRI] Invalid magic byte"));
        return false;
    }
    
    // Validate roles
    if (header->srcRole > static_cast<uint8_t>(AgriRole::ROLE_UNO_Q_MMC) ||
        header->dstRole > static_cast<uint8_t>(AgriRole::ROLE_UNO_Q_MMC)) {
        AGRI_DEBUG_PRINTLN(F("[AGRI] Invalid role in header"));
        return false;
    }
    
    return true;
}
