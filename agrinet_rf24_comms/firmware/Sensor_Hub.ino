
// agrinet_rf24_comms/firmware/esp32_sensor_hub/esp32_sensor_hub.ino

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

/**
 * ESP32 Sensor Hub Node (SHN)
 * - Polls RS485 Modbus sensors (Weather, Solar, Soil)
 * - Sends chunks of telemetry to UNO_Q (MMC)
 * - Low power PA for desk testing
 */

// -----------------------------
// HW CONFIG
// -----------------------------
static const uint8_t PIN_RF24_CE  = 4;
static const uint8_t PIN_RF24_CSN = 5;

// RS485 pins (if used)
static const uint8_t PIN_RS485_RX = 16;
static const uint8_t PIN_RS485_TX = 17;
static const uint8_t PIN_RS485_DE = 21;

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// Identity
static const uint8_t MY_HUB_ID = 1;

// Addresses
static uint8_t g_addrMaster[6];     // Destination (UNO_Q)
static uint8_t g_addrSensorHub[6];  // My listening address

// State
static uint32_t g_lastTxMs = 0;
static const uint32_t TX_INTERVAL_MS = 10000;

// ---------------------------------
// LOCAL DEFINITIONS (Bypassing Library Sync Issue)
// ---------------------------------
struct AgriSensorsFrameHeader_L {
  AgriPacketHeader h;
  uint8_t  hubId;         
  uint8_t  frameType;     
  uint8_t  frameIndex;    
  uint8_t  frameCount;    
};

struct __attribute__((packed)) AgriSensorsWx0_L {
  AgriSensorsFrameHeader_L fh;
  int16_t  airTemp_cC;       
  uint16_t rh_cP;            
  uint16_t pressure_hPa10;   
  uint16_t wind_ms100;       
  uint16_t windDir_deg;      
  uint16_t rain_mm10;        
  uint16_t flags;
};

struct __attribute__((packed)) AgriSensorsWx1_L {
  AgriSensorsFrameHeader_L fh;
  uint16_t solar_Wm2;
  int16_t  solarTemp_cC;
  uint32_t light_lux;
  uint16_t flags;
};

// -----------------------------
// HELPER: Address Generation
// -----------------------------
// We implement this locally as we cannot edit common .cpp conveniently
void getSensorHubAddressLocal(uint8_t hubId, uint8_t address[6]) {
  // "HQWxx" -> 48 51 57 [30+digit] [30+digit]
  // This matches the prefix 'HQ' (48 51) but uses 'W' (57) as discriminator
  address[0] = 0x48; // H
  address[1] = 0x51; // Q
  address[2] = 0x57; // W
  address[3] = 0x30 + (hubId / 10);
  address[4] = 0x30 + (hubId % 10);
  address[5] = 0x00;
}

// -----------------------------
// SETUP
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n[ESP32_SHN] Booting (Sensor Hub)"));

  if (!radio.begin()) {
    Serial.println(F("[ESP32_SHN] RF24 Fail"));
    while(1) delay(100);
  }
  Serial.println(F("[ESP32_SHN] RF24 OK"));

  // Config - match system
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 15);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);

  // Addresses
  // Master is typically HQ000 (0x48 0x51 0x30 0x30 0x30)
  // But let's reuse the helper if available, or hardcode base
  g_addrMaster[0] = 0x48; g_addrMaster[1] = 0x51;
  g_addrMaster[2] = 0x30; g_addrMaster[3] = 0x30; g_addrMaster[4] = 0x30;

  getSensorHubAddressLocal(MY_HUB_ID, g_addrSensorHub);

  Serial.print(F("[ESP32_SHN] My Addr (HQW01): "));
  for(int i=0; i<5; i++) Serial.print(g_addrSensorHub[i], HEX);
  Serial.println();

  // Open pipes
  // We write to Master
  radio.openWritingPipe(g_addrMaster);
  // We listen on our address (for config/ping)
  radio.openReadingPipe(1, g_addrSensorHub);
  radio.startListening();
}

// -----------------------------
// LOOP
// -----------------------------
void loop() {
  // Receive config/ping
  if (radio.available()) {
     // TODO: Handle CONFIG or PING from MMC
     // e.g. set poll interval
     uint8_t buf[32];
     radio.read(buf, sizeof(buf));
     Serial.println(F("[ESP32_SHN] RX packet (ignored for now)"));
  }

  // Periodic Telemetry
  if (millis() - g_lastTxMs >= TX_INTERVAL_MS) {
     g_lastTxMs = millis();
     sendTelemetryBurst();
  }
}

// -----------------------------
// TELEMETRY SENDER
// -----------------------------
void sendTelemetryBurst() {
  Serial.println(F("[ESP32_SHN] Sending Burst..."));

  const uint8_t TYPE_SENSORS_TELEMETRY = 0x50;
  const uint8_t ROLE_SENSOR_HUB        = 4;
  const uint8_t ROLE_MMC               = 2;

  // Frame 0: WX0 (Air)
  {
    AgriSensorsWx0_L pkt;
    // Header
    pkt.fh.h.magic   = AGRI_MAGIC_BYTE;
    pkt.fh.h.msgType = TYPE_SENSORS_TELEMETRY;
    pkt.fh.h.srcRole = ROLE_SENSOR_HUB;
    pkt.fh.h.dstRole = ROLE_MMC;
    pkt.fh.h.srcId   = MY_HUB_ID;
    pkt.fh.h.dstId   = 0;
    pkt.fh.h.seq     = 0; 

    pkt.fh.hubId      = MY_HUB_ID;
    pkt.fh.frameType  = 0; // WX0
    pkt.fh.frameIndex = 0;
    pkt.fh.frameCount = 2; 

    // Fake Data
    pkt.airTemp_cC    = 2450; 
    pkt.rh_cP         = 5500; 
    pkt.pressure_hPa10= 10132; 
    pkt.wind_ms100    = 340;  
    pkt.windDir_deg   = 180;
    pkt.rain_mm10     = 0;
    pkt.flags         = 0;

    radio.stopListening();
    if (!radio.write(&pkt, sizeof(pkt))) {
      Serial.println(F("[ESP32_SHN] Frame 0 Tx Fail"));
    } else {
      Serial.println(F("[ESP32_SHN] Frame 0 Tx OK"));
    }
  }

  delay(20); // gap

  // Frame 1: WX1 (Solar)
  {
    AgriSensorsWx1_L pkt;
    pkt.fh.h.magic   = AGRI_MAGIC_BYTE;
    pkt.fh.h.msgType = TYPE_SENSORS_TELEMETRY;
    pkt.fh.h.srcRole = ROLE_SENSOR_HUB;
    pkt.fh.h.dstRole = ROLE_MMC;
    pkt.fh.h.srcId   = MY_HUB_ID;
    pkt.fh.h.dstId   = 0;
    pkt.fh.h.seq     = 0;

    pkt.fh.hubId      = MY_HUB_ID;
    pkt.fh.frameType  = 1; // WX1
    pkt.fh.frameIndex = 1;
    pkt.fh.frameCount = 2;

    pkt.solar_Wm2     = 850;
    pkt.solarTemp_cC  = 3000;
    pkt.light_lux     = 50000;
    pkt.flags         = 0;

    if (!radio.write(&pkt, sizeof(pkt))) {
       Serial.println(F("[ESP32_SHN] Frame 1 Tx Fail"));
    } else {
       Serial.println(F("[ESP32_SHN] Frame 1 Tx OK"));
    }
    radio.startListening();
  }
}
