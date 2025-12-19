
// agrinet_rf24_comms/firmware/esp32_sensor_hub/esp32_sensor_hub.ino

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "agri_rf24_common.h"

// -----------------------------
// HW CONFIG
// -----------------------------
static const uint8_t PIN_RF24_CE  = 4;
static const uint8_t PIN_RF24_CSN = 5;

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);

// Identity
static const uint8_t MY_HUB_ID = 1;

// Addresses
static uint8_t g_addrMaster[6];     // Destination (UNO_Q)
static uint8_t g_addrSensorHub[6];  // My listening address

// State
static uint32_t g_lastTxMs = 0;
static const uint32_t TX_INTERVAL_MS = 10000;

// -----------------------------
// HELPER: Address Generation
// -----------------------------
void getSensorHubAddressLocal(uint8_t hubId, uint8_t address[6]) {
  // "HQWxx" -> 48 51 57 [30+digit] [30+digit]
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

  radio.setChannel(AGRI_RF24_CHANNEL);
  radio.setPALevel(RF24_PA_LOW); 
  radio.setDataRate(AGRI_RF24_DATARATE);
  radio.setRetries(5, 15);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);

  // Addresses
  // Master is "HQ000"
  g_addrMaster[0] = 0x48; g_addrMaster[1] = 0x51;
  g_addrMaster[2] = 0x30; g_addrMaster[3] = 0x30; g_addrMaster[4] = 0x30; // HQ000

  getSensorHubAddressLocal(MY_HUB_ID, g_addrSensorHub);

  Serial.print(F("[ESP32_SHN] My Addr (HQW01): "));
  for(int i=0; i<5; i++) Serial.print(g_addrSensorHub[i], HEX);
  Serial.println();

  // Uplink
  radio.openWritingPipe(g_addrMaster);
  // Downlink
  radio.openReadingPipe(1, g_addrSensorHub);
  radio.startListening();
}

// -----------------------------
// LOOP
// -----------------------------
void loop() {
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

  // Frame count: WX0 + WX1 + 2 Soils = 4 frames
  const uint8_t TOTAL_FRAMES = 4;

  radio.stopListening();
  
  // Frame 0: WX0 (Air)
  {
    AgriSensorsWx0 pkt;
    // Header
    pkt.fh.h.magic   = AGRI_MAGIC_BYTE;
    pkt.fh.h.msgType = static_cast<uint8_t>(AgriMsgType::SENSORS_TELEMETRY);
    pkt.fh.h.srcRole = static_cast<uint8_t>(AgriRole::ESP32_SENSOR_HUB);
    pkt.fh.h.dstRole = static_cast<uint8_t>(AgriRole::UNOQ_MMC);
    pkt.fh.h.srcId   = MY_HUB_ID;
    pkt.fh.h.dstId   = 0;
    pkt.fh.h.seq     = 0; 

    pkt.fh.hubId      = MY_HUB_ID;
    pkt.fh.frameType  = 0; // WX0
    pkt.fh.frameIndex = 0;
    pkt.fh.frameCount = TOTAL_FRAMES;

    pkt.airTemp_cC    = 2450; 
    pkt.rh_cP         = 5500; 
    pkt.pressure_hPa10= 10132; 
    pkt.wind_ms100    = 340;  
    pkt.windDir_deg   = 180;
    pkt.rain_mm10     = 0;
    pkt.flags         = 0;

    if (radio.write(&pkt, sizeof(pkt))) Serial.println(F("  Frame 0 OK"));
    else Serial.println(F("  Frame 0 Fail"));
  }
  
  delay(15); 

  // Frame 1: WX1 (Solar)
  {
    AgriSensorsWx1 pkt;
    pkt.fh.h.magic   = AGRI_MAGIC_BYTE;
    pkt.fh.h.msgType = static_cast<uint8_t>(AgriMsgType::SENSORS_TELEMETRY);
    pkt.fh.h.srcRole = static_cast<uint8_t>(AgriRole::ESP32_SENSOR_HUB);
    pkt.fh.h.dstRole = static_cast<uint8_t>(AgriRole::UNOQ_MMC);
    pkt.fh.h.srcId   = MY_HUB_ID;
    pkt.fh.h.dstId   = 0;
    pkt.fh.h.seq     = 0;

    pkt.fh.hubId      = MY_HUB_ID;
    pkt.fh.frameType  = 1; // WX1
    pkt.fh.frameIndex = 1;
    pkt.fh.frameCount = TOTAL_FRAMES;

    pkt.solar_Wm2     = 850;
    pkt.solarTemp_cC  = 3000;
    pkt.light_lux     = 50000;
    pkt.flags         = 0;

    if (radio.write(&pkt, sizeof(pkt))) Serial.println(F("  Frame 1 OK"));
    else Serial.println(F("  Frame 1 Fail"));
  }

  delay(15);

  // Frame 2 & 3: Soil Probes (Mocked Addresses 3 and 4)
  for (int i = 0; i < 2; i++) {
    AgriSensorsSoil pkt;
    pkt.fh.h.magic   = AGRI_MAGIC_BYTE;
    pkt.fh.h.msgType = static_cast<uint8_t>(AgriMsgType::SENSORS_TELEMETRY);
    pkt.fh.h.srcRole = static_cast<uint8_t>(AgriRole::ESP32_SENSOR_HUB);
    pkt.fh.h.dstRole = static_cast<uint8_t>(AgriRole::UNOQ_MMC);
    pkt.fh.h.srcId   = MY_HUB_ID;
    pkt.fh.h.dstId   = 0;
    pkt.fh.h.seq     = 0;

    pkt.fh.hubId      = MY_HUB_ID;
    pkt.fh.frameType  = 2; // SOIL
    pkt.fh.frameIndex = 2 + i;
    pkt.fh.frameCount = TOTAL_FRAMES;

    pkt.soilAddr      = 3 + i;
    pkt.vwc_cP        = 3500 + (i * 500); // 35.00% and 40.00%
    pkt.soilTemp_cC   = 2250 + (i * 10);  // 22.50 C
    pkt.ec_uScm       = 0;
    pkt.flags         = 0;

    if (radio.write(&pkt, sizeof(pkt))) {
       Serial.print(F("  Soil Packet ")); Serial.print(i); Serial.println(F(" OK"));
    } else {
       Serial.print(F("  Soil Packet ")); Serial.print(i); Serial.println(F(" Fail"));
    }
    delay(15);
  }

  radio.startListening();
}
