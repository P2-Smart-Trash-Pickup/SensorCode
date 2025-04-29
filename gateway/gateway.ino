#include <SPI.h>
#include <RH_RF95.h>

// Dragino Shield Pins
#define RFM95_CS 10
#define RFM95_INT 2
#define RFM95_RST 9

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1);
  }
  
  rf95.setFrequency(868.0);
  rf95.setSpreadingFactor(9);
  rf95.setSignalBandwidth(125000);
  Serial.println("Gateway ready");
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.print("RSSI: "); Serial.println(rf95.lastRssi());
      Serial.print("Received packet from node: ");
      Serial.println(buf[0], HEX); // Device ID
      
      Serial.print("Sequence: ");
      uint32_t seq = (uint32_t)buf[1] << 24 | (uint32_t)buf[2] << 16 | 
                     (uint32_t)buf[3] << 8 | (uint32_t)buf[4];
      Serial.println(seq);
      
      Serial.print("Weight1: ");
      uint16_t weight1 = (buf[5] << 8) | buf[6];
      Serial.print(weight1); Serial.println(" g");
      
      Serial.print("Weight2: ");
      uint16_t weight2 = (buf[7] << 8) | buf[8];
      Serial.print(weight2); Serial.println(" g");
      
      Serial.println("Last 10 distances (mm):");
      for (int i = 0; i < 10; i++) {
        uint16_t dist = (buf[9 + i*2] << 8) | buf[10 + i*2];
        Serial.print(dist); Serial.print(" ");
      }
      Serial.println("\n---");
    }
  }
}