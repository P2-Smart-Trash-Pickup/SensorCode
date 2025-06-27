#include <SPI.h>
#include <RH_RF95.h>
#include <AESLib.h>

// -------------------------
// AES Configuration
// -------------------------
AESLib aesLib;

// Same AES key as endnode
byte aesKey[16] = {
  0x60, 0x3D, 0xEB, 0x10, 0x15, 0xCA, 0x71, 0xBE,
  0x2B, 0x73, 0xAE, 0xF0, 0x85, 0x7D, 0x77, 0x81
};

// -------------------------
// Hardware Configuration
// -------------------------
#define RFM95_CS 10
#define RFM95_INT 2
#define RFM95_RST 9
#define LED_PIN 13

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// -------------------------
// Protocol Configuration
// -------------------------
#define ACK_SIGNAL 0xAA
#define FLAG_SENSOR_MODE 0x01
const uint32_t allowedDevices[] = { 0x11223344 };

// -------------------------
// Packet Structures
// -------------------------
#pragma pack(push, 1)
struct LoRaLightPacket {
  uint8_t version;
  uint32_t deviceId;
  uint32_t packetCounter;
  uint16_t weight1;
  uint16_t weight2;
  uint16_t distances[10];
  float volume;
  uint8_t flags;
  uint16_t crc;
};

struct CommandPacket {
  uint8_t commandId;
  uint8_t payload[4];
  uint16_t crc;
};
#pragma pack(pop)

#define CMD_SET_SENSOR_MODE 0x01

// -------------------------
// Global State
// -------------------------
int requestedSensorMode = -1;  // -1 = no command pending

// -------------------------
// AES Functions
// -------------------------
void generateIV(uint32_t counter, byte iv[16]) {
  memset(iv, 0, 16);
  memcpy(iv, &counter, sizeof(counter));
}

int decryptPacket(byte* input, byte* output, int length, uint32_t counter) {
  byte iv[16];
  memset(iv, 0, 16);
  byte temp[length];
  int decryptLength = aesLib.decrypt(input, length, temp, aesKey, sizeof(aesKey), iv);

  uint32_t netLength;
  memcpy(&netLength, temp, 4);
  int orgLen = ntohl(netLength);

  memcpy(output, temp + 4, orgLen);
  return orgLen;
}

int peekPackage(uint8_t* input, uint32_t length) {
  byte iv[16];
  memset(iv, 0, 16);
  Serial.println();
  byte temp[length];
  int decryptLength = aesLib.decrypt(input, length, temp, aesKey, sizeof(aesKey), iv);
  if (decryptLength <= 0) {
    Serial.println("Decrypt failed");
  }
  Serial.print("Decrypted (raw): ");
  for (int i = 0; i < length; i++) Serial.print(temp[i], HEX);
  Serial.println();
  uint32_t netLength;
  memcpy(&netLength, temp, 4);
  Serial.println(netLength, HEX);
  int orgLen = ntohl(netLength);


  return orgLen;
}

int encryptPacket(byte* input, byte* output, int length, uint32_t counter) {
  byte iv[16];
  memcpy(&iv, 0, 16);
  int paddedLength = length;
  if (length % 16 != 0) {
    paddedLength = length + (16 - (length % 16));
  }
  return aesLib.encrypt(input, paddedLength, output, aesKey, sizeof(aesKey), iv);
}

// -------------------------
// Utility Functions
// -------------------------
void blinkLed(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(duration);
  }
}

void printPacketHex(uint8_t* data, size_t length) {
  Serial.println("RX Hex ");
  Serial.print("(");
  Serial.print(length);
  Serial.print("): ");
  for (size_t i = 0; i < length; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

uint16_t calculateCRC(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

// -------------------------
// Initialization
// -------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.print("AES Key: ");
  for (int i = 0; i < sizeof(aesKey); i++) Serial.print(aesKey[i], HEX);
  Serial.println();

  Serial.println("[STATUS] Starting LoRaLight Gateway");

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("[ERROR] LoRa init failed");
    while (1)
      ;
  }

  if (!rf95.setFrequency(868.0)) {
    Serial.println("[ERROR] setFrequency failed");
    while (1)
      ;
  }

  rf95.setSpreadingFactor(9);
  rf95.setSignalBandwidth(125000);
  rf95.setCodingRate4(5);
  rf95.setTxPower(20, false);

  // Initialize AES library
  //aesLib.set_mode(AES_CBC);
  aesLib.set_paddingmode((paddingMode)0);

  Serial.println("[STATUS] LoRa radio OK @ 868.0 MHz");

  blinkLed(2, 200);
  Serial.println("[STATUS] Gateway ready");
}

// -------------------------
// Main Application
// -------------------------
void loop() {
  handleSerialInput();  // Check if user typed a command

  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      //blinkLed(1, 50);
      if (len < 100) {
        /*
        printPacketHex(buf, len);
        Serial.println("Pekking length");
        Serial.println(len);
        int org_len = peekPackage(buf, len);
        Serial.println(org_len)
        */
        printPacketHex(buf, len);

        Serial.println("Pekking length");
        Serial.println(len);
        int org_len = peekPackage(buf, len);
        Serial.println("Org len: ");
        Serial.print(org_len);
        if (org_len > 10000) {
          return;
        }
        Serial.println();

        // Temporary buffer for decryption
        __attribute__((aligned(4))) uint8_t decrypted[org_len];
        // First try decrypting with counter=0 to get the actual counter
        Serial.println("Tryin deceryption");
        int decryptedLength = decryptPacket((byte*)&buf, decrypted, len, 0);
        Serial.println("First decryption attempt result:");
        printPacketHex((byte*)&decrypted, decryptedLength);
        //processReceivedPacket(buf, len);
      }
    }
  }
}

// -------------------------
// Serial Command Parsing
// -------------------------
void handleSerialInput() {
  static String inputBuffer = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      inputBuffer.trim();
      if (inputBuffer.startsWith("sm")) {
        if (inputBuffer.endsWith("-ToF")) {
          requestedSensorMode = 0;
          Serial.println("[COMMAND] Mode request queued: VL53L0X (ToF)");
        } else if (inputBuffer.endsWith("-US")) {
          requestedSensorMode = 1;
          Serial.println("[COMMAND] Mode request queued: HC-SR04 (Ultrasonic)");
        } else {
          Serial.println("[ERROR] Unknown sensor mode command");
        }
      } else {
        Serial.println("[ERROR] Unknown command");
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

// -------------------------
// Packet Processing - UPDATED
// -------------------------
void processReceivedPacket(uint8_t* buf, uint8_t len) {
  Serial.println("\n[GATEWAY] Packet received");
  Serial.println("Received encrypted packet hex:");

  printPacketHex(buf, len);

  // The encrypted packet will be larger than the original
  if (len < sizeof(LoRaLightPacket)) {
    Serial.println("[ERROR] Packet too small to be encrypted LoRaLightPacket");
    return;
  }


  Serial.println("Pekking length");
  Serial.println(len);
  int org_len = peekPackage(buf, len);
  Serial.println("Org len: ");
  Serial.print(org_len);
  if (org_len > 10000) {
    return;
  }
  Serial.println();

  // Temporary buffer for decryption
  __attribute__((aligned(4))) uint8_t decrypted[org_len];
  // First try decrypting with counter=0 to get the actual counter
  Serial.println("Tryin deceryption");
  int decryptedLength = decryptPacket((byte*)&buf, decrypted, len, 0);
  Serial.println("First decryption attempt result:");
  printPacketHex((byte*)&decrypted, decryptedLength);

  if (decryptedLength >= sizeof(LoRaLightPacket)) {
    // Extract the actual counter from the decrypted packet
    LoRaLightPacket* tempPacket = (LoRaLightPacket*)decrypted;
    uint32_t actualCounter = tempPacket->packetCounter;

    Serial.print("[DEBUG] Extracted packet counter: ");
    Serial.println(actualCounter);

    // Now decrypt again with the correct counter
    decryptedLength = decryptPacket(buf, decrypted, len, actualCounter);

    if (decryptedLength >= sizeof(LoRaLightPacket)) {  // new stuff from here
      LoRaLightPacket* packet = (LoRaLightPacket*)decrypted;

      // Verify CRC
      uint16_t calculated_crc = calculateCRC(decrypted, sizeof(LoRaLightPacket) - 2);
      if (packet->crc != calculated_crc) {
        Serial.println("[ERROR] CRC mismatch");
        return;
      }

      // Verify device ID
      bool deviceValid = false;
      for (auto id : allowedDevices) {
        if (packet->deviceId == id) {
          deviceValid = true;
          break;
        }
      }

      if (!deviceValid) {
        Serial.print("[SECURITY] Rejected device: 0x");
        Serial.println(packet->deviceId, HEX);
        return;
      }

      // Print decrypted packet details
      printPacketDetails(packet);

      // Send response
      if (requestedSensorMode != -1) {
        CommandPacket cmd;
        cmd.commandId = CMD_SET_SENSOR_MODE;
        cmd.payload[0] = requestedSensorMode;
        cmd.payload[1] = 0;
        cmd.payload[2] = 0;
        cmd.payload[3] = 0;
        cmd.crc = calculateCRC((uint8_t*)&cmd, sizeof(cmd) - 2);

        // Encrypt the command
        byte encryptedCmd[sizeof(CommandPacket) + 16];
        int encryptedCmdLength = encryptPacket((byte*)&cmd, encryptedCmd, sizeof(cmd), packet->packetCounter);

        if (rf95.send(encryptedCmd, encryptedCmdLength)) {
          rf95.waitPacketSent();
          Serial.print("[GATEWAY] Sent encrypted CMD_SET_SENSOR_MODE = ");
          Serial.println(requestedSensorMode == 0 ? "VL53L0X" : "HC-SR04");
          requestedSensorMode = -1;
        } else {
          Serial.println("[ERROR] Failed to send command");
        }
      } else {
        uint8_t ack = ACK_SIGNAL;
        if (rf95.send(&ack, sizeof(ack))) {
          rf95.waitPacketSent();
          Serial.println("[GATEWAY] Sent ACK");
        } else {
          Serial.println("[ERROR] Failed to send ACK");
        }
      }
    } else {
      Serial.print("[ERROR] Decryption failed - expected at least ");
      Serial.print(sizeof(LoRaLightPacket));
      Serial.print(" bytes, got ");
      Serial.println(decryptedLength);

      // Debug output of decrypted data
      Serial.println("Decrypted data hex:");
      printPacketHex(decrypted, decryptedLength);
    }
  } else {
    Serial.println("[ERROR] Initial decryption failed");
  }
}

void printPacketDetails(LoRaLightPacket* packet) {
  Serial.println("-----------------------");
  Serial.print("Device: 0x");
  Serial.println(packet->deviceId, HEX);
  Serial.print("Counter: ");
  Serial.println(packet->packetCounter);
  Serial.print("Sensor Mode: ");
  Serial.println((packet->flags & FLAG_SENSOR_MODE) ? "HC-SR04" : "VL53L0X");
  Serial.print("Weights: ");
  Serial.print(packet->weight1);
  Serial.print("g, ");
  Serial.print(packet->weight2);
  Serial.println("g");
  Serial.print("Volume: ");
  Serial.println(packet->volume);

  Serial.println("Distances (mm):");
  for (int i = 0; i < 10; i++) {
    Serial.print("  ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(packet->distances[i]);
  }

  Serial.print("RSSI: ");
  Serial.print(rf95.lastRssi());
  Serial.println(" dBm");
  Serial.println("-----------------------");
}
