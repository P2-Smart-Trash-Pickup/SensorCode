#include <HX711_ADC.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <EEPROM.h>
#include <AESLib.h>

// -------------------------
// LoRaLight Protocol
// -------------------------ard
#pragma pack(push, 1)
struct LoRaLightPacket {
  uint8_t version = 0x01;
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

// -------------------------
// AES Configuration
// -------------------------
AESLib aesLib;

byte aesKey[16] = {
  0x60, 0x3D, 0xEB, 0x10, 0x15, 0xCA, 0x71, 0xBE,
  0x2B, 0x73, 0xAE, 0xF0, 0x85, 0x7D, 0x77, 0x81
};

// -------------------------
// System Configuration
// -------------------------
float unitVolume = 1.0;
#define RFM95_CS 10
#define RFM95_INT 2
#define RFM95_RST 9
#define LED_PIN 13

#define MAX_RETRIES 3
#define ACK_TIMEOUT 10000
#define ACK_SIGNAL 0xAA
#define FLAG_SENSOR_MODE 0x01
#define CMD_SET_SENSOR_MODE 0x01

const uint32_t DEVICE_ID = 0x11223344;
#define EEPROM_FCNT_ADDR 0
const unsigned long txInterval = 10000;

// -------------------------
// Debug Configuration
// -------------------------
#define DEBUG_RADIO_STATS 1
#define DEBUG_PACKET_CONTENT 1
#define DEBUG_TIMING 1
#define DEBUG_SENSOR_READINGS 1

// -------------------------
// Sensor Configuration
// -------------------------
//const int HX711_dout = 4;
//const int HX711_sck = 5;
//HX711_ADC LoadCell(HX711_dout, HX711_sck);

VL53L0X distanceSensor;
#define echoPin 6
#define trigPin 3
int sensorMode = 0;

// -------------------------
// Buffers
// -------------------------
const int weightHistorySize = 100;
float weightHistory[weightHistorySize];
int weightHistoryIndex = 0;
bool weightHistoryFilled = false;

#define DISTANCE_HISTORY_SIZE 10
uint16_t distanceHistory[DISTANCE_HISTORY_SIZE];
int distanceHistoryIndex = 0;

// -------------------------
// State
// -------------------------
RH_RF95 rf95(RFM95_CS, RFM95_INT);
unsigned long lastTxTime = 0;
uint32_t currentCounter = 0;
uint32_t txSuccessCount = 0;
uint32_t txFailCount = 0;
uint32_t commandCount = 0;

// -------------------------
// AES Functions
// -------------------------
void generateIV(uint32_t counter, byte iv[16]) {
  memset(iv, 0, 16);
  memcpy(iv, &counter, sizeof(counter));
}



int encryptPacket(byte* input, byte* output, int paddedLength, uint32_t length) {

  byte iv[16];
  memset(iv, 0, 16);

  uint32_t netLength = htonl(length);
  byte paddedInput[paddedLength];
  memcpy(paddedInput, &netLength, 4);
  memcpy(paddedInput + 4, input, length);
  memset(paddedInput + 4 + length, 0, paddedLength - (length + 4));

  return aesLib.encrypt(paddedInput, paddedLength, output, aesKey, sizeof(aesKey), iv);
}

int decryptPacket(byte* input, byte* output, int length) {
  byte iv[16];
  generateIV(0, iv);
  return aesLib.decrypt(input, length, output, aesKey, sizeof(aesKey), iv);
}

// -------------------------
// Function Declarations
// -------------------------
void preparePacket(LoRaLightPacket& packet);
bool attemptTransmission(LoRaLightPacket& packet);
bool waitForAck();
void executeCommand(CommandPacket* cmd);
void updateDistanceReading();
void handleSerialCommands();
void printSystemStatus();
void printPacketHex(uint8_t* data, size_t length);
uint16_t calculateCRC(const uint8_t* data, size_t length);
float getAverageWeight();
void blinkLed(int times, int duration);
void initLoRa();
void initSensors();
uint32_t readFCnt();
void writeFCnt(uint32_t val);

// -------------------------
// Setup
// -------------------------
void setup() {

  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.print("AES Key: ");
  for (int i = 0; i < sizeof(aesKey); i++) Serial.print(aesKey[i], HEX);
  Serial.println();

  Serial.println(F("[STATUS] Starting LoRaLight End Node"));
  Serial.println(F("[DEBUG] Compiled: " __DATE__ " " __TIME__));

  EEPROM.begin();
  currentCounter = readFCnt();
  initLoRa();
  initSensors();

  // Initialize AES library
  //aesLib.set_mode(AES_CBC);
  aesLib.set_paddingmode((paddingMode)0);

  Serial.println(F("[STATUS] System ready"));
  blinkLed(2, 200);
  printSystemStatus();
}

// -------------------------
// Main Loop
// -------------------------
void loop() {
  static unsigned long lastStatusTime = 0;
  //
  //  if (LoadCell.update()) {
  //    float weight = LoadCell.getData();
  //    weightHistory[weightHistoryIndex] = weight;
  //    weightHistoryIndex = (weightHistoryIndex + 1) % weightHistorySize;
  //    if (weightHistoryIndex == 0) weightHistoryFilled = true;
  //
  //#if DEBUG_SENSOR_READINGS
  //    static unsigned long lastWeightPrint = 0;
  //    if (millis() - lastWeightPrint > 1000) {
  //      Serial.print(F("[SENSOR] Weight: "));
  //      Serial.print(weight);
  //      Serial.println(F("g"));
  //      lastWeightPrint = millis();
  //    }
  //#endif
  //}

  updateDistanceReading();

  if (millis() - lastTxTime >= txInterval) {
#if DEBUG_TIMING
    Serial.print(F("[TIMING] Time since last TX: "));
    Serial.print(millis() - lastTxTime);
    Serial.println(F("ms"));
#endif

    LoRaLightPacket packet;
    preparePacket(packet);

    if (attemptTransmission(packet)) {
      currentCounter++;
      txSuccessCount++;
      writeFCnt(currentCounter);
      blinkLed(1, 100);
    } else {
      txFailCount++;
      blinkLed(3, 200);
    }

    lastTxTime = millis();
  }

  if (millis() - lastStatusTime > 30000) {
    printSystemStatus();
    lastStatusTime = millis();
  }

  handleSerialCommands();
}

// -------------------------
// Init LoRa
// -------------------------
void initLoRa() {
  Serial.println(F("[RADIO] Initializing LoRa..."));

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println(F("[ERROR] LoRa init failed"));
    while (1)
      ;
  }

  if (!rf95.setFrequency(868.0)) {
    Serial.println(F("[ERROR] setFrequency failed"));
    while (1)
      ;
  }

  rf95.setSpreadingFactor(9);
  rf95.setSignalBandwidth(125000);
  rf95.setCodingRate4(5);
  rf95.setTxPower(20, false);

  Serial.println(F("[RADIO] LoRa radio OK @ 868.0 MHz"));

#if DEBUG_RADIO_STATS
  Serial.println(F("[RADIO] Current Configuration:"));
  Serial.println(F("  Frequency: 868.0 MHz"));
  Serial.println(F("  Spreading Factor: 9"));
  Serial.println(F("  Signal Bandwidth: 125000 Hz"));
  Serial.println(F("  Coding Rate: 4/5"));
  Serial.println(F("  Tx Power: 20 dBm"));
#endif
}

// -------------------------
// Init Sensors
// -------------------------
void initSensors() {
  Serial.println(F("[SENSOR] Initializing sensors..."));

  //  LoadCell.begin();
  //  LoadCell.start(2000, true);
  //  if (LoadCell.getTareTimeoutFlag()) {
  //    Serial.println(F("[ERROR] HX711 timeout"));
  //    while (1)
  //      ;
  //  }
  //  LoadCell.setCalFactor(-40.10);
  //  Serial.println(F("[SENSOR] Load cell initialized"));

  if (sensorMode == 0) {
    Wire.begin();
    if (!distanceSensor.init()) {
      Serial.println(F("[ERROR] VL53L0X init failed"));
      while (1)
        ;
    }
    distanceSensor.startContinuous();
    Serial.println(F("[SENSOR] VL53L0X initialized (ToF mode)"));
  } else {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.println(F("[SENSOR] HC-SR04 initialized (Ultrasonic mode)"));
  }
}

// -------------------------
// Packet Creation
// -------------------------
void preparePacket(LoRaLightPacket& packet) {
  packet.deviceId = DEVICE_ID;
  packet.packetCounter = currentCounter;
  //packet.weight1 = (uint16_t)constrain(getAverageWeight(), 0, 65535);
  //packet.weight2 = (uint16_t)constrain(weightHistory[(weightHistoryIndex - 1 + weightHistorySize) % weightHistorySize], 0, 65535);
  packet.weight1 = 0;
  packet.weight2 = 0;

  for (int i = 0; i < 10; i++) {
    int idx = (distanceHistoryIndex - 1 - i + DISTANCE_HISTORY_SIZE) % DISTANCE_HISTORY_SIZE;
    packet.distances[i] = distanceHistory[idx];
  }

  packet.volume = unitVolume;  
  packet.flags = sensorMode & FLAG_SENSOR_MODE;
  packet.crc = calculateCRC((uint8_t*)&packet, sizeof(packet) - 2);

#if DEBUG_PACKET_CONTENT
  Serial.println(F("[PACKET] Prepared packet:"));
  Serial.print(F("  Version: 0x"));
  Serial.println(packet.version, HEX);
  Serial.print(F("  Device ID: 0x"));
  Serial.println(packet.deviceId, HEX);
  Serial.print(F("  Counter: "));
  Serial.println(packet.packetCounter);
  Serial.print(F("  Weight1: "));
  Serial.print(packet.weight1);
  Serial.println(F("g"));
  Serial.print(F("  Weight2: "));
  Serial.print(packet.weight2);
  Serial.println(F("g"));
  Serial.print(F("  Volume: ")); Serial.println(packet.volume); 
  Serial.print(F("  Flags: 0x"));
  Serial.println(packet.flags, HEX);
  Serial.print(F("  CRC: 0x"));
  Serial.println(packet.crc, HEX);
#endif
}

// -------------------------
// Transmission
// -------------------------
bool attemptTransmission(LoRaLightPacket& packet) {
  for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
    Serial.print(F("[TRANSMIT] Attempt "));
    Serial.print(retry + 1);
    Serial.print(F("/"));
    Serial.println(MAX_RETRIES);

    // calculate padded length
    uint32_t length = sizeof(packet);
    int paddedLength = length + 4;
    if (length % 16 != 0) {
      paddedLength = length + (16 - (length % 16));
    }
    // Encrypt the packet

    __attribute__((aligned(4))) uint8_t encrypted[paddedLength];
    int encryptedLength = encryptPacket((byte*)&packet, encrypted, paddedLength, length);

    Serial.println("Original packet hex:");
    printPacketHex((byte*)&packet, sizeof(packet));

    Serial.println("Encrypted packet hex:");
    printPacketHex((byte*)&encrypted, encryptedLength);

    unsigned long txStart = millis();
    if (rf95.send(encrypted, encryptedLength)) {
      Serial.println("Balls!");  // DEBUG
      rf95.waitPacketSent();
#if DEBUG_TIMING
      Serial.print(F("[TIMING] TX duration: "));
      Serial.print(millis() - txStart);
      Serial.println(F("ms"));
#endif

      Serial.println(F("[TRANSMIT] Packet sent, waiting for ACK..."));
      if (waitForAck()) {
        Serial.println(F("[SUCCESS] ACK or command received"));
        return true;
      }
    } else {
      Serial.println(F("[ERROR] Send failed"));
    }

    delay(1000 * (retry + 1));
  }

  Serial.println(F("[ERROR] Max retries exceeded"));
  return false;
}

bool waitForAck() {
  unsigned long startTime = millis();
  while (millis() - startTime < ACK_TIMEOUT) {
    if (rf95.available()) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      unsigned long recvStart = millis();
      if (rf95.recv(buf, &len)) {
#if DEBUG_TIMING
        Serial.print(F("[TIMING] RX duration: "));
        Serial.print(millis() - recvStart);
        Serial.println(F("ms"));
#endif

        Serial.print(F("[RECV] Received "));
        Serial.print(len);
        Serial.println(F(" bytes"));

        printPacketHex(buf, len);

        if (len == 1 && buf[0] == ACK_SIGNAL) {
          Serial.println(F("[ACK] Received valid ACK"));
          return true;
        } else if (len > sizeof(CommandPacket)) {
          // Try to decrypt as a command packet
          byte decrypted[sizeof(CommandPacket) + 16];
          int decryptedLength = decryptPacket(buf, decrypted, len);

          if (decryptedLength == sizeof(CommandPacket)) {
            CommandPacket* cmd = (CommandPacket*)decrypted;
            uint16_t crc = calculateCRC(decrypted, sizeof(CommandPacket) - 2);

            Serial.print(F("[COMMAND] Received command ID: 0x"));
            Serial.println(cmd->commandId, HEX);
            Serial.print(F("[COMMAND] CRC: Received 0x"));
            Serial.print(cmd->crc, HEX);
            Serial.print(F(" Calculated 0x"));
            Serial.println(crc, HEX);

            if (cmd->crc == crc) {
              commandCount++;
              executeCommand(cmd);
              return true;
            } else {
              Serial.println(F("[ERROR] CRC mismatch in command"));
            }
          }
        } else {
          Serial.println(F("[WARN] Invalid ACK/command packet"));
        }
      }
    }
#if DEBUG_TIMING
    static unsigned long lastDot = 0;
    if (millis() - lastDot > 500) {
      Serial.print(F("."));
      lastDot = millis();
    }
#endif
    delay(10);
  }

  Serial.println(F("\n[ERROR] ACK timeout"));
  return false;
}

// -------------------------
// Command Handling
// -------------------------
void executeCommand(CommandPacket* cmd) {
  Serial.print(F("[COMMAND] Executing command 0x"));
  Serial.println(cmd->commandId, HEX);

  switch (cmd->commandId) {
    case CMD_SET_SENSOR_MODE:
      sensorMode = cmd->payload[0];
      Serial.print(F("[COMMAND] Setting sensor mode to: "));
      Serial.println(sensorMode == 0 ? "VL53L0X" : "HC-SR04");
      initSensors();
      break;

    default:
      Serial.println(F("[COMMAND] Unknown command"));
      break;
  }
}

// -------------------------
// Distance Reading
// -------------------------
void updateDistanceReading() {
  static unsigned long lastDistancePrint = 0;

  if (sensorMode == 0) {
    uint16_t reading = distanceSensor.readRangeContinuousMillimeters();
    if (!distanceSensor.timeoutOccurred()) {
      distanceHistory[distanceHistoryIndex] = reading;
      distanceHistoryIndex = (distanceHistoryIndex + 1) % DISTANCE_HISTORY_SIZE;

#if DEBUG_SENSOR_READINGS
      if (millis() - lastDistancePrint > 1000) {
        Serial.print(F("[SENSOR] ToF Distance: "));
        Serial.print(reading);
        Serial.println(F("mm"));
        lastDistancePrint = millis();
      }
#endif
    }
  } else {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    uint16_t distance = (duration * 0.0344) / 2.0;
    distanceHistory[distanceHistoryIndex] = distance;
    distanceHistoryIndex = (distanceHistoryIndex + 1) % DISTANCE_HISTORY_SIZE;

#if DEBUG_SENSOR_READINGS
    if (millis() - lastDistancePrint > 1000) {
      Serial.print(F("[SENSOR] US Distance: "));
      Serial.print(distance);
      Serial.println(F("mm"));
      lastDistancePrint = millis();
    }
#endif
  }
}

// -------------------------
// Serial Commands
// -------------------------
void handleSerialCommands() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 't':
        //LoadCell.tareNoDelay();
        Serial.println(F("[ACTION] Tare initiated"));
        break;
      case '0':
      case '1':
        sensorMode = c - '0';
        initSensors();
        Serial.print(F("[CONFIG] Sensor mode set to "));
        Serial.println(sensorMode == 0 ? "VL53L0X" : "HC-SR04");
        break;
      case 's':
        Serial.println(F("[STATUS] Force sending packet"));
        lastTxTime = 0;
        break;
      case 'd':
        printSystemStatus();
        break;
    }
  }
}

// -------------------------
// EEPROM
// -------------------------
uint32_t readFCnt() {
  uint32_t val;
  EEPROM.get(EEPROM_FCNT_ADDR, val);
  if (val == 0xFFFFFFFF) {
    val = 0;
    EEPROM.put(EEPROM_FCNT_ADDR, val);
  }
  Serial.print(F("[EEPROM] Read counter: "));
  Serial.println(val);
  return val;
}

void writeFCnt(uint32_t val) {
  EEPROM.put(EEPROM_FCNT_ADDR, val);
  Serial.print(F("[EEPROM] Wrote counter: "));
  Serial.println(val);
}

// -------------------------
// Debug Functions
// -------------------------
void printSystemStatus() {
  Serial.println(F("\n===== SYSTEM STATUS ====="));
  Serial.print(F("Uptime: "));
  Serial.print(millis() / 1000);
  Serial.println(F("s"));

  Serial.print(F("Packet Counter: "));
  Serial.println(currentCounter);

  Serial.print(F("TX Success: "));
  Serial.print(txSuccessCount);
  Serial.print(F("  TX Fail: "));
  Serial.println(txFailCount);

  Serial.print(F("Commands Received: "));
  Serial.println(commandCount);

  Serial.print(F("Sensor Mode: "));
  Serial.println(sensorMode == 0 ? "VL53L0X (ToF)" : "HC-SR04 (Ultrasonic)");

  Serial.print(F("Last RSSI: "));
  Serial.print(rf95.lastRssi());
  Serial.println(F(" dBm"));

  Serial.print(F("Last SNR: "));
  Serial.print(rf95.lastSNR());
  Serial.println(F(" dB"));

  Serial.println(F("=======================\n"));
}

void printPacketHex(uint8_t* data, size_t length) {
  Serial.print(F("Packet Hex ("));
  Serial.print(length);
  Serial.print(F(" bytes): "));
  for (size_t i = 0; i < length; i++) {
    if (data[i] < 0x10) Serial.print(F("0"));
    Serial.print(data[i], HEX);
    Serial.print(F(" "));
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
// Misc Functions
// -------------------------
//float getAverageWeight() {
//  float sum = 0.0;
//  int count = weightHistoryFilled ? weightHistorySize : weightHistoryIndex;
//  for (int i = 0; i < count; i++) sum += weightHistory[i];
//  return sum / count;
//}

void blinkLed(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(duration);
  }
}
