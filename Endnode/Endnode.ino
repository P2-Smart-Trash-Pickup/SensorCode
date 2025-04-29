#include <HX711_ADC.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <RH_RF95.h>

// -------------------------
// LoRa Radio Configuration (Dragino Shield Fixed Pins)
// -------------------------
#define RFM95_CS 10  // Fixed by Dragino shield design
#define RFM95_INT 2  // Fixed by Dragino shield design
#define RFM95_RST 9  // Fixed by Dragino shield design

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Device configuration
const uint8_t DEVICE_ID = 0x01;  // Unique per device
uint32_t sequenceNumber = 0;

// -------------------------
// Weight Sensor Definitions (Using D4/D5)
// -------------------------
const int HX711_dout = 4; 
const int HX711_sck = 5;
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// -------------------------
// Distance Sensor Definitions
// -------------------------
VL53L0X distanceSensor;  // Uses I2C (A4/A5)
#define echoPin 3  // HC-SR04 echo (moved from D10)
#define trigPin 6  // HC-SR04 trig (moved from D11)
int sensorMode = 0; // 0=VL53L0X, 1=HC-SR04

// -------------------------
// Data Buffers
// -------------------------
const int weightHistorySize = 100;
float weightHistory[weightHistorySize];
int weightHistoryIndex = 0;
bool weightHistoryFilled = false;

#define DISTANCE_HISTORY_SIZE 10
uint16_t distanceHistory[DISTANCE_HISTORY_SIZE];
int distanceHistoryIndex = 0;

// -------------------------
// Timing Configuration
// -------------------------
unsigned long lastTxTime = 0;
const unsigned long txInterval = 10000;  // Transmit every 10 seconds

void initLoRa() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  // Configure for EU868 band, SF9
  rf95.setFrequency(868.0);
  rf95.setSpreadingFactor(9);
  rf95.setSignalBandwidth(125000);
  rf95.setCodingRate4(5);
  rf95.setTxPower(20, false);
  
  Serial.println("LoRa radio initialized");
}

void initOtherSensor() {
  if (sensorMode == 0) {
    // VL53L0X sensor initialization
    Wire.begin();
    if (!distanceSensor.init()) {
      Serial.println("Failed to initialize VL53L0X!");
      while (1);
    }
    distanceSensor.startContinuous();
  } else {
    // HC-SR04 sensor initialization
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port
  Serial.println("Starting LoRa Sensor Node");

  // Initialize LoRa
  initLoRa();

  // Initialize weight sensor
  LoadCell.begin();
  unsigned long stabilizingTime = 2000;
  boolean tareFlag = true;
  LoadCell.start(stabilizingTime, tareFlag);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("HX711 Timeout. Check wiring.");
    while (1);
  }
  LoadCell.setCalFactor(-40.10); // Calibration value

  // Initialize distance sensor
  Serial.println("Enter 0 for VL53L0X or 1 for HC-SR04");
  initOtherSensor();
}

float getAverageWeight() {
  float sum = 0.0;
  int count = weightHistoryFilled ? weightHistorySize : weightHistoryIndex;
  for (int i = 0; i < count; i++) sum += weightHistory[i];
  return sum / count;
}

void updateDistanceReading() {
  if (sensorMode == 0) {
    uint16_t reading = distanceSensor.readRangeContinuousMillimeters();
    if (!distanceSensor.timeoutOccurred()) {
      distanceHistory[distanceHistoryIndex] = reading;
      distanceHistoryIndex = (distanceHistoryIndex + 1) % DISTANCE_HISTORY_SIZE;
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
  }
}

void preparePayload(uint8_t *payload) {
  // Payload structure (29 bytes):
  // [0]: Device ID
  // [1-4]: Sequence number
  // [5-6]: Current weight (uint16_t grams)
  // [7-8]: Previous weight (uint16_t grams)
  // [9-28]: 10 distance readings (uint16_t mm each)
  
  uint16_t weight1 = (uint16_t)constrain(getAverageWeight(), 0, 65535);
  uint16_t weight2 = (uint16_t)constrain(weightHistory[(weightHistoryIndex - 1 + weightHistorySize) % weightHistorySize], 0, 65535);
  
  payload[0] = DEVICE_ID;
  
  // Big-endian sequence number
  payload[1] = (sequenceNumber >> 24) & 0xFF;
  payload[2] = (sequenceNumber >> 16) & 0xFF;
  payload[3] = (sequenceNumber >> 8) & 0xFF;
  payload[4] = sequenceNumber & 0xFF;
  
  // Weights
  payload[5] = highByte(weight1);
  payload[6] = lowByte(weight1);
  payload[7] = highByte(weight2);
  payload[8] = lowByte(weight2);
  
  // Distances
  for (int i = 0; i < 10; i++) {
    int idx = (distanceHistoryIndex - 1 - i + DISTANCE_HISTORY_SIZE) % DISTANCE_HISTORY_SIZE;
    payload[9 + i*2] = highByte(distanceHistory[idx]);
    payload[10 + i*2] = lowByte(distanceHistory[idx]);
  }
  
  sequenceNumber++;
}

void loop() {
  // Weight sensor update
  if (LoadCell.update()) {
    float weight = LoadCell.getData();
    weightHistory[weightHistoryIndex] = weight;
    weightHistoryIndex = (weightHistoryIndex + 1) % weightHistorySize;
    if (weightHistoryIndex == 0) weightHistoryFilled = true;
  }

  // Distance sensor update
  updateDistanceReading();

  // LoRa transmission
  if (millis() - lastTxTime >= txInterval) {
    uint8_t payload[29];
    preparePayload(payload);
    
    Serial.println("Sending LoRa packet...");
    rf95.send(payload, sizeof(payload));
    rf95.waitPacketSent();
    
    lastTxTime = millis();
  }

  // Serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') LoadCell.tareNoDelay();
    else if (c == '0' || c == '1') {
      sensorMode = c - '0';
      initOtherSensor();
    }
  }
}