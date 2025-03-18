#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
int sensorMode = 0;       // 0 = VL53L0X, 1 = HC-SR04
int lastSensorMode = -1;  // to track mode changes

#define echoPin 11  // For HC-SR04
#define trigPin 10  // For HC-SR04

// Function to (re)initialize the chosen sensor
void initSensorMode() {
  if (sensorMode == 0) {
    // Initialize VL53L0X sensor
    Wire.begin();
    if (!sensor.init()) {
      Serial.println("Failed to detect and initialize VL53L0X sensor!");
      while (1);
    }
    sensor.startContinuous();
    Serial.println("VL53L0X Distance Sensor Initialized.");
  } else {
    // Initialize HC-SR04 sensor
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.println("Distance measurement using HC-SR04.");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Enter 0 for VL53L0X sensor, 1 for HC-SR04 sensor.");
  
  // Initialize based on the default mode
  initSensorMode();
  lastSensorMode = sensorMode;
}

void loop() {
  // Check if a new mode command is received from the Serial Monitor
  if (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == '0') {
      sensorMode = 0;
    } else if (ch == '1') {
      sensorMode = 1;
    }
    // If sensor mode changed, reinitialize the sensor setup
    if (sensorMode != lastSensorMode) {
      Serial.println("Changing sensor mode...");
      initSensorMode();
      lastSensorMode = sensorMode;
    }
  }

  if (sensorMode == 0) {
    // VL53L0X: Average 10 measurements
    int numReadings = 10;
    uint32_t sum = 0;
    for (int i = 0; i < numReadings; i++) {
      uint16_t distance = sensor.readRangeContinuousMillimeters();
      if (sensor.timeoutOccurred()) {
        Serial.println("Sensor timeout!");
        return;
      }
      sum += distance;
      delay(50);
    }
    uint16_t avgDistance = sum / numReadings;
    float distance_cm = avgDistance / 10.0;
    Serial.print("Distance (VL53L0X): ");
    Serial.print(distance_cm, 1);
    Serial.println(" cm");
    delay(2000);
  } else {
    // HC-SR04: Average 50 measurements
    int numReadings = 50;
    float sum = 0.0;
    for (int i = 0; i < numReadings; i++) {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      long duration = pulseIn(echoPin, HIGH);
      float distance = (duration * 0.0344) / 2.0;  // distance in cm
      sum += distance;
      delay(50);  // Short delay between readings
    }
    float avgDistance = sum / numReadings;
    avgDistance += 1.5;  // Adjust for the sensor height
    Serial.print("Distance (HC-SR04): ");
    Serial.print(avgDistance, 2);  // Print with 2 decimals
    Serial.println(" cm");
    delay(2000);
  }
}
