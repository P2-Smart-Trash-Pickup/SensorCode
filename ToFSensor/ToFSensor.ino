#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
int sensorMode = 0;       // 0 = VL53L0X, 1 = HC-SR04
int lastSensorMode = -1;  // to track mode changes
int faultCounter = 0;     // counts consecutive unrealistic measurements

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

  float avgDistance = 0.0;
  
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
    uint16_t avgReading = sum / numReadings;
    avgDistance = avgReading / 10.0;  // converting mm to cm (dividing by 10)
    Serial.print("Distance (VL53L0X): ");
    Serial.print(avgDistance, 1);
    Serial.println(" cm");
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
      delay(50);
    }
    avgDistance = sum / numReadings;
    avgDistance += 1.5;  // Adjust for the sensor height
    Serial.print("Distance (HC-SR04): ");
    Serial.print(avgDistance, 2);
    Serial.println(" cm");
  }
  
  // Hardware error handling:
  // If the measured average is over 100.00 cm, consider it a fault.
  if (avgDistance > 100.0) {
    faultCounter++;
    Serial.print("Fault count: ");
    Serial.println(faultCounter);
    
    // On the 10th consecutive fault, pause for 5 minutes
    if (faultCounter == 10) {
      Serial.println("10 consecutive faults detected, pausing for 1 minute.");
      delay(300000);  // 5 minutes = 300000 ms
    }
    
    // After 20 consecutive faults, alert and wait for reset command "99"
    if (faultCounter >= 20) {
      Serial.println("Fault detected: Sensor may be faulty. Technician required.");
      Serial.println("Enter '99' to reset fault counter and resume measurements.");
      while (true) {
        if (Serial.available() > 0) {
          String input = Serial.readString();
          input.trim();
          if (input == "99") {
            Serial.println("Reset command received. Resuming measurements.");
            faultCounter = 0;
            break;
          }
        }
      }
    }
  } else {
    // If the measurement is within realistic limits, reset fault counter.
    faultCounter = 0;
  }
  
  delay(2000); // Wait 2 seconds before the next measurement cycle.
}
