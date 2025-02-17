#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open
  
  Wire.begin();

  // Initialize the sensor
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L0X sensor!");
    while (1);
  }

  // Start continuous measurements
  sensor.startContinuous();

  Serial.println("VL53L0X Distance Sensor Initialized.");
}

void loop() {
  // Take multiple readings for better precision
  int numReadings = 5;
  uint32_t sum = 0;

  for (int i = 0; i < numReadings; i++) {
    uint16_t distance = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()) {
      Serial.println("Sensor timeout!");
      return;
    }
    sum += distance;
    delay(50); // Short delay between readings
  }

  // Calculate the average and convert to cm
  uint16_t avgDistance = sum / numReadings;
  float distance_cm = avgDistance / 10.0;

  // Print distance in cm
  Serial.print("Distance: ");
  Serial.print(distance_cm, 1); // Print with 1 decimal place
  Serial.println(" cm");

  // Slow down printing
  delay(2000); // Print every 2 seconds
}
