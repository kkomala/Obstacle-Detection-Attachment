#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define SerialPort Serial
#define DEV_I2C Wire

// TOF Sensors
VL53L4CX sensor_vl53l4cx_sat(&Wire, A1);  

const int motorPin = 9; 

void setup() {

  Serial.begin(9600); // Define baud rate
  while (!Serial);  // wait for serial monitor to open
  
  DEV_I2C.begin();  // Initialize I2C bus

  Serial.println("TOF Connection Test");

  Serial.println("Before TOF 1 connection"); 
  sensor_vl53l4cx_sat.begin();  
  sensor_vl53l4cx_sat.VL53L4CX_Off(); // switch off satellite component
  sensor_vl53l4cx_sat.InitSensor(0x12); // initialize satellite component 
    Serial.println("TOF #1 starting measurement");
}

void loop() {
  detectStairs(sensor_vl53l4cx_sat); 

}

 float measureDistance(VL53L4CX& sensor) {
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int status;

  do {
    status = sensor.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    return pMultiRangingData->RangeData[0].RangeMilliMeter; // Return distance in mm
  } else {
    return 0; // Return 0 if measurement failed
  }
}

void detectStairs(VL53L4CX& sensor) {
    unsigned long startTime = millis();  // Get the current time
    unsigned long endTime = startTime + 5000; // Measure for 5 seconds
    float sumDistances = 0.0;  // Initialize sum of distances
    int numMeasurements = 0;   // Initialize number of measurements

    // Collect distance measurements during the first 5 seconds
    while (startTime < endTime) {
        // Measure distance at defined interval
      float distance = measureDistance(sensor_vl53l4cx_sat); 
        if (distance > 0) { // Check if distance measurement is valid
          sumDistances += distance;  // Add distance to sum
          numMeasurements++;  // Increment number of measurements
        }
      delay(1000);  // Wait for next measurement
    }

    if (numMeasurements > 0) {
    float averageDistance = sumDistances / numMeasurements;
    float range = 5.0; // Range of +/- 5 mm

    // Determine stair direction based on average distance and range
    float upperThreshold = averageDistance + range; // for ascending stairs
    float lowerThreshold = averageDistance - range; // for descending stairs 
  
    Serial.print("Average Distance: ");
    Serial.println(averageDistance);
    Serial.print("Stairs Range: ");
    Serial.print(lowerThreshold);
    Serial.print(" to ");
    Serial.println(upperThreshold);

    float currentDistance = measureDistance(sensor_vl53l4cx_sat);
    Serial.print("Current Distance: ");
    Serial.println(currentDistance);

    // Check if average distance falls within the range
    if (currentDistance > upperThreshold) {
      Serial.println("Stairs going up");
      analogWrite(motorPins, 150); // Medium intensity
    } else if (currentDistance < lowerThreshold) {
      Serial.println("Stairs going down");
      analogWrite(motorPins, 255); // Full intensity
    } else {
      Serial.println("No stairs detected");
    }
  } else {
    Serial.println("No valid measurements obtained");
  }
}
