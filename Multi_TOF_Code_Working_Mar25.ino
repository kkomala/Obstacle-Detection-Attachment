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
#define TCAADDR 0x70
#define DEV_I2C Wire

// TOF Sensors
VL53L4CX sensor_vl53l4cx_sat(&Wire, A1);      
VL53L4CX sensor_vl53l4cx_sat_2(&Wire, A3);  
VL53L4CX sensor_vl53l4cx_sat_3(&Wire, A5);

const int motorPins[] = {5, 7, 9}; // Assuming pins 5, 7, and 9 are used for the motors

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  Serial.begin(9600); // Define baud rate
  while (!Serial);  // wait for serial monitor to open
  
  DEV_I2C.begin();  // Initialize I2C bus

  Serial.println("TOF Connection Test");

  // Initialize TOF #1 at pin A1 
  tcaselect(0);
  Serial.println("Before TOF 1 connection"); 
  sensor_vl53l4cx_sat.begin();  
  sensor_vl53l4cx_sat.VL53L4CX_Off(); // switch off satellite component
  sensor_vl53l4cx_sat.InitSensor(0x12); // initialize satellite component 
    Serial.println("TOF #1 starting measurement");
  
  // Initialize TOF #2 at pin A3 
  tcaselect(2);
  sensor_vl53l4cx_sat_2.begin();
  sensor_vl53l4cx_sat_2.VL53L4CX_Off();
  sensor_vl53l4cx_sat_2.InitSensor(0x12);
  sensor_vl53l4cx_sat_2.VL53L4CX_StartMeasurement();
    Serial.println("TOF #2 starting measurement");

  // Initialize TOF #1 at pin A5
  tcaselect(4);
  sensor_vl53l4cx_sat_3.begin();    
  sensor_vl53l4cx_sat_3.VL53L4CX_Off();
  sensor_vl53l4cx_sat_3.InitSensor(0x12);
  sensor_vl53l4cx_sat_3.VL53L4CX_StartMeasurement();
    Serial.println("TOF #3 starting measurement");
}

void loop() {

  // Monitor TOF sensor 1
  monitorTOFSensor(sensor_vl53l4cx_sat, motorPins[0]);
  Serial.println("TOF 1"); 
  delay(10000);

  // Monitor TOF sensor 2
  monitorTOFSensor(sensor_vl53l4cx_sat_2, motorPins[1]); 
  Serial.println("TOF 2");
  delay(10000);

  // Monitor TOF sensor 3
  monitorTOFSensor(sensor_vl53l4cx_sat_3, motorPins[2]); 
  Serial.println("TOF 3");
  delay(10000); 
}

void monitorTOFSensor(VL53L4CX& sensor, int motorPins) {
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0;
  char report[64];
  int status;

  do {
    status = sensor.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
    SerialPort.print(report);
    for (int j = 0; j < no_of_object_found; j++) {
      if (j != 0) {
        SerialPort.print("\r\n                               ");
      }

      SerialPort.print(", D=");
      SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
      SerialPort.print("mm");
    }
    SerialPort.println("");
    if (status == 0) {
      status = sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
    }

    if (no_of_object_found > 0) {
      for (int j = 0; j < no_of_object_found; j++) {
        int distance = pMultiRangingData->RangeData[j].RangeMilliMeter;

        // Adjust motor intensity based on distance
        if (distance <= 100) {
          analogWrite(motorPins, 255); // Full intensity
        } else if (distance <= 200 && distance > 100) {
          analogWrite(motorPins, 150); // Medium intensity
        } else if (distance <= 500 && distance > 200) {
          analogWrite(motorPins, 75); // Low intensity
        } else {
          analogWrite(motorPins, 0); // Turn off motor
        }
      }
    } else {
      analogWrite(motorPins, 0); // Turn off motor if no objects detected
    }
  }
}
