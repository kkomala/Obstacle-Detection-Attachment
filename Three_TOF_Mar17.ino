#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

// TOF Sensors
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1);
VL53L4CX sensor_vl53l4cx_sat_2(&DEV_I2C, A3); // adjust
VL53L4CX sensor_vl53l4cx_sat_3(&DEV_I2C, A5); // adjust

const int motorPins[] = {5, 7, 9}; // Assuming pins 5, 7, and 9 are used for the motors

void setup() {

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CX satellite component.
  sensor_vl53l4cx_sat.begin();
  sensor_vl53l4cx_sat_2.begin();
  sensor_vl53l4cx_sat_3.begin();

  // Switch off VL53L4CX satellite component.
  sensor_vl53l4cx_sat.VL53L4CX_Off();
  sensor_vl53l4cx_sat_2.VL53L4CX_Off();
  sensor_vl53l4cx_sat_3.VL53L4CX_Off();

  // Initialize VL53L4CX satellite component.
  sensor_vl53l4cx_sat.InitSensor(0x12);
  sensor_vl53l4cx_sat_2.InitSensor(0x12);
  sensor_vl53l4cx_sat_3.InitSensor(0x12);

   // Start Measurements
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  sensor_vl53l4cx_sat_2.VL53L4CX_StartMeasurement();
  sensor_vl53l4cx_sat_3.VL53L4CX_StartMeasurement();

  // Set the Wire timeout (1 sec)
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(1000 /* us */, true /* reset_on_timeout */);
  #endif
}

void loop() {
  // TOF sensors -- constantly monitor for objects 
  monitorTOFSensor(sensor_vl53l4cx_sat, motorPins[0]); // TOF 1 
  monitorTOFSensor(sensor_vl53l4cx_sat_2, motorPins[1]); // TOF 2 
  monitorTOFSensor(sensor_vl53l4cx_sat_3, motorPins[2]); // TOF 3
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
