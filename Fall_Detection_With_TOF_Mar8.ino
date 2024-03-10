#include <bluefruit.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

// accel+gryo
#define LSM_CS 10
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

// TOF + vibrational motor 
#define DEV_I2C Wire
#define SerialPort Serial

const int motorPin = 9; // motor connected to pin 9 

Adafruit_LSM6DS3TRC lsm6ds3trc;

VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1); // TOF XSHUT connected to A1 

// Fall Detection service UUID
const uint8_t UUID16_SVC_FALLDETECTION[] = {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x23, 0x16, 0x00, 0x00};

// Fall Detection characteristic UUID
const uint8_t UUID16_CHR_FALLDETECTION[] = {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x24, 0x16, 0x00, 0x00};

BLEService falldetectionService(UUID16_SVC_FALLDETECTION);
BLECharacteristic falldetectionCharacteristic(UUID16_CHR_FALLDETECTION, BLERead | BLENotify, sizeof(uint16_t));

char message[] = "Fall Detected"; 

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

// Define states for fall detection
enum FallDetectionState {
  SAMPLING,
  POST_PEAK,
  POST_FALL,
  ACTIVITY_TEST
};

FallDetectionState currentState = SAMPLING;

unsigned long peakDetectionTime;
unsigned long postPeakStartTime;
unsigned long postFallStartTime;

void setup() {

  // Initialize I2C bus.
  DEV_I2C.begin();
  // Configure VL53L4CX satellite component.
  sensor_vl53l4cx_sat.begin();
  // Switch off VL53L4CX satellite component.
  sensor_vl53l4cx_sat.VL53L4CX_Off();
  // Initialize VL53L4CX satellite component.
  sensor_vl53l4cx_sat.InitSensor(0x12);
   // Start Measurements
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  Serial.println("TOF Starting measurement");

  // Start bluetooth 
  Bluefruit.begin();
  falldetectionService.begin();

  // configure fall detection characteristic
  falldetectionCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  falldetectionCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  falldetectionCharacteristic.setFixedLen(13);
  falldetectionCharacteristic.begin();

  // Start Bluetooth advertising
  Bluefruit.Advertising.addService(falldetectionService);
  Bluefruit.Advertising.start(0); // 0 means it will advertise forever
  startAdv();

Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause board until serial console opens

  Serial.println("Adafruit LSM6DS3TR-C test!");

// If LSM6DS3TR-C not found, print "failed to find"
  if (!lsm6ds3trc.begin_I2C()) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(10);
    }
  }

// If LSM6DS3TR-C is found, print "found!"
  Serial.println("LSM6DS3TR-C Found!");

 // Enable accelerometer with 104 Hz data rate, 4G
  lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

  // Enable gyro with 104 Hz data rate, 2000 dps
  lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);

  delay(10);

  lsm6ds3trc.configInt1(false, false, true); // accelerometer 
  lsm6ds3trc.configInt2(false, true, false); // gyro 
}

void startAdv() {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(falldetectionService);

  // Secondary Scan Response packet (optional)
  Bluefruit.ScanResponse.addName();

  // Start Advertising
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void loop() {
  
  // TOF sensor -- constantly monitor for objects 
  monitorTOFSensor(sensor_vl53l4cx_sat, motorPin);
  delay(500);

  // Fall detection
  falldetection(lsm6ds3trc); 
 
}

void monitorTOFSensor(VL53L4CX& sensor, int motorPin) {
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0;
  char report[64];
  int status;

Serial.println("Before status check");
  do {
    status = sensor.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);
Serial.println("After status check");

  if ((!status) && (NewDataReady != 0)) {
    status = sensor.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
    Serial.print(report);
    for (int j = 0; j < no_of_object_found; j++) {
      if (j != 0) {
        Serial.print("\r\n                               ");
      }

      Serial.print(", D=");
      Serial.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
      Serial.print("mm");
    }
    Serial.println("");
    if (status == 0) {
      status = sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
    }

    if (no_of_object_found > 0) {
      for (int j = 0; j < no_of_object_found; j++) {
        int distance = pMultiRangingData->RangeData[j].RangeMilliMeter;

        // Adjust motor intensity based on distance
        if (distance <= 100) {
          analogWrite(motorPin, 255); // Full intensity
        } else if (distance <= 200 && distance > 100) {
          analogWrite(motorPin, 150); // Medium intensity
        } else if (distance <= 500 && distance > 200) {
          analogWrite(motorPin, 75); // Low intensity
        } else {
          analogWrite(motorPin, 0); // Turn off motor
        }
      }
    } else {
      analogWrite(motorPin, 0); // Turn off motor if no objects detected
    }
  }
}

void falldetection(Adafruit_LSM6DS3TRC) {
  sensors_event_t accel;
  sensors_event_t gyro;
  lsm6ds3trc.getEvent(&accel, &gyro, NULL);
  delay(500);

   Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");


  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

float fallAccelThreshold = 9.8; // threshold is g 

float fallGyroThreshold =  45; // gyro threshold in degrees

if (abs(accel.acceleration.x) > fallAccelThreshold ||
       abs(accel.acceleration.y) > fallAccelThreshold ||
       abs(accel.acceleration.z) > fallAccelThreshold &&
       (abs(gyro.gyro.x) >= fallGyroThreshold ||
        abs(gyro.gyro.y) >= fallGyroThreshold ||
        abs(gyro.gyro.z) >= fallGyroThreshold)) {
         Serial.println("Sampling: Acceleration and angle change detected"); 
        }
}

// switch (currentState) {
//     case SAMPLING:
//       // Check if acceleration magnitude exceeds g and gyroscope magnitude exceeds 45 degrees
//       if (abs(accel.acceleration.x) > fallAccelThreshold ||
//       abs(accel.acceleration.y) > fallAccelThreshold ||
//       abs(accel.acceleration.z) > fallAccelThreshold &&
//       (abs(gyro.gyro.x) >= fallGyroThreshold ||
//        abs(gyro.gyro.y) >= fallGyroThreshold ||
//        abs(gyro.gyro.z) >= fallGyroThreshold)) {
//         Serial.println("Sampling: Acceleration and angle change detected"); 
//         peakDetectionTime = millis();
//         currentState = POST_PEAK;
//       }
//       break;

//     case POST_PEAK:
//       // Check for bouncing timer (1000 ms)
//       postPeakStartTime = millis();
//       if (millis() - peakDetectionTime > 1000) {
//         Serial.println("In post peak state");
//         //postPeakStartTime = millis();
//         currentState = POST_FALL;
//         Serial.println("Entering post fall state"); 
//       } else if (abs(accel.acceleration.x) > fallAccelThreshold ||
//       abs(accel.acceleration.y) > fallAccelThreshold ||
//       abs(accel.acceleration.z) > fallAccelThreshold &&
//       (abs(gyro.gyro.x) >= fallGyroThreshold ||
//        abs(gyro.gyro.y) >= fallGyroThreshold ||
//        abs(gyro.gyro.z) >= fallGyroThreshold)) {
//         // Return to Sampling state if another threshold peak is met within the bouncing interval
//         currentState = SAMPLING;
//       }
//       break;

//     case POST_FALL:
//       // Check for new threshold peak during post-fall interval
//       if (abs(accel.acceleration.x) <= fallAccelThreshold ||
//       abs(accel.acceleration.y) <= fallAccelThreshold ||
//       abs(accel.acceleration.z) <= fallAccelThreshold &&
//       (abs(gyro.gyro.x) <= fallGyroThreshold ||
//        abs(gyro.gyro.y) <= fallGyroThreshold ||
//        abs(gyro.gyro.z) <= fallGyroThreshold)) {
//         // Actual fall detected
//         Serial.println("Fall detected during post-fall state!");
//         falldetectionCharacteristic.notify(message, strlen(message));
//         Serial.println("Sent fall detection data"); 
//         currentState = SAMPLING;
//       }
//        else if (millis() - postFallStartTime > 1500) {
//         currentState = ACTIVITY_TEST;
//       }
//       break;

//     case ACTIVITY_TEST:
//       // Check if acceleration magnitude exceeds 0.5g
//       if (accel.acceleration.x >= 4.9 ||
//       accel.acceleration.y >= 4.9 ||
//       accel.acceleration.z >= 4.9) {
//         // False alarm, go back to Sampling state
//         currentState = SAMPLING;
//       } else {
//         // Actual fall detected
//         Serial.println("Fall detected during activity test!");
//         falldetectionCharacteristic.notify(message, strlen(message));
//         Serial.println("Sent fall detection data"); 

//         // Return to Sampling state after fall detection
//         currentState = SAMPLING;
//       }
//       break;
//   }
// }
