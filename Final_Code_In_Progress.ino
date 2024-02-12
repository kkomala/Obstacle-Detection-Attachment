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
#include <stdlib.h>

// accel+gyro 
#define LSM_CS 10
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

// TOF + vibrational motor 
#define DEV_I2C Wire
#define SerialPort Serial
#define NUM_MOTORS 3

const int motorPins[NUM_MOTORS] = {5, 7, 9}; // Assuming pins 5, 7, and 9 are used for the motors


Adafruit_LSM6DS3TRC lsm6ds3trc;

VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A0);    // TOF sensor 1 
VL53L4CX sensor_vl53l4cx_sat_2(&DEV_I2C, A3);  // TOF sensor 2  
VL53L4CX sensor_vl53l4cx_sat_3(&DEV_I2C, A5);  // TOF sensor 3

#define XSHUT_PIN_1 A1  // Xshut pin for TOF sensor 1
#define XSHUT_PIN_2 A3  // Xshut pin for TOF sensor 2
#define XSHUT_PIN_3 A5  // Xshut pin for TOF sensor 3

// Fall Detection service UUID
const uint8_t UUID16_SVC_FALLDETECTION[] = {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x23, 0x16, 0x00, 0x00};

// Fall Detection characteristic UUID
const uint8_t UUID16_CHR_FALLDETECTION[] = {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x24, 0x16, 0x00, 0x00};

// Battery service UUID 
const uint8_t CUSTOM_SVC_BATTERY[] = {0x8F, 0xAB, 0x3C, 0xE4, 0x7D, 0x6A, 0x2F, 0x81, 0xC2, 0x5E, 0xF9, 0x0A, 0x1B, 0xD8, 0x4E, 0x00};

// Battery characteristic UUID 
const uint8_t UUID16_CHR_BATTERY[] = {0x9A, 0x2C, 0x5F, 0x81, 0x3B, 0x6E, 0x4D, 0xAF, 0x8C, 0xE7, 0x1A, 0x5B, 0xF2, 0xD9, 0x7E, 0x3F}; 

BLEService falldetectionService(UUID16_SVC_FALLDETECTION);
BLECharacteristic falldetectionCharacteristic(UUID16_CHR_FALLDETECTION, BLERead | BLENotify, sizeof(uint16_t));
BLEService batteryService(CUSTOM_SVC_BATTERY);
BLECharacteristic batteryCharacteristic(UUID16_CHR_BATTERY, BLERead | BLENotify, 4);

char message[] = "Fall Detected"; // for fall detection notifications 

BLEDis bledis; // DIS (Device Information Service) helper class instance
BLEBas blebas; // BAS (Battery Service) helper class instance

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

// battery 
#define batteryPin A6 // D10

void setup(void) {

// Define new I2C addresses for the sensors
  uint8_t WantedAddress1 = 0x53;  // TOF sensor 1
  uint8_t WantedAddress2 = 0x54;  // TOF sensor 2
  uint8_t WantedAddress3 = 0x55;  // TOF sensor 3

  // Set Xshut LOW
  digitalWrite(XSHUT_PIN_1, LOW);
  digitalWrite(XSHUT_PIN_2, LOW);
  digitalWrite(XSHUT_PIN_3, LOW);
  delay(10); 

  // Raise the Xshut pin of sensor 1
  digitalWrite(XSHUT_PIN_1, HIGH);
  delay(10);

  // Call VL53L4CX_SetDeviceAddress to set the new I2C address for sensor 1
  sensor_vl53l4cx_sat.VL53L4CX_SetDeviceAddress(WantedAddress1);
  delay(10);

  // Power down sensor 1 by setting Xshut LOW
  digitalWrite(XSHUT_PIN_1, LOW);
  delay(10);

  // Raise the Xshut pin of sensor 2
  digitalWrite(XSHUT_PIN_2, HIGH);
  delay(10);

  // Call VL53LX_SetDeviceAddress to set the new I2C address for sensor 2
  sensor_vl53l4cx_sat_2.VL53L4CX_SetDeviceAddress(WantedAddress2);
  delay(10);

  // Power down sensor 2 by setting Xshut LOW
  digitalWrite(XSHUT_PIN_2, LOW);
  delay(10);

  // Raise the Xshut pin of sensor 3
  digitalWrite(XSHUT_PIN_3, HIGH);
  delay(10);

  // Call VL53LX_SetDeviceAddress to set the new I2C address for sensor 2
  sensor_vl53l4cx_sat_3.VL53L4CX_SetDeviceAddress(WantedAddress2);
  delay(10);

  // Power up sensors again
  digitalWrite(XSHUT_PIN_1, HIGH);
  digitalWrite(XSHUT_PIN_2, HIGH);
  delay(10);

  // Bluetooth
  Bluefruit.begin();
  falldetectionService.begin();
  batteryService.begin();
  
  // Configure fall detection characteristic
  falldetectionCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  falldetectionCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  falldetectionCharacteristic.setFixedLen(13); // 13 bytes of data sent ("message")
  falldetectionCharacteristic.begin();

  // Configure battery characteristic 
  batteryCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  batteryCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  batteryCharacteristic.setFixedLen(4); // 4 bytes of data sent --> double check 
  batteryCharacteristic.begin();

  // Start Bluetooth advertising
  Bluefruit.Advertising.addService(falldetectionService);
  Bluefruit.Advertising.addService(batteryService);
  Bluefruit.Advertising.start(0); // 0 means it will advertise forever
  startAdv();

  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  DEV_I2C.begin();
  // TOF 1
  sensor_vl53l4cx_sat.begin();
  sensor_vl53l4cx_sat.VL53L4CX_Off();
  sensor_vl53l4cx_sat.InitSensor(0x12);
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();

  // TOF 2
  sensor_vl53l4cx_sat_2.begin();
  sensor_vl53l4cx_sat_2.VL53L4CX_Off();
  sensor_vl53l4cx_sat_2.InitSensor(0x12); // Use the appropriate address for sensor 2
  sensor_vl53l4cx_sat_2.VL53L4CX_StartMeasurement();

  // TOF 3  
  sensor_vl53l4cx_sat_3.begin();
  sensor_vl53l4cx_sat_3.VL53L4CX_Off();
  sensor_vl53l4cx_sat_3.InitSensor(0x12); // Use the appropriate address for sensor 3
  sensor_vl53l4cx_sat_3.VL53L4CX_StartMeasurement();

  Serial.println("Adafruit LSM6DS3TR-C test!");

  if (!lsm6ds3trc.begin_I2C()) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS3TR-C Found!");


  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds3trc.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  Serial.print("Gyro range set to: ");
  switch (lsm6ds3trc.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; 
  }

  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds3trc.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds3trc.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  lsm6ds3trc.configInt1(false, false, true); // accelerometer 
  lsm6ds3trc.configInt2(false, true, false); // gyro 
}

void startAdv() {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(falldetectionService);
  Bluefruit.Advertising.addService(batteryService);

  // Secondary Scan Response packet (optional)
  Bluefruit.ScanResponse.addName();

  // Start Advertising
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void loop() {

// TOF sensors -- constantly monitor for objects 
  monitorTOFSensor(sensor_vl53l4cx_sat, motorPins[0]); // TOF 1 
  monitorTOFSensor(sensor_vl53l4cx_sat_2, motorPins[1]); // TOF 2 
  monitorTOFSensor(sensor_vl53l4cx_sat_3, motorPins[2]); // TOF 3

// fall detection
  sensors_event_t accel;
  sensors_event_t gyro;
  lsm6ds3trc.getEvent(&accel, &gyro, NULL);

// find magnitude of acceleration sensor
float accelMagnitude = sqrt(accel.acceleration.x * accel.acceleration.x +
                           accel.acceleration.y * accel.acceleration.y +
                           accel.acceleration.z * accel.acceleration.z);

// find magnitude of gyroscope sensor 
float gyroMagnitude = sqrt(gyro.gyro.x * gyro.gyro.x +
                          gyro.gyro.y * gyro.gyro.y +
                          gyro.gyro.z * gyro.gyro.z);

float accelThreshold = 29.4; // threshold is 3g 
bool accelerationMagnitude = (accelMagnitude >= accelThreshold);

// Step 1: Instability in walking
  if (accelerationMagnitude) {
    Serial.println("Instability detected!");
    // Proceed to the next step
  }

  // Step 2: Measuring Posture with Angle
  float angle = (acos(accel.acceleration.x / accelMagnitude) * 180.0)/PI;
  float angleThreshold =  45; // gyro threshold in degrees

  bool angleChangeDetected = (angle >= angleThreshold);

  if (angleChangeDetected) {
    Serial.println("Posture change detected!");
    // Proceed to the next step
  }
switch (currentState) {
    case SAMPLING:
      // Check if acceleration magnitude exceeds 3g and gyroscope magnitude exceeds 45 degrees
      if (accelerationMagnitude >= 29.4 && angleChangeDetected >= 45.0) {
        peakDetectionTime = millis();
        currentState = POST_PEAK;
      }
      break;

    case POST_PEAK:
      // Check for bouncing timer (1000 ms)
      if (millis() - peakDetectionTime > 1000) {
        postPeakStartTime = millis();
        currentState = POST_FALL;
      } else if (accelerationMagnitude >= 29.4 && angleChangeDetected >= 45.0) {
        // Return to Sampling state if another threshold peak is met within the bouncing interval
        currentState = SAMPLING;
      }
      break;

    case POST_FALL:
      // Check for new threshold peak during post-fall interval
      if (accelerationMagnitude <= 9.8 && angleChangeDetected < 45.0) {
        // Actual fall detected
        Serial.println("Fall detected!");
        falldetectionCharacteristic.notify(message, strlen(message));
        Serial.println("Sent fall detection data"); 
      }
       else if (millis() - postFallStartTime > 1500) {
        currentState = ACTIVITY_TEST;
      }
      break;

    case ACTIVITY_TEST:
      // Check if acceleration magnitude exceeds 0.5g
      if (accelerationMagnitude >= 4.9) {
        // False alarm, go back to Sampling state
        currentState = SAMPLING;
      } else {
        // Actual fall detected
        Serial.println("Fall detected!");
        falldetectionCharacteristic.notify(message, strlen(message));
        Serial.println("Sent fall detection data"); 

        // Return to Sampling state after fall detection
        currentState = SAMPLING;
      }
      break;
  }
  // Read the analog voltage
  int rawValue = analogRead(batteryPin);

  // Convert the raw value to voltage (assuming 3.3V reference voltage)
  float voltage = rawValue * (3.3 / 1023.0);

  // Convert voltage to string
  String voltageString = String(voltage, 2); // 2 decimal places

  // Print the battery voltage
  Serial.print("Battery Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  // send battery voltage 
  batteryCharacteristic.notify((uint8_t*)voltageString.c_str(), voltageString.length());

  // Delay before the next reading
  delay(1000);
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
