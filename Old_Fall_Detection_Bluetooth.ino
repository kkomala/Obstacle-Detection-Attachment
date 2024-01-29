#include <bluefruit.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define LSM_CS 10
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

#define DEV_I2C Wire
#define SerialPort Serial
#define motor 5

Adafruit_LSM6DS3TRC lsm6ds3trc;

// Fall Detection service UUID
const uint8_t UUID16_SVC_FALLDETECTION[] = {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x23, 0x16, 0x00, 0x00};

// Fall Detection characteristic UUID
const uint8_t UUID16_CHR_FALLDETECTION[] = {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x24, 0x16, 0x00, 0x00};

BLEService falldetectionService(UUID16_SVC_FALLDETECTION);
BLECharacteristic falldetectionCharacteristic(UUID16_CHR_FALLDETECTION, BLERead | BLENotify, sizeof(uint16_t));

void setup() {
  Bluefruit.begin();
  falldetectionService.begin();

  // configure fall detection characteristic
  falldetectionCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  falldetectionCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  falldetectionCharacteristic.setFixedLen(2);
  falldetectionCharacteristic.begin();

  falldetectionCharacteristic.setWriteCallback(fall_detection_write_callback);

  // Start Bluetooth advertising
  Bluefruit.Advertising.addService(falldetectionService);
  Bluefruit.Advertising.start(0); // 0 means it will advertise forever
  startAdv();

Serial.begin(115200);
  while (!Serial)
    delay(10);

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

// Set accelerometer range
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

// Set gyroscope range 
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

// Accelerometer data range 
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

// Gyroscope data range 
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

  // Secondary Scan Response packet (optional)
  Bluefruit.ScanResponse.addName();

  // Start Advertising
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  lsm6ds3trc.getEvent(&accel, &gyro, NULL);

// Accelerometer reading 
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

// Gyroscope reading 
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  float fallAccelThreshold = 9.8;
  float fallGyroThreshold = 20.0; 

  if (accel.acceleration.x > fallAccelThreshold ||
      accel.acceleration.y > fallAccelThreshold ||
      accel.acceleration.z > fallAccelThreshold &&
      (abs(gyro.gyro.x) > fallGyroThreshold ||
       abs(gyro.gyro.y) > fallGyroThreshold ||
       abs(gyro.gyro.z) > fallGyroThreshold)) {
    Serial.println("Fall detected!");
    delay(100);
  }
}

void fall_detection_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void) chr;
  (void) len; // len should be 1

  sensors_event_t accel;
  sensors_event_t gyro;
  lsm6ds3trc.getEvent(&accel, &gyro, NULL);

  float fallAccelThreshold = 9.8;
  float fallGyroThreshold = 20.0;

  if (accel.acceleration.x > fallAccelThreshold ||
      accel.acceleration.y > fallAccelThreshold ||
      accel.acceleration.z > fallAccelThreshold &&
      (abs(gyro.gyro.x) > fallGyroThreshold ||
       abs(gyro.gyro.y) > fallGyroThreshold ||
       abs(gyro.gyro.z) > fallGyroThreshold)) {
    Serial.println("Fall detected!");

    // Check if BLE connection is established
    if (Bluefruit.connected(conn_hdl)) {
      // Send notification to LightBlue app
      uint16_t value = 1; // You can modify this value based on your application
      falldetectionCharacteristic.write(&value, sizeof(value));
    }

    delay(100);
  }
}
