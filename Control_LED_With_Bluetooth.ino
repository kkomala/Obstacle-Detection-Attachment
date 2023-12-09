#include <bluefruit.h>


// LED service UUID
const uint8_t UUID16_SVC_LED_BUILTIN[] =
{
  0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00
};


// LED characteristic UUID
const uint8_t UUID16_CHR_LED_BUILTIN[] =
{
 0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x24, 0x15, 0x00, 0x00
};

int ledPin = 3; 
BLEService        ledService(UUID16_SVC_LED_BUILTIN);
BLECharacteristic ledCharacteristic(UUID16_CHR_LED_BUILTIN);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

void setup() {
  pinMode(ledPin, OUTPUT); // set up the LED
  Bluefruit.begin(); // initialize bluetooth
  ledService.begin(); //initialize LED service

  // configure LED characteristic
  ledCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  ledCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  ledCharacteristic.setFixedLen(1);
  ledCharacteristic.begin();
  ledCharacteristic.write8(0x01); // LED = on when connected

    ledCharacteristic.setWriteCallback(led_write_callback);

  // Start advertising
  Bluefruit.Advertising.addService(ledService);
  Bluefruit.Advertising.start(0); // 0 means it will advertise forever

  startAdv(); //start advertising
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include LED Service UUID
  Bluefruit.Advertising.addService(ledService);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
 
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html  
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void led_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl;
  (void) chr;
  (void) len; // len should be 1
  // data = 1 -> LED = On
  // data = 0 -> LED = Off
  if (data[0] == 1) {
    digitalWrite(ledPin, HIGH); // Turn LED on
  } else if (data[0] == 0) {
    digitalWrite(ledPin, LOW); // Turn LED off
  }
}

void loop() {
  // Check if a central device is connected
  if (Bluefruit.connected()) {
    uint8_t command = ledCharacteristic.read8();
    if (command == '1') {
      digitalWrite(ledPin, HIGH); // Turn LED on
    }
    else if (command == '0') {
      digitalWrite(ledPin, LOW); // Turn LED off
    }
  }
}

