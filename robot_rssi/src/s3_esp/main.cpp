#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <BLEUtils.h>

#define BEACON_NAME  "ROBOT_BEACON"
#define BEACON_UUID  "12345678-1234-1234-1234-123456789abc"

BLEAdvertising *pAdvertising;
BLEUUID targetUUID(BEACON_UUID);

void setup() {
  // Serial.begin(115200);
  delay(3000);
  // Serial.println("Khoi dong BLE Beacon...");

  BLEDevice::init(BEACON_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_P3); // +3dBm, phu hop tracking trong nha

  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setCompleteServices(targetUUID);

  BLEAdvertisementData scanResp;
  scanResp.setName(BEACON_NAME);

  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setScanResponseData(scanResp);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinInterval(0x20);
  pAdvertising->setMaxInterval(0x40);

  BLEDevice::startAdvertising();
  // Serial.println("Beacon dang phat!");
}

void loop() {
  delay(5000);
  // Serial.println("[Beacon] Dang chay...");
}
