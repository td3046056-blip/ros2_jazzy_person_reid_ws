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
  // Phai chi ro ESP_BLE_PWR_TYPE_ADV: kieu mac dinh (DEFAULT) KHONG ap cho quang ba, quang ba
  // van o +3 dBm. Do 30/09: o 1.5 m ca 3 board chi thay -86..-96 dBm, sat nguong nghe (~-97),
  // A mat nhieu goi (7.6 mau/s, ho toi 3.8 s). +9 dBm nang ca 3 board len 6 dB nhu nhau —
  // khong doi huong uoc luong (huong chi dua vao CHENH LECH giua cac board).
  BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);

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
