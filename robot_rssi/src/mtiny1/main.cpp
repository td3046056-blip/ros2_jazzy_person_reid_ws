#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_coexist.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ============================================================
// DIEN MAC cua NodeMCU vao day (lay tu Serial Monitor cua Master)
uint8_t masterMAC[] = {0x94, 0xE6, 0x86, 0x0D, 0xEC, 0x50};
// ============================================================

#define TARGET_UUID  "12345678-1234-1234-1234-123456789abc"
#define SENSOR_ID    'B'   // Cam bien B, dat o 120 do
#define SCAN_TIME    1
#define WIFI_CHANNEL 1

// ============================================================
//  Bo loc Median + EMA
//  - Median(5): loai spike dot ngot trong RSSI
//  - EMA(alpha=0.3): lam muot xu huong dai han
// ============================================================
#define MED_WIN   3
#define EMA_ALPHA 0.7f

struct MedianEMA {
  float buf[MED_WIN];
  int   idx   = 0;
  int   count = 0;
  float ema   = -100.f;
  bool  ready = false;
};

float medianOfN(float* a, int n) {
  float tmp[MED_WIN];
  memcpy(tmp, a, n * sizeof(float));
  for (int i = 1; i < n; i++)
    for (int j = i; j > 0 && tmp[j] < tmp[j - 1]; j--) {
      float t = tmp[j]; tmp[j] = tmp[j - 1]; tmp[j - 1] = t;
    }
  return tmp[n / 2];
}

void updateFilter(MedianEMA& f, float newVal) {
  f.buf[f.idx] = newVal;
  f.idx = (f.idx + 1) % MED_WIN;
  if (f.count < MED_WIN) f.count++;
  float med = medianOfN(f.buf, f.count);
  if (!f.ready) { f.ema = med; f.ready = true; }
  else          { f.ema = EMA_ALPHA * med + (1.f - EMA_ALPHA) * f.ema; }
}

// ============================================================

typedef struct {
  char  id;
  int   rssi;
  bool  valid;
} RSSIPacket;

BLEScan*  pBLEScan;
int       rssiRaw   = -100;
bool      gotSignal = false;
MedianEMA filter;

void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS)
    Serial.println("[ESP-NOW] Gui that bai!");
}

BLEUUID targetUUID(TARGET_UUID);
unsigned long lastSignal = 0;

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (!dev.haveServiceUUID()) return;
    if (!dev.isAdvertisingService(targetUUID)) return;
    rssiRaw   = dev.getRSSI();
    gotSignal = true;
    lastSignal = millis();
  }
};

void sendRSSI() {
  RSSIPacket pkt;
  pkt.id    = SENSOR_ID;
  pkt.rssi  = (int)filter.ema;
  pkt.valid = gotSignal;
  esp_now_send(masterMAC, (uint8_t*)&pkt, sizeof(pkt));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Mtiny SLAVE B (120 deg) — bo loc Median+EMA");

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAIL!");
    return;
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, masterMAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanCB(), true);
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(40);

  esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
  // Bat dau quet ngam lien tuc
  pBLEScan->start(0, nullptr, false);

  Serial.println("San sang!\n");
}

void loop() {
  if (millis() - lastSignal > 3000) gotSignal = false;

  if (gotSignal) {
    updateFilter(filter, rssiRaw);
    Serial.printf("[B] Raw:%d  Median+EMA:%.1f  (n=%d)\n",
                  rssiRaw, filter.ema, filter.count);
  } else {
    Serial.println("[B] Khong thay beacon");
  }

  sendRSSI();
  delay(50);
}
