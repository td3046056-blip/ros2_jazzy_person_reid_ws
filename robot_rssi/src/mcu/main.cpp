#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_coexist.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <math.h>

#define TARGET_UUID  "12345678-1234-1234-1234-123456789abc"
#define SCAN_TIME    1
#define WIFI_CHANNEL 1

// Goc lap dat thuc te (do, tinh tu huong truoc robot)
#define ANGLE_A      0.0    // NodeMCU o phia truoc
#define ANGLE_B    120.0    // Mtiny #1 o 120 do
#define ANGLE_C    240.0    // Mtiny #2 o 240 do

// Bu offset giua cac board khac loai (dien sau khi hieu chuan thuc te)
#define OFFSET_B     -5.0
#define OFFSET_C     -5.0

// Nguong do tin cay toi thieu de tin vao huong tinh duoc
#define CONFIDENCE_THRESHOLD 0.15

// Nguong goc (do) de quyet dinh quay trai/phai/di thang
#define ANGLE_DEADBAND 30.0

// ============================================================
//  Bo loc Median + EMA
//  - Median(5): loai spike dot ngot trong RSSI
//  - EMA(alpha=0.3): lam muot xu huong dai han
// ============================================================
#define MED_WIN   3      // giam xuong 3 de bot do tre buffer
#define EMA_ALPHA 0.7f   // tang len 0.7 de bam theo du lieu moi nhat nhanh nhat

struct MedianEMA {
  float buf[MED_WIN];    // circular buffer luu cac mau RSSI gan nhat
  int   idx   = 0;       // vi tri ghi tiep theo
  int   count = 0;       // so mau da co (toi da MED_WIN)
  float ema   = -100.f;  // gia tri EMA hien tai
  bool  ready = false;   // da co it nhat 1 mau chua
};

// Tinh median cua n phan tu dau tien trong mang a
float medianOfN(float* a, int n) {
  float tmp[MED_WIN];
  memcpy(tmp, a, n * sizeof(float));
  // Insertion sort nhe (n <= 5, chi phi khong dang ke)
  for (int i = 1; i < n; i++)
    for (int j = i; j > 0 && tmp[j] < tmp[j - 1]; j--) {
      float t = tmp[j]; tmp[j] = tmp[j - 1]; tmp[j - 1] = t;
    }
  return tmp[n / 2];
}

// Cap nhat filter voi gia tri RSSI moi
void updateFilter(MedianEMA& f, float newVal) {
  f.buf[f.idx] = newVal;
  f.idx = (f.idx + 1) % MED_WIN;
  if (f.count < MED_WIN) f.count++;

  float med = medianOfN(f.buf, f.count);  // buoc 1: loai spike
  if (!f.ready) {                          // buoc 2: EMA
    f.ema   = med;
    f.ready = true;
  } else {
    f.ema = EMA_ALPHA * med + (1.f - EMA_ALPHA) * f.ema;
  }
}

// ============================================================

typedef struct {
  char  id;     // 'B' hoac 'C'
  int   rssi;
  bool  valid;
} RSSIPacket;

BLEScan* pBLEScan;

MedianEMA filterA, filterB, filterC;         // bo loc cho 3 cam bien
bool  validA = false, validB = false, validC = false;
unsigned long lastA = 0, lastB = 0, lastC = 0;

// Nhan RSSI tu Slave qua ESP-NOW
void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  RSSIPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.id == 'B' && pkt.valid) {
    updateFilter(filterB, pkt.rssi + OFFSET_B);
    validB = true; lastB = millis();
  }
  if (pkt.id == 'C' && pkt.valid) {
    updateFilter(filterC, pkt.rssi + OFFSET_C);
    validC = true; lastC = millis();
  }
}

BLEUUID targetUUID(TARGET_UUID);

// Callback BLE scan cua chinh Master (cam bien A)
class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (!dev.haveServiceUUID()) return;
    if (!dev.isAdvertisingService(targetUUID)) return;
    updateFilter(filterA, dev.getRSSI());
    validA = true;
    lastA = millis();
  }
};

// Tinh huong bang vector-sum co trong so tu 3 RSSI da loc
void calcDirection() {
  // Slave hoac Master khong co du lieu qua 3s -> coi la mat tin hieu
  if (millis() - lastA > 3000) validA = false;
  if (millis() - lastB > 3000) validB = false;
  if (millis() - lastC > 3000) validC = false;

  if (!validA && !validB && !validC) {
    Serial.println("[DIR] Khong co tin hieu!");
    return;
  }

  // Chuyen RSSI (dBm) -> trong so tuyen tinh
  float wA = validA ? pow(10.0, filterA.ema / 10.0) : 0;
  float wB = validB ? pow(10.0, filterB.ema / 10.0) : 0;
  float wC = validC ? pow(10.0, filterC.ema / 10.0) : 0;

  float aA = ANGLE_A * M_PI / 180.0;
  float aB = ANGLE_B * M_PI / 180.0;
  float aC = ANGLE_C * M_PI / 180.0;

  // Vector sum
  float Vx = wA * cos(aA) + wB * cos(aB) + wC * cos(aC);
  float Vy = wA * sin(aA) + wB * sin(aB) + wC * sin(aC);

  float direction = atan2(Vy, Vx) * 180.0 / M_PI;
  
  float sumW = wA + wB + wC;
  if (sumW == 0) sumW = 1.0; // Chong chia 0
  float confidence = sqrt(Vx * Vx + Vy * Vy) / sumW;

  // Chi in ra man hinh 2 lan moi giay (moi 500ms) de de doc
  // nhung van tinh toan ngam 20 lan/giay
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.println("==========================================");
    Serial.printf(" A(0)  : %.1f dBm [%s]  (n=%d)\n", filterA.ema, validA ? "OK" : "X", filterA.count);
    Serial.printf(" B(120): %.1f dBm [%s]  (n=%d)\n", filterB.ema, validB ? "OK" : "X", filterB.count);
    Serial.printf(" C(240): %.1f dBm [%s]  (n=%d)\n", filterC.ema, validC ? "OK" : "X", filterC.count);
    Serial.printf(" Huong : %.1f deg | Tin cay: %.2f %s\n",
                  direction, confidence,
                  confidence > CONFIDENCE_THRESHOLD ? "OK" : "THAP");

    // Xac dinh lenh dieu khien
    const char* cmd_str = "GIU_HUONG";
    if (confidence > CONFIDENCE_THRESHOLD) {
      if      (direction >  ANGLE_DEADBAND) { cmd_str = "QUAY_PHAI"; Serial.println(" => QUAY PHAI"); }
      else if (direction < -ANGLE_DEADBAND) { cmd_str = "QUAY_TRAI"; Serial.println(" => QUAY TRAI"); }
      else                                  { cmd_str = "DI_THANG";  Serial.println(" => DI THANG"); }
    } else {
      Serial.println(" => GIU HUONG CU (tin cay thap)");
    }
    Serial.println("==========================================\n");

    // ---- JSON output cho ROS2 (phan tich boi rssi_serial_node) ----
    // Format co dinh: JSON: {...}\n
    // ROS2 chi doc dong bat dau bang "JSON:"
    Serial.printf("JSON:{\"angle\":%.1f,\"conf\":%.2f,\"cmd\":\"%s\","
                  "\"rssi_a\":%.1f,\"rssi_b\":%.1f,\"rssi_c\":%.1f,"
                  "\"valid_a\":%s,\"valid_b\":%s,\"valid_c\":%s}\n",
                  direction, confidence, cmd_str,
                  filterA.ema, filterB.ema, filterC.ema,
                  validA ? "true" : "false",
                  validB ? "true" : "false",
                  validC ? "true" : "false");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("NodeMCU MASTER — bo loc Median+EMA");

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("MAC cua toi: ");
  Serial.println(WiFi.macAddress());
  Serial.println("(Dien MAC nay vao ca 2 file mtiny1 va mtiny2)\n");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAIL!");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanCB(), true);
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(40);

  // Giup BLE + WiFi/ESP-NOW chia se radio on dinh hon
  esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);

  // Bat dau quet ngam lien tuc (khong block loop)
  pBLEScan->start(0, nullptr, false);

  Serial.println("San sang!\n");
}

void loop() {
  calcDirection();
  delay(50);
}
