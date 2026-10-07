// ============================================================
//  QUET RSSI — firmware CHUNG cho ca 3 board, cam thang USB vao laptop (30/09)
//
//  Thay kieu cu (Master + 2 Slave qua ESP-NOW, src/mcu, src/mtiny1, src/mtiny2). Ly do:
//   - ESP-NOW va quet BLE dung chung mot radio -> chi quet ~40% thoi gian
//   - Slave dua lai CUNG mot mau vao bo loc moi 50 ms; A loc 1 lan, B/C loc 2 lan,
//     B va C quet khac tham so -> ba cam bien tre khac nhau, trai/phai phan ung khong deu
//
//  O day KHONG loc gi: moi lan thay beacon in MOT dong mau tho. Loc + uoc luong huong
//  lam tren may tinh (sua khong can nap lai firmware). Cung mot file cho ca 3 board,
//  chi khac SENSOR_IDX trong platformio.ini -> khong the lech tham so quet nhu truoc.
//
//  Serial 115200, moi dong mot ban tin:
//    I,<id>,<dia chi BLE>          khi khoi dong
//    R,<id>,<seq>,<ms>,<rssi>      mot mau. seq dem MOI lan thay beacon (nhay coc = mat mau),
//                                  ms = millis() cua board luc nhan
//    H,<id>,<ms>,<n>,<drop>        moi 1 s: so mau da in trong 1 s qua, tong so mau bi bo
//                                  vi hang doi day
// ============================================================
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#ifndef SENSOR_IDX
#error "Thieu -DSENSOR_IDX=0/1/2 trong platformio.ini (0=A dau xe, 1=B sau phai, 2=C sau trai)"
#endif

static const char SENSOR_ID = "ABC"[SENSOR_IDX];

#define TARGET_UUID "12345678-1234-1234-1234-123456789abc"

// Quet lien tuc: window = interval. Het moi interval radio doi sang kenh quang ba khac
// (37 -> 38 -> 39). Beacon phat ca 3 kenh; moi kenh bi phan xa khac nhau nen khi xe va
// nguoi cung dung yen, trung binh qua 3 kenh moi xoa duoc sai so "dong bang" cua phan xa.
// 50 ms: cua so 0.3 s tren may tinh da di du ca 3 kenh.
#define SCAN_INTERVAL_MS 50
#define SCAN_WINDOW_MS   50

struct Sample {
  uint32_t seq;
  uint32_t ms;
  int8_t   rssi;
};

static QueueHandle_t     sampleQ;
static volatile uint32_t nDrop  = 0;
static uint32_t          seqAll = 0;   // chi task BLE ghi
static BLEUUID           targetUUID(TARGET_UUID);

// Chay trong task BLE: chi day mau vao hang doi, KHONG in Serial o day (in cham se lam
// task BLE bo lo goi quang ba).
class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (!dev.haveServiceUUID()) return;
    if (!dev.isAdvertisingService(targetUUID)) return;
    Sample s = { seqAll++, millis(), (int8_t)dev.getRSSI() };
    if (xQueueSend(sampleQ, &s, 0) != pdTRUE) nDrop++;
  }
};

BLEScan* pBLEScan;
uint32_t nSec     = 0;
uint32_t lastBeat = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  sampleQ = xQueueCreate(64, sizeof(Sample));

  // Khong khoi dong WiFi -> radio danh tron cho quet BLE
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanCB(), true);  // true: bao MOI goi, ke ca trung
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(SCAN_INTERVAL_MS);
  pBLEScan->setWindow(SCAN_WINDOW_MS);
  pBLEScan->start(0, nullptr, false);    // quet ngam khong dung

  Serial.printf("I,%c,%s\n", SENSOR_ID, BLEDevice::getAddress().toString().c_str());
  lastBeat = millis();
}

void loop() {
  Sample s;
  if (xQueueReceive(sampleQ, &s, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.printf("R,%c,%lu,%lu,%d\n", SENSOR_ID,
                  (unsigned long)s.seq, (unsigned long)s.ms, (int)s.rssi);
    nSec++;
  }
  uint32_t now = millis();
  if (now - lastBeat >= 1000) {
    Serial.printf("H,%c,%lu,%lu,%lu\n", SENSOR_ID,
                  (unsigned long)now, (unsigned long)nSec, (unsigned long)nDrop);
    nSec = 0;
    lastBeat = now;
  }
}
