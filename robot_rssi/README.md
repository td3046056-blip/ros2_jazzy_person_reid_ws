# Robot RSSI Tracking — 4 board (S3 Beacon + NodeMCU Master + 2 Mtiny Slave)

## Cấu trúc project
```
robot_rssi/
├── platformio.ini          ← cấu hình 4 environment (1 project, 4 board)
├── .vscode/tasks.json      ← task nạp tất cả 4 board bằng 1 lệnh
└── src/
    ├── s3_esp/main.cpp     ← Beacon (ESP32-S3 SuperMini, đeo trên người)
    ├── mcu/main.cpp        ← Master (NodeMCU ESP32, 0°, phía trước robot)
    ├── mtiny1/main.cpp     ← Slave B (Mtiny WROVER-IE #1, 120°)
    └── mtiny2/main.cpp     ← Slave C (Mtiny WROVER-IE #2, 240°)
```

## Bố trí trên robot (nhìn từ trên xuống)
```
              PHÍA TRƯỚC
           NodeMCU (Master)
                 0°
                 |
    Mtiny#2 ---- ROBOT ---- Mtiny#1
     240°                    120°
```
- Cả 3 cảm biến cùng độ cao, anten hướng ra mép ngoài robot.
- Thân robot nằm giữa mỗi cảm biến và tâm robot (tạo "bóng RF" giúp phân biệt hướng).
- Pin/động cơ đặt giữa robot, cách đều mỗi cảm biến tối thiểu 5-10cm.

## Bước cài đặt (làm đúng thứ tự)

### 1. Xác định COM port từng board
Cắm từng board một, sau mỗi lần cắm chạy:
```bash
pio device list
```
Cổng nào mới xuất hiện là của board vừa cắm. Ghi lại 4 cổng.

### 2. Điền COM port vào `platformio.ini`
Mở file, bỏ dấu `;` ở dòng `upload_port` của từng environment, điền đúng cổng:
```ini
[env:mcu]
...
upload_port = COM9          ← xoá dấu ; và điền đúng cổng
```

### 3. Nạp Master (mcu) TRƯỚC
```bash
pio run -e mcu -t upload
pio device monitor -e mcu
```
Copy dòng `MAC cua toi: AA:BB:CC:DD:EE:FF` hiện ra.

### 4. Điền MAC vào 2 file Slave
Mở `src/mtiny1/main.cpp` và `src/mtiny2/main.cpp`, sửa dòng:
```cpp
uint8_t masterMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
```
thành đúng MAC vừa copy (giữ định dạng `0xXX`, phân cách dấu phẩy).

### 5. Nạp 2 Slave và Beacon
```bash
pio run -e mtiny1 -t upload
pio run -e mtiny2 -t upload
pio run -e s3_esp -t upload
```

### Hoặc nạp cả 4 board bằng 1 lệnh (sau khi đã điền đủ COM port + MAC)
Trong VS Code: `Ctrl+Shift+P` → `Tasks: Run Task` → chọn **"Upload TAT CA 4 board"**

## Xem kết quả

Mở Serial Monitor của **Master (mcu)** — đây là nơi in ra hướng tính được:
```bash
pio device monitor -e mcu
```

Kết quả mong đợi:
```
==========================================
 A(0)  : -55.0 dBm [OK]
 B(120): -68.0 dBm [OK]
 C(240): -71.0 dBm [OK]
 Huong : 12.3 deg | Tin cay: 0.28 OK
 => DI THANG
==========================================
```

## Hiệu chuẩn (làm sau khi thấy dữ liệu ổn định)

1. **Offset giữa các board**: đặt cả 3 board cạnh nhau, cùng hướng, đo RSSI đồng thời ~30s, tính:
   ```
   OFFSET_B = RSSI_A_trungbinh − RSSI_B_trungbinh
   OFFSET_C = RSSI_A_trungbinh − RSSI_C_trungbinh
   ```
   Điền vào `src/mcu/main.cpp`:
   ```cpp
   #define OFFSET_B     <số đo được>
   #define OFFSET_C     <số đo được>
   ```

2. **Ptx, n cho công thức khoảng cách**: đứng ở 1m, 2m, 3m, 5m so với robot, ghi log RSSI trung bình mỗi khoảng cách, dùng để fit lại tham số suy hao log-distance nếu cần tính khoảng cách sau này.

## Tham số có thể chỉnh trong `src/mcu/main.cpp`

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `CONFIDENCE_THRESHOLD` | 0.15 | Độ tin cậy tối thiểu để tin vào hướng tính được |
| `ANGLE_DEADBAND` | 15.0° | Lệch góc tối thiểu để ra lệnh quay (chống lắc) |
| EMA alpha (trong `applyEMA`) | 0.25 | Độ mượt bộ lọc — càng thấp càng mượt nhưng trễ hơn |

## Bước tiếp theo
Sau khi hướng tính ổn định, thay các dòng `Serial.println(" => QUAY PHAI")` v.v. trong `calcDirection()` (file `src/mcu/main.cpp`) bằng lệnh điều khiển động cơ thật.
