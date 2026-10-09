# CLAUDE.md — Context dự án robot bám người + né vật cản

> File này cung cấp toàn bộ context cho Claude Code. Đọc hết trước khi sửa bất kỳ dòng code nào.

---

## 0. QUY TẮC BẮT BUỘC KHI SỬA CODE

**Đây là phần quan trọng nhất. Đọc kỹ trước khi làm gì.**

### 0.1 Chỉ sửa đúng chỗ cần sửa

- **KHÔNG** refactor, đổi tên, dọn dẹp, hay "cải thiện" code đang hoạt động.
- **KHÔNG** đổi giá trị tham số đã hiệu chỉnh (mục 3) trừ khi người dùng yêu cầu rõ ràng hoặc đo lại.
- **KHÔNG** thêm abstraction layer, class mới, hay chia nhỏ file khi chưa được yêu cầu.
- Khi sửa một hàm, giữ nguyên chữ ký hàm nếu có thể. Nếu buộc phải đổi, tìm **tất cả** nơi gọi và sửa đồng bộ.
- Trước khi sửa, nói rõ: sửa file nào, hàm nào, vì sao, và ảnh hưởng tới đâu.

### 0.2 Bất biến của hệ thống — vi phạm là hỏng xe

| Bất biến | Lý do |
|---|---|
| **Chỉ MỘT node được publish `/cmd_vel`** | ROS 2 không trộn lệnh. Hai nguồn ghi cùng topic → xe nhận xen kẽ → giật cục. Hiện tại chỉ `follow_planner_node` được phép. Công cụ thử tự ghi `/cmd_vel` (`calibrate_center`, `measure_speed.py`, `rssi_rotate.py`, `rssi_seek.py`) **chỉ chạy với `calibrate.launch.py`** (không có planner) — `rssi_*` tự từ chối nếu thấy publisher khác. **Ngoại lệ có chủ ý — `rssi_follow.launch.py` (02/10):** planner được đổi tham số `cmd_vel_topic` sang `/cmd_vel_follow`, còn `scripts/rssi_follow.py` là nguồn **duy nhất** ghi `/cmd_vel` (chuyển tiếp nguyên lệnh planner khi đang bám, tự xoay khi dò RSSI) — vẫn chỉ một nguồn trên `/cmd_vel`. |
| **Tham số hình học phải TRÙNG giữa `target_tracker_node` và `follow_planner_node`** | `lidar_yaw_offset_deg`, `lidar_angle_sign`, `lidar_x`, `lidar_y`, `front_len`, `rear_len`, `half_width`, `self_filter_enabled`, `blind_sectors_deg`. Lệch nhau → hai node thấy hai thế giới khác nhau. |
| **`self_filter_enabled` phải luôn `true`** | Tắt đi → tia đập vào thân xe (0.128 m) nằm trong footprint → `rect_clearance()` trả 0 → mọi quỹ đạo bị loại → xe kẹt `BLOCKED` vĩnh viễn. |
| **`sample_window_sec` ≥ 0.4** | Nhỏ hơn → cửa sổ lấy mẫu vận tốc quá hẹp → chi phí "mượt" áp đảo lợi ích tiến lên → xe đứng yên vĩnh viễn. Xem mục 7.2-A. |
| **`start_enabled: false`** | Xe không được tự chạy khi launch. Phải gọi `/follow/enable` thủ công. |
| **Tham số mảng phải PHẲNG** | ROS 2 không hỗ trợ mảng lồng nhau. `blind_sectors_deg: [246.0, 294.0]` chứ **không** phải `[[246, 294]]`. |
| **Mọi tham số khai báo với `ParameterDescriptor(dynamic_typing=True)`** | ROS 2 phân biệt nghiêm ngặt INTEGER với DOUBLE. `-p x:=90` với mặc định `0.0` sẽ ném `InvalidParameterTypeException`. |

### 0.3 Sau khi sửa

Chạy lại test offline (không cần ROS, chỉ cần numpy):

```bash
cd ~/ros2_ws/ros2_jazzy_person_reid_ws/src/person_follow_nav/scripts
python3 test_sim.py          # 6 test: geometry, footprint, chặn đường, dự đoán, né người, khe hẹp
python3 test_memory.py        # so sánh bộ nhớ góc tương đối vs odom
python3 test_self_filter.py   # lọc thân xe
```

Tất cả phải `DAT`. Nếu có test nào `LOI`, **sửa cho đến khi đạt hoặc giải thích rõ vì sao chấp nhận được** — đừng sửa tiêu chí test để nó pass.

### 0.4 Khi không chắc

Đánh dấu `[CẦN XÁC NHẬN]` và hỏi. **Không đoán.** Đây là robot vật lý — đoán sai thì xe đâm vào người hoặc tường.

### 0.5 Ngôn ngữ

Người dùng nói tiếng Việt. Trả lời tiếng Việt. Comment trong code viết tiếng Việt **không dấu** (để tránh lỗi encoding khi build trên máy khác).

---

## 1. TỔNG QUAN DỰ ÁN

Robot bám theo một người cụ thể (đã enroll qua ReID) và né vật cản bằng LiDAR 2D.

Ba bài toán chính người dùng đặt ra:

1. **Né vật cản rồi bám tiếp** — khi người hoặc vật chen vào giữa đường.
2. **Camera cố định không xoay được** — khi xe quay để né thì người ra khỏi khung hình. (Trước đây ghi FOV 62°; đo thật 30/09 là **114.4°** ngang, ống mắt cá, ở 640×480.)
3. **Không gian hẹp** — né sao cho vừa đủ khoảng cách để tối ưu chỗ trống.

### Môi trường

| | |
|---|---|
| Máy | Lenovo Legion R9000P ADR10, AMD Ryzen 9 8945HX, 16 GiB RAM, GPU NVIDIA (PCI `2d59`) + AMD Radeon tích hợp — GPU NVIDIA **chưa dùng được** (xem 13.7) |
| OS | Ubuntu 24.04.4 LTS |
| ROS | ROS 2 Jazzy |
| User / host | `thach@thachLG` |
| Workspace | `~/ros2_ws/ros2_jazzy_person_reid_ws` |
| Python | 3.12 |

### Phần cứng

| Thiết bị | Chi tiết |
|---|---|
| Khung xe | BW-DR03, vi sai, **hai bánh TRƯỚC là bánh chủ động** (đã xác nhận vật lý: tâm hai bánh trước đứng yên khi xoay tại chỗ) |
| LiDAR | SC-Mini M1C1, chip CH340. 360 tia, 1°/tia, `angle_min=0`, `angle_max=2π`, range 0.10–10.0 m, **~10 Hz** |
| Camera | KINGSEN USB, **cố định hướng thẳng trước, không xoay được**, **GÓC RỘNG, ống "mắt cá đều": FOV ngang 114.4°, dọc 85.4° ở 640×480** (người dùng đo bằng thước 5 vạch 30/09, `measure_fov.py` khớp sai số 0.1 px; số 62° ghi trước đây SAI). Góc gần như tỉ lệ thuận với pixel (f = 324 px): x = 160 px → 28.4°, mép → 56.7°. 640×480 là ảnh cắt giữa 1440×1080 của cảm biến. Cao **34 cm** so với sàn (30/09), dự định nâng lên ~70 cm. 640×480: MJPG ≤ 25 fps, YUYV ≤ 20 fps |
| RSSI | 3 board ESP32 (NodeMCU-32S đầu xe = A; 2 Mtiny WROVER-IE anten ngoài sau xe: B phải, C trái), tam giác cân cạnh bên 40 cm, đáy 48 cm (cao 32 cm). Beacon ESP32-S3 đeo sau thắt lưng hoặc cầm tay. **Từ 30/09 cắm thẳng USB vào laptop**, firmware chung `robot_rssi/src/scanner` (in mẫu thô, không lọc). Tracker vẫn **không** dùng RSSI (`rssi_enabled: false`); từ 01/10 có 2 node riêng `rssi_scanner` + `rssi_bearing` ("xoay dò hướng"), script thử `rssi_seek.py`, và từ 02/10 `rssi_follow.py` (bám bằng RSSI + LiDAR, planner nguyên bản lái). **Từ 07/10 kết hợp với camera:** `follow_nav_real.launch.py start_rssi:=true` chạy 2 node; planner dùng `/rssi/bearing` **chỉ trong SEARCH** để tìm lại người (mục 6.3) |

### Cổng thiết bị (dùng `by-id`, không cần udev rules)

```
Khung xe : /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30BF0NT-if00-port0
LiDAR    : /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
Camera   : /dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0
```

**Cảnh báo cho ngày cắm RSSI:** LiDAR và NodeMCU RSSI **đều là chip CH340** (`1a86:7523`) và **không có serial riêng**. Khi cắm cả hai, tên `by-id` sẽ trùng hệt và Linux chỉ giữ được một symlink. Hôm đó phải chuyển sang `by-path` hoặc chạy `scripts/setup_udev.py`. **Từ 30/09 (3 board RSSI cắm USB): mọi launch phải truyền `lidar_port:=/dev/serial/by-path/...`** — để mặc định `by-id` thì `sc_mini` có thể mở nhầm NodeMCU → không có `/scan`. `rssi_log.py` từ chối mở cổng đang bị tiến trình khác giữ để không giành dữ liệu của LiDAR.

**Máy `thach@thachLG` từ 08/10 (cắm hub `1a40:0101` gồm 3 board):** cổng ghi trong `~/rssi_env.sh` (`run_full.sh` tự source). Board A/B/C ở cổng hub 1/2/3 → `/dev/serial/by-path/pci-0000:05:00.3-usb-0:1.{1,2,3}:1.0-port0` (đã kiểm MAC: A `94:e6:86:0d:ec:52`, B `48:55:19:bc:73:6a`, C `48:55:19:bc:72:d2`); LiDAR USB `3-2.2` → `/dev/serial/by-path/pci-0000:05:00.4-usb-0:2.2:1.0-port0` (đường dẫn dự kiến — **[CẦN XÁC NHẬN]** sau khi gỡ brltty, mục 7.3); khung xe vẫn `by-id` (FTDI ở USB `3-1`, cổng riêng), camera vẫn `by-id`. Đổi ổ cắm USB thì sửa `~/rssi_env.sh`.

---

## 2. HÌNH HỌC XE — ĐÃ ĐO VÀ XÁC NHẬN

`base_link` = **tâm giữa hai bánh trước** = trục bánh chủ động = tâm quay.

| Tham số | Giá trị | Nghĩa |
|---|---|---|
| `front_len` | 0.14 m | base_link → mũi xe |
| `rear_len` | 0.33 m | base_link → đuôi xe |
| `half_width` | 0.30 m | nửa bề ngang |
| Bề ngang | **0.60 m** | |
| Chiều dài | 0.47 m | |
| `lidar_x` | 0.10 m | LiDAR nằm TRƯỚC base_link |
| `lidar_y` | 0.00 m | |
| LiDAR z | 0.18 m | so với mặt đất |
| `wheel_separation` | 0.40 m | |
| `wheel_radius` | 0.0632 m | |
| `encoder_ppr_default` | 750 | |

**Xe lệch về sau rất nhiều**: mũi chỉ 14 cm phía trước nhưng đuôi kéo 33 cm phía sau.

```
Bán kính góc TRƯỚC = hypot(0.14, 0.30) = 0.331 m
Bán kính góc SAU   = hypot(0.33, 0.30) = 0.446 m   ← quyết định
=> rotate_radius = 0.47  (0.446 + 2 cm dự phòng)
```

Khi xoay tại chỗ, **đuôi văng ra rất rộng**. Mô hình chữ nhật trong DWA kiểm tra footprint ở từng tư thế của quỹ đạo nên có tính điều này — mô hình hình tròn thì không.

LiDAR ở độ cao 18 cm: thấy chân bàn, chân ghế, ống quyển người — **không thấy** mặt bàn nhô ra, bậc thềm, dây điện. Đây là hạn chế vật lý, không sửa bằng phần mềm được.

---

## 3. GIÁ TRỊ HIỆU CHỈNH — ĐÃ CHỐT, KHÔNG ĐỔI TỰ Ý

```yaml
lidar_yaw_offset_deg: -90.0
lidar_angle_sign: 1.0
lidar_x: 0.10
lidar_y: 0.00
self_filter_enabled: true
self_filter_margin: 0.03
blind_sectors_deg: [246.0, 294.0]     # ĐỊNH DẠNG PHẲNG [lo1, hi1, ...], góc trong KHUNG LIDAR
front_len: 0.14
rear_len: 0.33
half_width: 0.30
rotate_radius: 0.47
margin_soft: 0.15
margin_hard: 0.06
```

**Driver BW-DR03** — nằm trong cả 3 launch file (không phải `follow_nav.yaml`), phải trùng nhau:

```yaml
max_percent: 60
max_linear: 0.49      # tốc độ THẬT ở 60% — KHÔNG đặt lại 0.226 (số calib ở max_percent 30)
max_angular: 2.46
```

### Quy ước khung toạ độ

```
base_link:  +x = mũi xe,  +y = BÊN TRÁI,  +yaw (angular.z dương) = QUAY TRÁI

LaserScan sc_mini: angle_min = 0, angle_max = 2π  (0..360°, KHÔNG phải -π..π)
  Công thức đổi:  a_base = lidar_angle_sign * a_laser + lidar_yaw_offset_rad
  Với xe này:     a_base = a_laser - 90°
  Kiểm chứng:     lidar 90°  → base 0°   (trước mũi)
                  lidar 180° → base +90° (bên trái)

camera_angle_deg trong /person_reid/target:
  dương = người ở BÊN PHẢI khung hình = bên phải xe = góc ÂM trong base_link
  => camera_angle_sign = -1.0
```

### Bằng chứng hiệu chỉnh

**Hướng (chế độ B):** vật cản 0.715 m tại lidar 90° → base +0.0°. Vật bên trái 0.343 m tại lidar 182.5° → base +92.5°. Sai lệch +2.5° cả hai.

Vật bên trái là **tấm phẳng**, kiểm chứng bằng mô hình `d(θ) = d_min/cos(θ−90°)`, sai lệch trung bình **0.6 cm** trên 13 điểm từ +72° đến +132°.

**Thân xe (chế độ A):** cụm tia ổn định 100% tại lidar 270–290°, khoảng cách 0.128 m, tương ứng base ≈ 180° (thẳng sau xe). Thêm hai bin yếu hơn ở 250–255° (0.116 m, 50%) và 265–270° (0.131 m, 7%). Cung `[246, 294]` bao trọn cả cụm.

**Tâm quay:** đường cong chữ V, độ tương phản 43% (5993 → 4195 ô lưới), đáy tại 0.10 m, khớp parabol cho **0.098 m**. `L = 0.10 − 0.10 = 0` → tâm quay ở trục trước. Xác nhận vật lý: tâm hai bánh trước đứng yên khi xoay.

Ba nguồn độc lập cùng cho `lidar_yaw_offset_deg = -90.0`: phép đo, `robot_rssi_ros2/SYSTEM_CONTEXT.md` (`lidar_front_center_deg: 90.0`), và bytecode khôi phục từ `vfh_rssi_controller.cpython-312.pyc`.

**Tốc độ driver:** `scripts/measure_speed.py` đo 58% PWM → 0.475 m/s, 29% → 0.238, 13% → 0.107 — tuyến tính, không vùng chết. Thước 1.50 m so với odom 1.503 m. Xoay 43% → 1.763 rad/s (theo odom). `max_linear = 60 × 0.475 / 58 = 0.49`, `max_angular = 60 × 1.763 / 43 = 2.46`. **Kiểm chứng sau khi sửa (17/09):** lệnh 0.22 → thước 64.5 cm / odom 64.0 cm (~0.213 m/s); lệnh xoay 0.8 → thật 135° / odom 130.9° (~0.80 rad/s); ở 5% PWM bánh vẫn quay. Chi tiết trong `CALIBRATION.md`.

---

## 4. CẤU TRÚC WORKSPACE

```
~/ros2_ws/ros2_jazzy_person_reid_ws/
├── src/
│   ├── person_follow_nav/        ← PACKAGE CHÍNH (mới, thay thế 2 controller cũ)
│   ├── person_follow_identity/   ← camera ReID → /person_reid/target
│   │                                (09/10) scripts/record_reid_data.py (ghi dữ liệu thật 2 người theo pha, ~/reid_data ngoài repo),
│   │                                eval_reid_data.py, eval_detector_data.py, replay_identity.py (chạy lại toàn bộ logic, đếm khoá nhầm)
│   ├── person_follow_robot/      ← CONTROLLER CŨ, KHÔNG DÙNG NỮA — nhưng ĐỪNG XOÁ: follow_nav_real.launch.py
│   │                                vẫn đọc config/identity_lock_kingsen.yaml ở đây
│   ├── robot_rssi_ros2/          ← RSSI serial + follow node CŨ (không dùng)
│   ├── bw_dr03_ros2/             ← driver khung xe
│   ├── sc_mini/                  ← driver LiDAR (C++, ament_cmake)
│   ├── robot_simulation/         ← Gazebo + Nav2 (ament_cmake)
│   ├── robot_model/
│   ├── person_reid_tracker/
│   └── thaihoa_follower_robot_sim/
├── robot_rssi/                   ← firmware PlatformIO cho ESP32 (ngoài src)
└── third_party/deep-person-reid-master/   ← có COLCON_IGNORE
```

### person_follow_nav

```
person_follow_nav/
├── package.xml            depends: rclpy, rcl_interfaces, std_msgs, std_srvs,
│                                   geometry_msgs, sensor_msgs, nav_msgs,
│                                   visualization_msgs, python3-numpy
│                          exec: python3-serial, python3-scipy (bản đồ lưới; thiếu thì tự dùng heapq)
├── setup.py               entry_points: target_tracker, follow_planner,
│                                        calibrate_lidar, calibrate_center,
│                                        rssi_scanner, rssi_bearing
├── setup.cfg
├── README.md              hướng dẫn 6 giai đoạn + tinh chỉnh tham số
├── CALIBRATION.md         hồ sơ hiệu chỉnh với số liệu thật
├── config/
│   ├── follow_nav.yaml    tham số cho CẢ HAI node
│   ├── rssi.yaml          tham số 2 node RSSI (01/10)
│   └── rssi_template.json mẫu hình dạng 3 board theo góc + hướng nhìn (từ rssi_rotate.py --calib trên xe)
├── launch/
│   ├── follow_nav_real.launch.py   toàn bộ hệ thống (lidar+driver+camera+2 node; start_rssi:=true thêm 2 node RSSI)
│   ├── test_avoid_only.launch.py   lidar+driver+planner, KHÔNG camera
│   ├── calibrate.launch.py         CHỈ lidar+driver (không planner)
│   ├── rssi.launch.py              CHỈ 2 node RSSI (không node nào ghi /cmd_vel); chạy cùng calibrate.launch.py
│   └── rssi_follow.launch.py       BÁM bằng RSSI + LiDAR, không camera: lidar+driver+2 node RSSI+planner (planner ghi /cmd_vel_follow)
├── person_follow_nav/
│   ├── geometry.py                 hàm dùng chung
│   ├── target_tracker_node.py
│   ├── follow_planner_node.py
│   ├── calibrate_lidar_node.py
│   ├── calibrate_center_node.py
│   ├── rssi_df.py                  thuật toán "xoay dò hướng" (thuần numpy; node, rssi_rotate, sim_rssi dùng chung)
│   ├── rssi_io.py                  đọc cổng serial 3 board (tự nối lại; KHÔNG mở cổng đang bị tiến trình khác giữ)
│   ├── rssi_scanner_node.py        3 board → /rssi/raw, /rssi/status
│   └── rssi_bearing_node.py        /rssi/raw + /odom → /rssi/bearing; service /rssi/reset
└── scripts/
    ├── test_sim.py            6 test offline (không cần ROS)
    ├── test_memory.py         so sánh bộ nhớ góc vs odom
    ├── sim_follow.py          mô phỏng vòng kín bám người (planner + tracker THẬT): cửa, người chen, vật thấp (LOWBOX), vách cao, RSSI giả
    ├── regress_follow.py      bộ HỒI QUY ~190 kịch bản sim_follow.py chạy song song, bảng ĐẠT/LỖI, so với commit cũ (--base)
    ├── test_self_filter.py    kiểm tra lọc thân xe
    ├── test_reacq.py          kiểm tra offline luật "khoá lại nhanh sau khi mất dấu" của rssi_follow.py (06/10)
    ├── preflight.sh           kiểm tra trước khi cho xe chạy
    ├── run_full.sh            chạy TOÀN BỘ hệ thống (+ RSSI) bằng một lệnh, kiểm tra trước (brltty, CH340, cổng, node cũ, symlink) (08/10)
    ├── watch_follow.py        theo dõi 1 dòng / 0.5 s: state, lệnh, nguồn, khoảng cách, camera, RSSI, topic cũ — CHỈ đọc topic (08/10)
    ├── fake_target.py         giả lập "người" để test không cần camera
    ├── measure_speed.py       đo tốc độ thật → max_linear / max_angular của driver
    ├── rssi_log.py            ghi mẫu RSSI thô từ 3 board quét (không cần ROS) — giai đoạn R0
    ├── rssi_rotate.py         thử nghiệm "xoay dò hướng": xe tự xoay, ước lượng hướng người từ pha RSSI
    ├── sim_rssi.py            mô phỏng offline RSSI: so thuật toán dò hướng + vòng kín "dò → đi" tìm lại người
    ├── rssi_seek.py           THỬ bám CHỈ bằng RSSI: dò → quay → đi tới người đeo beacon (tự ghi /cmd_vel — chỉ với calibrate.launch.py)
    ├── rssi_follow.py         BÁM bằng RSSI + LiDAR: RSSI nhận chủ, LiDAR giữ bám nhóm chân, planner nguyên bản lái xe (chỉ với rssi_follow.launch.py)
    ├── sil_rssi.py            mô phỏng vòng kín CÓ ROS: thế giới giả + 2 node RSSI thật + rssi_seek.py / (planner + rssi_follow.py) thật (miền ROS riêng)
    ├── diagnose.py            chẩn đoán khi xe không né
    ├── run_calibration.sh     chạy 4 bước hiệu chỉnh, tự lưu log
    ├── setup_udev.py          tạo udev rules (xử lý CH340 trùng serial)
    └── check_devices.sh       chẩn đoán cổng USB
```

---

## 5. KIẾN TRÚC PHẦN MỀM

```
camera USB ─► person_follow_identity ─► /person_reid/target ─┐
SC-Mini    ─► sc_mini                ─► /scan ──────────────┼─► target_tracker
BW-DR03    ─► decoded_serial_node    ─► /odom ──────────────┘        │
                                                                      ▼
                                                            /follow/target
                                                                      │
                            /scan, /odom ────────────────► follow_planner ◄── /rssi/bearing (chỉ trong SEARCH)
                                                                      │              ▲
                                                                 /cmd_vel     rssi_bearing_node ◄─ /rssi/raw ◄─ rssi_scanner_node ◄─ 3 board ESP32
                                                                      ▼        (start_rssi:=true)
                                                            decoded_serial_node
```

### Topic

| Topic | Kiểu | Nguồn → Đích |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | sc_mini → tracker, planner, diagnose |
| `/odom` | `nav_msgs/Odometry` | bw_dr03 → tracker, planner |
| `/person_reid/target` | `std_msgs/String` (JSON) | person_follow_identity → tracker |
| `/rssi/angle_deg`, `/rssi/confidence` | `std_msgs/Float32` | rssi_serial_node (**CŨ**, tắt: `rssi_enabled: false`) → tracker |
| `/rssi/raw` | `std_msgs/String` (JSON) | rssi_scanner_node → rssi_bearing_node (mẫu thô `[id, rssi, t]`) |
| `/rssi/status` | `std_msgs/String` (JSON) | rssi_scanner_node → giám sát (1 Hz: từng board `hz`, `dbm`, cổng) |
| `/rssi/bearing` | `std_msgs/String` (JSON) | rssi_bearing_node → planner (**chỉ trong SEARCH**, từ 07/10), `rssi_seek.py`, `rssi_follow.py`. Tracker không đọc |
| `/follow/target` | `std_msgs/String` (JSON) | tracker → planner (ở `rssi_follow.launch.py` không có tracker: `rssi_follow.py` → planner, `source` = `rssi+lidar` / `predicted` / `rssi_bearing`) |
| `/follow/target_marker` | `visualization_msgs/Marker` | tracker → RViz |
| `/follow/planner_status` | `std_msgs/String` (JSON) | planner → giám sát |
| `/follow/debug_markers` | `visualization_msgs/MarkerArray` | planner → RViz |
| `/cmd_vel` | `geometry_msgs/Twist` | **CHỈ** planner → bw_dr03 (ở `rssi_follow.launch.py`: **CHỈ** `rssi_follow.py` → bw_dr03) |
| `/cmd_vel_follow` | `geometry_msgs/Twist` | chỉ có ở `rssi_follow.launch.py`: planner → `rssi_follow.py` (chuyển tiếp sang `/cmd_vel` khi đang bám) |
| `/bw_dr03/sonar` | 2 sonar | bw_dr03 → **chưa dùng** |

### Service

| Service | Kiểu | Node | Tác dụng |
|---|---|---|---|
| `/follow/enable` | Trigger | planner | Bật bám |
| `/follow/disable` | Trigger | planner | Tắt bám, dừng xe |
| `/follow/stop` | Trigger | planner (ở `rssi_follow.launch.py`: `rssi_follow.py`, xem dòng `/rssi_follow/stop`) | **Dừng khẩn cấp** |
| `/follow/reset_tracker` | Trigger | tracker | Xoá bộ nhớ vị trí người |
| `/person_reid/start_enroll` | Trigger | identity | Bắt đầu học người |
| `/person_reid/finish_enroll` | Trigger | identity | Kết thúc học |
| `/rssi/reset` | Trigger | rssi_bearing | Xoá mẫu RSSI cũ trước một lần xoay dò mới (planner gọi khi bắt đầu xoay dò trong SEARCH) |
| `/rssi_follow/stop` | Trigger | `rssi_follow.py` | **Dừng khẩn** chế độ bám RSSI + LiDAR, mọi lúc. Ở `rssi_follow.launch.py` **`/follow/stop` cũng do `rssi_follow.py` nhận** (mọi lúc); service dừng của planner bị đổi tên thành `/rssi_follow/planner_stop` — để planner nhận thì lúc script tự xoay dò / lùi, planner vốn đã tắt, gọi `/follow/stop` không dừng được xe (mô phỏng 03/10: xe chạy tiếp 2 m) |

### Payload JSON

**`/person_reid/target`** (đọc bởi tracker): `camera_angle_deg`, `bbox` `[x1,y1,x2,y2]`, `target_found`, `identity_ready`, `status`, `ts` (dùng bù độ trễ — từ 30/09 là **lúc chụp khung**, trước đó là lúc xử lý xong nên bù gần như bằng 0). Thêm từ 30/09 (tracker không đọc): `occlusion`, `evidence`, `view_bucket`, `latency_ms`. `status` có thêm `TRACK_VERIFY_HOLD` = vẫn giữ khoá nhưng nghi là người khác nên `target_found: false`. **Thêm 07/10 (tracker đọc):** `bbox_top_elev_deg`, `bbox_bottom_elev_deg` (góc cao đỉnh/đáy bbox trong khung xe, đã bù `camera_pitch_deg`, cùng mô hình mắt cá với góc ngang), `bbox_top_cut`, `bbox_bottom_cut` (bbox chạm mép trên/dưới ảnh).

**`/follow/target`** (tracker xuất): `stamp`, `status`, `valid`, `source`, `measured_this_tick`, `confidence`, `age_since_fix_sec`, `odom_frame`, `robot{x,y,yaw}`, `odom_x`, `odom_y`, `vx`, `vy`, `speed`, `base_x`, `base_y`, `distance_m`, `bearing_rad`, `bearing_deg`, `in_camera_fov`. Thêm 07/10 (để xem log): `cam_dist_m`, `cam_dist_from` (`dau`/`chan`/`dau+chan`/`dau-cat`), `person_height_m`, `elev_bias_deg` (chiều cao người và sai lệch góc ngửa đã tự học).

**`/follow/planner_status`**: `stamp`, `enabled`, `state`, `note`, `cmd_v`, `cmd_w`, `target_distance_m`, `target_bearing_deg`, `target_source`, `avoid_side`, `clearance_m`, `chosen_heading_deg`, `front_clearance_m`, `n_obstacles`.

**`/rssi/bearing`** (5 Hz): `stamp`, `valid`, `reason`, `swept_deg`, `n_samples`, `levels{A,B,C}` (dBm, 2 s gần nhất), `beacon_age_sec`, `beacon_ok`; khi đã khớp được mẫu thêm `bearing_odom_rad` (**dùng cái này để quay xe**), `bearing_base_rad`, `bearing_base_deg` (so với mũi xe lúc này, trái dương), `corr`, `margin`, `cover`.

---

## 6. THUẬT TOÁN

### 6.1 `geometry.py` — hàm dùng chung

| Hàm | Vai trò |
|---|---|
| `wrap_pi`, `wrap_pi_np`, `angle_diff` | Chuẩn hoá góc về [−π, π] |
| `yaw_from_quaternion(x,y,z,w)` | Trích yaw, không cần `tf_transformations` |
| `scan_to_base_points(...)` | LaserScan → điểm trong base_link. **Trả về 4 giá trị**: `(pts, bear, rng, lidar_deg)` |
| `downsample_polar(...)` | Giảm điểm theo ô góc, giữ **toàn bộ** điểm trong 1.0 m |
| `cluster_points(...)` | Gom cụm liên tục trong một cung góc, dùng tìm "chân người" |
| `self_filter_mask(...)` | **Lọc tia đập vào thân xe** — hai lớp: footprint + blind sectors |
| `rect_clearance(lx, ly, F, R, HW)` | Khoảng cách từ mép footprint **chữ nhật** tới điểm; trả 0 nếu bên trong |
| `segment_blocked(pts, gx, gy, w)` | Đường thẳng tới đích có bị chắn không |
| `AlphaBeta2D` | Bộ lọc alpha-beta 2D cho vị trí người trong odom |

### 6.2 `target_tracker_node` — "người đang ở đâu trong khung odom?"

**Đây là node giải quyết vấn đề cốt lõi.** Nó lưu vị trí người ở khung `odom` (cố định với mặt đất) thay vì góc tương đối với thân xe. Khi xe quay, góc tương đối tới điểm đã nhớ **tự động cập nhật đúng**.

Nguồn dữ liệu, ưu tiên giảm dần:

| # | `source` | Độ chính xác | Khi nào |
|---|---|---|---|
| 1 | `camera+lidar` | góc ±1°, khoảng cách ±2 cm | Bình thường. Góc từ camera để tìm cụm, rồi **dùng luôn góc của cụm LiDAR** (chính xác hơn) |
| 2 | `camera+bbox` | góc ±1°, khoảng cách ±20 cm | LiDAR không thấy cụm nào hợp lệ |
| 3 | `lidar_track` | ±2 cm | Camera bị che nhưng chân người còn thấy, tối đa `lidar_only_max_sec` = 3.0 s |
| 4 | `predicted` | trôi dần | Bị che hoàn toàn, tối đa `predict_max_sec` = 2.5 s |
| 5 | `rssi_bearing` | ±20° | Mất > 1.2 s, **chỉ sửa hướng**, giữ nguyên khoảng cách dự đoán |

**Bù độ trễ camera:** YOLO + DeepSORT + ReID trễ 80–250 ms. Node giữ lịch sử yaw 1.5 s và trừ đi góc xe đã quay trong khoảng trễ đó (`compensate_camera_latency: true`). Ở 0.5 rad/s, 250 ms = 7° sai lệch — đủ để bám lệch ra ngoài khung hình.

**Mỗi phép đo chỉ dùng MỘT lần (sửa 17/09).** Tracker chạy 20 Hz nhưng LiDAR 10 Hz, camera 8 Hz. `_update()` chỉ gọi `filter.update()` khi có vòng quét mới (`scan_used_time`) hoặc khung camera mới cho bbox (`cam_used_time`). Giữa hai phép đo, trong `measurement_hold_sec` (0.3 s), giữ nguyên nhãn nguồn cũ — không nhảy sang `predicted` mỗi nhịp.

**Chặn nhảy vị trí:** cụm LiDAR cách vị trí dự đoán > `lidar_distance_max_jump_m` (0.6 m) bị bỏ — khi còn phép đo gần đây (< 1 s, **bbox của camera cũng tính**) và chưa quá `lidar_reject_max_sec` (3 s) kể từ lần cuối LiDAR khớp. Nhờ vậy người thứ hai đứng chen giữa xe và người đang bám (chân họ nằm trong cửa sổ ±10° quanh hướng camera, chân người đang bám bị che) **không bị nhận nhầm** trong 3 s đầu; khoảng cách lấy từ bbox. Quá 3 s thì nhận lại cụm LiDAR để đồng bộ, phòng dự đoán trôi. Tham số max_jump trước 17/09 khai báo nhưng **không dùng**; bản 17/09 tự tắt sau 1 s không có LiDAR → vẫn bám nhầm (7.2-V).

**Chặn vật che trước người (sửa 22/09, lần 2):** cụm LiDAR **gần xe hơn** vị trí dự đoán quá `occluder_margin_m` (0.3 m) bị bỏ — nó nằm giữa xe và người đang bám nên không thể là người đó. Áp cho cả ghép theo camera (cùng điều kiện với cổng nhảy ở trên) lẫn `_nearest_cluster_to` của `lidar_track`. Bắt được: người thứ hai đứng **sát** < 0.6 m trước người đang bám (cổng nhảy bỏ lọt), người đi ngang **che camera** (trước đây `lidar_track` vơ chân họ → xe bám theo người đi ngang), thanh cửa khi người đã qua cửa.

**Cổng vật che lúc camera không thấy (29/09):** ở `lidar_track`, ngưỡng co giãn theo tốc độ người: `lidar_track_occluder_margin_m` (0.15 m) + 0.25 m cho mỗi m/s, tối đa `occluder_margin_m` (0.3). Người đang bám **đứng yên** mà người thứ hai bước vào đứng chen ngay trước: **chân sau** của người thứ hai chỉ gần xe hơn 0.25 m, lọt ngưỡng 0.3 → tracker bám luôn họ (mô phỏng 73% thời gian). Ngưỡng chặt cố định 0.15 thì lại làm mất người **bước ngang nhanh về phía xe** (lệch tối đa 23° → 31°) — nên phải co giãn theo tốc độ.

**Vận tốc giảm dần khi mất hẳn:** hết `measurement_hold_sec` mà không có phép đo → `vx, vy *= e^(-dt/predict_velocity_decay_sec)` (1.0 s). Dự đoán dừng gần chỗ thấy người lần cuối thay vì trôi thẳng theo hướng cũ.

**Hình học DỌC của camera (07/10) — chặn nhận nhầm vật thấp che chân.** Camera (cao `camera_height_m` 0.34) gửi góc cao đỉnh/đáy bbox. Cụm LiDAR ở khoảng cách d chỉ có thể là chân người nếu đáy bbox nằm ở góc `−atan(camera_height/d)` (+ sai lệch góc ngửa tự học), dung sai `feet_elev_tol_deg` 4° (+6° tới khi học xong). Người đứng sau thùng thấp 30 cm: LiDAR (18 cm) không thấy chân mà thấy mặt trước thùng ngay hướng camera, còn camera (34 cm) nhìn qua thùng thấy người nhưng đáy bbox là **mép thùng** → cụm mặt thùng bị loại (trước đây tracker nhận nó là người → "cách 1 m" trong khi thật 2 m, xe dừng trước thùng — đúng log người dùng 07/10). Không có cụm hợp lệ thì `camera+bbox` lấy khoảng cách từ **đỉnh đầu** (`(person_height − camera_height)/tan(góc đỉnh)`), khớp với chân nếu chân thấy rõ. **Mép bbox chạm mép ảnh hoặc chân bị che chỉ cho CẬN TRÊN** của khoảng cách (góc đo được nhỏ hơn góc thật) → lấy số nhỏ nhất: xe sát ghế dài, đầu ra ngoài mép trên, chân bị ghế che → trước đây tính từ đáy bbox ra 3–6 m (thật 1.3 m). `person_height_m` (1.65, kẹp 1.2–2.1) và sai lệch góc ngửa (`elev_bias`) tự học mỗi lần `camera+lidar` khớp và thấy cả chân. Camera gửi bản cũ (không có góc cao) thì dùng `bbox_height_at_1m_px` như trước.

**Tự đo góc ngửa camera bằng LiDAR (08/10, `_check_geometry`).** Cổng chân ở trên giả định `camera_pitch_deg` / `camera_height_m` đúng: camera thật ngửa ~17° mà cấu hình 0 → đáy bbox thấp hơn chỗ chân 17° → cổng loại **cả cụm chân thật** → `camera+bbox` tính từ chân ra 0.5–0.8 m trong khi thật 2–3 m → planner tưởng đã tới, chỉ xoay tại chỗ (7.2-AO). Việc học cũ (`_learn_geometry`) chỉ chạy khi cụm đã qua cổng và chỉ trong ±15° nên không bao giờ tự sửa được. Mỗi khung camera có vòng quét mới: lấy cụm LiDAR **gần nhất** theo hướng camera (cách d ≤ 4 m), sai lệch `b = góc đáy bbox + atan(cao camera / d)`; chỉ nhận mẫu khi đỉnh và đáy bbox không chạm mép ảnh và chiều cao người suy từ đỉnh (đã bù b) trong 1.35–2.05 m — người sau vật thấp (cụm = mặt vật, đáy bbox = mép vật) cho ~0.9–1.2 m, cụm là vật sau lưng người cho > 3 m → bị loại. Đủ 12 mẫu trong 20 s, trung vị độ lệch ≤ 2.5° (dữ liệu thật 08/10: 1.6°), khác sai lệch đang dùng > 4° → thay luôn, đặt như đã học xong (dung sai cổng 4°) và in **WARN** `Goc ngua camera lech X do ... them +Y do` (Y = số độ cần cộng vào `camera_pitch_deg`). Khớp rồi thì in một lần `Hinh hoc camera khop LiDAR`. Giới hạn học tinh `_learn_geometry` nới ±15° → ±35°. Camera **cúi** so với cấu hình mà người luôn ở gần (đầu chạm mép trên) thì không có mẫu — vẫn bám được nhờ `lidar_track` và luật cận trên.

### 6.3 `follow_planner_node` — nguồn DUY NHẤT ghi `/cmd_vel`

Hai tầng.

**Tầng 1 — chọn khe (kiểu VFH+ / Follow-The-Gap)**

Quét các hướng từ −100° đến +100°, **bước 2°**. Với mỗi hướng, kiểm tra đi được `probe` mét mà footprint không chạm gì (`corridor = half_width + margin_hard = 0.36`). Thử lần lượt `probe_distances: [1.6, 1.1, 0.7, 0.45]` từ xa về gần.

Gom các hướng tự do liên tục thành **khe**, rồi **nhắm vào giữa khe**:
```python
pad = min(half, max(0.15 * half, radians(8.0)))
aim = clip(goal_bearing, lo + pad, hi - pad)
```
Khe rộng → bám sát hướng đích. Khe hẹp → về giữa khe.

Chi phí chọn khe: `2.2*c_goal + 0.45*c_prev + 0.8*c_fov + 0.7*c_side`.

**Tầng 2 — DWA**

- Cửa sổ lấy mẫu: `accel × sample_window_sec (0.5 s)`, **không phải** `accel × 1/control_hz`
- `n_samples_v: 7`, `n_samples_w: 17` → ~130 cặp
- Chân trời: `horizon_steps 12 × horizon_dt 0.10` = 1.2 s
- Va chạm: **footprint chữ nhật thật** ở từng tư thế (`rect_clearance`)
- `admissible = min_clear > margin_hard` (kèm luật thoát kẹt, xem dưới)
- `w_max` 1.0 rad/s, `accel_ang` 2.4 (sửa 29/09, trước 0.8 / 1.6)

Hàm chi phí:

| Thành phần | Trọng số | Công thức |
|---|---|---|
| `c_goal` | `w_goal 2.4` | `clip(1 − (d_start − d_end)/(v_max × horizon), 0, 2)` — chuẩn hoá theo **quãng đường tối đa**, không theo khoảng cách còn lại |
| `c_head` | `w_heading 1.0` | `\|wrap(góc tới đích − yaw cuối)\| / π` |
| `c_clear` | `w_clear 2.2` | `clip((soft − min_clear)/(soft − hard), 0, 1)²` với `soft` **thích nghi** |
| `c_speed` | `w_speed 0.55` | `\|V − v_pref\| / v_max`, `v_pref` giảm theo độ thoáng |
| `c_smooth` | `w_smooth 0.45` | Bậc hai của thay đổi `v`, `w` chuẩn hoá theo cửa sổ lấy mẫu |
| `c_fov` | `w_fov 1.6` | Phạt khi người ra ngoài `±fov_keep_deg (22°)` — **giải pháp cho camera cố định** |
| `c_side` | `w_side 0.9` | Phạt quay ngược bên né đã cam kết |
| `c_center` | `w_center 0.8` | Trung bình **dọc cả quỹ đạo** của `max(0, \|góc tới người\| − 4°) / fov_keep`. **Chỉ bật khi đường tới đích thoáng và không `AVOID`** (29/09) |
| tie-break | `1e-3` | `\|W\|/w_max` — ưu tiên đi thẳng khi hoà |

**Không có ràng buộc phanh riêng.** Chân trời 1.2 s đã đủ: thời gian phanh `v_max/accel = 0.63 s`, quãng đường phanh `v²/2a = 0.069 m` < quãng đường mô phỏng 0.26 m.

**Máy trạng thái:** `IDLE`, `FOLLOW`, `AVOID`, `OCCLUDED`, `SEARCH`, `ARRIVED`, `BLOCKED`, `ESTOP`.

**Tầng chọn khe dò tới tận chỗ người (sửa 22/09):** `_choose_heading` dò tới `chk_r = max(goal_r, dist − target_clear_radius_m)` (0.45 m), không chỉ tới điểm đích. Đích cách người `follow_distance` nên người thứ hai chen vào thường đứng **đúng tại đích** → trước đây hướng thẳng vẫn "thoáng", xe đi thẳng tới họ. Dịch phụ của DWA vẫn giới hạn trong `goal_r`. **Điều kiện vào `AVOID` (`segment_blocked`) vẫn chỉ tới điểm đích** — bản đầu 22/09 đo tới `chk_r` làm tia ngắm qua khung cửa hẹp sát thanh cửa bị coi là chắn → vào `AVOID` → ép vào thanh cửa (7.2-W).

**Chui khe hẹp — động tác riêng, không phải DWA (23/09, lần 2).** Cửa 0.81 m mà xe rộng 0.60 m, đuôi dài 0.33 m: lệch **14°** là hết lề an toàn. DWA nhìn trước 1.2 s (0.26 m) với 7 thành phần chi phí giằng nhau **không giữ nổi độ chính xác đó** — đó chính là lúc xe ép vào một bên khung cửa. Nên khi đường thẳng tới đích hoặc tới người không lọt, planner chuyển sang bộ điều khiển hình học `_gap_maneuver`:

1. `_gap_target` tìm khe: hai điểm LiDAR kề nhau theo góc, cách nhau 0.72 m tới `gap_waypoint_max_width_m` (1.2 m), nằm giữa xe và người, lệch hướng người < 80°. **Loại chân người** khỏi danh sách mép khe (0.35 m quanh vị trí người) — không loại thì "khe giữa chân người và thanh cửa" được chọn, trục sai hoàn toàn. **Loại khe giả**: khe phải mở ra ít nhất `gap_min_span_deg` (10°) nhìn từ xe và không nằm dọc tia nhìn (lệch > 30°) — tia LiDAR quét dọc tường ở góc rất chéo làm hai tia liền nhau cách nhau cả mét trên mặt tường, nhìn ra y như một khe 0.8 m.
2. Chỉ chạy khi mặt khe đã trong `gap_engage_range_m` (1.6 m). Xa hơn thì tầng chọn khe + DWA lái như thường.
3. Ba pha theo đúng thứ tự:
   - **Lùi ra** — còn lệch trục mà đã quá gần mặt khe: không lùi thì tự nhốt mình (tường ngay trước mũi, góc trước quét 0.33 m nên hết chỗ xoay, mà DWA không cho lùi). Giới hạn `gap_back_max_m` (0.40 m) vì **LiDAR mù thẳng phía sau** (`blind_sectors_deg`).
   - **Vào trục** — ngắm điểm trên trục khe theo kiểu pure-pursuit, nhìn trước 0.45–0.9 m. Ngắm ngang hông xe thì lệch 13 cm cũng ra lệnh quay 90° để "trượt ngang" → xe quay vòng tại chỗ.
   - **Qua khe** (`_gap_cross`) — giữ mũi theo pháp tuyến khe, vừa đi vừa sửa lệch ngang; chỉ bò `gap_cross_speed` (0.12 m/s) khi đã sát mặt khe. Lấy lệnh **đầu tiên an toàn** trong: đi tới theo luật lái → **đi thẳng** (bẻ lái trong khe làm đuôi văng) → xoay vuông mặt cửa → **lùi về trục** → (chỉ khi đã sát dưới ngưỡng) xoay nhẹ cho thoáng. Không bao giờ đứng im khi còn cách gỡ.
   - **"Đã vào trục"** tính theo bề rộng khe thật: `0.5·(khe − 2·half_width) − margin_hard − 0.01` (cửa 0.81 m → **3.5 cm**, trần `gap_axis_tol_m`), có trễ +3 cm khi đang chui. Dung sai cố định 12 cm cũ cho xe lệch 7.5 cm vào pha chui — vật lý không lọt → đứng im giữa cửa (7.2-AD).
   - **Lùi về trục** theo cung (pure-pursuit cho chiều lùi) tới một điểm trên trục khe phía sau, không lùi thẳng; đã bắt đầu lùi thì lùi đủ tới chỗ đứng chờ mới tiến lại.
   - Khi đang chui, mỗi nhịp chấp nhận khoảng hở **đo được** ≥ `gap_cross_margin_m` (**0.03**, không phải 0.06): xe lấy số nhỏ nhất trong ~15 tia chiếu vào thanh cửa nên nhiễu LiDAR kéo số đo thấp hơn thật 2–3 cm. Việc **có nên chui** (đã vào trục chưa) vẫn tính bằng `margin_hard`. Tách hai việc này là mấu chốt — gộp chung (hạ cả hai xuống 3 cm) thì kết quả **tệ hơn**.
4. **Mọi lệnh đều qua `_arc_clearance`** — mô phỏng đúng lệnh (v, w) đó trong 1.2 s và đo footprint chữ nhật, giống hệt bộ lọc va chạm của DWA. Không lệnh nào an toàn thì trả về `None` và rơi về đường cũ.

Đặt **trước** nhánh "xoay về hướng nhớ cuối" và dùng cả trong pha đi tới của `SEARCH`: người rẽ vào cửa bên hông thì tường che camera ngay, xe mà đứng xoay tại chỗ tìm người thì không bao giờ tới được cửa.

`_gap_target` chỉ nhận khe khi xe và người ở **hai phía** của khe (trừ khi xe đang đứng trong khe, tâm khe < 0.4 m), và xác định chiều chui **theo phía người**. Thiếu kiểm tra hai phía thì chiều theo người có thể lật 180° — mô phỏng 26/09: cửa trước 0.81 m, camera 100% → 11%.

**Khi xe ĐÃ lỡ sát vật (26/09).** Ngoài đời xe vẫn có lúc sát khung cửa dưới `margin_hard` (nhiễu LiDAR, quét 10 Hz, trượt bánh, `min_move_angular` nâng lệnh quay nhỏ lên 0.10 rad/s). Trước đây hai luật cùng khoá chết xe: DWA loại **mọi** quỹ đạo có chỗ nào ≤ 6 cm — kể cả quỹ đạo xoay **ra xa** khung cửa; còn `_can_rotate_in_place()` đòi trống **cả vòng 360°** bán kính 0.53 m, mà trong khung cửa 0.81 m thanh cửa chỉ cách ~0.4 m. Người dùng thấy: xe đứng im, bước sang phải để xe xoay theo cũng không được, xoay tay xe một chút thì qua. Thứ tự xử lý bây giờ:
1. **Luật thoát kẹt** (`_admissible`, dùng chung cho DWA và `_arc_ok`): đang sát vật ≤ `margin_hard` thì vẫn cho quỹ đạo nào **không lúc nào gần hơn hiện tại** (sai số 5 mm), **kết thúc xa hơn ít nhất 1 cm**, và không bao giờ dưới 1 cm. Chỉ mở thêm đường thoát, không nới gì khi xe đang thoáng.
2. **Xoay nhẹ về phía rộng** (`_gap_maneuver`): thử ±0.15, ±0.30 rad/s tại chỗ, chọn chiều làm xe xa khung cửa nhất — đúng việc người dùng làm bằng tay.
3. **Xoay về phía người theo góc thật** (`_can_turn`): các nhánh xoay về một hướng cụ thể (người ra khỏi camera, canh hướng tại chỗ, `SEARCH` quay về chỗ thấy cuối, `BLOCKED` xoay tìm lối) quét footprint đúng góc cần xoay thay vì đòi trống 360°. Pha quét của `SEARCH` vẫn dùng luật 360° vì nó xoay qua lại không giới hạn.
4. **Nhích tới theo cung** khi xoay tại chỗ sẽ quét đuôi vào vật — thường là vừa qua cửa, đuôi 0.33 m còn nằm giữa hai thanh cửa, xe đã đúng khoảng cách nên không muốn tiến.
5. **Lùi–xoay** (như lùi xe vào chuồng) chỉ khi không tiến, không xoay được, tối đa `gap_back_max_m`.

**Khe vừa đủ rộng thì KHÔNG né (sửa 23/09):** nếu đường thẳng tới đích lọt được hành lang `half_width + margin_hard` (0.72 m — đúng mức DWA đòi) thì `blocked = False`, xe cứ đi thẳng. Hành lang xét `AVOID` rộng hơn thực tế (`× block_corridor_scale 1.15` → 0.81 m) nên ở cửa 0.81 m nó **luôn** báo bị chắn dù xe đang thẳng hàng và thừa sức lọt → xe bẻ ra một bên rồi kẹt ở mép cửa (7.2-Z). Khe hẹp hơn 0.72 m vẫn bị chắn nên vẫn né như cũ.

**Chọn bên né (sửa 22/09, lần 2):** `_gap_side` chạy tầng chọn khe khi chưa thiên vị bên nào, lấy phía mà hướng tốt nhất lệch khỏi hướng đích; không rõ (< 2°) mới dùng `_choose_avoid_side` (khoảng trống trung bình hai bên — ở khung cửa hai bên gần bằng nhau nên nó rơi vào "theo dấu góc tới người", tức tung đồng xu). Hết `avoid_side_hold_sec` thì **chọn lại cả khi vẫn bị chắn** (trước đây khoá chết). Rời `AVOID` bằng bất kỳ đường nào (`OCCLUDED`, `SEARCH`…) mà đường tới đích đã thoáng thì **xoá `avoid_side`** — trước đây chỉ xoá ở nhánh `AVOID → FOLLOW`, còn lại `c_side` phạt mãi việc quay về phía đã né (7.2-X).

**Vùng chết hướng (sửa 17/09):** người lệch < `bearing_deadband_deg` (4°) → đích coi như thẳng trước mũi (`goal_b = 0`). Góc cụm chân rung vài độ mỗi bước; không có vùng chết thì DWA bẻ lái ±0.1 rad/s liên tục.

**Camera không thấy người** (`source` không bắt đầu bằng `camera` — tức `lidar_track`, `predicted`, `rssi_bearing`) và lệch > `occluded_turn_deg` (15°) → **xoay tại chỗ về phía đó trước** (trễ: tới dưới 7.5°), rồi mới tiến. Chỉ khi `_can_turn()` (quét footprint đúng góc cần xoay), và **không** khi đang `AVOID` hay đang đi vòng theo bản đồ (07/10, 13.36). Tốc độ `clip(2.2·góc, ±0.9·w_max)` — trước 29/09 là `1.5·góc, ±0.6·w_max` = tối đa 0.48 rad/s, người dùng thấy xoay theo người chậm. Ngưỡng thấp hơn nửa FOV vì góc dự đoán thường trễ hơn góc thật ~10°.

**SEARCH (sửa 17/09, viết lại 07/10)** tính từ lần cuối có mục tiêu **hợp lệ** (`last_valid_time`), bắt đầu ngay khi mất nếu đã lưu vị trí người:
1. **Pha đi tới:** lái tới cách chỗ thấy người lần cuối (`last_target_odom`, khung odom) một `follow_distance`, tối đa `search_goto_max_sec` (8 s). Đường bị chắn thì **đi theo bản đồ lưới** (đặt TRƯỚC luật "lệch > 60° thì xoay về phía người" — đang vòng qua vật mà xoay về thì mũi chúc vào vật); bản đồ báo "đứng đây là tốt nhất" thì sang pha 2. Không bị chắn: chọn khe + DWA, chỉ xoay tại chỗ khi đích lệch > 60°. **Đường tới chỗ người bị chắn** (người thứ hai đứng chen che mất người đang bám) thì coi như chưa tới nơi (7.2-AE).
2. **Quay mặt** về chỗ thấy cuối (≤ 4 s, `_can_turn`) rồi đứng 0.8 s cho camera/ReID kịp nhận lại — camera 114° nên thường thấy lại ngay. Trước 07/10 không có pha này: tới nơi mà mũi lệch 37°, rồi luật 360° cạnh thùng → bỏ cuộc dù người ngay đó.
3. **Có RSSI** (`/rssi/bearing` còn mới và `beacon_ok`): `spin` — gọi `/rssi/reset`, xoay tại chỗ một chiều `rssi_spin_w` (0.9 rad/s, cần trống cả vòng — không đủ chỗ thì `open`: nhích ≤ 0.6 m ra hướng thoáng nhất rồi xoay), nhận hướng hợp lệ khi đã xoay > 200° và tin nhắn mới hơn lúc bắt đầu; → `rssi_face`: quay camera về hướng beacon, đứng `rssi_face_hold_sec` 1.5 s chờ ReID → chưa thấy thì `rssi_go`: đi `rssi_go_dist_m` 1.5 m theo hướng đó (bản đồ nếu bị chắn) rồi xoay dò lại. Xoay hết 760° / 18 s không khớp: thử lại, 2 lần thì quét thường. Tổng tối đa `rssi_search_max_sec` 90 s.
4. **Không RSSI:** quét qua lại mỗi 3.5 s, mỗi chiều chỉ khi `_can_turn(±0.6 rad)` (13.14: luật 360° cạnh thùng/góc tường là bỏ cuộc ngay). Hết `search_max_sec` hoặc không xoay được chiều nào → `IDLE`, `last_valid_time = 0`. `/follow/enable` xoá vị trí đã lưu **và pha tìm** (13.34).

**RSSI chỉ dùng để TÌM LẠI, không dùng lúc đang bám** (đo thật 30/09–01/10): hướng beacon chỉ có khi xe xoay tại chỗ ≥ 250° (~8 s), sai ~15°, không đo được khoảng cách, đứng yên không phân biệt trái/phải. Camera + LiDAR lúc bám chính xác ±1° / ±2 cm, và camera/ReID là thứ **xác nhận đúng người** sau khi RSSI chỉ hướng. Mô phỏng 07/10 (`an_sau_vach`: người đi nhanh qua cửa sang phòng bên, đứng khuất sau vách; `sau_tu`: nấp sau tủ cao): không RSSI → `IDLE`, camera 3–6%; có RSSI → tìm lại, kết thúc `FOLLOW` cách 0.95–1.14 m.

**Bản đồ lưới + tìm đường (07/10) — `_nav_plan` / `_nav_get` / `_nav_follow`.** Bật khi đường thẳng tới đích bị chắn (`blocked`), hoặc **đích chạm lề an toàn** liên tục 1.5 s (`goal_bad`: xe đứng ở đích, quay mặt về người, thì footprint sát vật ≤ `margin_hard` — vd. đích ngay trước thùng mà người đứng sau; trước đây đường thẳng "thoáng" nên xe ở `FOLLOW`, DWA không tới được đích → đứng im mãi với v = 0, đúng 13.12; trễ 1.5 s để người thứ hai bước vào đứng ngay đích rồi đi ngay thì xe chờ như cũ, giữ camera).
- Lưới `nav_grid_res_m` 0.06 m quanh xe từ điểm LiDAR (bỏ điểm trong `nav_person_clear_m` 0.35 quanh người). Ô cấm: tâm xe cách vật < `nav_lethal_m` 0.30 (= `half_width`); trong `half_width + margin_hard + nav_soft_m` thì đắt dần (×1 → ×4 → ×7.75). Xe đang lỡ sát vật hơn ngưỡng thì các ô quanh xe không gần vật hơn chỗ đang đứng vẫn được đi (để thoát ra). Dijkstra (scipy `csgraph`; không có scipy thì heapq, lưới 0.08 m).
- Đích = ô có `J = đường đi + nav_far_weight·max(0, d_người − follow_distance)` nhỏ nhất (4; 8 khi người đi > 0.25 m/s) — đứng trước thùng cách người 1.4 m (J ≈ 1.6) rẻ hơn vòng 2 m để tới 1.0 m; người xa hơn thì vòng. Ô tốt nhất ngay dưới xe (< `nav_hold_radius_m` 0.15) → `hold`: dừng, chỉ canh hướng, `ARRIVED` ("vat chan giua — dung o cho tot nhat").
- **Giữ bên vòng** (`nav_side`): vật đối xứng (ghế dài thẳng trước người) thì hai bên rẻ ngang nhau, lập lại đường mỗi 0.2 s lại đổi bên → xe xoay qua xoay lại tại chỗ mãi. Chỉ đổi bên khi bên kia rẻ hơn rõ (J < 1.25 lần + 0.5 m).
- **Bám đường** (`_nav_follow`): điểm ngắm = điểm xa nhất trên đường trong `nav_lookahead_m` 1.2 m mà đoạn thẳng tới đó thoáng hành lang `half_width + margin_soft`. Lệch > `nav_turn_first_deg` 75° (trễ xuống 25°) → **xoay tại chỗ** rồi mới đi (DWA cắt góc: mũi chéo 45°, góc trước chạm mặt vật — chữ U, đầu ghế dài). Còn lại DWA với `c_fov` nới ra `nav_fov_keep_deg` 50° (22° thì kéo mũi về phía người = về phía vật đang chắn), `c_center` tắt. DWA đứng im, hoặc **quẹo ngược hướng đường liên tục 0.8 s** (xoay về phía đường thì góc mũi quét vào vật nên DWA chọn quẹo ngược, chui dần vào góc kẹt) → xoay về hướng đường, không xoay được thì **lùi một đoạn ngắn** (hạn mức chung `gap_back_max_m` 0.40 với động tác chui khe; điểm ngắm phía trước: lùi kèm xoay mũi về phía nó). Điểm ngắm ở **phía sau** (> 110°): còn thấy người thì đứng chờ 2 s (vật là người thì họ thường bước đi), rồi xoay, rồi mới lùi (LiDAR mù sau).
- Chui khe hẹp (`_gap_maneuver`) **chỉ khi**: chưa tới khoảng cách bám **hoặc người đang đi > 0.15 m/s** (người bước qua cửa chậm thì xe đã sát 1 m mà vẫn phải canh trục ngay), bản đồ không báo `hold`, và đường đi (nếu có) đi qua khe đó (< 0.35 m). Trước 07/10 khe nào 0.74–1.2 m giữa xe và người cũng bị chui — kể cả khe giữa hai thùng khi bên kia trống ("vào trục giữa khe" rồi "kẹt hoàn toàn"), và khe giữa hai món đồ khi xe đã tới nơi (13.37).
- Nhánh "người ra khỏi camera → xoay về hướng nhớ cuối" **không chạy khi đang `AVOID` hoặc đang đi vòng theo bản đồ** (13.36: giằng co, mũi chúc vào vật).
- Chi phí CPU (đo trên laptop của xe, đang chạy song song 14 mô phỏng): lập đường 5 ms TB, tối đa 8 ms, lập lại mỗi 0.2 s; cả nhịp điều khiển p95 ~11 ms (ngân sách 66 ms ở 15 Hz).
- Kết quả mô phỏng (`regress_follow.py --base 6d0aa54`, camera ±55°, tracker + planner mới so với bản 6d0aa54): người sau thùng 50×30 cm, thùng thứ hai bên trái/phải cách 0.5 m (khe hẹp hơn xe) hoặc 0.8 m, xe xuất phát giữa / lệch về phía khe: **13/13 tới 1.11–1.15 m, luôn vòng bên trống**, hở vật ≥ 0.14 m (bản cũ 5/13: 3 lần đứng `AVOID` cách 1.34–1.38 m, 4 lần dừng ở 1.6–2.4 m vì tưởng tới nơi, 1 lần chui khe 0.8 m tới sát người 0.74 m; 2 lần qua được cũng là chui khe 0.8 m sát 7–8 cm). 17 kịch bản khó hơn (chữ U, ghế dài 1.4 m, 3 thùng lộn xộn, đi thẳng vào khe, người đi vòng qua thùng, camera ngửa 12° cấu hình sai ±3°, người cao 1.5/1.9 m): **16/17** tới 0.8–1.4 m, không chạm (bản cũ 3/17). Ca còn lại: người 1.9 m đứng sau thùng ngay từ đầu (chưa học được chiều cao) → xe tưởng đã tới, dừng ở 1.48 m; có vài giây thấy người rõ trước đó thì tới 1.13 m. Giữ camera lúc vòng: 82–100% (vòng chữ U 70%). Hồi quy cửa không xấu đi: cửa trước, cửa bên hông, lưới 15 tư thế (tường mỏng/dày) 15/15, xe đã sát khung 12/12 (bản cũ 11/12, 1 lần chạm), gần giữa cửa nhiễu 1 cm 30/30, nhiễu 2 cm 26–27/30 (bản cũ 25/30).

### 6.4 Ba kỹ thuật cốt lõi

**Goal đặt lùi.** Đích **không** đặt tại người mà lùi lại `follow_distance` về phía xe:
```python
goal_r = max(0.0, dist - self.follow_distance)
gx, gy = goal_r * cos(bearing), goal_r * sin(bearing)
```
Lực kéo của đích và lực đẩy của vật cản tự cân bằng tại đúng 1.0 m. **Không cần loại người khỏi danh sách vật cản** — đây là cách sửa sạch cho "Bug#4" trong ghi chú cũ của người dùng (loại target → xe đâm vào vật phía sau người; không loại → xe coi người là tường).

**Margin thích nghi.** Ngưỡng mềm tự hạ xuống mức tốt nhất mà môi trường cho phép:
```python
d_sort = where(admissible, d_end, inf)
k = ceil(0.10 * n_admissible)
c_ref = max(min_clear[argsort(d_sort)[:k]])     # top 10% GẦN ĐÍCH PHỤ NHẤT
soft = clip(c_ref, margin_hard + 0.02, margin_soft)
```
**Chi tiết sống còn:** chỉ lấy mẫu từ nhóm gần đích phụ nhất. Lọc theo "có tiến về đích" thôi thì KHÔNG đủ — quỹ đạo quẹo 20° vẫn tiến được 97% quãng đường nhưng thoáng cao hơn hẳn, kéo ngưỡng lên 0.22 m, khiến đi thẳng qua khe bị phạt còn quẹo tránh không bị phạt. Xe quẹo, lệch tâm khe, rồi kẹt.

**Chi phí giữ khung hình.** Khi có hai đường né tương đương, xe chọn đường vẫn "liếc" thấy người. Khi buộc phải né sang bên làm mất hình, bộ nhớ odom giữ vị trí người và xe tự quay lại đúng hướng sau khi né xong.

### 6.5 Lọc thân xe — bắt buộc

LiDAR quét 360° nên thấy cả cột đỡ, dây điện, mép sàn của **chính robot**. Các điểm này nằm trong footprint → `rect_clearance()` = 0 → mọi quỹ đạo bị loại → **xe kẹt `BLOCKED` vĩnh viễn**.

Hai lớp:
1. Bỏ mọi điểm rơi vào footprint thu nhỏ `self_filter_margin` (0.03 m). An toàn vì vật cản thật không thể "xuất hiện" bên trong footprint mà trước đó không đi qua biên.
2. Bỏ các cung góc trong `blind_sectors_deg` (khung LiDAR), dùng cho cột đỡ cao hơn mép footprint.

Planner log mỗi 10 giây: `self-filter: bo N/M tia dap vao than xe` (M = số điểm hợp lệ trong 5 m). **Với xe này N khoảng 20–50:** ~20 là thân xe (15 tia ổn định 100% + các bin yếu), tăng tới ~49 khi phía sau xe có vật trong 5 m, vì cung `[246, 294]` bỏ **mọi** điểm dù xa hay gần. Đo 17/09: 19/227. N = 0 → bộ lọc không ăn. N > 150 → có gì đó chắn LiDAR. Kiểm tra quyết định: `diagnose.py` mục 3 "Do thoang nho nhat quanh footprint" phải > `margin_hard` — ≈ 0 là còn điểm thân xe lọt qua. (Tài liệu cũ ghi 45–55 là **sai**.)

---

## 7. LỖI ĐÃ GẶP

### 7.1 Lỗi trong code CŨ (person_follow_robot, robot_rssi_ros2)

| # | Lỗi | Ghi chú |
|---|---|---|
| 1 | **Bộ nhớ mục tiêu lưu sai hệ quy chiếu** | `follow_controller.py` giữ `camera_angle_deg` (góc **tương đối thân xe**) trong 1.5 s. Đo bằng mô phỏng: sau 1.2 s quay 35°, sai số **36.9°**, vượt nửa FOV (±31°). Đây là lý do gốc xe không thể vừa né vừa nhớ người. |
| 2 | **Hai node cùng publish `/cmd_vel`** | `person_follow_controller` + `rssi_follow_node`. Comment v1.9 trong code cũ đã mô tả đúng hiện tượng giật cục. |
| 3 | **Code VFH mất nguồn** | `SYSTEM_CONTEXT.md` mô tả `vfh_rssi_controller.py`, `vfh_params.yaml`, 2 launch file — không file nào tồn tại. Khôi phục được **61 tham số + cấu trúc 1025 dòng** từ `__pycache__/vfh_rssi_controller.cpython-312.pyc` (60 KB, 27/07). Đã sao lưu ra `~/vfh_backup.pyc`. |
| 4 | **Mô hình footprint hình tròn** | `robot_radius: 0.22` < nửa bề ngang thật → góc xe quệt vật cản mà thuật toán không biết |
| 5 | **Khoảng cách từ bbox height** | Rất nhiễu, phải hiệu chỉnh tay. Đã thay bằng ghép LiDAR. |
| 6 | **`_sector_min_distance` lấy `values[1]`** | Giá trị nhỏ thứ hai → bỏ qua chân ghế, chân bàn (chỉ 1–3 tia) |
| 7 | **`package.xml` sai** | Dùng thẻ `<n>` thay vì `<name>`; thiếu `<depend>sensor_msgs</depend>` dù subscribe LaserScan |
| 8 | **`max_percent` không nhất quán** | Launch đặt 30 nhưng tài liệu ghi phải 60 thì bánh mới quay |

### 7.2 Lỗi thiết kế trong code MỚI — đã sửa, ghi lại để không tái phạm

| # | Lỗi | Triệu chứng | Cách sửa |
|---|---|---|---|
| A | **Cửa sổ lấy mẫu DWA quá hẹp** | `accel × 1/control_hz = 0.023 m/s`. Lợi ích tiến lên trong 1.2 s chỉ 2.8 cm, nhỏ hơn **10 lần** chi phí "mượt". **Xe đứng yên vĩnh viễn.** | `sample_window_sec: 0.5`; giới hạn gia tốc thật ở `_emit()` |
| B | **Chi phí đích chuẩn hoá sai** | `c_goal = d_end/d_start` → đích càng xa tín hiệu càng yếu | Chuẩn hoá theo `v_max × horizon` |
| C | **DWA thuần bị bẫy cực tiểu địa phương** | Nhìn trước 1.2 s ≈ 0.26 m, thấy "đi thẳng thì đụng, rẽ thì cũng không gần đích hơn" → đứng im | Thêm tầng chọn khe nhìn xa 1.6 m |
| D | **Ngưỡng margin thích nghi lấy mẫu sai** | Lọc theo "có tiến về đích" không đủ (xem 6.4) | Lấy top 10% gần đích phụ nhất |
| E | **Vách phạt tốc độ gián đoạn** | `v_cap` cộng thẳng `+1.5` → "quẹo ra chỗ rộng để chạy nhanh" rẻ hơn "đi chậm chui khe" → xe không bao giờ vào khe | `c_speed = \|V − v_pref\|/v_max` liên tục |
| F | **Ràng buộc phanh sai về ý niệm** | `v ≤ sqrt(2a(clear−hard))` — nhưng `clear` phần lớn là thoáng **ngang** khi chui khe, phanh không giúp gì | Bỏ; chân trời 1.2 s đã đủ |
| G | **`np.argmin` phá vỡ đối xứng** | Cảnh đối xứng hoàn hảo → chi phí `w=+0.4` và `w=−0.4` bằng nhau **từng bit** → `argmin` luôn trả chỉ số nhỏ hơn → **luôn quẹo phải** → lệch dần rồi kẹt | Thêm `1e-3 * \|W\|/w_max` |
| H | **Bước quét hướng 4° quá thô** | Sai số 4° ở tầm 1.6 m = **11 cm lệch ngang**, gần hết lề an toàn | Đổi sang 2° |
| I | **Bám mép khe** | Chọn hướng tự do gần đích nhất → khe rộng không sao, khe hẹp ăn hết lề | Follow-The-Gap: nhắm giữa khe |
| J | **Thiếu bộ lọc thân xe** | 3 tia ở 0.128 m trong footprint → mọi quỹ đạo bị loại → **kẹt `BLOCKED` vĩnh viễn** | `self_filter_mask` |
| K | **`blind_sectors_deg` mảng lồng nhau** | ROS 2 chỉ hỗ trợ `int64[]`, `double[]`, `string[]` → node chết khi khởi động | Định dạng phẳng |
| L | **Tham số INTEGER vs DOUBLE** | `-p expect_deg:=90` với mặc định `0.0` → `InvalidParameterTypeException` | `ParameterDescriptor(dynamic_typing=True)` cho mọi tham số |
| M | **`calibrate_lidar` báo sai vật bên trái** | `self_return_max_m 0.35` > vật ở 0.343 m → nhầm là thân xe; `target_min_m 0.35` cũng cao hơn; lấy trọng tâm cung góc → vật **phẳng** cho kết quả lệch 15° | Hạ ngưỡng, dùng hồ sơ thân xe, cửa sổ hẹp ±10° quanh điểm gần nhất |
| N | **`calibrate_center` kiểm tra thoáng luôn thất bại** | Lấy `min` toàn bộ tia, mà cụm thân xe 0.128 m luôn có đó | Áp `self_filter_mask` trước |
| O | **`/scan` chạy 10 Hz không phải 20 Hz** | Checklist ghi "≥ 15 Hz" làm người dùng tưởng hỏng | Sửa thành ≥ 8 Hz, `scan_timeout_sec: 0.6` |
| P | **`max_linear` driver sai 2.16 lần** — launch dùng `max_percent 60` nhưng giữ `0.226` (số calib ở 30%) | Lệnh 0.22 m/s → xe chạy 0.475 m/s; lệnh xoay 0.8 → 1.76 rad/s. Người dùng thấy "xe chạy hơi nhanh". Dự đoán của DWA (quãng phanh, văng đuôi) sai cùng tỉ lệ | Đo bằng `measure_speed.py` + thước → `max_linear 0.49`, `max_angular 2.46`. **Không** giảm tốc bằng cách hạ `v_max` (xem mục 13.12) |
| Q | **Tracker đưa lại phép đo cũ mỗi nhịp 20 Hz** | Bộ lọc tưởng người đứng yên → vận tốc bị kéo về 0 → bám trễ khi người rẽ, dự đoán sai khi mất hình | Mỗi vòng quét / khung hình dùng 1 lần + `measurement_hold_sec` (xem 6.2) |
| R | **SEARCH không bao giờ chạy** | Planner tính "mất người" bằng `target_time`, mà tracker gửi `/follow/target` 20 Hz **cả khi `valid=false`** → `age` luôn ≈ 0 → mất người thì xe đứng yên mãi | `last_valid_time` (xem 6.3) |
| S | **Dự đoán vận tốc không đổi + không quay về phía người** | Người dùng báo: *"khi predicted xe chỉ đi thẳng"*. Mô phỏng góc tường: người rẽ khuất, dự đoán trôi thẳng, xe đi thẳng `w=0` rồi đứng yên | Giảm dần vận tốc; xoay về hướng nhớ cuối khi camera không thấy; SEARCH lái tới chỗ thấy lần cuối |
| T | **`lidar_distance_max_jump_m` khai báo nhưng không dùng** | Mô phỏng: chân người khuất sau góc tường → cửa sổ ±10° bắt nhầm đoạn tường → vận tốc ước lượng vọt 1.75 m/s, dự đoán bay 75° | Cổng chặn nhảy vị trí (xem 6.2) |
| V | **Bám nhầm người thứ hai đứng chen giữa + đích rơi đúng chỗ họ** (22/09) | Người dùng báo: *"người thứ 2 bước vào giữa thì xe cứ chạy thẳng về chân người thứ 2 dù camera vẫn thấy người đang bám"*. Mô phỏng: cổng chặn nhảy tự tắt sau 1 s không có LiDAR → nhận chân người thứ hai (58% thời gian ước lượng gần họ hơn, lệch tới 2.1 m), xe tới cách họ 0.62 m rồi đứng chờ | Cổng giữ khi camera còn thấy, tối đa `lidar_reject_max_sec`; kiểm tra đường chắn tới tận chỗ người (`target_clear_radius_m`) → xe `AVOID` vòng qua. **Đừng** tắt `lidar_track` khi camera còn thấy — đã thử, làm bước ngang nhanh mất camera 0.2 → 1.0 s |
| W | **Khung cửa hẹp: có lần qua, có lần ép vào thanh cửa** (22/09) | Người dùng báo: *"có lần đi thẳng vào thanh cửa dù camera vẫn detect, như nhầm thanh cửa là chân người"*. Mô phỏng cửa 0.82 m, người đi lệch: **không phải nhầm chân** (sai ước lượng 0.23 m) — tia ngắm sát thanh cửa → `AVOID` → `_choose_avoid_side` hai bên bằng nhau → chọn theo dấu góc (+8.6° → trái = phía tường), khoá chết bên đó → xe quay +46° ép vào thanh cửa. Bản đầu 22/09 (vào `AVOID` khi đo tới tận người) làm hay gặp hơn | `_gap_side` chọn bên theo khe đi được; cho chọn lại sau `avoid_side_hold_sec`; `AVOID` chỉ xét tới điểm đích |
| X | **Né xong người rẽ về phía đã né thì xe xoay rất chậm** (22/09) | Người dùng báo. Mô phỏng: sau khi rời `AVOID` qua `OCCLUDED`, `avoid_side` vẫn còn → mọi lệnh quay về phía đó bị cộng `0.9·\|w\|/0.8` → xe chỉ quay 0.10 rad/s dù người lệch 22° | Xoá `avoid_side` khi không còn `AVOID` và đường đã thoáng. Mô phỏng: lệch TB 20.8° → 11.3°, cuối kịch bản 20.5° → 2.5° |
| Y | **Xe bám theo người đi ngang** (22/09) | Người dùng báo. Mô phỏng: người đi ngang che camera → `lidar_track` lấy cụm gần dự đoán nhất = chân người đi ngang → ước lượng lệch 0.92 m. Cổng nhảy sau đó còn "bảo vệ" dự đoán sai | Bỏ cụm gần xe hơn dự đoán quá `occluder_margin_m` (cả ghép camera lẫn `lidar_track`) → 0% bám nhầm |
| Z | **Cửa 0.81 m: xe bẻ ra mép cửa rồi kẹt** (23/09) | Người dùng báo: cửa thật 0.81 m, hai bên trống thì **không vào được**, bám vào một bên khung cửa; có hành lang dẫn vào thì qua tốt. Mô phỏng đúng cửa 0.81 m: lúc xe **thẳng hàng hoàn hảo** (lệch tâm 0.00 m) vẫn vào `AVOID` vì hành lang xét `AVOID` rộng 0.81 m = đúng bề rộng cửa → bẻ lái ra khỏi tâm (w −0.42) → kẹt, tâm xe cách thanh cửa 0.23 m | Đường thẳng lọt hành lang 0.72 m thì không vào `AVOID` (xem 6.3). Mô phỏng: 0.81 m đi thẳng / đi lệch / đi chéo / có hành lang đều qua; 0.55 m vẫn không chui |
| AA | **Người rẽ vào cửa bên hông: xe kẹt ở mép cửa** (23/09) | Người dùng báo: đang đi thẳng rồi **rẽ phải vào cửa** 0.81 m thì xe kẹt, cứ hướng vào khung cửa (log: `AVOID` liên tục ~23 s, vài nhịp `BLOCKED`, nguồn vẫn `camera+lidar`). Mô phỏng `cua_ben`: xe dừng cách thanh cửa 0.39 m, mất người 21.6 s rồi `IDLE`. Đi chéo qua khe hẹp là **bất khả thi về hình học** — phải tới ngang tâm cửa rồi mới quay vào | `_gap_target`: ngắm trục vuông góc giữa khe (xem 6.3). Mô phỏng: qua được cửa bên hông; người chen giữa mất camera 3.8 → 1.5 s |
| AB | **Người rẽ vào cửa bên hông: xe canh góc rồi vẫn kẹt ở một bên khung cửa** (23/09, lần 2) | Người dùng báo sau khi thử bản `4765acb`: *"khi xe bắt đầu tự chỉnh góc để đi vào cửa 0.81 m thì nó chia khoảng cách không đều, góc phải rộng hơn góc trái nên xe bị kẹt ngay góc bên trái; tôi chủ động xoay xe sang phải một tí thì đi qua được"*. Mô phỏng: xe tới được trục cửa nhưng **mũi chưa vuông với mặt cửa** (lệch 40–50°), khoảng hở với khung cửa chỉ 0.02 m. Ba nguyên nhân độc lập: (a) `_gap_target` lấy **chân người** làm mép khe → trục sai hẳn 0.6 m; (b) tia LiDAR quét dọc tường ở góc chéo tạo **khe giả** rộng 0.8 m; (c) DWA không giữ nổi độ chính xác ±14° mà cửa 0.81 m đòi hỏi | `_gap_maneuver` — bộ điều khiển hình học ba pha, mọi lệnh qua `_arc_clearance` (xem 6.3). Lưới thử riêng động tác chui cửa (15 tư thế xuất phát): **8/15 → 15/15**, hở nhỏ nhất với tường **0.02 → 0.07 m** |
| AC | **Xe lỡ sát khung cửa thì đứng im mãi, dù chỉ cần xoay nhẹ là qua** (26/09) | Người dùng báo: kẹt góc trái ở thanh cửa trái; xoay tay xe tại chỗ một chút về phía rộng thì qua; bước sang phải để xe xoay theo thì xe **không làm gì được**. Mô phỏng 12 tư thế đã sát khung cửa 3–4.5 cm: bản `5285377` chỉ 4/12 tự thoát, các ca sát 3 cm đứng `BLOCKED` mãi | Luật thoát kẹt `_admissible`; `_can_turn` quét đúng góc thay cho luật 360°; nhích tới khi đuôi chặn; lùi–xoay là cách cuối (xem 6.3). Mô phỏng: **8/12** tự thoát, cả 8 ca đều bám tiếp được người (`ARRIVED`/`FOLLOW`). 4 ca còn lại xem 13.19 |
| AD | **Xe đứng im giữa cửa 0.81 m, "lệch −3..−7 độ"** (29/09) | Người dùng báo sau bản `bc3dcdf`: xe gần như đã ở giữa cửa, thực tế đi được, nhưng đứng mãi với `note` = `chui khe hep — qua khe theo truc (lech -3 do)`; có lần qua có lần không. Mô phỏng **chỉ tái hiện được khi tường có độ dày** (`WALL_T=0.12`) — tường mỏng như giấy thì không. Ba nguyên nhân: (a) luật lái `psi + 1.2·lệch_ngang`: lệch tâm và lệch góc trái dấu nhau, cộng lại ~2° rơi vào vùng chết → `w=0`, tiến thì sát khung → `v=0`; (b) dung sai "đã vào trục" 12 cm cố định trong khi cửa 0.81 m chỉ cho ~4 cm; (c) nhiễu LiDAR: xe lấy số nhỏ nhất trong ~15 tia nên tưởng hở 3–4 cm khi thật là 7 cm | `_gap_cross` (chuỗi lệnh dự phòng, ưu tiên đi thẳng), dung sai vào trục theo khe thật, lùi theo cung về trục, `gap_cross_margin_m` 0.03 (xem 6.3). Xe gần giữa cửa tường dày: nhiễu 2 cm **7/30 → 28/30**, nhiễu 1 cm **20/30 → 30/30**; chui từ xa tường dày **6/15 → 15/15** |
| AE | **Người thứ hai chủ động bước vào: lúc né, lúc bò tới sát chân họ** (29/09) | Người dùng báo. Hai nguyên nhân: (a) người thứ hai che kín cả camera lẫn chân người đang bám → `SEARCH` lái tới "cách chỗ thấy cuối 1 m" = đúng chỗ người thứ hai → xe bò tới sát họ; (b) tracker nhận nhầm chân sau của người thứ hai (chỉ gần hơn 0.25 m, lọt cổng 0.3 m) — mô phỏng 73%. Lúc camera còn thấy một phần người đang bám thì `AVOID` vẫn né được → "lúc né lúc không" | `SEARCH` dò đường tới tận chỗ người; cổng vật che `lidar_track` co giãn theo tốc độ người. Lưới 24 ca: lỗi **7 → 3**, nhận nhầm **73% → 0%**. 3 ca còn lại xem 13.22 |
| AF | **Xe xoay theo người chậm khi người đi tốc độ bình thường** (29/09) | Người dùng báo. Mô phỏng người đi 1 m/s rẽ: DWA chỉ chọn `w` ≈ 0.28 rad/s dù được phép 0.8 — chi phí hướng chỉ xét **tư thế cuối** (1.2 s) nên xe quay từ từ cho vừa hết 1.2 s, người đi tiếp → trễ ~15°. Xoay tìm người khi ra khỏi camera bị chặn 0.48 rad/s. Tăng `w_heading` hay hạ `fov_keep_deg` **không có tác dụng** (đã thử) | `c_center` trung bình dọc quỹ đạo, chỉ bật khi đường thoáng; `w_max` 1.0, `accel_ang` 2.4; xoay tìm người `2.2·góc, ±0.9·w_max`. Người đi 1 m/s: lệch 15.1° → **13.3°**, giữ khung hình 92–95% → **100%**; bước ngang nhanh 6.4° → **3.6°**; đi thẳng 1.6° → 0.6°. Bật `c_center` cả lúc né thì xe lao vào người chen (test 5 LỖI) — phải chặn |
| AG | **Camera/ReID: lẫn người mặc giống, mất khoá khi bị che, nhận lại chậm** (30/09, chỉ sửa phần camera) | Người dùng báo 4 vấn đề: nhầm người mặc giống, khoá không ổn định khi vào/ra đám đông hay bị che một phần, nhận lại chậm, méo/tỉ lệ người đổi theo khoảng cách ở camera góc rộng. Nguyên nhân đo được: (a) extractor nạp **BGR** vào mạng học bằng RGB — Market-1501 Rank-1 **39.8%** (đúng ra 75.5%); (b) `mediapipe 0.10.35` bỏ `mp.solutions` → gait chỉ còn bóng Otsu + tỉ lệ bbox, lại **bắt buộc** khi tìm lại; (c) `verification_fail_limit: 1` — một khung bị che là bỏ khoá; (d) `recover_min_combined 0.80` loại ~40% khung của chính người thật; (e) gallery âm học cả người **đứng sau** mục tiêu (crop chứa điểm ảnh mục tiêu) → mục tiêu bị loại về sau; (f) ở ~1 m camera 62° không thấy hết người → ảnh bị cắt so với gallery toàn thân: người thật 0.79, lẫn vào người lạ 0.72; (g) timer 8 Hz đọc bộ đệm V4L2 → khung cũ; ReID tính 2 lần/khung; `ts` = lúc xử lý xong; (h) OpenBLAS đa luồng (tích ma trận của DeepSORT) tranh CPU với torch → xử lý trượt 29 → 59 ms/khung sau vài giây | Sửa RGB; tắt gait; khoá theo điểm bằng chứng (tối đa 3, khung bị che trừ 0.25), bỏ ngay khi mâu thuẫn rõ; tìm lại bằng bằng chứng từng ứng viên; gallery theo kiểu khung (toàn thân/cắt đầu/cắt chân) + mẫu enroll neo; gallery âm theo từng người, chỉ học người không bị che; mục tiêu biến mất khi đang chạm ai → bỏ ID (DeepSORT hay tráo ID lúc chồng nhau), tìm lại bằng ngoại hình; dùng lại feature DeepSORT, YOLO 640×480, luồng đọc camera riêng, `ts` = lúc chụp, 15 Hz, `OPENBLAS_NUM_THREADS=1` đầu `node.py` (**không** đặt `MKL_NUM_THREADS`/`OMP_NUM_THREADS`: torch đọc chúng và chạy 1 luồng). Khung 6 người: 68.7 → 40.3 ms. Chi tiết + số liệu: `src/person_follow_identity/README.md`, mô phỏng `scripts/sim_identity.py` |
| AH | **Camera KINGSEN chỉ ra 10 fps, ảnh nhoè** (30/09, đo khi cắm vào laptop) | Node nhận 10.1 fps (config đòi 15 Hz). OpenCV tự chọn YUYV (≤ 20 fps ở 640×480, MJPG mới ≤ 25), và camera bật sẵn `backlight_compensation=1` = chế độ thiếu sáng: giảm còn **nửa fps**, phơi sáng ~80 ms. Cờ `exposure_dynamic_framerate` bị firmware bỏ qua. Nhoè 28 px (người đi 1 m/s ở 1.5 m): ReID Rank-1 76.7% → 59.2%, YOLO mất hết người ở conf 0.55 | `camera_fourcc: MJPG`, tắt `backlight_compensation`, phơi sáng thủ công do phần mềm chỉnh theo độ sáng, **tối đa 30 ms** (39 ms sát chu kỳ 40 ms → camera tụt 12.5 fps). Kết quả: 25 fps sau 7–45 s khởi động, xử lý 15 Hz, trễ 38–65 ms. `det_conf_thres` 0.55 → 0.40 vì người chỉ lộ một phần chỉ cho conf 0.4–0.7. **FOV thật 114.4°, ống mắt cá đều, không phải 62°** (người dùng đo 30/09): với code cũ (tuyến tính 62°), người lệch giữa khung bị báo ~một nửa góc thật (thật 28.4° → báo 15.5°), lọt cửa sổ ghép LiDAR ±10° của tracker → rơi về `camera+bbox` với góc sai; có thể góp phần vào việc xoay theo người chậm (7.2-AF). Đã sửa phía camera: `camera_angle_model: calibrated` + tham số mắt cá khớp từ 5 vạch |
| AI | **YOLO sót 1 khung là mất sạch track** (30/09, chạy thật với KINGSEN) | Khung YOLO không phát hiện được ai thì `detect_and_track` bỏ qua hẳn DeepSORT → không xuất track nào (6 lần / 52 s đang bám), và DeepSORT không được tính tuổi ở khung trống (`max_age` chỉ đếm khung có người → cảnh trống lâu rồi người mới vào vẫn có thể nối ID cũ) | Cập nhật DeepSORT với danh sách rỗng: track vừa mất 1 khung được xuất ở vị trí dự đoán (`conf = 0`), khung sau thì hết. Trình khoá vẫn báo track dự đoán cho liên tục nhưng không chấm điểm, không học, không nhận lại trên nó |
| AJ | **Người đứng sau thùng thấp: tracker nhận mặt thùng là người** (07/10, xe thật) | Người dùng báo: người đứng sau thùng 50×30 cm cao ~30 cm, cách xe ~2 m, camera (34 cm) vẫn thấy người nhưng bbox không trọn chân; lúc kẹt log báo người cách ~1 m. LiDAR (18 cm) không thấy chân người mà thấy mặt trước thùng **đúng hướng camera** → cửa sổ ±10° ghép nhầm → `camera+lidar` với khoảng cách của thùng. Mô phỏng: "giu khoang cach 0.74m" trong khi thật 1.84 m | Hình học dọc camera (6.2): đáy bbox phải ở góc `−atan(cao camera / d)` mới là chân ở khoảng cách d → loại cụm mặt thùng; khoảng cách từ đỉnh đầu; mép bị cắt/che chỉ là cận trên. Camera gửi thêm góc cao bbox (`person_follow_identity/core.py`) |
| AK | **Người sau thùng, thùng thứ hai cách 50 cm một bên: xe đi vào phía có thùng rồi kẹt** (07/10, xe thật) | Người dùng báo: có lần xe không đi bên trống mà đi phía có thùng thứ hai (có thể trước đó xe đang lệch về phía đó). Ba nguyên nhân (mô phỏng): (a) tầng chọn khe dò thẳng 1.6/1.1/0.7/0.45 m, xa thì không hướng nào lọt nên lùi về tầm ngắn — lúc đó khe 0.5 m giữa hai thùng cũng "thoáng"; (b) `_gap_maneuver` coi khe 0.5–1.2 m giữa hai thùng là khung cửa → "vào trục giữa khe" rồi "kẹt hoàn toàn"; (c) điểm đích (cách người 1 m) nằm **trong** thùng → DWA đứng im mãi | Bản đồ lưới + Dijkstra tới chỗ đứng tốt nhất (6.3): chọn bên theo đường đi được thật, giữ bên, khe chỉ chui khi nằm trên đường đi, `hold` khi đứng đây là tốt nhất. Mô phỏng 13/13 (bản cũ 6/13) |
| AL | **SEARCH bỏ cuộc ngay cạnh thùng dù người ngay đó** (07/10, mô phỏng) | Tới chỗ thấy cuối mà mũi lệch 37°, rồi pha quét đòi trống cả vòng 360° bán kính 0.53 m (cạnh thùng thì luôn sai) → `IDLE` | Pha "quay mặt về chỗ thấy cuối" + quét theo góc xoay được thật (`_can_turn`); có RSSI thì xoay dò hướng beacon (6.3) |
| AM | **Đích ngay trước vật mà đường thẳng "thoáng": xe đứng im mãi ở `FOLLOW`** (07/10, mô phỏng; chính là 13.12) | Camera ngửa cấu hình sai 3° → ước lượng người gần hơn → đích rơi trong lề an toàn trước thùng; `segment_blocked` chỉ xét hành lang tới đích nên không "bị chắn", DWA không tới được đích → v = 0 mãi | `goal_bad`: footprint đặt ở đích chạm `margin_hard` liên tục 1.5 s → dùng bản đồ (thường `hold` hoặc vòng qua). Trễ 1.5 s để người thứ hai bước vào đích rồi đi ngay thì xe vẫn chờ, giữ camera |
| AN | **Bám đường bản đồ bằng DWA: cắt góc rồi kẹt; vật đối xứng thì xoay qua lại mãi** (07/10, mô phỏng) | Chữ U: DWA đi chéo 45° tới điểm ngắm, góc mũi chạm mặt cánh chữ U, đường bản đồ bắt đầu bằng đoạn lùi mà DWA không lùi → đứng im. Ghế dài thẳng trước người: hai bên rẻ ngang nhau, mỗi lần lập lại đường lại đổi bên. Người đi vòng qua thùng: xoay về phía đường thì góc mũi quét vào góc thùng → DWA quẹo ngược, chui vào góc kẹt | `_nav_follow`: lệch > 75° thì xoay tại chỗ trước; DWA quẹo ngược đường 0.8 s → xoay/lùi ngắn; điểm ngắm phía sau → chờ 2 s rồi xoay rồi mới lùi; giữ bên vòng; `nav_lethal_m` 0.30 (= nửa bề ngang, trước 0.24 cho đường đi sát hơn thân xe) |
| AO | **Xe chỉ xoay theo người, không đi tới dù người cách ~3 m** (08/10, xe thật, lần chạy đầu `run_full.sh`) | Log: nguồn gần như luôn `camera+bbox`, `cam_dist_from = chan`, tracker báo **0.44–0.9 m**; LiDAR trong bag thấy cụm chân theo hướng camera ở 1.1–1.4 m (lúc enroll) và 2.0–3.7 m (lúc đi ra) → planner `ARRIVED` / `canh huong tai cho`. Suy ngược từ 274 mẫu: đáy bbox thấp hơn chỗ chân **17.2°** (ổn định theo khoảng cách: 1–1.6 m −19°, 1.6–2.4 m −16.5°, 2.4–4 m −17°; giả thuyết camera nâng 0.70 m không khớp) → camera thật **ngửa ~17°**, `camera_pitch_deg: 0`. Cổng chân 07/10 (dung sai 10° khi chưa học) loại cụm chân thật; học góc ngửa chỉ ±15° và chỉ khi đã qua cổng → kẹt. Bản trước 07/10 không có cổng nên không lộ. Mô phỏng `CAM_PITCH=18 CAM_PITCH_ERR=-18`: xe đứng yên, người đi xa tới 6.4 m | `_check_geometry` (6.2): đo sai lệch bằng cụm LiDAR gần nhất + kiểm chiều cao người, tự bù + WARN. `regress_follow.py` thêm nhóm `pitch`: **1/6 → 6/6**; toàn bộ 160/171 → **165/171**, chỉ 5 ca `pitch` đổi kết quả. **[CẦN XÁC NHẬN]** góc ngửa thật để sửa `camera_pitch_deg` |
| AP | **Camera thỉnh thoảng khoá nhầm người khác dù khác màu áo quần** (08–09/10, xe thật) | Log thật 08/10: 31 lần nhận lại, 16 lần sau đó chính hệ thống nghi người khác (bám nhầm 0.8–12 s). Mạng ReID của DeepSORT (`ckpt.t7`, Market-1501) cho người lạ trung vị **0.794** so với kho chủ (ngưỡng nhận lại 0.80, một người trong khung 0.85), tối đa 0.870; người ở xa AUC 0.80. Dữ liệu quay 09/10 (`~/reid_data/20261009_145308`): chạy lại toàn bộ logic → **khoá nhầm B 60% số khung khi chủ vắng**, đợt dài 8.9 s: kho người lạ trống (B chưa từng đứng cùng chủ) → B 0.856 > 0.85 → nhận nhầm → kho chủ học luôn B (ngưỡng học 0.70) → tự củng cố. Đổi YOLO không sửa được (YOLO chỉ báo có người) | **OSNet x0.5 (MSMT17) + YOLO26n 416 chạy bằng onnxruntime**, ngưỡng ReID đặt lại từ dữ liệu thật (nhận lại / học kho chủ > mức tối đa của người lạ 0.707). Khoá nhầm 8.6% → **0.1%** (2 khung lẻ = hộp dự đoán khi YOLO sót), bám chủ lúc đi tự do 47.5% → 59.4%; detector đúng số người 91.3% → 91.8% nhưng nhanh gấp 3; pipeline 2 lõi 70.6 → ~21 ms/khung. Chi tiết: README gói camera |
| U | **Xe lắc qua lại khi bám thẳng** | Người dùng báo: *"không bám thẳng theo người"*. Mô phỏng: `w` đổi chiều 42 lần / 22 s | Vùng chết hướng → 25 lần. Còn lắc nhẹ do góc cụm chân rung |

### 7.3 Lỗi môi trường / build

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `CMake Error: CMakeCache.txt directory is different` | Workspace bị di chuyển qua 4 vị trí và đổi user (`thaihoa` → `thach`). CMake ghi đường dẫn tuyệt đối vào cache. Chỉ ảnh hưởng package `ament_cmake` (`sc_mini`, `robot_simulation`), package Python build bình thường. | `rm -rf build install log` rồi build lại |
| `Can't access port /dev/robot_lidar` | `99-robot-usb.rules` dùng `KERNELS=="1-2.3"` — số hiệu cổng USB vật lý của **máy cũ** | Dùng đường dẫn `by-id` (xem mục 1) |
| `AMENT_PREFIX_PATH ... doesn't exist` | Shell còn giữ biến môi trường sau khi `rm -rf install` | Mở terminal mới |
| `-Wunused-variable` của `sc_mini` | Code gốc nhà sản xuất | Vô hại, bỏ qua |
| `colcon build --symlink-install` báo `error: [Errno 17] File exists: .../build/person_follow_nav/launch/follow_nav_real.launch.py -> .../install/...` (07/10) | Có lần build **không** `--symlink-install`: `install/` giữ **bản chép** launch/config thay vì symlink, lần build symlink sau không ghi đè được. **Hậu quả thật:** `install/person_follow_robot/.../identity_lock_kingsen.yaml` là bản chép lúc **30/09 11:44** (gói này build 30/09 12:48 không symlink) — trước hai commit hiệu chỉnh mắt cá (14:49, 17:01). `follow_nav_real.launch.py` đọc file trong `install/` nên **camera chạy FOV 62° kiểu pinhole, không phải mắt cá 114.4°** (góc người lệch giữa khung báo thiếu ~40%) ở mọi lần chạy launch từ chiều 30/09 tới 07/10. Mã Python thì không bị (chạy thẳng từ `src` qua symlink thư mục) | 07/10 đã sửa: xoá 3 file chép trong `install/person_follow_nav/share/.../{launch,config}` và `build/`+`install/` của riêng `person_follow_robot`, build lại `--symlink-install` → mọi launch/config là symlink về `src`. Kiểm tra: `ls -la install/<gói>/share/<gói>/config/` phải toàn `->`. **Luôn build với `--symlink-install`** |
| Driver in `BW-DR03 decoder connected` nhưng **không có `/odom`** (01/10, máy `thaihoa@th`) | `decoded_serial_node` chỉ đọc thụ động và **chỉ phát `/odom` khi khung xe gửi gói `cd eb d7`**. Mạch FTDI vẫn hiện trên USB kể cả khi khung xe tắt nguồn → cổng mở được, 0 byte tới | Bật công tắc nguồn khung xe / kiểm tra dây khung xe ↔ FTDI. Kiểm tra: `ros2 topic hz /odom` (20–50 Hz) |
| `sc_mini` chạy, không lỗi, nhưng **không có `/scan`**; kernel báo `usb 1-2.3: failed to send control message: -32`, mở cổng `Input/output error` (01/10) | Mạch CH340 của LiDAR **treo** (phải rút cắm lại). `sc_mini` chỉ gửi lệnh start `A5 F0` **một lần** lúc mở cổng và gửi stop `A5 F5` khi thoát. Lần treo xảy ra khi LiDAR bị bật/tắt nhiều lần liên tiếp trong vài phút (do thử nghiệm). **[CẦN XÁC NHẬN]** nguyên nhân gốc (nguồn/cáp/cổng của máy này) | Rút cắm lại cáp USB LiDAR đúng ổ cũ rồi launch lại; nếu tái diễn khi dùng bình thường thì đổi cáp/cổng hoặc dùng hub có nguồn. **06/10 lại treo 2 lần** (12:23, 15:22): `sc_mini` mở được cổng, đọc thẳng cổng bằng pyserial ra **0 byte cả khi gửi `A5 F0`**; rút cắm lại thì chạy (pyserial gửi `A5 F0` sau khi mở cổng ~2 s → ~9 kB/s). Gọi `ros2 service call /start_motor std_srvs/srv/Empty {}` rồi chờ ~10 s cũng đã giúp một lần. Kiểm nhanh: `ros2 topic echo /scan --once` (đợi đủ 10 s — `ros2 topic hz` 6–8 s dễ báo nhầm là không có) |
| Cắm hub `1a40:0101` (3 board RSSI) xong thì LiDAR **mất `ttyUSB`**: `/dev/serial/by-path/...:2.2:1.0-port0` không có, `by-id` `usb-1a86_USB_Serial` trỏ sang một board; interface CH340 của LiDAR có driver `usbfs` thay vì `ch341` (08/10, máy `thach@thachLG`) | Gói **`brltty`** (đọc chữ nổi cho người mù, Ubuntu cài sẵn): `/usr/lib/udev/rules.d/85-brltty.rules` dòng 87 khớp `1a86/7523` (CH340) khi có hub `1a40:0101` → udev bật `brltty-udev.service` → `brltty` chiếm thiết bị qua usbfs. Ba board vẫn có `ttyUSB` nhưng LiDAR thì không | `sudo systemctl stop brltty-udev.service` rồi `sudo apt remove brltty` (chỉ gỡ đúng gói đó; `ubuntu-desktop-minimal`/`orca` chỉ *khuyến nghị*) hoặc `sudo systemctl mask brltty-udev.service`; rút cắm lại USB LiDAR. `run_full.sh --check` báo HỎNG nếu `brltty` đang chạy hoặc có CH340 không gắn `ch341`. **[CẦN XÁC NHẬN]** người dùng đã gỡ và LiDAR đã có lại cổng |
| `person_reid_tracker` (YOLO + DeepSORT của camera) chạy **mã 29/09** dù `src` đã sửa (09/10, máy `thach@thachLG`) | Gói từng build **không** `--symlink-install` (30/09 12:48) → `install/person_reid_tracker/lib/.../site-packages/` là **bản chép** → bản sửa 30/09 17:01 (7.2-AI: YOLO sót 1 khung là mất sạch track) **chưa từng chạy trên xe**. Cùng loại lỗi với `identity_lock_kingsen.yaml` (dòng trên). `bw_dr03_ros2` cũng là bản chép nhưng khớp `src`; `robot_rssi_ros2` cũ không dùng | 09/10: xoá `build/` + `install/` của riêng `person_reid_tracker`, build lại `--symlink-install` → trỏ về `src`. Kiểm: `python3 -c "import person_reid_tracker.yolo_deepsort_pipeline as m; print(m.__file__)"` phải ra đường dẫn trong `build/` (symlink về `src`), không phải `install/.../site-packages/` |
| Beacon chỉ phát **~2 phút sau mỗi lần bật** rồi im hẳn; board vẫn chạy, `Khong thay beacon N s` liên tục (08/10, máy `thach@thachLG`) | Log 08/10: có beacon 11:51:11→11:53:20 (129 s) rồi mất tới hết lần chạy; lần sau 12:35:27→12:35:45, mất, có lại ~12:38:05→12:40:19 (~134 s), mất; lần sau nữa không có từ đầu cho tới khi người dùng bật lại (12:51 có −73…−79 dBm, 18–20 mẫu/s). Firmware beacon (`robot_rssi/src/s3_esp`) chỉ khởi động BLE rồi phát mãi, không có gì tự dừng; board quét lọc đúng UUID → **nguồn beacon**. **[CẦN XÁC NHẬN]** beacon cấp nguồn bằng gì — nghi **sạc dự phòng tự ngắt** khi dòng tải nhỏ (BLE vài chục mA, dưới ngưỡng giữ nguồn của nhiều loại) | Dùng chế độ dòng nhỏ của sạc dự phòng (nhiều loại: bấm 2 lần nút), hoặc pin không tự ngắt (LiPo/18650 + mạch tăng áp, hay 3 pin AA vào chân 5V). `run_full.sh` mục 4b đọc thử 3 board 6 s trước khi launch (không thấy beacon → CẢNH BÁO, không chặn); `rssi_scanner_node` giờ cảnh báo cả khi **chưa từng** thấy beacon từ lúc khởi động (trước đây im lặng) |
| `New publisher discovered on topic '/scan', offering incompatible QoS ... RELIABILITY` | `sc_mini` phát `/scan` kiểu `SensorDataQoS` (best effort); node đăng ký mặc định (reliable) không nhận được gói nào | Đăng ký `/scan` bằng `qos_profile_sensor_data` (đã sửa trong `rssi_rotate.py` 01/10; tracker/planner vốn đã đúng) |
| Driver báo liên tục `Serial error: [Errno 5] Input/output error`, script báo `mat /odom`; kernel: `usb 1-2.1: clear tt 1 ... error -71` rồi `USB disconnect` cả hub (06/10, 2 lần trong 15 phút) | Hub USB nhỏ `1a40:0101` (cổng 2.1) gánh **cả 3 board RSSI lẫn FTDI khung xe** (2.1.4). Cả 2 lần hub reset đều **đúng lúc động cơ bắt đầu xoay** (dòng khởi động → sụt áp / nhiễu qua dây FTDI). Driver không tự mở lại cổng. **Nguy hiểm:** mất cổng thì lệnh dừng không tới khung xe — LiDAR cho thấy xe **xoay tiếp ~0.87 rad/s thêm ~3 s** (khung xe tự giữ lệnh cuối ~3 s; `cmd_timeout` 1 s của node chỉ có tác dụng khi node còn ghi được cổng) | FTDI khung xe cắm **thẳng một cổng laptop riêng**, 3 board RSSI qua hub **có nguồn ngoài**; lõi ferrite trên dây FTDI; kiểm pin. Sau đó launch lại T1. **06/10 15:28 đã làm:** FTDI ở `1-2.2` (cổng riêng, `by-id` không đổi), hub nhỏ `1-2.1` chỉ còn 3 board (`by-path` 2.1.1–2.1.3 không đổi) — hub nhỏ vẫn báo vài lỗi `-71` lúc cắm lại (bản thân hub/dây kém). **Thử 06/10 15:38** (`calibrate.launch.py` + `measure_speed.py 0.0 --w ±0.8 / ±0.9 --sec 3`, 4 lần xoay tại chỗ, 2 lần cuối đổi chiều liên tiếp ở 0.9 rad/s như xoay dò): **0 lỗi USB, 0 lỗi serial**, `/odom` liên tục; w thật/lệnh 0.95–0.98 (khớp `max_angular` 2.46). **[CẦN XÁC NHẬN]** chưa chạy lâu (buổi chiều lỗi 2 lần trong ~15 phút) — xem lại kernel log sau lần chạy bám kế tiếp; nguyên nhân gốc (nguồn hay nhiễu); driver tự nối lại = sửa gói `bw_dr03_ros2` (cần người dùng đồng ý) |

---

## 8. TRẠNG THÁI HIỆN TẠI

### Đã xong

- **Giai đoạn 0** — build sạch, 8/8 package
- **Giai đoạn 1** — hiệu chỉnh LiDAR hoàn tất, toàn bộ giá trị ở mục 3 đã xác nhận
- **Giai đoạn 2** — đo footprint, xác nhận tâm quay
- **Giai đoạn 3** — né vật cản không camera, **nghiệm thu 6/6 trên xe thật (17/09)**, không va chạm

### Giai đoạn 3 — kết quả

Trước đây người dùng báo *"lidar không xoay mà xe chỉ chạy thẳng"* — lúc đó xe còn chạy nhanh gấp 2.16 lần lệnh (lỗi 7.2-P).

**Kết quả 17/09 trên xe thật** (sau khi sửa `max_linear`):
- `preflight.sh`: `/scan` 9.9 Hz, `/odom` 50.2 Hz, đúng 1 nguồn ghi `/cmd_vel`, `/follow/stop` sẵn sàng. Cảnh báo "n_obstacles=0" là **báo sai** do `ros2 topic echo` cắt chuỗi — đã sửa bằng `--full-length`.
- `diagnose.py`: LiDAR **đang quay** (33% tia thay đổi giữa hai vòng, 9.8 Hz, 66% tia hợp lệ); planner thấy 109 vật cản; bộ lọc bỏ 19/227 điểm; độ thoáng quanh footprint 0.838 m. `ros2 node list | grep -c sc_mini` = 1.
- `fake_target.py 1.2`: xe dừng trước vật cản, không va chạm (3.1 đạt).
- `fake_target.py 2.5`, vật cản **giữa đường** cách mũi ~1 m: xe **né ổn** (người dùng xác nhận). `diagnose.py` lần có vật cản: vật gần nhất trước mặt 1.12 m tại +2° (= 0.98 m từ mũi, khớp thực tế → kiểm chứng thêm hiệu chỉnh LiDAR); bộ lọc bỏ 22/220; `preflight.sh` đã sửa đọc đúng `n_obstacles=135`. (Lần `diagnose.py` đầu chưa đặt vật cản nên chỉ thấy vật ở 2.20 m.)

- Bảng 3.2–3.7 trong README (người dùng chạy 17/09): 3.2, 3.3, 3.4, 3.5, 3.7 **đúng như mong đợi**. 3.6 (hai thùng cách 0.40 m, xe cách 1 m): **không** chui qua khe; hai bên thoáng thì xe vòng ra ngoài thay vì `BLOCKED`; trong hành lang hẹp (không vòng được) thì `BLOCKED`. Đây là **đúng thiết kế**: `_choose_heading` quét ±100°, chỉ `BLOCKED` khi không hướng nào đi được (`phi is None`), và `fake_target` ở khung `base_link` nên mục tiêu xoay theo xe. Kỳ vọng 3.6 cũ trong README ghi thiếu — đã sửa.

### Đang làm

**Giai đoạn 4 — ghép camera.** Kiểm tra trước khi cho xe bám người:
1. **Tốc độ ReID trên CPU** (xem 13.7): chạy `follow_nav_real.launch.py`, xem log `identity_lock_node` dòng `Models loaded; ...`, đo `ros2 topic hz /person_reid/target` và `ros2 topic hz /cmd_vel` (phải ≥ 10 Hz khi YOLO đang chạy — mục 12 "Watchdog driver").
2. **Chưa bật `/follow/enable`**: enroll, đứng cách xe 2 m, `source` phải là `camera+lidar` (lệnh giám sát mục 10).
3. Rồi mới chạy kịch bản 4.1–4.5 trong README.

**09/10 — nâng camera: YOLO26n + OSNet x0.5 (onnxruntime), chống khoá nhầm người, chuẩn bị mini PC i5-5200U (7.2-AP).** Người dùng xác nhận 08/10: **chạy tất cả kịch bản bám + né đã rất đúng**; còn camera lâu lâu khoá nhầm người, hỏi nâng YOLOv5 → YOLOv8. Phân tích (13.43): lỗi do ReID, không do YOLO. Người dùng đồng ý làm theo đề xuất + đổi OSNet + nâng YOLO26n nếu đáng. Đã làm: công cụ ghi dữ liệu thật (người dùng + 1 người nữa quay 8 pha; lần đầu không theo pha → quay lại), đánh giá 6 mạng ReID + 6 cấu hình detector, chạy lại toàn bộ logic (`replay_identity.py`); tích hợp onnxruntime vào pipeline (tuỳ chọn, để trống = PyTorch như cũ), tắt spin của onnxruntime; cài `onnxruntime 1.31 --no-deps` cho Python của ROS (numpy vẫn 1.26); venv riêng `~/.venvs/yolo` để xuất ONNX; build lại `person_reid_tracker` kiểu symlink (7.3); đổi `identity_lock_kingsen.yaml` sang cấu hình mới (giá trị cũ ghi kèm). Node thật chạy với video từ đoạn quay: 15 Hz, 20–23 ms/khung (2 người, 2 luồng). **[CẦN XÁC NHẬN] chưa chạy trên xe** (13.44).

**08/10 — chuẩn bị chạy thật toàn bộ (camera + LiDAR + RSSI) trên máy `thach@thachLG`.** Người dùng cắm thêm hub 3 board ESP32, các dây khác giữ nguyên, nhờ tạo lệnh chạy đầy đủ + kịch bản thử. Đã làm: `~/rssi_env.sh` (cổng, mục 1), `scripts/run_full.sh` (kiểm tra rồi launch, log `run_logs/run_*.log`), `scripts/watch_follow.py` (theo dõi, log `run_logs/watch_*.txt`), README mục "Chạy toàn bộ bằng một lệnh + kịch bản thử theo bậc" (B0 hình học camera chưa bật bám → B1 bám cơ bản → B2–B4 vật thấp → B5 hồi quy cửa → B6 người thứ hai → B7 RSSI tìm người khuất → B8 đối chứng không RSSI). Phát hiện: **brltty chiếm LiDAR** (7.3) — người dùng phải gỡ bằng sudo trước. Ba board tự báo đúng A/B/C, chạy được, lúc kiểm tra beacon tắt (0 mẫu). **Chạy thật 11:51–11:57** (`run_logs/run_1008_115109.log`, `watch_1008_115251.txt`, bag `~/bags/follow_1008_115318` — chỉ có `/scan /odom /cmd_vel` vì lệnh bag dán bị cắt → thêm `run_full.sh --bag`): người dùng gỡ brltty xong, LiDAR chạy. **Xe chỉ xoay theo người, không đi tới dù người cách ~3 m** — camera ngửa ~17° mà cấu hình 0° (7.2-AO), đã sửa bằng `_check_geometry`. Beacon có tín hiệu tới 11:53:20 (−66…−71 dBm, 18 mẫu/s mỗi board) rồi mất hẳn (22 lần `Khong thay beacon`) — board vẫn mở cổng bình thường → **[CẦN XÁC NHẬN]** beacon tắt nguồn (pin dự phòng tự ngắt khi dòng nhỏ?). Camera 25 ↔ 12.5 fps như 13.32. **Chạy lại bản sửa 12:35–12:51** (`run_1008_123525.log`, `run_1008_124751.log`; bag `follow_1008_123729`, `_124249` lại chỉ có `/scan /odom /cmd_vel` — lệnh tự gõ, không phải `run_full.sh --bag`; ghi thử từ hệ thống thật thì đủ topic JSON): người dùng xác nhận **camera khá đúng, bám gần đúng khoảng cách, qua vật cản như kịch bản ổn**; còn RSSI không tìm lại được người vì **beacon chỉ phát ~2 phút sau mỗi lần bật** (7.3). Hai lần beacon có tín hiệu lúc mất người (12:38:17, 12:51:00): `SEARCH` đi tới chỗ thấy cuối → quay mặt → bắt đầu `xoay do huong beacon` → xoay ~100° thì camera đã thấy lại người (0.65 m; 3.6 m) → bám tiếp — **chưa lần nào cần tới hướng beacon** (cần xoay ≥ 250°, người phải khuất camera suốt lúc xoay).

**07/10 — gộp RSSI (nhánh `rssi-thaihoa`, commit `6d0aa54`) + kết hợp camera/RSSI + vật thấp che chân.** Người dùng xác nhận bản cửa 29/09 (`59a42bb`) **qua cửa thành công trên xe thật**. Báo thêm (chỉ camera, chưa RSSI): người đứng sau thùng 50×30 cm cao ~30 cm, cách ~2 m, có lúc một bên có thùng thứ hai cách 50 cm (bên kia trống) → có lần xe đi vào phía có thùng rồi kẹt; lúc kẹt log báo người cách ~1 m. Đã làm:
- **Tracker** nhận mặt thùng là người (7.2-AJ) → hình học dọc camera (6.2); camera gửi thêm góc cao bbox.
- **Planner**: bản đồ lưới + tìm đường (7.2-AK, AM, AN; 6.3), SEARCH viết lại có pha quay mặt + RSSI (7.2-AL), sửa 13.34 / 13.36 / 13.37.
- **Kết hợp camera + RSSI**: RSSI **chỉ để tìm lại** khi mất người (SEARCH), camera/ReID xác nhận đúng người, lúc bám dùng camera + LiDAR. `follow_nav_real.launch.py start_rssi:=true lidar_port:=$PL ports:="$PA,$PB,$PC"` chạy 2 node RSSI mới (thay `rssi_serial_node` cũ). `rssi_follow.launch.py` tắt RSSI trong planner (`rssi_follow.py` tự lo).
- **Phát hiện khi build**: `install/` giữ bản chép cũ của `identity_lock_kingsen.yaml` (30/09 11:44) → mọi lần chạy `follow_nav_real.launch.py` từ chiều 30/09 tới 07/10 camera dùng FOV 62° pinhole chứ không phải mắt cá 114.4° (7.3). Đã build lại thành symlink. Các lần thử 30/09 chiều – 07/10 (qua cửa, vật thấp) chạy với góc camera sai — **[CẦN XÁC NHẬN]** người dùng có chạy camera bằng cách khác không.
- Mô phỏng `regress_follow.py --base 6d0aa54` (165 kịch bản có tiêu chí): **159/165 ĐẠT, bản cũ 131/165**. Vật thấp 13/13 (5/13), kịch bản khó 16/17 (3/17), cửa 108/112 (103/112: cửa trước + bên hông 10/10 (7/10), gần giữa cửa nhiễu 1 cm 30/30 (30/30), nhiễu 2 cm 26/30 (25/30), lưới chui cửa 30/30 (30/30), đã sát khung 12/12 (11/12, có 1 lần chạm)), người chen 20/21 (20/21), RSSI tìm người khuất 2/2 (bản cũ không có). Người chen giữa / né xong rẽ: giữ camera 100% (6.3). Test offline 3/3 ĐẠT, `test_reacq.py` 10/10. **[CẦN XÁC NHẬN] chưa chạy trên xe.**
- `sil_rssi.py` (chế độ `rssi_follow` — cũng chạy planner mới): lần đầu 36/37, `b_phong3b` bám nhầm 19.5 s vì lần dò đầu sai +39°; chạy lại 2 lần mỗi bản (mới và 6d0aa54, seed 5 và 7): **4/4 ĐẠT**, chỉ số gần như trùng (tới 3/3 chỗ sau 44 s / 79 s, cách người TB 1.23 m) → do sai số RSSI phát lại ngẫu nhiên, không do planner. Chạy lại cả bộ chính với code cuối: **37/37 ĐẠT** (112 lần dò: sai TB 13.0°, lớn nhất 43°, 98 % trong ±30°).

**Kết quả 17/09:** bước 1–2 đạt — `/person_reid/target` 8.0 Hz, `/cmd_vel` ~14.8 Hz (ReID chạy CPU), `source = camera+lidar`. Chạy thử bám người, người dùng báo 3 vấn đề: xe không bám thẳng (lắc), rẽ theo không kịp, và khi `predicted` xe đi thẳng thay vì quay về hướng thấy người lần cuối. Đã sửa tracker + planner (lỗi 7.2-Q…U), kiểm bằng `scripts/sim_follow.py`. **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**

**Kết quả 22/09 (xe thật, sau bản 17/09):** người dùng xác nhận bám người "cải thiện rất tốt"; có `SEARCH` → `FOLLOW` khi mất rồi thấy lại. Lỗi còn: **người thứ hai chủ động bước vào giữa** thì xe đa số lần chạy thẳng tới chân họ dù camera vẫn thấy người đang bám; họ bước ra thì bám tốt lại. Log nguồn lúc đó nhảy `lidar_track`/`camera+bbox`. Đã sửa (7.2-V), kiểm bằng `sim_follow.py` kịch bản `chan_giua`/`cat_ngang`/`chan_sat`. **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**

**29/09 (xe thật, bản `bc3dcdf`):** người dùng báo ba việc: (1) cửa 0.81 m khi người rẽ vào: xe gần như ở giữa cửa, thực tế lọt, nhưng đứng im với `note` "qua khe theo truc (lech -3..-5 do)", lúc qua lúc không — muốn sửa tuyệt đối; (2) xoay theo người hơi chậm so với tốc độ đi bình thường; (3) người thứ hai chủ động bước vào: lúc né, lúc bò tới sát chân họ. Cả ba tái hiện được và đã sửa (7.2-AD, AE, AF). Mô phỏng thêm tường dày (`WALL_T`), nhiễu LiDAR (`LIDAR_NOISE`), kịch bản người thứ hai tham số hoá (`chen`) và người đi 1 m/s (`vong_nguoi_*`). **07/10 người dùng xác nhận: đã chạy thật bản này, đi qua cửa thành công** (log `note_29.txt`).

**30/09 — RSSI, bắt đầu làm lại (chưa tích hợp).** Kiểm tra code cũ: offset B/C là số đoán (-5 dB → mô phỏng lệch hướng TB 30°), slave đưa lại cùng một mẫu vào bộ lọc mỗi 50 ms, A lọc 1 lần còn B/C lọc 2 lần và quét khác tham số (trái/phải trễ khác nhau), `rssi_serial_node` xoá bộ đệm = vứt dữ liệu mới, tracker đưa điểm RSSI qua `filter.update()` nên nhiễu hướng thành **vận tốc giả** (~0.9 m/s/nhịp), và dùng mỗi mẫu 2 lần. Giới hạn vật lý (mô phỏng, giả định thân xe che 10 dB, nhiễu 4 dB): trung bình 5 mẫu vẫn sai ~12–15° → RSSI **không** dùng để bám khi camera còn thấy, chỉ để chọn đúng chiều quay khi người ra khỏi khung và giữ hướng khi bị che lâu. Kế hoạch R0 (đo) → R1 (firmware mẫu thô, USB — **đã viết**, biên dịch được, chưa nạp) → R2 (hiệu chỉnh mẫu anten bằng cọc + LiDAR) → R3 (bộ lọc xác suất trong khung odom) → R4 (tracker: chỉ sửa hướng, không đụng vận tốc) → R5 mô phỏng → R6 xe thật. **R0 đo trên bàn (30/09, máy `thaihoa@th`, cổng trong `~/rssi_env.sh`):** firmware `scanner` chạy tốt, ~20 mẫu/s mỗi board, không mất mẫu. **Beacon cũ phát yếu:** `setPower()` kiểu `DEFAULT` không áp cho quảng bá → sửa `s3_esp` thành `ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV` (+9 dBm), đã nạp; ở 30 cm B/C −47 dBm. **Offset board** (xoay vòng 3 vị trí quanh beacon 30 cm, mô hình board+vị trí dư 0.3 dB): so với trung bình A −6.9, B +1.4, C +5.4 dB (A thấp hơn C ~12 dB; đoạn đo bị xáo trộn cho ±1–2 dB). Anten mạch in của A **rất nhạy hướng/vật che** (cùng khoảng cách, đổi chỗ đặt: −82 ↔ −57 dBm). **Ba kênh quảng bá chênh tới 13 dB** ở một vị trí (phân bố hai cụm) → bộ ước lượng phải lấy trung bình qua cửa sổ đủ dài để đi hết 3 kênh, không dùng trung vị. Offset này đổi khi lắp lên xe — R2 hiệu chỉnh lại trên xe. **Trên xe (30/09, S2–S5):** chỉ số trái/phải `C − B` (đã trừ offset bàn) = **+11.6 dB** khi beacon ở bên trái 90°, **−13.1 dB** bên phải 90°, ≈ 0 thẳng trước → B/C lắp **đúng bên**; cửa sổ 0.25 s đã đúng 100% ở ±90°, thẳng trước báo nhầm trái/phải 3% (cửa sổ 0.5 s, ngưỡng 4 dB). Người đeo sau thắt lưng quay lưng về xe: `C − B` lệch −3.6 dB (ngưỡng 4 dB quá sát). Trước/sau dựa vào A **kém tin** (hai lần đo cùng chỗ trước mũi lệch nhau ~5 dB). Người quay mặt về xe: A tụt 19 dB, B/C chỉ 6–8 dB → thân người che **không đều** giữa các board. 0.8 m yếu hơn 1.5 m → RSSI **không** dùng đo khoảng cách. Đeo sau thắt lưng quay lưng: mạnh nhất (−63 dBm ở 1.5 m) và ít nhiễu nhất; cầm tay nhiễu gấp 2–3 lần. **Người đeo beacon ở ±45° (ngay ngoài FOV camera) — RSSI KHÔNG phân biệt được trái/phải:** `C − B` = −5.7 dB (trái 45°) và −5.1 dB (phải 45°), gần như bằng nhau; người thẳng trước −3.7 (1.5 m) và −5.2 (3 m) → có **độ lệch về phải ~−4..−6 dB** khi người đeo (giá đỡ thì ≈ 0). Ngưỡng 8 dB, cửa sổ 1 s: trái 45° bị báo **PHẢI** 30% số cửa sổ, người trước 3 m báo phải 32%. Chỉ ở ±90° (giá đỡ) mới rõ ±12 dB. **Hệ quả:** không dùng RSSI để chọn chiều quay lúc người vừa ra khỏi khung hình — sẽ quay SAI chiều. Ở 3 m A còn −79 dBm, 16.5 mẫu/s (đủ tầm). Hướng cải thiện phần cứng chưa thử: tấm phản xạ kim loại phía trong anten B/C, hoặc anten định hướng (patch) chĩa ra hai bên. **[CẦN XÁC NHẬN] chưa đo người ở ±90°, chưa đo khi người đang đi (phản xạ thay đổi, có thể tốt hơn đứng yên).** **Hướng mới — xoay dò hướng (30/09):** khi phải TÌM LẠI người, xe xoay tại chỗ; mức từng board lên xuống theo yaw như sóng sin, **pha** chỉ hướng người trong khung odom — không phụ thuộc offset board và độ lệch do người đeo. `scripts/rssi_rotate.py` (ghi + `--analyze` + `--calib`), an toàn: từ chối khi thiếu `/odom`, có node khác ghi `/cmd_vel`, thiếu `/scan` hoặc vật < 0.6 m. **Mô phỏng offline `scripts/sim_rssi.py` (30/09):** mô hình tái tạo cả 7 số đo `C − B` thật (chế độ `kiemtra`), phản xạ Rician riêng 3 kênh; phía sau lưng chưa đo nên chạy 2 biến thể. So 5 thuật toán: `tinh` (C−B đứng yên) chỉ ~30–40% trong ±30°; xoay dò sóng hài (`hai_cal`) và so khớp **mẫu hình dạng** (`mau`, lấy từ một lần hiệu chỉnh ở chỗ khác) đều ~100%, sai trung vị 6–8°; chỉ `mau` còn đúng khi xoay thiếu vòng (270°/180°: 100%/98%). Người **đi vòng quanh xe lúc xe xoay** làm hỏng mọi cách giả định đứng yên (~10%) → `mau_dong` (tìm thêm tốc độ góc người) 90% nhưng đứng yên tụt 92%; bản **kết hợp `kh5`** (chỉ tin mô hình người đi khi khớp tốt hơn ≥ 5%) cân bằng: đứng yên 97–100%, đi vòng 78%. Vòng kín "dò → đi" (người ẩn 2.5–6 m, giả định camera chỉ thấy trong 2 m): SEARCH hiện tại 0–5%, RSSI tĩnh 54%, **`kh5`, xoay 0.9 rad/s, mỗi lần dò 270°, đi 2 m: đứng yên 100% (TV 25.8 s), người đi 0.3 m/s 77%** — trần do xe 0.22 m/s chậm hơn người. Camera thấy 3 m: 100%/96%; 5 m (phòng trống): chỉ xoay 1 vòng đã 68–74%, RSSI 100%/100% (~4 s). Đã sửa `rssi_rotate.estimate`: một board yếu không còn bỏ cả ước lượng (người quay mặt về xe 17% → 98%). `rssi_rotate.py --calib` giờ lưu mẫu `mau_rssi.json`; `--analyze` in cả `hai_cal`, `mau`, `kh5`. Mô phỏng chưa kiểm: nhòe ảnh camera/ReID khi xoay 0.9 rad/s, vật cản khi đi. **Xoay dò trên XE THẬT (01/10, máy `thaihoa@th`, `rssi_logs/quay_*.csv`):** hiệu chỉnh 2 vòng 0.4 rad/s (thật 0.36), người trước mũi 1.5 m → mẫu đúng như lý thuyết (A mạnh phía trước nhưng nhìn lệch trái +35°; B mạnh phải-sau; C mạnh trái-sau; đỉnh–đáy 12–16 dB); vòng 1 làm mẫu đoán vòng 2 sai 0–5°. Ba vị trí khác ở 2 m, 1 vòng lệnh 0.9 rad/s (thật 0.82, ~7.8 s): **+90° sai +8/−13/−13°** (`hai_cal`/`mau`/`kh5`), **180° sai +3/+11/+11°**, **−45° sai −36/−51/−46°** → đạt 2/3 trong ±30°. Ở −45° cả 3 board cùng chỉ lệch ~35° (đồng thuận 0.98) — chưa phân biệt được do người đứng lệch vạch hay sóng phản xạ: hướng thật lúc đó **đặt bằng mắt**, và **beacon bị thân người che ở mọi lần** (TB A −87, B −83, C −82 dBm, yếu hơn LOS ~15 dB). Kiểm tra chéo mẫu giữa các lần đo lệch 10–50° — kém mô phỏng; `hai_cal` ≥ `mau` trên 3 lần này; xoay thiếu vòng (180°) không tin được. Mẫu số nhỏ, chưa kết luận thuật toán. `rssi_rotate.py` từ 01/10: đo **hướng thật của người bằng LiDAR** trước khi xoay (cụm gần nhất ±40° quanh hướng nhập; thẳng sau xe là cung mù), in mức tín hiệu lúc đứng yên + cảnh báo beacon bị che (< −78 dBm), lưu alpha trong `mau_rssi.json`, `--repeat N`, kiểm tra khoảng trống tính từ tâm quay (0.53 m). **LiDAR trên máy này tự ngừng gửi dữ liệu** (2 lần 01/10: `sc_mini` còn chạy, đọc 0 byte/s, kernel không báo lỗi, `/start_motor` không cứu được → phải rút cắm lại) — xem 7.3. **Hiệu chỉnh lần 2 (`q2_cal000`, beacon nhìn thẳng xe: B −64, C −62 dBm):** độ tương phản tăng gấp đôi (đỉnh–đáy B 12 → 28 dB, C 13.5 → 18.5 dB); mẫu B và C **lặp lại được** giữa hai lần hiệu chỉnh (tương quan 0.80/0.84, không lệch góc), mẫu A thì không (0.04) — A đóng góp ít. Hướng nhìn đo được: A +9, B −110, C +116 (gần hình học 0/−114/+114). **Bẫy đo hướng thật bằng LiDAR:** người vận hành cũng là người đeo beacon, lúc bấm Enter còn đứng cạnh laptop trên xe → quét ngay thì bắt nhầm đồ đạc (lần này −33° thay vì 0°). `rssi_rotate.py` giờ **đếm ngược `--delay` 8 s** cho người đi tới vị trí rồi mới quét, nhận người là **vật mới xuất hiện so với nền lúc đầu** (không bị đồ đạc lừa; còn sót: vị trí đích nằm đúng hướng chỗ đứng cạnh laptop nhưng xa hơn thì không thấy → tự dùng hướng nhập), các lần `--repeat` sau bám theo vị trí đã biết; `--no-lidar-truth` khi phân tích để dùng lại hướng nhập. **Vòng đo 2 trên xe (01/10, `rssi_logs/q2_*`, 11 lần: 4 vị trí ~2 m × 2–3 lần lặp, beacon nhìn thẳng xe, 1 vòng 0.82 rad/s ≈ 7.8 s):** so với hướng nhập (đặt bằng mắt, đã bù góc xe lệch giữa các lần lặp) — so khớp mẫu `mau` **11/11 trong ±30°** (TB 14.5°, lớn nhất 24°), `kh5` 11/11 (15.3°/24°), `hai_cal` 11/11 (14.6°/29°), bỏ board A (`mau_BC`) 11/11 (14.0°/24°). Xoay thiếu vòng: 270° đầu (5.8 s) `mau` 11/11 (lớn nhất 28°), `kh5` chỉ 8/11; 180° (3.9 s) `mau` 10/11 (lớn nhất 44°). **Độ lặp lại rất tốt:** 3 lần lặp cùng vị trí, `mau` chỉ về cùng một hướng trong khung odom, lệch nhau 1–7°. Sai số TB 14° gồm cả sai số đặt người bằng mắt. → Trên dữ liệu thật chọn **`mau` (so khớp mẫu), quét 270–360°**; `kh5` chỉ đáng dùng khi người đang đi (chưa đo thật). **Bẫy đã gặp:** sau mỗi vòng xoay xe dừng lệch **+17..+21°** so với lúc đầu (dừng theo odom + trớn) → hướng người so với mũi xe đổi theo; script cũ không bù nên in sai số lần lặp 2, 3 lớn giả. `rssi_rotate.py` giờ đo góc lệch đó bằng **so khớp hai đám điểm LiDAR** (`scan_heading_shift`) và ghi `truth_deg`/`truth_src`/`heading_shift_deg`; bản gốc JSON của vòng 2 ở `rssi_logs/goc_q2/`. Nhận người bằng LiDAR ở vòng 2 **thất bại cả 11 lần** vì ngưỡng điểm tính trên từng chân (ở 2 m mỗi chân 2–3 tia, LiDAR mất ~50% tia) → đã đổi sang tính trên cả người (chưa kiểm trên xe). Chưa đo: người đang đi, xa hơn 2 m, phòng khác, beacon bị che (vòng 1 cho thấy kém hơn).

**01/10 — hai node RSSI + script thử "bám theo bằng RSSI" (chưa chạy xe, chưa đưa vào planner).** Người dùng yêu cầu: *"viết 2 node, nếu tốt thì test thử RSSI bám theo chủ thể"*, và trước đó: không đụng tracker/planner, chỉ sửa local không đẩy GitHub — đã giữ đúng.

- **Thuật toán** tách ra `person_follow_nav/rssi_df.py` (thuần numpy; node, `rssi_rotate.py`, `sim_rssi.py` dùng chung một mã). Lớp `RotationDF` gom mẫu + odom theo thời gian thực, **tự xoá khi xe rời chỗ xoay > 0.30 m**, và chỉ báo `valid` khi: đã xoay ≥ 250° trong 14 s gần nhất **và** (`corr ≥ 0.45` & `margin ≥ 0.10`) **hoặc** (`corr ≥ 0.35` & `margin ≥ 0.25` — khớp hơi kém nhưng không mơ hồ). `corr` = tương quan sóng đo được với mẫu; `margin` = hướng tốt nhất hơn hướng tốt nhì (lệch ≥ 40°) bao nhiêu phần. Ngưỡng lấy từ **182 cửa sổ quét (250–360°) cắt từ 14 lần đo thật**: luật trên nhận 152, sai TB 15.0°, lớn nhất 44°, **không lần nào > 45°**; `margin < 0.10` thì 6/17 sai > 45° (toàn lần beacon bị thân người che). Nới thành `corr ≥ 0.35 & margin ≥ 0.20` thì lọt một lần sai 61° — đừng nới.
- **`rssi_scanner_node`**: đọc 3 board (`rssi_io.py`: tự nối lại khi rút/cắm, **không mở cổng đang bị tiến trình khác giữ** — tránh giành dữ liệu LiDAR, không xoá bộ đệm) → `/rssi/raw` + `/rssi/status`. **`rssi_bearing_node`**: `/rssi/raw` + `/odom` → `/rssi/bearing`, service `/rssi/reset`. Hai node **chỉ lắng nghe, không bao giờ ghi `/cmd_vel`**; muốn có hướng thì ai đó phải cho xe xoay tại chỗ. Tham số `config/rssi.yaml`, mẫu `config/rssi_template.json` (= `rssi_logs/mau_rssi.json`, hiệu chỉnh lại khi đổi chỗ lắp board/anten). Phát lại 14 lần đo thật qua `RotationDF`: trước khi xoay / mới xoay 180° → không hợp lệ; hết vòng 13/14 hợp lệ, mọi lần hợp lệ sai ≤ 24°; lần sai 46° (vòng 1, beacon bị che) bị loại vì "mơ hồ" (margin 0.04). Chạy node thật với 3 pty phát lại một lần đo: ra đúng hướng như tính offline (−80°, −20°, +155° so với −80°, −20°, +150°); rút/cắm lại cổng thì tự đọc tiếp.
- **`scripts/rssi_seek.py`** (script THỬ, tự ghi `/cmd_vel` → chỉ chạy với `calibrate.launch.py`): lặp **DÒ** (xoay ≥ 330° tới khi `/rssi/bearing` hợp lệ, tối đa 760°) → **QUAY** về `bearing_odom_rad` → **ĐI** thẳng tối đa 1.2 m ở 0.18 m/s → dừng khi vật trước mũi < 0.70 m ("TỚI"), rồi chờ; vật trước mũi biến mất 3 s (so với khoảng cách lúc tới chứ không đòi "trống hẳn" — phòng nhỏ thì sau lưng người là tường) hoặc hết `--hold-sec` 25 s thì dò lại. **Không né vật cản**; LiDAR chỉ để dừng, đổi tia bằng đúng `scan_to_base_points` + `self_filter_mask` của planner, gộp các vòng quét trong 0.45 s (mỗi vòng mất ~40 % tia). Luật an toàn: chỉ xoay khi quanh tâm quay trống > 0.53 m; hành lang tiến rộng 0.90 m; không bao giờ lùi; `/scan` cũ quá 0.6 s hoặc mất `/odom` → dừng, chờ 5 s rồi thoát; mất beacon → dừng, chờ 20 s; thấy publisher khác trên `/cmd_vel` → dừng hẳn; Ctrl-C/SIGTERM → gửi lệnh dừng. Vì sai số hướng 25° ở 2 m = lệch ngang 0.85 m: gặp vật sau khi đã đi > 0.7 m từ chỗ dò, hoặc vật chỉ chạm **mép** hành lang → dò lại ngay tại đó để xác nhận (tối đa 2 lần, rồi báo "BỊ CHẶN"); vật sắp lướt sát bên hông (< 0.62 m từ tâm quay) → dừng sớm khi còn đủ chỗ xoay, dò lại. Ghi `rssi_seek_*.jsonl` (sự kiện + vết 2 Hz + đám điểm LiDAR lúc dò xong) và `.csv` (mẫu thô + odom, cùng định dạng `rssi_rotate.py`).
- **Mô phỏng vòng kín có ROS `scripts/sil_rssi.py`** — chạy **đúng** 2 node + `rssi_seek.py` với thế giới giả trong miền ROS riêng (`ROS_DOMAIN_ID` 60..; tiến trình thế giới từ chối chạy ở miền < 50 nên không lệnh nào tới được xe thật): mô hình driver BW-DR03 lấy từ `decoded_serial_node` (chia % nguyên, tối thiểu 5 %, vùng chết, `cmd_timeout` 1 s) + trễ 0.08 s + quán tính bánh 0.25 s (khớp trớn ~18° đo được khi dừng xoay), `/scan` 10 Hz kiểu SensorDataQoS mất 40 % tia, 3 board = 3 pty. RSSI **phát lại mẫu đo thật** (11 lần `q2_*`, lấy theo góc người nhìn từ xe; mỗi lần xe/người đổi chỗ thì bốc một lần đo khác) với **mẫu hiệu chỉnh thật** → sai số hướng giống xe thật (mô hình `sim_rssi` lạc quan hơn: TB ~9°); đồ đạc trong phòng lấy từ **vòng quét LiDAR thật** `rssi_logs/q2_m045_1.json`. **Kết quả (16 kịch bản, 2 lượt): tất cả ĐẠT.** Người đứng yên 2–2.5 m (phòng thật 4 vị trí + phòng trống): tới cách người 0.88–0.98 m sau **27–44 s** (2–3 lần dò), 4.4 m: ~60 s (4 lần dò); bám theo 3 chỗ liên tiếp: tới đủ 3/3, mỗi lần chuyển chỗ 3–4.5 m mất **40–60 s**; 93 lần dò: sai TB 13°, 92/93 trong ±30°, lớn nhất 46° (dò đúng lúc người đang đi sang chỗ khác) (trước khi nới ngưỡng `corr`: 2 kịch bản mất thêm 32 s vì hai lần dò đúng hướng (sai 4–7°) bị loại ở corr 0.39–0.44). Khoảng hở nhỏ nhất: đồ đạc 0.18 m, người (lúc xe đang chạy) 0.39 m, người thứ hai đi cắt ngang trước mũi 0.23 m (kịch bản cắt sát hơn, ngoài bộ: 0.15 m — xe chỉ kịp phanh). Sự cố: LiDAR/odom chết giữa lúc đi → xe đi thêm 16–18 cm rồi dừng; script bị `kill -9` → 24 cm (driver tự dừng sau 1 s); có node khác ghi `/cmd_vel` từ đầu, hoặc người đứng cách tâm quay < 0.53 m → xe không nhúc nhích; beacon mất 10 s rồi có lại → dừng chờ rồi tới nơi.
- **Giới hạn đã thấy trong mô phỏng (không sửa được bằng RSSI):** (1) **đồ đạc nằm gần hướng người (trong ±25–30°) và gần xe hơn người** → xe dừng ở đó và coi là đã tới — RSSI không đo được khoảng cách; mức tín hiệu khi người **đứng yên** cũng dao động 10–20 dB giữa các cửa sổ 2 s (đo trên `s4_nguoi_3m0`, `s5_camtay_dungyen`, `s6_nguoi_phai45`) nên không dùng được để nhận "beacon đã dịch chuyển" hay "đã tới gần" — đã thử, bỏ; (2) **người đi liên tục** (0.25 m/s) → không bao giờ bắt kịp, sai số hướng tới 51° vì người di chuyển lúc xe đang xoay: người **phải đứng yên** ~8 s mỗi lần dò; (3) chậm: ~20 s cho mỗi 1.2 m. → RSSI một mình chỉ đủ để **tìm lại và tiến về phía** người; bám mượt vẫn phải là camera + LiDAR. Bước sau (khi người dùng đồng ý): đưa "xoay dò" vào `SEARCH` của planner để quay camera về phía beacon, camera/ReID xác nhận đúng chủ rồi bám tiếp.
- **[CẦN XÁC NHẬN] chưa chạy `rssi_seek.py` trên xe thật.** Chưa biết: mẫu RSSI ở cự ly < 1 m (hiệu chỉnh ở 2 m), ở phòng khác, khi người quay mặt về xe (vòng 1: kém), độ trượt bánh khi xoay nhiều vòng liên tiếp.

**02–03/10 — bám bằng RSSI + LiDAR, không camera (`scripts/rssi_follow.py` + `launch/rssi_follow.launch.py`; chưa chạy xe).** Người dùng yêu cầu: *"hoàn chỉnh rssi nhanh nhất để nó bám theo đúng trước đã rồi mới kết hợp nó với camera"*; vẫn không sửa tracker/planner, chỉ sửa local, không đẩy GitHub.

- **Chia việc:** RSSI **nhận chủ** (xoay dò ~8 s → hướng beacon), LiDAR **giữ bám** nhóm chân 10 Hz, `follow_planner_node` **nguyên bản** lái (né vật cản, chui cửa, giữ 1 m). Launch đổi planner sang ghi `/cmd_vel_follow` (script là nguồn **duy nhất** ghi `/cmd_vel`: chuyển tiếp lệnh planner khi bám, tự xoay khi dò), đổi tên service dừng của planner thành `/rssi_follow/planner_stop` (script nhận `/follow/stop` để dừng được **mọi lúc**), và `occluded_turn_deg` 15 → 60 (13.36). Không có `target_tracker_node`: `/follow/target` do script phát, `source` = `rssi+lidar` / `predicted` / `rssi_bearing`; không bao giờ để planner đang bật thấy mục tiêu `valid=false` (13.34).
- **Bám nhóm chân chỉ bằng LiDAR** (không có camera sửa sai): gộp các cụm cách nhau < 0.32 m thành một người (bám từng cụm thì điểm đo nhảy giữa hai chân → mất dấu giả); gộp 3 vòng quét bù odom (một vòng mất ~40 % tia làm tường vỡ thành "chân"); chỉ lấy cụm trong 0.28–0.45 m quanh dự đoán; cổng vật che co giãn theo tốc độ như tracker; bỏ cụm trùng **mặt nạ đồ đạc đứng yên** (chụp lúc xe đứng yên); bỏ "mảnh nền kẹp giữa hai vật gần hơn" (khúc tường nhìn qua khe hai chân) với cửa sổ **3°** — 6° thì loại luôn chủ đứng lọt giữa hai thùng (kịch bản `ve_s6`). Điểm LiDAR lấy theo các vòng quét trong 0.45 s tính từ **vòng mới nhất** (không tính từ đồng hồ): phép so nền ~0.5 s chặn việc nhận `/scan`, tính từ đồng hồ thì ngay sau đó không còn điểm nào → "không thấy ai" dù chủ đứng ở 1.6 m; `/scan` cũ quá 0.6 s thì `guard()` vẫn dừng xe.
- **Chọn đúng người giữa đồ đạc** (sai số ~15° nên cung ±35° thường có cả đồ đạc giống chân): điểm = (lệch hướng beacon / **20°**)² + khoảng cách/6 + phạt bề rộng lạ − 2.5 nếu nhóm **mới xuất hiện** so với nền (nền 360° gom suốt vòng xoay dò trước, có cả cung mù sau đuôi) + 2.0 nếu "vật cũ" (không phạt ngay sau khi vừa bác mục tiêu: lúc xe bám nhầm thì chủ đứng yên nên thành "vật cũ") − 1.0 nếu ở chỗ vừa mất dấu; bỏ vật RSSI đã bác (180 s). Hệ số 20° (trước 15°: dốc gấp ~2.5 lần mô hình sai số σ ≈ 17°, chủ mới đi tới lệch 33° thua một đồ đạc cũ lệch 6°) chọn từ việc **chấm lại 138 lần khoá** trong nhật ký mô phỏng 03/10: 117 → 122 lần chọn đúng chủ, không lần nào xấu đi; các lần sai còn lại đều là chủ và người/vật giống chân **cùng đứng sẵn** (không có bằng chứng "vật mới"). ≥ 2 ứng viên mà không biết cái nào mới → kiểm tra RSSI ngay khi tới nơi. Quy trình bắt đầu: đứng cạnh hông xe bấm Enter rồi mới đi ra trước mũi xe → chủ là "vật mới".
- **Kiểm tra lại bằng RSSI** khi xe + mục tiêu đứng yên (20 s, gấp đôi dần tới 160 s; mục tiêu chưa từng nhúc nhích: tối đa 40 s): lệch ≤ 40° xác nhận; 40–75° đo lại; hai lần liền > 40° hoặc > 75° → bỏ. **Chuyển** sang nhóm chân mới trong ±35° quanh hướng beacon khi mục tiêu chưa từng nhúc nhích từ lúc khoá, **hoặc** đang đứng đúng chỗ lần quét trước có vật đứng yên (LiDAR đã trượt sang người/vật đứng yên khi chủ đi sát qua — `b_hai_san`; nhóm mới không cần khớp beacon hơn mục tiêu cũ: chuyển đúng cả khi lệch −18° so với −12°). **Có người di chuyển trong lúc xoay dò** (nhóm chân mới so với nửa đầu lần dò) → bỏ kết quả, dò lại: 318 lần dò mô phỏng, người đứng yên sai > 30° chỉ 1 %, người đang đi 30 % (tới 64°) và `corr`/`margin` không lộ ra.
- **Gỡ kẹt:** planner đứng im / lắc tại chỗ khi chưa tới gần (13.12) → lùi ≥ 0.25 m qua chỗ vừa đi + quay mặt về mục tiêu, tối đa 2 lần, rồi kiểm tra RSSI. Hết chỗ xoay dò (vật trong 0.53 m) → lùi; không lùi được → nhờ planner nhích ra chỗ thoáng; vẫn không → đứng chờ rồi thử lại (không thoát). Đã tới nơi + người đứng yên → **giữ xe đứng im** (13.37) tới khi người lệch > 12° / đi tiếp.
- **Phát hiện về planner, chưa sửa, cần người dùng quyết:** 13.34 (node chết khi `/follow/enable` lúc `SEARCH` đi tới — sửa 1 dòng), 13.36 (`occluded_turn_deg` 15° giằng co với `AVOID`), 13.37 (lắc khi `ARRIVED` trong phòng nhiều đồ).
- **Mô phỏng `sil_rssi.py`** (planner thật, 2 node RSSI thật, RSSI = mẫu đo thật phát lại, phòng = 2 vòng quét LiDAR thật; từ 03/10 thêm **méo quét LiDAR khi xe xoay** và **odom đếm thiếu góc 1 %**): bộ chính **33/33 ĐẠT** (16 `rssi_seek` + 17 bám, gồm 3 ca dừng khẩn), bộ mở rộng `--more` **24/24 ĐẠT**, 6 kịch bản bám với LiDAR quay chiều ngược (`--world-extra "--skew -1"`) **6/6 ĐẠT**. Chế độ bám: người đứng yên 2.4–3.2 m → tới cách ~1.1 m; 3 chỗ liên tiếp → tới đủ, chuyển chỗ 3–4.5 m mất ~35–45 s (gồm 1 lần dò); người đi liên tục 0.18–0.20 m/s → bám 93–96 % thời gian, cách TB 1.3 m; hở nhỏ nhất với đồ đạc 0.13 m, với người khi xe chạy ~0.6 m (người thứ hai đi nhanh cắt sát mũi: 0.10 m — planner chỉ kịp phanh, 13.22); dừng khẩn: xe dừng < 1 cm sau 0.6 s; LiDAR chết giữa lúc bám: xe đi thêm ~20 cm rồi dừng. 96 lần dò bộ chính (lượt cuối): sai TB 14.5°, lớn nhất 46°, 98 % trong ±30°.
- **Giới hạn đã thấy (không sửa được bằng RSSI):** (1) RSSI không phân biệt hai vật **đứng yên** cách nhau < ~40° nhìn từ xe và không đo khoảng cách: chủ **đứng sẵn** từ đầu cạnh người/vật giống chân → lần khoá đầu có thể nhầm, tự sửa khi tới gần kiểm tra (`b_hai_san`: bám nhầm 12.5 s); vật to che kín chân chủ → khoá vật cho tới khi chủ đi (`b_che_kin`, `ve_s5`); (2) người phải đứng yên, quay lưng về xe ~8 s mỗi lần xe xoay dò; (3) xe ≤ 0.22 m/s, LiDAR mất dấu ở > ~3 m; (4) người khuất hẳn sau vách: xe chờ và dò lại định kỳ (`b_khuat`).
- **06/10 — chạy thật lần đầu** (máy `thaihoa@th`, phòng nhiều đồ đạc; bag `~/bags/rssi_1006_102110` (mcap), log `rssi_logs/rssi_follow_102854.*` (lần 1, `--verify-sec 0 --max-sec 120`), `rssi_follow_103037.*` (lần 2, mặc định), `t1_101922.log`). **Lần 1:** 54 s, bám 40 s, khoá đúng theo hướng beacon (lệch +8°) nhưng mọi ứng viên "?" — phép so nền đạt 0.43–0.46 < ngưỡng 0.45: lúc bấm Enter người đứng sát hông xe che một mảng tầm nhìn, và xe đã quay 129° nên cung mù sau đuôi lệch chỗ (nền chụp muộn 5 s thì đạt 0.57). **Lần 2:** 224 s, bám 145 s (65 %), xoay dò 63 s (8 lần), cách người trung vị 1.23 m (10–90 %: 0.83–1.76 m); khoá đầu "vật MỚI" lệch beacon −3°. **3 lần mất dấu, cả 3 tự tìm lại** (sau 19 s, 34 s, 18 s): (a) người đi ~1 m/s, xe 0.22 m/s → cách 2.6 m thì LiDAR mất chân; (b) người đi lại sát xe (0.44 m) rồi vòng sang trước-trái (vệt điểm di chuyển trong khung odom) — dự đoán nằm yên trước mũi, chân thật cách dự đoán 0.9–1.2 m, cụm gần dự đoán bị mặt nạ "đồ đạc đứng yên" bỏ; sau đó có vật ở góc sau-trái cách tâm quay 0.49 m < 0.53 m (**[CẦN XÁC NHẬN]** người hay đồ đạc lọt vào khi xe xoay) → xe chờ 4 s mới xoay dò; (c) người đi qua khe giữa đồ đạc, bị che (planner đang `chui khe hep`). 3 lần dò bị bỏ vì có người đi lại lúc xe xoay (nhóm mới lệch hướng beacon 1–17°) → mất thêm 24 s. An toàn: chỉ script ghi `/cmd_vel` (lệnh cuối là dừng; mọi khoảng trống > 0.25 s đều sau lệnh dừng), không lùi lần nào; hở nhỏ nhất với vật cố định lúc xe chạy 5.1 cm (planner "nhích tới để xoay" cạnh đồ bên trái), một điểm 0 cm chỉ có trong 1 vòng quét = nhiễu; launch log không lỗi; self-filter bỏ 7–17 tia (thấp hơn ~20 vì LiDAR máy này mất ~40 % tia). Vận tốc trong `/odom` nhiễu (driver nhận gói từng chùm 3 tin cách 0.5–1 ms → tới 2.5 m/s, 12 rad/s) — không node nào dùng, vị trí odom không nhảy. **[CẦN XÁC NHẬN]** có người khác trong phòng không, xe có chạm vật không.
- **06/10 — sửa sau lần chạy thật (người dùng đồng ý "phương án 1"; chỉ `rssi_follow.py`, không đụng tracker/planner):**
  - **A. Khoá lại nhanh sau khi mất dấu** (`reacq_candidate`, `moving`, `lost_path`; `drive_to(..., reacq=3 s)`): trong 3 s đầu sau `MAT DAU`, tìm trên **đoạn đường người có thể đã đi** (từ chỗ LiDAR thấy cuối, kéo dài theo vận tốc lúc đó tối đa 3 s; bán kính 0.6 m + 0.25 m/s, ≤ 1.6 m). Có **đúng một** nhóm chân **đang đi** — chỗ đó cách đây 1.2–3 s còn **trống** (tia LiDAR đi xuyên qua; xét theo đúng tư thế xe từng vòng quét, nên đồ đạc vừa lọt vào tầm nhìn cho `None`, không bao giờ bị coi là đang đi) — và giữ được 0.6 s → khoá luôn (`KHOA LAI NHANH`), không xoay dò, kiểm tra RSSI ở lần đứng yên đầu tiên. **Không** khoá nhanh khi: ≥ 2 nhóm đang đi trong vùng tìm + 0.8 m; có nhóm 'giống người' đứng yên trong 0.6 m quanh chỗ thấy cuối / quanh nhóm đó; nhóm đó gần xe hơn đường đi > 0.3 m (người khác đi ngang che); đã khoá nhanh 2 lần liền chưa có RSSI xác nhận. Bản đầu tìm quanh **điểm** dự đoán (vận tốc đã giảm dần như `coast()`) → mô phỏng: người bị che vẫn đi 0.4 m/s ra khỏi vùng tìm trước khi kịp khoá. Rủi ro còn lại: chủ bị che kín mà **đúng một** người khác đi qua gần đường đi của chủ → khoá nhầm tới lần kiểm tra RSSI. Không khoá được thì xe xoay dò chậm hơn trước tối đa 3 s.
  - **B. Nhận kết quả dò khi có đúng một nhóm mới:** có người di chuyển trong lúc xoay dò mà chỉ có **đúng một** nhóm chân mới, lệch hướng beacon ≤ 25° và **đã đứng yên** (0.8–1.6 s trước chỗ đó đã có vật) → nhận kết quả, ưu tiên nhóm đó khi chọn (−3 điểm), kiểm tra RSSI sớm. Còn lại dò lại như cũ (người qua đường còn đang đi → dò lại).
  - **C. So nền chỉ trên phần đã nhìn thấy** (`new_vs_background`): bỏ các điểm mà lúc chụp nền bị che / nằm trong cung mù; ngưỡng 0.45 → **0.80** (`BG_MATCH_MIN`), chọn trên các cặp vòng quét của bag thật (odom thật vs cố ý làm lệch): ảnh chụp 5 vòng (260 cặp) nhận đúng 94 % (cũ 93 %), nhận nhầm khi lệch 25° 2 % (cũ 8 %), dời 0.3 m 5 % (cũ 15 %); nền 360° lúc xoay dò (200 cặp) 98 % (cũ 99 %), lệch 25° 36 % (cũ 56 %), dời 0.3 m 34 % (cũ 66 %). Cách tính mới mà giữ ngưỡng 0.45 thì nhận nhầm tới 93 % — **đừng hạ**. Lần chạy 1: 0.44 → 1.00.
  - **Phát lại dữ liệu thật vào hàm thật:** A khoá lại ở lần mất 2 (ngay lúc mất) và lần mất 3 (sau ~2.7 s, đúng chỗ RSSI khoá được 18 s sau) — ngoài đời hai lần này tốn 34 s và 18 s; lần mất 1 (người đã dừng) không khoá → xoay dò như cũ. B nhận cả 3 lần phải dò lại (lệch beacon 17°, 1°, 8°) → đỡ ~24 s.
  - **Mô phỏng:** `test_reacq.py` (offline) 10/10; `sil_rssi.py` thêm 4 kịch bản — `b_sat_hong` (chủ đi sát xe vòng ra sau), `b_di_luc_do` (chủ đi trong lúc xe dò), `b_an_mot` (chủ bị che 2.5 s khi đang đi → phải `KHOA LAI NHANH`), `b_an_hai` (như trên + người đi cùng → **không** được khoá nhanh) — và tuỳ chọn `--pass-r`, `--move-on-spin`, `--companion`, `--hide-walk`, `--follow-script` (chạy bản `rssi_follow.py` khác để so). Cũ → mới: `b_an_mot` khoá lại sau 8.8 → 1.5 s; `b_di_luc_do` khoá ở 16 → 8 s; `b_an_hai` 8.8 → 11.7 s (chờ hết 3 s mới xoay dò). Bộ chính **37/37**, `--more` **24/24**, LiDAR quay ngược 10/10 ĐẠT; 3 test bắt buộc ĐẠT. Mô phỏng **không** tái hiện được lần mất 2 ngoài đời (chủ đi cách hông xe 0.12–0.37 m vẫn không mất dấu): chân người trong mô phỏng sạch, phòng trống không có đồ đạc để mặt nạ nền bỏ nhầm.
  - **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**
- **06/10 chiều — chạy thật bản sửa** (bag `~/bags/rssi_1006_122440`, log `rssi_follow_122614.*` (275 s), `123317.*` (25 s), `123521.*` (130 s), `t1_122400/123302/123437.log`; 5 lần chạy khác thoát ngay vì **0 tin `/odom`** — đúng luật): **B chạy đúng 3/3** (nhận lần dò đầu, lệch beacon −12°, −10°, −7°, không phải dò lại). **A 0/5 lần khoá nhanh** — phát lại vào hàm thật cho thấy đều đúng luật: người đã ở xa / đứng yên, hoặc có nhóm đứng yên ngay chỗ thấy cuối (0.1–0.3 m — có thể chính chủ đứng đó, hoặc đồ đạc; lần +246.7 s có nhóm đang đi cách 1.3 m khớp người đi 0.64 m/s nhưng bị luật này chặn). Mất dấu 5 lần trong 405 s bám, khoá lại bằng xoay dò sau 12–19 s; người đi 0.5–0.64 m/s (xe 0.22), cách trung vị 1.37–1.45 m; 2 lần xoay kiểm tra đều không xong (không đủ chỗ xoay / mất dấu ngay trong lúc xoay). Né vật cản: planner dừng đúng (AVOID dừng cách vật 7 cm), không thấy va chạm do planner. **Ba vấn đề phần cứng:** (1) hub USB reset khi động cơ khởi động → mất khung xe, xe tự xoay tiếp ~3 s (mục 7.3) — **phải sửa trước mọi lần chạy sau**; (2) **xe tự thấy chính nó ở góc sau-trái** (−0.30…−0.40, +0.25…+0.40) — đúng chỗ board C (anten ngoài) / dây cáp: 8.6 % vòng quét (sáng) → 16.6 % (chiều) có điểm trong / sát footprint ở đó, rơi vào dải 3 cm không bị lọc → planner `BLOCKED` lúc đang chạy 0.22 m/s (12:35:55), từ chối xoay; "vật cách tâm quay 0.49 m ở +135°" làm xe chờ 4 s buổi sáng **có lẽ là chính nó** — cần đi dây / anten tránh độ cao 15–21 cm rồi chạy lại `calibrate_lidar mode:=self_scan` (mục 12: lắp thêm phụ kiện thì phải hiệu chỉnh lại); (3) 12:29:46–51 có vật **nằm trong** chiếu footprint ở mũi phải (0.11–0.14, −0.11 và 0–0.12, −0.12) suốt 4 s — tay / chân / dây chạm đầu xe, **[CẦN XÁC NHẬN]** với người dùng.
- **06/10 15:28 — sửa phần cứng (người dùng làm tay) + kiểm (chỉ LiDAR, không driver):** FTDI sang cổng laptop riêng (mục 7.3); dây / anten xếp lại tránh độ cao 15–21 cm. Xe đứng yên ở chỗ trống (vật gần footprint nhất 0.24 m, gần tâm quay nhất 0.70 m), 300 vòng quét: điểm **sát mép footprint < 6 cm = 0 %** (trước sửa 8.6–16.6 %, chủ yếu góc sau-trái) — **hết**. Còn điểm **trong** thân xe ở (−0.05, +0.15) 21.7 % vòng quét (dây / anten vừa xếp, lớp lọc footprint đã bỏ). `self_scan` (100 vòng): thân xe lidar 250–265° và 270–300° (0.10–0.13 m, 100 %), thêm 295–305° (0.10 m) và 220–230° (0.23 m, 2–15 %) — **đều nằm trong footprint** nên lớp 1 của `self_filter_mask` đã bỏ; script gợi ý `blind_sectors_deg: [248, 267, 268, 302]` nhưng **giữ nguyên `[246, 294]`** (nới cung mù thì mù thêm vật thật phía sau-phải, mà phép đo 30 s cho 0 % điểm lọt qua với cấu hình hiện tại). Hồ sơ `self_scan` lưu ở thư mục tạm, không ghi đè `~/.ros/`.
- **06/10 15:45 — chạy thật sau khi sửa phần cứng** (`rssi_follow_154508.*` 303 s, `t1_154319.log`; không ghi bag): **0 sự kiện USB** trong kernel suốt 8 phút launch (11 lần xoay dò + ~5 phút chạy; trước sửa: 2 lần hub reset / ~15 phút), driver không lỗi serial. Người dùng xác nhận: **xe không chạm gì; có người khác đi qua mà xe vẫn bám đúng người đeo beacon**. **A lần đầu chạy ngoài đời:** mất dấu +286.7 s → `KHOA LAI NHANH` sau **3.1 s** (thường 12–19 s), bám mượt người đang đi 13 s tới khi dừng cách 1 m. **B** đúng 2 lần (lệch beacon −14°, −20°). Mất dấu 6 lần; 1 lần dò xoay 760° vẫn khớp kém (corr 0.28) → mất thêm 16 s; xoay kiểm tra không xong (không đủ chỗ). **Lỗi phần mềm tìm ra:** `guard()` báo **nhầm** `mat /odom` (+54.1 s, tự hết sau 1.3 s, kernel không có sự kiện USB) → bỏ một lần dò hợp lệ (beacon +130°, corr 0.69), mất ~11 s. Nguyên nhân: sau mỗi lần xoay dò script tính nặng 0.4–0.9 s (so nền, chọn mục tiêu) mà không nhận tin → `/odom` "cũ" > 0.5 s; **có từ trước** (code buổi sáng cũng có khoảng trống `/odom` tới 0.88 s). **Sửa (người dùng đồng ý):** `rssi_seek.py` `Seeker.guard()` — lỗi có thể phục hồi (mất `/odom`, `/scan`, `/rssi/bearing`, beacon) thì nhận hết tin đang chờ 0.15 s rồi mới xét lại; lỗi phải dừng ngay (hết giờ, node khác ghi `/cmd_vel`) giữ nguyên; mất thật thì xe đi thêm tối đa ~3 cm. Kiểm: 4 test offline ĐẠT; `sil_rssi.py` bộ chính **37/37**, `--more` **24/24**; cảm biến chết giữa lúc đi: xe đi thêm 19–21 cm (trước 16–20 cm, giới hạn 25 cm); dừng khẩn vẫn 0 cm sau 0.6 s. **07/10 chạy thật bản sửa `guard()`** (`rssi_follow_101950.*` 81 s, `102147.*` 199 s, `t1_101750.log`, bag `~/bags/rssi_1007_101850`): **0 lần dừng do `guard()`**, không còn báo nhầm mất `/odom`; mất dấu 6 lần, khoá lại đủ 6; driver không lỗi serial; kernel không có sự kiện USB trong lúc chạy (3 lần `USB disconnect` ở cổng LiDAR 1-2.3 lúc 10:16–10:17 và 10:26 — trước/sau lần chạy, khi cắm rút). Người dùng: "chạy khá tốt". Chưa làm phần giả lập LiDAR ngừng trên xe (đã kiểm trong mô phỏng). Người dùng: **chưa làm phần camera**.

**30/09 — tối ưu riêng phần camera/ReID cho chỗ đông người (7.2-AG).** Người dùng yêu cầu chỉ sửa phần camera, không đụng phần còn lại — đã giữ đúng: không sửa `person_follow_nav`, driver, LiDAR; chỉ `person_follow_identity`, `person_reid_tracker` và `identity_lock_kingsen.yaml`. Kiểm bằng Market-1501, mô phỏng `sim_identity.py` và node thật với webcam laptop (xem 13.24–13.30). **[CẦN XÁC NHẬN] chưa chạy lại trên xe.** Lần enroll đầu tiên với bản mới: đi từ ~3 m lại gần ~0.8 m rồi lùi ra, xoay trái/phải.

**26/09 — câu hỏi của người dùng về logic lùi xe** (chưa chạy thật bản `5285377`): *"lúc kẹt tôi chỉ xoay tay xe tại chỗ một chút về phía rộng là qua, không đẩy tới hay lùi; tôi bước sang phải để xe xoay theo thì xe không làm được"*. Đúng: hai luật an toàn khoá chết xe khi đã sát khung cửa (7.2-AC). Đã sửa, và đổi thứ tự: xoay nhẹ về phía rộng trước, lùi là cách cuối. **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**

**Kết quả 23/09 lần 3 (xe thật, bản 4765acb):** người dùng chạy lại kịch bản rẽ vào cửa bên hông. Xe **đã tự chỉnh góc để vào cửa** (hành vi mới của `_gap_target`) nhưng vẫn kẹt: chia khoảng cách hai bên không đều, kẹt ở góc trái xe; xoay tay sang phải một chút thì qua. Trạng thái `AVOID`, nguồn `camera+lidar` suốt — tracker bám **đúng người**, lỗi nằm ở planner. Đã sửa bằng `_gap_maneuver` (7.2-AB). Kiểm bằng hai lưới mô phỏng mới: lưới động tác chui cửa (`cua_ben_yen` + `START`, 15 tư thế) **8/15 → 15/15**, hở nhỏ nhất 0.02 → 0.07 m; 17 kịch bản hồi quy không xấu đi, cửa trước 0.81 m còn bám thẳng hơn. **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**

**Kết quả 23/09 lần 2 (xe thật, bản 799d643):** người dùng test **rẽ vào cửa bên hông** (đi thẳng rồi vòng phải qua cửa 0.81 m, hai bên trống) — xe vẫn kẹt, cứ hướng vào khung cửa; log `AVOID` liên tục ~23 s. Tái hiện bằng `sim_follow.py` kịch bản `cua_ben` (`SIDE_DOOR=3.0:0.81:-0.9`) và đã sửa bằng `_gap_target` (7.2-AA). **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**

**Kết quả 23/09 (xe thật, bản 6ddbe62):** trường hợp 1 (người thứ hai đứng chen giữa) và 3 (đứng sát trước) **đều đạt**. Còn lại: **cửa thật 0.81 m**, hai bên trống thì xe không vào được mà bám vào một bên khung cửa (người dùng thấy nguồn nhảy `lidar_track`/`camera+bbox` nên nghĩ nhầm thanh cửa là chân — thực ra nhảy nguồn là **đúng**: chân người bị khung cửa che nên tracker dùng bbox, và lỗi nằm ở planner). Có vách dọc hai bên (lúc đi ra) thì qua tốt. Đã sửa (7.2-Z), kiểm bằng `sim_follow.py` với `DOOR=3.2:0.81` (đi thẳng/lệch/chéo/hành lang) — **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**

**Kết quả 22/09 lần 2 (xe thật, bản 8f4b15c; log `source_*.txt`, `state_*.txt` ở gốc workspace):** trường hợp 1 (đứng chen giữa) xe vòng qua được. Người dùng báo thêm: (a) người thứ hai **đi ngang** thì xe như bám theo họ; (b) **qua khung cửa vừa xe**: có lần qua, có lần ép vào thanh cửa; (c) **né xong người rẽ nhanh** thì xe xoay chậm, dù bám bình thường rẽ tốt. Cả ba tái hiện được trong mô phỏng và đã sửa (7.2-W, X, Y). **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**

### Chưa làm

- Giai đoạn 5 — không gian hẹp thật + RSSI
- Giai đoạn 6 — checklist thực địa

### Test offline

| # | Test | File | Trạng thái |
|---|---|---|---|
| 1 | `scan_to_base_points` đổi góc | `test_sim.py` | ĐẠT |
| 2 | `rect_clearance` footprint chữ nhật | `test_sim.py` | ĐẠT |
| 3 | `segment_blocked` phát hiện chen ngang | `test_sim.py` | ĐẠT |
| 4 | `AlphaBeta2D` dự đoán khi bị che 1.5 s | `test_sim.py` | ĐẠT (sai số 0.00 m) |
| 5 | Mô phỏng né người chen ngang | `test_sim.py` | ĐẠT |
| 6 | Chui khe hẹp 0.85 m | `test_sim.py` | ĐẠT |
| 7 | Bộ nhớ góc tương đối vs odom | `test_memory.py` | ĐẠT |
| 8 | Lọc thân xe chặn lỗi `BLOCKED` | `test_self_filter.py` | ĐẠT |
| 9 | Vòng kín RSSI: 2 node + `rssi_seek.py` (16 kịch bản) + planner thật + `rssi_follow.py` (21 kịch bản); `--more`: 24 biến thể chế độ bám | `sil_rssi.py` | ĐẠT 37/37, `--more` 24/24 — **cần ROS**, ~15 + 10 phút, không thuộc 3 test bắt buộc ở 0.3 |
| 12 | Hồi quy vòng kín bám người: cửa (trước, bên hông, gần giữa cửa nhiễu 1–2 cm, lưới 15 tư thế, đã sát khung), người thứ hai chen, xoay theo người, vật thấp, kịch bản khó, RSSI | `regress_follow.py` | 08/10: **165/171** (thêm nhóm `pitch` 6/6; 6 ca LỖI y như 07/10: cửa nhiễu 2 cm 4/30, người chen 1/21, người cao 1.9 m sau thùng) — không cần ROS, ~30 s trên 32 nhân, `--base <commit>` để so; không thuộc 3 test bắt buộc ở 0.3 |
| 11 | Khoá lại nhanh sau khi mất dấu (`rssi_follow.py`): một người đang đi → khoá; hai người cùng đi / người đứng ở chỗ vừa mất / vật che / chân ghế / đồ vừa lọt vào tầm nhìn → không khoá; người bị che vẫn đi thẳng → khoá | `test_reacq.py` | ĐẠT 10/10 (offline, chỉ numpy) |

**[CẦN XÁC NHẬN]** Hai test từng chạy ad-hoc nhưng **chưa đóng gói vào `scripts/`**: kiểm tra định dạng `blind_sectors_deg` phẳng, và quét giới hạn khe hẹp (test 10). Nếu cần dùng lại thì phải viết lại.

---

## 9. GIỚI HẠN KHÔNG GIAN HẸP

Xe rộng **0.60 m** nên đây là vấn đề thật. Kết quả quét tham số bằng mô phỏng (tường dài 6 m để xe không đi vòng được):

| `margin_soft` | `margin_hard` | Khe nhỏ nhất chui được | Lý thuyết |
|---|---|---|---|
| 0.22 | 0.08 | 0.84 m | 0.76 m |
| 0.18 | 0.08 | 0.84 m | 0.76 m |
| **0.15** | **0.06** | **0.80 m** | 0.72 m |
| 0.12 | 0.05 | 0.80 m | 0.70 m |

Đang dùng `0.15 / 0.06`. Hạ `margin_hard` dưới 0.06 **không** giúp thêm, chỉ bớt an toàn.

Cửa phòng tiêu chuẩn 0.80 m là **sát nút**. Test với khe ≥ 0.90 m trước, rồi mới thử cửa thật. **Đừng test 0.75 m rồi tưởng thuật toán hỏng.**

**Vì sao cửa hẹp khó (hình học, 23/09).** Tầng chọn khe kiểm tra một **dải thẳng rộng 0.72 m** từ xe tới đích. Ở cửa rộng `W`, dải đó cắt mặt cửa thành đoạn dài `0.72 / cos θ` với θ là góc lệch của hướng đi so với pháp tuyến cửa. Với `W = 0.81`:

| θ | Bề rộng dải cắt mặt cửa | Điểm cắt được phép lệch tâm cửa |
|---|---|---|
| 0° | 0.72 m | ±4.5 cm |
| 20° | 0.77 m | ±2 cm |
| ≥ 27° | > 0.81 m | **không hướng nào lọt** |

Khi không hướng nào lọt ở tầm dò xa, code lùi về tầm dò ngắn hơn; lúc đó mặt cửa còn xa hơn tầm dò nên mọi hướng đều "thoáng" và xe nhắm thẳng vào người, tức nhắm vào vùng có thanh cửa. Vì vậy **xe phải gần thẳng hàng trước khi tới cửa**. Hành lang hoặc vật chắn dọc hai bên giúp ép xe vào thế thẳng hàng — đúng như quan sát thực tế: đi ra (có vách hai bên) thì qua tốt hơn đi vào (hai bên trống).

Từ 23/09, `_gap_maneuver` (mục 6.3) tự lo việc canh trục: khi đường thẳng không lọt, xe lái tới trục vuông góc của khe, canh mũi cho vuông mặt cửa, rồi mới bò qua. Lưới mô phỏng 15 tư thế xuất phát (lệch trục tới ±0.6 m, lệch góc ±15°) qua được **15/15**, hở nhỏ nhất với tường 0.07 m.

**Cửa hẹp phụ thuộc mạnh vào nhiễu LiDAR (29/09).** Xe lấy khoảng cách **nhỏ nhất** trong ~15 tia chiếu vào thanh cửa, nên nhiễu kéo số đo thấp hơn thật khoảng 1.7 lần độ lệch chuẩn. Xe đã ở gần giữa cửa 0.81 m tường dày 12 cm (30 tư thế, bản sau 29/09):

| Nhiễu LiDAR | Qua cửa | Đứng im > 3 s |
|---|---|---|
| 1 cm | 30/30 | 0 |
| 2 cm | 28/30 | 3 |
| 3 cm | 16/30 | 0 |

Hồ sơ hiệu chỉnh đo tấm phẳng sai 0.6 cm trung bình (mục 3) nên xe thật có lẽ ở mức ~1 cm — **[CẦN XÁC NHẬN]** (13.21).

Giới hạn vật lý vẫn còn và **không sửa được bằng phần mềm**: cửa hẹp hơn 0.72 m thì không có cách nào lọt; cửa 0.81 m chỉ dư mỗi bên ~10 cm, và lệch trục 14° là hết lề — đó là lý do phải canh vuông trước khi chui, không đi chéo.

---

## 10. LỆNH THƯỜNG DÙNG

### Build

```bash
cd ~/ros2_ws/ros2_jazzy_person_reid_ws
colcon build --symlink-install --packages-select person_follow_nav
source install/setup.bash
```

### Dọn tiến trình cũ (làm trước mỗi lần test)

```bash
pkill -f sc_mini; pkill -f decoded_serial; pkill -f follow_planner
pkill -f target_tracker; pkill -f fake_target; pkill -f ros2
sleep 2 && ros2 node list          # phải trống
```

### Hiệu chỉnh

```bash
# Chạy cả 4 bước, tự lưu log vào calib_logs/
bash src/person_follow_nav/scripts/run_calibration.sh

# Hoặc từng bước (cần calibrate.launch.py đang chạy ở terminal khác)
ros2 launch person_follow_nav calibrate.launch.py

ros2 run person_follow_nav calibrate_lidar --ros-args -p mode:=self_scan  # đo thân xe
ros2 run person_follow_nav calibrate_lidar                                # vật trước mũi
ros2 run person_follow_nav calibrate_lidar --ros-args -p expect_deg:=90.0 # vật bên trái
ros2 run person_follow_nav calibrate_lidar --ros-args -p expect_deg:=-90.0
ros2 run person_follow_nav calibrate_center                               # đo tâm quay
```

**Chỉ chép `blind_sectors_deg` từ chế độ `self_scan`.** Ở các bước khác có vật test trong phòng, script không phân biệt được đâu là thân xe.

**Không dùng `test_avoid_only.launch.py` cho `calibrate_center`** — file đó chạy `follow_planner_node`, node này publish `/cmd_vel` liên tục ở 15 Hz nên lệnh xoay bị chen.

### Test giai đoạn 3

```bash
# T1
ros2 launch person_follow_nav test_avoid_only.launch.py

# T2 — kiểm tra trước
bash src/person_follow_nav/scripts/preflight.sh
python3 src/person_follow_nav/scripts/diagnose.py     # khi xe không né

# T2 — giả lập người (1.2 m: xe nhích 0.2 m rồi dừng — an toàn nhất)
python3 src/person_follow_nav/scripts/fake_target.py 1.2 --watch

# T3
ros2 service call /follow/enable std_srvs/srv/Trigger {}
```

### Chạy đầy đủ (giai đoạn 4)

```bash
# 08/10 — MỘT LỆNH (máy thachLG, cổng trong ~/rssi_env.sh): kiểm tra brltty/CH340/cổng/node cũ/symlink rồi launch
bash src/person_follow_nav/scripts/run_full.sh              # --check: chỉ kiểm tra; --no-rssi: không 2 node RSSI
python3 src/person_follow_nav/scripts/watch_follow.py --geom   # terminal khác: 1 dòng / 0.5 s, chỉ đọc topic
bash src/person_follow_nav/scripts/run_full.sh --bag          # terminal khác: ghi bag ~/bags/follow_<ngày_giờ> (đủ topic JSON)
# Kịch bản thử theo bậc B0–B8: README mục "Chạy toàn bộ bằng một lệnh + kịch bản thử theo bậc (08/10)"

ros2 launch person_follow_nav follow_nav_real.launch.py

ros2 service call /person_reid/start_enroll  std_srvs/srv/Trigger {}
# đi quanh camera 20–30 giây
ros2 service call /person_reid/finish_enroll std_srvs/srv/Trigger {}
ros2 service call /follow/enable             std_srvs/srv/Trigger {}

# CÓ RSSI (07/10): camera + LiDAR bám, RSSI chỉ dùng khi MẤT NGƯỜI (SEARCH xoay dò ~8 s rồi quay camera về
# hướng beacon). LiDAR và board RSSI cùng chip CH340 → BẮT BUỘC by-path cho cả hai (source ~/rssi_env.sh)
ros2 launch person_follow_nav follow_nav_real.launch.py start_rssi:=true lidar_port:=$PL ports:="$PA,$PB,$PC"
ros2 topic echo /rssi/status --field data --full-length --once       # 3 board, mỗi board ~16–20 mẫu/s
# Mỗi lần xe xoay dò (note "mat nguoi — xoay do huong beacon"): người đeo beacon đứng yên, quay lưng về xe

# Mô phỏng vòng kín bám người (không cần ROS) — chạy lại sau khi sửa tracker/planner
cd src/person_follow_nav/scripts
CAM_HALF=55 LOWBOX="1.30:-0.25:1.60:0.25;1.30:0.75:1.60:1.05" PX=2.4 START="0:0.35:5" python3 sim_follow.py -q sau_vat_thap
CAM_HALF=55 RSSI=1 TALLWALL="3.0:-6:3.0:1.5;3.0:2.4:3.0:6" python3 sim_follow.py -q an_sau_vach
```

**Luôn build với `--symlink-install`** (7.3): build không symlink một lần là `install/` giữ bản chép cũ của launch/config, lần sau sửa `src` mà xe vẫn chạy bản cũ. Kiểm tra nhanh: `ls -la install/person_follow_robot/share/person_follow_robot/config/` — `identity_lock_kingsen.yaml` phải là `->`.

### RSSI (01–03/10 — thử nghiệm: `rssi_seek.py` chỉ với `calibrate.launch.py`; `rssi_follow.py` chỉ với `rssi_follow.launch.py`)

Máy `thaihoa@th`: `source ~/rssi_env.sh` cho `$WS`, `$PA $PB $PC` (3 board) và `$PL` (LiDAR). Cổng là `by-path` — đổi ổ cắm thì phải sửa file đó.

```bash
# T1 — CHỈ lidar + driver (không planner)
ros2 launch person_follow_nav calibrate.launch.py lidar_port:=$PL
# T2 — 2 node RSSI (không node nào ghi /cmd_vel)
ros2 launch person_follow_nav rssi.launch.py ports:="$PA,$PB,$PC"

# T3 — kiểm tra
ros2 topic echo /rssi/status  --field data --full-length --once   # 3 board, mỗi board ~16–20 mẫu/s
ros2 topic echo /rssi/bearing --field data --full-length          # valid=false cho tới khi xe xoay tại chỗ ≥ 250°

# T3 — thử CHỈ RSSI (dò → quay → đi từng đoạn; script tự ghi /cmd_vel; nhật ký rssi_seek_*.jsonl + .csv ở thư mục hiện tại)
cd $WS/rssi_logs
python3 $WS/src/person_follow_nav/scripts/rssi_seek.py --once     # tới người một lần rồi thoát
python3 $WS/src/person_follow_nav/scripts/rssi_seek.py            # tới → chờ → người sang chỗ khác → tìm lại

# ── BÁM LIÊN TỤC bằng RSSI + LiDAR (02–03/10) — launch KHÁC: có planner, planner ghi /cmd_vel_follow ──
# T1 (thay cho cả T1 + T2 ở trên)
ros2 launch person_follow_nav rssi_follow.launch.py lidar_port:=$PL ports:="$PA,$PB,$PC"
# T2 (nhật ký rssi_follow_*.jsonl + .csv). Đứng CẠNH HÔNG xe bấm Enter; trong lúc đếm ngược đi ra trước mũi xe
# 2–2.5 m rồi đứng yên, quay lưng về xe. Mỗi lần xe xoay dò: đứng yên, quay lưng về xe
cd $WS/rssi_logs
python3 $WS/src/person_follow_nav/scripts/rssi_follow.py --delay 10
# DỪNG KHẨN: Ctrl-C ở T2; hoặc từ terminal khác (mọi lúc, kể cả lúc xe tự xoay dò / lùi):
#            ros2 service call /follow/stop std_srvs/srv/Trigger {}      (ở launch này script nhận /follow/stop)

# Hiệu chỉnh lại mẫu (đổi chỗ lắp board/anten): người đứng trước mũi 2 m, beacon nhìn thẳng xe
python3 $WS/src/person_follow_nav/scripts/rssi_rotate.py --ports $PA $PB $PC --person-deg 0 --turns 2 --w 0.4 --out cal.csv
python3 $WS/src/person_follow_nav/scripts/rssi_rotate.py --analyze cal.csv --calib --no-lidar-truth
cp mau_rssi.json $WS/src/person_follow_nav/config/rssi_template.json

# Mô phỏng vòng kín có ROS (không cần xe, miền ROS riêng) — chạy lại sau khi sửa rssi_df / 2 node / rssi_seek / rssi_follow
python3 $WS/src/person_follow_nav/scripts/sil_rssi.py             # bộ chính 37 kịch bản (~15 phút, 3 đợt), phải "TAT CA DAT"
python3 $WS/src/person_follow_nav/scripts/test_reacq.py           # offline (chỉ numpy): luật khoá lại nhanh của rssi_follow.py, phải "DAT"
python3 $WS/src/person_follow_nav/scripts/sil_rssi.py --more      # bộ mở rộng 24 kịch bản chế độ bám (~10 phút)
python3 $WS/src/person_follow_nav/scripts/sil_rssi.py b_phong3 b_lientuc   # chỉ vài kịch bản (--list để xem tên)
```

Muốn dừng xe khi đang chạy `rssi_seek.py`: bước ra đứng **trước mũi xe** (xe dừng và chờ), rồi Ctrl-C ở T3.

### Giám sát

```bash
# Message là String chứa JSON: `--field source/state` KHÔNG chạy (Invalid field), và phải có
# --full-length vì mặc định ros2 topic echo cắt chuỗi dài bằng "..."
ros2 topic echo /follow/target --field data --full-length | grep --line-buffered -oP '"source": "\K[^"]+'        # tracker đang dùng nguồn nào
ros2 topic echo /follow/planner_status --field data --full-length | grep --line-buffered -oP '"state": "\K[A-Z_]+' # planner đang làm gì
ros2 topic info /cmd_vel --verbose                     # kiểm tra chỉ 1 publisher
ros2 topic hz /scan                                    # ~10 Hz
ros2 topic hz /odom                                    # ~20–50 Hz
```

### Dừng khẩn cấp

```bash
ros2 service call /follow/stop std_srvs/srv/Trigger {}
```

---

## 11. BẢNG TINH CHỈNH

| Triệu chứng | Sửa |
|---|---|
| Xe đứng yên không nhúc nhích | Kiểm tra self-filter (N ≠ 0, độ thoáng quanh footprint trong `diagnose.py` > 0.06), và `sample_window_sec` ≥ 0.4 |
| Xe đi lòng vòng, không bám sát | Tăng `w_goal` |
| Xe đi sát vật cản quá | Tăng `w_clear` hoặc `margin_soft` |
| Xe giật, đổi hướng liên tục | Tăng `w_smooth` |
| Xe do dự trái/phải khi né | Tăng `avoid_side_hold_sec` |
| Xe né xong hay mất người | Tăng `w_fov` |
| Xe dừng trước khe lẽ ra chui được | Kiểm tra `half_width` trước, rồi mới giảm `margin_hard` |
| Bánh không quay ở lệnh nhỏ | Tăng `min_move_linear` (0.035) / `min_move_angular` (0.10) |
| Xe lắc qua lại khi bám người đi thẳng | Tăng `bearing_deadband_deg` (4) — đổi lại xe để người lệch nhiều hơn mới chỉnh |
| Người ra khỏi camera mà xe quay lại chậm | Giảm `occluded_turn_deg` (15) |
| Mất người thì xe chạy quá xa / quá lâu mới quét | Giảm `search_goto_max_sec` (8); 0 = tắt pha đi tới |
| Dự đoán "bay" theo hướng người đi khi mất hình | Giảm `predict_velocity_decay_sec` (1.0) |
| Người thứ hai đứng chen lâu thì xe bám nhầm họ | Tăng `lidar_reject_max_sec` (3) — đổi lại lâu đồng bộ lại hơn nếu dự đoán trôi |
| Xe đi vòng cả khi người thứ hai đứng xa ngoài đường / vòng nhầm chính người đang bám | Tăng `target_clear_radius_m` (0.45) |
| Xe canh trục cả với khe rộng (không cần), hoặc không canh với khe hơi rộng | Chỉnh `gap_waypoint_max_width_m` (1.2); 0 = tắt hẳn động tác chui khe |
| Xe vào động tác chui khe quá sớm, chạy chậm cả đoạn đường dài | Giảm `gap_engage_range_m` (1.6) |
| Xe qua cửa quá chậm / quá nhanh mà cà vào khung | Chỉnh `gap_cross_speed` (0.12) |
| Xe bắt nhầm "khe" khi chạy dọc tường (tia quét chéo) | Tăng `gap_min_span_deg` (10) |
| Xe lùi ra vào nhiều lần trước cửa | Giảm `gap_back_max_m` (0.40); 0 = cấm lùi — đổi lại dễ kẹt sát cửa hơn |
| Xe qua cửa nhưng lệch hẳn một bên | Giảm `gap_axis_tol_m` (0.12) — đổi lại xe canh trục lâu hơn |
| Xe đứng im / lắt nhắt giữa cửa dù thực tế lọt | Kiểm tra nhiễu LiDAR (mục 9); hạ `gap_cross_margin_m` (0.03) — **đừng** hạ `margin_hard` |
| Xe cà vào khung cửa khi chui | Tăng `gap_cross_margin_m` (0.03 → tối đa 0.06 như trước 29/09) — đổi lại dễ đứng im giữa cửa hơn |
| Xe xoay theo người vẫn chậm khi người rẽ | Tăng `w_center` (0.8) — quá lớn thì ảnh hưởng bước né |
| Người đứng yên, người thứ hai chen sát trước mà tracker bám nhầm họ | Giảm `lidar_track_occluder_margin_m` (0.15) — đổi lại người bước nhanh về phía xe dễ bị coi là vật che |
| Người đang bám đi về phía xe nhanh mà nguồn nhảy sang `camera+bbox` | Tăng `occluder_margin_m` (0.3) — đổi lại người thứ hai đứng sát trước dễ bị nhận nhầm hơn |
| Người đứng sau vật thấp mà xe vòng qua dù đứng trước vật cũng đủ gần | Giảm `nav_far_weight` (4) — mỗi mét còn xa người "đáng" bao nhiêu mét đường vòng |
| Người đứng sau vật mà xe đứng trước vật, không chịu vòng | Tăng `nav_far_weight` (4); xem `cam_dist_m`/`cam_dist_from` trong `/follow/target` — khoảng cách ước sai thì sửa hình học camera trước |
| Đi vòng theo bản đồ mà xe xoay tại chỗ nhiều | Tăng `nav_turn_first_deg` (75) — quá cao (90) thì DWA cắt góc, mũi chạm vật |
| Đi vòng theo bản đồ mà mất camera lâu | Giảm `nav_fov_keep_deg` (50) — 22 thì chi phí camera kéo mũi về phía vật đang chắn |
| Đường vòng sát vật quá | Tăng `nav_soft_m` (0.25) / `nav_soft_cost` (3) — **không** hạ `nav_lethal_m` dưới `half_width` (0.30) |
| Tracker báo người gần/xa hơn thật khi chân bị che (`camera+bbox`) | Kiểm `camera_height_m` (0.34) và `camera_pitch_deg` (identity) đúng thực tế; xem `person_height_m`, `elev_bias_deg` trong `/follow/target` (sai lệch góc ngửa học được > ±3° là cấu hình sai) |
| Mất người mà xe không xoay dò RSSI | `ros2 topic echo /rssi/bearing` phải có `beacon_ok: true`; `rssi_search_enabled` (true); không đủ chỗ xoay thì xe nhích ra (`open`) trước |
| Xe quá rụt rè trong hành lang | Giảm `margin_soft` |
| Xe cọ tường | Tăng `margin_hard` |

---

## 12. VẤN ĐỀ THỰC TẾ CẦN LƯU Ý

**Odom trôi.** Encoder BW-DR03 trôi 2–5% quãng đường, chưa kể trượt bánh. Bộ nhớ odom chỉ dùng trong 2.5 s nên trôi tối đa vài cm — chấp nhận được. **Đừng tăng `predict_max_sec` quá 4 s.**

**LiDAR quét ở một độ cao cố định (18 cm).** Thấy chân người, chân bàn — **không thấy** mặt bàn nhô ra, bậc thềm, dây điện. Hạn chế vật lý. Hai sonar sẵn có trên BW-DR03 (`/bw_dr03/sonar`) có thể bù cho vật thấp — **chưa dùng**.

**Người là vật cản động.** Người đi cắt ngang 1.2 m/s trong khi xe chạy 0.22 m/s — xe không "né" kịp theo nghĩa lập kế hoạch, nó chỉ chậm lại và chờ. Điều này đúng và an toàn. Đừng tăng tốc độ để né nhanh hơn.

**Camera thật ra là góc rộng (đo 30/09: 114.4° ngang, 85.4° dọc ở 640×480, không phải 62°).** Ở 1.0 m trường nhìn ngang ~3 m. Phía điều hướng vẫn giữ người trong ±22° (`fov_keep_deg`) — chặt hơn cần thiết nhưng giữ người ở vùng giữa ít méo. Ống rộng méo thùng ở mép: người ở mép khung bị kéo/nén, ReID kém hơn ở giữa.

**Ánh sáng ngược.** ReID dùng đặc trưng màu (`color_weight: 0.20`). Đi từ trong nhà ra cửa sổ làm màu đổi hoàn toàn và ReID mất target. Enroll ở đúng điều kiện ánh sáng định demo.

**Watchdog driver.** `cmd_timeout: 1.0` — xe tự dừng nếu 1 s không nhận lệnh. Planner chạy 15 Hz nên an toàn, nhưng nếu CPU quá tải (YOLO trên CPU) thì tần số tụt và xe giật. Kiểm tra `ros2 topic hz /cmd_vel` ≥ 10 Hz.

**Khi nào phải hiệu chỉnh lại LiDAR:** tháo lắp lại LiDAR dù chỉ nới ốc; lắp thêm phụ kiện lên xe; log self-filter vượt hẳn ~50 (bình thường 20–50); xe né sai hướng hoặc kẹt `BLOCKED` mà không có gì chắn.

---

## 13. [CẦN XÁC NHẬN] — CHƯA CHẮC CHẮN

Các mục sau **chưa được kiểm chứng**. Không tự đoán, hãy hỏi người dùng hoặc đo trước khi dựa vào.

1. **Vị trí camera trên xe** (`camera_x`, `camera_y`) chưa đo. Tracker hiện coi góc camera như đo tại gốc `base_link`. Nếu camera đặt ở mũi (x ≈ +0.14), thị sai ở khoảng cách gần đáng kể: ở 0.8 m và góc 30°, sai số ≈ 5°. Hiện chấp nhận được vì cửa sổ ghép cụm LiDAR là ±10° và góc cuối lấy từ cụm LiDAR, nhưng nếu cần chính xác hơn thì phải thêm tham số và so góc **nhìn từ camera** thay vì từ base_link.

2. **Khoảng cách hai trục (wheelbase) chưa đo bằng thước.** Phép đo tâm quay cho `L = 0` và người dùng xác nhận tâm hai bánh trước đứng yên khi xoay. Nhưng trước đó người dùng từng mô tả "bánh sau mới chính là bánh chủ động" — mâu thuẫn chưa được giải thích dứt điểm. Con số đo được là con số code cần, nên không chặn tiến độ.

3. **Hai bánh sau là bánh dẫn hướng (caster) hay có động cơ?** Chưa rõ.

4. **`bbox_height_at_1m_px: 420.0`** kế thừa từ config cũ, **chưa kiểm chứng**. Chỉ dùng khi LiDAR không tìm được cụm nào, nên ít ảnh hưởng.

5. **Quy ước góc RSSI** (`rssi_angle_sign: -1.0`) **chưa kiểm chứng** — mạch RSSI chưa cắm. Cách kiểm tra: đứng **bên phải** xe, nếu `/rssi/angle_deg` dương thì `-1.0` đúng. *(Chỉ áp cho đường CŨ `rssi_serial_node` → `/rssi/angle_deg`, đang tắt. Hai node mới 01/10 không dùng tham số này: hướng tính trong khung odom từ mẫu hiệu chỉnh trên xe.)*

6. **Hai sonar `/bw_dr03/sonar`** — chưa đọc, chưa biết định dạng và độ tin cậy.

7. **YOLO/ReID đang chạy CPU** (kiểm tra 17/09). Máy có GPU NVIDIA rời (PCI `2d59`, dòng RTX 50) + Radeon tích hợp, nhưng `nvidia-smi` báo *"No devices were found"* (driver không nhận GPU) và torch cài là `2.12.0+cpu` → `core.choose_device("auto")` rơi về CPU. **Chưa đo tốc độ thật** (`ros2 topic hz /person_reid/target`). Chỉ cần xử lý GPU nếu tốc độ không đủ — việc đó cần sửa driver NVIDIA (sudo, khởi động lại) và cài torch bản CUDA: **người dùng quyết định**.

8. **Nội dung chi tiết của `person_follow_identity`** — đã đọc `node.py` và `core.py` để lấy định dạng payload, nhưng **chưa kiểm chứng toàn bộ các trường** ở runtime.

9. **Độ chính xác odom** (`encoder_ppr_default: 750`) — **quãng đường đã kiểm chứng** (16/09): thước 1.50 m / odom 1.503 m. **Góc xoay: odom đếm thiếu ~3%** (thật 135° / odom 130.9°; 34° / 33°) → `wheel_separation` hiệu dụng ≈ 0.388 thay vì 0.40. **Chưa đổi** — đọc góc bằng mắt sai 1–3°, cần xoay ~2 vòng (`measure_speed.py 0.0 --w 0.8 --sec 15`) để chốt. **01/10:** so khớp vòng quét LiDAR trước/sau 9 lần xoay 1 vòng ở 0.82 rad/s cho xe quay thật nhiều hơn odom 2.6–4.5°/vòng (**≈ 1%**, không phải 3%) — đo bằng LiDAR nên tin hơn đọc bằng mắt; vẫn chưa đổi tham số. Nếu đổi thì tính lại `max_angular` (≈ 2.52), vì 2.46 lấy từ odom.

10. **ĐÃ XÁC NHẬN 17/09 — lỗi "LiDAR không xoay, xe chỉ chạy thẳng" không còn.** `diagnose.py` cho thấy LiDAR quay và planner dùng dữ liệu; xe thật đã né được vật cản với `fake_target.py 2.5` (xem mục 8). Nguyên nhân cũ chưa rõ — có thể do lỗi tốc độ 7.2-P.

11. **Hành vi thực địa của thuật toán** — giai đoạn 3 (né vật cản với mục tiêu giả) **đã đạt 6/6 trên xe thật 17/09** (mục 8). Phần có camera/ReID (giai đoạn 4 trở đi) **chưa chạy thật**.

12. **Planner kẹt đứng yên vĩnh viễn cạnh vật cản (thấy trong mô phỏng, chưa sửa).** Test 5 của `test_sim.py` chỉ ĐẠT với đúng kịch bản gốc. Dời người chen ngang 5 cm, hoặc cho người đi 0.10 m/s, thì 11/18 biến thể xe dừng cách người chen ~0.28 m rồi đứng yên mãi ở `AVOID` (DWA chọn v=w=0). `stuck_time_sec` được khai báo nhưng không dùng → xe thật không có cơ chế thoát kẹt. Hạ `v_max` xuống ≤ 0.21 cũng làm test 5 `LOI` vì cùng lý do. **Trên xe thật 17/09 chưa gặp**: vật cản giữa đường, 3.3/3.4 (thùng lệch 20 cm) đều né được. Vẫn có thể lộ ra với người đi chậm ở giai đoạn 4. **07/10:** dạng "đích nằm trong / ngay trước vật" đã sửa (`goal_bad` → bản đồ lưới, 7.2-AM); bản đồ còn gỡ được kẹt cạnh vật nhờ xoay/lùi ngắn (7.2-AN). `stuck_time_sec` vẫn không dùng.

13. **ĐÃ XÁC NHẬN 17/09 — bánh quay được ở 5% PWM.** Lệnh nhỏ nhất của planner (`min_move_linear 0.035`, `min_move_angular 0.10`) đều ra 5%. Đo: `measure_speed.py 0.035` → thước 11.5 cm / odom 11.4 cm; `--w 0.10` → thật 34° / odom 33°. Giữ nguyên hai tham số.

14. **Sửa bám người 17/09 (7.2-Q…U) mới kiểm bằng mô phỏng vòng kín, chưa chạy xe thật.** Số liệu `sim_follow.py` (camera ±25°), gốc → mới: đi thẳng `w` đổi chiều 42 → 25 lần; bước ngang 1 m/s mất camera lâu nhất 0.7 → 0.2 s; ra khỏi khung (LiDAR mất chân) lệch trung bình 6.0° → 3.6°; rẽ gắt 0.5 m/s và đi chéo 0.8 m/s không đổi. **Góc tường gắt** (người rẽ sát mép tường, nhanh gấp đôi xe): xe giờ quay về phía người, vào SEARCH và lái tới góc, nhưng **vẫn không thấy lại người** — tới góc thì mép tường trong 0.53 m nên `_can_rotate_in_place()` (đòi trống cho cả vòng 360°) không cho quét → `IDLE`. Có thể cần kiểm tra quét theo góc xoay thực (footprint quét trong DWA) thay vì cả vòng. **26/09:** các nhánh xoay về một hướng cụ thể đã chuyển sang `_can_turn` (quét đúng góc); riêng pha quét của `SEARCH` vẫn dùng luật 360°. Góc tường thoáng hơn thì cả code cũ lẫn mới đều không mất người. **07/10:** pha quét của `SEARCH` cũng chuyển sang `_can_turn(±0.6 rad)`, thêm pha "quay mặt về chỗ thấy cuối" (7.2-AL); mô phỏng `goc_tuong` hết `IDLE` (camera 27% → 41%, kết thúc `FOLLOW`). Riêng xoay dò RSSI vẫn đòi trống cả vòng (nó xoay 1–2 vòng) — không đủ chỗ thì nhích ra trước.

15. **Tham số khai báo nhưng không dùng:** `fov_cost_only_when_visible` (DWA luôn bật chi phí FOV) và `stuck_time_sec` (xem 13.12). Chưa sửa.

16. **`_gap_maneuver` là chỗ DUY NHẤT planner tự sinh lệnh `(v, w)` không qua DWA** (23/09). Vẫn qua bộ lọc va chạm: `_arc_clearance` mô phỏng đúng lệnh đó 1.2 s với footprint chữ nhật và đòi thoáng > `margin_hard`. Nó cũng là chỗ duy nhất xe **lùi** (tối đa `gap_back_max_m` 0.40 m) trong khi LiDAR mù thẳng phía sau — dựa vào giả định "chỗ vừa đi qua thì trống". Từ 26/09 lùi là **cách cuối cùng**: trước đó xe thử xoay nhẹ về phía rộng, rồi nhích tới theo cung. **Chưa chạy trên xe thật.** Khi test lần đầu nên đứng sẵn cạnh `/follow/stop`.

17. **Người đi nhanh hơn xe rồi rẽ khuất vào cửa thì vẫn mất người** (mô phỏng 23/09). Ma trận 15 lần chạy kiểu "người vừa đi vừa rẽ vào cửa bên hông": bản cũ 7/15, bản mới 5/15 — nhưng phần lớn trượt là do **mất hẳn người** trước khi tới cửa (camera 62°, người 0.25 m/s so với xe 0.22 m/s), không phải do kẹt ở khung cửa; quét tham số `gap_axis_tol_m` 0.12/0.20/0.28 không đổi kết quả, xác nhận nguyên nhân nằm ở chỗ khác. Lưới đo riêng động tác chui cửa (người đứng yên) thì 15/15. Cần đo lại trên xe thật trước khi kết luận.

18. **Người thứ hai đứng SÁT trước người đang bám** — bản 8f4b15c bám nhầm (mô phỏng `chan_sat` 0.4 m: 65%). Bản 22/09 lần 2 có cổng vật che (`occluder_margin_m`): mô phỏng **0%**. Vẫn chưa phân biệt được nếu người thứ hai đứng **ngang hàng** (cùng khoảng cách, lệch < ~10°) với người đang bám. Khi xe vòng qua người thứ hai, khe hở với chân họ có thể chỉ cỡ `margin_hard` (6 cm) — margin đã hiệu chỉnh (mục 3), không đổi tự ý.

19. **Xe kẹt với góc xe đã chìa QUA thanh cửa, lơ lửng trên đoạn tường bên cạnh** (mô phỏng 26/09, mũi lệch ~25° so với pháp tuyến cửa). Xoay về phía cửa sẽ ép góc đó xuống tường, xoay ngược lại thì an toàn nhưng quay lưng với cửa; lùi–xoay trong mô phỏng cũng chưa gỡ được. 4/12 ca của lưới "đã kẹt" rơi vào đây. Khác với ca người dùng mô tả (góc xe tì vào **mặt trong** thanh cửa — ca đó đã tự thoát được). Nếu gặp ngoài đời, ghi lại `note` và tư thế xe.

20. **`gap_cross_margin_m` = 0.03 (29/09)** — khoảng hở **đo được** tối thiểu khi đang chui khe, thấp hơn `margin_hard` 0.06. Lý do: số đo luôn bị nhiễu kéo thấp hơn thật (mục 9); để 0.06 thì xe đứng im giữa cửa 0.81 m dù thật hở 7 cm. Mô phỏng: khoảng hở **thật** nhỏ nhất vẫn ≥ 5 cm. Ngoài động tác chui khe vẫn là 0.06. **Chưa chạy xe thật** — muốn an toàn như cũ thì đặt lại 0.06.

21. **Độ nhiễu thật của SC-Mini ở tầm 0.3–0.5 m chưa đo.** Qua cửa 0.81 m phụ thuộc mạnh vào nó (bảng mục 9). Cách đo: đặt xe đứng yên cách tường phẳng 0.4 m, ghi 100 vòng quét, tính độ lệch chuẩn của tia vuông góc.

22. **Người thứ hai bước ngang SÁT mũi xe khi xe đang chạy tới** (mô phỏng 29/09, 3/24 ca): xe phanh và dừng cách chân họ ~9 cm (trên `margin_hard` 6 cm, không chạm), rồi bò quanh. Người đi 1 m/s vào đường xe ở 35 cm thì không cách nào tránh rộng hơn. Không sửa.

23. **Test 5 của `test_sim.py` nhạy hỗn loạn với `w_max`/`accel_ang`** (29/09): lưới lấy mẫu `w` của DWA đổi theo hai số này. 1.0/2.4 ĐẠT, 1.1/2.4 và 0.8/2.4 LỖI (xe đi thẳng `w=0` dù người lệch 20°). Cùng loại mong manh với 13.12. Đổi hai số này thì phải chạy lại test 5.

24. **Camera/ReID bản 30/09 (7.2-AG) chưa chạy trên xe.** Mới kiểm bằng: Market-1501 (độ chính xác mạng), mô phỏng `sim_identity.py` (ảnh người thật ghép theo hình học camera, DeepSORT giả lập), node thật với webcam laptop, và node thật với **KINGSEN cắm vào laptop** (30/09): tự enroll 80 mẫu sau 35.7 s, `TRACKING` với độ giống 0.85–0.98, loại đúng một đối tượng thứ hai đã học là "không phải mục tiêu", nhận lại sau khi ra khỏi khung 0.4–0.6 s; 15 Hz khi camera 25 fps, 26–33 ms/khung, trễ trung vị 56 ms, CPU ~5–6 lõi. Ngưỡng ReID đặt theo phân bố Market-1501 (nhiều camera khác nhau) — trên xe cùng một camera nên người thật sẽ có điểm cao hơn; nếu xe hay `TRACK_VERIFY_WARN` hoặc nhận lại chậm thì xem `reason` trước khi hạ ngưỡng.

25. **`processing_hz` camera 8 → 15 (30/09).** 6 người ~48 ms/khung trên CPU máy này. Phải kiểm tra lại `ros2 topic hz /cmd_vel` ≥ 10 Hz khi YOLO chạy (mục 12 "Watchdog driver"); nếu tụt thì hạ `processing_hz` về 10.

26. **`ts` giờ là lúc chụp khung** → phần bù trễ của `target_tracker_node` (`compensate_camera_latency`) bắt đầu có tác dụng thật (~30–80 ms góc quay của xe). Không sửa tracker; nếu thấy góc người lệch khi xe quay nhanh thì so `latency_ms` trong payload.

27. **Độ cao camera: 34 cm (người dùng báo 30/09), sẽ nâng ~70 cm.** Với FOV dọc đo được 85.4°, ở 34 cm nhìn ngang người cách 1 m lộ từ sàn tới ~1.26 m (thiếu vai, đầu): ReID Rank-1 58.2% (toàn thân 75.8%); ngửa 10° → tới 1.65 m, 73.5%; ngửa ~12° → toàn thân; 70 cm nhìn ngang → tới 1.62 m, 72.6% (Market-1501 ReID theo dải, bảng trong README gói camera). Khuyến nghị: ngửa ~12° (đứng cách 1 m thấy cả đầu), ghi góc vào `camera_pitch_deg`. **[CẦN XÁC NHẬN]** góc ngửa thực tế sau khi lắp. (Bảng đầu tiên 30/09 tính theo FOV 62° ra "chỉ thấy chân, 22%, ngửa 20°" — sai do FOV sai.) `bbox_height_at_1m_px: 420` (fallback khoảng cách của tracker, 13.4) sẽ sai khi đổi độ cao/góc — chỉ dùng khi LiDAR không thấy chân. Enroll nên đi từ ~3 m lại gần ~0.8 m.

28. **Người mặc gần như giống hệt mục tiêu + DeepSORT tráo ID khi hai người chồng nhau**: mô phỏng với 2 người **giống nhất trong 750 người** vẫn còn báo nhầm 0.33–0.38 s/phút (phần lớn vài khung, dài nhất 3 s). Đám đông người ngẫu nhiên: 0–0.01 s/phút. Chạy lại với hình học camera thật (ống mắt cá 114.4°, 34 cm, sau khi thêm `TRACK_VERIFY_HOLD` 30/09): nhầm 0–0.03 s/phút ở mọi cấu hình; thiếu 20.2% (đông thường, nhìn ngang) / 19.0% (ngửa 12°), 43.7% / 30.6% (2 người giống nhất). Bản cũ: nhầm 0.12–0.29 s/phút, thiếu 25–65%. Bản cũ: 0 nhầm nhưng thiếu 94–97% (gần như không nhận lại sau lần mất đầu). Bảng đầy đủ trong README gói camera. Không có cách nào chỉ bằng ngoại hình phân biệt người giống hệt khi mục tiêu bị che hoàn toàn.

29. **OSNet chưa dùng.** Đo Market-1501: OSNet-AIN x1.0 (chưa từng học Market) R1 69.9% nhưng chậm 4.6 lần; mạng DeepSORT (đã sửa RGB, học trên chính Market) 75.5%. Ở môi trường thật cả hai đều "khác miền" nên OSNet-AIN có thể tốt hơn — chỉ đổi khi có dữ liệu thật để so. Trọng số MIT tại `huggingface.co/kaiyangzhou/osnet`.

30. **File trùng ở gốc `src/person_follow_identity/*.py`** (core.py, node.py, …) là bản sao CŨ, không được cài (setup.py chỉ lấy thư mục `person_follow_identity/`). Người dùng 30/09: **để yên, không xoá** (cả các file không dùng khác).

31. **FOV KINGSEN: ĐÃ ĐO 30/09 — ngang 114.4°, dọc 85.4° ở 640×480, ống mắt cá đều.** Người dùng đo bằng thước 5 vạch (camera cách tường 1.00 m; vạch 1→3 98.5 cm, 3→5 54, 5→7 54, 7→9 98.5; lần đo đầu "2 mép cách 2.75 m" là số chưa chính xác, đã được người dùng sửa). `measure_fov.py` khớp mô hình mắt cá OpenCV, sai số 0.1 px: `camera_matrix [324,0,320, 0,324,240, 0,0,1]`, `dist_coeffs [-0.01077,0,0,0]`, `camera_fisheye: true`, `camera_angle_model: calibrated`. Pinhole cùng FOV sai 9° ở x = 160 px. Tâm ảnh giả định ở giữa khung (vạch đo đối xứng nên theo chiều ngang là đúng). Phía điều hướng (`follow_nav.yaml`) vẫn ghi `camera_fov_deg: 62.0` ở tracker và planner — **không sửa** theo yêu cầu chỉ đụng phần camera; ở tracker nó chỉ dùng cho cờ `in_camera_fov` trong log/JSON, ở planner khai báo nhưng không dùng. `fov_keep_deg 22` / `occluded_turn_deg 15` vẫn an toàn (chặt hơn cần thiết) với camera 108°.

32. **Firmware KINGSEN tự nhảy 25 ↔ 12.5 fps** (đo 30/09): mở camera xong 12.5 fps trong 8–60 s rồi mới lên 25; thỉnh thoảng tự quay về 12.5 fps dù phơi sáng và độ sáng ảnh không đổi. Không điều khiển v4l2 nào khoá được (`exposure_dynamic_framerate` bị bỏ qua). Đã giảm: đặt `v4l2-ctl` trước khi mở camera (~8 s thay vì 17–24 s), chống nhấp nháy 50 Hz (15 s thay vì 45–60 s khi tắt), phơi sáng bắt đầu 30 ms, đổi tối đa 1 lần/5 s. Ở 12.5 fps phơi sáng vẫn ≤ 30 ms. Log `camera: nhan ... fps` cho biết fps thật; nếu luôn ~12.5 thì kiểm tra `v4l2-ctl -d <cam> --get-ctrl=backlight_compensation,auto_exposure` (phải là 0 và 1).

33. **Hai node RSSI + `rssi_seek.py` (01/10) chưa chạy trên xe.** Mới kiểm bằng: phát lại 14 lần đo thật qua node, và mô phỏng vòng kín `sil_rssi.py` (RSSI = mẫu đo thật phát lại, phòng = vòng quét LiDAR thật). Mô phỏng **không** có: trượt bánh khi xoay nhiều vòng liên tiếp, LiDAR tự ngừng gửi (7.3), mẫu RSSI ở cự ly < 1 m (mẫu hiệu chỉnh ở 2 m), phòng khác, người quay mặt về xe. `rssi_seek.py` là chỗ thứ hai (sau `_gap_maneuver`, mục 16) xe chạy bằng lệnh **không qua DWA** — chỉ đi thẳng/xoay tại chỗ, không lùi; lần đầu chạy thật nên để người đeo beacon đứng chỗ trống, không có đồ đạc giữa xe và người.

34. **`follow_planner_node` CHẾT nếu gọi `/follow/enable` khi đang ở `SEARCH` pha "đi tới"** (tái hiện 02/10, miền ROS riêng): `_enable_cb` đặt `last_target_odom = None` nhưng không xoá `search_goto` → nhịp điều khiển kế tiếp chạy `ox, oy = self.last_target_odom` → `TypeError` → node thoát, xe đứng (driver tự dừng sau 1 s). Với hệ camera: gọi `/follow/enable` lần nữa đúng lúc xe đang lái tới chỗ thấy người lần cuối là gặp. **Chưa sửa** (người dùng yêu cầu không đụng planner trong đợt RSSI). `rssi_follow.py` tránh bằng cách không bao giờ để planner đang bật thấy mục tiêu `valid=false`, và chờ 0.25 s sau `/follow/disable` (planner về `IDLE`) rồi mới bật lại. Sửa đề xuất, 1 dòng: thêm `self.search_goto = False` trong `_enable_cb`. **ĐÃ SỬA 07/10** (khi gộp RSSI vào planner theo yêu cầu "kết hợp camera và RSSI"): `_enable_cb` xoá `search_goto` và `search_phase`.

35. **`rssi_follow.py` (bám bằng RSSI + LiDAR, 02–03/10) — chạy thật lần đầu 06/10 (mục 8): 2 lần, không thấy va chạm, 3 lần mất dấu đều tự tìm lại.** Bản sửa ngay sau đó (06/10: khoá lại nhanh A, nhận kết quả dò B, so nền C — mục 8) đã chạy thật 06/10 chiều (122614, 123317, 123521, 154508): B đúng 5/5 lần, A khoá nhanh đúng 1 lần (3.1 s), không va chạm; sửa `guard()` báo nhầm mất `/odom` (mục 8) đã chạy thật 07/10: 280 s, 0 báo nhầm. Trước đó kiểm bằng mô phỏng vòng kín `sil_rssi.py` (planner thật, 2 node RSSI thật, RSSI = mẫu đo thật phát lại, phòng = hai vòng quét LiDAR thật). Từ 03/10 mô phỏng **có** hai hiệu ứng thật: **méo quét khi xe xoay** (LiDAR quét một vòng mất 0.1 s; xe xoay 0.86 rad/s thì điểm lệch tới ~5°, 0.17 m ở 2 m — tham số `--skew` của thế giới, mặc định như thật) và **odom đếm thiếu góc 1 %** (`--odom-yaw-scale 0.99`; khung odom lệch dần khỏi khung thế giới, bộ chấm điểm so trong khung thân xe). Mặt nạ "đồ đạc đứng yên" lúc bám lấy từ ảnh chụp khi xe **đứng yên**; nền 360° gom lúc xoay chỉ dùng để nhận "vật mới". Mô phỏng **không** có: (a) chiều quay thật của SC-Mini (giả định chỉ số tia tăng theo thời gian; `--skew -1` là chiều ngược) và độ trễ USB; (b) trượt bánh; (c) mẫu RSSI ở cự ly ~1 m — lần đo **kiểm tra** luôn diễn ra khi xe đứng sau người 1 m, trong khi mẫu hiệu chỉnh ở 2 m; (d) **người quay mặt về xe** (beacon sau thắt lưng bị thân che): vòng đo 1 trên xe cho sai tới 36–51° → vượt ngưỡng 40° hai lần liên tiếp là xe **bỏ đúng chủ** — khi thử phải quay lưng về xe lúc xe xoay, hoặc chạy `--verify-sec 0` trước; (e) chân người thật (ống quần rộng, đứng chụm chân), phòng khác, trượt bánh. Lệnh riêng của script (không qua DWA): xoay tại chỗ (cần trống 0.53 m quanh tâm quay) và **lùi thẳng** 0.06 m/s tối đa 0.45 m — chỉ khi dải LiDAR mù sau đuôi nằm trọn trong vùng thân xe vừa đi qua 60 s gần đây và hai bên đuôi không có điểm LiDAR; đây là chỗ **thứ ba** xe có thể lùi/chạy ngoài DWA (sau `_gap_maneuver` mục 16 và `rssi_seek.py` mục 33). Lần đầu chạy thật: đứng sẵn cạnh Ctrl-C / `/follow/stop`.

36. **Nhánh "người ra khỏi camera → xoay tại chỗ về phía người" giằng co với `AVOID`** (planner; lộ ra trong mô phỏng 03/10). Nguồn không bắt đầu bằng `camera` và người lệch > `occluded_turn_deg` (15°) → planner dừng lại xoay về phía người; nhưng muốn vòng qua vật cản thì mũi xe **phải** lệch khỏi hướng người quá 15° → xe xoay ngược lại, rồi lại né, cứ thế chúc mũi vào vật cản tới khi DWA chọn `v = w = 0` mãi (đúng hiện tượng 13.12). Kịch bản thùng lệch 0.3 m trên đường tới người (xe phải vòng qua): ngưỡng 15° kẹt 4/5 lần chạy; ngưỡng 60° kẹt 0/19 (xe né một mạch, hở vật 0.13 m). `rssi_follow.launch.py` đặt 60° (ở đó nguồn luôn là `rssi+lidar` nên nhánh này bật suốt). **`follow_nav_real.launch.py` vẫn 15°** — với camera, nhánh này chỉ bật khi nguồn là `lidar_track`/`predicted` (camera mất người **trong lúc đang né**), nên hiếm hơn nhưng cùng cơ chế; có thể là một nguyên nhân của 13.12 trên xe. **ĐÃ SỬA 07/10 cho hệ camera** đúng hướng đề xuất: nhánh xoay tại chỗ không chạy khi đang `AVOID` hoặc đang đi vòng theo bản đồ; `occluded_turn_deg` của hệ camera vẫn 15°. `rssi_follow.launch.py` vẫn đặt 60° (không đổi).

37. **Planner nhúc nhích xoay khi đã `ARRIVED` trong phòng nhiều đồ đạc** (mô phỏng 03/10, phòng dựng từ vòng quét LiDAR thật). `blocked_person` gần như luôn đúng — chính chân người nằm trong hành lang "tới người" — nên nhịp nào cũng gọi `_gap_target`; hai món đồ cách nhau 0.74–1.2 m bị coi là "khe" → `_gap_maneuver` ra lệnh xoay ~0.16 rad/s dù xe đã tới nơi, rồi "canh hướng tại chỗ" xoay ngược lại; lệnh xoay nhỏ nhất của driver ~0.2 rad/s nên lần nào cũng trôi quá vùng chết ±4° → xe cứ lắc nhẹ sau lưng người (mỗi 1–4 s một nhịp). `rssi_follow.py` tự chặn: đã tới nơi + người đứng yên thì thay lệnh planner bằng lệnh dừng, tới khi người lệch > 12° / đi tiếp. **Hệ camera không có lớp chặn này** — nếu trên xe thấy xe lắc nhẹ khi đứng sau người trong phòng nhiều đồ thì đây là nguyên nhân. **ĐÃ SỬA 07/10 trong planner:** chỉ chui khe khi chưa tới khoảng cách bám **hoặc** người đang đi > 0.15 m/s, bản đồ không báo `hold`, và khe nằm trên đường đi của bản đồ (6.3). Chưa có mô phỏng phòng nhiều đồ dựng từ vòng quét thật cho hệ camera để đo lại.

38. **Hình học dọc camera (07/10) dựa vào `camera_height_m` (0.34) và `camera_pitch_deg` (0 trong `identity_lock_kingsen.yaml`) đúng thực tế.** Sai lệch góc ngửa nhỏ được tự học từ đáy bbox mỗi lần `camera+lidar` khớp (cần ≥ 10 lần, thấy cả chân); **đổi độ cao camera (dự định 70 cm) thì phải sửa `camera_height_m`** — không tự học được. `person_height_m` 1.65 là giá trị đầu: người cao hơn mà đứng sau vật ngay từ đầu (chưa học) thì xe tưởng đã tới, dừng sớm (mô phỏng người 1.9 m: dừng ở 1.48 m thay vì ~1.1). **[CẦN XÁC NHẬN] trên xe:** đứng thoáng trước xe ~2 m vài giây, xem `/follow/target`: `elev_bias_deg` trong ±3°, `person_height_m` tiến về chiều cao thật, `cam_dist_m` gần `distance_m` (±15%). **08/10:** lần chạy đầu cho thấy camera thật **ngửa ~17°** (đo gián tiếp bằng LiDAR, 274 mẫu) mà `camera_pitch_deg` = 0 → xe chỉ xoay, không đi (7.2-AO). Tracker giờ tự đo và bù sai lệch tới ±35° (`_check_geometry`, 6.2) và in WARN kèm số độ cần sửa. **[CẦN XÁC NHẬN]** góc ngửa thật (đo bằng thước đo góc / app điện thoại, hoặc lấy số trong WARN) rồi sửa `camera_pitch_deg` trong `identity_lock_kingsen.yaml` — sửa đúng thì log báo `Hinh hoc camera khop LiDAR`, không còn phụ thuộc vào vài giây đầu người đứng thoáng cho tracker tự đo.

39. **RSSI trong `SEARCH` của planner (07/10) chưa chạy trên xe.** Mới kiểm bằng `sim_follow.py RSSI=1` (RSSI giả: sai ~15°, hợp lệ sau 250°) và `sil_rssi.py` (chỉ chế độ `rssi_follow`, planner thật nhưng `rssi_search_enabled: false`). Lần đầu: người đeo beacon đứng yên, quay lưng về xe; xe xoay tối đa 760° (~1.5 vòng) ở 0.9 rad/s mỗi lần dò, tìm tối đa 90 s; đứng sẵn cạnh `/follow/stop`. Chưa biết: ReID nhận lại khi xe vừa quay xong (ảnh nhoè lúc xoay 0.9 rad/s), hướng beacon khi người ở 1 m (mẫu hiệu chỉnh ở 2 m). **08/10:** chạy trên xe 2 lần (beacon có tín hiệu): cả hai lần camera thấy lại người sau ~100° xoay (ReID nhận lại được trong lúc xoay 0.9 rad/s) nên `SEARCH` kết thúc trước khi có hướng beacon — phần dùng hướng beacon (người khuất hẳn sau vách, B7) **vẫn chưa thử**; beacon phải được cấp nguồn ổn định trước (7.3).

40. **Bản đồ lưới coi xe là hình tròn bán kính `nav_lethal_m` 0.30 quanh trục trước** — đuôi 0.33 m và góc sau 0.446 m không có trong bản đồ. Lệnh thật vẫn qua DWA / `_arc_ok` (footprint chữ nhật) nên không đâm, nhưng đường bản đồ có thể là đường xe không đi được (cần xoay ở chỗ chật) → lúc đó `_nav_follow` xoay / lùi ngắn để vào đường. Đây là chỗ **thứ tư** xe có thể lùi (sau `_gap_maneuver` 16, `rssi_follow.py` 35): tối đa `gap_back_max_m` 0.40 m dùng chung, chỉ khi không tiến và không xoay được; trong 16 kịch bản khó chỉ ca người thứ hai đứng sát mũi xe mới cần lùi. **Chưa chạy trên xe thật.**

41. **Các lần chạy `follow_nav_real.launch.py` từ chiều 30/09 tới 07/10 dùng cấu hình camera CŨ** (`install/` giữ bản chép 30/09 11:44 của `identity_lock_kingsen.yaml`: FOV 62° pinhole, phơi sáng 20 ms — 7.3). Kết quả thử cửa thành công (07/10) và quan sát vật thấp là với góc camera sai (người lệch giữa khung báo thiếu ~40%). **[CẦN XÁC NHẬN]** người dùng có chạy node camera bằng cách khác (vd. `--params-file` trỏ thẳng `src/`) không; nếu không thì lần chạy tới là lần đầu camera mắt cá chạy cùng planner.

42. **Thay đổi planner 07/10 cũng chạy trong chế độ `rssi_follow` (không camera)** vì `rssi_follow.launch.py` dùng planner nguyên bản. `sil_rssi.py` bộ chính lần đầu sau thay đổi: 36/37 ĐẠT, `b_phong3b` bám nhầm 19.5 s (lần dò đầu sai +39°); chạy lại cùng kịch bản 2 lần mỗi bản (mới / 6d0aa54): 4/4 ĐẠT, chỉ số trùng nhau (mục 8); cả bộ chính với code cuối: 37/37 ĐẠT. Kết quả `sil_rssi.py` cho thấy bộ chính có biến động ngẫu nhiên (thời gian thực + mẫu RSSI phát lại) — một lần LỖI lẻ thì chạy lại kịch bản đó trước khi kết luận. Lần chạy thật kế tiếp của `rssi_follow.py` là lần đầu với planner mới.

43. **Camera thỉnh thoảng khoá nhầm người — do ReID, KHÔNG do YOLO (phân tích 09/10). ĐÃ SỬA 09/10 (7.2-AP): OSNet x0.5 + YOLO26n 416 bằng onnxruntime, ngưỡng mới; khoá nhầm trên dữ liệu thật 8.6% → 0.1%.** Người dùng (08/10, sau khi bám + né đã đạt) hỏi có nên nâng YOLOv5 lên YOLOv8 để hết khoá nhầm người khác áo quần, và mini PC sau này chỉ có **i5-5200U (2 lõi/4 luồng, Broadwell), 8 GB RAM**. Số liệu từ 2 bag thật chiều 08/10 (`~/bags/follow_1008_155118`, `_155900`, ~10 phút, 1–4 người trong khung): lúc bám đúng chủ, độ giống ReID với chủ trung vị 0.95 nhưng với **người khác** cũng 0.81–0.83 (p90 0.88–0.89); chênh `pos − max(neg, rival)` p10 chỉ **0.05**; màu áo quần của chính chủ chỉ ~0.65. **31 lần nhận lại (`REID_RECOVERED`), 16 lần sau đó chính hệ thống báo `TRACK_ID_SUSPECT`/`TRACK_VERIFY_HOLD` trong 0.8–12 s** (= xe bám nhầm ngần ấy giây); không ngưỡng đơn nào tách được (màu ≥ 0.55: chặn 14/16 nhầm nhưng chặn oan 11/15 lần giữ được; chênh ≥ 0.08: 11/16 và 4/15). → Gốc là **đặc trưng ReID của DeepSORT (`ckpt.t7`, học trên Market-1501) quá yếu với camera mắt cá thấp này**; YOLO chỉ báo "có người", đổi detector không đổi việc nhận ai là chủ. **Chi phí CPU đo trên máy này** (lớp `YoloDeepSortPipeline` thật, ảnh 640×480): YOLOv5n 640: 17 ms (16 luồng) / 22 ms (2 lõi) / 44 ms (1 lõi); 416: 10/11/16 ms; ReID DeepSORT 1 người 8/13/22 ms, **6 người 20/67/105 ms** — ReID tốn hơn YOLO khi có vài người. i5-5200U chậm hơn ~2.5–3 lần mỗi lõi → ước ~150–200 ms/khung khi có 3 người (~5 Hz, chưa tính planner/LiDAR chạy chung) — **[CẦN XÁC NHẬN] đo trên chính mini PC**. YOLOv8n tốn ~2 lần YOLOv5n (8.7 vs 4.5 GFLOPs; COCO mAP 37.3 vs 28.0); nếu sau này đổi detector thì YOLO26n (01/2026, NMS-free, CPU ONNX 38.9 ms so với YOLO11n 56.1, YOLOv8n 80.4 theo bảng Ultralytics) hợp CPU yếu hơn v8n. OpenVINO chính thức chỉ hỗ trợ Intel thế hệ 6+ (i5-5200U là thế hệ 5) → mini PC nên dùng ONNX Runtime. Gói `ultralytics` kéo numpy/OpenCV riêng, dễ xung đột ROS — chỉ dùng để xuất ONNX ở môi trường riêng. **Khuyến nghị đã báo người dùng:** giữ YOLOv5n; sửa ReID (mạng mạnh + nhẹ hơn, vd. OSNet trong `third_party/deep-person-reid-master`, đánh giá trên ảnh thật từ chính camera trước khi thay) + nhận lại có "thử thách"; tối ưu cho mini PC (ONNX Runtime, YOLO 416, ReID chỉ tính khi cần). Script phân tích: tạm trong scratchpad phiên 09/10 (đọc `/person_reid/target` từ bag: `reason`, `reid_similarity`, `negative_similarity`, `color_similarity`).

44. **Camera mới (09/10, 7.2-AP) chưa chạy trên xe; mini PC chưa đo.** Đã kiểm: dữ liệu thật quay từ chính camera (2 người khác màu áo quần rõ ràng, một phòng, một ánh sáng), chạy lại toàn bộ logic, node thật với video. **Chưa biết:** (a) người mặc **gần giống** chủ — dữ liệu chưa có ca này (OSNet tách tốt hơn rõ nhưng ngưỡng đặt theo 2 người khác màu); (b) ánh sáng khác / ngược sáng; (c) đông người hơn 2; (d) tốc độ thật trên i5-5200U (ước 10–15 Hz từ phép đo ghim 2 lõi, ×2.5–3); (e) `osnet_x0_25` nếu mini PC không kịp (bộ ngưỡng riêng trong README gói camera). Lần đầu trên xe: enroll như cũ, rồi thử đúng tình huống lỗi cũ — chủ đi khuất, người thứ hai đứng một mình trong khung — `status` phải là `LOST_REJECTED_CANDIDATE`, không được `REID_RECOVERED`. Nếu nhận lại chủ chậm hơn trước thì xem `reason` (điểm `reid`) trước khi hạ ngưỡng. Quay lại bản cũ: để trống `detector_onnx` / `reid_onnx` + trả các ngưỡng ghi "(ckpt.t7: …)" trong `identity_lock_kingsen.yaml`.
