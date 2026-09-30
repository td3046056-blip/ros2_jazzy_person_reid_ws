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
| **Chỉ MỘT node được publish `/cmd_vel`** | ROS 2 không trộn lệnh. Hai nguồn ghi cùng topic → xe nhận xen kẽ → giật cục. Hiện tại chỉ `follow_planner_node` được phép. |
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
| RSSI | 3 board ESP32/NodeMCU ở 0°/120°/240° — **hiện CHƯA cắm**, đang tắt trong config |

### Cổng thiết bị (dùng `by-id`, không cần udev rules)

```
Khung xe : /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30BF0NT-if00-port0
LiDAR    : /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
Camera   : /dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0
```

**Cảnh báo cho ngày cắm RSSI:** LiDAR và NodeMCU RSSI **đều là chip CH340** (`1a86:7523`) và **không có serial riêng**. Khi cắm cả hai, tên `by-id` sẽ trùng hệt và Linux chỉ giữ được một symlink. Hôm đó phải chuyển sang `by-path` hoặc chạy `scripts/setup_udev.py`.

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
├── setup.py               entry_points: target_tracker, follow_planner,
│                                        calibrate_lidar, calibrate_center
├── setup.cfg
├── README.md              hướng dẫn 6 giai đoạn + tinh chỉnh tham số
├── CALIBRATION.md         hồ sơ hiệu chỉnh với số liệu thật
├── config/
│   └── follow_nav.yaml    tham số cho CẢ HAI node
├── launch/
│   ├── follow_nav_real.launch.py   toàn bộ hệ thống (lidar+driver+camera+2 node)
│   ├── test_avoid_only.launch.py   lidar+driver+planner, KHÔNG camera
│   └── calibrate.launch.py         CHỈ lidar+driver (không planner)
├── person_follow_nav/
│   ├── geometry.py                 hàm dùng chung
│   ├── target_tracker_node.py
│   ├── follow_planner_node.py
│   ├── calibrate_lidar_node.py
│   └── calibrate_center_node.py
└── scripts/
    ├── test_sim.py            6 test offline (không cần ROS)
    ├── test_memory.py         so sánh bộ nhớ góc vs odom
    ├── test_self_filter.py    kiểm tra lọc thân xe
    ├── preflight.sh           kiểm tra trước khi cho xe chạy
    ├── fake_target.py         giả lập "người" để test không cần camera
    ├── measure_speed.py       đo tốc độ thật → max_linear / max_angular của driver
    ├── diagnose.py            chẩn đoán khi xe không né
    ├── run_calibration.sh     chạy 4 bước hiệu chỉnh, tự lưu log
    ├── setup_udev.py          tạo udev rules (xử lý CH340 trùng serial)
    └── check_devices.sh       chẩn đoán cổng USB
```

---

## 5. KIẾN TRÚC PHẦN MỀM

```
camera USB ─► person_follow_identity ─► /person_reid/target ─┐
SC-Mini    ─► sc_mini                ─► /scan ──────────────┤
NodeMCU    ─► rssi_serial_node       ─► /rssi/* ────────────┼─► target_tracker
BW-DR03    ─► decoded_serial_node    ─► /odom ──────────────┘        │
                                                                      ▼
                                                            /follow/target
                                                                      │
                            /scan, /odom ────────────────► follow_planner
                                                                      │
                                                                 /cmd_vel
                                                                      ▼
                                                            decoded_serial_node
```

### Topic

| Topic | Kiểu | Nguồn → Đích |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | sc_mini → tracker, planner, diagnose |
| `/odom` | `nav_msgs/Odometry` | bw_dr03 → tracker, planner |
| `/person_reid/target` | `std_msgs/String` (JSON) | person_follow_identity → tracker |
| `/rssi/angle_deg`, `/rssi/confidence` | `std_msgs/Float32` | rssi_serial_node → tracker |
| `/follow/target` | `std_msgs/String` (JSON) | tracker → planner |
| `/follow/target_marker` | `visualization_msgs/Marker` | tracker → RViz |
| `/follow/planner_status` | `std_msgs/String` (JSON) | planner → giám sát |
| `/follow/debug_markers` | `visualization_msgs/MarkerArray` | planner → RViz |
| `/cmd_vel` | `geometry_msgs/Twist` | **CHỈ** planner → bw_dr03 |
| `/bw_dr03/sonar` | 2 sonar | bw_dr03 → **chưa dùng** |

### Service

| Service | Kiểu | Node | Tác dụng |
|---|---|---|---|
| `/follow/enable` | Trigger | planner | Bật bám |
| `/follow/disable` | Trigger | planner | Tắt bám, dừng xe |
| `/follow/stop` | Trigger | planner | **Dừng khẩn cấp** |
| `/follow/reset_tracker` | Trigger | tracker | Xoá bộ nhớ vị trí người |
| `/person_reid/start_enroll` | Trigger | identity | Bắt đầu học người |
| `/person_reid/finish_enroll` | Trigger | identity | Kết thúc học |

### Payload JSON

**`/person_reid/target`** (đọc bởi tracker): `camera_angle_deg`, `bbox` `[x1,y1,x2,y2]`, `target_found`, `identity_ready`, `status`, `ts` (dùng bù độ trễ — từ 30/09 là **lúc chụp khung**, trước đó là lúc xử lý xong nên bù gần như bằng 0). Thêm từ 30/09 (tracker không đọc): `occlusion`, `evidence`, `view_bucket`, `latency_ms`. `status` có thêm `TRACK_VERIFY_HOLD` = vẫn giữ khoá nhưng nghi là người khác nên `target_found: false`.

**`/follow/target`** (tracker xuất): `stamp`, `status`, `valid`, `source`, `measured_this_tick`, `confidence`, `age_since_fix_sec`, `odom_frame`, `robot{x,y,yaw}`, `odom_x`, `odom_y`, `vx`, `vy`, `speed`, `base_x`, `base_y`, `distance_m`, `bearing_rad`, `bearing_deg`, `in_camera_fov`.

**`/follow/planner_status`**: `stamp`, `enabled`, `state`, `note`, `cmd_v`, `cmd_w`, `target_distance_m`, `target_bearing_deg`, `target_source`, `avoid_side`, `clearance_m`, `chosen_heading_deg`, `front_clearance_m`, `n_obstacles`.

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

**Camera không thấy người** (`source` không bắt đầu bằng `camera` — tức `lidar_track`, `predicted`, `rssi_bearing`) và lệch > `occluded_turn_deg` (15°) → **xoay tại chỗ về phía đó trước** (trễ: tới dưới 7.5°), rồi mới tiến. Chỉ khi `_can_turn()` (quét footprint đúng góc cần xoay). Tốc độ `clip(2.2·góc, ±0.9·w_max)` — trước 29/09 là `1.5·góc, ±0.6·w_max` = tối đa 0.48 rad/s, người dùng thấy xoay theo người chậm. Ngưỡng thấp hơn nửa FOV vì góc dự đoán thường trễ hơn góc thật ~10°.

**SEARCH (sửa 17/09)** tính từ lần cuối có mục tiêu **hợp lệ** (`last_valid_time`), bắt đầu ngay khi mất nếu đã lưu vị trí người:
1. **Pha đi tới:** lái (chọn khe + DWA) tới cách chỗ thấy người lần cuối (`last_target_odom`, khung odom) một `follow_distance`, tối đa `search_goto_max_sec` (8 s). Chỉ xoay tại chỗ khi đích lệch > 60°. **Đường tới chỗ người bị chắn** (người thứ hai đứng chen che mất người đang bám) thì coi như chưa tới nơi và dò hướng tới tận chỗ người (như `chk_r`) để **vòng qua** — trước 29/09 điểm đích rơi đúng chỗ người thứ hai, xe bò tới sát chân họ rồi đứng quay tìm (7.2-AE).
2. **Pha quét:** xoay qua lại như cũ. Hết `search_max_sec` hoặc không đủ chỗ xoay → `IDLE`, `last_valid_time = 0` → không quét lại tới khi thấy người. `/follow/enable` xoá vị trí đã lưu.

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
| U | **Xe lắc qua lại khi bám thẳng** | Người dùng báo: *"không bám thẳng theo người"*. Mô phỏng: `w` đổi chiều 42 lần / 22 s | Vùng chết hướng → 25 lần. Còn lắc nhẹ do góc cụm chân rung |

### 7.3 Lỗi môi trường / build

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `CMake Error: CMakeCache.txt directory is different` | Workspace bị di chuyển qua 4 vị trí và đổi user (`thaihoa` → `thach`). CMake ghi đường dẫn tuyệt đối vào cache. Chỉ ảnh hưởng package `ament_cmake` (`sc_mini`, `robot_simulation`), package Python build bình thường. | `rm -rf build install log` rồi build lại |
| `Can't access port /dev/robot_lidar` | `99-robot-usb.rules` dùng `KERNELS=="1-2.3"` — số hiệu cổng USB vật lý của **máy cũ** | Dùng đường dẫn `by-id` (xem mục 1) |
| `AMENT_PREFIX_PATH ... doesn't exist` | Shell còn giữ biến môi trường sau khi `rm -rf install` | Mở terminal mới |
| `-Wunused-variable` của `sc_mini` | Code gốc nhà sản xuất | Vô hại, bỏ qua |

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

**Kết quả 17/09:** bước 1–2 đạt — `/person_reid/target` 8.0 Hz, `/cmd_vel` ~14.8 Hz (ReID chạy CPU), `source = camera+lidar`. Chạy thử bám người, người dùng báo 3 vấn đề: xe không bám thẳng (lắc), rẽ theo không kịp, và khi `predicted` xe đi thẳng thay vì quay về hướng thấy người lần cuối. Đã sửa tracker + planner (lỗi 7.2-Q…U), kiểm bằng `scripts/sim_follow.py`. **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**

**Kết quả 22/09 (xe thật, sau bản 17/09):** người dùng xác nhận bám người "cải thiện rất tốt"; có `SEARCH` → `FOLLOW` khi mất rồi thấy lại. Lỗi còn: **người thứ hai chủ động bước vào giữa** thì xe đa số lần chạy thẳng tới chân họ dù camera vẫn thấy người đang bám; họ bước ra thì bám tốt lại. Log nguồn lúc đó nhảy `lidar_track`/`camera+bbox`. Đã sửa (7.2-V), kiểm bằng `sim_follow.py` kịch bản `chan_giua`/`cat_ngang`/`chan_sat`. **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**

**29/09 (xe thật, bản `bc3dcdf`):** người dùng báo ba việc: (1) cửa 0.81 m khi người rẽ vào: xe gần như ở giữa cửa, thực tế lọt, nhưng đứng im với `note` "qua khe theo truc (lech -3..-5 do)", lúc qua lúc không — muốn sửa tuyệt đối; (2) xoay theo người hơi chậm so với tốc độ đi bình thường; (3) người thứ hai chủ động bước vào: lúc né, lúc bò tới sát chân họ. Cả ba tái hiện được và đã sửa (7.2-AD, AE, AF). Mô phỏng thêm tường dày (`WALL_T`), nhiễu LiDAR (`LIDAR_NOISE`), kịch bản người thứ hai tham số hoá (`chen`) và người đi 1 m/s (`vong_nguoi_*`). **[CẦN XÁC NHẬN] chưa chạy lại trên xe.**

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
ros2 launch person_follow_nav follow_nav_real.launch.py

ros2 service call /person_reid/start_enroll  std_srvs/srv/Trigger {}
# đi quanh camera 20–30 giây
ros2 service call /person_reid/finish_enroll std_srvs/srv/Trigger {}
ros2 service call /follow/enable             std_srvs/srv/Trigger {}
```

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

5. **Quy ước góc RSSI** (`rssi_angle_sign: -1.0`) **chưa kiểm chứng** — mạch RSSI chưa cắm. Cách kiểm tra: đứng **bên phải** xe, nếu `/rssi/angle_deg` dương thì `-1.0` đúng.

6. **Hai sonar `/bw_dr03/sonar`** — chưa đọc, chưa biết định dạng và độ tin cậy.

7. **YOLO/ReID đang chạy CPU** (kiểm tra 17/09). Máy có GPU NVIDIA rời (PCI `2d59`, dòng RTX 50) + Radeon tích hợp, nhưng `nvidia-smi` báo *"No devices were found"* (driver không nhận GPU) và torch cài là `2.12.0+cpu` → `core.choose_device("auto")` rơi về CPU. **Chưa đo tốc độ thật** (`ros2 topic hz /person_reid/target`). Chỉ cần xử lý GPU nếu tốc độ không đủ — việc đó cần sửa driver NVIDIA (sudo, khởi động lại) và cài torch bản CUDA: **người dùng quyết định**.

8. **Nội dung chi tiết của `person_follow_identity`** — đã đọc `node.py` và `core.py` để lấy định dạng payload, nhưng **chưa kiểm chứng toàn bộ các trường** ở runtime.

9. **Độ chính xác odom** (`encoder_ppr_default: 750`) — **quãng đường đã kiểm chứng** (16/09): thước 1.50 m / odom 1.503 m. **Góc xoay: odom đếm thiếu ~3%** (thật 135° / odom 130.9°; 34° / 33°) → `wheel_separation` hiệu dụng ≈ 0.388 thay vì 0.40. **Chưa đổi** — đọc góc bằng mắt sai 1–3°, cần xoay ~2 vòng (`measure_speed.py 0.0 --w 0.8 --sec 15`) để chốt. Nếu đổi thì tính lại `max_angular` (≈ 2.52), vì 2.46 lấy từ odom.

10. **ĐÃ XÁC NHẬN 17/09 — lỗi "LiDAR không xoay, xe chỉ chạy thẳng" không còn.** `diagnose.py` cho thấy LiDAR quay và planner dùng dữ liệu; xe thật đã né được vật cản với `fake_target.py 2.5` (xem mục 8). Nguyên nhân cũ chưa rõ — có thể do lỗi tốc độ 7.2-P.

11. **Hành vi thực địa của thuật toán** — giai đoạn 3 (né vật cản với mục tiêu giả) **đã đạt 6/6 trên xe thật 17/09** (mục 8). Phần có camera/ReID (giai đoạn 4 trở đi) **chưa chạy thật**.

12. **Planner kẹt đứng yên vĩnh viễn cạnh vật cản (thấy trong mô phỏng, chưa sửa).** Test 5 của `test_sim.py` chỉ ĐẠT với đúng kịch bản gốc. Dời người chen ngang 5 cm, hoặc cho người đi 0.10 m/s, thì 11/18 biến thể xe dừng cách người chen ~0.28 m rồi đứng yên mãi ở `AVOID` (DWA chọn v=w=0). `stuck_time_sec` được khai báo nhưng không dùng → xe thật không có cơ chế thoát kẹt. Hạ `v_max` xuống ≤ 0.21 cũng làm test 5 `LOI` vì cùng lý do. **Trên xe thật 17/09 chưa gặp**: vật cản giữa đường, 3.3/3.4 (thùng lệch 20 cm) đều né được. Vẫn có thể lộ ra với người đi chậm ở giai đoạn 4.

13. **ĐÃ XÁC NHẬN 17/09 — bánh quay được ở 5% PWM.** Lệnh nhỏ nhất của planner (`min_move_linear 0.035`, `min_move_angular 0.10`) đều ra 5%. Đo: `measure_speed.py 0.035` → thước 11.5 cm / odom 11.4 cm; `--w 0.10` → thật 34° / odom 33°. Giữ nguyên hai tham số.

14. **Sửa bám người 17/09 (7.2-Q…U) mới kiểm bằng mô phỏng vòng kín, chưa chạy xe thật.** Số liệu `sim_follow.py` (camera ±25°), gốc → mới: đi thẳng `w` đổi chiều 42 → 25 lần; bước ngang 1 m/s mất camera lâu nhất 0.7 → 0.2 s; ra khỏi khung (LiDAR mất chân) lệch trung bình 6.0° → 3.6°; rẽ gắt 0.5 m/s và đi chéo 0.8 m/s không đổi. **Góc tường gắt** (người rẽ sát mép tường, nhanh gấp đôi xe): xe giờ quay về phía người, vào SEARCH và lái tới góc, nhưng **vẫn không thấy lại người** — tới góc thì mép tường trong 0.53 m nên `_can_rotate_in_place()` (đòi trống cho cả vòng 360°) không cho quét → `IDLE`. Có thể cần kiểm tra quét theo góc xoay thực (footprint quét trong DWA) thay vì cả vòng. **26/09:** các nhánh xoay về một hướng cụ thể đã chuyển sang `_can_turn` (quét đúng góc); riêng pha quét của `SEARCH` vẫn dùng luật 360°. Góc tường thoáng hơn thì cả code cũ lẫn mới đều không mất người.

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
