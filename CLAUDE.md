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
2. **Camera cố định không xoay được** — FOV chỉ 62°, khi xe quay để né thì người ra khỏi khung hình.
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
| Camera | KINGSEN USB, **cố định hướng thẳng trước, không xoay được**, FOV 62° |
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

**`/person_reid/target`** (đọc bởi tracker): `camera_angle_deg`, `bbox` `[x1,y1,x2,y2]`, `target_found`, `identity_ready`, `status`, `ts` (dùng bù độ trễ).

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
- `admissible = min_clear > margin_hard`

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
| tie-break | `1e-3` | `\|W\|/w_max` — ưu tiên đi thẳng khi hoà |

**Không có ràng buộc phanh riêng.** Chân trời 1.2 s đã đủ: thời gian phanh `v_max/accel = 0.63 s`, quãng đường phanh `v²/2a = 0.069 m` < quãng đường mô phỏng 0.26 m.

**Máy trạng thái:** `IDLE`, `FOLLOW`, `AVOID`, `OCCLUDED`, `SEARCH`, `ARRIVED`, `BLOCKED`, `ESTOP`.

**Đường bị chắn tính tới tận chỗ người (sửa 22/09):** `segment_blocked` và tầng chọn khe kiểm tra tới `max(goal_r, dist − target_clear_radius_m)` (0.45 m), không chỉ tới điểm đích. Đích cách người `follow_distance` nên người thứ hai chen vào thường đứng **đúng tại đích** — trước đây nằm ngoài đoạn xe→đích nên không bị coi là chắn, xe cứ đi thẳng tới họ. Dịch phụ của DWA vẫn giới hạn trong `goal_r`.

**Vùng chết hướng (sửa 17/09):** người lệch < `bearing_deadband_deg` (4°) → đích coi như thẳng trước mũi (`goal_b = 0`). Góc cụm chân rung vài độ mỗi bước; không có vùng chết thì DWA bẻ lái ±0.1 rad/s liên tục.

**Camera không thấy người** (`source` không bắt đầu bằng `camera` — tức `lidar_track`, `predicted`, `rssi_bearing`) và lệch > `occluded_turn_deg` (15°) → **xoay tại chỗ về phía đó trước** (trễ: tới dưới 7.5°), rồi mới tiến. Chỉ khi `_can_rotate_in_place()`. Ngưỡng thấp hơn nửa FOV vì góc dự đoán thường trễ hơn góc thật ~10°.

**SEARCH (sửa 17/09)** tính từ lần cuối có mục tiêu **hợp lệ** (`last_valid_time`), bắt đầu ngay khi mất nếu đã lưu vị trí người:
1. **Pha đi tới:** lái (chọn khe + DWA) tới cách chỗ thấy người lần cuối (`last_target_odom`, khung odom) một `follow_distance`, tối đa `search_goto_max_sec` (8 s). Chỉ xoay tại chỗ khi đích lệch > 60°.
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
| Xe quá rụt rè trong hành lang | Giảm `margin_soft` |
| Xe cọ tường | Tăng `margin_hard` |

---

## 12. VẤN ĐỀ THỰC TẾ CẦN LƯU Ý

**Odom trôi.** Encoder BW-DR03 trôi 2–5% quãng đường, chưa kể trượt bánh. Bộ nhớ odom chỉ dùng trong 2.5 s nên trôi tối đa vài cm — chấp nhận được. **Đừng tăng `predict_max_sec` quá 4 s.**

**LiDAR quét ở một độ cao cố định (18 cm).** Thấy chân người, chân bàn — **không thấy** mặt bàn nhô ra, bậc thềm, dây điện. Hạn chế vật lý. Hai sonar sẵn có trên BW-DR03 (`/bw_dr03/sonar`) có thể bù cho vật thấp — **chưa dùng**.

**Người là vật cản động.** Người đi cắt ngang 1.2 m/s trong khi xe chạy 0.22 m/s — xe không "né" kịp theo nghĩa lập kế hoạch, nó chỉ chậm lại và chờ. Điều này đúng và an toàn. Đừng tăng tốc độ để né nhanh hơn.

**Camera FOV 62° là hẹp.** Ở 1.0 m, trường nhìn ngang chỉ 1.2 m. Người bước sang ngang 0.6 m là ra khỏi khung. Nếu được đổi phần cứng, camera FOV 90–120° cải thiện nhiều hơn bất kỳ tinh chỉnh thuật toán nào.

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

14. **Sửa bám người 17/09 (7.2-Q…U) mới kiểm bằng mô phỏng vòng kín, chưa chạy xe thật.** Số liệu `sim_follow.py` (camera ±25°), gốc → mới: đi thẳng `w` đổi chiều 42 → 25 lần; bước ngang 1 m/s mất camera lâu nhất 0.7 → 0.2 s; ra khỏi khung (LiDAR mất chân) lệch trung bình 6.0° → 3.6°; rẽ gắt 0.5 m/s và đi chéo 0.8 m/s không đổi. **Góc tường gắt** (người rẽ sát mép tường, nhanh gấp đôi xe): xe giờ quay về phía người, vào SEARCH và lái tới góc, nhưng **vẫn không thấy lại người** — tới góc thì mép tường trong 0.53 m nên `_can_rotate_in_place()` (đòi trống cho cả vòng 360°) không cho quét → `IDLE`. Có thể cần kiểm tra quét theo góc xoay thực (footprint quét trong DWA) thay vì cả vòng. Góc tường thoáng hơn thì cả code cũ lẫn mới đều không mất người.

15. **Tham số khai báo nhưng không dùng:** `fov_cost_only_when_visible` (DWA luôn bật chi phí FOV) và `stuck_time_sec` (xem 13.12). Chưa sửa.

16. **Người thứ hai đứng SÁT trước người đang bám (< `lidar_distance_max_jump_m` = 0.6 m)** — cổng chặn không phân biệt được hai người; mô phỏng `chan_sat` (0.4 m): tracker vẫn bám nhầm 65% thời gian, xe dừng cách người thứ hai ~1 m và chờ, họ đi thì bám lại đúng. Chấp nhận được (không va chạm) nhưng chưa giải quyết. Khi xe vòng qua người thứ hai, khe hở với chân họ có thể chỉ cỡ `margin_hard` (6 cm) — margin đã hiệu chỉnh (mục 3), không đổi tự ý.
