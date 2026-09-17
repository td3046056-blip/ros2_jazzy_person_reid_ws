# person_follow_nav

Bám người + né vật cản cho robot BW-DR03, dùng camera ReID + LiDAR 2D + RSSI.

Package này **thay thế** `person_follow_robot/follow_controller` và
`robot_rssi_ros2/rssi_follow_node`. Không chạy đồng thời cả ba — xem mục "Vấn đề #3".

---

## 1. Tại sao xe hiện tại không né được vật cản

Tôi đã đọc toàn bộ workspace. Đây là 8 vấn đề tìm thấy, xếp theo mức nghiêm trọng.

### Vấn đề #1 — Bộ nhớ mục tiêu lưu sai hệ quy chiếu (lỗi gốc rễ)

`follow_controller.py` giữ `last_valid_payload` trong 1.5 s khi mất target, rồi
dùng lại `camera_angle_deg` cũ. Nhưng `camera_angle_deg` là **góc so với thân xe**,
không phải vị trí trong không gian.

Ngay khi xe quay để né, góc đó sai. Đo bằng mô phỏng (`test7.py`):

| Thời điểm | Yaw xe | Góc THẬT tới người | Code cũ nhớ | Sai số |
|---|---|---|---|---|
| 0.0 s | 0° | 0.0° | 0.0° | 0.0° |
| 0.6 s | 17.5° | −17.9° | 0.0° | **17.9°** |
| 1.2 s | 35.0° | −36.9° | 0.0° | **36.9°** |

Camera của bạn có FOV 62°, tức nửa trường nhìn ±31°. Chỉ cần né 1.2 giây là sai số
36.9° đã **vượt quá** nửa trường nhìn. Xe quay theo góc nhớ sai → người ra khỏi
khung hình → mất hẳn, không bao giờ bắt lại được.

**Cách sửa:** lưu vị trí người ở khung `odom` (cố định với mặt đất). Mỗi chu kỳ tính
lại góc tương đối từ odom hiện tại. Sai số còn **0.0°**. Đây là việc `target_tracker`
làm.

### Vấn đề #2 — Hai node cùng ghi `/cmd_vel`

`person_follow_controller` và `rssi_follow_node` đều publish `/cmd_vel`. ROS 2 không
trộn lệnh — xe nhận xen kẽ hai luồng, kết quả là giật cục. Chính comment v1.9 trong
`follow_controller.py` của bạn đã mô tả đúng hiện tượng này:

> *"lidar_follow_enhancer also publishes to cmd_vel_topic while it is disabling us
> ... the two publishers race on the same topic and the avoidance motion comes out
> jerky/stuttering"*

Package này chỉ có **một** node ghi `/cmd_vel`.

### Vấn đề #3 — Code VFH được mô tả trong tài liệu nhưng không tồn tại

`robot_rssi_ros2/SYSTEM_CONTEXT.md` mô tả chi tiết:

- `robot_rssi_ros2/vfh_rssi_controller.py`
- `config/vfh_params.yaml`
- `launch/vfh_test_only.launch.py`
- `launch/rssi_vfh_follow.launch.py`

**Không file nào có trong zip.** Toàn bộ công sức VFH (Bug#2 → Bug#6, anti-oscillation,
BACKTRACK thoát bẫy chữ U) đã mất hoặc chưa commit. `follow_controller.py` cũng tham
chiếu `lidar_follow_enhancer` — cũng không có.

Package này dựng lại phần đó, có kèm test chứng minh hoạt động.

### Vấn đề #4 — Mô hình hình tròn khiến xe không chui được khe hẹp

Xe bạn rộng **0.46 m**, dài **0.57 m** (đọc từ `robot_core.xacro`).

| Mô hình | Khe tối thiểu cần | Khe 0.75 m |
|---|---|---|
| Hình tròn (bán kính ngoại tiếp 0.366 m) | 0.73 m | Sát nút, thực tế từ chối |
| Chữ nhật thật | 0.48 m | Lọt, dư 0.27 m |

Thêm vào đó `emergency_distance: 0.50 m` của bạn khiến xe dừng trước mọi khe. Package
này dùng footprint chữ nhật thật (`rect_clearance`) — test đã chứng minh xe chui được
khe 0.75 m với độ thoáng 10.8 cm.

### Vấn đề #5 — Khoảng cách từ bbox height rất nhiễu

`_bbox_distance_m()` tính `d = target_distance × ref_px / h`. Chiều cao bbox dao động
mạnh khi người bị che một phần, cúi, giơ tay — và phải hiệu chỉnh tay
(`bbox_height_at_target_distance_px: 420.0`).

Bạn có LiDAR đo được ±2 cm. Package này lấy **góc từ camera** (chính xác ±1°) và
**khoảng cách từ LiDAR** tại đúng góc đó. Không cần hiệu chỉnh gì.

### Vấn đề #6 — Lọc nhiễu LiDAR bỏ sót vật cản mảnh

```python
values.sort()
idx = 1 if len(values) >= 8 else 0
return values[idx]        # lấy giá trị nhỏ THỨ HAI
```

Chân ghế, chân bàn, cột nhỏ thường chỉ tạo 1–3 tia. Lấy giá trị nhỏ thứ hai là bỏ
qua chúng. Package này giữ **toàn bộ** điểm trong bán kính 1.0 m
(`downsample_polar(keep_all_within_m=1.0)`).

### Vấn đề #7 — Lỗi khai báo package

- `robot_rssi_ros2/package.xml` thiếu `<depend>sensor_msgs</depend>` dù node subscribe
  `LaserScan` → `rosdep install` sẽ không cài, build sạch sẽ lỗi.
- `robot_rssi_ros2/setup.py` có `glob('config/*.yaml')` nhưng thư mục `config/` không
  tồn tại.
- `package.xml` dùng thẻ `<n>` thay vì `<name>` — không đúng schema.

### Vấn đề #8 — `max_percent` không nhất quán

`rssi_follow.launch.py` đặt `max_percent: 30`, nhưng `SYSTEM_CONTEXT.md` ghi rõ:

> *"Driver BW-DR03 mặc định 30% → tăng lên 60% để bánh xe quay được"*

Launch file trong package này dùng 60.

---

## 2. Kiến trúc mới

```
camera USB ─► person_follow_identity ─► /person_reid/target ─┐
SC-Mini    ─► sc_mini               ─► /scan ───────────────┤
NodeMCU    ─► rssi_serial_node      ─► /rssi/* ─────────────┼─► target_tracker
BW-DR03    ─► decoded_serial_node   ─► /odom ───────────────┘        │
                                                                      ▼
                                                            /follow/target
                                                        (vị trí người trong odom)
                                                                      │
                            /scan, /odom ─────────────────► follow_planner
                                                                      │
                                                                 /cmd_vel
                                                                      ▼
                                                            decoded_serial_node
```

### `target_tracker` — trả lời "người đang ở đâu trong khung odom?"

Nguồn dữ liệu, ưu tiên giảm dần:

| # | Nguồn | Độ chính xác | Khi nào dùng |
|---|---|---|---|
| 1 | Camera + LiDAR ghép | Góc ±1°, khoảng cách ±2 cm | Bình thường |
| 2 | Camera + bbox | Góc ±1°, khoảng cách ±20 cm | LiDAR không thấy (người sau bàn) |
| 3 | LiDAR bám cụm | ±2 cm | Camera bị che nhưng chân người còn thấy |
| 4 | Dự đoán alpha-beta | Trôi dần | Bị che hoàn toàn, tối đa 2.5 s |
| 5 | RSSI bearing | ±20° | Mất > 1.2 s, chỉ sửa hướng |

Node cũng **bù độ trễ YOLO/ReID** bằng lịch sử odom. YOLO + DeepSORT + ReID trễ
80–250 ms; khi xe quay 0.5 rad/s thì 250 ms = 7° sai lệch, đủ để bám lệch.

### `follow_planner` — nguồn duy nhất ghi `/cmd_vel`

Hai tầng:

**Tầng 1 — chọn khe (kiểu VFH+).** Quét các hướng từ −100° đến +100° mỗi 4°, kiểm
tra hướng nào đi được 1.6 m mà footprint không chạm gì. Chọn hướng rẻ nhất theo:
gần hướng người + giữ hướng đã chọn lần trước + giữ người trong khung hình + tôn
trọng bên né đã cam kết.

**Tầng 2 — DWA.** Sinh ~130 cặp `(v, ω)`, mô phỏng 1.2 s, chấm điểm và chọn tốt nhất.

Tại sao cần cả hai: DWA thuần chỉ nhìn trước 1.2 s ≈ 0.26 m ở tốc độ xe bạn. Khi có
người đứng chắn, DWA thấy "đi thẳng thì đụng, rẽ thì cũng không gần đích hơn" → chọn
đứng im. Đây là bẫy cực tiểu địa phương kinh điển. Tầng chọn khe nhìn xa 1.6 m nên
thấy được đường vòng.

---

## 3. Ba kỹ thuật cốt lõi giải quyết đúng câu hỏi của bạn

### 3.1 Goal đặt lùi — giải quyết "Bug#4" trong ghi chú của bạn

Ghi chú của bạn:

> *"Bug#4: `_front_obstacle()` exclude target → gây xe lao vào vật cản sau người →
> đổi sang không exclude"*

Loại người khỏi danh sách vật cản thì xe đâm vào thứ phía sau người. Không loại thì
xe coi người là tường và dừng từ xa. Cả hai đều sai.

Cách sửa sạch: **không loại gì cả**, nhưng đặt đích cách người đúng `follow_distance`
về phía xe:

```python
goal_r = max(0.0, dist - self.follow_distance)
gx = goal_r * math.cos(bearing)
gy = goal_r * math.sin(bearing)
```

Lực kéo của đích và lực đẩy của vật cản tự cân bằng tại đúng 1.0 m. Không cần logic
đặc biệt nào.

### 3.2 Margin thích nghi — giải quyết "né vừa đúng khoảng cách để tối ưu không gian hẹp"

Hai ngưỡng thay vì một:

- `margin_hard = 0.07 m` — tuyệt đối không phạm, xe từ chối mọi quỹ đạo vi phạm
- `margin_soft = 0.22 m` — mong muốn, chi phí tăng bậc hai khi xuống dưới

Ở chỗ rộng xe giữ 22 cm cho thoải mái. Trong khe hẹp, ngưỡng mềm **tự hạ xuống mức
tốt nhất mà môi trường cho phép**:

```python
prog = d_start - d_end
useful = admissible & (prog >= max(0.30 * best_prog, 0.02))
c_ref = np.percentile(min_clear[useful], 80)
soft = np.clip(c_ref, margin_hard + 0.02, margin_soft)
```

Chi tiết quan trọng: chỉ lấy mẫu trong nhóm **có tiến về đích**. Nếu lấy cả những quỹ
đạo quẹo ra chỗ rộng (thoáng 0.4 m) thì ngưỡng tham chiếu bị kéo lên 0.4 m, khe hẹp
0.13 m vẫn bị phạt nặng, và xe vẫn dừng trước khe. Tôi đã gặp đúng lỗi này khi test
và phải sửa.

Tốc độ cũng tự giảm theo độ thoáng: chỗ hẹp đi chậm, chỗ rộng đi nhanh. Không có mode
switch nào.

### 3.3 Chi phí giữ khung hình — giải quyết "camera cố định không xoay được"

Thêm một thành phần chi phí phạt các quỹ đạo đẩy người ra ngoài ±22°:

```python
tb = wrap_pi(atan2(ty - ye, tx - xe) - te)
c_fov = clip((abs(tb) - fov_keep_rad) / (pi - fov_keep_rad), 0, 1)
```

Khi có hai đường né tương đương, xe chọn đường vẫn "liếc" thấy người. Khi buộc phải
né sang bên làm mất hình, bộ nhớ odom giữ vị trí người và xe tự quay lại đúng hướng
sau khi né xong.

---

## 4. Kết quả test offline

Chạy `python3 test_sim.py` (không cần ROS, chỉ cần numpy).

| # | Test | Kết quả |
|---|---|---|
| 1 | Đổi góc LiDAR → base_link (lidar 90° → 0°, lidar 180° → +90°) | ĐẠT |
| 2 | `rect_clearance` footprint chữ nhật, 4 trường hợp | ĐẠT |
| 3 | `segment_blocked` phát hiện người chen ngang tại 1.00 m | ĐẠT |
| 4 | Alpha-beta dự đoán khi bị che 1.5 s, sai số 0.00 m | ĐẠT |
| 5 | Né người chen ngang rồi bám tiếp | ĐẠT |
| 6 | Chui khe hẹp 0.75 m | ĐẠT |
| 7 | So sánh bộ nhớ góc tương đối vs odom | ĐẠT |
| 8 | Self-filter chặn lỗi kẹt `BLOCKED` vĩnh viễn | ĐẠT |
| 9 | `blind_sectors_deg` định dạng phẳng, lọc đúng dữ liệu thật | ĐẠT |

**Test 5 chi tiết** — người đi 0.15 m/s, người lạ đứng chắn tại (2.0, 0), hành lang
rộng 2.2 m:

```
 t(s)   xe_x   xe_y  yaw   d_người  góc    v      w     trạng thái
  2.0   0.39   0.02   10    2.93   -9.6  0.220  0.178  AVOID/trái
  6.0   1.23   0.27   30    2.70  -34.7  0.220  0.179  AVOID/trái
 10.0   1.82   0.59   15    2.77  -27.2  0.147 -0.130  AVOID/trái
 12.0   2.11   0.64    0    2.78  -14.1  0.185 -0.161  FOLLOW
 16.0   2.91   0.52  -11    2.57   -0.3  0.220 -0.000  FOLLOW
 26.0   5.08   0.15   -7    1.85    2.1  0.220  0.000  FOLLOW

 Khoảng cách gần nhất tới người chắn: 0.416 m  (giới hạn cứng 0.07 m)
 Đã vượt qua được người chắn: True
```

Xe vòng trái, giữ 41.6 cm với người chắn, quay lại bám và thu hẹp khoảng cách từ
2.93 m xuống 1.85 m. Đúng hành vi mong muốn.

**Test 6** — xe rộng 0.48 m chui khe 0.75 m: qua được sau 163 bước (≈11 s), độ thoáng
nhỏ nhất 10.8 cm.

### Hai lỗi thiết kế mà test phát hiện

Ghi lại vì có thể hữu ích cho báo cáo của bạn.

**Lỗi A — cửa sổ lấy mẫu DWA quá hẹp.** Ban đầu tôi lấy mẫu vận tốc trong cửa sổ
`accel × 1/control_hz = 0.35 × 0.067 = 0.023 m/s`. Lợi ích tiến lên trong 1.2 s chỉ
2.8 cm:

```
c_goal(v=0)      = 1.0000  → ×2.4 = 2.4000
c_goal(v=v_hi)   = 0.9860  → ×2.4 = 2.3664
LỢI ÍCH khi tăng tốc: 0.0336
c_smooth(v=v_hi) = 0.5000  → ×0.7 = 0.3500
CHI PHÍ khi tăng tốc: 0.3500        ← lớn gấp 10 lần lợi ích
```

Xe đứng yên vĩnh viễn. Sửa: lấy mẫu theo `accel × sample_window_sec (0.5 s)`, giới
hạn gia tốc thật ở khâu `_emit()`.

**Lỗi B — chi phí đích chuẩn hóa sai.** Ban đầu `c_goal = d_end / d_start`. Đích càng
xa tín hiệu càng yếu. Sửa: chuẩn hóa theo quãng đường tối đa có thể đi
`v_max × horizon`, cho tín hiệu không phụ thuộc khoảng cách.

---

## 5. Kế hoạch triển khai — 6 giai đoạn

Làm đúng thứ tự. Mỗi giai đoạn có tiêu chí nghiệm thu rõ ràng; đừng sang bước sau khi
bước trước chưa đạt.

### Giai đoạn 0 — Cài đặt và build (30 phút)

```bash
cd ~/ros2_ws/ros2_jazzy_person_reid_ws/src
unzip ~/Downloads/person_follow_nav.zip

cd ~/ros2_ws/ros2_jazzy_person_reid_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select person_follow_nav
source install/setup.bash
```

Sửa luôn hai lỗi khai báo ở `robot_rssi_ros2/package.xml`:

```xml
<name>robot_rssi_ros2</name>      <!-- thay cho <n> -->
<depend>sensor_msgs</depend>      <!-- thêm dòng này -->
```

**Nghiệm thu:** `ros2 pkg executables person_follow_nav` liệt kê 3 executable.

### Giai đoạn 1 — Hiệu chỉnh LiDAR (30 phút, BẮT BUỘC)

Gồm **hai chế độ**, chạy theo thứ tự.

#### 1A — Đo thân xe (chỉ làm một lần)

LiDAR quét 360° nên nó thấy cả cột đỡ, dây điện, mép sàn của **chính robot**. Những
điểm này nằm trong footprint. `rect_clearance()` trả về 0 cho điểm trong footprint,
tức "đã va chạm" → mọi quỹ đạo bị loại → `_dwa()` trả `None` mỗi chu kỳ → **xe kẹt
vĩnh viễn ở trạng thái `BLOCKED`, không bao giờ nhúc nhích**.

Dọn sạch quanh xe, không để vật gì trong bán kính 2 m:

```bash
ros2 run person_follow_nav calibrate_lidar --ros-args -p mode:=self_scan
```

Script vẽ bản đồ 360° và đánh dấu cung góc nào là thân xe:

```
  goc lidar  goc base   k.cach  on dinh  do thi (1 o = 10cm)
   270- 275      +180    0.128m    100%  #  <== TRONG FOOTPRINT, PHAI LOC
```

Chép dòng `blind_sectors_deg` mà nó in ra vào `config/follow_nav.yaml`, **cả hai node**.

Bộ lọc còn tự động bỏ mọi điểm rơi vào trong footprint (`self_filter_enabled: true`,
bật sẵn), nên kể cả lắp thêm phụ kiện sau này vẫn an toàn. `blind_sectors_deg` dùng
cho cột đỡ cao hơn mép footprint.

#### 1B — Đo tâm quay (bắt buộc: bánh sau là bánh chủ động)

Cần **cả lidar và driver xe** — node phải đọc `/odom` và gửi `/cmd_vel` để xoay.

```bash
# Terminal 1
ros2 launch person_follow_nav calibrate.launch.py

# Terminal 2 — kiểm tra trước
ros2 topic hz /scan        # ~10 Hz
ros2 topic hz /odom        # ~20-50 Hz

# Terminal 2 — rồi chạy
ros2 run person_follow_nav calibrate_center
```

Đừng dùng `test_avoid_only.launch.py` cho việc này: file đó chạy `follow_planner_node`,
node này publish `/cmd_vel` liên tục ở 15 Hz (Twist rỗng khi đang tắt). Hai nguồn ghi
cùng topic thì lệnh xoay bị lệnh rỗng chen vào, xe giật rồi dừng.

Xe tự xoay một vòng tại chỗ — dọn trống bán kính 1 m. Xem chi tiết trong
[`CALIBRATION.md`](CALIBRATION.md).

#### 1C — Hiệu chỉnh hướng

Đây là bước quan trọng nhất. Sai tham số này thì xe né vật cản ở... bên cạnh và lao
thẳng vào vật trước mặt.

```bash
# T1
ros2 launch sc_mini sc_mini.launch.py port:=/dev/robot_lidar

# T2 — đặt thùng carton ngay trước mũi xe, cách 0.6–1.0 m, xung quanh dọn trống
ros2 run person_follow_nav calibrate_lidar
```

Chép `lidar_yaw_offset_deg` vào **cả hai** node trong `config/follow_nav.yaml`.

Kiểm tra cuối: chuyển vật cản sang **bên trái** xe, chạy lại. Phải ra ≈ **+90°**.
Nếu ra −90° thì LiDAR lắp ngược → đặt `lidar_angle_sign: -1.0` rồi làm lại.

**Nghiệm thu:** vật trước mũi → 0° ±5°; vật bên trái → +90° ±5°.

#### Quy trình đầy đủ — bốn bước

Chạy tất cả bằng một lệnh, có nhắc từng bước và tự lưu log:

```bash
cd ~/ros2_ws/ros2_jazzy_person_reid_ws
bash src/person_follow_nav/scripts/run_calibration.sh
```

Hoặc chạy tay từng bước:

```bash
# 1. Dọn sạch quanh xe
ros2 run person_follow_nav calibrate_lidar --ros-args -p mode:=self_scan

# 2. Vật cản TRƯỚC MŨI XE
ros2 run person_follow_nav calibrate_lidar

# 3. Vật cản BÊN TRÁI XE
ros2 run person_follow_nav calibrate_lidar --ros-args -p expect_deg:=90

# 4. Vật cản BÊN PHẢI XE
ros2 run person_follow_nav calibrate_lidar --ros-args -p expect_deg:=-90
```

Bước 1 lưu hồ sơ thân xe vào `~/.ros/lidar_self_profile.npz`. Các bước sau tự nạp
file này để phân biệt thân xe với vật cản test. Không có hồ sơ thì script dùng ngưỡng
tạm và có thể nhầm.

Cả ba bước 2–4 đều báo `DUNG` thì hiệu chỉnh hoàn tất.

Kết quả đo thực tế của xe này đã ghi trong [`CALIBRATION.md`](CALIBRATION.md) — giữ
file đó để đối chiếu khi nghi LiDAR bị xê dịch.

**Chỉ chép `blind_sectors_deg` từ bước 1.** Ở bước 2–4 có vật test trong phòng, script
không phân biệt được đâu là thân xe đâu là vật test.

#### Nếu báo khoảng cách sai (ví dụ đặt vật 60 cm nhưng in ra 0.128 m)

Đó là tia đập vào thân xe, không phải vật cản của bạn. Chạy 1A trước. Bản đồ 360° sẽ
cho thấy ngay cung góc nào là thân xe.

Nếu sau khi lọc vẫn không tìm thấy vật cản, nguyên nhân thường gặp nhất là **vật thấp
hơn tầm quét LiDAR**. LiDAR quét ở một độ cao cố định — thùng carton thấp hơn LiDAR
thì nó không thấy gì cả. Dùng vật cao ngang LiDAR, ví dụ chai nước 1.5 L đặt trên ghế.

Vật thật sự gần hơn 0.35 m thì nới ngưỡng:

```bash
ros2 run person_follow_nav calibrate_lidar --ros-args \
  -p target_min_m:=0.25 -p self_return_max_m:=0.20
```

### Giai đoạn 2 — Đo footprint thật (15 phút)

Dùng thước đo xe của bạn, cập nhật `config/follow_nav.yaml`:

| Tham số | Nghĩa | Mặc định |
|---|---|---|
| `front_len` | Tâm `base_link` → mũi xe | 0.30 |
| `rear_len` | Tâm → đuôi xe | 0.30 |
| `half_width` | Nửa bề ngang | 0.24 |
| `lidar_x`, `lidar_y` | Vị trí LiDAR so với tâm | 0.0, 0.0 |

`base_link` ở đâu? Với `decoded_serial_node`, nó nằm giữa trục hai bánh. Nếu bánh
không ở giữa xe thì `front_len ≠ rear_len`.

Cũng sửa TF trong launch file cho đúng chiều cao LiDAR thật:

```python
arguments=["0.0", "0.0", "0.35", "0", "0", "0", "base_link", "laser"]
#            x     y      z(chiều cao LiDAR)
```

**Nghiệm thu:** số đo khớp thước, cộng thêm 1–2 cm dự phòng.

### Giai đoạn 3 — Test né vật cản KHÔNG camera (1–2 giờ)

Bước này loại bỏ hoàn toàn biến số ReID khỏi phương trình. Đừng bỏ qua.

```bash
# T1
ros2 launch person_follow_nav test_avoid_only.launch.py

# T2 — kiểm tra trước khi cho xe chạy
bash src/person_follow_nav/scripts/preflight.sh
```

`preflight.sh` kiểm tra: topic tồn tại, tần số `/scan` và `/odom`, **chỉ có một nguồn
ghi `/cmd_vel`**, planner đang chạy, và `/follow/stop` sẵn sàng. Sửa hết mục `HONG`
rồi mới đi tiếp.

#### 3.0 — Kê xe lên hộp, bánh không chạm đất

```bash
# T2
python3 src/person_follow_nav/scripts/fake_target.py 1.2 --watch

# T3
ros2 service call /follow/enable std_srvs/srv/Trigger {}
```

Cần thấy:

- Log planner có `self-filter: bo N/360 tia dap vao than xe`, **N khoảng 45–55**.
  N = 0 nghĩa là bộ lọc không ăn, xe sẽ kẹt `BLOCKED` ngay khi xuống đất.
- `cmd_v` dương, tăng dần từ 0 lên khoảng 0.05–0.1 rồi về 0 khi tới đích.
- Bánh **thật sự quay** ở `cmd_v` nhỏ nhất. Nếu bánh đứng im ở `v = 0.035` thì tăng
  `min_move_linear` lên 0.05.

#### 3.1 — Xuống đất, khoảng cách ngắn

Vẫn `fake_target.py 1.2`. Mục tiêu giả cách 1.2 m, khoảng cách bám 1.0 m, nên xe chỉ
nhích **0.2 m rồi dừng**. Đây là test an toàn nhất: xác nhận xe biết dừng.

#### 3.2 trở đi — Có vật cản

```bash
python3 src/person_follow_nav/scripts/fake_target.py 2.5 --watch
```

Mục tiêu giả ở khung `base_link` nên nó **luôn** cách xe 2.5 m — xe chạy liên tục
cho đến khi gặp vật cản. Luôn sẵn sàng bấm Ctrl-C (script tự gọi `/follow/stop`).

| # | Bố trí | Hành vi mong đợi |
|---|---|---|
| 3.2 | Không vật cản | Xe tiến thẳng đều, `state = FOLLOW` |
| 3.3 | Thùng giữa đường, lệch trái 20 cm | Vòng phải rồi về đường thẳng, `state = AVOID` |
| 3.4 | Thùng giữa đường, lệch phải 20 cm | Vòng trái |
| 3.5 | Hai thùng cách nhau **0.90 m** | Chui qua giữa, chậm lại |
| 3.6 | Hai thùng cách nhau **0.40 m** | **Không** chui, xoay tìm lối, `state = BLOCKED` |
| 3.7 | Tường chắn kín | Dừng, không đâm |

Xe rộng 0.60 m nên khe tối thiểu thực tế là **0.80 m** — xem mục "Giới hạn không gian
hẹp" trong `CALIBRATION.md`. Đừng test 0.75 m và tưởng hỏng.

**Cách tinh chỉnh** khi hành vi chưa đúng:

| Triệu chứng | Sửa |
|---|---|
| Xe đứng yên không nhúc nhích | Kiểm tra N của self-filter, và `sample_window_sec` ≥ 0.4 |
| Xe đi lòng vòng, không bám sát | Tăng `w_goal` |
| Xe đi sát vật cản quá | Tăng `w_clear` hoặc `margin_soft` |
| Xe giật, đổi hướng liên tục | Tăng `w_smooth` |
| Xe do dự trái/phải khi né | Tăng `avoid_side_hold_sec` |
| Xe dừng trước khe lẽ ra chui được | Kiểm tra `half_width`, rồi mới giảm `margin_hard` |
| Bánh không quay ở lệnh nhỏ | Tăng `min_move_linear` / `min_move_angular` |

**Nghiệm thu:** 6/6 kịch bản đúng, xe không va chạm lần nào.

#### Khi xe không né — chẩn đoán

```bash
# để test_avoid_only.launch.py đang chạy, đặt vật cản cách mũi xe ~1 m
python3 src/person_follow_nav/scripts/diagnose.py
```

Script phân biệt hai nguyên nhân hoàn toàn khác nhau:

**A — LiDAR không quay.** Nhận ra bằng cách so sánh các vòng quét liên tiếp: nếu tỉ lệ
tia thay đổi dưới 2%, dữ liệu đóng băng. Nguyên nhân hay gặp: nguồn 5 V không đủ
(SC-Mini cần ~500 mA lúc khởi động — đừng cắm qua hub thụ động), hoặc **đang chạy hai
launch file cùng lúc** nên hai node `sc_mini` tranh cùng một cổng serial.

```bash
ros2 node list | grep -c sc_mini     # phải là 1
```

**B — LiDAR quay nhưng planner không dùng dữ liệu.** Script tự chạy lại đúng chuỗi
xử lý của planner (đổi khung → lọc thân xe → tính clearance) và in ra từng bước, nên
thấy ngay mất dữ liệu ở đâu.

### Giai đoạn 4 — Ghép camera (2–3 giờ)

```bash
ros2 launch person_follow_nav follow_nav_real.launch.py

ros2 service call /person_reid/start_enroll  std_srvs/srv/Trigger {}
# đi quanh camera 20–30 giây
ros2 service call /person_reid/finish_enroll std_srvs/srv/Trigger {}
ros2 service call /follow/enable             std_srvs/srv/Trigger {}
```

Kiểm tra `target_tracker` trước khi cho xe chạy:

```bash
ros2 topic echo /follow/target --field source
```

Đứng cách xe 2 m, bạn phải thấy `camera+lidar`. Nếu thấy `camera+bbox` thì LiDAR
không tìm được cụm người — kiểm tra `assoc_window_deg` và `person_width_max_m`.

Kịch bản test:

| # | Kịch bản | Hành vi mong đợi |
|---|---|---|
| 4.1 | Đi thẳng, không vật cản | Bám đều, giữ 1.0 m |
| 4.2 | Rẽ trái/phải | Bám theo, không mất hình |
| 4.3 | Người thứ hai đi cắt ngang | Xe chậm lại, vòng qua, bám tiếp |
| 4.4 | Người thứ hai đứng chắn 3 s | Xe vòng qua, `source` chuyển `predicted` rồi về `camera+lidar` |
| 4.5 | Bạn đi khuất sau góc tường | Xe đi tới vị trí nhớ cuối rồi quét tìm |

Với 4.4, theo dõi:

```bash
ros2 topic echo /follow/planner_status --field state
```

Chuỗi mong đợi: `FOLLOW` → `AVOID` → `OCCLUDED` → `FOLLOW`.

**Nghiệm thu:** 4.4 thành công 8/10 lần trở lên.

### Giai đoạn 5 — Không gian hẹp và RSSI (2 giờ)

Test hành lang thật, cửa ra vào, giữa hai bàn.

Nếu xe quá rụt rè trong hành lang, giảm `margin_soft` từ 0.22 xuống 0.16. Nếu xe cọ
tường, tăng `margin_hard` từ 0.07 lên 0.10.

RSSI: kiểm tra quy ước góc trước.

```bash
ros2 topic echo /rssi/angle_deg
```

Đứng **bên phải** xe. Nếu giá trị **dương** thì `rssi_angle_sign: -1.0` là đúng (khớp
với camera). Nếu âm thì đổi thành `+1.0`.

RSSI chỉ sửa **hướng**, không cung cấp khoảng cách — đó là thiết kế có chủ ý, vì RSSI
đo khoảng cách rất kém. Nó chỉ là lưới an toàn khi camera mất > 1.2 s.

**Nghiệm thu:** xe qua được cửa rộng 0.80 m; RSSI kéo được xe quay đúng chiều khi che
camera hoàn toàn.

### Giai đoạn 6 — Checklist thực địa

Trước mỗi lần demo:

- [ ] `ros2 topic hz /scan` ≥ 8 Hz (SC-Mini thực đo ~10 Hz, không phải 20 Hz)
- [ ] `ros2 topic hz /odom` ≥ 20 Hz
- [ ] `ros2 topic hz /person_reid/target` ≥ 10 Hz
- [ ] `ros2 node list` — chỉ có **một** node publish `/cmd_vel`
  (kiểm tra: `ros2 topic info /cmd_vel --verbose`)
- [ ] Pin xe > 70% (điện áp thấp làm PWM không đủ, xe giật)
- [ ] Đã hiệu chỉnh LiDAR sau khi tháo lắp lại
- [ ] `/follow/stop` gọi được và xe dừng trong 0.3 s

Trong RViz2, thêm:

- `/scan` (LaserScan)
- `/follow/target_marker` (Marker) — trụ tròn, xanh = tin cậy cao, đỏ = đang dự đoán
- `/follow/debug_markers` (MarkerArray) — cầu xanh là đích phụ, đường vàng là hướng
  tới người

---

## 6. Những vấn đề thực tế khác cần lưu ý

Các điểm bạn chưa hỏi nhưng sẽ gặp.

**Odom trôi.** Encoder BW-DR03 trôi 2–5% quãng đường, chưa kể trượt bánh. Bộ nhớ odom
chỉ dùng trong 2.5 s nên trôi tối đa vài cm — chấp nhận được. Đừng tăng
`predict_max_sec` lên quá 4 s; sau mốc đó nên quét tìm thay vì tin bộ nhớ.

**LiDAR quét ở một độ cao cố định.** Nếu LiDAR đặt cao 0.35 m thì nó thấy chân người
và chân bàn, nhưng **không thấy** mặt bàn nhô ra, bậc thềm, dây điện. Đây là hạn chế
vật lý, không sửa bằng phần mềm được. Hai sonar sẵn có trên BW-DR03
(`/bw_dr03/sonar`) có thể bù cho vật thấp — cân nhắc thêm một lớp an toàn đọc topic
đó.

**Người là vật cản động.** LiDAR chỉ thấy vị trí hiện tại, không thấy hướng đi. Người
đi cắt ngang 1.2 m/s trong khi xe chạy 0.22 m/s — xe không "né" kịp theo nghĩa lập kế
hoạch, nó chỉ chậm lại và chờ. Điều này đúng và an toàn. Đừng cố tăng tốc độ để né
nhanh hơn.

**Camera FOV 62° là hẹp.** Ở khoảng cách 1.0 m, trường nhìn ngang chỉ 1.2 m. Người
bước sang ngang 0.6 m là ra khỏi khung. Nếu dự án cho phép đổi phần cứng, camera FOV
90–120° sẽ cải thiện đáng kể hơn bất kỳ tinh chỉnh thuật toán nào.

**Ánh sáng ngược.** ReID dùng đặc trưng màu (`color_weight: 0.20`). Đi từ trong nhà
ra cửa sổ làm màu đổi hoàn toàn và ReID sẽ mất target. Enroll ở đúng điều kiện ánh
sáng định demo.

**Watchdog driver.** `cmd_timeout: 1.0` nghĩa là xe tự dừng nếu 1 s không nhận lệnh.
Planner chạy 15 Hz nên an toàn, nhưng nếu CPU quá tải (YOLO trên CPU) thì tần số tụt
và xe giật. Kiểm tra `ros2 topic hz /cmd_vel` phải ổn định ≥ 10 Hz. Nếu không, giảm
`processing_hz` của node camera hoặc bật CUDA.

**Tăng tốc bằng GPU.** Máy bạn có Ryzen 9 8945HX với Radeon tích hợp. Nếu
`identity_lock_node` báo `using CPU` thì YOLO đang chạy CPU và sẽ rất chậm. Cân nhắc
ONNX Runtime hoặc giảm `img_size` từ 640 xuống 416.

---

## 7. Tham chiếu tham số

Xem chú thích đầy đủ trong `config/follow_nav.yaml`. Những tham số hay phải sửa nhất:

| Tham số | Mặc định | Sửa khi |
|---|---|---|
| `lidar_yaw_offset_deg` | −90.0 | **Luôn phải hiệu chỉnh** |
| `half_width` | 0.30 | Xe rộng khác 0.60 m |
| `follow_distance_m` | 1.00 | Muốn bám gần/xa hơn |
| `margin_soft` | 0.15 | Xe quá rụt rè trong hành lang → giảm |
| `margin_hard` | 0.06 | Xe cọ tường → tăng |
| `v_max` | 0.22 | Là tốc độ thật (driver đã hiệu chỉnh `max_linear` 0.49). Đổi thì chạy lại `test_sim.py` |
| `w_goal` / `w_clear` | 2.4 / 2.2 | Cân bằng bám sát vs an toàn |
| `fov_keep_deg` | 22.0 | Camera FOV khác 62° |
| `predict_max_sec` | 2.5 | Người hay bị che lâu hơn → tăng, tối đa 4 |

---

## 8. Lệnh thường dùng

```bash
# Bật / tắt / dừng khẩn
ros2 service call /follow/enable  std_srvs/srv/Trigger {}
ros2 service call /follow/disable std_srvs/srv/Trigger {}
ros2 service call /follow/stop    std_srvs/srv/Trigger {}

# Xóa bộ nhớ vị trí người (khi tracker bám nhầm)
ros2 service call /follow/reset_tracker std_srvs/srv/Trigger {}

# Theo dõi
ros2 topic echo /follow/target          # tracker nghĩ người ở đâu
ros2 topic echo /follow/planner_status  # planner đang làm gì
ros2 topic echo /follow/target --field source
ros2 topic echo /follow/planner_status --field state

# Kiểm tra không có node nào tranh /cmd_vel
ros2 topic info /cmd_vel --verbose

# Build lại sau khi sửa
colcon build --symlink-install --packages-select person_follow_nav
source install/setup.bash
```

## 9. Test offline

```bash
cd ~/ros2_ws/ros2_jazzy_person_reid_ws/src/person_follow_nav
python3 scripts/test_sim.py      # 6 test, không cần ROS
python3 scripts/test_memory.py       # so sánh bộ nhớ góc vs odom
python3 scripts/test_self_filter.py  # kiểm tra bộ lọc thân xe
```

Chạy lại sau mỗi lần sửa tham số trong `config/follow_nav.yaml` để biết mình có làm
hỏng gì không.

---

## 10. Cổng thiết bị

Launch file đã đặt sẵn đường dẫn `by-id` cho máy của bạn, chạy được ngay:

| Thiết bị | Đường dẫn |
|---|---|
| Khung xe BW-DR03 | `/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30BF0NT-if00-port0` |
| LiDAR SC-Mini | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |
| Camera KINGSEN | `/dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0` |

`by-id` bền hơn `ttyUSB0` (không đổi theo thứ tự cắm) và không cần udev rules.

### Cảnh báo cho ngày cắm mạch RSSI

LiDAR hiện có tên `usb-1a86_USB_Serial-if00-port0`. Chuỗi `1a86_USB_Serial` sinh ra từ
VID:PID của chip CH340 — **không phải serial riêng**. NodeMCU RSSI cũng là CH340, nên
khi cắm vào nó sẽ tạo ra tên **trùng hệt**. Linux chỉ giữ được một symlink, và cái nào
thắng là ngẫu nhiên theo thứ tự cắm.

Hôm đó phải chuyển sang `by-path` (theo cổng vật lý, luôn phân biệt được):

```bash
ls -l /dev/serial/by-path/
# pci-0000:00:14.0-usb-0:2.3:1.0-port0 -> ../../ttyUSB0
# pci-0000:00:14.0-usb-0:2.4:1.0-port0 -> ../../ttyUSB1

ros2 launch person_follow_nav follow_nav_real.launch.py \
  start_rssi:=true \
  lidar_port:=/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.3:1.0-port0 \
  rssi_port:=/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.4:1.0-port0
```

Hoặc chạy `scripts/setup_udev.py` để sinh udev rules đúng cho máy này.

### Bật lại RSSI

RSSI đang tắt mặc định (`rssi_enabled: false` trong config, `start_rssi:=false` trong
launch). Khi cắm mạch rồi, sửa `rssi_enabled: true` và thêm `start_rssi:=true`. Tracker
chạy bình thường khi tắt — chỉ mất lớp dự phòng cuối cùng lúc camera mất hình quá 1.2 s.

---

## 11. Xử lý lỗi cổng USB (`Can't access port /dev/robot_lidar`)

### Nguyên nhân

File `99-robot-usb.rules` cũ nhận diện thiết bị bằng **đường dẫn cổng USB vật lý**:

```
SUBSYSTEM=="tty", KERNELS=="1-2.3", ... SYMLINK+="robot_lidar"
                  ^^^^^^^^^^^^^^^^
```

Chuỗi `1-2.3` là số hiệu cổng trên bo mạch của **máy cũ**. Máy mới có topology USB
khác nên không rule nào khớp, symlink không được tạo.

Phải dùng `KERNELS` vì Lidar SC-Mini và NodeMCU RSSI đều là chip CH340 (1a86:7523)
và **không có serial riêng** — đúng như ghi chú trong file rules cũ của bạn. Chỉ
BW-DR03 (FTDI FT231X) có serial thật `D30BF0NT`.

`setup_usb_rules.py` cũ không xử lý được ca này: nó ưu tiên `ATTRS{serial}`, mà hai
CH340 lại trùng serial, nên sinh rule xung đột.

### Sửa

```bash
cd ~/ros2_ws/ros2_jazzy_person_reid_ws/src/person_follow_nav/scripts

# Chẩn đoán trước — xem đang có gì
bash check_devices.sh

# Tạo lại rules cho máy này (rút/cắm lại từng thiết bị để nhận diện)
python3 setup_udev.py

sudo cp /tmp/99-robot-usb.rules /etc/udev/rules.d/99-robot-usb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sleep 2 && ls -l /dev/robot_*
```

### Cách nhanh hơn, không cần udev

Linux tự tạo sẵn symlink theo cổng vật lý, dùng ngay được:

```bash
ls -l /dev/serial/by-path/
# ví dụ: pci-0000:00:14.0-usb-0:2.3:1.0-port0 -> ../../ttyUSB0

ros2 launch sc_mini sc_mini.launch.py \
  port:=/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.3:1.0-port0
```

Ổn định như `KERNELS`, không cần quyền root. Nhược điểm: chuỗi dài, và vẫn phải cắm
đúng cổng.

### Cách nhanh nhất để test

```bash
ls /dev/ttyUSB*
ros2 launch sc_mini sc_mini.launch.py port:=/dev/ttyUSB0
```

Nếu sai cổng thì thử `ttyUSB1`, `ttyUSB2`. Dùng tạm khi đang thử nghiệm; số thứ tự
`ttyUSBx` đổi theo thứ tự cắm nên không dùng cho bản chạy thật được.

### Quyền truy cập

Nếu báo `Permission denied` thay vì `Can't access port`:

```bash
sudo usermod -aG dialout $USER
# phải ĐĂNG XUẤT và đăng nhập lại mới có hiệu lực
```

### Ghi nhớ

Với thiết bị nhận diện bằng cổng vật lý, sau này **bắt buộc cắm đúng cổng USB đó**.
Đổi cổng là symlink biến mất. Nên dán nhãn lên cổng USB trên máy.

Cảnh báo `AMENT_PREFIX_PATH ... doesn't exist` khi build lại sau khi `rm -rf install`
là bình thường — do shell còn giữ biến môi trường của lần source trước. Mở terminal
mới hoặc `source install/setup.bash` lại là hết.

Cảnh báo `-Wunused-variable` của `sc_mini` cũng vô hại — build vẫn thành công
(`Finished <<< sc_mini`). Đó là code gốc của nhà sản xuất LiDAR.
