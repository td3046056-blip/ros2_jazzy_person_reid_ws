# Hồ sơ hiệu chỉnh LiDAR — BW-DR03

Ghi lại các lệnh đã chạy, số liệu đo được và kết luận. Giữ file này để không phải
đo lại, và để đối chiếu khi nghi ngờ LiDAR bị xê dịch.

| | |
|---|---|
| Máy | `thachLG` — Lenovo Legion R9000P, Ubuntu 24.04, ROS 2 Jazzy |
| Workspace | `~/ros2_ws/ros2_jazzy_person_reid_ws` |
| LiDAR | SC-Mini M1C1, CH340 |
| Cổng | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |

---

## Thông số LaserScan đo được

```
angle_min = 0.0000 rad      angle_max = 6.2832 rad   (0 .. 2π, KHÔNG phải -π..π)
số tia    = 360             angle_increment = 0.01745 rad  (1.00°)
range_min = 0.100 m         range_max = 10.000 m
```

Góc chạy 0–360° chứ không phải −180…+180° như đa số LiDAR khác. Mọi tính toán góc
trong `person_follow_nav` đã tính đến điều này.

---

## Bước 1 — Đo thân xe (`mode:=self_scan`)

```bash
ros2 run person_follow_nav calibrate_lidar --ros-args -p mode:=self_scan
```

Điều kiện: dọn sạch quanh xe, không để vật gì trong bán kính 2 m.

### Kết quả

Phát hiện cụm tia rất gần, ổn định 100% qua 60 vòng quét:

| Góc LiDAR | Góc base_link | Khoảng cách | Độ ổn định |
|---|---|---|---|
| 250–255° | +163° | 0.116 m | 50% |
| 265–270° | +178° | 0.131 m | 7% |
| 270–275° | −178° | 0.129 m | **100%** |
| 275–280° | −173° | 0.128 m | **100%** |
| 280–285° | −168° | 0.128 m | **100%** |
| 285–290° | −162° | 0.128 m | 83% |

Tất cả nằm quanh hướng **thẳng sau xe** (base ≈ 180°), cách tâm 12.8 cm. Đây là
thân xe — cột đỡ LiDAR hoặc mép sàn phía sau.

### Tại sao phải lọc

Điểm ở 0.128 m nằm bên trong footprint (mũi 0.30 m, đuôi 0.30 m, nửa rộng 0.24 m).
`rect_clearance()` trả về 0 cho điểm trong footprint, nghĩa là "đã va chạm". Chỉ cần
ba tia như vậy là mọi quỹ đạo bị loại, `_dwa()` trả `None` mỗi chu kỳ, và xe kẹt vĩnh
viễn ở trạng thái `BLOCKED`.

### Giá trị đã áp dụng

```yaml
self_filter_enabled: true
self_filter_margin: 0.03
blind_sectors_deg: [246.0, 294.0]
```

Định dạng **phẳng** theo cặp `[lo1, hi1, lo2, hi2, ...]`. ROS 2 không hỗ trợ mảng
lồng nhau làm tham số — kiểu hợp lệ chỉ có `int64[]`, `double[]`, `string[]`.

Cung `[246, 294]` bao trọn cả cụm, kể cả hai bin độ ổn định thấp (250–255° và
265–270°). Tương ứng base ≈ ±21° quanh hướng thẳng sau. Xe chủ yếu đi tiến nên không
ảnh hưởng.

Hồ sơ thân xe lưu tại `~/.ros/lidar_self_profile.npz`, các bước sau tự nạp.

---

## Bước 2 — Vật cản trước mũi xe

```bash
ros2 run person_follow_nav calibrate_lidar
```

### Kết quả

```
Vật cản tìm thấy:       0.715 m
Góc trong khung lidar:  90.0°
Góc trong base_link:    +0.0°
=> ĐÚNG
```

Dấu hiệu trong bản đồ — khoảng cách gần như bằng nhau trên cung 75–105°, tâm ở 90°:

| Góc LiDAR | Góc base | Khoảng cách |
|---|---|---|
| 75–80° | −12° | 0.727 m |
| 80–85° | −8° | 0.718 m |
| 85–90° | −2° | 0.716 m |
| **90–95°** | **+2°** | **0.715 m** |
| 95–100° | +8° | 0.718 m |
| 100–105° | +12° | 0.727 m |

---

## Bước 3 — Vật cản bên trái xe

```bash
ros2 run person_follow_nav calibrate_lidar --ros-args -p expect_deg:=90
```

### Kết quả

```
Vật cản tìm thấy:       0.343 m
Góc trong khung lidar:  182.5°
Góc trong base_link:    +92.5°
Sai lệch so với khai báo (+90°): +2.5°
=> ĐÚNG
```

### Ghi chú — lần chạy đầu báo sai

Lần đầu script báo `+75.0°` và kết luận SAI. Đó là lỗi của script, không phải LiDAR.
Ba nguyên nhân, đều đã sửa:

1. `self_return_max_m` mặc định 0.35 m, mà vật ở 0.343 m — thấp hơn ngưỡng nên bị xếp
   nhầm vào nhóm thân xe. Đã hạ xuống 0.22 m, và thay bằng hồ sơ đo ở bước 1.
2. `target_min_m` mặc định 0.35 m, cũng cao hơn 0.343 m nên loại luôn các bin đúng.
   Đã hạ xuống 0.25 m.
3. Vật là **tấm phẳng**, không phải vật nhỏ. Script lấy trọng tâm cả cung góc, mà với
   tấm phẳng cung góc trải rộng lệch về một phía → trọng tâm cho +105° thay vì +90°.
   Đã đổi sang cửa sổ hẹp ±10° quanh điểm gần nhất.

### Kiểm chứng: vật đúng là tấm phẳng ở +90°

Với tấm phẳng vuông góc hướng bên trái, khoảng cách phải tuân theo
`d(θ) = d_min / cos(θ − 90°)`:

| Góc base | Đo được | Mô hình | Sai lệch |
|---|---|---|---|
| +72° | 0.361 m | 0.361 m | 0.0 cm |
| +78° | 0.352 m | 0.351 m | 0.1 cm |
| +83° | 0.346 m | 0.346 m | 0.0 cm |
| +88° | 0.344 m | 0.343 m | 0.1 cm |
| **+92°** | **0.343 m** | **0.343 m** | **0.0 cm** |
| +97° | 0.344 m | 0.346 m | 0.2 cm |
| +103° | 0.348 m | 0.352 m | 0.4 cm |
| +108° | 0.354 m | 0.361 m | 0.7 cm |
| +118° | 0.384 m | 0.388 m | 0.4 cm |
| +128° | 0.417 m | 0.435 m | 1.8 cm |

Sai lệch trung bình 0.6 cm. Khớp gần như hoàn hảo → vật thật sự ở +90°.

### Cảnh báo

Lần chạy này script từng gợi ý `blind_sectors_deg: [[168, 197], [268, 287]]`.
**Không được chép.** Cung `[168, 197]` chính là vật test — chép vào là robot mù hẳn
bên trái. Chỉ lấy `blind_sectors_deg` từ bước 1.

---

## Bước 4 — Vật cản bên phải xe (chưa chạy)

```bash
ros2 run person_follow_nav calibrate_lidar --ros-args -p expect_deg:=-90
```

Kỳ vọng: góc base ≈ −90°, sai lệch dưới 10°.

Bước này xác nhận chiều quay. Nếu ra **+90°** thay vì −90° thì LiDAR lắp ngược, phải
đặt `lidar_angle_sign: -1.0` rồi làm lại từ bước 2.

---

## Kết luận — giá trị đã chốt

Chép vào `config/follow_nav.yaml`, **cả hai node** `target_tracker_node` và
`follow_planner_node`:

```yaml
lidar_yaw_offset_deg: -90.0
lidar_angle_sign: 1.0
lidar_x: 0.0
lidar_y: 0.0
self_filter_enabled: true
self_filter_margin: 0.03
blind_sectors_deg: [246.0, 294.0]
```

Các giá trị này đã được điền sẵn trong package.

Khớp với `robot_rssi_ros2/SYSTEM_CONTEXT.md` (`lidar_front_center_deg: 90.0`) và với
bytecode khôi phục từ `vfh_rssi_controller.cpython-312.pyc` — ba nguồn độc lập cùng
xác nhận.

---

## CẢNH BÁO — `base_link` đang đặt sai

Bánh **sau** là bánh chủ động. Xe vi sai quay quanh **tâm trục bánh chủ động**, nên
odometry của driver lấy gốc ở trục **sau**. Nhưng các số đo bên dưới lấy từ trục
**trước**. Hai khung lệch nhau đúng bằng khoảng cách hai trục `L`.

Hệ quả nếu không sửa, với `L = 0.25 m`:

| | Đang khai (L=0) | Thực tế (L=0.25) |
|---|---|---|
| `front_len` | 0.14 | **0.39** |
| `rear_len` | 0.33 | **0.08** |
| `lidar_x` | 0.10 | **0.35** |
| Bán kính ngoại tiếp | 0.446 | **0.492** |
| Khi xoay, phần văng ra | đuôi | **mũi** |

Đảo ngược hoàn toàn. Mọi điểm LiDAR bị đặt sai chỗ 25 cm, và `rotate_radius` thiếu
5 cm — xe tưởng xoay được tại chỗ trong khi mũi đập vào tường.

### Đo tâm quay thật

```bash
ros2 run person_follow_nav calibrate_center
```

Xe tự xoay một vòng tại chỗ. Với mỗi giá trị `lidar_x` thử nghiệm, công cụ dựng bản
đồ điểm trong khung `odom`. Nếu `lidar_x` đúng, vật cản đứng yên luôn rơi vào đúng
một chỗ → bản đồ sắc nét. Nếu sai, vật cản bị trải thành vòng tròn bán kính bằng
đúng sai số → bản đồ nhòe.

Kiểm chứng thuật toán bằng mô phỏng với `lidar_x` thật = 0.35 m:

```
  0.25 ->   7987 ô  ##########
  0.30 ->   4560 ô  #####
  0.35 ->    534 ô    <== ĐÁY
  0.40 ->   4560 ô  #####
  0.50 ->  11702 ô  ###############

  Đo được: 0.35 m      sai số 0.0 cm
```

Cách này không cần biết khoảng cách hai trục, và đúng cả khi xe là skid-steer bốn
bánh chủ động (tâm quay ở giữa, không ở trục nào).

**Cần vật cản cố định quanh xe** trong khoảng 0.4–3.0 m để làm mốc. Phòng trống hoàn
toàn sẽ không đo được — công cụ sẽ báo "đường cong quá phẳng".

### Kết quả đo thực tế trên xe này

```
   lidar_x thử    độ nhòe (ô lưới)
  ----------------------------------
          0.04                5966  ###
          0.08                4476
          0.09                4208
          0.10                4195    <== SẮC NÉT NHẤT
          0.11                4329
          0.12                4584
          0.16                5993  ###

  Tâm quay thật cách lidar 0.10 m về phía sau
  => L = 0.10 - 0.10 = 0.00 m
```

Độ tương phản 43% (ngưỡng tin cậy của công cụ là 5%), đường cong chữ V trơn và đối
xứng (lệch 2–3% ở ±1–3 cm). Khớp parabol quanh đáy cho **0.098 m** — làm tròn 0.10
là đúng.

`L = 0` nghĩa là **tâm quay nằm ở trục trước**, đúng chỗ đã đo `base_link`. Cấu hình
hiện tại không cần sửa gì.

### Mâu thuẫn với mô tả bánh xe

Nếu bánh sau là bánh chủ động thì tâm quay phải ở trục sau, tức `L > 0`. Phép đo nói
`L = 0`. Giải thích khả dĩ, theo thứ tự xác suất:

1. Xe vi sai với **hai bánh chủ động ở trước**, hai bánh sau là bánh dẫn hướng. Khớp
   hoàn toàn với số đo.
2. Cái được gọi là "trục trước" khi đo thật ra chính là trục bánh chủ động.
3. Skid-steer bốn bánh: tâm quay ở giữa hai trục, tức `L/2`. Nhưng khi đó `L = 0`
   vẫn có nghĩa hai trục trùng nhau — vô lý.

Kiểm tra 30 giây: dán băng dính đánh dấu sàn dưới tâm trục trước và tâm trục sau, cho
xe xoay chậm tại chỗ, xem điểm nào đứng yên.

Dù sao thì **con số đo được mới là con số code cần**. `lidar_x` phải tính so với khung
mà odometry dùng, và đó chính xác là thứ phép đo này trả về — không phụ thuộc bánh nào
quay.

### Cách thủ công

Đo bằng thước khoảng cách `L` từ tâm trục trước đến tâm trục sau, rồi:

```
lidar_x   = 0.10 + L
front_len = 0.14 + L
rear_len  = 0.33 − L
```

## Hình học xe (đo từ trục TRƯỚC — cần cộng L)

| | |
|---|---|
| `front_len` | 0.14 m (base_link → mũi) |
| `rear_len` | 0.33 m (base_link → đuôi) |
| `half_width` | 0.30 m |
| Bề ngang | **0.60 m** |
| Chiều dài | 0.47 m |
| `lidar_x` / `lidar_y` / z | 0.10 / 0.00 / 0.18 m |

Xe **lệch về sau rất nhiều**: mũi chỉ 14 cm phía trước nhưng đuôi kéo 33 cm phía
sau. Hai hệ quả:

**Bán kính ngoại tiếp = hypot(0.33, 0.30) = 0.446 m**, quyết định bởi góc sau chứ
không phải góc trước. Xoay tại chỗ cần 0.45 m thoáng quanh xe → `rotate_radius: 0.47`.

**Khi xoay, đuôi văng ra rất rộng.** Mô hình chữ nhật trong DWA kiểm tra footprint ở
từng tư thế của quỹ đạo nên có tính điều này — mô hình hình tròn thì không.

LiDAR ở độ cao 18 cm: thấy chân bàn, chân ghế, ống quyển người — nhưng **không thấy**
mặt bàn nhô ra, bậc thềm, dây điện.

## Giới hạn không gian hẹp

Xe rộng 0.60 m nên đây là vấn đề thật. Kết quả quét tham số bằng mô phỏng:

| `margin_soft` | `margin_hard` | Khe nhỏ nhất chui được | Lý thuyết |
|---|---|---|---|
| 0.22 | 0.08 | 0.84 m | 0.76 m |
| 0.18 | 0.08 | 0.84 m | 0.76 m |
| 0.15 | 0.06 | **0.80 m** | 0.72 m |
| 0.12 | 0.05 | 0.80 m | 0.70 m |

Đang dùng `0.15 / 0.06`. Hạ `margin_hard` xuống dưới 0.06 không giúp thêm, chỉ bớt
an toàn.

Thực tế: cửa phòng tiêu chuẩn 0.80 m là **sát nút**. Nên test với khe ≥ 0.90 m trước,
rồi mới thử cửa thật.

## Bẫy kiểu dữ liệu tham số ROS 2

ROS 2 phân biệt nghiêm ngặt `INTEGER` với `DOUBLE`. Gõ `-p expect_deg:=90` khi tham số
khai báo mặc định `0.0` sẽ báo lỗi:

```
InvalidParameterTypeException: Trying to set parameter 'expect_deg' to '90'
of type 'INTEGER', expecting type 'DOUBLE'
```

Cách chữa nhanh: thêm dấu chấm — `-p expect_deg:=90.0`.

Package đã bật `ParameterDescriptor(dynamic_typing=True)` cho mọi tham số nên giờ
nhận cả hai kiểu. Nhưng nếu bạn sửa code và thêm tham số mới, nhớ dùng cùng cách.

## Tốc độ driver BW-DR03 — `max_linear` / `max_angular`

Driver đổi `/cmd_vel` sang % PWM **mở vòng**: `pwm = int(v / max_linear × max_percent)`,
tối thiểu 5%. Vì vậy `max_linear` phải là tốc độ **thật** của xe ở `max_percent`.

**Lỗi đã gặp:** `bw_dr03.launch.py` ghi `max_linear 0.226 / max_angular 1.10` "đã calib
thực tế" ở `max_percent 30`. Launch của `person_follow_nav` dùng `max_percent 60` nhưng giữ
nguyên 0.226 → lệnh 0.22 m/s ra 58% PWM → xe chạy **0.475 m/s**, nhanh gấp 2.16 lần planner
tưởng. Quãng đường, quãng phanh và góc văng đuôi mà DWA dự đoán đều sai theo cùng tỉ lệ.

### Kết quả đo 16/09/2026 (`scripts/measure_speed.py` + `calibrate.launch.py`)

| Lệnh | PWM | Odom ổn định | Thật / lệnh | Odom tổng | Thước |
|---|---|---|---|---|---|
| v = 0.22 m/s | 58% | 0.475 m/s | 2.16 | 1.503 m | 1.50 m |
| v = 0.11 m/s | 29% | 0.238 m/s | 2.16 | 0.740 m | 0.72 m |
| v = 0.05 m/s | 13% | 0.107 m/s | 2.14 | 0.315 m | 0.30 m |
| w = 0.80 rad/s | ±43% | 1.763 rad/s | 2.20 | quay 306.8° | chưa đo |

- **Tuyến tính, đi qua gốc:** 0.0082 m/s mỗi 1% PWM từ 13% đến 58%, không thấy vùng chết.
- **Odom đúng:** lần dài nhất lệch 0.3 cm / 150 cm. Hai lần ngắn lệch 1.5–2 cm, giống sai số
  đánh dấu cố định hơn là sai tỉ lệ.
- **Xoay khớp với đi thẳng:** 2 × 0.0082 / 0.40 = 0.041 rad/s mỗi % → 43% ra 1.76 rad/s.
  Nhưng số xoay lấy từ odom nên phụ thuộc `wheel_separation 0.40` — **góc thật chưa đo.**

### Giá trị đã áp dụng (phải trùng ở cả 3 launch file)

```
max_percent: 60
max_linear:  0.49    # = 60 × 0.475 / 58  (driver cắt phần lẻ: gửi 58%, không phải 58.4%)
max_angular: 2.46    # = 60 × 1.763 / 43
```

Sau khi sửa: lệnh 0.22 m/s → 26% → ~0.213 m/s thật; lệnh xoay 0.80 → 19% → ~0.78 rad/s.
Thấp hơn lệnh ~3% do driver cắt phần lẻ, lệch về phía an toàn.

Lệnh nhỏ nhất của planner (`min_move_linear 0.035`, `min_move_angular 0.10`) giờ đều ra 5%
PWM (trước là 9% và 5%). **[CẦN XÁC NHẬN]** bánh có quay ở 5% không:

```bash
python3 src/person_follow_nav/scripts/measure_speed.py 0.035
python3 src/person_follow_nav/scripts/measure_speed.py 0.0 --w 0.10
```

## Khi nào phải đo lại

- Tháo lắp lại LiDAR, dù chỉ nới ốc
- Lắp thêm phụ kiện lên xe (giá đỡ, pin, camera)
- Log của planner báo số tia bị lọc nhảy vọt:
  `self-filter: bo 47/360 tia dap vao than xe` (bình thường là 3–8)
- Xe né vật cản sai hướng, hoặc kẹt `BLOCKED` mà không có gì chắn
- **Tốc độ:** đổi `max_percent`, thay bánh/động cơ, hoặc xe chạy nhanh/chậm rõ rệt so với
  lệnh → chạy lại `scripts/measure_speed.py`

Chạy lại toàn bộ bằng một lệnh:

```bash
bash src/person_follow_nav/scripts/run_calibration.sh
```
