# person_follow_identity

Package ROS2 Jazzy mới để khóa đúng **một người đã được enrollment**, không tự chọn người khác khi target mất khỏi camera.

Package này dùng lại detector/tracker hiện có trong `person_reid_tracker`:

- YOLOv5 person detector
- DeepSORT ngắn hạn
- ReID extractor `ckpt.t7`

Phần mới:

- Không auto-lock người mới sau khi mất target.
- Có trạng thái enrollment bằng phím/service/topic.
- Positive gallery cho người cần follow.
- Negative gallery cho người không phải target.
- So khớp `topk_mean`, margin với người thứ hai, margin với negative gallery.
- Reject khi không chắc, robot nên dừng thay vì follow nhầm.

## Cài vào workspace hiện tại

Giải nén/copy thư mục này vào:

```bash
ros2_jazzy_person_reid_ws/src/person_follow_identity
```

Build lại:

```bash
cd ~/ros2_ws/ros2_jazzy_person_reid_ws
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## Chạy

```bash
ros2 launch person_follow_identity identity_lock.launch.py
```

Trong cửa sổ camera:

- `e`: bắt đầu học người cần follow
- `f`: kết thúc enrollment sớm nếu đã đủ mẫu
- `r`: reset hoàn toàn, học người khác
- `q`: thoát

Hoặc dùng ROS2 service:

```bash
ros2 service call /person_reid/start_enroll std_srvs/srv/Trigger {}
ros2 service call /person_reid/finish_enroll std_srvs/srv/Trigger {}
ros2 service call /person_reid/reset std_srvs/srv/Trigger {}
```

Hoặc topic command:

```bash
ros2 topic pub --once /person_reid/command std_msgs/msg/String "{data: 'enroll'}"
ros2 topic pub --once /person_reid/command std_msgs/msg/String "{data: 'reset'}"
```

## Quy trình test đúng

1. Chỉ để người A trong vùng camera hoặc đứng gần tâm khung hình.
2. Nhấn `e`.
3. Trong 30 giây, người A xoay nhẹ trái/phải, tiến/lùi, quay lưng một chút.
4. Khi status `ENROLLMENT_DONE`, bắt đầu test A/B.
5. Nếu A ra khỏi camera và B đi vào, status phải là `LOST` hoặc `LOST_REJECTED_CANDIDATE`, không được publish B làm target.

## Tối ưu cho chỗ đông người (29/09)

Config dùng trên xe: `person_follow_robot/config/identity_lock_kingsen.yaml` (file `config/identity_lock.yaml` của gói này giống hệt).

| Thay đổi | Vì sao (số đo) |
|---|---|
| Mạng ReID nhận ảnh **RGB** (trước là BGR) | Market-1501: Rank-1 39.8% → 75.5%, mAP 18.6 → 51.8. DeepSORT cũng dùng feature này để nối ID nên ít tráo ID hơn |
| Dùng lại feature DeepSORT, không chạy ReID lần 2 | ReID ~2–4 ms/người — phần tốn CPU nhất khi đông người |
| YOLO chạy khung 640×480 thay vì 640×640 | ~20 ms → ~15 ms |
| Luồng đọc camera riêng, luôn lấy khung mới nhất | Trước: bộ đệm V4L2 trả khung cũ ~100 ms |
| DeepSORT được cập nhật cả ở khung YOLO không thấy ai | Trước: khung đó bị bỏ qua hẳn → YOLO sót 1 khung là mất sạch track (chạy thật: 6 lần / 52 s đang bám) và `max_age` chỉ đếm khung có người. Track chỉ là vị trí dự đoán (`conf = 0`) vẫn được báo 1 khung nhưng không chấm điểm, không học, không nhận lại |
| BLAS của numpy 1 luồng (`OPENBLAS_NUM_THREADS=1` đầu `node.py`) | DeepSORT tích luỹ 100 feature/track rồi tính khoảng cách bằng tích ma trận; OpenBLAS đa luồng quay chờ bận, tranh CPU với torch → xử lý trượt từ 29 lên 53–59 ms/khung sau vài giây. Sau khi sửa: ổn định 25–29 ms. **Không** đặt `MKL_NUM_THREADS`/`OMP_NUM_THREADS` — torch đọc hai biến đó và chạy 1 luồng (66 ms) |
| Tổng thể (khung 6 người, CPU máy này) | cũ 68.7 ms/khung (config cũ còn khoá 8 Hz) → mới 40.3 ms; webcam 0–1 người: 15.0 Hz, 25–29 ms, trễ từ lúc chụp 30–60 ms |
| `ts` trong `/person_reid/target` = **lúc chụp** | Trước là lúc xử lý xong → tracker gần như không bù được góc xe quay trong lúc xử lý |
| Biết ai che ai; khung bị che không học vào gallery, chỉ trừ ít điểm | Trước: một khung bị che/quay lưng là bỏ khoá (`verification_fail_limit: 1`) |
| Bỏ khoá **ngay** khi mâu thuẫn rõ | Ảnh giống người đã biết hơn mục tiêu, người khác giống mục tiêu hơn hẳn, hoặc ngoại hình track đổi đột ngột (DeepSORT tráo ID) |
| Kiểm tra hỏng vì dấu hiệu "là người khác" → giữ khoá nhưng **tạm không báo** (`TRACK_VERIFY_HOLD`) | Hỏng chỉ vì điểm thấp (bị che, quay lưng) thì vẫn báo `TRACK_VERIFY_WARN` như cũ để tracker có dữ liệu liên tục |
| Tìm lại bằng điểm bằng chứng (khớp +1, khớp mạnh +2, hỏng −1) | Rõ ràng thì 2 khung; mơ hồ thì không bao giờ |
| Gallery âm tách từng người, chỉ học người **không chạm** bbox mục tiêu | bbox chồng nhau chứa điểm ảnh của mục tiêu → mục tiêu bị loại về sau |
| Gallery theo **kiểu khung** (toàn thân / cắt đầu / cắt chân) | Đứng gần xe bị cắt khung: người thật so với gallery toàn thân chỉ còn 0.79 (người lạ 0.72); cùng kiểu cắt: 0.89 |
| Tắt gait | `mediapipe 0.10.35` đã bỏ `mp.solutions` → Pose không chạy; phần còn lại (bóng Otsu, tỉ lệ bbox) đổi theo khoảng cách hơn là theo người, lại bắt buộc 6 khung khi tìm lại |
| Góc camera theo mô hình pinhole / hiệu chỉnh | Công thức tuyến tính cũ sai tới 1.2° ở FOV 62°, tới 11° ở FOV 120° |

### Enroll (quan trọng hơn trước)

Trong 30 giây enroll, người cần bám nên **đi từ ~3 m lại gần tới ~0.8 m rồi lùi ra**, xoay trái/phải, quay lưng một lần. Ở 1 m camera 62° không thấy hết người nên ảnh bị cắt — enroll đủ các "kiểu khung" thì lúc bám gần mới so khớp đúng. Dòng `reason` khi enroll có đếm mẫu theo kiểu khung `full/top-cut/bottom-cut/both`. Người khác đứng xa trong lúc enroll là **có lợi** (được học làm "không phải mục tiêu"), miễn là không đứng chạm vào người enroll.

### Theo dõi

- Log mỗi 10 s: `camera: X Hz, xu ly TB Y ms, tre tu luc chup Z ms, N nguoi, status ...`
- Payload thêm: `occlusion` (tỉ lệ bị che), `evidence` (điểm khoá/tìm lại), `view_bucket` (0 toàn thân, 1 cắt đầu, 2 cắt chân, 3 cả hai), `latency_ms`.

### Camera góc rộng

```bash
python3 scripts/calibrate_camera.py --source /dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0
```
In ra `camera_matrix`, `dist_coeffs`, FOV thật — dán vào yaml, đặt `camera_angle_model: "calibrated"`; tuỳ chọn `undistort_frame: true` để khử méo cả khung trước YOLO/ReID (người ở mép ảnh hết bị kéo nghiêng). Ống mắt cá (>~120°) thêm `--fisheye` và `camera_fisheye: true`.

### Camera KINGSEN trên xe (đo 30/09, cắm vào laptop)

**Tốc độ khung.** Ở 640×480: MJPG tối đa 25 fps, YUYV chỉ 15–20 fps. Code cũ không chọn định dạng nên OpenCV lấy YUYV, và camera lại bật sẵn `backlight_compensation=1` (chế độ thiếu sáng, **giảm còn nửa fps**, phơi sáng ~80 ms/khung):

| Cấu hình | fps nhận thật |
|---|---|
| Như cũ (YUYV, phơi sáng tự động) | **10.1** |
| MJPG, phơi sáng tự động | 12.5 |
| MJPG + `backlight_compensation=0` + phơi sáng thủ công 10–30 ms | **25** (sau 7–45 s khởi động) |
| Phơi sáng 39 ms | tụt 12.5 (sát chu kỳ 40 ms) |

Phơi sáng dài làm nhoè khi người đi hoặc xe quay (người đi 1 m/s ở 1.5 m: 80 ms ≈ 28 px, 30 ms ≈ 11 px). Đo được: ReID Rank-1 76.7% → **59.2%** ở mức nhoè 28 px (73.9% ở 11 px); YOLO mất **toàn bộ** người trong ảnh mẫu ở conf 0.55 khi nhoè 28 px.

Config bây giờ: `camera_fourcc: MJPG`, `camera_fps: 25`, `camera_exposure_mode: fixed_fps` (phần mềm tự chỉnh phơi sáng theo độ sáng vùng giữa-dưới ảnh, tối đa 30 ms, bội số 10 ms để không nhấp nháy dưới đèn 50 Hz), `camera_v4l2_controls: backlight_compensation=0,exposure_dynamic_framerate=0,power_line_frequency=1`. Log mỗi 10 s in fps nhận, thời gian phơi sáng và độ sáng. Chỗ quá tối (log "Anh toi") thì đổi `camera_exposure_mode: auto`.

Node thật với KINGSEN: trước 10 Hz, trễ 77–97 ms → sau 15 Hz (camera 25 fps), 26–33 ms/khung, trễ 27–65 ms, CPU ~5–6 lõi.

**Firmware KINGSEN tự nhảy 25 ↔ 12.5 fps** (đo 30/09): mở camera xong chạy 12.5 fps một lúc (8–60 s) rồi mới lên 25; thỉnh thoảng tự quay về 12.5 fps dù không đổi gì (độ sáng ảnh không đổi). Không điều khiển v4l2 nào khoá được. Giảm được bằng: đặt `v4l2-ctl` **trước** khi mở camera (lên 25 fps sau ~8 s thay vì 17–24 s), bật chống nhấp nháy 50 Hz (15 s thay vì 45–60 s khi tắt), phơi sáng bắt đầu ở 30 ms và chỉ đổi khi ảnh lệch nhiều, tối đa 1 lần/5 s. Ở 12.5 fps phơi sáng vẫn ≤ 30 ms nên ảnh vẫn nét hơn nhiều so với 80 ms trước đây.

**FOV — camera GÓC RỘNG, ống "mắt cá đều".** Người dùng đo 30/09 bằng `scripts/measure_fov.py` (camera cách tường 1.00 m; vạch 1→3 98.5 cm, 3→5 54 cm, 5→7 54 cm, 7→9 98.5 cm). Script khớp mô hình mắt cá OpenCV (Kannala–Brandt), sai số 0.1 px, camera lệch tường 0.1°:

| Vạch (x px) | Đo được | Mô hình khớp | Pinhole cùng FOV | Code cũ trên xe (tuyến tính, 62°) |
|---|---|---|---|---|
| 3 (160) | 28.4° | 28.4° | 37.8° | 15.5° |
| 9 (637) | 56.7° | 56.7° | 57.0° | 30.7° |

→ **FOV ngang 114.4°, dọc 85.4°**; góc gần như tỉ lệ thuận với pixel (f = 324 px). Config: `camera_angle_model: "calibrated"`, `camera_matrix: [324, 0, 320, 0, 324, 240, 0, 0, 1]`, `dist_coeffs: [-0.01077, 0, 0, 0]`, `camera_fisheye: true`. Số 62° dùng từ trước là **sai**: người lệch giữa khung bị báo khoảng **một nửa góc thật** (thật 28.4° → báo 15.5°), lọt khỏi cửa sổ ±10° mà `target_tracker_node` dùng để ghép cụm chân LiDAR, nên tracker rơi về `camera+bbox` với góc sai — có thể là một phần lý do xe xoay theo người chậm trước đây. Không bật `undistort_frame` với ống này (khử méo về pinhole kéo giãn mép ~1.5 lần, góc ảnh ~2.3 lần). `measure_fov.py` đã thử trên ống giả lập (camera lệch tường 2°): khớp lại đúng tham số.

**Độ cao và góc ngửa camera** (FOV dọc đo được 85.4°). ReID Rank-1 khi chỉ thấy một dải cơ thể (Market-1501, mẫu enroll và mẫu nhận cùng kiểu cắt):

| Lắp camera | ở 1 m thấy (m, từ sàn) | Rank-1 ở 1 m | ở 1.5 m |
|---|---|---|---|
| (toàn thân, tham chiếu) | | 75.8% | 75.8% |
| **34 cm, nhìn ngang (hiện tại)** | 0–1.26 (tới ngực) | **58.2%** | toàn thân |
| **34 cm, ngửa 10°** | 0–1.65 | **73.5%** | toàn thân |
| 34 cm, ngửa 12° | toàn thân | 75.8% | toàn thân |
| 70 cm, nhìn ngang | 0–1.62 | 72.6% | toàn thân |

Ngửa camera 34 cm lên **~12°** cho kết quả tương đương (hơi tốt hơn) nâng lên 70 cm. Cách chỉnh: đứng cách xe 1 m, ngửa tới khi thấy cả đầu trong khung; ghi góc vào `camera_pitch_deg`. (Bảng cũ tính theo 62° cho kết quả "34 cm chỉ thấy chân, phải ngửa 20°" — **sai** do FOV sai.) YOLOv5n với người chỉ lộ một phần chỉ cho conf ~0.4–0.7 nên `det_conf_thres` hạ 0.55 → 0.40 (phát hiện sai tăng 2 → 6 trên 128 ảnh coco128; khoá ReID loại được).

### Chạy thật với KINGSEN cắm vào laptop (30/09)

Node tự enroll người trước camera: đủ 80 mẫu sau 35.7 s (có cả mẫu toàn thân lẫn bị cắt khung). Sau đó `TRACKING` với độ giống 0.85–0.98; một đối tượng thứ hai (giống gallery mục tiêu 0.84–0.85, đã học là "không phải mục tiêu" 0.99) bị loại đúng mọi lần; mục tiêu ra khỏi khung rồi quay lại được nhận lại sau 0.4–0.6 s; góc báo từ −50° đến +32°. Trễ từ lúc chụp: trung vị 56 ms, p90 84 ms.

### Mô phỏng offline (ảnh người thật)

`scripts/sim_identity.py` ghép khung từ ảnh Market-1501 theo hình học camera, mô phỏng DeepSORT (kể cả tráo ID khi hai người cắt nhau) và so sánh logic khoá cũ/mới trên cùng kịch bản. Cần tải Market-1501 (xem đầu file).

Kết quả 30/09 (12 lần mỗi dòng, 60 s bám, 4 người khác đi cắt ngang, DeepSORT tráo ID 30% mỗi lần hai người cắt nhau; ảnh mục tiêu lúc bám **khác** ảnh lúc enroll):

| Đám đông | Bản | Hz | Báo nhầm (s/phút) | Thiếu | Nhận lại sau khi ra khỏi khung | Sau khi bị chắn 3 s |
|---|---|---|---|---|---|---|
| Thường (người ngẫu nhiên) | cũ | 8 | 0.02 | 93.9% | 33.7 s | không |
| | cũ + chỉ sửa RGB | 8 | 0.03 | 71.2% | 2.6 s | 5.8 s |
| | **mới** | 8 | 0 | 17.6% | 0.88 s | 0.75 s |
| | **mới** | 15 | 0.01 (2 lần × 1 khung) | 17.7% | 0.07 s | 0.77 s |
| Khó (2/4 người là người **giống mục tiêu nhất trong 750**) | cũ | 8 | 0 | 96.6% | không | không |
| | **mới** | 8 | 0.38 (max 3.0) | 33.7% | 0.38 s | 1.9 s |
| | **mới** | 15 | 0.33 (max 1.6) | 26.7% | 0.10 s | 2.5 s |

*(Bảng trên chạy với giả định cũ: FOV 62°, camera cao 0.6 m — người ở gần bị cắt khung nhiều nên bản cũ rất tệ.)*

**Chạy lại với hình học camera thật** (ống mắt cá 114.4°, cao 34 cm; cả hai bản 8 Hz; 12 lần mỗi dòng):

| Đám đông | Camera | Bản | Nhầm (s/phút) | Thiếu | Nhận lại | Sau khi bị chắn | Rớt khoá |
|---|---|---|---|---|---|---|---|
| Thường | nhìn ngang | cũ | 0.12 | 32.8% | 0.38 s | 0.62 s | 5.2 |
| Thường | nhìn ngang | **mới** | 0.03 | 20.2% | 0.38 s | 0.56 s | 1.2 |
| Thường | ngửa 12° | cũ | 0.21 | 25.3% | 0.38 s | 0.56 s | 3.3 |
| Thường | ngửa 12° | **mới** | **0** | **19.0%** | 0.38 s | 0.62 s | **1.1** |
| 2 người giống nhất | nhìn ngang | cũ | 0.29 | 64.9% | 0.31 s | 4.06 s | 6.7 |
| 2 người giống nhất | nhìn ngang | **mới** | **0** | 43.7% | 0.25 s | 1.75 s | 6.2 |
| 2 người giống nhất | ngửa 12° | cũ | 0.12 | 56.8% | 0.25 s | 1.50 s | 7.8 |
| 2 người giống nhất | ngửa 12° | **mới** | **0** | **30.6%** | 0.31 s | 1.19 s | **4.8** |

Phần lớn lần nhầm còn lại trước khi sửa lần cuối nằm ở trạng thái `TRACK_VERIFY_WARN`: DeepSORT tráo ID đúng lúc hai người chồng nhau, track đang khoá (giờ là người kia, đang bị che) vẫn được báo trong khi mục tiêu thật đứng rõ ngay bên cạnh. Bây giờ: đối thủ ảnh sạch khớp ≥ 0.88 và hơn track đang khoá ≥ 0.16 thì bỏ khoá ngay; kiểm tra hỏng vì dấu hiệu "là người khác" thì giữ khoá nhưng **tạm không báo** (`TRACK_VERIFY_HOLD`, `target_found: false`). Đổi lại tỉ lệ thiếu tăng ~1.5 điểm. Mô phỏng không tính méo ở mép ảnh ngoài phép chiếu mắt cá và coi YOLO luôn phát hiện được người lộ ≥ 35% (lạc quan cho cả hai bản).

Bản cũ "không nhầm" vì gần như không bám được ai sau lần mất đầu tiên. Ca khó là giới hạn của ngoại hình: người giống mục tiêu 0.90 (bằng trung vị của chính người thật) xuất hiện khi mục tiêu đang khuất thì không có gì để phân biệt — bản mới dùng thời gian thử thách 3 s sau khi nhận lại để chuyển sang người giống hơn, nên phần lớn lần nhầm chỉ vài khung.

## YOLO26n + OSNet x0.5 bằng ONNX Runtime (09/10) — chống khoá nhầm người, chạy được trên mini PC

**Vấn đề:** xe thỉnh thoảng khoá nhầm sang người khác dù khác màu áo quần. Phân tích log thật 08/10: mạng ReID của DeepSORT (`ckpt.t7`, học trên Market-1501) cho người lạ điểm 0.81–0.89 so với chủ, ngang ngưỡng nhận lại 0.80–0.86. Đổi YOLO **không** sửa được việc này (YOLO chỉ báo "có người"), nên đổi mạng ReID và đặt lại ngưỡng, đồng thời nâng detector để chạy nhanh trên mini PC i5-5200U.

**Dữ liệu thật** (`~/reid_data/20261009_145308`, ghi bằng `scripts/record_reid_data.py`, 2 người: A áo xanh sáng + quần sẫm, B áo đen + quần sáng, 8 pha, 3418 khung). Chạy lại **toàn bộ** logic camera trên đoạn quay (`scripts/replay_identity.py`, enroll A ở pha 2, chấm bbox mục tiêu theo màu áo/quần):

| Cấu hình | Khoá nhầm | Pha chỉ có B (chủ vắng) | Đợt nhầm dài nhất | Bám chủ khi đi tự do |
|---|---|---|---|---|
| Cũ: YOLOv5n PyTorch + `ckpt.t7` | 227/2630 khung (8.6%) | **60.1%** | **8.9 s** | 47.5% |
| **Mới: YOLO26n 416 + OSNet x0.5** | **2/2630 (0.1%)** — cả hai là hộp dự đoán 1 khung khi YOLO sót | 0.3% (1 khung) | 1 khung | **59.4%** |

Lần nhầm 8.9 s của bản cũ: kho "người lạ" còn trống (B chưa từng đứng cùng lúc với chủ) → B đạt 0.856 > ngưỡng 0.85 → nhận nhầm → kho ảnh chủ học luôn B (ngưỡng học 0.70 < điểm của B) → điểm tự tăng lên 0.95.

Phân bố điểm so với kho chủ (`scripts/eval_reid_data.py`): `ckpt.t7` chủ p10 0.876 / người lạ trung vị 0.794, tối đa 0.870, người ở xa (bbox < 150 px) AUC chỉ 0.80; **OSNet x0.5** chủ p5 0.798 / người lạ tối đa **0.707**, người ở xa AUC 0.99.

Detector (`scripts/eval_detector_data.py`, 2854 khung biết trước số người):

| Detector | Đúng số người | Pha khó (sát mép, rất gần, xa, nấp) | Thời gian |
|---|---|---|---|
| YOLOv5n 640×480 (cũ) | 91.3% | 85.2% | 19.9 ms |
| YOLOv5n 416×320 | 83.6% (thừa hộp 7.8%) | 81.8% | 8.9 ms |
| **YOLO26n 416×320 @0.30** | **91.8%** | **85.5%** | **6.4 ms** |

Tốc độ cả pipeline, ghim **2 lõi** (giả lập mini PC i5-5200U), 4 người: cũ 70.6 ms/khung → **~21 ms**. OSNet x0.25 nhanh gấp ~2 lần x0.5, khoá nhầm trên dữ liệu này cũng 0.1% nhưng tách người kém hơn (d′ 5.6 so với 6.3) — để dành nếu mini PC không kịp.

**Cấu hình** (`person_follow_robot/config/identity_lock_kingsen.yaml`): `detector_onnx: "yolo26n_416x320.onnx"`, `reid_onnx: "osnet_x0_5_msmt17.onnx"`, `ort_threads: 2`, `det_conf_thres: 0.30`, `deepsort_max_dist: 0.30` và các ngưỡng ReID mới (mỗi dòng ghi kèm giá trị cũ "(ckpt.t7: …)"). Để trống `detector_onnx` / `reid_onnx` và trả các ngưỡng về giá trị cũ là quay lại bản PyTorch. Mô hình nằm trong `person_reid_tracker/model_assets/` (xuất ONNX từ `yolo26n.pt` của Ultralytics 8.4.174 và OSNet MSMT17 của torchreid). Giấy phép: YOLO26 là AGPL-3.0 (như YOLOv5), OSNet/torchreid là MIT.

**Triển khai lên mini PC:**

```bash
# Python của ROS cần thêm onnxruntime (--no-deps: không đụng numpy 1.26 của ROS; 1.31 đã kiểm chạy được)
python3 -m pip install --user --no-deps --break-system-packages onnxruntime==1.31.0
python3 -c "import onnxruntime, numpy; print(onnxruntime.__version__, numpy.__version__)"
# Build LUÔN với --symlink-install (bản chép trong install/ từng làm xe chạy mã cũ — CLAUDE.md 7.3)
colcon build --symlink-install --packages-select person_reid_tracker person_follow_identity person_follow_robot
```

Chạy rồi xem log `camera: nhan … fps, xu ly … Hz, TB … ms/khung`. Nếu xử lý < 10 Hz: hạ `processing_hz` về 10, hoặc đổi sang `osnet_x0_25_msmt17.onnx` với bộ ngưỡng riêng (đo cùng dữ liệu): `deepsort_max_dist 0.28, reid_threshold 0.75, current_min_reid 0.61, recover_min_reid 0.71, recover_strong_reid 0.84, reid_margin 0.09, global_search_min_reid 0.74, recovery_single_candidate_min_reid 0.74`, còn lại như x0.5.

**Ghi dữ liệu mới + đánh giá lại** (khi đổi camera, độ cao/góc camera, ánh sáng, hay mô hình):

```bash
python3 scripts/record_reid_data.py                     # 2 người A/B, ~5 phút, theo hướng dẫn trên cửa sổ
python3 scripts/eval_reid_data.py                       # so 6 mạng ReID (~/.cache/reid_models/*.pth)
~/.venvs/yolo/bin/python scripts/eval_detector_data.py  # so detector (cần onnxruntime; model trong ~/.cache/yolo_models)
python3 scripts/replay_identity.py                      # chạy lại toàn bộ logic với cấu hình hiện tại: số khung khoá nhầm
python3 scripts/replay_identity.py --set reid_onnx=… --override nguong.yaml --name thu   # thử cấu hình khác
```

Dữ liệu ghi nằm ở `~/reid_data/` (ngoài repo, có ảnh người — không đẩy lên GitHub). Ngưỡng ReID đặt theo nguyên tắc: nhận lại / học kho chủ phải **cao hơn mức tối đa của người lạ**, giữ khoá dưới p1 của chủ, khoảng chênh nhân theo độ giãn thang điểm.

## Nếu có GPU

Config dùng `device: auto`. Nếu PyTorch nhìn thấy CUDA hoặc ROCm, package sẽ dùng backend đó. Nếu không, nó tự chạy CPU. Với AMD Radeon trên laptop, PyTorch thường không dùng được GPU nếu chưa cài ROCm build, nên CPU là mặc định an toàn.
