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
| BLAS của numpy 1 luồng (`OPENBLAS_NUM_THREADS=1` đầu `node.py`) | DeepSORT tích luỹ 100 feature/track rồi tính khoảng cách bằng tích ma trận; OpenBLAS đa luồng quay chờ bận, tranh CPU với torch → xử lý trượt từ 29 lên 53–59 ms/khung sau vài giây. Sau khi sửa: ổn định 25–29 ms. **Không** đặt `MKL_NUM_THREADS`/`OMP_NUM_THREADS` — torch đọc hai biến đó và chạy 1 luồng (66 ms) |
| Tổng thể (khung 6 người, CPU máy này) | cũ 68.7 ms/khung (config cũ còn khoá 8 Hz) → mới 40.3 ms; webcam 0–1 người: 15.0 Hz, 25–29 ms, trễ từ lúc chụp 30–60 ms |
| `ts` trong `/person_reid/target` = **lúc chụp** | Trước là lúc xử lý xong → tracker gần như không bù được góc xe quay trong lúc xử lý |
| Biết ai che ai; khung bị che không học vào gallery, chỉ trừ ít điểm | Trước: một khung bị che/quay lưng là bỏ khoá (`verification_fail_limit: 1`) |
| Bỏ khoá **ngay** khi mâu thuẫn rõ | Ảnh giống người đã biết hơn mục tiêu, người khác giống mục tiêu hơn hẳn, hoặc ngoại hình track đổi đột ngột (DeepSORT tráo ID) |
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

Node thật với KINGSEN: trước 10 Hz, trễ 77–97 ms → sau 15 Hz (camera 25 fps), 27–33 ms/khung, trễ 38–65 ms.

**FOV.** Ảnh 640×480 là **ảnh cắt giữa** 1440×1080 của cảm biến (so khớp đặc trưng giữa hai chế độ, điểm ảnh vuông) → FOV ngang hẹp hơn chế độ 1080p. Nếu 62° là thông số của nhà sản xuất cho 1080p thì ở 640×480 chỉ ~43–49°. Đo bằng thước: `python3 scripts/measure_fov.py` (hướng dẫn trong file), dán `camera_fov_deg` vào yaml.

**Độ cao và góc ngửa camera.** ReID Rank-1 khi chỉ thấy một dải cơ thể (Market-1501, mẫu enroll và mẫu nhận cùng kiểu cắt; giả định FOV dọc 48.5° — nếu FOV hẹp hơn thì thấy ít hơn nữa):

| Lắp camera | ở 1 m thấy (m, tính từ sàn) | Rank-1 ở 1 m | Rank-1 ở 1.5 m |
|---|---|---|---|
| (toàn thân, tham chiếu) | | 75.8% | 75.8% |
| 34 cm, nhìn ngang | 0–0.79 (chân) | **22.4%** | 44.9% |
| 34 cm, ngửa 20° | 0.27–1.31 | **53.3%** | **67.3%** |
| 70 cm, nhìn ngang | 0.25–1.15 | 45.3% | 63.5% |
| 70 cm, ngửa 10° | 0.45–1.38 | 50.4% | 63.4% |

Ngửa camera để ở khoảng bám (1 m) mép trên khung ảnh chạm ngang vai: 34 cm → ngửa ~20°, 70 cm → ngửa ~10°. Ghi góc đã lắp vào `camera_pitch_deg` (sửa góc ngang ~1–1.5° ở mép khung). YOLOv5n với người chỉ lộ một phần chỉ cho conf ~0.4–0.7 nên `det_conf_thres` hạ 0.55 → 0.40 (phát hiện sai tăng 2 → 6 trên 128 ảnh coco128; khoá ReID loại được).

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

Bản cũ "không nhầm" vì gần như không bám được ai sau lần mất đầu tiên. Ca khó là giới hạn của ngoại hình: người giống mục tiêu 0.90 (bằng trung vị của chính người thật) xuất hiện khi mục tiêu đang khuất thì không có gì để phân biệt — bản mới dùng thời gian thử thách 3 s sau khi nhận lại để chuyển sang người giống hơn, nên phần lớn lần nhầm chỉ vài khung.

## Nếu có GPU

Config dùng `device: auto`. Nếu PyTorch nhìn thấy CUDA hoặc ROCm, package sẽ dùng backend đó. Nếu không, nó tự chạy CPU. Với AMD Radeon trên laptop, PyTorch thường không dùng được GPU nếu chưa cài ROCm build, nên CPU là mặc định an toàn.
