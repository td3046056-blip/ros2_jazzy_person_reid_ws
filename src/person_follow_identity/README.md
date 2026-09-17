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

## Nếu có GPU

Config dùng `device: auto`. Nếu PyTorch nhìn thấy CUDA hoặc ROCm, package sẽ dùng backend đó. Nếu không, nó tự chạy CPU. Với AMD Radeon trên laptop, PyTorch thường không dùng được GPU nếu chưa cài ROCm build, nên CPU là mặc định an toàn.
