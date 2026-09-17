# person_follow_robot

Package bringup + controller cho robot follow người đã enroll bằng `person_follow_identity` và điều khiển xe BW-DR03 qua `/cmd_vel`.

Luồng chính:

```text
USB Camera -> person_follow_identity -> /person_reid/target
/bw_dr03/sonar -----------------------> person_follow_controller -> /cmd_vel -> bw_dr03_ros2
```

Mục tiêu mặc định: xe giữ khoảng cách người cần follow khoảng **1.0 m**.

## Cài đặt

Đặt package này vào cùng workspace với các package:

- `person_reid_tracker`
- `person_follow_identity`
- `bw_dr03_ros2`
- `robot_model` và `robot_simulation` nếu muốn mô phỏng

```bash
cd ~/ros2_ws/ros2_jazzy_person_reid_ws/src
unzip ~/Downloads/person_follow_robot_package.zip

cd ~/ros2_ws/ros2_jazzy_person_reid_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install --packages-select person_follow_robot bw_dr03_ros2 person_follow_identity
source install/setup.bash
```

## Chạy robot thật

```bash
ros2 launch person_follow_robot person_follow_real.launch.py \
  camera_source:=/dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0 \
  serial_port:=/dev/bw_dr03 \
  target_distance_m:=1.0
```

Controller mặc định **chưa cho xe chạy ngay**. Làm theo thứ tự:

```bash
# 1) Học người A
ros2 service call /person_reid/start_enroll std_srvs/srv/Trigger {}

# Đợi enrollment xong hoặc kết thúc tay khi đủ mẫu
ros2 service call /person_reid/finish_enroll std_srvs/srv/Trigger {}

# 2) Cho phép xe chạy theo người A
ros2 service call /person_follow/enable std_srvs/srv/Trigger {}

# Dừng khẩn / tắt follow
ros2 service call /person_follow/stop std_srvs/srv/Trigger {}
```

## Kiểm tra trạng thái

```bash
ros2 topic echo /person_reid/target
ros2 topic echo /person_follow/status
ros2 topic echo /cmd_vel
ros2 topic echo /bw_dr03/sonar
```

## Calibrate khoảng cách 1m

Đặt người A cách camera/xe đúng 1.0m, enroll và track A, rồi chạy:

```bash
ros2 run person_follow_robot follow_calibrator --ros-args -p target_distance_m:=1.0
```

Nó sẽ gợi ý hai thông số:

- `bbox_height_at_target_distance_px`
- `sonar_scale_to_m`

Sau đó sửa trong:

```bash
src/person_follow_robot/config/follow_controller.yaml
```

## Thông số quan trọng

- `target_distance_m`: khoảng cách mong muốn, mặc định 1.0 m.
- `sonar_scale_to_m`: nếu `/bw_dr03/sonar` hiển thị 100 ở 1m thì để 0.01; nếu hiển thị 1.0 thì để 1.0.
- `bbox_height_at_target_distance_px`: chiều cao bbox khi người đứng đúng 1m.
- `angular_sign`: nếu xe quay ngược hướng người, đổi từ `-1.0` sang `1.0`.
- `max_linear_forward`, `max_angular`: giới hạn tốc độ.
- `emergency_stop_m`: sonar nhỏ hơn ngưỡng này thì dừng.

## Nguyên tắc an toàn

Khi mất target hoặc identity chưa chắc chắn, controller sẽ gửi STOP. Không bật `search_when_lost` cho demo thật nếu chưa có bumper/LiDAR/safety đầy đủ.
