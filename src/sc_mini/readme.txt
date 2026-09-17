# sc_mini ROS 2 Jazzy port

Đây là bản port tối thiểu từ source ROS 1/catkin sang ROS 2/ament cho SC Mini LiDAR.

## Build trên ROS 2 Jazzy

```bash
cd ~/ros2_ws/src
# copy thư mục sc_mini này vào ~/ros2_ws/src

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select sc_mini --symlink-install
source install/setup.bash
```

## Cài udev rule

```bash
cd ~/ros2_ws/src/sc_mini
sudo cp sc_mini.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Rút cắm lại LiDAR, rồi kiểm tra:

```bash
ls -l /dev/sc_mini
ls -l /dev/ttyUSB*
```

Nếu chưa có `/dev/sc_mini`, có thể chạy bằng port trực tiếp, ví dụ `/dev/ttyUSB0`.

## Chạy driver

```bash
ros2 launch sc_mini sc_mini.launch.py
```

Nếu dùng `/dev/ttyUSB0`:

```bash
ros2 launch sc_mini sc_mini.launch.py port:=/dev/ttyUSB0
```

## Chạy driver kèm RViz2

```bash
ros2 launch sc_mini view_sc_mini.launch.py
```

Hoặc:

```bash
ros2 launch sc_mini view_sc_mini.launch.py port:=/dev/ttyUSB0
```

## Test topic

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```

RViz2:

- Fixed Frame: `laser`
- Add -> By topic -> `/scan` -> LaserScan

## Service bật/tắt motor

```bash
ros2 service call /stop_motor std_srvs/srv/Empty {}
ros2 service call /start_motor std_srvs/srv/Empty {}
```

## Ghi chú

- Mặc định: `port=/dev/sc_mini`, `baud_rate=115200`, `frame_id=laser`.
- Bản port này chưa được compile trực tiếp trong môi trường ChatGPT vì container không có ROS 2 Jazzy. Nếu build lỗi, gửi log `colcon build --packages-select sc_mini --event-handlers console_direct+` để sửa tiếp.
