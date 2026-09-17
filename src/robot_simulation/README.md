# Robot Simulation Package

ROS 2 Jazzy simulation package cho robot 4 bánh tùy chỉnh.

## Thông số robot

| Thông số | Giá trị |
|----------|---------|
| Bánh xe chính (phía trước) | Đường kính 12cm |
| Khoảng cách ngang 2 bánh chính | 40cm |
| Bánh xe nhỏ linh động (phía sau) | Đường kính 4.5cm |
| Khoảng cách ngang 2 bánh nhỏ | 31cm |
| Khoảng cách trục trước-sau | 24.5cm |
| Chiều dài khung xe | 57cm |
| Chiều rộng khung xe | 46cm |
| Chiều cao đến thiết bị | 26cm |
| Khoảng sáng gầm | 4cm |

## Cấu trúc package

```
robot_simulation/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── robot_sim.launch.py      # Launch chính (Gazebo + RViz + Controllers)
│   └── robot_teleop.launch.py   # Teleop bằng bàn phím
├── urdf/
│   ├── robot.urdf.xacro         # File URDF chính
│   ├── robot_core.xacro         # Cấu trúc robot (khung, bánh xe)
│   ├── materials.xacro          # Định nghĩa vật liệu
│   ├── ros2_control.xacro       # ros2_control cho Gazebo
│   └── gazebo.xacro             # Plugin Gazebo Harmonic
├── config/
│   ├── robot_controllers.yaml   # Cấu hình diff_drive_controller
│   └── teleop_twist_joy.yaml    # Cấu hình joystick
└── rviz/
    └── robot.rviz               # Cấu hình RViz
```

## Cài đặt

### 1. Prerequisites

Cài đặt ROS 2 Jazzy và Gazebo Harmonic:

```bash
# ROS 2 Jazzy (đã có sẵn)
# Gazebo Harmonic
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge

# ros2_control
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control

# Các package bổ sung
sudo apt install ros-jazzy-teleop-twist-keyboard ros-jazzy-xacro
```

### 2. Build package

```bash
cd ~/ros2_ws/src
cp -r /path/to/robot_simulation .
cd ~/ros2_ws
colcon build --packages-select robot_simulation
source install/setup.bash
```

## Chạy mô phỏng

### Launch đầy đủ (Gazebo + RViz + Controllers)

```bash
ros2 launch robot_simulation robot_sim.launch.py
```

### Chỉ Gazebo (không RViz)

```bash
ros2 launch robot_simulation robot_sim.launch.py use_rviz:=false
```

### Teleop bằng bàn phím

Mở terminal mới:

```bash
ros2 launch robot_simulation robot_teleop.launch.py
```

Hoặc dùng `ros2 run`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

### Gửi lệnh velocity trực tiếp

```bash
# Tiến thẳng
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once

# Xoay
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}" --once

# Dừng
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Topics quan trọng

| Topic | Type | Mô tả |
|-------|------|-------|
| `/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` | Điều khiển vận tốc |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | Odometry |
| `/tf` | `tf2_msgs/TFMessage` | Transform |
| `/joint_states` | `sensor_msgs/JointState` | Trạng thái khớp |
| `/robot_description` | `std_msgs/String` | URDF |

## Kiểm tra controllers

```bash
# Liệt kê controllers
ros2 control list_controllers

# Trạng thái diff_drive_controller
ros2 control list_hardware_interfaces

# Thông tin robot
ros2 topic echo /diff_drive_controller/odom
ros2 topic echo /joint_states
```

## Ghi chú

- Driver BW-DR03 được mô phỏng qua `gz_ros2_control` plugin
- 2 bánh chính (front_left, front_right) được điều khiển bởi `diff_drive_controller`
- 2 bánh caster (rear_left, rear_right) tự do xoay (passive)
- `base_link` đặt tại mặt đất (z=0) để tương thích với navigation stack

## Troubleshooting

### Lỗi "gz sim not found"

```bash
source /usr/share/gazebo/setup.bash
# hoặc
export GZ_VERSION=harmonic
```

### Lỗi controller không load được

```bash
ros2 run controller_manager spawner diff_drive_controller --controller-manager /controller_manager
```

### Kiểm tra robot_description

```bash
ros2 param get /robot_state_publisher robot_description
```
