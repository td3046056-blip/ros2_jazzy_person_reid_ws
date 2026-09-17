# Mô Tả Hệ Thống Robot Bám Người — robot_rssi_ros2

## Phần Cứng
- **Robot:** BW-DR03 (differential drive, 2 bánh)
- **Lidar:** SC-Mini 2D (360°, 20Hz), lắp trên xe với góc 90° là hướng thẳng trước
- **RSSI:** NodeMCU ESP32 đọc BLE beacon người đeo, publish góc và confidence
- **Driver xe:** `bw_dr03_ros2` package, node `decoded_serial_node`
- **Cổng USB cố định (udev rules):**
  - `/dev/robot_lidar` → Lidar SC-Mini
  - `/dev/robot_base` → Robot BW-DR03
  - `/dev/robot_rssi` → Mạch RSSI NodeMCU

## Thuật Toán Chính — VFH + RSSI
- **File:** `robot_rssi_ros2/vfh_rssi_controller.py`
- **Thuật toán:** VFH (Vector Field Histogram) tùy biến cho bài toán bám người
- **Ưu tiên nguồn điều khiển:**
  1. Camera ReID (`/person_reid/target`) — PRIMARY
  2. RSSI BLE (`/rssi/angle_deg`) — FALLBACK
  3. Bộ nhớ Alpha-beta 2D — khi mất tín hiệu tạm thời
  4. Teleop supervisor (`/cmd_vel_teleop`) — khi test riêng VFH

### State Machine Các Mode:
| Mode | Điều kiện | Hành vi |
|------|-----------|---------|
| FOLLOW | Camera thấy người | Bám theo camera |
| FOLLOW_RSSI | Chỉ có RSSI | Bám theo RSSI bearing |
| RECOVER_TARGET | Mất tín hiệu tạm | Xoay về hướng nhớ cuối cùng |
| SLOW_GAP | front < 1.20m | Chậm lại, bẻ nhẹ |
| AVOID_GAP | front < 0.80m | Gần dừng, bẻ gấp |
| AVOID_TURN_ONLY | front < 0.50m | Dừng hoàn toàn, xoay tại chỗ (bypass EMA) |
| ESCAPE_TURN | Kẹt > 2.0s | Xoay mạnh thoát ra (bypass EMA) |
| ESCAPE_REVERSE | Kẹt, có chỗ lùi | Lùi ra sau |
| BACKTRACK | ESCAPE_TURN > 5s | Đi ngược vết đường cũ thoát bẫy chữ U |
| TELEOP | Có lệnh teleop, không có gì khác | Forward lệnh teleop |
| WAIT_TARGET | Không có tín hiệu gì | Đứng im |

### Thông Số Quan Trọng (trong config/vfh_params.yaml):
- `safe_distance: 0.80` — Bắt đầu né gấp (m)
- `emergency_distance: 0.50` — Dừng + xoay tại chỗ (m)
- `avoid_angular: 0.45` — Vận tốc xoay khi né bình thường (rad/s)
- `avoid_angular_fast: 0.65` — Vận tốc xoay khẩn cấp (rad/s)
- `stuck_time: 2.0` — Thời gian bị chặn trước khi trigger ESCAPE (s)
- `max_linear: 0.12` — Tốc độ tiến tối đa (m/s)
- `max_percent: 60` — % PWM motor tối đa trong driver BW-DR03

### Các Fix Quan Trọng Đã Thực Hiện:
1. **Bug#2:** w_smooth gốc=3.0 gây góc né nhỏ → giảm xuống 1.5
2. **Bug#3:** EMA filter gây trễ phản ứng khẩn cấp → bypass EMA cho AVOID_TURN_ONLY/ESCAPE
3. **Bug#4:** _front_obstacle() exclude target → gây xe lao vào vật cản sau người → đổi sang không exclude
4. **Bug#5:** rssi_ema_alpha=0.30 gây trễ RSSI 640ms → tăng lên 0.55
5. **Bug#6:** stuck_time=3.5s quá dài → giảm xuống 2.0s
6. **Anti-oscillation:** Hysteresis 8cm ở AVOID_TURN_ONLY và ESCAPE_TURN để tránh mode flip
7. **max_percent fix:** Driver BW-DR03 mặc định 30% → tăng lên 60% để bánh xe quay được
8. **avoid_angular_fast:** Thêm tham số mới (0.65 rad/s) riêng cho mode khẩn cấp

## Lệnh Chạy Hệ Thống

### Test VFH + Teleop (không RSSI):
```bash
# T1: Lidar
ros2 launch sc_mini sc_mini.launch.py port:=/dev/robot_lidar

# T2: VFH + Driver
ros2 launch robot_rssi_ros2 vfh_test_only.launch.py

# T3: Teleop (publish sang /cmd_vel_teleop để không conflict với VFH)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_teleop
```

### Chạy Thực Tế Với RSSI:
```bash
# T1: Lidar
ros2 launch sc_mini sc_mini.launch.py port:=/dev/robot_lidar

# T2: Toàn bộ stack
ros2 launch robot_rssi_ros2 rssi_vfh_follow.launch.py

# T3: Bật bám
ros2 service call /rssi_follow/enable std_srvs/srv/Trigger

# Dừng khẩn cấp
ros2 service call /rssi_follow/stop std_srvs/srv/Trigger
```

### Monitor:
```bash
ros2 topic echo /rosout | grep -E "mode=|MODE"
```

## Rebuild Sau Khi Sửa:
```bash
cd /home/thaihoa/ros2_ws
colcon build --base-paths src/ros2_jazzy_person_reid_ws --packages-select robot_rssi_ros2
source install/setup.bash
```
