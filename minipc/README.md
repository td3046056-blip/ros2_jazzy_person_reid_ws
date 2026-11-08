# Mini PC trên xe (i5-5200U, 8 GB) — cài đặt và vận hành

Máy thật: **Ubuntu 20.04 + ROS 1 Noetic của hãng Xiaoqiang** → ROS 2 Jazzy (cần Ubuntu 24.04) chạy **trong Docker**.
Thư mục này là bản sao các file nằm NGOÀI repo trên máy (`~/robot_docker/`, `~/rssi_env.sh`) để cài lại được.

| File | Đặt ở đâu trên máy | Vai trò |
|---|---|---|
| `Dockerfile` | `~/robot_docker/` | Image `robot-jazzy:2` (osrf/ros:jazzy-desktop + serial, scipy, opencv, rosbag2, v4l-utils, psmisc...) |
| `run_container_auto.sh` | `~/robot_docker/` | Tạo container `ros2_jazzy` **tự chạy khi bật máy** (`--restart unless-stopped --init`, giờ theo máy thật) |
| `run_container.sh` | `~/robot_docker/` | Tạo container chạy tay (`sleep infinity`) |
| `start_web.sh` | `~/robot_docker/` | Lệnh chính của container: giữ web `robot_web` luôn chạy; KHÔNG tự bật hệ robot; `docker stop` → `/follow/stop` → Ctrl-C launch |
| `web.env` | `~/robot_docker/` | `WEB_IMAGE_MAX_FPS=5` (đọc lại mỗi lần web khởi động) |
| `rssi_env.sh.example` | `~/rssi_env.sh` | Cổng thiết bị (`PL PA PB PC PROBOT PCAM`) — đổi ổ cắm USB thì sửa |

Mật khẩu web ở `~/.config/robot_web.conf` (JSON `{"password": "..."}`, chmod 600) — **không** đưa vào repo.

## Cài từ đầu
```bash
mkdir -p ~/robot_docker && cp minipc/{Dockerfile,run_container*.sh,start_web.sh,web.env} ~/robot_docker/
cp minipc/rssi_env.sh.example ~/rssi_env.sh            # rồi kiểm lại cổng (dưới)
docker build -t robot-jazzy:2 ~/robot_docker
bash ~/robot_docker/run_container_auto.sh
# torch + onnxruntime vào ~/.local (nằm trên máy thật, giữ khi tạo lại container), numpy giữ bản 1.26 của ROS:
docker exec -it ros2_jazzy pip install --user --no-deps --break-system-packages onnxruntime
docker exec -it ros2_jazzy bash -c 'cd ~/ros2_ws/ros2_jazzy_person_reid_ws && colcon build --symlink-install'
```
Vào container: `docker exec -it ros2_jazzy bash`. Web: `http://<IP mini PC>:8080`.

## Đã làm trên máy thật (10–13/10, có hỏi người dùng)
- Tắt `startup.service` (hệ ROS 1 Xiaoqiang): `node_controller.py` cứ 300 s `pkill -2 sc_mini` → giết cả driver LiDAR ROS 2 trong container.
- Tắt `ModemManager`, `mongod`, `sharplink`; gỡ `brltty` (chiếm CH340); tắt `apt-daily.timer` + `apt-daily-upgrade.timer`
  (tự cập nhật chiếm 1 nhân ~75 phút sau mỗi lần bật máy và có thể khởi động lại Docker giữa lúc xe chạy).
  Cập nhật tay khi xe nghỉ: `sudo apt update && sudo apt upgrade`.
- Turbo CPU bị BIOS tắt (`no_turbo=1`, tối đa 2.1 GHz) — để nguyên (người dùng chọn).

## Cổng thiết bị (13/10)
- Luật udev cũ của hãng `/etc/udev/rules.d/95-persistent-serial.rules` dùng `SYMLINK=` cho cổng 2-2 / 2-4 / 2-7 → **xoá mất**
  đường dẫn `by-id` / `by-path` của thiết bị ở các cổng đó. Vì vậy: khung xe (FTDI, cổng 2-2) = `/dev/stm32Car`,
  LiDAR (CH340, cổng 2-7) = `/dev/galileo_serial`; 3 board RSSI qua hub ở cổng 2-3 = `by-path ...usb-0:3.{1,2,3}`.
- Cắm hub RSSI sang cổng khác → `by-path` đổi → `run_full.sh --check` báo HỎNG (gặp 13/10 14:59).
- Camera KINGSEN cắm cổng 2-5 từng lỗi `-71` (không khởi tạo được) → dùng cổng 2-4.

## Mạng
- Router trên xe: mini PC `192.168.1.104` (DHCP), điện thoại / laptop vào Wi-Fi router, mở `http://192.168.1.104:8080`.
- Cách cũ: dây LAN tới laptop chia mạng (mini PC `10.42.0.11`).
- ROS 2 chỉ liên lạc trong máy (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) → máy khác cùng mạng không thể ghi `/cmd_vel` lên xe.

## Hiệu năng sau tối ưu (13/10) — chi tiết trong CLAUDE.md mục 8
Xe đứng yên, hệ đầy đủ: CPU ~50–60 % (trước ~90 %). Lúc bám: planner 15 Hz, tracker 20 Hz, LiDAR 10 Hz, odom 50 Hz;
camera OSNet x0.5: 1 người ~6–8 Hz, 2 người ~5–6 Hz, 3 người ~4–5 Hz (laptop ~15 Hz).
