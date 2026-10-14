#!/usr/bin/env bash
# Tao container ROS 2 'ros2_jazzy' cho robot bam nguoi tu image robot-jazzy:1.
#   --privileged + /dev:/dev : cong USB (khung xe, LiDAR, 3 board RSSI, camera), cam/rut lai van thay
#   --network host           : trang web robot_web nghe thang tren IP cua may (cong 8080)
#   /home/xiaoqiang          : cung duong dan trong/ngoai -> ~/rssi_env.sh, ~/.config/robot_web.conf,
#                              ~/bags, ~/.local (torch, onnxruntime) nam tren may that
# Web + he robot (run_full.sh) chay CHUNG container nay (web tim tien trinh qua /proc).
# Vao container: docker exec -it ros2_jazzy bash
set -e
NAME=${NAME:-ros2_jazzy}
IMAGE=${IMAGE:-robot-jazzy:1}

if docker container inspect "$NAME" >/dev/null 2>&1; then
    echo "Da co container $NAME — dung/doi ten no truoc (docker stop $NAME; docker rename $NAME ${NAME}_old)" >&2
    exit 1
fi

docker run -d --name "$NAME" \
    --privileged -v /dev:/dev \
    --network host --ipc host \
    -v /home/xiaoqiang:/home/xiaoqiang \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY="${DISPLAY:-:0}" \
    "$IMAGE"
docker ps --filter "name=^${NAME}$" --format '{{.Names}}  {{.Image}}  {{.Status}}'
