#!/usr/bin/env bash
# =============================================================================
# run_container_auto.sh — tao container 'ros2_jazzy' TU CHAY khi bat may (buoc 8, 10/10)
# Khac run_container.sh (chay tay, CMD sleep infinity):
#   --restart unless-stopped : Docker (enabled luc boot) tu bat lai container; container chet cung tu bat lai.
#                              'docker stop ros2_jazzy' thi thoi (toi khi 'docker start ros2_jazzy').
#   --init                   : PID 1 la docker-init, don tien trinh mo coi (run_full.sh, ros2 launch...)
#   --stop-timeout 30        : du thoi gian /follow/stop + Ctrl-C launch (start_web.sh) truoc khi SIGKILL
#   /etc/localtime           : gio trong container = gio may that (truoc la UTC -> ten log lech gio)
#   CMD start_web.sh         : giu web robot_web luon chay; KHONG tu bat he robot (bam Khoi dong tren web)
# Doi image_max_fps cua web: WEB_IMAGE_MAX_FPS=15 bash run_container_auto.sh
# Vao container: docker exec -it ros2_jazzy bash      Log web: run_logs/web_autostart.log
# =============================================================================
set -e
NAME=${NAME:-ros2_jazzy}
IMAGE=${IMAGE:-robot-jazzy:2}

if docker container inspect "$NAME" >/dev/null 2>&1; then
    echo "Da co container $NAME — dung/doi ten no truoc (docker stop $NAME; docker rename $NAME ${NAME}_old)" >&2
    exit 1
fi

docker run -d --name "$NAME" \
    --restart unless-stopped --init --stop-timeout 30 \
    --privileged -v /dev:/dev \
    --network host --ipc host \
    -v /home/xiaoqiang:/home/xiaoqiang \
    -v /etc/localtime:/etc/localtime:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e WEB_IMAGE_MAX_FPS="${WEB_IMAGE_MAX_FPS:-8}" \
    "$IMAGE" bash /home/xiaoqiang/robot_docker/start_web.sh
docker ps --filter "name=^${NAME}$" --format '{{.Names}}  {{.Image}}  {{.Status}}'
