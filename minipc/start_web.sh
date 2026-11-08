#!/usr/bin/env bash
# =============================================================================
# start_web.sh — lenh chinh (CMD) cua container ros2_jazzy khi tu chay luc bat may (buoc 8, 10/10)
#   - Giu trang web robot_web luon chay: web thoat / chet thi tu bat lai sau 3 s.
#   - KHONG tu bat he robot: nguoi dung bam "Khoi dong" tren dien thoai (start_enabled: false nhu cu).
#   - He robot do web bat (run_full.sh) o nhom tien trinh rieng -> web khoi dong lai KHONG tat xe.
#   - docker stop / tat may (SIGTERM): /follow/stop -> Ctrl-C nhom launch + nhom ghi bag -> tat web.
# PID 1 la docker-init (docker run --init): don tien trinh mo coi.
# Log: run_logs/web_autostart.log (nhat ky nut bam cua web van o run_logs/web_*.log)
# =============================================================================
source /opt/ros/jazzy/setup.bash
WS=$HOME/ros2_ws/ros2_jazzy_person_reid_ws
mkdir -p "$WS/run_logs"
LOG=$WS/run_logs/web_autostart.log
log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

WEB_PID=""
stop_all() {
  trap '' TERM INT
  log "nhan tin hieu tat container -> dung xe + tat he robot"
  timeout 6 ros2 service call /follow/stop std_srvs/srv/Trigger {} 2>&1 | grep -E "success|message" >> "$LOG"
  sleep 0.5
  # Ctrl-C ca nhom tien trinh (nhu nut "Tat he thong" cua web): launch he robot + ros2 bag
  for pat in "follow_nav_real.launch.py" "ros2 bag record"; do
    for pid in $(pgrep -f "$pat"); do
      pg=$(ps -o pgid= -p "$pid" | tr -d ' ')
      [ -n "$pg" ] && kill -INT -- "-$pg" 2>/dev/null && log "SIGINT nhom $pg ($pat)"
    done
  done
  for _ in $(seq 30); do
    pgrep -f "follow_nav_real.launch.py|ros2 bag record" >/dev/null || break
    sleep 0.5
  done
  pgrep -f "follow_nav_real.launch.py|ros2 bag record" >/dev/null && log "CANH BAO: launch/bag chua tat sau 15 s"
  if [ -n "$WEB_PID" ]; then
    kill -INT "$WEB_PID" 2>/dev/null
    wait "$WEB_PID" 2>/dev/null
  fi
  log "da tat"
  exit 0
}
trap stop_all TERM INT

log "container khoi dong"
while true; do
  [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
  # Tham so web doi duoc KHONG can tao lai container: ~/robot_docker/web.env (vd. WEB_IMAGE_MAX_FPS=5),
  # doc lai moi lan web khoi dong. Khong co file thi dung bien cua container (mac dinh 8).
  [ -f "$HOME/robot_docker/web.env" ] && source "$HOME/robot_docker/web.env"
  FPS=${WEB_IMAGE_MAX_FPS:-8}
  log "bat web_server (image_max_fps $FPS)"
  "$WS/install/robot_web/lib/robot_web/web_server" --ros-args -p image_max_fps:="$FPS" >> "$LOG" 2>&1 &
  WEB_PID=$!
  wait "$WEB_PID"
  log "web_server thoat (ma $?) -> bat lai sau 3 s"
  sleep 3
done
