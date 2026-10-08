#!/usr/bin/env bash
# =============================================================================
# run_full.sh — chay TOAN BO he thong bam nguoi tren xe that (08/10)
#   LiDAR + driver khung xe + camera/ReID + tracker + planner + 2 node RSSI (3 board ESP32)
#
#   bash src/person_follow_nav/scripts/run_full.sh             # kiem tra roi launch (co RSSI)
#   bash src/person_follow_nav/scripts/run_full.sh --check     # chi kiem tra, khong launch
#   bash src/person_follow_nav/scripts/run_full.sh --no-rssi   # chi camera + LiDAR (khong 2 node RSSI)
#   bash src/person_follow_nav/scripts/run_full.sh --force     # launch ca khi kiem tra co muc HONG
#   bash src/person_follow_nav/scripts/run_full.sh -- follow_distance_m:=1.2   # tham so them cho launch
#   bash src/person_follow_nav/scripts/run_full.sh --bag       # (terminal khac) CHI ghi bag: ~/bags/follow_<ngay_gio>
#
# Cong thiet bi lay tu ~/rssi_env.sh (PL = LiDAR, PA PB PC = 3 board RSSI, PROBOT = khung xe,
# PCAM = camera). LiDAR va 3 board cung chip CH340 -> phai dung by-path (by-id trung ten nhau).
# Doi o cam USB thi sua file do. Log launch: run_logs/run_<ngay_gio>.log
#
# Xe KHONG tu chay (start_enabled: false): sau khi launch phai enroll roi goi /follow/enable.
# Dung khan:  ros2 service call /follow/stop std_srvs/srv/Trigger {}
# =============================================================================
G=$'\033[92m'; Y=$'\033[93m'; R=$'\033[91m'; B=$'\033[1m'; X=$'\033[0m'
fail=0
ok()   { echo "  ${G}OK${X}    $1"; }
warn() { echo "  ${Y}CANH BAO${X}  $1"; }
bad()  { echo "  ${R}HONG${X}  $1"; fail=1; }

CHECK_ONLY=0; RSSI=1; FORCE=0; BAG_ONLY=0; EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --check)   CHECK_ONLY=1 ;;
    --no-rssi) RSSI=0 ;;
    --force)   FORCE=1 ;;
    --bag)     BAG_ONLY=1 ;;
    --)        shift; EXTRA=("$@"); break ;;
    -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
    *)         echo "Tham so la: $1 (xem --help)"; exit 2 ;;
  esac
  shift
done

ENV_FILE=${RSSI_ENV:-$HOME/rssi_env.sh}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# setup.bash cua ROS doc bien chua dat -> khong dung set -u
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash

# Ghi bag (08/10: lenh dai dan qua 2 dong bi cat -> bag chi co /scan /odom /cmd_vel, thieu cac topic JSON)
if [ $BAG_ONLY = 1 ]; then
  mkdir -p "$HOME/bags"
  OUT="$HOME/bags/follow_$(date +%m%d_%H%M%S)"
  echo "Ghi bag -> $OUT (Ctrl-C de dung; khong ghi anh camera)"
  exec ros2 bag record -o "$OUT" --topics /scan /odom /cmd_vel /follow/target /follow/planner_status \
    /person_reid/target /rssi/bearing /rssi/status /rssi/raw /tf /tf_static
fi

echo "=============================================================="
echo "${B} KIEM TRA TRUOC KHI CHAY TOAN BO HE THONG${X}  ($([ $RSSI = 1 ] && echo 'co RSSI' || echo 'khong RSSI'))"
echo "=============================================================="

echo
echo "${B}1. File cong thiet bi + workspace${X}"
if [ -f "$ENV_FILE" ]; then
  source "$ENV_FILE"; ok "$ENV_FILE"
else
  bad "khong co $ENV_FILE (can PL PA PB PC PROBOT PCAM — xem CLAUDE.md muc 10)"
fi
WS=${WS:-$(cd "$SCRIPT_DIR/../../.." && pwd)}
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"; ok "workspace $WS"
else
  bad "chua build: khong co $WS/install/setup.bash"
fi

echo
echo "${B}2. brltty (dich vu chu noi cho nguoi mu — hay chiem chip CH340)${X}"
if pgrep -x brltty >/dev/null; then
  bad "brltty dang chay — no chiem LiDAR (CH340 sau hub 1a40:0101). Sua mot lan:"
  echo "        sudo systemctl stop brltty-udev.service"
  echo "        sudo apt remove brltty        # hoac giu goi: sudo systemctl mask brltty-udev.service"
  echo "        roi RUT CAM LAI day USB LiDAR (dung o cu)"
elif [ -f /usr/lib/udev/rules.d/85-brltty.rules ] && [ "$(systemctl is-enabled brltty-udev.service 2>/dev/null)" != "masked" ]; then
  warn "brltty khong chay nhung luat udev con — rut cam USB lan sau co the bi chiem lai (sudo apt remove brltty)"
else
  ok "brltty khong chay, khong tu bat lai"
fi

echo
echo "${B}3. Chip CH340 (LiDAR + 3 board) phai co driver ch341${X}"
n340=0
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = "1a86" ] && [ "$(cat "$d/idProduct")" = "7523" ] || continue
  n340=$((n340 + 1))
  port=$(basename "$d")
  drv=""
  [ -e "$d/$port:1.0/driver" ] && drv=$(basename "$(readlink "$d/$port:1.0/driver")")
  tty=$(ls "$d/$port:1.0" 2>/dev/null | grep -m1 '^ttyUSB')
  if [ "$drv" = "ch341" ]; then ok "USB $port -> ${tty:-?}"
  else bad "USB $port KHONG co driver ch341 (driver: ${drv:-khong co}) — bi brltty giu? go brltty roi rut cam lai"; fi
done
need=$([ $RSSI = 1 ] && echo 4 || echo 1)
[ "$n340" -lt "$need" ] && bad "chi thay $n340 chip CH340, can $need ($([ $RSSI = 1 ] && echo 'LiDAR + 3 board' || echo 'LiDAR'))"

echo
echo "${B}4. Cong thiet bi${X}"
DEVS="PL PROBOT PCAM"
[ $RSSI = 1 ] && DEVS="$DEVS PA PB PC"
declare -A DESC=([PL]="LiDAR" [PROBOT]="khung xe" [PCAM]="camera" [PA]="board A dau xe" [PB]="board B sau phai" [PC]="board C sau trai")
for v in $DEVS; do
  p=${!v}
  if [ -z "$p" ]; then bad "$v (${DESC[$v]}) chua dat trong $ENV_FILE"; continue; fi
  if [ ! -e "$p" ]; then bad "$v = $p KHONG ton tai (${DESC[$v]})"; continue; fi
  real=$(readlink -f "$p")
  pids=$(fuser "$real" 2>/dev/null | xargs)
  if [ -n "$pids" ]; then
    bad "$v -> $real dang bi tien trinh khac giu: $(ps -o pid=,comm= -p "${pids// /,}" 2>/dev/null | xargs)"
  else
    ok "$v -> $real  (${DESC[$v]})"
  fi
done
if [ $RSSI = 1 ] && [ -e "$PL" ]; then
  rl=$(readlink -f "$PL")
  for v in PA PB PC; do
    [ -e "${!v}" ] && [ "$(readlink -f "${!v}")" = "$rl" ] && bad "$v TRUNG thiet bi voi LiDAR ($rl) — sua $ENV_FILE"
  done
fi

if [ $RSSI = 1 ] && [ -e "$PA" ] && [ -e "$PB" ] && [ -e "$PC" ]; then
  echo
  echo "${B}4b. Beacon (doc thu 3 board ~6 s; RSSI chi dung khi MAT nguoi, khong co beacon van chay duoc)${X}"
  # 08/10: beacon phat ~2 phut sau moi lan bat roi tat (nghi sac du phong tu ngat khi dong nho) -> xe khong
  # tim lai duoc nguoi bang RSSI ma khong ai biet. Mo cong lam board khoi dong lai (~1.5 s) — vo hai.
  tmpcsv="${TMPDIR:-/tmp}/beacon_check_$$.csv"
  out=$(timeout 20 python3 "$SCRIPT_DIR/rssi_log.py" --ports "$PA" "$PB" "$PC" --sec 6 --out "$tmpcsv" 2>&1)
  rc=$?
  rm -f "${tmpcsv%.csv}"*.csv
  if [ $rc -eq 0 ]; then
    ok "beacon dang phat (moi board nen >= 10 mau/s):"
    echo "$out" | sed -n '/KET QUA/,$p' | tail -n +2 | sed 's/^/      /'
  elif echo "$out" | grep -q "dang bi tien trinh khac giu"; then
    warn "khong doc thu duoc beacon: cong board dang bi tien trinh khac giu (he thong dang chay?)"
  else
    warn "KHONG thay beacon — xe van bam duoc, nhung MAT nguoi thi khong xoay do RSSI duoc (B7). Kiem tra:"
    echo "$out" | tail -n 2 | sed 's/^/        /'
    echo "        den beacon con sang? cap nguon bang sac du phong: nhieu loai TU NGAT sau ~1-2 phut vi beacon an"
    echo "        qua it dong — dung che do dong nho (thuong bam 2 lan nut) / pin khong tu ngat"
  fi
fi

echo
echo "${B}5. ROS${X}"
echo "        ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0 (mac dinh)}"
if ros2 pkg prefix person_follow_nav >/dev/null 2>&1; then ok "goi person_follow_nav da build"
else bad "khong thay goi person_follow_nav — build lai"; fi
nodes=$(timeout 10 ros2 node list 2>/dev/null)
crit=$(echo "$nodes" | grep -E 'sc_mini|decoded_serial|follow_planner|target_tracker|identity_lock|rssi_|fake_target')
if [ -n "$crit" ]; then
  bad "con node cu dang chay — chay them launch se co 2 nguon ghi /cmd_vel. Don truoc (CLAUDE.md muc 10):"
  echo "$crit" | sed 's/^/        /'
  echo "        pkill -f sc_mini; pkill -f decoded_serial; pkill -f follow_planner; pkill -f target_tracker"
  echo "        pkill -f identity_lock; pkill -f rssi_; pkill -f fake_target; pkill -f ros2"
elif [ -n "$nodes" ]; then
  warn "co node khac dang chay:"; echo "$nodes" | sed 's/^/        /'
else
  ok "khong co node cu"
fi

echo
echo "${B}6. install/ phai la symlink ve src (build --symlink-install, CLAUDE.md 7.3)${X}"
nok=0
for f in person_follow_robot/share/person_follow_robot/config/identity_lock_kingsen.yaml \
         person_follow_nav/share/person_follow_nav/config/follow_nav.yaml \
         person_follow_nav/share/person_follow_nav/config/rssi.yaml \
         person_follow_nav/share/person_follow_nav/config/rssi_template.json \
         person_follow_nav/share/person_follow_nav/launch/follow_nav_real.launch.py \
         person_follow_nav/share/person_follow_nav/launch/rssi.launch.py; do
  p="$WS/install/$f"
  if [ ! -e "$p" ]; then bad "thieu install/$f — build lai"; continue; fi
  case "$(readlink -f "$p")" in
    "$WS/src/"*) nok=$((nok + 1)) ;;
    *) bad "install/$f la BAN CHEP cu — colcon build --symlink-install" ;;
  esac
done
[ $nok -gt 0 ] && ok "$nok file cau hinh/launch tro ve src"

echo
echo "=============================================================="
if [ $fail -ne 0 ]; then
  echo "${R}${B} CO MUC HONG — sua truoc khi chay.${X}"
  if [ $CHECK_ONLY = 1 ]; then exit 1; fi
  if [ $FORCE = 0 ]; then echo " (muon chay bat chap: them --force)"; exit 1; fi
  echo "${Y} --force: van chay.${X}"
else
  echo "${G}${B} KIEM TRA DAT.${X}"
fi
[ $CHECK_ONLY = 1 ] && exit 0

mkdir -p "$WS/run_logs"
LOG="$WS/run_logs/run_$(date +%m%d_%H%M%S).log"
ARGS=(lidar_port:="$PL" robot_port:="$PROBOT" camera_source:="$PCAM")
[ $RSSI = 1 ] && ARGS+=(start_rssi:=true ports:="$PA,$PB,$PC")
S=src/person_follow_nav/scripts
cat <<EOF

${B}Terminal khac (cd $WS && source install/setup.bash):${X}
  T2 theo doi  : python3 $S/watch_follow.py --geom      (sau ~20 s: bash $S/preflight.sh)
  T3 ghi bag   : bash $S/run_full.sh --bag
  T4 dieu khien: ros2 service call /person_reid/start_enroll std_srvs/srv/Trigger {}
                 ros2 service call /follow/enable           std_srvs/srv/Trigger {}
                 ros2 service call /follow/stop             std_srvs/srv/Trigger {}   # DUNG KHAN
Log launch: $LOG

ros2 launch person_follow_nav follow_nav_real.launch.py ${ARGS[*]} ${EXTRA[*]}
EOF
set -o pipefail
# tee bo qua Ctrl-C: launch tat het cac node (va in log luc tat) roi tee moi ket thuc
ros2 launch person_follow_nav follow_nav_real.launch.py "${ARGS[@]}" "${EXTRA[@]}" 2>&1 | (trap '' INT; tee "$LOG")
