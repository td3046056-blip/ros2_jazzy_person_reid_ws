#!/usr/bin/env bash
# =============================================================================
# preflight.sh — kiem tra truoc khi cho xe chay tren san (giai doan 3)
# Chay khi test_avoid_only.launch.py DANG chay o terminal khac.
# =============================================================================
G=$'\033[92m'; Y=$'\033[93m'; R=$'\033[91m'; B=$'\033[1m'; X=$'\033[0m'
fail=0
ok()   { echo "  ${G}OK${X}    $1"; }
warn() { echo "  ${Y}CANH BAO${X}  $1"; }
bad()  { echo "  ${R}HONG${X}  $1"; fail=1; }

echo "=============================================================="
echo "${B} KIEM TRA TRUOC KHI CHAY (giai doan 3)${X}"
echo "=============================================================="

echo
echo "${B}1. Topic${X}"
for t in /scan /odom; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then ok "$t ton tai"
  else bad "$t KHONG co"; fi
done

echo
echo "${B}2. Tan so${X}"
chk_hz () {
  local topic=$1 minhz=$2
  local out
  out=$(timeout 6 ros2 topic hz "$topic" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
  if [ -z "$out" ]; then bad "$topic khong co du lieu"; return; fi
  awk -v h="$out" -v m="$minhz" -v t="$topic" -v g="$G" -v r="$R" -v x="$X" \
    'BEGIN{ if (h+0 >= m+0) printf "  %sOK%s    %s = %.1f Hz (can >= %s)\n", g,x,t,h,m;
            else            printf "  %sHONG%s  %s = %.1f Hz (can >= %s)\n", r,x,t,h,m }'
  awk -v h="$out" -v m="$minhz" 'BEGIN{exit (h+0>=m+0)?0:1}' || fail=1
}
chk_hz /scan 8
chk_hz /odom 15

echo
echo "${B}3. Ai dang ghi /cmd_vel${X}"
n=$(ros2 topic info /cmd_vel 2>/dev/null | grep -oP 'Publisher count: \K\d+')
n=${n:-0}
if   [ "$n" -eq 0 ]; then bad "khong ai ghi /cmd_vel — planner chua chay?"
elif [ "$n" -eq 1 ]; then ok "dung 1 nguon ghi /cmd_vel"
else bad "$n nguon cung ghi /cmd_vel — xe se GIAT. Tat bot node."
     ros2 topic info /cmd_vel --verbose 2>/dev/null | grep -A1 "Node name" | sed 's/^/        /'
fi

echo
echo "${B}4. Trang thai planner${X}"
st=$(timeout 4 ros2 topic echo /follow/planner_status --once 2>/dev/null)
if [ -z "$st" ]; then
  bad "khong co /follow/planner_status — follow_planner chua chay"
else
  nobs=$(echo "$st" | grep -oP '"n_obstacles":\s*\K\d+')
  state=$(echo "$st" | grep -oP '"state":\s*"\K[A-Z_]+')
  fc=$(echo "$st" | grep -oP '"front_clearance_m":\s*\K[0-9.]+')
  ok "state=$state  n_obstacles=${nobs:-?}  thoang truoc=${fc:-?} m"
  if [ "${nobs:-0}" -eq 0 ]; then
    warn "n_obstacles=0 — lidar khong thay gi, hoac bo loc bo het. Kiem tra lai."
  fi
fi

echo
echo "${B}5. Bo loc than xe${X}"
echo "  Xem log cua follow_planner, phai co dong nhu:"
echo "    self-filter: bo N/360 tia dap vao than xe"
echo "  Voi xe nay N nen vao khoang ${B}45-55${X}."
echo "    N = 0      -> bo loc KHONG an, xe se ket BLOCKED ngay"
echo "    N > 150    -> co gi do dang chan lidar"

echo
echo "${B}6. Cong tac dung${X}"
if ros2 service list 2>/dev/null | grep -qx "/follow/stop"; then
  ok "/follow/stop san sang"
  echo "        ros2 service call /follow/stop std_srvs/srv/Trigger {}"
else
  bad "/follow/stop KHONG co"
fi

echo
echo "=============================================================="
if [ "$fail" -eq 0 ]; then
  echo " ${G}${B}SAN SANG.${X} Nhung lan chay dau van KE XE LEN HOP."
else
  echo " ${R}${B}CHUA SAN SANG.${X} Sua cac muc HONG o tren truoc."
fi
echo "=============================================================="
