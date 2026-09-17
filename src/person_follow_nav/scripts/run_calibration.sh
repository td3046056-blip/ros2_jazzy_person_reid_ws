#!/usr/bin/env bash
# =============================================================================
# run_calibration.sh — chay toan bo 4 buoc hieu chinh LiDAR, co nhac tung buoc.
#
#   bash run_calibration.sh
#
# Truoc khi chay: LiDAR phai dang chay o terminal khac.
#   ros2 launch sc_mini sc_mini.launch.py \
#     port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
#
# Ket qua moi buoc duoc luu vao thu muc calib_logs/ de doi chieu ve sau.
# =============================================================================
set -u

OUT="calib_logs"
mkdir -p "$OUT"
STAMP=$(date +%Y%m%d_%H%M%S)

B=$'\033[1m'; G=$'\033[92m'; Y=$'\033[93m'; R=$'\033[91m'; X=$'\033[0m'

line() { printf '%s\n' "======================================================================"; }

# --- kiem tra /scan co du lieu khong -----------------------------------------
line
echo "${B}KIEM TRA LIDAR${X}"
line
echo "Dang doi du lieu tren /scan (toi da 10 giay)..."
if timeout 10 ros2 topic echo /scan --once > /dev/null 2>&1; then
  echo "${G}OK — /scan co du lieu.${X}"
else
  echo "${R}KHONG co du lieu tren /scan.${X}"
  echo
  echo "Mo terminal khac va chay:"
  echo "  ros2 launch sc_mini sc_mini.launch.py \\"
  echo "    port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
  echo
  echo "Neu bao khong mo duoc cong, kiem tra:  ls -l /dev/serial/by-id/"
  exit 1
fi

run_step () {
  local title="$1"; shift
  local prep="$1";  shift
  local file="$1";  shift

  echo
  line
  echo "${B}${title}${X}"
  line
  echo "${Y}${prep}${X}"
  echo
  read -r -p "Nhan Enter khi da san sang (hoac 's' de bo qua buoc nay): " ans
  if [ "$ans" = "s" ] || [ "$ans" = "S" ]; then
    echo "Bo qua."
    return 1
  fi
  local path="$OUT/${STAMP}_${file}.txt"
  "$@" 2>&1 | tee "$path"
  echo
  echo "${G}Da luu: $path${X}"
  return 0
}

run_step \
  "BUOC 1 / 4 — DO THAN XE" \
  "Don SACH quanh xe. Khong de vat gi trong ban kinh 2m.
Buoc nay luu ho so than xe vao ~/.ros/lidar_self_profile.npz
va in ra dong blind_sectors_deg de chep vao config." \
  "1_self_scan" \
  ros2 run person_follow_nav calibrate_lidar --ros-args -p mode:=self_scan

run_step \
  "BUOC 2 / 4 — VAT CAN TRUOC MUI XE" \
  "Dat vat can NGAY TRUOC MUI XE, cach 0.6 - 1.0 m.
Vat phai CAO NGANG TAM QUET lidar — thung thap hon lidar thi lidar khong thay.
Don trong hai ben." \
  "2_truoc_xe" \
  ros2 run person_follow_nav calibrate_lidar

run_step \
  "BUOC 3 / 4 — VAT CAN BEN TRAI XE" \
  "Chuyen vat can sang BEN TRAI xe (ngang hong trai), cach 0.3 - 1.0 m." \
  "3_ben_trai" \
  ros2 run person_follow_nav calibrate_lidar --ros-args -p expect_deg:=90

run_step \
  "BUOC 4 / 4 — VAT CAN BEN PHAI XE" \
  "Chuyen vat can sang BEN PHAI xe (ngang hong phai), cach 0.3 - 1.0 m.
Buoc nay xac nhan CHIEU QUAY. Neu ra +90 thay vi -90 thi lidar lap nguoc." \
  "4_ben_phai" \
  ros2 run person_follow_nav calibrate_lidar --ros-args -p expect_deg:=-90

echo
line
echo "${B}XONG${X}"
line
echo "Log da luu trong: $(pwd)/$OUT/"
ls -1 "$OUT" | tail -4 | sed 's/^/  /'
echo
echo "${B}Viec can lam tiep:${X}"
echo "  1. Chep blind_sectors_deg tu BUOC 1 (chi buoc 1!) vao config/follow_nav.yaml"
echo "     - dien vao CA HAI node: target_tracker_node va follow_planner_node"
echo "  2. Kiem tra buoc 2, 3, 4 deu bao DUNG"
echo "  3. Neu co buoc nao bao SAI DAU -> dat lidar_angle_sign: -1.0 roi chay lai"
echo
echo "${R}KHONG chep blind_sectors_deg tu buoc 2, 3, 4.${X}"
echo "Luc do co vat test trong phong, script khong phan biet duoc"
echo "dau la than xe dau la vat test."
echo
