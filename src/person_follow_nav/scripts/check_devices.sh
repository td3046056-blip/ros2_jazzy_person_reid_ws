#!/usr/bin/env bash
# check_devices.sh — chan doan nhanh cong USB truoc khi chay robot
echo "=============================================================="
echo " 1. Cac cong serial dang co"
echo "=============================================================="
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  KHONG CO cong nao. Kiem tra day USB va nguon thiet bi."

echo
echo "=============================================================="
echo " 2. Symlink co dinh (do udev rules tao ra)"
echo "=============================================================="
ls -l /dev/robot_* 2>/dev/null || echo "  KHONG CO symlink. Chay: python3 setup_udev.py"

echo
echo "=============================================================="
echo " 3. Symlink san co cua he thong (khong can udev rules)"
echo "=============================================================="
echo "--- by-id (theo nhan thiet bi) ---"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "  (trong)"
echo "--- by-path (theo cong vat ly - LUON dung duoc) ---"
ls -l /dev/serial/by-path/ 2>/dev/null || echo "  (trong)"

echo
echo "=============================================================="
echo " 4. Nhan dang tung cong"
echo "=============================================================="
for p in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$p" ] || continue
  echo "--- $p ---"
  udevadm info -q property -n "$p" | grep -E "^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_VENDOR=|ID_MODEL=|ID_PATH=)" | sed 's/^/    /'
done

echo
echo "=============================================================="
echo " 5. Quyen truy cap"
echo "=============================================================="
echo "  User hien tai: $(whoami)"
echo "  Cac nhom:      $(groups)"
if groups | grep -qw dialout; then
  echo "  -> DA o trong nhom dialout. OK."
else
  echo "  -> CHUA o trong nhom dialout. Chay:"
  echo "     sudo usermod -aG dialout $(whoami)"
  echo "     (phai DANG XUAT va dang nhap lai moi co hieu luc)"
fi

echo
echo "=============================================================="
echo " 6. Kernel co nhan thiet bi khong (20 dong cuoi)"
echo "=============================================================="
sudo dmesg 2>/dev/null | grep -iE "usb|ch341|ftdi|cp210|tty" | tail -20 || \
  echo "  Can sudo: sudo dmesg | grep -iE 'usb|ch341|ftdi' | tail -20"
