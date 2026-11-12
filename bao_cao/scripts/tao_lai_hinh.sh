#!/usr/bin/env bash
# Tao lai TOAN BO hinh trong bao_cao/hinh tu code + du lieu (thu tu phu thuoc).
#   bash bao_cao/scripts/tao_lai_hinh.sh            # dung JSON mo phong da co trong bao_cao/du_lieu
#   bash bao_cao/scripts/tao_lai_hinh.sh --mo-phong # chay lai ca mo phong cac phien ban cu (~10 phut, JOBS=12)
set -euo pipefail
cd "$(dirname "$0")"
export OPENBLAS_NUM_THREADS=1

# 1. Bag chay that -> pickle (can ROS). Bag nam ngoai repo (~/bags), pickle o ~/bao_cao_cache.
mkdir -p ~/bao_cao_cache
for b in follow_1107_104423 follow_1107_112843; do
  if [ ! -f ~/bao_cao_cache/$b.pkl ]; then
    if [ -d ~/bags/$b ]; then
      ( source /opt/ros/jazzy/setup.bash && python3 ../../src/person_follow_nav/scripts/dump_bag.py ~/bags/$b ~/bao_cao_cache/$b.pkl )
    else
      echo "CANH BAO: khong co ~/bags/$b -> bo qua hinh xe that / loc than xe"
    fi
  fi
done

# 2. Mo phong cac phien ban (git archive ban cu -> ~/bao_cao_cache/pkg_*)
if [ "${1:-}" = "--mo-phong" ]; then
  python3 mo_phong_hinh.py --lai
fi

# 3. Hinh
python3 hinh_kien_truc.py
python3 hinh_he_thong.py
python3 hinh_thuat_toan.py 2>&1 | grep -v '^\['
python3 hinh_rssi.py
python3 hinh_tong_hop.py
python3 hinh_mo_phong.py
if [ -f ~/bao_cao_cache/follow_1107_104423.pkl ]; then python3 hinh_xe_that.py; fi
echo "Xong: $(ls ../hinh/*.png | wc -l) hinh trong bao_cao/hinh"
