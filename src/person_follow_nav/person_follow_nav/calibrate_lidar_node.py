"""
calibrate_lidar_node.py
=======================
Cong cu HIEU CHINH LIDAR — chay truoc khi dung he thong.

Node nay lam BA viec:

  1. VE BAN DO 360 do quanh xe, de ban NHIN THAY lidar dang thay gi.
  2. PHAT HIEN TIA DAP VAO THAN XE (self-return). LiDAR quet 360 do nen no thay
     ca cot do, day dien, mep san xe cua chinh no. Neu khong loc, cac diem nay
     nam BEN TRONG footprint => rect_clearance() = 0 => moi quy dao bi loai
     => xe KET VINH VIEN o trang thai BLOCKED, khong bao gio nhuc nhich.
  3. TINH lidar_yaw_offset_deg tu vat can ban dat truoc mui xe.

HAI CHE DO
----------
  A. Do than xe (chay TRUOC, chi 1 lan):
       Don SACH quanh xe, khong de vat gi trong ban kinh 2m.
       ros2 run person_follow_nav calibrate_lidar --ros-args -p mode:=self_scan
     Ket qua: danh sach blind_sectors_deg de chep vao config.

  B. Hieu chinh huong (chay SAU):
       Dat vat can hep (thung carton, chai nuoc) NGAY TRUOC MUI XE, cach 0.6-1.0m.
       ros2 run person_follow_nav calibrate_lidar
     Ket qua: lidar_yaw_offset_deg.

KIEM TRA CUOI
-------------
  Doi vat can sang BEN TRAI xe, chay lai che do B.
  Voi tham so dung, phai ra goc khoang +90 do (KHONG phai -90).
  Neu ra -90 do  =>  lidar lap nguoc  =>  dat lidar_angle_sign: -1.0
"""

from __future__ import annotations

import math
import os
import warnings
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class CalibrateLidarNode(Node):

    # ROS 2 phan biet nghiem ngat INTEGER voi DOUBLE. Neu khai bao mac dinh 0.0
    # (DOUBLE) ma nguoi dung go `-p expect_deg:=90` thi ROS coi 90 la INTEGER
    # va NEM LOI InvalidParameterTypeException. Rat de dinh khi go lenh tay.
    # dynamic_typing=True cho phep ca hai, code doc ra van ep bang float().
    _DYN = ParameterDescriptor(dynamic_typing=True)

    def _p(self, name, default):
        self.declare_parameter(name, default, self._DYN)

    def __init__(self) -> None:
        super().__init__("calibrate_lidar_node")

        self._p("mode", "heading")          # "heading" | "self_scan"
        self._p("scan_topic", "/scan")
        self._p("samples", 60)
        # Bo qua tia gan hon nguong nay khi tim vat can (loai self-return)
        # Nguong DU PHONG khi chua co ho so than xe. Phai nho hon moi vat can
        # test ban se dat. Than xe cua ban o 0.128m nen 0.22 la du.
        self._p("self_return_max_m", 0.22)
        # Huong ban DAT vat can, tinh trong base_link (do).
        #   0   = truoc mui xe   (mac dinh, dung de tinh offset)
        #   90  = ben trai xe    (dung cho buoc kiem tra cuoi)
        #  -90  = ben phai xe
        self._p("expect_deg", 0.0)
        # Ho so than xe do o che do A
        self._p("self_profile_path", "~/.ros/lidar_self_profile.npz")
        # Chi tim vat can hieu chinh trong khoang nay
        # Vat can test phai xa hon nguong nay. Than xe cua ban o 0.128m nen
        # 0.25 la du de tach. DUNG dat cao hon — vat test co the o 0.30-0.35m.
        self._p("target_min_m", 0.25)
        self._p("target_max_m", 1.60)
        # Tham so dang dung, de doi chieu
        self._p("current_yaw_offset_deg", -90.0)
        self._p("current_angle_sign", 1.0)
        # Kich thuoc xe, de canh bao diem nam trong footprint
        self._p("front_len", 0.30)
        self._p("rear_len", 0.30)
        self._p("half_width", 0.24)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.mode = str(g("mode")).lower()
        self.scan_topic = str(g("scan_topic"))
        self.n_samples = max(5, int(g("samples")))
        self.self_max = float(g("self_return_max_m"))
        self.expect_deg = float(g("expect_deg"))
        self.profile_path = os.path.expanduser(str(g("self_profile_path")))
        self.tgt_min = float(g("target_min_m"))
        self.tgt_max = float(g("target_max_m"))
        self.cur_off = float(g("current_yaw_offset_deg"))
        self.cur_sign = float(g("current_angle_sign"))
        self.front_len = float(g("front_len"))
        self.rear_len = float(g("rear_len"))
        self.half_width = float(g("half_width"))

        self.nbin = 72                       # o 5 do
        self.acc: List[np.ndarray] = []
        self.count = 0
        self.scan_info: Optional[str] = None

        self.create_subscription(LaserScan, self.scan_topic, self._cb, qos_profile_sensor_data)

        print()
        print("=" * 74)
        if self.mode == "self_scan":
            print("  CHE DO A — DO THAN XE")
            print("  Hay DON SACH quanh xe, khong de vat gi trong ban kinh 2m.")
        else:
            print("  CHE DO B — HIEU CHINH HUONG")
            print("  Dat vat can NGAY TRUOC MUI XE, cach 0.6-1.0m, xung quanh trong.")
        print(f"  Dang thu thap {self.n_samples} vong quet tu {self.scan_topic} ...")
        print("=" * 74)

    # ─────────────────────────────────────────────────────────────────────

    def _cb(self, msg: LaserScan) -> None:
        r = np.asarray(msg.ranges, dtype=np.float64)
        n = r.shape[0]
        if n == 0:
            return
        a = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment

        if self.scan_info is None:
            self.scan_info = (
                f"angle_min={msg.angle_min:.4f} rad  angle_max={msg.angle_max:.4f} rad\n"
                f"  so tia={n}  angle_increment={msg.angle_increment:.5f} rad "
                f"({math.degrees(msg.angle_increment):.2f} do)\n"
                f"  range_min={msg.range_min:.3f}m  range_max={msg.range_max:.3f}m"
            )

        valid = np.isfinite(r) & (r > max(msg.range_min, 0.02)) & (r < msg.range_max)
        binned = np.full(self.nbin, np.nan)
        if np.any(valid):
            deg = np.degrees(a[valid]) % 360.0
            idx = np.clip((deg / (360.0 / self.nbin)).astype(np.int64), 0, self.nbin - 1)
            rv = r[valid]
            order = np.lexsort((rv, idx))
            i_s = idx[order]
            first = np.ones(i_s.shape[0], dtype=bool)
            first[1:] = i_s[1:] != i_s[:-1]
            binned[i_s[first]] = rv[order][first]

        self.acc.append(binned)
        self.count += 1
        if self.count >= self.n_samples:
            self._report()
            rclpy.shutdown()

    # ─────────────────────────────────────────────────────────────────────

    def _to_base_deg(self, lidar_deg: float) -> float:
        rad = self.cur_sign * math.radians(lidar_deg) + math.radians(self.cur_off)
        return math.degrees(math.atan2(math.sin(rad), math.cos(rad)))

    def _inside_footprint(self, base_deg: float, rng: float) -> bool:
        x = rng * math.cos(math.radians(base_deg))
        y = rng * math.sin(math.radians(base_deg))
        return (-self.rear_len <= x <= self.front_len) and (abs(y) <= self.half_width)

    @staticmethod
    def _merge_sectors(bins: List[int], step: float, med: np.ndarray, nbin: int
                       ) -> List[Tuple[float, float, float]]:
        """Gom o lien tuc thanh cung goc. Tra ve [(lo_deg, hi_deg, min_range)]."""
        if not bins:
            return []
        s = sorted(bins)
        groups: List[List[int]] = []
        cur = [s[0]]
        for b in s[1:]:
            if b == cur[-1] + 1:
                cur.append(b)
            else:
                groups.append(cur)
                cur = [b]
        groups.append(cur)
        # noi nhom dau va cuoi neu vong qua moc 0/360
        if len(groups) > 1 and groups[0][0] == 0 and groups[-1][-1] == nbin - 1:
            groups[0] = groups[-1] + groups[0]
            groups.pop()
        out = []
        for gp in groups:
            lo = gp[0] * step if gp[0] <= gp[-1] else gp[0] * step
            hi = (gp[-1] + 1) * step
            dmin = float(np.nanmin(med[np.array(gp)]))
            out.append((lo, hi, dmin))
        return out

    def _save_profile(self, med: np.ndarray) -> None:
        try:
            os.makedirs(os.path.dirname(self.profile_path), exist_ok=True)
            np.savez(self.profile_path, med=med, nbin=self.nbin)
            print()
            print(f"  Da luu ho so than xe vao: {self.profile_path}")
            print("  Che do B se tu nap file nay de phan biet than xe voi vat can test.")
        except Exception as exc:
            print(f"  (Khong luu duoc ho so: {exc})")

    def _load_profile(self) -> Optional[np.ndarray]:
        if self.mode == "self_scan":
            return None
        try:
            d = np.load(self.profile_path)
            if int(d["nbin"]) != self.nbin:
                return None
            return np.asarray(d["med"], dtype=np.float64)
        except Exception:
            return None

    def _report(self) -> None:
        A = np.vstack(self.acc)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            med = np.nanmedian(A, axis=0)
        seen = np.sum(~np.isnan(A), axis=0) / float(A.shape[0])
        step = 360.0 / self.nbin

        print()
        print("=" * 74)
        print("  THONG TIN LASERSCAN")
        print("=" * 74)
        print("  " + (self.scan_info or "?"))

        print()
        print("=" * 74)
        print("  BAN DO QUANH XE  (median cua %d vong quet)" % A.shape[0])
        print("=" * 74)
        print(f"  {'goc lidar':>11} {'goc base':>9} {'k.cach':>8} {'on dinh':>8}  do thi (1 o = 10cm)")
        print("  " + "-" * 70)
        for i in range(self.nbin):
            if np.isnan(med[i]):
                continue
            ld = i * step
            bd = self._to_base_deg(ld + step / 2.0)
            d = float(med[i])
            bar_n = min(40, max(1, int(d / 0.1)))
            flag = ""
            if d < self.self_max and seen[i] > 0.8:
                flag = "  <== GAN, nghi la THAN XE"
                if self._inside_footprint(bd, d):
                    flag = "  <== TRONG FOOTPRINT, PHAI LOC"
            print(f"  {ld:6.0f}-{ld+step:4.0f} {bd:+8.0f} {d:7.3f}m {seen[i]*100:6.0f}%  "
                  f"{'#' * bar_n}{flag}")

        # Nhan dien than xe: uu tien HO SO do o che do A.
        # Nguong cung khong dung duoc, vi vat can test cua ban co the gan hon
        # nguong do (vd tam phang ben trai o 0.343m) va se bi nham la than xe.
        prof = self._load_profile()
        if prof is not None:
            self_bins = [
                i for i in range(self.nbin)
                if (not np.isnan(med[i])) and (not np.isnan(prof[i]))
                and prof[i] < 0.60 and abs(med[i] - prof[i]) < 0.06
            ]
            self.profile_used = True
        else:
            self_bins = [
                i for i in range(self.nbin)
                if (not np.isnan(med[i])) and med[i] < self.self_max and seen[i] > 0.8
            ]
            self.profile_used = False
        sectors = self._merge_sectors(self_bins, step, med, self.nbin)

        print()
        print("=" * 74)
        print("  TIA DAP VAO THAN XE")
        print("=" * 74)
        if self.mode != "self_scan":
            if getattr(self, "profile_used", False):
                print("  Dang dung ho so than xe do o che do A.")
                if sectors:
                    print("  Cac cung goc khop ho so (da bo qua khi hieu chinh):")
                    for lo, hi, dmin in sectors:
                        print(f"    lidar {lo:6.1f} .. {hi:6.1f} do   gan nhat {dmin:.3f}m")
                else:
                    print("  Khong cung goc nao khop ho so.")
            else:
                print("  CHUA CO HO SO THAN XE. Dang dung nguong tam %.2fm." % self.self_max)
                print("  Hay chay che do A truoc de ket qua chinh xac:")
                print("    ros2 run person_follow_nav calibrate_lidar "
                      "--ros-args -p mode:=self_scan")
            print()
            print("  LUU Y: KHONG chep blind_sectors_deg tu che do B.")
            print("  O che do B co vat can test trong phong, script khong the phan biet")
            print("  duoc dau la than xe dau la vat test. Chi che do A moi cho so dung.")
        elif not sectors:
            print("  Khong co tia nao gan hon %.2fm mot cach on dinh." % self.self_max)
            print("  Lidar khong bi than xe che. Tot.")
        else:
            print("  Cac cung goc luon co vat rat gan — gan nhu chac chan la than xe:")
            print()
            worst_inside = False
            for lo, hi, dmin in sectors:
                mid = (lo + hi) / 2.0
                bd_lo, bd_hi = self._to_base_deg(lo), self._to_base_deg(hi)
                inside = self._inside_footprint(self._to_base_deg(mid), dmin)
                worst_inside |= inside
                tag = "   [NAM TRONG FOOTPRINT]" if inside else ""
                print(f"    lidar {lo:6.1f} .. {hi:6.1f} do   "
                      f"(base {bd_lo:+6.1f} .. {bd_hi:+6.1f} do)   "
                      f"gan nhat {dmin:.3f}m{tag}")
            print()
            if worst_inside:
                print("  !!! CANH BAO NGHIEM TRONG !!!")
                print("  Co diem nam BEN TRONG footprint cua xe.")
                print("  Neu khong loc, follow_planner coi do la va cham, loai bo MOI quy dao,")
                print("  va xe KET VINH VIEN o trang thai BLOCKED, khong bao gio nhuc nhich.")
                print()
            print("  >>> CHEP VAO config/follow_nav.yaml (CA HAI node):")
            print()
            print("        self_filter_enabled: true")
            parts = []
            for lo, hi, _ in sectors:
                parts.append(f"{max(0.0, lo - 2.0):.1f}")
                parts.append(f"{min(360.0, hi + 2.0):.1f}")
            print("        blind_sectors_deg: [" + ", ".join(parts) + "]")
            print()
            print("  (Da noi rong moi ben 2 do cho an toan. Goc trong KHUNG LIDAR.)")
            print("  Dinh dang PHANG theo cap [lo1, hi1, lo2, hi2, ...] — ROS 2 khong")
            print("  nhan mang long nhau lam tham so.")
            print("  Ngoai ra self_filter_enabled con tu loai moi diem roi vao trong")
            print("  footprint, nen ke ca lap them phu kien sau nay van an toan.")

        if self.mode == "self_scan":
            self._save_profile(med)
            print()
            print("=" * 74)
            print("  XONG CHE DO A.")
            print("  Buoc tiep: dat vat can truoc mui xe, chay lai KHONG co mode:=self_scan")
            print("=" * 74)
            print()
            return

        mask = np.array([
            (not np.isnan(med[i])) and (self.tgt_min <= med[i] <= self.tgt_max)
            and (i not in self_bins)
            for i in range(self.nbin)
        ])

        print()
        print("=" * 74)
        print("  HIEU CHINH HUONG")
        print("=" * 74)
        if not np.any(mask):
            print(f"  Khong tim thay vat can nao trong khoang "
                  f"{self.tgt_min:.2f}-{self.tgt_max:.2f}m")
            print("  (da bo qua cac cung goc than xe o tren).")
            print()
            print("  Kiem tra:")
            print("   - Da dat vat can truoc mui xe chua?")
            print("   - Vat can co CAO NGANG TAM QUET lidar khong? Lidar quet o mot do")
            print("     cao co dinh; thung thap hon lidar thi lidar khong thay gi ca.")
            print(f"   - Vat can co nam trong {self.tgt_min:.2f}-{self.tgt_max:.2f}m khong?")
            print("   - Neu vat can that su o gan hon, tang tham so:")
            print("       -p target_min_m:=0.25 -p self_return_max_m:=0.20")
            print("=" * 74)
            print()
            return

        cand = np.where(mask)[0]
        k = int(cand[int(np.argmin(med[cand]))])
        dist = float(med[k])

        # Chi gom cac o RAT GAN muc nho nhat, trong cua so hep +-10 do.
        #
        # Tai sao khong gom ca cung goc rong: voi VAT PHANG (tuong, tam bia),
        # khoang cach tang theo d_min/cos(theta). Cung goc trai ra rat rong ve
        # mot phia, va trong tam cua no lech khoi phuong vuong goc that su.
        # Vi du that: tam phang ben trai xe trai tu base +72 den +132 do,
        # trong tam cho ra +105 do trong khi dap an dung la +90 do.
        # Huong co khoang cach NHO NHAT moi la phuong vuong goc.
        tol = max(0.02, 0.03 * dist)
        win = int(round(10.0 / step))
        grp = [k]
        for d in (1, -1):
            for m in range(1, win + 1):
                j = (k + d * m) % self.nbin
                if mask[j] and (med[j] - dist) < tol:
                    grp.append(j)
                else:
                    break
        angs = np.radians([(i * step + step / 2.0) for i in grp])
        ld_center = math.degrees(math.atan2(
            float(np.mean(np.sin(angs))), float(np.mean(np.cos(angs)))
        )) % 360.0
        width_deg = len(grp) * step

        cur_base = self._to_base_deg(ld_center)
        err = math.degrees(math.atan2(
            math.sin(math.radians(cur_base - self.expect_deg)),
            math.cos(math.radians(cur_base - self.expect_deg)),
        ))

        where = {0.0: "TRUOC MUI XE", 90.0: "BEN TRAI XE",
                 -90.0: "BEN PHAI XE", 180.0: "PHIA SAU XE"}.get(
                     self.expect_deg, f"huong {self.expect_deg:+.0f} do")

        if dist < self.tgt_min * 1.15:
            print(f"  [!] Vat can ({dist:.3f}m) rat sat nguong target_min_m="
                  f"{self.tgt_min:.2f}m.")
            print(f"      Neu nghi ket qua sai, ha nguong: -p target_min_m:=0.20")
            print()
        print(f"  Vat can tim thay:       {dist:.3f} m")
        print(f"  Goc trong khung lidar:  {ld_center:.1f} do   "
              f"(rong ~{width_deg:.0f} do, {len(grp)} o)")
        print(f"  Goc trong base_link:    {cur_base:+.1f} do   "
              f"(voi offset={self.cur_off:.1f}, sign={self.cur_sign:.1f})")
        print()
        print(f"  Ban khai bao da dat vat can o: {where}  (expect_deg={self.expect_deg:+.0f})")
        print(f"  Sai lech: {err:+.1f} do")
        print()

        if abs(err) < 10.0:
            print("  ========================  DUNG  ========================")
            print("  Tham so hien tai chinh xac. KHONG can doi gi.")
            print()
            print(f"        lidar_yaw_offset_deg: {self.cur_off:.1f}")
            print(f"        lidar_angle_sign: {self.cur_sign:.1f}")
        elif abs(abs(err) - 180.0) < 20.0:
            print("  ========================  SAI DAU  =====================")
            print("  Goc do duoc nguoc 180 do. Lidar lap NGUOC (up mat quet xuong).")
            print()
            print(f"        lidar_angle_sign: {-self.cur_sign:.1f}")
            print(f"        lidar_yaw_offset_deg: {-ld_center * -self.cur_sign:.1f}")
            print()
            print("  Sau khi doi, chay lai ca hai buoc kiem tra.")
        else:
            new_off = self.cur_off - err
            new_off = math.degrees(math.atan2(
                math.sin(math.radians(new_off)), math.cos(math.radians(new_off))))
            print("  ========================  LECH  ========================")
            print(f"  Lech {err:+.1f} do so voi huong ban khai bao.")
            print()
            print(f"        lidar_yaw_offset_deg: {new_off:.1f}")
            print(f"        lidar_angle_sign: {self.cur_sign:.1f}")
            print()
            print("  Neu ban CHAC vat can dat dung huong, chep so tren vao config.")
            print("  Neu khong chac, dat lai vat can cho that dung roi chay lai.")
        print()
        print("  " + "-" * 70)
        print("  QUY TRINH DAY DU:")
        print("    1. Don sach quanh xe:")
        print("       ros2 run person_follow_nav calibrate_lidar "
              "--ros-args -p mode:=self_scan")
        print("    2. Vat can TRUOC MUI XE:")
        print("       ros2 run person_follow_nav calibrate_lidar")
        print("    3. Vat can BEN TRAI XE:")
        print("       ros2 run person_follow_nav calibrate_lidar "
              "--ros-args -p expect_deg:=90")
        print("    4. Vat can BEN PHAI XE:")
        print("       ros2 run person_follow_nav calibrate_lidar "
              "--ros-args -p expect_deg:=-90")
        print("    Ca 3 buoc 2-4 deu bao DUNG thi hieu chinh hoan tat.")
        print("=" * 74)
        print()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrateLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
