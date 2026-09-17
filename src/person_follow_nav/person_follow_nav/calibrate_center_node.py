"""
calibrate_center_node.py
========================
Do TAM QUAY THAT cua xe, va tu do suy ra lidar_x / front_len / rear_len dung.

VAN DE
------
Xe vi sai quay quanh TAM TRUC BANH CHU DONG. Ban do kich thuoc tu truc TRUOC,
nhung banh chu dong la banh SAU — nen odometry lay goc o truc SAU. Hai khung
lech nhau dung bang khoang cach hai truc (L).

Hau qua neu khong sua:
  - Moi diem lidar bi dat sai cho L met
  - front_len / rear_len dao nguoc: dang khai "duoi vang ra" trong khi thuc te
    la "MUI vang ra" khi xoay
  - Ban kinh ngoai tiep sai => rotate_radius sai => xe tuong xoay duoc tai cho
    trong khi mui dap vao tuong

CACH NAY DO TRUC TIEP
---------------------
Cho xe xoay tai cho mot vong. Voi moi gia tri lidar_x thu nghiem, dung ban do
diem trong khung odom. Neu lidar_x DUNG, mot vat can dung yen luon roi vao dung
mot cho => ban do SAC NET. Neu sai, vat can bi tra ra thanh mot vong tron ban
kinh bang dung sai so => ban do NHOE.

Chon gia tri cho ban do sac net nhat. Cach nay:
  - Khong can biet khoang cach hai truc
  - Dung ca khi xe la skid-steer 4 banh chu dong (tam quay o giua, khong o truc)
  - Do dung cai ta can: vi tri lidar so voi TAM QUAY THAT

AN TOAN
-------
Xe se TU XOAY TAI CHO. Don trong ban kinh 1m quanh xe truoc khi chay.
Node kiem tra do thoang truoc, va dung ngay khi Ctrl-C.

CACH DUNG
---------
  ros2 run person_follow_nav calibrate_center

  Tuy chon:
    -p rotate_speed:=0.3        toc do xoay (rad/s)
    -p turns:=1.0               so vong xoay
    -p search_min:=0.0          khoang do lidar_x
    -p search_max:=0.60
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .geometry import self_filter_mask, wrap_pi, yaw_from_quaternion


class CalibrateCenterNode(Node):

    _DYN = ParameterDescriptor(dynamic_typing=True)

    def _p(self, name, default):
        self.declare_parameter(name, default, self._DYN)

    def __init__(self) -> None:
        super().__init__("calibrate_center_node")
        self._p("scan_topic", "/scan")
        self._p("odom_topic", "/odom")
        self._p("cmd_vel_topic", "/cmd_vel")
        self._p("lidar_yaw_offset_deg", -90.0)
        self._p("lidar_angle_sign", 1.0)
        self._p("rotate_speed", 0.30)
        self._p("turns", 1.0)
        self._p("search_min", 0.00)
        self._p("search_max", 0.60)
        self._p("search_step", 0.01)
        self._p("grid_cell_m", 0.02)
        self._p("use_range_min", 0.40)
        self._p("use_range_max", 3.00)
        self._p("required_clearance_m", 0.75)
        # Loc tia dap vao than xe. Bat buoc: neu khong, cum tia 0.128m cua
        # than xe lam kiem tra do thoang LUON that bai, va ban do bi them mot
        # vong tron gia quay theo xe.
        self._p("self_filter_enabled", True)
        self._p("self_filter_margin", 0.03)
        self._p("blind_sectors_deg", [0.0])
        # So do cua BAN, tinh tu TRUC TRUOC
        self._p("measured_front_len", 0.14)
        self._p("measured_rear_len", 0.33)
        self._p("measured_lidar_x", 0.10)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.scan_topic = str(g("scan_topic"))
        self.odom_topic = str(g("odom_topic"))
        self.yaw_off = math.radians(float(g("lidar_yaw_offset_deg")))
        self.sign = float(g("lidar_angle_sign"))
        self.w_cmd = abs(float(g("rotate_speed")))
        self.turns = float(g("turns"))
        self.s_min = float(g("search_min"))
        self.s_max = float(g("search_max"))
        self.s_step = float(g("search_step"))
        self.cell = float(g("grid_cell_m"))
        self.r_min = float(g("use_range_min"))
        self.r_max = float(g("use_range_max"))
        self.need_clear = float(g("required_clearance_m"))
        self.m_front = float(g("measured_front_len"))
        self.m_rear = float(g("measured_rear_len"))
        self.m_lidar = float(g("measured_lidar_x"))
        self.self_filter = bool(g("self_filter_enabled"))
        self.self_margin = float(g("self_filter_margin"))
        bs = list(g("blind_sectors_deg") or [])
        self.blind = [[float(bs[i]), float(bs[i + 1])] for i in range(0, len(bs) - 1, 2)
                      if float(bs[i]) >= 0.0]

        self.scan: Optional[LaserScan] = None
        self.yaw = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.o_start: Optional[Tuple[float, float]] = None
        self.yaw_ok = False
        self.samples: List[Tuple[np.ndarray, np.ndarray, float]] = []
        self.yaw_total = 0.0
        self.yaw_prev: Optional[float] = None
        self.phase = "wait"
        self.t0 = time.time()

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.pub = self.create_publisher(Twist, str(g("cmd_vel_topic")), 10)
        self.create_timer(1.0 / 20.0, self._tick)

        print()
        print("=" * 72)
        print("  DO TAM QUAY THAT CUA XE")
        print("=" * 72)
        print("  !! XE SE TU XOAY TAI CHO !!")
        print(f"  Don trong ban kinh {self.need_clear:.2f}m quanh xe.")
        print("  Nhan Ctrl-C bat ky luc nao de dung.")
        print()
        print("  Can co vat can co dinh quanh xe (tuong, ghe, thung) trong khoang")
        print(f"  {self.r_min:.1f}-{self.r_max:.1f}m de lam moc. Phong trong hoan toan se KHONG do duoc.")
        print("=" * 72)

    # ─────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.ox = float(msg.pose.pose.position.x)
        self.oy = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        y = yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        if self.yaw_prev is not None:
            self.yaw_total += abs(wrap_pi(y - self.yaw_prev))
        self.yaw_prev = y
        self.yaw = y
        self.yaw_ok = True

    def _raw(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=np.float64)
        n = r.shape[0]
        a_l = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.05)) & (r < msg.range_max)
        r, a_l = r[ok], a_l[ok]
        a_b = self.sign * a_l + self.yaw_off
        keep = np.ones(r.shape[0], dtype=bool)
        if self.self_filter and r.size:
            pts = np.stack([self.m_lidar + r * np.cos(a_b), r * np.sin(a_b)], axis=1)
            keep = self_filter_mask(pts, np.degrees(a_l) % 360.0,
                                    self.m_front, self.m_rear, 0.30,
                                    self.self_margin, self.blind)
        return r[keep], a_b[keep]

    def _points(self, msg: LaserScan) -> Tuple[np.ndarray, np.ndarray]:
        """Tra ve (r, goc_trong_base_khi_lidar_x=0) cua cac tia hop le."""
        r, a = self._raw(msg)
        ok = (r > self.r_min) & (r < self.r_max)
        return r[ok], a[ok]

    def _tick(self) -> None:
        if self.scan is None or not self.yaw_ok:
            el = time.time() - self.t0
            if el > 4.0 and int(el * 2) % 6 == 0:
                miss = []
                if self.scan is None:
                    miss.append(self.scan_topic)
                if not self.yaw_ok:
                    miss.append(self.odom_topic)
                print(f"  Dang cho: {', '.join(miss)}  ({el:.0f}s)")
            if el > 20.0:
                print()
                print("  " + "=" * 68)
                if self.scan is None:
                    print(f"  THIEU {self.scan_topic} — lidar chua chay:")
                    print("    ros2 launch sc_mini sc_mini.launch.py \\")
                    print("      port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
                if not self.yaw_ok:
                    print(f"  THIEU {self.odom_topic} — DRIVER XE chua chay.")
                    print("  Node nay can driver de (a) doc odom, (b) gui lenh xoay.")
                    print()
                    print("  Cach don gian nhat — chay lidar + driver cung luc:")
                    print("    ros2 launch person_follow_nav calibrate.launch.py")
                    print()
                    print("  Roi o terminal khac:")
                    print("    ros2 run person_follow_nav calibrate_center")
                print("  " + "=" * 68)
                rclpy.shutdown()
            return

        if self.phase == "wait":
            r, _ = self._raw(self.scan)
            near = float(np.min(r)) if r.size else 9.9
            if near < self.need_clear:
                print(f"  DUNG: co vat can cach {near:.2f}m, can toi thieu "
                      f"{self.need_clear:.2f}m. Don quang roi chay lai.")
                rclpy.shutdown()
                return
            print(f"  Do thoang gan nhat {near:.2f}m — OK. Bat dau xoay...")
            self.phase = "spin"
            self.yaw_total = 0.0
            self.yaw_prev = None
            return

        if self.phase == "spin":
            target = self.turns * 2.0 * math.pi
            if self.yaw_total >= target:
                self.pub.publish(Twist())
                self.phase = "done"
                print(f"  Da xoay {math.degrees(self.yaw_total):.0f} do, "
                      f"thu thap {len(self.samples)} vong quet. Dang tinh...")
                self._solve()
                rclpy.shutdown()
                return
            t = Twist()
            t.angular.z = self.w_cmd
            self.pub.publish(t)
            if self.o_start is None:
                self.o_start = (self.ox, self.oy)
            rr, aa = self._points(self.scan)
            if rr.size > 10:
                # Luu ca vi tri odom, khong gia dinh xe dung yen tai goc.
                # Xoay tai cho ma banh truot khong deu thi odom van troi vai cm;
                # bo qua cho troi do se lam nhoe ban do va lam le ket qua.
                self.samples.append((rr, aa, self.yaw, self.ox, self.oy))
            if len(self.samples) % 40 == 0 and self.samples:
                print(f"    ... {math.degrees(self.yaw_total):5.0f} / "
                      f"{math.degrees(target):.0f} do")

    # ─────────────────────────────────────────────────────────────────

    def _sharpness(self, lidar_x: float) -> int:
        """So o luoi bi chiem. Cang IT = ban do cang sac net = lidar_x cang dung."""
        cells = set()
        inv = 1.0 / self.cell
        for rr, aa, yaw, rx, ry in self.samples:
            bx = lidar_x + rr * np.cos(aa)
            by = rr * np.sin(aa)
            c, s = math.cos(yaw), math.sin(yaw)
            ox = rx + c * bx - s * by
            oy = ry + s * bx + c * by
            ix = np.floor(ox * inv).astype(np.int64)
            iy = np.floor(oy * inv).astype(np.int64)
            cells.update(zip(ix.tolist(), iy.tolist()))
        return len(cells)

    def _solve(self) -> None:
        if len(self.samples) < 20:
            print("  Khong du du lieu. Can vat can co dinh quanh xe.")
            return

        xs = np.arange(self.s_min, self.s_max + 1e-9, self.s_step)
        scores = np.array([self._sharpness(float(x)) for x in xs], dtype=np.float64)
        k = int(np.argmin(scores))
        best = float(xs[k])

        # Tinh khoang cach hai truc tu ket qua
        L = best - self.m_lidar
        front = self.m_front + L
        rear = self.m_rear - L
        rc = max(math.hypot(front, 0.30), math.hypot(abs(rear), 0.30))

        print()
        print("=" * 72)
        print("  KET QUA")
        print("=" * 72)
        print(f"  {'lidar_x thu':>12}  {'do nhoe (o luoi)':>18}")
        print("  " + "-" * 34)
        lo = max(0, k - 6)
        hi = min(len(xs), k + 7)
        smin, smax = scores.min(), scores.max()
        for i in range(lo, hi):
            bar = int(28 * (scores[i] - smin) / max(1.0, smax - smin))
            mark = "  <== SAC NET NHAT" if i == k else ""
            print(f"  {xs[i]:12.2f}  {int(scores[i]):18d}  {'#' * bar}{mark}")
        print()

        # Khop parabol quanh day de lay dinh chinh xac hon muc phan giai luoi
        lo2 = max(0, k - 3)
        hi2 = min(len(xs), k + 4)
        peak = best
        if hi2 - lo2 >= 3:
            try:
                cf = np.polyfit(xs[lo2:hi2], scores[lo2:hi2], 2)
                if cf[0] > 0:
                    pk = -cf[1] / (2.0 * cf[0])
                    if xs[lo2] <= pk <= xs[hi2 - 1]:
                        peak = float(pk)
            except Exception:
                pass

        drift = 0.0
        if self.o_start is not None:
            drift = math.hypot(self.ox - self.o_start[0], self.oy - self.o_start[1])
        print(f"  Odom troi khi xoay: {drift*100:.1f} cm", end="")
        if drift > 0.08:
            print("   [!] LON — banh truot khong deu hoac san tron.")
            print("      Ket qua van dung (da bu bang odom) nhung nen chay lai")
            print("      tren san co ma sat tot de chac chan.")
        else:
            print("   (tot)")
        print(f"  Dinh khop parabol:  {peak:.3f} m  (luoi tho: {best:.2f} m)")
        print()

        conf = (smax - smin) / max(1.0, smin)
        if conf < 0.05:
            print("  [!] Duong cong qua phang — ket qua khong dang tin.")
            print("      Can nhieu vat can co dinh hon quanh xe. Thu lai gan tuong,")
            print("      hoac dat vai thung carton quanh xe roi chay lai.")
            print()

        print(f"  Tam quay that cach lidar {best:.2f} m ve phia sau.")
        print(f"  => Khoang cach hai truc L = {best:.2f} - {self.m_lidar:.2f} = {L:.2f} m")
        print()
        print("  >>> CHEP VAO config/follow_nav.yaml (CA HAI node):")
        print()
        print(f"        lidar_x: {best:.2f}")
        print(f"        front_len: {front:.2f}")
        print(f"        rear_len: {max(0.02, rear):.2f}")
        print(f"        half_width: 0.30")
        print(f"        rotate_radius: {rc + 0.02:.2f}")
        print()
        print(f"  Ban kinh ngoai tiep = {rc:.3f} m, quyet dinh boi "
              f"{'MUI' if front > abs(rear) else 'DUOI'} xe.")
        if rear < 0:
            print("  [!] rear_len am — so do hoac ket qua co van de. Do lai bang thuoc:")
            print("      L = khoang cach tu tam truc TRUOC toi tam truc SAU.")
        print("=" * 72)
        print()

    def destroy_node(self) -> bool:
        try:
            for _ in range(3):
                self.pub.publish(Twist())
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrateCenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n  Da dung xe.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
