#!/usr/bin/env python3
"""
rssi_seek.py — THU "bam theo nguoi bang RSSI": xe tu tim va di toi nguoi deo beacon CHI bang RSSI (khong camera).

Day la script THU, khong phai hanh vi chinh thuc cua xe: no tu ghi /cmd_vel nen CHI chay voi
calibrate.launch.py (khong co planner). KHONG co ne vat can — LiDAR chi de DUNG an toan.

CHU TRINH
---------
  DO    xoay tai cho ~1 vong cho rssi_bearing_node tim huong beacon (RSSI chi cho huong khi xe XOAY)
  QUAY  quay mui ve huong do
  DI    di thang toi da --step m. Dung khi:
          - truoc mui co vat/nguoi trong --stop-gap m            -> TOI
          - co vat sap luot qua sat ben hong (< 0.62 m tu tam)   -> dung som, DO lai (nguoi co the o ngay do)
          - het buoc                                             -> DO lai
        Gap vat sau khi da di > 0.7 m tu cho do huong, hoac vat chi cham MEP duong di -> chua chac la nguoi:
        DO lai ngay tai do de xac nhan.
  TOI   dung cho. Khi vat/nguoi truoc mui di khoi (nguoi sang cho khac roi DUNG YEN) hoac sau --hold-sec
        -> lam lai tu DO

RSSI khong do duoc khoang cach (do 30/09) nen "da toi" = LiDAR thay vat ngay truoc mui theo huong beacon.
Neu do la do dac chu khong phai nguoi thi xe cung dung o do — day la gioi han cua ban thu nay.
Sai so huong moi lan do ~15 do (toi da ~25-30 do, do 01/10) nen xe thuong can 2-3 chu ky moi toi noi.

CAN 3 TERMINAL (deu da source ROS + install/setup.bash cua repo nay)
--------------------------------------------------------------------
  T1: ros2 launch person_follow_nav calibrate.launch.py lidar_port:=$PL
  T2: ros2 launch person_follow_nav rssi.launch.py ports:="$PA,$PB,$PC"
  T3: python3 rssi_seek.py                # them --once de dung han sau lan toi dau tien

AN TOAN
-------
  - Tu choi chay neu co node khac ghi /cmd_vel, hoac thieu /odom, /scan, /rssi/bearing, hoac khong nghe thay beacon.
  - /scan cu qua 0.6 s (LiDAR ngung gui) -> DUNG ngay. Mat beacon -> DUNG va cho.
  - Chi xoay khi quanh tam quay trong > 0.53 m; chi tien khi hanh lang truoc mui (rong 0.90 m) trong.
  - Khong bao gio lui. Toc do tien 0.18 m/s.
  - LiDAR cao 18 cm: KHONG thay mat ban, bac them, day dien; va MU thang phia sau -> don san, nhat la sau duoi xe.
  - MUON DUNG XE: buoc ra dung truoc mui xe (xe dung va cho), roi Ctrl-C o laptop.
    Script chet thi driver tu dung sau 1 s (cmd_timeout).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # goc package
# Doi tia LiDAR + loc than xe bang DUNG ham cua planner/tracker -> script nay nhin vat can y nhu planner
from person_follow_nav.geometry import scan_to_base_points, self_filter_mask  # noqa: E402
from person_follow_nav.rssi_df import wrap  # noqa: E402

G = "\033[92m"
R = "\033[91m"
Y = "\033[93m"
B = "\033[1m"
X = "\033[0m"

# Hinh hoc xe + hieu chinh LiDAR — CLAUDE.md muc 2/3 (= config/follow_nav.yaml), DA CHOT, khong doi o day
FRONT_LEN = 0.14
REAR_LEN = 0.33
HALF_WIDTH = 0.30
LIDAR_X = 0.10
LIDAR_Y = 0.00
LIDAR_YAW_OFFSET_DEG = -90.0
LIDAR_ANGLE_SIGN = 1.0
SELF_FILTER_MARGIN = 0.03
BLIND_SECTORS_DEG = [[246.0, 294.0]]     # khung LiDAR — xe MU thang phia sau
CLEAR_MIN_M = 0.47 + 0.06                # rotate_radius + margin_hard: cung luat voi _can_rotate_in_place() cua planner
MAX_USE_RANGE = 6.0
SIDE_MARGIN = 0.15          # hanh lang tien = nua be ngang + ngan nay moi ben (= margin_soft)
PASS_R = 0.62               # vat vao gan TAM QUAY hon ngan nay khi dang di -> dung som (con > 0.53 nen van xoay duoc)
CONFIRM_MAX_M = 0.7         # gap vat sau khi di qua ngan nay tu cho do huong -> sai so huong co the da dua xe toi
#                             NHAM vat (25 do o 2 m = lech ngang 0.85 m) -> do lai ngay tai do de xac nhan
SCAN_STALE_SEC = 0.6
SCAN_KEEP_SEC = 0.45        # gop cac vong quet trong ngan nay: LiDAR mat ~40% tia moi vong, vat nho de lot 1 vong
CTRL_HZ = 15.0


class Stop(Exception):
    """Phai dung xe. wait = so giay cho phuc hoi (0 = dung han)."""

    def __init__(self, why: str, wait: float) -> None:
        super().__init__(why)
        self.why = why
        self.wait = wait


class Seeker:

    def __init__(self, args, name: str = "rssi_seek") -> None:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self.rclpy = rclpy
        self.Twist = Twist
        self.Trigger = Trigger
        self.a = args
        self.node = Node(name)
        self.yaw = None
        self.x = self.y = 0.0
        self.t_odom = 0.0
        self.scans: deque = deque(maxlen=8)      # (t, x, y, yaw, diem Nx2 trong base_link luc do)
        self.t_scan = 0.0
        self.brg = None
        self.t_brg = 0.0
        self.pub = None
        self.stop_req = False            # Ctrl-C / SIGTERM: chi dat co; spin() moi nem KeyboardInterrupt (xem main)
        self.block_wait = 20.0           # do_scan: cho toi da ngan nay giay cho quanh xe trong lai
        self.state = "KHOI DONG"
        self.t0 = time.time()
        self.t_rec = 0.0
        self.t_pubchk = 0.0
        self.n_pub = 1
        self.log = open(args.log, "w") if args.log else None
        # Mau tho RSSI + odom (cung dinh dang rssi_rotate.py) de phan tich lai tung lan do sau khi chay
        self.raw = open(os.path.splitext(args.log)[0] + ".csv", "w") if args.log else None
        if self.raw:
            self.raw.write("t_pc,kind,id,seq,rssi,yaw,x,y\n")

        self.node.create_subscription(Odometry, "/odom", self._odom, 50)
        self.node.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.node.create_subscription(String, "/rssi/bearing", self._brg, 10)
        self.node.create_subscription(String, "/rssi/raw", self._raw, 50)
        self.reset_cli = self.node.create_client(Trigger, "/rssi/reset")

    # ── callback ─────────────────────────────────────────────────────────
    def _odom(self, m) -> None:
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.x, self.y = m.pose.pose.position.x, m.pose.pose.position.y
        self.t_odom = time.time()
        if self.raw:
            self.raw.write(f"{self.t_odom:.4f},O,,,,{self.yaw:.5f},{self.x:.4f},{self.y:.4f}\n")

    def _scan(self, m) -> None:
        if self.yaw is None:
            return
        pts, _bear, _rng, ldeg = scan_to_base_points(
            np.asarray(m.ranges, dtype=float), m.angle_min, m.angle_increment, m.range_min, m.range_max,
            LIDAR_X, LIDAR_Y, math.radians(LIDAR_YAW_OFFSET_DEG), LIDAR_ANGLE_SIGN, MAX_USE_RANGE)
        if len(pts):
            pts = pts[self_filter_mask(pts, ldeg, FRONT_LEN, REAR_LEN, HALF_WIDTH, SELF_FILTER_MARGIN, BLIND_SECTORS_DEG)]
        self.scans.append((time.time(), self.x, self.y, self.yaw, pts))
        self.t_scan = time.time()
        self.on_scan(pts)

    def on_scan(self, pts: np.ndarray) -> None:
        """Moc cho lop con (rssi_follow.py bam cum chan theo tung vong quet). O day khong lam gi."""

    def abort_scan(self) -> bool:
        """Moc cho lop con: True = bo do giua chung (vd. muc tieu dang bam bat dau di chuyen)."""
        return False

    def _brg(self, m) -> None:
        try:
            self.brg = json.loads(m.data)
            self.t_brg = time.time()
        except json.JSONDecodeError:
            pass

    def _raw(self, m) -> None:
        if not self.raw:
            return
        try:
            for board, rssi, t in json.loads(m.data).get("samples", []):
                self.raw.write(f"{float(t):.4f},R,{board},,{rssi},,,\n")
        except (json.JSONDecodeError, ValueError, TypeError):
            pass

    # ── tien ich ─────────────────────────────────────────────────────────
    def write(self, rec: dict) -> None:
        if self.log:
            rec.update({"t": round(time.time(), 2), "st": self.state, "x": round(self.x, 3), "y": round(self.y, 3),
                        "yaw": None if self.yaw is None else round(self.yaw, 4)})
            self.log.write(json.dumps(rec) + "\n")
            self.log.flush()

    def say(self, text: str, col: str = "", **extra) -> None:
        print(f"[{time.time() - self.t0:6.1f} s] {col}{text}{X if col else ''}", flush=True)
        self.write(dict(msg=text, **extra))

    def spin(self, sec: float, check: bool = True) -> None:
        t_end = time.time() + sec
        while time.time() < t_end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
            if check and self.stop_req:
                raise KeyboardInterrupt

    def cmd(self, v: float, w: float) -> None:
        m = self.Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.pub.publish(m)
        if time.time() - self.t_rec >= 0.5:          # vet 2 Hz de phan tich sau
            self.t_rec = time.time()
            b = self.brg or {}
            self.write({"v": round(v, 3), "w": round(w, 3), "valid": b.get("valid"), "swept": b.get("swept_deg"),
                        "brg": b.get("bearing_odom_rad"), "corr": b.get("corr"), "margin": b.get("margin")})
        self.spin(1.0 / CTRL_HZ)

    def halt(self) -> None:
        if self.pub is not None:
            for _ in range(4):
                self.pub.publish(self.Twist())
                self.spin(0.04, check=False)

    def points(self, keep: float = SCAN_KEEP_SEC) -> np.ndarray:
        """Diem LiDAR cua cac vong quet trong `keep` giay tinh tu vong quet MOI NHAT, doi ve base_link HIEN TAI theo odom
        (Nx2). Tinh tu vong moi nhat chu khong tu dong ho: mot phep tinh nang (so nen ~0.5 s) chan viec nhan /scan, tinh
        tu dong ho thi ngay sau do khong con diem nao -> "khong thay ai" (mo phong 03/10). /scan cu thi guard() dung xe."""
        ref = max((t for t, *_ in self.scans), default=time.time())
        c0, s0 = math.cos(self.yaw), math.sin(self.yaw)
        out = []
        for t, x, y, yaw, p in self.scans:
            if ref - t > keep or not len(p):
                continue
            c, s = math.cos(yaw), math.sin(yaw)
            wx = x + c * p[:, 0] - s * p[:, 1] - self.x
            wy = y + s * p[:, 0] + c * p[:, 1] - self.y
            out.append(np.stack([c0 * wx + s0 * wy, -s0 * wx + c0 * wy], 1))
        return np.concatenate(out) if out else np.zeros((0, 2))

    def problem(self):
        """(ly do, so giay cho phuc hoi) neu KHONG du dieu kien cho xe chuyen dong; None neu on."""
        now = time.time()
        if now - self.t0 > self.a.max_sec:
            return f"het thoi gian thu ({self.a.max_sec:.0f} s)", 0.0
        if now - self.t_pubchk > 1.0:
            self.t_pubchk = now
            self.n_pub = self.node.count_publishers("/cmd_vel")
        if self.n_pub > 1:
            return "co node khac dang ghi /cmd_vel (planner?) — chi chay voi calibrate.launch.py", 0.0
        if now - self.t_scan > SCAN_STALE_SEC:
            return f"LiDAR ngung gui du lieu ({now - self.t_scan:.1f} s)", 5.0
        if now - self.t_odom > 0.5:
            return "mat /odom (driver / khung xe)", 5.0
        if now - self.t_brg > 2.0:
            return "mat /rssi/bearing (T2 con chay khong?)", 5.0
        if not self.brg.get("beacon_ok", False):
            return f"mat beacon ({self.brg.get('beacon_age_sec')} s khong nghe thay)", 20.0
        return None

    def guard(self) -> None:
        p = self.problem()
        if p and p[1] > 0.0:
            # Loi CO THE phuc hoi (mat /odom, /scan, /rssi/bearing, beacon): script co the vua tinh nang 0.4-0.9 s (so nen,
            # chon muc tieu sau khi xoay do) nen chua kip nhan cac tin dang cho trong hang doi -> nhan het roi xet lai.
            # Chay that 06/10: bao NHAM "mat /odom" ngay sau mot lan xoay do -> bo mot lan do hop le, mat them ~11 s.
            # Mat that thi 0.15 s sau van mat -> dung nhu cu (xe di them toi da ~3 cm o 0.22 m/s).
            self.spin(0.15)
            p = self.problem()
        if p:
            raise Stop(*p)

    def rotate_clear(self):
        """(du trong de xoay?, khoang cach vat gan TAM QUAY nhat, huong do cua no)."""
        p = self.points()
        if not len(p):
            return True, 99.0, 0.0
        d = np.hypot(p[:, 0], p[:, 1])
        i = int(np.argmin(d))
        return bool(d[i] >= CLEAR_MIN_M), float(d[i]), math.degrees(math.atan2(p[i, 1], p[i, 0]))

    @staticmethod
    def front_gap(p: np.ndarray) -> float:
        """Khoang trong truoc MUI xe trong hanh lang tien (m). inf neu khong co gi."""
        m = (p[:, 0] > 0.0) & (np.abs(p[:, 1]) <= HALF_WIDTH + SIDE_MARGIN)
        return float(p[m, 0].min() - FRONT_LEN) if m.any() else float("inf")

    @staticmethod
    def near_ahead(p: np.ndarray) -> float:
        """Khoang cach tu TAM QUAY toi vat gan nhat o phia truoc / ngang hong (khong tinh vat da lui ra sau)."""
        m = p[:, 0] > -0.10
        return float(np.hypot(p[m, 0], p[m, 1]).min()) if m.any() else float("inf")

    # ── cac pha ──────────────────────────────────────────────────────────
    def reset_bearing(self) -> None:
        if not self.reset_cli.service_is_ready():
            return
        fut = self.reset_cli.call_async(self.Trigger.Request())
        t = time.time()
        while not fut.done() and time.time() - t < 1.0:
            self.spin(0.02)

    def do_scan(self):
        """Xoay tai cho toi khi rssi_bearing_node co huong hop le. Tra ve huong (rad, khung odom) hoac None."""
        self.state = "DO"
        self.reset_bearing()
        swept, prev, t_start = 0.0, self.yaw, time.time()
        t_limit = math.radians(self.a.max_scan_deg) / self.a.w_scan * 1.6 + 6.0
        t_block, warned = 0.0, 0.0
        while True:
            self.guard()
            if self.abort_scan():                # ca luc dang cho quanh xe trong lai, khong chi luc dang xoay
                self.halt()
                return None
            ok, d, bdeg = self.rotate_clear()
            if not ok:
                self.pub.publish(self.Twist())
                if time.time() - warned > 3.0:
                    warned = time.time()
                    self.say(f"cho: co vat cach tam quay {d:.2f} m o huong {bdeg:+.0f} do (can > {CLEAR_MIN_M:.2f} m moi xoay)", Y)
                self.spin(0.2)
                t_block += 0.2
                prev = self.yaw
                if t_block > self.block_wait:
                    self.say(f"cho {self.block_wait:.0f} s van khong du cho xoay.", Y)
                    return None
                continue
            self.cmd(0.0, self.a.w_scan)
            swept += abs(wrap(self.yaw - prev))
            prev = self.yaw
            b = self.brg
            if (math.degrees(swept) >= self.a.min_scan_deg and b.get("valid") and b.get("stamp", 0.0) > t_start
                    and b.get("swept_deg", 0.0) >= self.a.min_scan_deg - 30.0):
                b_ok = b
                break
            if math.degrees(swept) >= self.a.max_scan_deg or time.time() - t_start - t_block > t_limit:
                self.halt()
                self.say(f"xoay {math.degrees(swept):.0f} do van chua co huong hop le: {b.get('reason', '?')} "
                         f"(corr {b.get('corr')}, margin {b.get('margin')})", Y, scan_fail=b)
                return None
        self.halt()
        self.spin(0.6)                       # cho xe dung han + nhan ban tin moi nhat
        b = self.brg
        if not b.get("valid"):               # vua dung thi tut duoi nguong: van dung ket qua hop le luc ket thuc xoay
            b = b_ok                         # (huong tinh trong khung odom nen xe troi them vai do cung khong sao)
        lv = b.get("levels", {})
        p = self.points()
        self.say(f"DO xong ({math.degrees(swept):.0f} do, {time.time() - t_start:.1f} s): beacon o "
                 f"{math.degrees(wrap(b['bearing_odom_rad'] - self.yaw)):+.0f} do so voi mui xe (corr {b['corr']:.2f}, margin {b['margin']:.2f}; muc "
                 + "/".join(f"{lv.get(k, 0):.0f}" for k in "ABC") + " dBm)", G,
                 scan_ok=b, pts=[[round(float(u), 2), round(float(v), 2)] for u, v in p[::2]])
        return float(b["bearing_odom_rad"])

    def do_turn(self, target: float) -> bool:
        self.state = "QUAY"
        t_start = time.time()
        while True:
            self.guard()
            err = wrap(target - self.yaw)
            if abs(err) <= math.radians(6.0):
                break
            ok, d, bdeg = self.rotate_clear()
            if not ok or time.time() - t_start > 12.0:
                self.halt()
                self.say(f"khong quay duoc ve huong can quay (con lech {math.degrees(err):+.0f} do; vat gan tam quay nhat "
                         f"{d:.2f} m o {bdeg:+.0f} do)", Y)
                return False
            w = max(-0.8, min(0.8, 1.6 * err))
            if abs(w) < 0.2:
                w = math.copysign(0.2, err)
            self.cmd(0.0, w)
        self.halt()
        self.spin(0.3)
        return True

    def do_drive(self, heading: float):
        """Di thang theo heading (khung odom). Tra ve (ket qua, gia tri, quang duong da di, vat chan co o GIUA khong):
        ('toi', khoang trong truoc mui) | ('ke ben', kc toi tam quay) | ('het buoc', quang duong)."""
        self.state = "DI"
        x0, y0 = self.x, self.y
        armed = False
        while True:
            self.guard()
            p = self.points()
            dist = math.hypot(self.x - x0, self.y - y0)
            gap = self.front_gap(p)
            if gap < self.a.stop_gap:
                self.halt()
                # Vat chan nam trong be ngang THAN XE (giua duong) hay chi cham mep hanh lang (15 cm le moi ben)?
                blk = (p[:, 0] > 0.0) & (p[:, 0] - FRONT_LEN < gap + 0.15) & (np.abs(p[:, 1]) <= HALF_WIDTH)
                return "toi", gap, dist, bool(blk.any())
            near = self.near_ahead(p)
            if near > PASS_R + 0.08:
                armed = True                 # vat da o san sat ben luc bat dau di thi cho qua (hanh lang van giu 15 cm)
            elif armed and near < PASS_R:
                self.halt()
                return "ke ben", near, dist, False
            # Het buoc ma dang luot ngang mot vat (< 0.53 m tu tam quay) thi KHONG xoay do duoc o day:
            # di tiep toi da 0.5 m cho vat lui ra sau roi moi dung
            if dist >= self.a.step and (dist >= self.a.step + 0.5 or float(np.hypot(p[:, 0], p[:, 1]).min(initial=9.0)) >= CLEAR_MIN_M + 0.02):
                self.halt()
                return "het buoc", dist, dist, False
            slow = gap < self.a.stop_gap + 0.5 or (armed and near < PASS_R + 0.2)
            v = min(self.a.v, 0.10) if slow else self.a.v
            err = wrap(heading - self.yaw)
            self.cmd(v, max(-0.4, min(0.4, 1.2 * err)))

    def hold(self, gap0: float) -> str:
        """Dung cho. 'di khoi' khi vat da dung truoc mui (cach gap0) khong con o do du --settle-sec — so voi gap0 chu
        khong doi "trong han": phong nho thi sau lung nguoi la tuong. 'het gio' sau --hold-sec (0 = cho mai)."""
        self.state = "CHO"
        t_start, t_free = time.time(), None
        while True:
            if time.time() - self.t0 > self.a.max_sec:
                return "het thoi gian"
            self.pub.publish(self.Twist())
            self.spin(0.2)
            if time.time() - self.t_scan > SCAN_STALE_SEC:
                t_free = None
                continue
            if self.front_gap(self.points()) > gap0 + 0.30:
                t_free = t_free or time.time()
                if time.time() - t_free >= self.a.settle_sec:
                    return "di khoi"
            else:
                t_free = None
            if self.a.hold_sec > 0 and time.time() - t_start > self.a.hold_sec:
                return "het gio"

    def wait_recover(self, sec: float) -> bool:
        """Xe dung yen, cho dieu kien tro lai (toi da sec giay). True neu da on dinh lai 1 s lien."""
        t_start, t_ok = time.time(), None
        while time.time() - t_start < sec:
            self.pub.publish(self.Twist())
            self.spin(0.2)
            p = self.problem()
            if p is None:
                t_ok = t_ok or time.time()
                if time.time() - t_ok >= 1.0:
                    return True
            else:
                t_ok = None
                if p[1] <= 0.0:
                    return False
        return False

    # ── chay ─────────────────────────────────────────────────────────────
    def run(self) -> int:
        t = time.time()
        while time.time() - t < 1.5 or (time.time() - t < 6.0 and (self.yaw is None or not self.scans or self.brg is None)):
            self.spin(0.1)
        if self.yaw is None:
            print(f"{R}Khong co /odom — T1 (calibrate.launch.py) da chay chua, khung xe da bat nguon chua?{X}")
            return 1
        if not self.scans:
            print(f"{R}Khong co /scan — LiDAR khong gui du lieu (rut cam lai LiDAR, launch lai T1).{X}")
            return 1
        if self.brg is None:
            print(f"{R}Khong co /rssi/bearing — T2 (rssi.launch.py) da chay chua?{X}")
            return 1
        n_pub = self.node.count_publishers("/cmd_vel")
        if n_pub > 0:
            print(f"{R}Dang co {n_pub} node publish /cmd_vel (follow_planner?). Chi chay voi calibrate.launch.py.{X}")
            return 1
        if self.brg.get("reason") == "khong co file mau":
            print(f"{R}rssi_bearing_node khong co file mau — chay rssi_rotate.py --calib roi launch lai T2.{X}")
            return 1
        if not self.brg.get("beacon_ok", False):
            print(f"{R}Khong nghe thay beacon — beacon da bat chua? (ros2 topic echo /rssi/status --field data --full-length){X}")
            return 1
        if not self.reset_cli.wait_for_service(timeout_sec=2.0):
            print(f"{Y}Khong thay service /rssi/reset (rssi_bearing_node ban cu?) — van chay, khong xoa duoc mau cu.{X}")
        self.t0 = time.time()
        if self.a.delay > 0:
            print(f"{B}Di toi vi tri va DUNG YEN (deo beacon sau that lung, quay lung ve xe). Xe bat dau xoay sau "
                  f"{self.a.delay:.0f} s...{X}", flush=True)
            t_end = time.time() + self.a.delay
            while time.time() < t_end:
                self.spin(min(2.0, max(0.05, t_end - time.time())))
                lv = self.brg.get("levels", {})
                print(f"   con {max(0.0, t_end - time.time()):3.0f} s   muc A/B/C: "
                      + "/".join(f"{lv[k]:.0f}" if k in lv else "--" for k in "ABC") + " dBm", flush=True)
        self.pub = self.node.create_publisher(self.Twist, "/cmd_vel", 10)
        self.t0 = time.time()
        self.say(f"Bat dau: buoc {self.a.step} m, dung cach vat {self.a.stop_gap} m truoc mui, toi da {self.a.max_cycles} chu ky "
                 f"/ {self.a.max_sec:.0f} s. Muon dung: dung truoc mui xe roi Ctrl-C.", B, args=vars(self.a))
        fails = 0
        unconf = None            # khoang trong truoc mui cua lan gap vat CHUA xac nhan (dang do lai tai cho)
        recheck = 0              # so lan lien tiep da do lai tai cho
        for cyc in range(1, self.a.max_cycles + 1):
            try:
                self.say(f"--- chu ky {cyc}: DO (xoay tim huong beacon) ---")
                phi = self.do_scan()
                gap, sure = None, True
                if phi is None and unconf is not None:
                    self.say("khong do lai duoc de xac nhan — coi nhu da toi.", Y)
                    gap = unconf
                elif phi is None:
                    fails += 1
                    if fails >= 3:
                        self.say("3 lan lien tiep khong tim duoc huong — dung.", R)
                        return 2
                    continue
                unconf = None
                if gap is None:
                    fails = 0
                    if not self.do_turn(phi):
                        continue
                    res, val, driven, centered = self.do_drive(phi)
                    if res == "het buoc":
                        recheck = 0
                        self.say(f"DI xong buoc {val:.2f} m, chua gap gi truoc mui — do lai.")
                        continue
                    if res == "ke ben":
                        recheck = 0
                        self.say(f"DUNG SOM: co vat cach tam xe {val:.2f} m o sat ben — do lai xem co phai nguoi khong.")
                        continue
                    gap = val
                    # Chac la nguoi khi: vat nam GIUA duong va huong vua do o gan (di <= 0.7 m). Sai so huong 25 do o
                    # 2 m = lech ngang 0.85 m: di xa roi moi gap vat, hoac vat chi cham mep hanh lang, thi de la vat khac.
                    sure = centered and (driven <= CONFIRM_MAX_M or not self.a.confirm)
                    if not sure and self.a.confirm and recheck < 2:
                        recheck += 1
                        unconf = val
                        self.say(f"GAP VAT cach mui xe {val:.2f} m ({'giua duong' if centered else 'o MEP duong di'}, sau khi di "
                                 f"{driven:.2f} m) — do lai tai day de xac nhan.", Y)
                        continue
                recheck = 0
                if sure:
                    self.say(f"TOI: co vat/nguoi cach mui xe {gap:.2f} m theo huong beacon — dung cho.", G, arrived=round(gap, 3))
                else:
                    where = "o MEP duong di, khong phai ngay truoc mui" if not centered else "da do lai 2 lan van chua chac la nguoi"
                    self.say(f"BI CHAN: vat cach mui xe {gap:.2f} m ({where}) — script nay khong biet ne vat can. Dung cho.",
                             Y, blocked=round(gap, 3))
                if self.a.once:
                    return 0 if sure else 3
                why = self.hold(gap)
                if why == "het thoi gian":
                    break
                self.say("nguoi/vat truoc mui da di khoi — tim lai." if why == "di khoi"
                         else "het thoi gian cho — do lai xem beacon o dau.")
            except Stop as e:
                self.halt()
                self.say(f"DUNG: {e.why}", R)
                if e.wait <= 0.0:
                    return 2
                self.state = "CHO PHUC HOI"
                if not self.wait_recover(e.wait):
                    self.say(f"cho {e.wait:.0f} s van chua phuc hoi — dung han.", R)
                    return 2
                self.say("da phuc hoi — lam lai tu DO.", G)
        self.say("Ket thuc.", B)
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Thu bam theo nguoi deo beacon chi bang RSSI (do - quay - di)")
    ap.add_argument("--step", type=float, default=1.2, help="quang duong toi da moi lan di (m)")
    ap.add_argument("--v", type=float, default=0.18, help="toc do tien (m/s), toi da 0.22")
    ap.add_argument("--w-scan", type=float, default=0.9, help="toc do xoay do (rad/s) — 0.9 da kiem tren xe (that ~0.82)")
    ap.add_argument("--stop-gap", type=float, default=0.70, help="dung khi vat truoc MUI xe gan hon ngan nay (m)")
    ap.add_argument("--min-scan-deg", type=float, default=330.0, help="xoay it nhat ngan nay moi tin huong (do)")
    ap.add_argument("--max-scan-deg", type=float, default=760.0, help="xoay toi da ngan nay ma chua co huong thi bo (do)")
    ap.add_argument("--settle-sec", type=float, default=3.0, help="truoc mui trong lai du ngan nay moi do lai (s)")
    ap.add_argument("--hold-sec", type=float, default=25.0,
                    help="toi noi roi ma vat truoc mui khong di: cho ngan nay roi do lai (0 = cho mai)")
    ap.add_argument("--no-confirm", dest="confirm", action="store_false",
                    help="khong do lai de xac nhan khi gap vat sau mot doan di dai")
    ap.add_argument("--max-cycles", type=int, default=20)
    ap.add_argument("--max-sec", type=float, default=420.0)
    ap.add_argument("--delay", type=float, default=8.0, help="dem nguoc truoc khi xe bat dau (s) — de di toi vi tri")
    ap.add_argument("--once", action="store_true", help="dung han sau lan toi dau tien")
    ap.add_argument("--log", default=time.strftime("rssi_seek_%H%M%S.jsonl"))
    args = ap.parse_args()
    args.v = min(args.v, 0.22)
    args.w_scan = min(max(args.w_scan, 0.3), 1.0)
    args.stop_gap = max(args.stop_gap, 0.45)

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    s = Seeker(args)

    # Ctrl-C / SIGTERM chi dat co; spin() nem KeyboardInterrupt o cho an toan. Nem thang tu trinh xu ly tin hieu thi
    # co khi roi dung luc rclpy dang nhan ban tin (ma C++) va bien thanh RuntimeError -> khong bat duoc (thay 02/10).
    def _stop(*_):
        s.stop_req = True
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    rc = 1
    try:
        rc = s.run()
    except KeyboardInterrupt:
        print(f"\n{Y}Ctrl-C — dung xe{X}")
        rc = 130
    finally:
        s.halt()
        if s.log:
            print(f"Nhat ky: {os.path.abspath(args.log)}  (+ .csv mau tho)")
            s.log.close()
            s.raw.close()
        s.node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
