#!/usr/bin/env python3
"""
sil_rssi.py — MO PHONG VONG KIN CO ROS cho phan RSSI (01/10): the gioi gia + code THAT se chay tren xe.

Khac sim_rssi.py (thuan numpy, chi thuat toan): o day chay DUNG ba thu se chay tren xe —
rssi_scanner_node (doc cong serial; o day la pty), rssi_bearing_node, scripts/rssi_seek.py, va o che do BAM ca
follow_planner_node NGUYEN BAN + scripts/rssi_follow.py (launch rssi_follow.launch.py) — noi voi mot the gioi gia:
  - xe: mo hinh driver BW-DR03 (chia % nguyen, toi thieu 5 %, vung chet, cmd_timeout 1 s) + tre lenh + quan tinh banh
  - /odom 50 Hz, dem THIEU goc xoay 1 % nhu xe that (khung odom lech dan khoi the gioi); /scan 10 Hz kieu SensorDataQoS,
    360 tia, mat 40 % tia moi vong, co tia than xe trong cung mu, va MEO QUET nhu LiDAR quay that (1 vong 0.1 s: xe
    dang xoay thi tia cuoi vong lech goc so voi tia dau vong)
  - 3 board RSSI = 3 pty in dong "R,<id>,<seq>,<ms>,<rssi>":
      --rssi real (mac dinh): PHAT LAI MAU DO THAT (rssi_logs/q2_*.csv, 11 lan do tren xe) theo goc nguoi nhin tu xe,
                              dung mau hieu chinh THAT config/rssi_template.json -> sai so huong giong xe that
                              (TB ~13-15 do, lon nhat ~25-30 do)
      --rssi sim            : mo hinh sim_rssi.py (lac quan hon: TB ~9 do)
  - phong: trong, hoac DO DAC THAT lay tu mot vong quet LiDAR cua phong thu (rssi_logs/q2_m045_1.json)
  - nguoi deo beacon: hai chan (LiDAR thay; dang di thi buoc truoc/sau luan phien); dung yen, sang cho khac sau khi
    xe toi, hoac di lien tuc; luc dau co the dung CANH XE roi moi di ra vi tri (nhu nguoi van hanh that)
  - nguoi thu hai khong deo beacon: di cat ngang, dung yen canh duong di, hoac DI CUNG chu (--companion)
  - chu di sat xe (--pass-r), di sang cho khac giua luc xe xoay do (--move-on-spin), bi che kin mot luc khi dang di
    (--hide-walk: LiDAR khong thay, RSSI van nhu thuong)
  - su co: LiDAR / odom / beacon ngung, node la ghi /cmd_vel, script bi kill

AN TOAN: moi tien trinh chay trong MIEN ROS RIENG (ROS_DOMAIN_ID >= 60) nen khong lenh nao toi duoc xe that,
ke ca khi driver that dang chay o mien mac dinh. Tien trinh "the gioi" tu choi chay neu ROS_DOMAIN_ID < 50.

CACH DUNG (da source ROS + install/setup.bash cua repo)
  python3 sil_rssi.py                    # bo chinh: 16 kich ban rssi_seek + 21 kich ban rssi_follow (~15 phut) + bang DAT / LOI
  python3 sil_rssi.py b_an_mot --follow-script /duong/dan/rssi_follow_cu.py   # so ban cu / ban moi tren cung kich ban
  python3 sil_rssi.py --more             # bo MO RONG: 24 bien the che do bam (hat ngau nhien, phong thu hai, vong thung...)
  python3 sil_rssi.py phongF b_phong3    # chi vai kich ban (ten b_* r_* ve_* x_* = che do BAM: planner that + rssi_follow.py)
  python3 sil_rssi.py --world-extra "--skew -1"   # gia dinh khac ve cam bien (LiDAR quay chieu nguoc, ...)
  python3 sil_rssi.py --list
  python3 sil_rssi.py --rssi sim         # dung mo hinh thay cho mau do that
  python3 sil_rssi.py --keep /tmp/x      # giu nhat ky (world.jsonl, seek.jsonl, seek.log) de xem lai
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
LOGS = os.path.normpath(os.path.join(PKG, "..", "..", "rssi_logs"))
sys.path.insert(0, HERE)
sys.path.insert(0, PKG)

F_LEN, R_LEN, HW = 0.14, 0.33, 0.30          # footprint (CLAUDE.md muc 2)
LIDAR_X = 0.10
LEG_R, LEG_SEP = 0.055, 0.09


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ─────────────────────────── phong that tu mot vong quet LiDAR ───────────────────────────

def room_from_scan(path: str):
    """Do dac cua phong thu: moi tia LiDAR that -> mot vat tron 4 cm; noi cac diem ke nhau < 0.30 m (mat tuong,
    canh tu) de tia mo phong khong lot qua. Bo cac diem la NGUOI dung luc quet (quanh huong da biet)."""
    from rssi_rotate import scan_to_base
    m = json.load(open(path))
    pts = sorted(scan_to_base(m["scan_ranges"], m["scan_angle_min"], m["scan_angle_inc"], m["scan_range_max"]))
    t = float(m.get("truth_deg", 999.0))
    keep = [(d * math.cos(math.radians(b)), d * math.sin(math.radians(b))) for b, d in pts
            if not (abs((b - t + 180.0) % 360.0 - 180.0) <= 14.0 and 1.1 <= d <= 2.4)]
    circles = [(x, y, 0.04) for x, y in keep]
    for (x1, y1), (x2, y2) in zip(keep, keep[1:] + keep[:1]):
        g = math.hypot(x2 - x1, y2 - y1)
        if 0.08 < g < 0.30:
            n = int(g / 0.06)
            for k in range(1, n + 1):
                f = k / (n + 1)
                circles.append((x1 + f * (x2 - x1), y1 + f * (y2 - y1), 0.04))
    return circles


class RealRSSI:
    """Phat lai MAU RSSI THAT (cac lan do q2_* tren xe, 01/10) theo goc nguoi nhin tu xe.
    Moi lan xe (hoac nguoi) doi cho > 0.3 m thi boc mot lan do khac (phan xa khac). Mau lay ngau nhien trong
    +-8 do quanh goc hien tai cua CHINH lan do do -> giu nguyen hinh dang + sai lech that cua lan do ay."""

    def __init__(self, rng, pattern: str) -> None:
        from rssi_rotate import load as load_run
        self.rng = rng
        self.runs = []
        for f in sorted(glob.glob(pattern)):
            t_r, sid, rssi, t_o, yaw_u, meta = load_run(f)
            if "truth_deg" not in meta or "t_rot_start" not in meta:
                continue
            sel = (t_o >= meta["t_rot_start"]) & (t_o <= meta["t_rot_end"])
            t_o, yaw_u = t_o[sel], yaw_u[sel]
            keep = (t_r >= t_o[0]) & (t_r <= t_o[-1])
            th = wrap(yaw_u[0] + math.radians(meta["truth_deg"]) - np.interp(t_r[keep], t_o, yaw_u))
            dur = t_o[-1] - t_o[0]
            run = {"name": os.path.basename(f), "d": float(meta.get("dist_m", 2.0))}
            for b in "ABC":
                m = sid[keep] == b
                o = np.argsort(th[m])
                run[b] = (th[m][o], rssi[keep][m][o], m.sum() / dur)
            self.runs.append(run)
        self.cur = None
        self.anchor = None
        self.used = []

    def tick(self, b: str, theta: float, dist: float, key: tuple, dt: float):
        if self.anchor is None or math.hypot(key[0] - self.anchor[0], key[1] - self.anchor[1]) > 0.3 \
                or math.hypot(key[2] - self.anchor[2], key[3] - self.anchor[3]) > 0.3:
            self.anchor = key
            self.cur = self.runs[int(self.rng.integers(len(self.runs)))]
            self.used.append(self.cur["name"])
        th, rs, rate = self.cur[b]
        near = np.abs(wrap(th - theta)) <= math.radians(8.0)
        k = int(near.sum())
        if k == 0:
            return []
        dens = k / (len(th) * 16.0 / 360.0)                  # mat do mau that quanh goc nay so voi trung binh
        out = []
        for _ in range(int(self.rng.poisson(rate * dt * dens))):
            v = float(rs[near][int(self.rng.integers(k))]) - 24.0 * math.log10(max(0.4, dist) / self.cur["d"])
            if v > -97.0:
                out.append(int(round(v)))
        return out


# ─────────────────────────────────── the gioi gia ───────────────────────────────────

def world_main(argv) -> None:
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) < 50:
        sys.exit("sil_rssi --world: phai dat ROS_DOMAIN_ID >= 50 (mien rieng) — khong chay chung mien voi xe that")
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    import sim_rssi as SR
    from person_follow_nav.rssi_df import save_template

    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--robot", default="0,0,0", help="x,y,yaw_do")
    ap.add_argument("--person", default="2.5,0", help="'x,y;x,y;...' cac diem nguoi dung lan luot")
    ap.add_argument("--circles", default="", help="do dac tron 'x,y,r;x,y,r'")
    ap.add_argument("--room-scan", default="", help="vong quet LiDAR that (.json cua rssi_rotate) lam do dac")
    ap.add_argument("--intruder", default="", help="nguoi thu hai di cat ngang: x0,y0,x1,y1,t_bat_dau,toc_do")
    ap.add_argument("--walk-always", type=float, default=0.0, help="nguoi di lien tuc qua cac diem (m/s)")
    ap.add_argument("--walk-start", type=float, default=0.0, help="voi --walk-always: dung yen toi giay nay roi moi di")
    ap.add_argument("--gait", type=float, default=0.12, help="bien do chan buoc truoc/sau khi dang di (m); 0 = hai chan song song")
    ap.add_argument("--impatient", type=float, default=0.0,
                    help="dung cho qua ngan nay giay ma xe chua toi thi nguoi tu di sang cho ke (0 = cho mai)")
    ap.add_argument("--enter", type=float, default=0.0,
                    help="nguoi luc dau dung CANH XE (nhu nguoi van hanh bam phim tren laptop), toi giay nay moi di ra vi tri dau")
    ap.add_argument("--pass-r", type=float, default=1.05,
                    help="nguoi di vong qua xe o cach tam xe ngan nay (m); chay that 06/10: nguoi di sat toi 0.44 m")
    ap.add_argument("--move-on-spin", type=float, default=0.0,
                    help="nguoi dung o diem dau cho toi khi xe da xoay ngan nay do (lan do dau) roi moi di sang diem thu hai "
                         "= nguoi di chuyen TRONG LUC xe xoay do (0 = tat)")
    ap.add_argument("--companion", default="",
                    help="nguoi thu hai (khong deo beacon) DI CUNG chu: 'dx,dy' lech so voi chu (khung the gioi), chi xuat hien "
                         "khi chu roi diem dau; chu dung lai o diem thu hai thi nguoi nay di tiep 4 m roi khuat")
    ap.add_argument("--hide-walk", default="",
                    help="'t0,t1': tu t0 toi t1 giay sau khi chu roi diem dau, LiDAR KHONG thay chu (va nguoi di cung) = di "
                         "khuat sau vat; RSSI van nhu thuong")
    ap.add_argument("--rssi", default="real", choices=("sim", "real"))
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--K", type=float, default=3.0)
    ap.add_argument("--rear", default="manh")
    ap.add_argument("--dropout", type=float, default=0.40)
    ap.add_argument("--skew", type=float, default=1.0,
                    help="LiDAR that quet 1 vong mat 0.1 s: tia i do som hon luc phat (359-i)/360 x 0.1 s -> xe dang xoay thi "
                         "diem bi lech goc (0.86 rad/s: toi ~5 do). 1 = nhu that, -1 = LiDAR quay chieu nguoc lai, 0 = tat")
    ap.add_argument("--odom-yaw-scale", type=float, default=0.99,
                    help="odom dem THIEU goc xoay (do 01/10: xe quay that nhieu hon odom 2.6-4.5 do moi vong ~ 1 %%)")
    ap.add_argument("--delay", type=float, default=0.08, help="tre lenh -> banh (s)")
    ap.add_argument("--tau", type=float, default=0.25, help="hang so thoi gian banh (s); voi tre: xe troi ~18 do khi dung xoay 0.86 rad/s (do 01/10)")
    ap.add_argument("--dwell", type=float, default=5.0, help="xe dung ngan nay giay thi nguoi sang cho khac")
    ap.add_argument("--walk", type=float, default=0.8)
    ap.add_argument("--lidar-dead-after", type=float, default=0.0)
    ap.add_argument("--lidar-back-after", type=float, default=0.0)
    ap.add_argument("--beacon-dead-after", type=float, default=0.0)
    ap.add_argument("--beacon-back-after", type=float, default=0.0)
    ap.add_argument("--odom-dead-after", type=float, default=0.0)
    a = ap.parse_args(argv)
    os.makedirs(a.dir, exist_ok=True)

    class World(Node):

        def __init__(self) -> None:
            super().__init__("sil_world")
            self.rng = np.random.default_rng(a.seed)
            self.room = (-4.0, 4.0, -3.5, 3.5)
            self.circles = [tuple(float(v) for v in q.split(",")) for q in a.circles.split(";") if q]
            if a.room_scan:
                self.circles += room_from_scan(a.room_scan)
                self.room = (-5.0, 5.0, -5.0, 5.0)
            self.C = np.array(self.circles, float).reshape(-1, 3)
            x, y, yaw = (float(v) for v in a.robot.split(","))
            self.x, self.y, self.yaw = x, y, math.radians(yaw)
            self.ox_, self.oy_, self.oyaw = self.x, self.y, self.yaw     # tu the theo ODOM (lech dan khoi tu the that)
            self.yaw_u = self.yaw                                        # yaw that khong cuon +-pi (de noi suy)
            self.hist = deque(maxlen=64)                                 # (t, x, y, yaw_u) 0.6 s gan nhat
            self.way = [tuple(float(v) for v in q.split(",")) for q in a.person.split(";")]
            self.pi = 0
            self.px, self.py = self.way[0]
            self.walking = False
            self.t_wait = None
            self.entered = a.enter <= 0
            if a.enter > 0:                    # dung canh xe, ben phai, cach tam 0.7 m
                self.px = self.x + 0.1 * math.cos(self.yaw) + 0.7 * math.sin(self.yaw)
                self.py = self.y + 0.1 * math.sin(self.yaw) - 0.7 * math.cos(self.yaw)
            self.hx, self.hy = 1.0, 0.0        # huong dang di cua nguoi
            self.moving = False
            self.t_follow = None               # tu luc xe lan dau toi gan nguoi: thong ke khoang cach bam
            self.d_sum = self.d_n = self.d_far = 0.0
            self.d_max = 0.0
            self.intr = [float(v) for v in a.intruder.split(",")] if a.intruder else None
            self.comp = [float(v) for v in a.companion.split(",")] if a.companion else None
            self.cxy = None                    # vi tri nguoi di cung (None = chua xuat hien)
            self.cdir = (1.0, 0.0)
            self.c_left = 4.0                  # quang duong con lai sau khi chu dung (roi khuat)
            self.rot = 0.0                     # tong goc xe da xoay (rad) — cho --move-on-spin
            self.hide = [float(v) for v in a.hide_walk.split(",")] if a.hide_walk else None
            self.t_leave = None                # luc chu roi diem dau (di sang diem thu hai)
            self.vr = self.vl = 0.0            # toc do banh that (m/s)
            self.tr = self.tl = 0.0            # toc do banh muc tieu
            self.cmdq = []                     # (t_ap_dung, vr, vl)
            self.t_cmd = 0.0
            self.t0 = self.t_prev = self.t_stop = time.time()
            self.t_wp = self.t0                # luc nguoi dung lai o diem hien tai
            self.min_obs = self.min_person_mv = self.min_intr_mv = 9.0
            self.n_cmd = 0
            self.arrivals = []

            self.real = None
            if a.rssi == "real":
                self.real = RealRSSI(self.rng, os.path.join(LOGS, "q2_[pm]*.csv"))
                if not self.real.runs:
                    sys.exit(f"khong co mau do that trong {LOGS} — dung --rssi sim")
                shutil.copy(os.path.join(PKG, "config", "rssi_template.json"), os.path.join(a.dir, "tmpl.json"))
            else:
                self.cfg = SR.Cfg(K=a.K, rear=a.rear)
                self.field = SR.Field(self.rng, a.K)
                tmpl, alpha = SR.calibrate(np.random.default_rng(a.seed + 1000), self.cfg)   # hieu chinh o "cho khac"
                save_template(tmpl, os.path.join(a.dir, "tmpl.json"), alpha)
            self.masters = {}
            self.seq = {b: 0 for b in "ABC"}
            for b in "ABC":
                m, s = os.openpty()
                link = os.path.join(a.dir, f"p_{b}")
                if os.path.lexists(link):
                    os.remove(link)
                os.symlink(os.ttyname(s), link)
                os.close(s)                 # giu fd slave thi rssi_scanner_node coi la "cong dang bi giu" va khong mo
                os.set_blocking(m, False)
                self.masters[b] = m
                self._w(b, f"I,{b},00:00:00:00:00:0{b}\n")

            self.log = open(os.path.join(a.dir, "world.jsonl"), "w")
            self.po = self.create_publisher(Odometry, "/odom", 50)
            self.ps = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
            self.create_subscription(Twist, "/cmd_vel", self._cmd, 10)
            self.create_timer(0.01, self._step)
            self.create_timer(0.02, self._odom)
            self.create_timer(0.10, self._scan)
            self.create_timer(0.05, self._rssi)
            self.create_timer(0.10, self._rec)

        def _w(self, b: str, text: str) -> None:
            try:
                os.write(self.masters[b], text.encode())
            except (BlockingIOError, OSError):
                pass

        # ── driver BW-DR03 (decoded_serial_node.cmd_vel_callback) + dong hoc ──
        def _cmd(self, m) -> None:
            self.n_cmd += 1
            now = time.time()
            self.t_cmd = now
            v, w = m.linear.x, m.angular.z
            if abs(v) < 0.02 and abs(w) < 0.02:
                tr = tl = 0.0
            else:
                xn = max(-1.0, min(1.0, v / 0.49))          # max_linear
                zn = max(-1.0, min(1.0, w / 2.46))          # max_angular
                r, l = xn + zn, xn - zn
                mx = max(abs(r), abs(l), 1.0)

                def wheel(val: float) -> float:
                    if abs(val) < 0.02:
                        return 0.0
                    pct = max(5, min(60, int(abs(val) * 60)))   # max_percent 60, toi thieu 5 %
                    return math.copysign(pct * 0.49 / 60.0, val)
                tr, tl = wheel(r / mx), wheel(l / mx)
            self.cmdq.append((now + a.delay, tr, tl))

        def _step(self) -> None:
            now = time.time()
            dt = min(0.05, now - self.t_prev)
            self.t_prev = now
            while self.cmdq and self.cmdq[0][0] <= now:
                _, self.tr, self.tl = self.cmdq.pop(0)
            if now - self.t_cmd > 1.0:                      # cmd_timeout cua driver
                self.tr = self.tl = 0.0
            k = 1.0 - math.exp(-dt / a.tau)
            self.vr += (self.tr - self.vr) * k
            self.vl += (self.tl - self.vl) * k
            v = 0.5 * (self.vr + self.vl)
            w = (self.vr - self.vl) / 0.40                  # wheel_separation
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.yaw_u += w * dt
            self.yaw = wrap(self.yaw_u)
            self.ox_ += v * math.cos(self.oyaw) * dt        # odom: cung quang duong, goc xoay dem thieu
            self.oy_ += v * math.sin(self.oyaw) * dt
            self.oyaw = wrap(self.oyaw + w * a.odom_yaw_scale * dt)
            self.hist.append((now, self.x, self.y, self.yaw_u))
            self.rot += abs(w) * dt
            if abs(v) > 0.01 or abs(w) > 0.03:
                self.t_stop = now
            self._person(now, dt)
            self._companion(dt)
            self._clear(abs(v) > 0.02 or abs(w) > 0.05)

        def _companion(self, dt: float) -> None:
            """Nguoi di cung chu (--companion): xuat hien khi chu roi diem dau, di song song lech (dx, dy); chu dung o diem
            thu hai thi nguoi nay di tiep theo huong cu 4 m roi khuat."""
            if self.comp is None or self.c_left <= 0.0:
                return
            if self.pi == 1 and self.walking:
                self.cxy = (self.px + self.comp[0], self.py + self.comp[1])
                self.cdir = (self.hx, self.hy)
            elif self.cxy is not None:
                step = a.walk * dt
                self.cxy = (self.cxy[0] + self.cdir[0] * step, self.cxy[1] + self.cdir[1] * step)
                self.c_left -= step
                if self.c_left <= 0.0:
                    self.cxy = None

        def _person(self, now: float, dt: float) -> None:
            d_rp = math.hypot(self.px - self.x, self.py - self.y)
            if self.t_follow is None and d_rp < 1.5:
                self.t_follow = now
            if self.t_follow is not None:
                self.d_sum += d_rp
                self.d_n += 1
                self.d_far += d_rp > 2.5
                self.d_max = max(self.d_max, d_rp)
            self.moving = False
            if not self.entered:
                if now - self.t0 < a.enter:
                    return
                self.entered = self.walking = True          # di ra vi tri dau tien
            if a.walk_always > 0 and not self.walking and now - self.t0 >= a.walk_start and self.pi > 0:
                self.walking = True
            speed = a.walk_always if (a.walk_always > 0 and self.pi > 0) else a.walk
            if a.walk_always > 0 and self.pi == 0 and not self.walking and now - self.t0 >= a.walk_start:
                self.pi, self.walking = 1, True
            if (a.move_on_spin > 0 and self.pi == 0 and not self.walking and len(self.way) > 1
                    and math.degrees(self.rot) >= a.move_on_spin):
                self.pi, self.walking, self.t_wait = 1, True, None      # di sang diem thu hai giua luc xe dang xoay do
            if not self.walking and self.t_wait is None:
                self.t_wait = now
            # "Xe da toi": xe co CHUYEN DONG sau khi nguoi dung lai o diem nay (do / chay toi), roi dung yen du lau o gan.
            # Khong co dieu kien dau thi diem dau tien cach xe < 1.7 m bi tinh la "da toi" ngay luc xe con chua bat dau.
            if (a.walk_always <= 0 and not self.walking and self.pi + 1 < len(self.way)
                    and self.t_stop > self.t_wp and now - self.t_stop >= a.dwell and d_rp < 1.7):
                self.arrivals.append({"t": round(now - self.t0, 1), "d": round(d_rp, 2), "ang": round(math.degrees(
                    wrap(math.atan2(self.py - self.y, self.px - self.x) - self.yaw)), 1)})
                self.pi += 1
                self.walking, self.t_wait = True, None
            elif (a.impatient > 0 and a.walk_always <= 0 and not self.walking and self.pi + 1 < len(self.way)
                  and now - self.t_wait >= a.impatient):
                self.pi += 1                    # xe mai khong toi: nguoi that se khong dung cho mai
                self.walking, self.t_wait = True, None
            if not self.walking:
                return
            if self.pi == 1 and self.t_leave is None:
                self.t_leave = now
            tx, ty = self.way[self.pi]
            d = math.hypot(tx - self.px, ty - self.py)
            if d < speed * dt + 1e-6:
                self.px, self.py = tx, ty
                if a.walk_always > 0 and self.pi > 0:
                    self.pi = self.pi % (len(self.way) - 1) + 1     # vong qua cac diem 1..n-1
                else:
                    self.walking = False
                    self.t_wp = now
                return
            ux, uy = (tx - self.px) / d, (ty - self.py) / d
            rx, ry = self.px - self.x, self.py - self.y
            dr = math.hypot(rx, ry)
            if dr < a.pass_r:                               # nguoi that di VONG qua xe chu khong di xuyen qua no
                nx, ny = rx / dr, ry / dr
                inward = -(ux * nx + uy * ny)
                if inward > 0:
                    ux, uy = ux + inward * nx, uy + inward * ny
                    if math.hypot(ux, uy) < 0.2:
                        ux, uy = -ny, nx
                    if dr < a.pass_r - 0.10:
                        ux, uy = ux + 0.6 * nx, uy + 0.6 * ny
                    k = math.hypot(ux, uy)
                    ux, uy = ux / k, uy / k
            self.px += ux * speed * dt
            self.py += uy * speed * dt
            self.hx, self.hy, self.moving = ux, uy, True

        def legs(self):
            """Hai chan nguoi deo beacon. Dung yen: xep ngang so voi huong nhin tu xe. Dang di: hai chan lech
            trai/phai theo huong di va buoc truoc/sau luan phien (LiDAR luc thay mot cum, luc thay hai)."""
            if self.moving and a.gait > 0:
                sw = a.gait * math.sin(2.0 * math.pi * 0.9 * (time.time() - self.t0))
                lx, ly = -self.hy, self.hx
                return [(self.px + LEG_SEP * lx + sw * self.hx, self.py + LEG_SEP * ly + sw * self.hy, LEG_R),
                        (self.px - LEG_SEP * lx - sw * self.hx, self.py - LEG_SEP * ly - sw * self.hy, LEG_R)]
            g = math.atan2(self.py - self.y, self.px - self.x) + math.pi / 2
            return [(self.px + LEG_SEP * math.cos(g), self.py + LEG_SEP * math.sin(g), LEG_R),
                    (self.px - LEG_SEP * math.cos(g), self.py - LEG_SEP * math.sin(g), LEG_R)]

        def comp_legs(self):
            if self.cxy is None:
                return []
            cx, cy = self.cxy
            ux, uy = self.cdir
            sw = a.gait * math.sin(2.0 * math.pi * 0.9 * (time.time() - self.t0) + 1.3)
            return [(cx - LEG_SEP * uy + sw * ux, cy + LEG_SEP * ux + sw * uy, LEG_R),
                    (cx + LEG_SEP * uy - sw * ux, cy - LEG_SEP * ux - sw * uy, LEG_R)]

        def intr_legs(self):
            if self.intr is None:
                return []
            x0, y0, x1, y1, ts, sp = self.intr
            L = math.hypot(x1 - x0, y1 - y0)
            f = max(0.0, min(1.0, (time.time() - self.t0 - ts) * sp / L))
            ix, iy = x0 + f * (x1 - x0), y0 + f * (y1 - y0)
            self.ixy = (ix, iy)
            ux, uy = (x1 - x0) / L, (y1 - y0) / L           # chan truoc - chan sau theo huong di
            return [(ix + 0.12 * ux, iy + 0.12 * uy, LEG_R), (ix - 0.12 * ux, iy - 0.12 * uy, LEG_R)]

        def _rect_d(self, wx, wy):
            """Khoang cach tu footprint chu nhat toi diem (mang hoac so) trong khung the gioi."""
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            lx = c * (wx - self.x) + s * (wy - self.y)
            ly = -s * (wx - self.x) + c * (wy - self.y)
            dx = np.maximum(np.maximum(-R_LEN - lx, 0.0), lx - F_LEN)
            dy = np.maximum(np.maximum(-HW - ly, 0.0), ly - HW)
            return np.hypot(dx, dy)

        def _clear(self, moving: bool) -> None:
            if len(self.C):
                self.min_obs = min(self.min_obs, float((self._rect_d(self.C[:, 0], self.C[:, 1]) - self.C[:, 2]).min()))
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            for lx, ly in ((F_LEN, HW), (F_LEN, -HW), (-R_LEN, HW), (-R_LEN, -HW)):
                wx, wy = self.x + c * lx - s * ly, self.y + s * lx + c * ly
                self.min_obs = min(self.min_obs, wx - self.room[0], self.room[1] - wx, wy - self.room[2], self.room[3] - wy)
            if moving:      # chi tinh luc XE dang chuyen dong: xe dung yen ma nguoi buoc sat lai thi khong phai loi cua xe
                for cx, cy, r in self.legs():
                    self.min_person_mv = min(self.min_person_mv, float(self._rect_d(cx, cy)) - r)
                for cx, cy, r in self.intr_legs() + self.comp_legs():
                    self.min_intr_mv = min(self.min_intr_mv, float(self._rect_d(cx, cy)) - r)

        # ── cam bien ──
        def _dead(self, after: float, back: float) -> bool:
            t = time.time() - self.t0
            return bool(after) and t > after and (not back or t < back)

        def _odom(self) -> None:
            if self._dead(a.odom_dead_after, 0.0):
                return
            o = Odometry()
            o.header.stamp = self.get_clock().now().to_msg()
            o.header.frame_id = "odom"
            o.child_frame_id = "base_link"
            o.pose.pose.position.x = self.ox_
            o.pose.pose.position.y = self.oy_
            o.pose.pose.orientation.z = math.sin(self.oyaw / 2.0)
            o.pose.pose.orientation.w = math.cos(self.oyaw / 2.0)
            self.po.publish(o)

        def _scan(self) -> None:
            if self._dead(a.lidar_dead_after, a.lidar_back_after):
                return
            # Tu the xe luc TUNG TIA duoc do (LiDAR quay 1 vong 0.1 s; ban tin phat khi xong vong)
            idx = np.arange(360)
            x_i, y_i, yaw_i = np.full(360, self.x), np.full(360, self.y), np.full(360, self.yaw_u)
            if a.skew != 0.0 and len(self.hist) >= 2:
                frac = (359 - idx) / 360.0 if a.skew > 0 else idx / 360.0
                t_i = time.time() - abs(a.skew) * 0.1 * frac
                h = np.array(self.hist)
                x_i, y_i, yaw_i = np.interp(t_i, h[:, 0], h[:, 1]), np.interp(t_i, h[:, 0], h[:, 2]), np.interp(t_i, h[:, 0], h[:, 3])
            ox = x_i + LIDAR_X * np.cos(yaw_i)
            oy = y_i + LIDAR_X * np.sin(yaw_i)
            ang = yaw_i + np.radians(idx - 90.0)                     # a_base = a_laser - 90
            dx, dy = np.cos(ang), np.sin(ang)
            rng = np.full(360, np.inf)
            xmin, xmax, ymin, ymax = self.room
            with np.errstate(divide="ignore", invalid="ignore"):
                for t in ((xmin - ox) / dx, (xmax - ox) / dx, (ymin - oy) / dy, (ymax - oy) / dy):
                    hx, hy = ox + t * dx, oy + t * dy
                    ok = (t > 0) & (hx >= xmin - 1e-6) & (hx <= xmax + 1e-6) & (hy >= ymin - 1e-6) & (hy <= ymax + 1e-6)
                    rng = np.where(ok & (t < rng), t, rng)
            hidden = (self.hide is not None and self.t_leave is not None
                      and self.hide[0] <= time.time() - self.t_leave <= self.hide[1])
            people = [] if hidden else self.legs() + self.comp_legs()
            for cx, cy, r in self.circles + people + self.intr_legs():
                bx, by = cx - ox, cy - oy
                b = bx * dx + by * dy
                disc = b * b - (bx * bx + by * by - r * r)
                ok = disc >= 0
                t = b - np.sqrt(np.where(ok, disc, 0.0))
                ok &= t > 0
                rng = np.where(ok & (t < rng), t, rng)
            rng = rng + self.rng.normal(0.0, 0.01, 360)
            rng[(rng < 0.10) | (rng > 10.0)] = np.inf
            rng[self.rng.random(360) < a.dropout] = np.inf
            rng[270:291] = 0.128                                     # than xe (cung mu 246-294)
            m = LaserScan()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "laser"
            m.angle_min = 0.0
            m.angle_increment = math.radians(1.0)
            m.angle_max = 2.0 * math.pi
            m.range_min = 0.10
            m.range_max = 10.0
            m.ranges = [float(v) for v in rng]
            self.ps.publish(m)

        def _rssi(self) -> None:
            if self._dead(a.beacon_dead_after, a.beacon_back_after):
                return
            now = time.time() - self.t0
            if self.real is not None:
                th = wrap(math.atan2(self.py - self.y, self.px - self.x) - self.yaw)
                d = math.hypot(self.px - self.x, self.py - self.y)
                for b in "ABC":
                    for v in self.real.tick(b, th, d, (self.x, self.y, self.px, self.py), 0.05):
                        self.seq[b] += 1
                        self._w(b, f"R,{b},{self.seq[b]},{int(now * 1000)},{v}\n")
                return
            for b in "ABC":
                n = int(self.rng.poisson(SR.HZ * 0.05))
                if n == 0:
                    continue
                t = np.sort(self.rng.uniform(now - 0.05, now, n))
                one = np.ones(n)
                tk, rk = SR.rx(b, t, self.x * one, self.y * one, self.yaw * one, self.px * one, self.py * one,
                               self.field, self.cfg, self.rng)
                for ti, ri in zip(tk, rk):
                    self.seq[b] += 1
                    self._w(b, f"R,{b},{self.seq[b]},{int(ti * 1000)},{int(ri)}\n")

        def _rec(self) -> None:
            self.log.write(json.dumps({
                "t": round(time.time(), 2), "x": round(self.x, 3), "y": round(self.y, 3), "yaw": round(self.yaw, 4),
                "px": round(self.px, 3), "py": round(self.py, 3), "v": round(0.5 * (self.vr + self.vl), 3),
                "w": round((self.vr - self.vl) / 0.40, 3),
                "ix": None if self.intr is None else round(getattr(self, "ixy", (0, 0))[0], 3),
                "iy": None if self.intr is None else round(getattr(self, "ixy", (0, 0))[1], 3),
                "cx": None if self.cxy is None else round(self.cxy[0], 3),
                "cy": None if self.cxy is None else round(self.cxy[1], 3)}) + "\n")
            self.log.flush()

        def finish(self) -> None:
            json.dump({"t0": self.t0, "min_obs": round(self.min_obs, 3), "min_person_mv": round(self.min_person_mv, 3),
                       "min_intr_mv": round(self.min_intr_mv, 3), "arrivals": self.arrivals, "n_way": len(self.way),
                       "n_cmd": self.n_cmd, "real_runs": self.real.used if self.real else None,
                       "d_mean": round(self.d_sum / self.d_n, 3) if self.d_n else None, "d_max": round(self.d_max, 3),
                       "far_frac": round(self.d_far / self.d_n, 3) if self.d_n else None,
                       "d": round(math.hypot(self.px - self.x, self.py - self.y), 3),
                       "ang": round(math.degrees(wrap(math.atan2(self.py - self.y, self.px - self.x) - self.yaw)), 1)},
                      open(os.path.join(a.dir, "world_summary.json"), "w"), indent=1)

    rclpy.init()
    w = World()
    try:
        rclpy.spin(w)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        w.finish()


def pubcmd_main(argv) -> None:
    """Node 'la' ghi /cmd_vel (gia lap planner bi chay nham): bat dau sau argv[0] giay, keo dai argv[1] giay."""
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) < 50:
        sys.exit("sil_rssi --pubcmd: phai dat ROS_DOMAIN_ID >= 50")
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    time.sleep(float(argv[0]))
    rclpy.init()
    n = Node("planner_gia")
    p = n.create_publisher(Twist, "/cmd_vel", 10)
    t0 = time.time()
    while time.time() - t0 < float(argv[1]):
        p.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.07)
    n.destroy_node()
    rclpy.shutdown()


# ─────────────────────────────────── bo kich ban ───────────────────────────────────
# check: toi | bam | dung (xe phai dung han, rc = 2) | khong_chay (rc = 1/2, xe khong nhuc nhich) | an_toan
ROOM = os.path.join(LOGS, "q2_m045_1.json")
P = "--room-scan " + ROOM                     # phong that; vi tri nguoi chon o cho trong cua phong do
CASES = [
    # --- nguoi dung yen, phong THAT (do dac tu vong quet LiDAR) ---
    dict(name="phongF", world=f"{P} --person 2.0,0.0 --seed 3", seek="--once", tmax=220, check="toi", room=True),
    dict(name="phongB", world=f"{P} --person=-2.5,0.25 --seed 3", seek="--once", tmax=220, check="toi", room=True),
    dict(name="phongBL", world=f"{P} --person=-2.0,1.5 --seed 4", seek="--once", tmax=220, check="toi", room=True),
    dict(name="phongBR", world=f"{P} --person=-2.0,-0.75 --seed 4", seek="--once", tmax=220, check="toi", room=True),
    # --- phong trong ---
    dict(name="truoc", world="--person 2.5,0 --seed 1", seek="--once", tmax=200, check="toi"),
    dict(name="sau", world="--person=-2.5,0.3 --seed 2", seek="--once", tmax=220, check="toi"),
    dict(name="xa4", world="--robot=-1.5,-1.0,29 --person 1.8,1.9 --seed 2", seek="--once", tmax=260, check="toi"),
    # --- bam theo: xe toi -> nguoi sang cho khac ---
    dict(name="bam", world="--person 2.2,0.5;0.5,2.6;-2.2,1.0 --seed 3", seek="", tmax=260, check="bam"),
    dict(name="phongbam", world=f"{P} --person 2.0,0.0;-2.5,0.25;-2.0,1.5 --seed 4", seek="", tmax=260, check="bam", room=True),
    # --- su co: xe phai dung ---
    dict(name="lidar_chet", world="--person 2.5,0 --seed 1 --lidar-dead-after 21", seek="--once", tmax=120, check="dung",
         fault=21.0, max_cm=25.0),
    dict(name="odom_chet", world="--person 2.5,0 --seed 1 --odom-dead-after 21", seek="--once", tmax=120, check="dung",
         fault=21.0, max_cm=25.0),
    dict(name="beacon_mat", world="--person 2.5,0 --seed 1 --beacon-dead-after 21 --beacon-back-after 31", seek="--once",
         tmax=200, check="toi"),
    dict(name="kill9", world="--person 2.5,0 --seed 1", seek="--once", tmax=100, check="kill", sig=(signal.SIGKILL, 12.0),
         max_cm=30.0),
    dict(name="pub_la", world="--person 2.5,0 --seed 1", seek="--once", tmax=60, check="khong_chay", pubcmd=(0.0, 60.0)),
    dict(name="chan_xoay", world="--person 0.48,0.1 --seed 1", seek="--once", tmax=120, check="khong_chay"),
    # --- nguoi thu hai di cat ngang truoc mui luc xe dang chay ---
    dict(name="nguoi_cat", world="--person 2.5,0 --seed 1 --intruder=0.9,1.5,0.9,-1.5,18,1.0", seek="", tmax=120,
         check="an_toan"),
]


# Che do BAM (rssi_follow.py + planner that): nguoi di 0.4 m/s giua cac cho dung; xe chay toi da 0.22 m/s
FW = "--walk 0.4 --dwell 4 --impatient 60"     # xe 60 s khong toi thi nguoi tu di tiep (nguoi that khong dung cho mai)
EN = "--enter 12"                # nguoi dung canh xe, giay 12 moi di ra vi tri dau; script dem nguoc 9 s (--delay 9)
FOLLOW_CASES = [
    dict(name="b_dung", mode="follow", world="--person 2.5,0.3 --seed 1", seek="", tmax=70, check="theo"),
    dict(name="b_3cho", mode="follow", world=f"--person 2.2,0.5;0.5,2.6;-2.2,1.0 --seed 3 {FW} {EN}", seek="--delay 9", tmax=200,
         check="theo"),
    dict(name="b_phong3", mode="follow", world=f"{P} --person 2.0,0.0;-2.5,0.25;-2.0,1.5 --seed 4 {FW} {EN}", seek="--delay 9",
         tmax=230, check="theo", room=True),
    dict(name="b_phong3b", mode="follow", world=f"{P} --person=-2.5,0.25;1.8,-0.2;-2.0,-0.75 --seed 5 {FW} {EN}", seek="--delay 9",
         tmax=230, check="theo", room=True),
    # nhu tren nhung KHONG co thong tin "vat moi" luc dau (nguoi da dung san): phai tu sua bang kiem tra RSSI
    dict(name="b_phong3c", mode="follow", world=f"{P} --person 2.0,0.0;-2.5,0.25;-2.0,1.5 --seed 4 {FW}", seek="", tmax=260,
         check="theo", room=True, max_wrong=45.0, miss=1),
    # thung chan lech tren duong di (van thay nguoi): planner phai VONG QUA — rssi_seek.py thi dung lai o thung
    dict(name="b_vong", mode="follow", world="--person 3.2,0.0 --circles 1.6,0.30,0.20 --seed 1", seek="", tmax=90, check="theo"),
    # thung nam DUNG giua, che kin chan nguoi: khoa nham thung la GIOI HAN da biet (RSSI khong do duoc khoang cach)
    dict(name="b_che_kin", mode="follow", world="--person 3.2,0.0 --circles 1.6,0.05,0.20 --seed 1", seek="", tmax=60,
         check="an_toan"),
    # nguoi di cham LIEN TUC quanh phong (bat dau di sau khi xe da khoa)
    dict(name="b_lientuc", mode="follow", world="--person 2.0,0.0;2.0,2.2;-2.0,2.2;-2.0,-2.0;2.0,-2.0 --walk-always 0.18 "
         f"--walk-start 36 --seed 2 {EN}", seek="--delay 9", tmax=190, check="lien_tuc"),
    # nguoi THU HAI (khong deo beacon) di cat ngang giua xe va chu luc dang bam
    dict(name="b_cat_ngang", mode="follow", world="--person 3.4,0.0 --seed 1 --intruder=2.2,1.6,2.2,-1.6,22,0.9", seek="",
         tmax=80, check="theo"),
    # nguoi thu hai DUNG YEN cach duong di cua chu 0.65 m (chu di ra tu canh xe nhu quy trinh that)
    dict(name="b_dung_canh", mode="follow", world=f"--person 2.4,0.0;0.3,2.4 --circles 1.75,1.63,0.055;1.93,1.63,0.055 --seed 2 {FW} {EN}",
         seek="--delay 9", tmax=170, check="theo"),
    # chu di SAT vai (0.45 m) qua nguoi thu hai dang dung yen
    dict(name="b_di_sat", mode="follow", world=f"--person 2.4,0.0;0.3,2.4 --circles 1.60,1.50,0.055;1.78,1.50,0.055 --seed 2 {FW} {EN}",
         seek="--delay 9", tmax=190, check="theo", max_wrong=8.0),
    # chu va nguoi thu hai cung DUNG SAN tu dau, cach nhau ~32 do nhin tu xe (khong co bang chung "vat moi"): RSSI sai
    # ~15-25 do nen lan khoa dau co the nham nguoi thu hai — GIOI HAN da biet; phai TU SUA khi toi gan kiem tra lai.
    # Duoc phep lo cho dau (sua mat ~30-50 s, chu het kien nhan 60 s di tiep — nhu r_phongc_*), cuoi cung phai bam dung chu.
    dict(name="b_hai_san", mode="follow", world=f"--person 2.4,0.0;0.3,2.4 --circles 1.75,1.63,0.055;1.93,1.63,0.055 --seed 2 {FW}",
         seek="", tmax=170, check="theo", max_wrong=25.0, miss=1),
    # chu di khuat sau vach (LiDAR mat chan) -> mat dau -> toi cho thay cuoi -> RSSI do lai
    dict(name="b_khuat", mode="follow", world="--person 2.2,0.0;2.4,-2.6 --circles " + ";".join(
        f"{1.0 + 0.12 * i:.2f},-1.3,0.07" for i in range(14)) + f" --seed 3 {FW}", seek="", tmax=200, check="an_toan"),
    # dung khan /follow/stop: luc dang bam (planner lai) va luc script dang TU XOAY do (planner da tat — truoc 03/10 xe chay
    # tiep 2 m vi /follow/stop la cua planner; nay rssi_follow.launch.py doi ten service do, script nhan /follow/stop)
    dict(name="b_stop", mode="follow", world="--person 3.5,0.0 --seed 1", seek="", tmax=60, check="dung_khan", stop_at=16.0,
         rc=130),
    dict(name="b_stop_do", mode="follow", world="--person 3.5,0.0 --seed 1", seek="", tmax=60, check="dung_khan", stop_at=3.0,
         rc=130),
    dict(name="b_stop_rf", mode="follow", world="--person 3.5,0.0 --seed 1", seek="", tmax=60, check="dung_khan", stop_at=3.0,
         stop_srv="/rssi_follow/stop", rc=130),
    dict(name="b_lidar_chet", mode="follow", world="--person 3.5,0.0 --seed 1 --lidar-dead-after 24", seek="", tmax=90,
         check="dung", fault=24.0, max_cm=25.0),
    # --- 06/10, sau lan chay that dau tien: cac tinh huong da gap tren xe ---
    # chu di lai SAT xe roi vong doc hong xe ra phia sau (chay that: mat dau, 34 s moi khoa lai)
    dict(name="b_sat_hong", mode="follow", world=f"--person 2.2,0.2;-1.6,0.6;1.2,2.2 --pass-r 0.8 --walk 0.6 --dwell 4 "
         f"--impatient 60 --seed 3 {EN}", seek="--delay 9", tmax=200, check="theo"),
    # chu di sang cho khac TRONG LUC xe dang xoay do lan dau (chay that: 3 lan phai do lai, ton them 24 s). Chu tu roi
    # diem dau giua luc xe xoay nen lan "toi diem dau" khong bao gio co -> miss=1
    dict(name="b_di_luc_do", mode="follow", world="--person 2.2,1.2;2.4,-0.5;0.4,2.4 --move-on-spin 60 --walk 0.6 --dwell 4 "
         "--impatient 60 --seed 2", seek="", tmax=170, check="theo", miss=1),
    # chu dang di thi bi che kin 2.5 s (khuat sau vat) -> mat dau; hien ra van dang di, gan cho du doan, khong co ai khac
    # -> phai KHOA LAI NGAY, khong xoay do
    dict(name="b_an_mot", mode="follow", world=f"--person 2.4,0.0;2.4,-2.4;0.6,-2.4 --hide-walk 1.0,3.5 {FW} --seed 3 {EN}",
         seek="--delay 9", tmax=200, check="theo", must=["KHOA LAI NHANH"]),
    # nhu tren nhung co nguoi thu hai DI CUNG chu (lech 0.6 m): hien ra la HAI nguoi dang di gan cho du doan -> KHONG duoc
    # khoa nhanh (LiDAR khong biet ai la chu) -> xoay do bang RSSI nhu cu
    dict(name="b_an_hai", mode="follow", world=f"--person 2.4,0.0;2.4,-2.4;0.6,-2.4 --hide-walk 1.0,3.5 --companion 0.6,0.0 "
         f"{FW} --seed 3 {EN}", seek="--delay 9", tmax=220, check="theo", must_not=["KHOA LAI NHANH"]),
]
CASES += FOLLOW_CASES

# Bo MO RONG cho che do BAM (python3 sil_rssi.py --more): hat ngau nhien khac, thu tu cho dung khac, phong thu hai
# (cung can phong, xe dung cho khac), vong qua thung, va "khoa nham thung roi chu buoc sang ben".
ROOM2 = os.path.join(LOGS, "q2_p135_1.json")
P2 = "--room-scan " + ROOM2
T3 = "--person 2.2,0.5;0.5,2.6;-2.2,1.0"
LT = "--person 2.0,0.0;2.0,2.2;-2.0,2.2;-2.0,-2.0;2.0,-2.0"


def _f(name, world, enter=True, tmax=230, check="theo", **kw):
    return dict(name=name, mode="follow", world=world + (f" {EN}" if enter else ""), seek="--delay 9" if enter else "",
                tmax=tmax, check=check, **kw)


MORE_CASES = [
    _f("r_phong3_s6", f"{P} --person 2.0,0.0;-2.5,0.25;-2.0,1.5 --seed 6 {FW}", room=True),
    _f("r_phong3_s7", f"{P} --person 2.0,0.0;-2.5,0.25;-2.0,-0.75 --seed 7 {FW}", room=True),
    _f("r_phong3b_s8", f"{P} --person=-2.5,0.25;1.8,-0.2;-2.0,-0.75 --seed 8 {FW}", room=True),
    # nguoi DUNG SAN tu dau (khong co bang chung "vat moi"): duoc phep lo cho dau, phai tu sua khi nguoi di chuyen
    _f("r_phongc_s9", f"{P} --person=-2.0,1.5;2.0,0.0;-2.5,0.25 --seed 9 {FW}", enter=False, tmax=250, room=True,
       max_wrong=45.0, miss=1),
    _f("r_phongc_s10", f"{P} --person=-2.0,-0.75;-2.5,0.25;2.0,0.0 --seed 10 {FW}", enter=False, tmax=250, room=True,
       max_wrong=45.0, miss=1),
    _f("r_p2_s1", f"{P2} --person 1.5,-0.25;-2.5,0.25;-1.5,-0.5 --seed 11 {FW}", room2=True),
    _f("r_p2_s2", f"{P2} --person=-2.5,0.25;1.5,-0.25;-1.5,-0.5 --seed 12 {FW}", room2=True),
    _f("r_p2_s3", f"{P2} --person=-1.5,-0.5;1.5,-0.25;-2.5,0.25 --seed 13 {FW}", room2=True),
    _f("r_p2_c4", f"{P2} --person 1.5,-0.25;-2.5,0.25;-1.5,-0.5 --seed 14 {FW}", enter=False, tmax=250, room2=True,
       max_wrong=45.0, miss=1),
    _f("r_3cho_s13", f"{T3} --seed 13 {FW}", tmax=200),
    _f("r_3cho_nhanh", f"{T3} --seed 14 --walk 0.8 --dwell 4", tmax=200),
    _f("r_lientuc_s3", f"{LT} --walk-always 0.20 --walk-start 36 --seed 3", tmax=190, check="lien_tuc"),
    _f("r_lientuc_phong", f"{P} --person 2.0,0.0;-1.0,-0.3;-2.5,0.25;-1.0,-0.3;2.0,0.0 --walk-always 0.18 --walk-start 36 --seed 4",
       tmax=190, check="lien_tuc", room=True),
    # nguoi thu hai di NHANH (1.2 m/s) cat sat truoc mui: planner chi kip phanh (gioi han cua planner, CLAUDE.md 13.22)
    _f("r_cat2", "--person 3.4,0.0 --seed 5 --intruder=1.9,-1.6,1.9,1.6,24,1.2", enter=False, tmax=80),
    # thung lech tren duong toi nguoi: planner phai vong qua (nguong xoay-tai-cho 60 do trong rssi_follow.launch.py)
    _f("ve_s1", "--person 3.2,0.0 --circles 1.6,0.30,0.20 --seed 1", tmax=100),
    _f("ve_s2", "--person 3.2,0.0 --circles 1.6,0.30,0.20 --seed 2", tmax=100),
    _f("ve_s3", "--person 3.2,0.0 --circles 1.6,0.30,0.20 --seed 3", tmax=100),
    _f("ve_s4", "--person 3.2,0.0 --circles 1.6,-0.30,0.20 --seed 4", tmax=100),
    _f("ve_s6", "--person 3.2,0.0 --circles 1.4,0.35,0.15;2.2,-0.45,0.15 --seed 6", tmax=100),
    # thung TO che kin chan nguoi: khoa nham thung la gioi han da biet -> chi xet an toan
    _f("ve_s5", "--person 3.2,0.0 --circles 1.5,0.15,0.25 --seed 5", tmax=100, check="an_toan"),
    # khoa nham thung (nguoi dung san), roi chu buoc sang ben ~1 m: phai nhan ra nguoi MOI di toi va chuyen sang
    *[_f(f"x_doi_s{k}", f"--person 3.2,0.0;2.4,-0.6 --circles 1.6,0.30,0.20 --seed {k} --walk 0.4 --dwell 4 --impatient 35",
         enter=False, tmax=150, max_wrong=90.0, miss=1) for k in (2, 3, 7, 8)],
]


WORLD_EXTRA: list = []          # --world-extra (ap cho moi kich ban)
FOLLOW_SCRIPT = [os.path.join(HERE, "rssi_follow.py")]   # --follow-script (so ban cu / ban moi tren cung kich ban)


def run_case(c: dict, dom: int, out: str, rssi: str) -> dict:
    d = os.path.join(out, c["name"])
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    env = dict(os.environ, ROS_DOMAIN_ID=str(dom))
    me = [sys.executable, os.path.abspath(__file__)]
    procs = []

    def spawn(cmd, log, **kw):
        p = subprocess.Popen(cmd, env=env, stdout=open(os.path.join(d, log), "w"), stderr=subprocess.STDOUT, **kw)
        procs.append(p)
        return p

    follow = c.get("mode") == "follow"
    wp = spawn(me + ["--world", "--dir", d, "--rssi", rssi] + c["world"].split() + WORLD_EXTRA, "world.log")
    time.sleep(2.5)
    if follow:      # planner THAT (ghi /cmd_vel_follow) + 2 node RSSI; lidar/driver la the gioi gia
        lcmd = ["ros2", "launch", "person_follow_nav", "rssi_follow.launch.py", "start_lidar:=false", "start_driver:=false"]
    else:
        lcmd = ["ros2", "launch", "person_follow_nav", "rssi.launch.py"]
    lp = spawn(lcmd + [f"ports:={d}/p_A,{d}/p_B,{d}/p_C", f"template_file:={d}/tmpl.json"] + c.get("launch", "").split(),
               "nodes.log", start_new_session=True)
    time.sleep(5.0 if follow else 4.0)
    if c.get("pubcmd"):
        spawn(me + ["--pubcmd", str(c["pubcmd"][0]), str(c["pubcmd"][1])], "pub.log")
    sp = spawn([sys.executable, FOLLOW_SCRIPT[0] if follow else os.path.join(HERE, "rssi_seek.py"), "--delay", "0",
                "--log", os.path.join(d, "seek.jsonl")] + c["seek"].split(), "seek.log")
    t_sig = None
    try:
        if c.get("stop_at"):        # nguoi dung goi dung khan giua chung (/follow/stop cua planner, hoac service cua script)
            try:
                sp.wait(timeout=c["stop_at"])
            except subprocess.TimeoutExpired:
                subprocess.run(["ros2", "service", "call", c.get("stop_srv", "/follow/stop"), "std_srvs/srv/Trigger", "{}"],
                               env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
                t_sig = time.time()
        if c.get("sig"):
            try:
                sp.wait(timeout=c["sig"][1])
            except subprocess.TimeoutExpired:
                sp.send_signal(c["sig"][0])
                t_sig = time.time()
        sp.wait(timeout=c["tmax"])
    except subprocess.TimeoutExpired:
        sp.send_signal(signal.SIGINT)
        try:
            sp.wait(timeout=10)
        except subprocess.TimeoutExpired:
            sp.kill()
    rc = sp.returncode
    time.sleep(2.0)                              # de the gioi ghi nhan xe da dung han
    wp.send_signal(signal.SIGINT)
    try:
        os.killpg(lp.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    time.sleep(1.5)
    for p in procs:
        if p.poll() is None:
            try:
                (os.killpg(p.pid, signal.SIGTERM) if p is lp else p.terminate())
            except ProcessLookupError:
                pass
    for p in procs:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    return {"name": c["name"], "rc": rc, "dir": d, "t_sig": t_sig}


def _jl(path: str):
    return [json.loads(x) for x in open(path)] if os.path.exists(path) else []


def odom_to_world(e: dict, w: dict, p):
    """Diem p (khung odom cua script, ban ghi e co tu the odom x/y/yaw) -> khung the gioi (ban ghi w: tu the that)."""
    c, s_ = math.cos(-e["yaw"]), math.sin(-e["yaw"])
    dx, dy = p[0] - e["x"], p[1] - e["y"]
    bx, by = c * dx - s_ * dy, s_ * dx + c * dy
    return (w["x"] + math.cos(w["yaw"]) * bx - math.sin(w["yaw"]) * by,
            w["y"] + math.sin(w["yaw"]) * bx + math.cos(w["yaw"]) * by)


def _travel(world, t_from: float):
    """(quang duong m, goc xoay do) xe con di tu thoi diem t_from."""
    dist = ang = 0.0
    prev = None
    for r in world:
        if r["t"] >= t_from and prev is not None:
            dist += math.hypot(r["x"] - prev["x"], r["y"] - prev["y"])
            ang += abs(wrap(r["yaw"] - prev["yaw"]))
        prev = r
    return dist, math.degrees(ang)


def judge(c: dict, r: dict):
    """-> (dat?, dong mo ta, danh sach sai so huong)."""
    world, seek = _jl(os.path.join(r["dir"], "world.jsonl")), _jl(os.path.join(r["dir"], "seek.jsonl"))
    sp = os.path.join(r["dir"], "world_summary.json")
    if not world or not os.path.exists(sp):
        return False, "khong co nhat ky the gioi (ROS da source chua? xem world.log)", []
    s = json.load(open(sp))
    wt = np.array([w["t"] for w in world])
    errs = []
    for e in seek:
        if "scan_ok" in e:      # so trong khung THAN XE: odom dem thieu goc nen khung odom lech dan khoi khung the gioi
            w = world[int(np.argmin(np.abs(wt - e["t"])))]
            errs.append(math.degrees(wrap((e["scan_ok"]["bearing_odom_rad"] - e["yaw"])
                                          - (math.atan2(w["py"] - w["y"], w["px"] - w["x"]) - w["yaw"]))))
    t_start = next((e["t"] for e in seek if "args" in e), None)
    t_arr = next((e["t"] for e in seek if "arrived" in e), None)
    n_cyc = sum(1 for e in seek if e.get("msg", "").startswith("--- chu ky"))
    safe = s["min_obs"] > 0.05 and s["min_person_mv"] > 0.05 and s["min_intr_mv"] > 0.05
    txt = (f"rc {r['rc']}, {n_cyc} chu ky, ho vat {s['min_obs']:.2f} m, ho nguoi (xe dang chay) "
           f"{min(s['min_person_mv'], s['min_intr_mv']):.2f} m")
    k = c["check"]
    if c.get("mode") == "follow":
        # Vet 2 Hz cua rssi_follow.py: muc tieu dang bam (odom = khung the gioi) so voi vi tri that cua nguoi
        n_bam = n_wrong = 0
        for e in seek:
            if e.get("st") == "BAM" and "trk" in e:
                w = world[int(np.argmin(np.abs(wt - e["t"])))]
                n_bam += 1
                tx, ty = odom_to_world(e, w, e["trk"])
                n_wrong += math.hypot(tx - w["px"], ty - w["py"]) > 0.6
        t_end = seek[-1]["t"] if seek else 0.0
        n_do = sum(1 for e in seek if e.get("msg", "").startswith("--- lan do"))
        n_lost = sum(1 for e in seek if e.get("lost"))
        t_lock = next((e["t"] for e in seek if "lock" in e), None)
        cover = (n_bam * 0.5) / max(1.0, t_end - t_lock) if t_lock else 0.0
        txt = (f"{n_do} lan do, mat dau {n_lost} lan, bam {cover * 100:.0f}% thoi gian sau khi khoa, bam NHAM {n_wrong * 0.5:.1f} s, "
               f"cach nguoi TB {s['d_mean']} m (xa nhat {s['d_max']} m), ho vat {s['min_obs']:.2f} m, ho nguoi (xe dang chay) "
               f"{min(s['min_person_mv'], s['min_intr_mv']):.2f} m, rc {r['rc']}")
        if k == "theo":             # phai toi duoc tung cho nguoi dung va khong bam nham
            # miss: so cho duoc phep bo lo (kich ban nguoi DUNG SAN tu dau — khong co bang chung "vat moi" — co the khoa nham
            # do dac o cho dau tien cho toi khi nguoi di chuyen: gioi han da biet)
            ok = (len(s["arrivals"]) >= s["n_way"] - 1 - c.get("miss", 0) and s["d"] <= 1.5 and safe
                  and n_wrong * 0.5 <= c.get("max_wrong", 2.0) and t_lock is not None)
            txt = (f"toi {len(s['arrivals']) + (1 if s['d'] <= 1.5 else 0)}/{s['n_way']} cho ("
                   + ", ".join(f"{x['t']:.0f} s" for x in s["arrivals"]) + f"), cuoi cach nguoi {s['d']:.2f} m; " + txt)
        elif k == "lien_tuc":       # nguoi di lien tuc: xe phai giu duoc khoang cach
            ok = (safe and t_lock is not None and s["d_mean"] is not None and s["d_mean"] <= c.get("max_mean", 1.8)
                  and (s["far_frac"] or 0.0) <= 0.15 and n_wrong * 0.5 <= c.get("max_wrong", 2.0))
        elif k == "dung_khan":      # goi dung khan giua chung: xe dung, script tu thoat
            dist, ang = _travel(world, (r["t_sig"] or 0.0) + 0.6)
            ok = (r["rc"] == c.get("rc", 0) and dist < 0.08 and ang < 10.0 and abs(world[-1]["v"]) < 0.01
                  and abs(world[-1]["w"]) < 0.02 and safe)
            txt = f"{c.get('stop_srv', '/follow/stop')}: sau 0.6 s xe con di {dist * 100:.0f} cm, xoay {ang:.0f} do; " + txt
        elif k == "dung":
            dist, _ = _travel(world, s["t0"] + c["fault"])
            ok = r["rc"] == 2 and dist * 100.0 <= c["max_cm"] and abs(world[-1]["v"]) < 0.01 and safe
            txt = f"xe di them {dist * 100:.0f} cm sau khi cam bien chet (cho phep {c['max_cm']:.0f}); " + txt
        else:
            ok = safe
        # su kien bat buoc phai co / khong duoc co trong nhat ky cua script (vd. "KHOA LAI NHANH")
        msgs = [e.get("msg", "") for e in seek]
        for m in c.get("must", []):
            if not any(m in x for x in msgs):
                ok, txt = False, txt + f"; THIEU su kien '{m}'"
        for m in c.get("must_not", []):
            if any(m in x for x in msgs):
                ok, txt = False, txt + f"; CO su kien '{m}' (khong duoc co)"
        return ok, txt, errs
    if k == "toi":
        ok = r["rc"] == 0 and s["d"] <= 1.3 and safe
        txt = (f"toi sau {t_arr - t_start:.0f} s, " if t_arr and t_start else "KHONG toi, ") + f"cach nguoi {s['d']:.2f} m, " + txt
    elif k == "bam":
        ok = len(s["arrivals"]) >= s["n_way"] - 1 and s["d"] <= 1.3 and safe
        txt = (f"toi {len(s['arrivals']) + (1 if s['d'] <= 1.3 else 0)}/{s['n_way']} cho ("
               + ", ".join(f"{x['t']:.0f} s" for x in s["arrivals"]) + f"), cuoi cach nguoi {s['d']:.2f} m, " + txt)
    elif k == "dung":
        dist, _ = _travel(world, s["t0"] + c["fault"])
        ok = r["rc"] == 2 and dist * 100.0 <= c["max_cm"] and abs(world[-1]["v"]) < 0.01 and safe
        txt = f"xe di them {dist * 100:.0f} cm sau khi cam bien chet (cho phep {c['max_cm']:.0f}), " + txt
    elif k == "kill":
        dist, _ = _travel(world, r["t_sig"] or 0.0)
        ok = dist * 100.0 <= c["max_cm"] and abs(world[-1]["v"]) < 0.01 and abs(world[-1]["w"]) < 0.02
        txt = f"script bi kill -9: xe di them {dist * 100:.0f} cm roi driver tu dung (cho phep {c['max_cm']:.0f}), " + txt
    elif k == "khong_chay":
        dist, ang = _travel(world, 0.0)
        ok = r["rc"] in (1, 2) and dist < 0.01 and ang < 1.0
        txt = f"xe di {dist * 100:.0f} cm, xoay {ang:.0f} do (phai = 0), " + txt
    else:                                        # an_toan
        ok = safe
    return ok, txt, errs


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "--world":
        world_main(sys.argv[2:])
        return 0
    if len(sys.argv) > 1 and sys.argv[1] == "--pubcmd":
        pubcmd_main(sys.argv[2:])
        return 0
    ap = argparse.ArgumentParser(description="Mo phong vong kin co ROS: 2 node RSSI + rssi_seek.py voi the gioi gia")
    ap.add_argument("names", nargs="*", help="ten kich ban (mac dinh: tat ca)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--more", action="store_true", help="chay bo MO RONG (24 bien the che do bam) thay cho bo chinh")
    ap.add_argument("--world-extra", default="", help="them tham so cho the gioi o MOI kich ban, vd. \"--skew -1\" (LiDAR quay "
                    "chieu nguoc), \"--odom-yaw-scale 1.0\" (odom khong troi)")
    ap.add_argument("--rssi", default="real", choices=("real", "sim"))
    ap.add_argument("--follow-script", default=FOLLOW_SCRIPT[0],
                    help="ban rssi_follow.py dung cho che do bam (mac dinh: ban trong thu muc nay) — de so ban cu / ban moi")
    ap.add_argument("--domain", type=int, default=60, help="ROS_DOMAIN_ID dau tien (moi kich ban mot mien)")
    ap.add_argument("--keep", default="", help="thu muc giu nhat ky (mac dinh: thu muc tam, xoa sau khi cham)")
    ap.add_argument("--jobs", type=int, default=14, help="so kich ban chay song song moi dot (moi cai ~0.5 nhan CPU)")
    a = ap.parse_args()
    if a.list:
        for c in CASES + MORE_CASES:
            print(f"  {c['name']:15s} [{c['check']}]  {c['world']}")
        return 0
    if shutil.which("ros2") is None:
        print("Chua source ROS (khong thay lenh ros2).")
        return 1
    if not 50 <= a.domain <= 101 - min(a.jobs, 50) + 1:
        # Linux: ROS_DOMAIN_ID > 101 roi vao dai cong tam cua he dieu hanh -> DDS tim nhau chap chon (thu 03/10: mien
        # 120+ mat /odom, goi service qua 15 s). Mien < 50 de danh cho xe that.
        print(f"--domain {a.domain}: can 50 <= mien dau va mien cuoi (+ --jobs - 1) <= 101.")
        return 1
    WORLD_EXTRA[:] = a.world_extra.split()
    FOLLOW_SCRIPT[0] = os.path.abspath(a.follow_script)
    rssi = a.rssi
    if rssi == "real" and not glob.glob(os.path.join(LOGS, "q2_[pm]*.csv")):
        print(f"Khong co mau do that trong {LOGS} -> dung mo hinh (--rssi sim).")
        rssi = "sim"
    if a.names:
        cases = [c for c in CASES + MORE_CASES if c["name"] in a.names]
    else:
        cases = MORE_CASES if a.more else CASES
    for key, path in (("room", ROOM), ("room2", ROOM2)):
        if not os.path.exists(path):
            skipped = [c["name"] for c in cases if c.get(key)]
            cases = [c for c in cases if not c.get(key)]
            if skipped:
                print(f"Bo qua {skipped}: khong co vong quet phong that {path}")
    if not cases:
        print("Khong co kich ban nao.")
        return 1
    import tempfile
    out = a.keep or tempfile.mkdtemp(prefix="sil_rssi_")
    os.makedirs(out, exist_ok=True)
    jobs = max(1, a.jobs)
    waves = [cases[i:i + jobs] for i in range(0, len(cases), jobs)]
    print(f"Chay {len(cases)} kich ban, moi dot {jobs} cai song song ({len(waves)} dot; mien ROS {a.domain}..; RSSI: "
          f"{'mau do THAT' if rssi == 'real' else 'mo hinh'}), ~{sum(max(c['tmax'] for c in w) + 25 for w in waves)} s ...", flush=True)
    res = {}

    def work(i: int, c: dict) -> None:
        res[c["name"]] = run_case(c, a.domain + i, out, rssi)
        print(f"   xong {c['name']}", flush=True)
    for wave in waves:
        th = [threading.Thread(target=work, args=(i, c)) for i, c in enumerate(wave)]
        for t in th:
            t.start()
            time.sleep(0.7)
        for t in th:
            t.join()
    print()
    all_ok, all_err = True, []
    for c in cases:
        ok, txt, errs = judge(c, res[c["name"]])
        all_ok &= ok
        all_err += errs
        print(f"  {c['name']:15s} {'DAT' if ok else 'LOI'}  {txt}")
        if errs:
            print(f"  {'':15s}      sai so huong tung lan do: " + " ".join(f"{e:+.0f}" for e in errs))
    if all_err:
        e = np.abs(all_err)
        print(f"\n{len(e)} lan do huong: sai so TB {e.mean():.1f} do, lon nhat {e.max():.0f} do, trong +-30 do: {np.mean(e <= 30) * 100:.0f}%")
    print("TONG KET: " + ("TAT CA DAT" if all_ok else "CO KICH BAN LOI"))
    if a.keep:
        print(f"Nhat ky: {out}")
    else:
        shutil.rmtree(out, ignore_errors=True)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
