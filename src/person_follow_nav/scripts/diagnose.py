#!/usr/bin/env python3
"""
diagnose.py — chan doan khi xe khong ne vat can.

Phan biet HAI nguyen nhan hoan toan khac nhau:

  A. LIDAR KHONG QUAY (dau quet dung im)
     -> /scan co the van publish nhung du lieu dong bang hoac rac
     -> planner khong thay gi, hoac thay sai

  B. LIDAR QUAY BINH THUONG nhung planner khong dung du lieu
     -> bo loc than xe bo qua nhieu, sai tham so hinh hoc, hoac
        planner khong nhan duoc /follow/target

CACH DUNG
---------
  Chay khi test_avoid_only.launch.py DANG chay, va dat mot vat can
  cach mui xe khoang 1m:

      python3 diagnose.py

  Xe khong can chay. Khong goi /follow/enable truoc khi chan doan xong.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from person_follow_nav.geometry import (  # noqa: E402
    rect_clearance, scan_to_base_points, self_filter_mask,
)

G, Y, R, B, X = "\033[92m", "\033[93m", "\033[91m", "\033[1m", "\033[0m"

# Phai trung config/follow_nav.yaml
YAW_OFF = -90.0
SIGN = 1.0
LIDAR_X = 0.10
FRONT, REAR, HW = 0.14, 0.33, 0.30
MARGIN_HARD = 0.06
BLIND = [[246.0, 294.0]]
SELF_MARGIN = 0.03


class Diag(Node):

    def __init__(self) -> None:
        super().__init__("diagnose")
        self.scans: List[np.ndarray] = []
        self.msg: Optional[LaserScan] = None
        self.status: Optional[str] = None
        self.target: Optional[str] = None
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(String, "/follow/planner_status", self._st, 10)
        self.create_subscription(String, "/follow/target", self._tg, 10)

    def _scan(self, m: LaserScan) -> None:
        self.msg = m
        self.scans.append(np.asarray(m.ranges, dtype=np.float64))

    def _st(self, m: String) -> None:
        self.status = m.data

    def _tg(self, m: String) -> None:
        self.target = m.data


def main() -> None:
    rclpy.init()
    n = Diag()
    print()
    print("=" * 70)
    print(f"{B}  CHAN DOAN — dat mot vat can cach mui xe ~1m{X}")
    print("=" * 70)
    print("  Dang thu thap 4 giay...")
    t0 = time.time()
    while time.time() - t0 < 4.0:
        rclpy.spin_once(n, timeout_sec=0.1)

    verdict = []

    # ── 1. /scan co du lieu khong ────────────────────────────────────
    print(f"\n{B}1. /scan{X}")
    if not n.scans:
        print(f"  {R}KHONG CO du lieu tren /scan.{X}")
        print("  -> sc_mini chua chay, hoac cong USB bi node khac giu.")
        print("  -> Kiem tra: ros2 node list | grep sc_mini")
        print("               (neu thay 2 node sc_mini => dang chay 2 launch file)")
        rclpy.shutdown()
        return
    hz = len(n.scans) / 4.0
    print(f"  Nhan {len(n.scans)} vong quet trong 4s = {hz:.1f} Hz", end="")
    print(f"  {G}(binh thuong ~10 Hz){X}" if hz >= 6 else f"  {R}(QUA THAP){X}")

    A = np.vstack(n.scans)
    finite = np.isfinite(A) & (A > 0.05) & (A < 20.0)
    frac = finite.mean()
    print(f"  Ti le tia hop le: {frac*100:.0f}%", end="")
    if frac < 0.15:
        print(f"  {R}<== RAT THAP{X}")
        verdict.append(("A", "Hau het tia vo hieu — dau quet nhieu kha nang KHONG QUAY."))
    else:
        print(f"  {G}(tot){X}")

    # ── 2. Lidar co THAT SU quay khong ───────────────────────────────
    print(f"\n{B}2. Dau quet co dang quay khong{X}")
    if A.shape[0] >= 5:
        d = np.abs(np.diff(A, axis=0))
        d = d[np.isfinite(d)]
        move = float(np.mean(d)) if d.size else 0.0
        changed = float(np.mean(d > 0.01)) if d.size else 0.0
        print(f"  Thay doi trung binh giua 2 vong quet: {move*100:.1f} cm")
        print(f"  Ti le tia co thay doi:                {changed*100:.0f}%")
        if changed < 0.02:
            print(f"  {R}=> DU LIEU DONG BANG. Dau quet KHONG QUAY.{X}")
            verdict.append(("A", "Cac vong quet giong het nhau — dau quet dung im."))
        else:
            print(f"  {G}=> Du lieu thay doi binh thuong, dau quet dang quay.{X}")

    # ── 3. Planner nhin thay gi ──────────────────────────────────────
    print(f"\n{B}3. Sau khi doi sang base_link + loc than xe{X}")
    m = n.msg
    pts, bear, rng, ldeg = scan_to_base_points(
        np.asarray(m.ranges, dtype=np.float64), float(m.angle_min),
        float(m.angle_increment), float(m.range_min), float(m.range_max),
        LIDAR_X, 0.0, math.radians(YAW_OFF), SIGN, 5.0)
    print(f"  Diem thu duoc trong tam 5m:       {pts.shape[0]}")
    if pts.shape[0]:
        keep = self_filter_mask(pts, ldeg, FRONT, REAR, HW, SELF_MARGIN, BLIND)
        p2 = pts[keep]
        print(f"  Bo loc than xe bo:                {int((~keep).sum())}"
              f"  {G}(binh thuong ~20-50, tuy sau xe co vat trong 5m){X}")
        print(f"  Con lai:                          {p2.shape[0]}")
        if p2.shape[0] == 0:
            print(f"  {R}=> BO LOC BO SACH. Sai tham so hinh hoc.{X}")
            verdict.append(("B", "self_filter bo het diem — kiem tra blind_sectors_deg."))
        else:
            b2 = np.degrees(np.arctan2(p2[:, 1], p2[:, 0]))
            d2 = np.hypot(p2[:, 0], p2[:, 1])
            fr = np.abs(b2) < 25.0
            if np.any(fr):
                print(f"  Vat can gan nhat TRUOC MAT (±25°): "
                      f"{d2[fr].min():.2f} m tai goc {b2[fr][np.argmin(d2[fr])]:+.0f}°")
            else:
                print(f"  {Y}Khong co diem nao trong cung ±25° truoc mat.{X}")
                verdict.append(("B", "Khong thay vat can truoc mat du ban da dat."))
            c = rect_clearance(p2[:, 0], p2[:, 1], FRONT, REAR, HW)
            print(f"  Do thoang nho nhat quanh footprint: {c.min():.3f} m"
                  f"  (margin_hard={MARGIN_HARD})")
            if c.min() <= 0.0:
                print(f"  {R}=> Co diem TRONG footprint — xe se ket BLOCKED.{X}")
                verdict.append(("B", "Con diem trong footprint sau khi loc."))

    # ── 4. Planner dang bao gi ───────────────────────────────────────
    print(f"\n{B}4. /follow/planner_status{X}")
    if n.status is None:
        print(f"  {R}KHONG CO. follow_planner_node chua chay.{X}")
        verdict.append(("B", "follow_planner_node khong chay."))
    else:
        import json
        try:
            s = json.loads(n.status)
            print(f"  state={s.get('state')}  note={s.get('note')}")
            print(f"  n_obstacles={s.get('n_obstacles')}  "
                  f"thoang_truoc={s.get('front_clearance_m')}  "
                  f"huong_chon={s.get('chosen_heading_deg')}")
            print(f"  cmd_v={s.get('cmd_v')}  cmd_w={s.get('cmd_w')}  "
                  f"enabled={s.get('enabled')}")
            if (s.get('n_obstacles') or 0) == 0:
                verdict.append(("B", "planner bao n_obstacles=0 — no khong nhan /scan."))
        except Exception as e:
            print(f"  Khong doc duoc JSON: {e}")

    print(f"\n{B}5. /follow/target{X}")
    print("  " + (n.target[:150] if n.target else f"{Y}KHONG CO — chay fake_target.py chua?{X}"))

    # ── Ket luan ─────────────────────────────────────────────────────
    print()
    print("=" * 70)
    if not verdict:
        print(f"{G}{B}  KHONG THAY VAN DE.{X}")
        print("  Lidar quay tot, planner thay vat can. Neu xe van di thang,")
        print("  hay chay lai voi vat can dat LECH sang mot ben 20cm — vat dat")
        print("  chinh giua co the khien xe do du giua trai va phai.")
    else:
        a = [v for k, v in verdict if k == "A"]
        b = [v for k, v in verdict if k == "B"]
        if a:
            print(f"{R}{B}  NGUYEN NHAN A — LIDAR KHONG QUAY{X}")
            for v in a:
                print(f"    - {v}")
            print()
            print("  Kiem tra theo thu tu:")
            print("   1. Nhin truc tiep: dau SC-Mini co quay khong?")
            print("   2. Nguon 5V du chua? Lidar can ~500mA luc khoi dong.")
            print("      Cam vao cong USB co nguon rieng, khong qua hub thu dong.")
            print("   3. Co DANG CHAY 2 launch file cung luc khong?")
            print("        ros2 node list | grep -c sc_mini     # phai la 1")
            print("      Neu la 2 -> tat bot, hai node tranh cung mot cong serial.")
            print("   4. Rut cam lai lidar roi chay lai launch.")
        if b:
            print(f"{Y}{B}  NGUYEN NHAN B — PLANNER KHONG DUNG DU LIEU{X}")
            for v in b:
                print(f"    - {v}")
    print("=" * 70)
    print()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
