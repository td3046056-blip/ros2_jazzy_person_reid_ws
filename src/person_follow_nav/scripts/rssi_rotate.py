#!/usr/bin/env python3
"""
rssi_rotate.py — thu nghiem "XOAY DO HUONG" bang RSSI (30/09).

VI SAO
------
Do 30/09: xe DUNG YEN thi RSSI khong phan biet duoc nguoi o trai 45 hay phai 45 (C-B ca hai
~ -5.5 dB) — offset tung board va do lech do nguoi deo beacon (~ -4..-6 dB) lon ngang tin hieu.
Khi mat camera va phai TIM LAI nguoi, xe duoc phep XOAY. Khi xoay, moi board lan luot quay mat ve
phia nguoi: do manh cua no len xuong theo yaw nhu mot song sin. PHA cua song do chi ra huong nguoi:
  - offset tung board, do lech do nguoi deo = hang so cong them -> KHONG doi pha
  - anten di chuyen vai chuc cm khi xoay -> phan xa (fading) trung binh ra
  - tinh trong khung ODOM -> xe quay the nao cung khong tre
Script nay de DO xem cach do chinh xac toi dau truoc khi dua vao planner (chua dung trong xe).

CACH DUNG
---------
  T1 (CHI lidar + driver, KHONG planner — giong calibrate_center):
    ros2 launch person_follow_nav calibrate.launch.py lidar_port:=/dev/serial/by-path/<cong LiDAR>

  T2: nguoi deo beacon SAU THAT LUNG, quay lung ve xe, dung cach tam truc banh truoc 1.5-2 m,
      o goc --person-deg so voi MUI XE LUC BAT DAU (trai duong, phai am). Dung yen den het.
    python3 rssi_rotate.py --ports $PA $PB $PC --person-deg 90 --out quay_p090.csv

  Phan tich lai cac lan da ghi:
    python3 rssi_rotate.py --analyze quay_*.csv
  Hieu chinh tu mot lan da biet huong nguoi: in huong "nhin" tung board va LUU MAU hinh dang
  (mau_rssi.json) — thuat toan tot nhat trong sim_rssi.py (kh5) can mau nay:
    python3 rssi_rotate.py --analyze quay_000.csv --calib
    python3 rssi_rotate.py --analyze quay_p090.csv quay_m045.csv quay_180.csv   # danh gia tren cac lan KHAC

AN TOAN
-------
Xe xoay tai cho, duoi quet ban kinh 0.47 m: script TU CHOI chay neu LiDAR thay vat trong 0.53 m tinh tu tam quay,
neu co node khac dang publish /cmd_vel (planner), hoac khong co /odom. Ctrl-C = dung xe ngay.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import sys
import threading
import time
from typing import Dict, List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # goc package
from rssi_log import port_users, reader  # noqa: E402
# Thuat toan nam trong package (dung chung voi rssi_bearing_node) — o day chi con phan do tren xe
from person_follow_nav.rssi_df import (  # noqa: E402,F401
    ALPHA_DEG, BOARDS, build_template, est_template, est_template_moving, estimate, load_alpha,
    load_template, save_template, wrap)

G = "\033[92m"
R = "\033[91m"
Y = "\033[93m"
B = "\033[1m"
X = "\033[0m"

# Kiem tra khoang trong truoc khi xoay. Tinh tu TAM QUAY (base_link = tam truc banh truoc), dung cac gia tri
# da chot trong CLAUDE.md muc 3 va cung luat voi _can_rotate_in_place() cua planner: rotate_radius + margin_hard.
BLIND_LIDAR_DEG = (246.0, 294.0)     # cung than xe / cot do (khung LiDAR) — bo qua; LiDAR MU thang phia sau
SELF_MAX_M = 0.20                    # tia dap vao than xe (0.116-0.131 m)
LIDAR_X = 0.10                       # LiDAR nam truoc base_link
LIDAR_YAW_OFFSET_DEG = -90.0         # a_base = a_lidar - 90
CLEAR_MIN_M = 0.47 + 0.06            # rotate_radius + margin_hard = 0.53 m tinh tu tam quay

def scan_to_base(ranges, angle_min: float, angle_inc: float, range_max: float):
    """Tia LiDAR hop le -> (huong do, khoang cach m) tinh tu TAM QUAY (base_link), da bo than xe va cung mu."""
    out = []
    for i, r in enumerate(ranges):
        a = math.degrees(angle_min + i * angle_inc) % 360.0
        if r is None or not math.isfinite(r) or r < SELF_MAX_M or r > range_max:
            continue
        if BLIND_LIDAR_DEG[0] <= a <= BLIND_LIDAR_DEG[1]:
            continue
        ab = math.radians(a + LIDAR_YAW_OFFSET_DEG)
        bx, by = LIDAR_X + r * math.cos(ab), r * math.sin(ab)
        out.append((math.degrees(math.atan2(by, bx)), math.hypot(bx, by)))
    return out


def _wd(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _clusters(pts, gap_deg: float = 4.0, gap_m: float = 0.35):
    """Gom diem (huong do, khoang cach m) thanh cum lien tiep theo huong -> [(huong TB, khoang cach TB, so diem)]."""
    if not pts:
        return []
    ps = sorted(pts)
    groups, cur = [], [ps[0]]
    for b, d in ps[1:]:
        if b - cur[-1][0] <= gap_deg and abs(d - cur[-1][1]) <= gap_m:
            cur.append((b, d))
        else:
            groups.append(cur)
            cur = [(b, d)]
    groups.append(cur)
    out = []
    for c in groups:
        sx = sum(math.cos(math.radians(b)) for b, _ in c)
        sy = sum(math.sin(math.radians(b)) for b, _ in c)
        out.append((math.degrees(math.atan2(sy, sx)), sum(d for _, d in c) / len(c), len(c)))
    return out


def _merge_legs(pick, cls):
    """Nguoi co hai chan = hai cum sat nhau: gop cac cum cach cum da chon <= 0.45 m (0.6 m thi gop ca do dac
    dung canh nguoi -> huong lech 8 do trong thu nghiem)."""
    px, py = pick[1] * math.cos(math.radians(pick[0])), pick[1] * math.sin(math.radians(pick[0]))
    sx = sy = n = 0.0
    for b, d, k in cls:
        x, y = d * math.cos(math.radians(b)), d * math.sin(math.radians(b))
        if math.hypot(x - px, y - py) <= 0.45:
            sx += x * k; sy += y * k; n += k
    return math.degrees(math.atan2(sy, sx)), math.hypot(sx, sy) / n, int(n)


def person_new_object(bg_pts, now_pts, person_deg: float, n_scans: int,
                      win_deg: float = 60.0, rmin: float = 0.7, rmax: float = 4.5):
    """Huong THAT cua nguoi = VAT MOI xuat hien so voi luc bat dau (truoc khi nguoi di toi vi tri).
    Tim "cum gan nhat quanh huong nhap" thi bat nham do dac (01/10: nguoi con dung canh laptop luc quet,
    script lay mot vat o -33 do lam nguoi). So voi nen thi do dac tu bi loai. None neu khong co vat moi."""
    bg = {}
    for b, d in bg_pts:
        k = int(round(b))
        bg[k] = min(bg.get(k, 99.0), d)
    new = [(b, d) for b, d in now_pts
           if rmin <= d <= rmax and abs(_wd(b - person_deg)) <= win_deg
           and d < min(bg.get(int(round(b)) + j, 99.0) for j in (-2, -1, 0, 1, 2)) - 0.30]
    # Nguong tinh tren CA NGUOI (hai chan gop lai). 01/10: o 2 m moi chan chi trung 2-3 tia va LiDAR mat ~50% tia
    # moi vong -> nguong "moi cum >= so vong quet" loai mat chan that o ca 11 lan do.
    cls = [c for c in _clusters(new) if c[2] >= 2]
    groups = [_merge_legs(c, cls) for c in cls]
    groups = [g for g in groups if g[2] >= max(4, int(0.5 * n_scans))]
    if not groups:
        return None
    return max(groups, key=lambda g: (g[2], -abs(_wd(g[0] - person_deg))))


def person_near(now_pts, expect_deg: float, expect_dist: float, n_scans: int, win_deg: float = 30.0, dtol: float = 0.4):
    """Lan lap sau: nguoi van dung yen -> lay cum gan vi tri da biet nhat (huong +-win, khoang cach +-dtol)."""
    cand = [(b, d) for b, d in now_pts if abs(_wd(b - expect_deg)) <= win_deg and abs(d - expect_dist) <= dtol]
    cls = [c for c in _clusters(cand) if c[2] >= 2]
    groups = [g for g in (_merge_legs(c, cls) for c in cls) if g[2] >= max(3, int(0.3 * n_scans))]
    if not groups:
        return None
    return min(groups, key=lambda g: abs(_wd(g[0] - expect_deg)))


def scan_heading_shift(ref_pts, now_pts):
    """Goc xe DA XOAY giua hai thoi diem (do, duong = xoay trai), do bang so khop hai dam diem LiDAR (xe chi xoay tai
    cho quanh base_link). Sau moi vong xoay xe dung LECH ~18 do so voi luc dau (01/10) -> huong nguoi so voi mui xe
    doi theo; khong bu thi sai so lan lap 2, 3 bi cong oan 18, 36 do. Tra ve (goc, ti le diem khop)."""
    def xy(pts):
        return np.array([(d * math.cos(math.radians(b)), d * math.sin(math.radians(b))) for b, d in pts if d <= 5.0])
    A, Bp = xy(ref_pts), xy(now_pts)
    if len(A) < 20 or len(Bp) < 20:
        return 0.0, 0.0
    if len(A) > 400:
        A = A[:: len(A) // 400 + 1]
    if len(Bp) > 400:
        Bp = Bp[:: len(Bp) // 400 + 1]

    def score(k: float, tol: float) -> int:
        c, s_ = math.cos(math.radians(k)), math.sin(math.radians(k))
        Br = Bp @ np.array([[c, s_], [-s_, c]])
        return int((np.sqrt(((A[:, None, :] - Br[None, :, :]) ** 2).sum(2)).min(1) < tol).sum())
    best = max(range(-180, 180), key=lambda k: score(k, 0.12))
    fine = max(np.arange(best - 1.0, best + 1.01, 0.25), key=lambda k: score(k, 0.10))
    return float(fine), score(fine, 0.12) / len(A)


def load(path: str):
    rows = list(csv.DictReader(open(path, newline="")))
    meta_path = os.path.splitext(path)[0] + ".json"
    meta = json.load(open(meta_path)) if os.path.exists(meta_path) else {}
    r = [x for x in rows if x["kind"] == "R"]
    o = [x for x in rows if x["kind"] == "O"]
    t_r = np.array([float(x["t_pc"]) for x in r])
    sid = np.array([x["id"] for x in r])
    rssi = np.array([float(x["rssi"]) for x in r])
    t_o = np.array([float(x["t_pc"]) for x in o])
    yaw_u = np.unwrap(np.array([float(x["yaw"]) for x in o]))
    return t_r, sid, rssi, t_o, yaw_u, meta


def analyze(path: str, alpha_deg: Dict[str, float], calib: bool, tmpl_path: str = "", use_lidar: bool = True) -> None:
    t_r, sid, rssi, t_o, yaw_u, meta = load(path)
    # Khi --calib: file nay dung de TAO mau -> khong danh gia mau tren chinh no (se dung gia)
    tmpl = None if calib else load_template(tmpl_path)
    t_rot0 = meta.get("t_rot_start", t_o[0])
    t_rot1 = meta.get("t_rot_end", t_o[-1])
    sel = (t_o >= t_rot0) & (t_o <= t_rot1)
    t_o, yaw_u = t_o[sel], yaw_u[sel]
    if len(t_o) < 10:
        print(f"{R}{path}: khong co doan xoay{X}")
        return
    truth = None
    src = ""
    if "truth_deg" in meta and use_lidar:
        truth = wrap(yaw_u[0] + math.radians(float(meta["truth_deg"])))
        src = f", nguoi o {meta['truth_deg']:+.0f} do so voi mui xe [{meta.get('truth_src', '')}] (nhap {meta.get('person_deg', 0):+.0f})"
    elif "person_deg" in meta:
        truth = wrap(yaw_u[0] + math.radians(float(meta["person_deg"])))
        src = f", nguoi o {meta['person_deg']:+.0f} do so voi mui luc dau (NHAP BANG MAT)"
    prog = np.abs(yaw_u - yaw_u[0])
    print(f"\n{B}{path}{X}  xoay {math.degrees(prog[-1]):.0f} do trong {t_o[-1] - t_o[0]:.1f} s" + src)
    segs = [("ca doan", t_o[0], t_o[-1])]
    n_turn = int((prog[-1] + math.radians(5.0)) // (2.0 * math.pi))   # thieu vai do cuoi van tinh du vong
    for k in range(n_turn):
        i0 = np.searchsorted(prog, k * 2.0 * math.pi)
        i1 = min(len(t_o) - 1, np.searchsorted(prog, (k + 1) * 2.0 * math.pi))
        segs.append((f"vong {k + 1}", t_o[i0], t_o[i1]))
    def err_txt(phi: Optional[float]) -> str:
        if phi is None:
            return "  khong uoc luong duoc"
        s = f" huong {math.degrees(phi):+7.1f}"
        if truth is not None:
            err = math.degrees(wrap(phi - truth))
            s += f"  {G if abs(err) <= 30 else R}sai {err:+6.1f} do{X}"
        return s

    for name, a, b in segs:
        so = (t_o >= a) & (t_o <= b)
        res = estimate(t_r, sid, rssi, t_o[so], yaw_u[so], alpha_deg)
        s = f"  {name:8s} hai_cal" + err_txt(None if res is None else res["phi"])
        if truth is not None and name == "ca doan":
            s += f"   (that {math.degrees(truth):+.1f})"
        if res is not None:
            s += f"  | bien do {res['amp_db']:4.1f} dB, dong thuan {res['agree']:.2f} |"
            s += " ".join(f" {k}:{v['amp_db']:4.1f}dB@{math.degrees(v['phi']):+5.0f}"
                          for k, v in res["boards"].items())
        print(s)
        if tmpl is not None:
            ts, ys = t_o[so], yaw_u[so]
            print(f"  {'':8s} mau    " + err_txt(est_template(t_r, sid, rssi, ts, ys, tmpl)))
            print(f"  {'':8s} kh5    " + err_txt(est_template_moving(t_r, sid, rssi, ts, ys, tmpl, ts[-1], margin=0.05)))
        if calib and truth is not None and name == "ca doan":
            sug = None
            if res is not None:
                sug = {k: round(math.degrees(wrap(truth - v["raw_arg"])), 1) for k, v in res["boards"].items()}
                print(f"  {Y}--calib: huong nhin do duoc tren xe: {sug}{X}")
            new = build_template(t_r, sid, rssi, t_o, yaw_u, truth)
            if new:
                save_template(new, tmpl_path, sug)
                print(f"  {Y}--calib: da luu mau {sorted(new)} + huong nhin vao {tmpl_path} — cac lan do KHAC tu doc,"
                      f" khong can --alpha{X}")

def record(args) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan

    for p in args.ports:
        if not os.path.exists(p):
            print(f"{R}Khong co cong {p}{X}")
            return 1
        if port_users(p):
            print(f"{R}Cong {p} dang bi tien trinh khac giu (LiDAR?). Kiem tra lai.{X}")
            return 1

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("rssi_rotate")
    st = {"yaw": None, "scan": None, "scans": []}
    odom_log: List[tuple] = []

    def on_odom(msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        st["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        odom_log.append((time.time(), st["yaw"]))

    def on_scan(msg: LaserScan) -> None:
        st["scan"] = msg
        st["scans"].append((time.time(), scan_to_base(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max)))
        del st["scans"][:-80]

    node.create_subscription(Odometry, "/odom", on_odom, 50)
    # sc_mini phat /scan kieu SensorDataQoS (best effort) — dang ky mac dinh (reliable) thi KHONG nhan duoc
    # goi nao (loi 01/10: "offering incompatible QoS ... RELIABILITY"). Cac node khac cua du an cung dung kieu nay.
    node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)

    def spin(sec: float) -> None:
        t_end = time.time() + sec
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.02)

    pub = None
    stop = threading.Event()
    interrupted = False
    try:
        # Cho toi da 5 s: DDS can ~1 s de tim thay nhau, sc_mini bo 5 vong quet dau
        t_wait = time.time()
        while time.time() - t_wait < 5.0 and (st["yaw"] is None or (st["scan"] is None and not args.no_scan_check)):
            spin(0.1)
        if st["yaw"] is None:
            if node.count_publishers("/odom") == 0:
                print(f"{R}Khong co /odom: KHONG co node nao phat /odom.{X} calibrate.launch.py da chay o terminal khac chua"
                      " (va con dang chay)? Kiem tra: ros2 node list")
            else:
                print(f"{R}Khong co /odom: driver DANG chay nhung KHUNG XE khong gui du lieu.{X}\n"
                      "  -> Bat cong tac nguon khung xe BW-DR03 (pin); kiem tra day tu khung xe toi mach FTDI.\n"
                      "  -> Kiem tra: ros2 topic hz /odom  (phai ra 20-50 Hz)")
            return 1
        n_pub = node.count_publishers("/cmd_vel")
        if n_pub > 0:
            print(f"{R}Dang co {n_pub} node publish /cmd_vel (follow_planner?). Chi chay voi calibrate.launch.py.{X}")
            return 1
        if not args.no_scan_check:
            sc = st["scan"]
            if sc is None:
                if node.count_publishers("/scan") == 0:
                    print(f"{R}Khong co /scan: khong co node nao phat /scan{X} (sc_mini chua chay?).")
                else:
                    print(f"{R}Khong co /scan: sc_mini DANG chay nhung LiDAR khong gui du lieu.{X}\n"
                          "  -> Sai lidar_port? Rut cam lai cap USB cua LiDAR roi launch lai. Kiem tra: ros2 topic hz /scan")
                print("  (Them --no-scan-check de bo qua, CHI khi chac chan trong 0.6 m quanh xe.)")
                return 1
        # Nen: cac vong quet luc BAT DAU (nguoi chua vao vi tri — thuong con dung canh laptop tren xe)
        t_bg = time.time()
        while time.time() - t_bg < 1.5 and st["scan"] is not None and len(st["scans"]) < 12:
            spin(0.1)
        bg_pts = [pt for _, pts in st["scans"] for pt in pts]

        expect = getattr(args, "_first", None)           # da co lan lap truoc -> nguoi dang dung san o vi tri
        delay = args.delay if expect is None else min(args.delay, 2.0)
        if delay > 0:
            if expect is None:
                print(f"{B}Dem nguoc {delay:.0f} s — DI TOI vi tri (huong {args.person_deg:+.0f} do, cach xe 1.5-2.5 m), "
                      f"de beacon nhin ve xe roi DUNG YEN.{X}")
            t_d = time.time()
            last = None
            while time.time() - t_d < delay:
                left = int(math.ceil(delay - (time.time() - t_d)))
                if left != last and expect is None:
                    print(f"  con {left} s", flush=True)
                    last = left
                spin(0.1)

        now_scans = [(t, pts) for t, pts in st["scans"] if t >= time.time() - 1.3]
        now_pts = [pt for _, pts in now_scans for pt in pts]
        if not args.no_scan_check:
            if not now_scans:
                print(f"{R}LiDAR NGUNG gui du lieu trong luc cho (sc_mini con chay nhung khong co /scan moi).{X}\n"
                      "  -> Ctrl-C launch o Terminal 1, rut cam lai cap USB cua LiDAR, launch lai.")
                return 1
            worst = min(((d, bdeg) for bdeg, d in now_pts), default=(99.0, 0.0))
            if worst[0] < CLEAR_MIN_M:
                print(f"{R}Vat can cach tam quay {worst[0]:.2f} m (can > {CLEAR_MIN_M:.2f} m) o huong {worst[1]:+.0f} do"
                      f" so voi mui xe (trai duong){X} — duoi xe quet ban kinh 0.45 m khi xoay.\n"
                      "  -> Don vat do / dung xa xe hon roi chay lai. (LiDAR mu thang phia sau: tu nhin bang mat.)")
                return 1
            print(f"Khoang trong quanh xe: vat gan nhat cach tam quay {worst[0]:.2f} m (huong {worst[1]:+.0f} do) — du cho xoay.")

        # HUONG THAT cua nguoi so voi mui xe luc bat dau lan do nay (truth_deg) + nguon goc cua no
        lidar_person = None
        scan_meta = {}
        shift = 0.0
        first = getattr(args, "_first", None)            # lan lap dau: {truth, dist, pts, yaw, src}
        if now_scans:
            sc = st["scan"]
            scan_meta = {"scan_angle_min": sc.angle_min, "scan_angle_inc": sc.angle_increment, "scan_range_max": sc.range_max,
                         "scan_ranges": [round(float(r), 3) if math.isfinite(r) else None for r in sc.ranges]}
        if first is None:
            if now_scans:
                lidar_person = person_new_object(bg_pts, now_pts, args.person_deg, len(now_scans))
            if lidar_person is not None:
                truth_deg, truth_src = lidar_person[0], "LiDAR: vat moi xuat hien"
                diff = _wd(truth_deg - args.person_deg)
                print(f"{G if abs(diff) <= 15 else Y}LiDAR thay nguoi o {truth_deg:+.0f} do, cach {lidar_person[1]:.2f} m "
                      f"[vat moi xuat hien] — ban nhap {args.person_deg:+.0f} do (lech {diff:+.0f}). Dung so cua LiDAR lam chuan.{X}")
            else:
                truth_deg, truth_src = float(args.person_deg), "huong nhap (dat bang mat)"
                print(f"{Y}LiDAR KHONG thay ai moi buoc vao quanh huong {args.person_deg:+.0f} do — dung huong ban nhap (dat bang mat).\n"
                      f"  Muon LiDAR do: bam Enter khi CHUA dung o vi tri, roi di toi do trong luc dem nguoc. (Thang sau xe: cung mu.){X}")
            args._first = {"truth": truth_deg, "dist": lidar_person[1] if lidar_person else None,
                           "pts": now_pts, "yaw": st["yaw"], "src": truth_src}
        else:
            if now_scans and first["pts"]:
                shift, frac = scan_heading_shift(first["pts"], now_pts)
                how = f"so khop LiDAR, khop {frac * 100:.0f}%"
                if frac < 0.5:
                    shift, how = math.degrees(wrap(st["yaw"] - first["yaw"])), "odom (LiDAR khop kem)"
            else:
                shift, how = math.degrees(wrap(st["yaw"] - first["yaw"])), "odom"
            truth_deg = _wd(first["truth"] - shift)
            truth_src = f"{first['src']} + bu xe lech {shift:+.1f} do ({how})"
            if first["dist"] is not None and now_scans:
                lidar_person = person_near(now_pts, truth_deg, first["dist"], len(now_scans))
                if lidar_person is not None:
                    truth_deg, truth_src = lidar_person[0], "LiDAR: bam theo lan truoc"
            print(f"So voi lan 1 xe da lech {shift:+.1f} do ({how}) -> nguoi dang o {truth_deg:+.0f} do so voi mui xe.")

        q: "queue.Queue" = queue.Queue()
        threads = [threading.Thread(target=reader, args=(p, 115200, q, stop), daemon=True) for p in args.ports]
        for th in threads:
            th.start()
        out = open(args.out, "w", newline="")
        w = csv.writer(out)
        w.writerow(["t_pc", "kind", "id", "seq", "rssi", "yaw"])
        n_odom_written = 0
        static_rssi: Optional[dict] = {}

        def drain() -> None:
            nonlocal n_odom_written
            while True:
                try:
                    m = q.get_nowait()
                except queue.Empty:
                    break
                if m[0] == "R":
                    _, t, sid, seq, _ms, rssi, _port = m
                    w.writerow([f"{t:.4f}", "R", sid, seq, rssi, ""])
                    if static_rssi is not None:
                        static_rssi.setdefault(sid, []).append(rssi)
                elif m[0] == "ERR":
                    print(f"{R}  {m[1]}: {m[2]}{X}")
            for t, y in odom_log[n_odom_written:]:
                w.writerow([f"{t:.4f}", "O", "", "", "", f"{y:.5f}"])
            n_odom_written = len(odom_log)

        print(f"Ghi dung yen 3 s (board khoi dong)...")
        t0 = time.time()
        while time.time() - t0 < 3.0:
            spin(0.1)
            drain()
        lv = {k: sum(v) / len(v) for k, v in sorted(static_rssi.items()) if v}
        static_rssi = None
        if len(lv) < 3:
            print(f"{R}Chi thay board {sorted(lv)} — thieu board hoac beacon tat?{X}")
        else:
            print("Muc tin hieu luc dung yen: " + ", ".join(f"{k} {v:.0f} dBm" for k, v in lv.items()))
            if max(lv.values()) < -78.0:
                print(f"{Y}  Tin hieu YEU (board manh nhat < -78 dBm): beacon co dang bi THAN NGUOI che khong?\n"
                      f"  Beacon phai NHIN THANG ve xe: deo sau that lung va quay lung ve xe, hoac deo truoc bung neu quay mat.{X}")

        pub = node.create_publisher(Twist, "/cmd_vel", 10)
        target = args.turns * 2.0 * math.pi
        t_limit = target / abs(args.w) * 2.0 + 5.0
        yaw_prev = st["yaw"]
        swept = 0.0
        t_rot0 = time.time()
        print(f"Xoay {args.turns:.1f} vong o {args.w:+.2f} rad/s (Ctrl-C de dung)...")
        cmd = Twist()
        cmd.angular.z = float(args.w)
        last_print = t_rot0
        while swept < target and time.time() - t_rot0 < t_limit:
            pub.publish(cmd)
            spin(1.0 / 15.0)
            drain()
            swept += abs(wrap(st["yaw"] - yaw_prev))
            yaw_prev = st["yaw"]
            if time.time() - last_print > 2.0:
                last_print = time.time()
                print(f"  da xoay {math.degrees(swept):5.0f} / {math.degrees(target):.0f} do")
        t_rot1 = time.time()
        for _ in range(5):
            pub.publish(Twist())
            spin(0.05)
        drain()
        out.close()
        meta = {"person_deg": args.person_deg, "w": args.w, "turns": args.turns,
                "t_rot_start": t_rot0, "t_rot_end": t_rot1, "dist_m": args.dist, "static_dbm": lv}
        meta["truth_deg"] = round(truth_deg, 1)
        meta["truth_src"] = truth_src
        meta["heading_shift_deg"] = round(shift, 2)
        if lidar_person is not None:
            meta["person_lidar_dist"] = round(lidar_person[1], 2)
        meta.update(scan_meta)
        json.dump(meta, open(os.path.splitext(args.out)[0] + ".json", "w"))
        if swept < target:
            print(f"{Y}Het gio truoc khi du vong (xoay {math.degrees(swept):.0f} do) — banh truot?{X}")
    except KeyboardInterrupt:
        print(f"\n{Y}Ctrl-C — dung xe{X}")
        interrupted = True
    finally:
        if pub is not None:
            for _ in range(5):
                pub.publish(Twist())
                time.sleep(0.03)
        stop.set()
        node.destroy_node()
        rclpy.shutdown()
    if os.path.exists(args.out) and os.path.exists(os.path.splitext(args.out)[0] + ".json"):
        analyze(args.out, args.alpha_dict, calib=False, tmpl_path=args.mau)
    return 130 if interrupted else 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Thu nghiem xoay do huong bang RSSI")
    ap.add_argument("--ports", nargs="+", help="cong serial 3 board quet (by-path)")
    ap.add_argument("--person-deg", type=float, help="huong nguoi so voi mui xe luc bat dau (trai duong)")
    ap.add_argument("--dist", type=float, default=1.5, help="khoang cach nguoi (chi de ghi chu)")
    ap.add_argument("--w", type=float, default=0.4, help="toc do xoay rad/s (duong = xoay trai)")
    ap.add_argument("--turns", type=float, default=2.0, help="so vong")
    ap.add_argument("--out", default=time.strftime("quay_%H%M%S.csv"))
    ap.add_argument("--delay", type=float, default=8.0,
                    help="dem nguoc (s) sau khi bam Enter de nguoi DI TOI vi tri truoc khi LiDAR do huong that")
    ap.add_argument("--no-lidar-truth", action="store_true",
                    help="khi phan tich: dung huong da NHAP lam chuan, bo qua so LiDAR da luu (vd. LiDAR bat nham vat)")
    ap.add_argument("--repeat", type=int, default=1,
                    help="lap lai N lan lien tiep (nguoi dung yen); file <out>_1.csv, _2.csv...; Ctrl-C dung ca chuoi")
    ap.add_argument("--no-scan-check", action="store_true", help="bo kiem tra khoang trong bang LiDAR")
    ap.add_argument("--analyze", nargs="+", metavar="CSV", help="chi phan tich cac file da ghi")
    ap.add_argument("--alpha", help="huong nhin A,B,C (do, base_link, trai duong)")
    ap.add_argument("--calib", action="store_true",
                    help="tu file biet huong nguoi: in huong nhin tung board VA luu mau vao --mau")
    ap.add_argument("--mau", default="mau_rssi.json", help="file mau hinh dang (tao bang --calib)")
    args = ap.parse_args()

    alpha = dict(ALPHA_DEG)
    if args.alpha:
        a, b, c = (float(x) for x in args.alpha.split(","))
        alpha = {"A": a, "B": b, "C": c}
    elif not args.calib and load_alpha(args.mau):
        alpha.update(load_alpha(args.mau))
        print(f"Dung huong nhin tu {args.mau}: " + ", ".join(f"{k} {v:+.1f}" for k, v in sorted(alpha.items())))
    args.alpha_dict = alpha
    if args.analyze:
        for p in args.analyze:
            analyze(p, alpha, args.calib, args.mau, not args.no_lidar_truth)
        return 0
    if not args.ports or args.person_deg is None:
        ap.error("can --ports va --person-deg (hoac --analyze)")
    stem, ext = os.path.splitext(args.out)
    outs = [args.out] if args.repeat <= 1 else [f"{stem}_{i + 1}{ext}" for i in range(args.repeat)]
    for o in outs:
        if os.path.exists(o):
            ap.error(f"{o} da co — dat ten khac")
    for i, o in enumerate(outs):
        if len(outs) > 1:
            print(f"\n{B}===== Lan {i + 1}/{len(outs)} -> {o} ====={X}")
        args.out = o
        rc = record(args)
        if rc != 0:
            return rc
    return 0


if __name__ == "__main__":
    sys.exit(main())
