"""
test_reacq.py — kiem tra OFFLINE (chi can numpy) luat "khoa lai nhanh sau khi mat dau" cua rssi_follow.py (06/10).

Dung vong quet LiDAR gia (do tia tren hinh tron: chan nguoi, chan ghe, tuong phong; LiDAR 360 tia, mat 30 % tia, cung mu
sau duoi nhu xe that) roi goi DUNG ham that Follower.moving() / Follower.reacq_candidate():
  1. mot nguoi dang di gan cho du doan, khong co ai khac       -> phai khoa lai duoc, dung nguoi do
  2. HAI nguoi cung di gan cho du doan                          -> KHONG duoc khoa (khong biet ai la chu)
  3. mot nguoi di + mot nguoi DUNG ngay cho du doan             -> KHONG duoc khoa (chu co the dang dung do)
  4. chi co chan ghe dung yen o cho du doan                     -> KHONG duoc khoa
  5. nguoi di ngang GIUA xe va cho du doan (vat che)            -> KHONG duoc khoa
  6. chan ghe vua lot vao tam nhin (truoc do nam trong cung mu) -> moving() = None, KHONG duoc khoa
  7. moving(): nguoi dung yen -> False, nguoi dang di -> True
  8. nguoi bi che 2.5 s nhung VAN DI THANG, hien ra cach cho thay cuoi 1.8 m -> phai khoa duoc (tim theo duong di)
"""
import math
import os
import sys
import types
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
CLOCK = [0.0]
import rssi_follow as rf  # noqa: E402
import rssi_seek as rs  # noqa: E402
from person_follow_nav.geometry import scan_to_base_points, self_filter_mask  # noqa: E402

rf.time = types.SimpleNamespace(time=lambda: CLOCK[0])
rs.time = types.SimpleNamespace(time=lambda: CLOCK[0])
ROOM = (-4.0, 4.0, -3.5, 3.5)
LEG_R, LEG_SEP = 0.055, 0.09


def legs(x, y, hx=1.0, hy=0.0):
    """Hai chan nguoi o (x, y), xep ngang so voi huong (hx, hy)."""
    return [(x - LEG_SEP * hy, y + LEG_SEP * hx, LEG_R), (x + LEG_SEP * hy, y - LEG_SEP * hx, LEG_R)]


def chair(x, y):
    return [(x - 0.12, y, 0.02), (x + 0.12, y, 0.02)]


def scan(rx, ry, ryaw, circles, rng):
    """Vong quet that cua sc_mini (0..360 do, a_base = a_laser - 90) -> diem base_link sau loc than xe."""
    idx = np.arange(360)
    ox, oy = rx + rs.LIDAR_X * math.cos(ryaw), ry + rs.LIDAR_X * math.sin(ryaw)
    ang = ryaw + np.radians(idx - 90.0)
    dx, dy = np.cos(ang), np.sin(ang)
    r = np.full(360, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for t in ((ROOM[0] - ox) / dx, (ROOM[1] - ox) / dx, (ROOM[2] - oy) / dy, (ROOM[3] - oy) / dy):
            hx, hy = ox + t * dx, oy + t * dy
            ok = (t > 0) & (hx >= ROOM[0] - 1e-6) & (hx <= ROOM[1] + 1e-6) & (hy >= ROOM[2] - 1e-6) & (hy <= ROOM[3] + 1e-6)
            r = np.where(ok & (t < r), t, r)
    for cx, cy, cr in circles:
        bx, by = cx - ox, cy - oy
        b = bx * dx + by * dy
        disc = b * b - (bx * bx + by * by - cr * cr)
        ok = disc >= 0
        t = b - np.sqrt(np.where(ok, disc, 0.0))
        r = np.where(ok & (t > 0) & (t < r), t, r)
    r[rng.random(360) < 0.30] = np.inf
    pts, _b, _r, ld = scan_to_base_points(r, 0.0, math.radians(1.0), 0.10, 10.0, rs.LIDAR_X, rs.LIDAR_Y,
                                          math.radians(rs.LIDAR_YAW_OFFSET_DEG), rs.LIDAR_ANGLE_SIGN, rs.MAX_USE_RANGE)
    if len(pts):
        pts = pts[self_filter_mask(pts, ld, rs.FRONT_LEN, rs.REAR_LEN, rs.HALF_WIDTH, rs.SELF_FILTER_MARGIN,
                                   rs.BLIND_SECTORS_DEG)]
    return pts


def run(world, robot=lambda t: (0.0, 0.0, 0.0), t_end=3.5, seed=1):
    """world(t) -> danh sach hinh tron; robot(t) -> (x, y, yaw). Quet 10 Hz tu 0 toi t_end, nhu Seeker._scan + on_scan."""
    f = rf.Follower.__new__(rf.Follower)
    f.scans, f.recent = deque(maxlen=8), deque(maxlen=36)
    f.bg_mask, f.rejected, f.acc = None, [], None
    rng = np.random.default_rng(seed)
    for k in range(int(round(t_end / 0.1)) + 1):
        t = k * 0.1
        CLOCK[0] = t
        f.x, f.y, f.yaw = robot(t)
        f.scans.append((t, f.x, f.y, f.yaw, scan(f.x, f.y, f.yaw, world(t), rng)))
        f.recent.append(f.scans[-1])
    return f


def walker(x0, y0, x1, y1, v):
    """Nguoi di thang tu (x0, y0) toi (x1, y1) voi toc do v: vi tri luc t."""
    L = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / L, (y1 - y0) / L
    return lambda t: (x0 + ux * min(L, v * t), y0 + uy * min(L, v * t), ux, uy)


res = []


def check(name, got, want_xy=None, tol=0.30):
    if want_xy is None:
        ok = got is None
        txt = "khong khoa" if got is None else f"KHOA ({got[0]:+.2f},{got[1]:+.2f})"
    else:
        ok = got is not None and math.hypot(got[0] - want_xy[0], got[1] - want_xy[1]) <= tol
        txt = "khong khoa" if got is None else f"khoa ({got[0]:+.2f},{got[1]:+.2f}), nguoi that ({want_xy[0]:+.2f},{want_xy[1]:+.2f})"
    res.append(ok)
    print(f"  {'DAT' if ok else 'LOI'}  {name}: {txt}")


print("=" * 72)
print("TEST — khoa lai nhanh sau khi mat dau (rssi_follow.py: moving / reacq_candidate)")
print("=" * 72)
A = walker(1.6, 1.4, 1.6, -1.4, 0.6)          # nguoi di ngang truoc xe, 0.6 m/s
P = A(3.0)[:2]                                 # cho thay cuoi: tre 0.5 s so voi nguoi that
SEG = ((P, P), 1.0)                            # vung tim: quanh mot diem, ban kinh 1 m (ca 1-7)

f = run(lambda t: legs(*A(t)))
check("1. mot nguoi dang di gan cho du doan", f.reacq_candidate(*SEG), A(3.5)[:2])

B = walker(2.2, 1.4, 2.2, -1.4, 0.6)          # nguoi thu hai di song song, xa hon 0.6 m
f = run(lambda t: legs(*A(t)) + legs(*B(t)))
check("2. HAI nguoi cung di gan cho du doan", f.reacq_candidate(*SEG))

# nguoi dung yen cach cho du doan 0.56 m, cach nguoi dang di 0.85 m (hai nhom TACH rieng — gan hon thi hai nhom gop lam
# mot va bi loai vi ly do khac, khong kiem duoc luat nay)
f = run(lambda t: legs(*A(t)) + legs(1.35, 0.1))
check("3. mot nguoi di + mot nguoi DUNG o cho du doan", f.reacq_candidate(*SEG))

f = run(lambda t: chair(1.6, -0.5))
check("4. chi co chan ghe dung yen o cho du doan", f.reacq_candidate(((1.6, -0.4), (1.6, -0.4)), 1.0))

C = walker(1.6, 1.4, 1.6, -1.4, 0.6)
f = run(lambda t: legs(*C(t)))
check("5. nguoi di ngang GIUA xe va cho du doan (vat che)", f.reacq_candidate(((2.4, -0.4), (2.4, -0.4)), 1.0))

# xe xoay cham 0.2 rad/s: chan ghe cach 1.5 m o +147.5 do luc cuoi; tu 0.5 toi 2.3 s (= 1.2-3 s truoc luc xet) ca hai
# chan nam tron trong cung mu sau duoi (156-204 do), tu ~3 s moi lot vao tam nhin
spin = lambda t: (0.0, 0.0, -0.7 + 0.2 * t)
CH = (1.5 * math.cos(math.radians(147.5)), 1.5 * math.sin(math.radians(147.5)))
f = run(lambda t: chair(*CH), robot=spin, t_end=3.5)
mv = f.moving(*CH)
ok6 = mv is None
res.append(ok6)
print(f"  {'DAT' if ok6 else 'LOI'}  6a. chan ghe vua lot vao tam nhin: moving() = {mv} (phai la None)")
check("6b. ... khong duoc khoa", f.reacq_candidate(((CH[0] + 0.05, CH[1] - 0.1),) * 2, 1.0))

f = run(lambda t: legs(*A(t)) + legs(1.0, 1.2))
m_still, m_walk = f.moving(1.0, 1.2), f.moving(*A(3.5)[:2])
ok7 = m_still is False and m_walk is True
res.append(ok7)
print(f"  {'DAT' if ok7 else 'LOI'}  7. moving(): nguoi dung yen = {m_still} (phai False), nguoi dang di = {m_walk} (phai True)")

# nguoi di 0.6 m/s, bi che kin tu 0.5 toi 3.0 s; lan cuoi thay o (1.6, 1.1) di xuong -y: vung tim keo dai theo van toc
D = walker(1.6, 1.4, 1.6, -1.4, 0.6)
f = run(lambda t: [] if 0.5 <= t < 3.0 else legs(*D(t)))
seg8 = ((D(0.5)[0], D(0.5)[1]), (D(0.5)[0], D(0.5)[1] - 0.6 * 3.0))
check("8. nguoi bi che 2.5 s van di thang, hien ra xa cho thay cuoi 1.8 m", f.reacq_candidate(seg8, 0.6 + 0.25 * 3.0), D(3.5)[:2])
check("8b. ... nhung neu chi tim quanh cho thay cuoi (cach cu) thi khong thay", f.reacq_candidate((seg8[0], seg8[0]), 1.0))

print(f"\n=> {'DAT' if all(res) else 'LOI'}  ({sum(res)}/{len(res)})")
print("=" * 72)
sys.exit(0 if all(res) else 1)
