#!/usr/bin/env python3
"""
test_fast_geometry.py — 3 ham NHANH cua geometry.py (13/10, toi uu cho mini PC i5-5200U) phai cho KET QUA GIONG
HET TUNG BIT doan code goc trong follow_planner_node.py (chep nguyen van duoi day lam mau doi chung):

  1. rect_clearance_min  == rect_clearance(lx, ly, ...).min(axis=2)        (_dwa)
  2. segments_blocked    == [segment_blocked(...)[0] cho tung huong]       (_choose_heading)
  3. grid_min_dist2      == vet can (GX - qx)**2 + (GY - qy)**2 theo khoi 48 diem, MOI o < cutoff;
                            va o cam / chi phi o cua ban do luoi (_nav_plan) giong het

Khong can ROS, chi numpy:  python3 test_fast_geometry.py   -> phai "=> DAT"
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from person_follow_nav.geometry import (  # noqa: E402
    grid_min_dist2, rect_clearance, rect_clearance_min, segment_blocked, segments_blocked,
)

F, R, HW = 0.14, 0.33, 0.30          # footprint xe (CLAUDE.md muc 3)
rng = np.random.default_rng(13)
ok_all = True


def report(name, ok, extra=""):
    global ok_all
    ok_all &= ok
    print(f"  {'DAT' if ok else 'LOI'}  {name}  {extra}")


def scene(m):
    """Diem vat can giong that: tuong xa, vat gan, ca diem sat / trong footprint."""
    a = rng.uniform(-math.pi, math.pi, m)
    r = np.concatenate([rng.uniform(0.15, 4.0, m - m // 5), rng.uniform(0.0, 0.6, m // 5)])
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


# ── 1. DWA: do thoang footprint ─────────────────────────────────────────────────────────────────────────
print("1. rect_clearance_min (DWA)")
bad = 0
cases = 0
for _ in range(300):
    n_v, n_w, k = int(rng.integers(1, 9)), int(rng.integers(1, 18)), 12
    m = int(rng.integers(1, 420))
    V = np.repeat(rng.uniform(-0.1, 0.25, n_v), n_w)
    W = np.tile(rng.uniform(-1.0, 1.0, n_w), n_v)
    t = np.arange(1, k + 1) * 0.10
    theta = W[:, None] * t[None, :]
    small = np.abs(W) < 1e-4
    Wsafe = np.where(small, 1e-4, W)
    Rr = V / Wsafe
    X = np.where(small[:, None], V[:, None] * t[None, :], Rr[:, None] * np.sin(theta))
    Y = np.where(small[:, None], 0.0, Rr[:, None] * (1.0 - np.cos(theta)))
    P = scene(m)
    # --- ban goc (_dwa) ---
    dx = P[None, None, :, 0] - X[:, :, None]
    dy = P[None, None, :, 1] - Y[:, :, None]
    c = np.cos(theta)[:, :, None]
    s = np.sin(theta)[:, :, None]
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    ref = rect_clearance(lx, ly, F, R, HW).min(axis=2)
    new = rect_clearance_min(X, Y, theta, P, F, R, HW)
    cases += 1
    if not np.array_equal(ref, new):
        bad += 1
report("giong tung bit ban goc", bad == 0, f"({cases} canh, {bad} khac)")

# ── 2. Tang chon khe: duong thang co bi chan ────────────────────────────────────────────────────────────
print("2. segments_blocked (chon khe)")
bad = 0
cases = 0
cand = np.radians(np.arange(-100.0, 100.1, 2.0))
for _ in range(400):
    m = int(rng.integers(0, 400))
    P = scene(m) if m > 0 else np.zeros((0, 2))
    reach = float(rng.choice([1.6, 1.1, 0.7, 0.45, 0.35, 0.0005]))
    corridor = float(rng.choice([0.36, 0.414, 0.45]))
    goals = [(reach * math.cos(phi), reach * math.sin(phi)) for phi in cand]
    ref = np.array([segment_blocked(P, gx, gy, corridor)[0] for gx, gy in goals], dtype=bool)
    new = segments_blocked(P, goals, corridor)
    cases += 1
    if not np.array_equal(ref, new):
        bad += 1
report("giong tung bit ban goc", bad == 0, f"({cases} canh x 101 huong, {bad} khac)")

# ── 3. Ban do luoi: khoang cach toi vat ─────────────────────────────────────────────────────────────────
print("3. grid_min_dist2 (ban do luoi)")
nav_lethal, nav_soft, nav_soft_cost = 0.30, 0.25, 3.0
r_in = HW + 0.06
cutoff = max(nav_lethal, r_in + nav_soft)
bad_d = bad_c = 0
cases = 0
for _ in range(120):
    res = float(rng.choice([0.06, 0.08]))
    dist = float(rng.uniform(0.5, 4.0))
    x0, x1 = -1.0, float(np.clip(dist + 1.2, 2.0, 4.5))
    ey = float(np.clip(dist + 1.0, 1.8, 3.5))
    nxc = int(math.ceil((x1 - x0) / res))
    nyc = int(math.ceil(2.0 * ey / res))
    xs = x0 + (np.arange(nxc) + 0.5) * res
    ys = -ey + (np.arange(nyc) + 0.5) * res
    GX, GY = np.meshgrid(xs.astype(np.float32), ys.astype(np.float32))
    m = int(rng.integers(0, 420))
    Q = (scene(m) if m > 0 else np.zeros((0, 2))).astype(np.float32)
    # --- ban goc (_nav_plan) ---
    d2 = np.full(GX.shape, np.inf, dtype=np.float32)
    for i in range(0, Q.shape[0], 48):
        q = Q[i:i + 48]
        dd = (GX[..., None] - q[:, 0]) ** 2 + (GY[..., None] - q[:, 1]) ** 2
        d2 = np.minimum(d2, dd.min(axis=2))
    new_d2 = grid_min_dist2(GX, GY, Q, cutoff)
    cases += 1
    dref, dnew = np.sqrt(d2), np.sqrt(new_d2)
    near = dref < cutoff
    if not (np.array_equal(dref[near], dnew[near]) and np.all(dnew[~near] >= cutoff)):
        bad_d += 1
    # o cam + chi phi o (dung nhu _nav_plan) phai giong het
    lr, ln = dref < nav_lethal, dnew < nav_lethal
    cr = 1.0 + nav_soft_cost * np.clip((r_in + nav_soft - dref) / nav_soft, 0.0, 1.5) ** 2
    cn = 1.0 + nav_soft_cost * np.clip((r_in + nav_soft - dnew) / nav_soft, 0.0, 1.5) ** 2
    if not (np.array_equal(lr, ln) and np.array_equal(cr, cn)):
        bad_c += 1
report("khoang cach < nguong giong tung bit", bad_d == 0, f"({cases} luoi, {bad_d} khac)")
report("o cam + chi phi o giong het", bad_c == 0, f"({cases} luoi, {bad_c} khac)")

print()
print("=> DAT" if ok_all else "=> LOI")
sys.exit(0 if ok_all else 1)
