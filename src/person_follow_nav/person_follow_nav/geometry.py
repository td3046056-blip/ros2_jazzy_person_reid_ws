"""
geometry.py
===========
Ham dung chung cho ca target_tracker_node va follow_planner_node.

Quy uoc khung toa do (RẤT QUAN TRỌNG — sai cai nay la xe chay lung tung):

  base_link:  +x = MUI XE (thang truoc)
              +y = BEN TRAI xe
              +yaw (angular.z duong) = QUAY TRAI

  LaserScan cua sc_mini:  angle_min = 0.0, angle_max = 2*pi (KHONG phai -pi..pi!)
              => tia thu i co goc a_laser = angle_min + i*angle_increment,  a_laser in [0, 2pi)
              => goc nay CHUA chac ung voi mui xe. Phai hieu chinh bang
                 lidar_yaw_offset_deg (xem scripts/calibrate_lidar_front.py).

  Cong thuc doi goc:  a_base = lidar_angle_sign * a_laser + lidar_yaw_offset_rad

              lidar_angle_sign = +1.0 neu lidar lap NGUA (mat quet huong len)
              lidar_angle_sign = -1.0 neu lidar lap UP NGUOC (bi guong)

  camera_angle_deg trong /person_reid/target:
              duong = nguoi o BEN PHAI khung hinh = ben phai xe = goc AM trong base_link
              => bearing_rad = -radians(camera_angle_deg)   (camera_angle_sign = -1.0)
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

TWO_PI = 2.0 * math.pi


# ─────────────────────────────────────────────────────────────────────────────
# Goc
# ─────────────────────────────────────────────────────────────────────────────

def wrap_pi(angle: float) -> float:
    """Dua goc ve khoang [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def wrap_pi_np(a: np.ndarray) -> np.ndarray:
    """Ban vector hoa cua wrap_pi."""
    return np.arctan2(np.sin(a), np.cos(a))


def angle_diff(a: float, b: float) -> float:
    """Hieu goc ngan nhat a - b, ket qua trong [-pi, pi]."""
    return wrap_pi(a - b)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Trich yaw tu quaternion (khong can tf_transformations)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ─────────────────────────────────────────────────────────────────────────────
# LaserScan -> diem trong base_link
# ─────────────────────────────────────────────────────────────────────────────

def scan_to_base_points(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    lidar_x: float,
    lidar_y: float,
    lidar_yaw_offset_rad: float,
    lidar_angle_sign: float,
    max_use_range: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Doi LaserScan thanh mang diem (M,2) trong base_link.

    Returns
    -------
    pts       : (M, 2) toa do [x, y] trong base_link, met
    bear      : (M,)   bearing so voi goc base_link, rad in [-pi, pi]
    rng       : (M,)   khoang cach tu goc base_link toi diem, met
    lidar_deg : (M,)   goc goc cua tia trong KHUNG LIDAR, do in [0, 360)

    Luu y: rng KHONG phai range tho cua lidar — no da tinh lai tu goc base_link,
    nen neu lidar lech tam xe thi van dung.
    """
    r = np.asarray(ranges, dtype=np.float64)
    n = r.shape[0]
    if n == 0:
        empty = np.zeros((0, 2), dtype=np.float64)
        return empty, np.zeros(0), np.zeros(0), np.zeros(0)

    idx = np.arange(n, dtype=np.float64)
    a_laser = angle_min + idx * angle_increment

    lo = max(float(range_min), 0.02)
    hi = min(float(range_max), float(max_use_range))
    valid = np.isfinite(r) & (r > lo) & (r < hi)
    if not np.any(valid):
        empty = np.zeros((0, 2), dtype=np.float64)
        return empty, np.zeros(0), np.zeros(0), np.zeros(0)

    r = r[valid]
    a_laser = a_laser[valid]

    a_base = lidar_angle_sign * a_laser + lidar_yaw_offset_rad
    px = lidar_x + r * np.cos(a_base)
    py = lidar_y + r * np.sin(a_base)

    pts = np.stack([px, py], axis=1)
    bear = np.arctan2(py, px)
    rng = np.hypot(px, py)
    # Goc trong KHUNG LIDAR (do), can cho self_filter_mask(blind_sectors_deg)
    lidar_deg = np.degrees(a_laser) % 360.0
    return pts, bear, rng, lidar_deg


def downsample_polar(
    pts: np.ndarray,
    bear: np.ndarray,
    rng: np.ndarray,
    bucket_deg: float = 2.0,
    keep_all_within_m: float = 1.0,
) -> np.ndarray:
    """Giam so diem de DWA chay nhanh, nhung KHONG lam mat vat can.

    Chia 360 do thanh cac o rong bucket_deg, moi o chi giu diem GAN NHAT.
    Rieng cac diem gan hon keep_all_within_m thi giu HET (an toan la uu tien).
    """
    if pts.shape[0] == 0:
        return pts

    near_mask = rng <= keep_all_within_m
    near_pts = pts[near_mask]

    bucket = max(0.5, float(bucket_deg))
    nb = int(math.ceil(360.0 / bucket))
    b_idx = ((np.degrees(bear) % 360.0) / bucket).astype(np.int64)
    b_idx = np.clip(b_idx, 0, nb - 1)

    # Voi moi bucket lay diem co rng nho nhat: sort theo (bucket, rng) roi lay dau moi nhom
    order = np.lexsort((rng, b_idx))
    b_sorted = b_idx[order]
    first = np.ones(b_sorted.shape[0], dtype=bool)
    first[1:] = b_sorted[1:] != b_sorted[:-1]
    far_pts = pts[order][first]

    if near_pts.shape[0] == 0:
        return far_pts
    out = np.vstack([near_pts, far_pts])
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Phan cum (clustering) — dung de tach "chan nguoi" khoi tuong/vat can
# ─────────────────────────────────────────────────────────────────────────────

class Cluster:
    __slots__ = ("cx", "cy", "bearing", "range_m", "width_m", "count", "min_range")

    def __init__(self, cx: float, cy: float, width_m: float, count: int, min_range: float) -> None:
        self.cx = cx
        self.cy = cy
        self.bearing = math.atan2(cy, cx)
        self.range_m = math.hypot(cx, cy)
        self.width_m = width_m
        self.count = count
        self.min_range = min_range

    def __repr__(self) -> str:  # pragma: no cover - debug only
        return (f"Cluster(r={self.range_m:.2f}m, bear={math.degrees(self.bearing):.1f}deg, "
                f"w={self.width_m:.2f}m, n={self.count})")


def cluster_points(
    pts: np.ndarray,
    bear: np.ndarray,
    rng: np.ndarray,
    bearing_center: float,
    bearing_half_window: float,
    gap_threshold_m: float = 0.18,
    min_points: int = 2,
) -> List[Cluster]:
    """Gom cac diem lidar trong mot cung goc thanh cac cum lien tuc.

    Hai diem ke nhau (theo goc) thuoc cung cum neu khoang cach Euclid < gap_threshold_m.
    Dung de tim "nguoi" trong cung goc ma camera bao.
    """
    if pts.shape[0] == 0:
        return []

    d = wrap_pi_np(bear - bearing_center)
    sel = np.abs(d) <= bearing_half_window
    if not np.any(sel):
        return []

    sub_pts = pts[sel]
    sub_bear = bear[sel]
    sub_rng = rng[sel]

    order = np.argsort(sub_bear)
    sub_pts = sub_pts[order]
    sub_rng = sub_rng[order]

    clusters: List[Cluster] = []
    start = 0
    for i in range(1, sub_pts.shape[0] + 1):
        split = True
        if i < sub_pts.shape[0]:
            step = math.hypot(
                sub_pts[i, 0] - sub_pts[i - 1, 0],
                sub_pts[i, 1] - sub_pts[i - 1, 1],
            )
            split = step > gap_threshold_m
        if split:
            seg = sub_pts[start:i]
            if seg.shape[0] >= min_points:
                cx = float(np.mean(seg[:, 0]))
                cy = float(np.mean(seg[:, 1]))
                width = float(math.hypot(seg[-1, 0] - seg[0, 0], seg[-1, 1] - seg[0, 1]))
                clusters.append(
                    Cluster(cx, cy, width, int(seg.shape[0]), float(np.min(sub_rng[start:i])))
                )
            start = i
    return clusters


# ─────────────────────────────────────────────────────────────────────────────
# Kiem tra va cham theo footprint chu nhat (KHONG dung hinh tron)
# ─────────────────────────────────────────────────────────────────────────────

def self_filter_mask(
    pts: np.ndarray,
    bear_lidar_deg: np.ndarray,
    front_len: float,
    rear_len: float,
    half_width: float,
    margin: float = 0.03,
    blind_sectors_deg: Optional[List[List[float]]] = None,
) -> np.ndarray:
    """Tra ve mask True cho cac diem CAN GIU (khong phai than xe).

    TAI SAO BAT BUOC PHAI CO:
    LiDAR quet 360 do nen no thay ca cot do, day dien, mep san cua CHINH XE.
    Nhung diem do nam ben trong footprint. rect_clearance() tra ve 0.0 cho diem
    ben trong footprint, tuc la "da va cham". Ket qua: MOI quy dao deu bi loai,
    _dwa() tra None moi chu ky, va xe ket vinh vien o trang thai BLOCKED.

    Loc hai lop:
      1. Bo moi diem roi vao trong footprint (thu nho lai margin de khong bo nham
         vat can that dang cham vao mep xe).
      2. Bo cac cung goc co dinh do nguoi dung khai bao (blind_sectors_deg),
         tinh trong KHUNG LIDAR — dung cho cot do cao hon mep footprint.

    Lop 1 an toan vi: vat can that khong the "xuat hien" ben trong footprint ma
    truoc do khong di qua bien footprint — luc o bien no da duoc nhin thay va
    xu ly roi.
    """
    if pts.shape[0] == 0:
        return np.zeros(0, dtype=bool)

    keep = np.ones(pts.shape[0], dtype=bool)

    fl = max(0.0, front_len - margin)
    rl = max(0.0, rear_len - margin)
    hw = max(0.0, half_width - margin)
    inside = (
        (pts[:, 0] <= fl) & (pts[:, 0] >= -rl)
        & (np.abs(pts[:, 1]) <= hw)
    )
    keep &= ~inside

    if blind_sectors_deg:
        d = bear_lidar_deg % 360.0
        for sec in blind_sectors_deg:
            if sec is None or len(sec) < 2:
                continue
            lo = float(sec[0]) % 360.0
            hi = float(sec[1]) % 360.0
            if lo <= hi:
                keep &= ~((d >= lo) & (d <= hi))
            else:                       # cung goc vong qua moc 0/360
                keep &= ~((d >= lo) | (d <= hi))
    return keep


def rect_clearance(
    local_x: np.ndarray,
    local_y: np.ndarray,
    front_len: float,
    rear_len: float,
    half_width: float,
) -> np.ndarray:
    """Khoang cach tu MEP footprint chu nhat toi diem, theo he toa do robot.

    local_x, local_y: toa do diem vat can trong he cua robot tai mot tu the.
    Tra ve 0.0 neu diem nam TRONG footprint (va cham).

    Dung chu nhat that thay vi hinh tron giup xe 46x57cm chui duoc khe hep
    ma xe hinh tron ban kinh 0.37m se tuong la khong lot.
    """
    dxb = np.maximum(-rear_len - local_x, local_x - front_len)
    dyb = np.maximum(-half_width - local_y, local_y - half_width)
    ox = np.maximum(dxb, 0.0)
    oy = np.maximum(dyb, 0.0)
    return np.sqrt(ox * ox + oy * oy)


def segment_blocked(
    pts: np.ndarray,
    goal_x: float,
    goal_y: float,
    corridor_half_width: float,
    step_m: float = 0.10,
) -> Tuple[bool, float]:
    """Kiem tra duong THANG tu goc base_link toi (goal_x, goal_y) co bi chan khong.

    Tra ve (bi_chan, khoang_cach_toi_diem_chan_gan_nhat).
    Dung de quyet dinh khi nao chuyen tu FOLLOW sang AVOID.
    """
    if pts.shape[0] == 0:
        return False, float("inf")

    length = math.hypot(goal_x, goal_y)
    if length < 1e-3:
        return False, float("inf")

    ux = goal_x / length
    uy = goal_y / length

    # Chieu cac diem len truc di chuyen
    proj = pts[:, 0] * ux + pts[:, 1] * uy          # doc theo duong
    perp = -pts[:, 0] * uy + pts[:, 1] * ux         # vuong goc duong

    inside = (proj > 0.05) & (proj < length) & (np.abs(perp) < corridor_half_width)
    if not np.any(inside):
        return False, float("inf")
    return True, float(np.min(proj[inside]))


# ─────────────────────────────────────────────────────────────────────────────
# Bo loc alpha-beta 2D cho vi tri muc tieu
# ─────────────────────────────────────────────────────────────────────────────

class AlphaBeta2D:
    """Bo loc alpha-beta (Kalman rut gon) cho muc tieu chuyen dong deu trong odom.

    Dung thay Kalman day du vi: it tham so hon, khong can ma tran, du chinh xac
    cho nguoi di bo (< 1.5 m/s), va de tune bang tay.
    """

    def __init__(self, alpha: float = 0.55, beta: float = 0.12, max_speed: float = 2.0) -> None:
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.max_speed = float(max_speed)
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.initialized = False
        self.last_time: Optional[float] = None

    def reset(self) -> None:
        self.initialized = False
        self.vx = 0.0
        self.vy = 0.0
        self.last_time = None

    def predict(self, now: float) -> Tuple[float, float]:
        """Du doan vi tri toi thoi diem now (KHONG cap nhat trang thai noi bo)."""
        if not self.initialized or self.last_time is None:
            return self.x, self.y
        dt = max(0.0, now - self.last_time)
        return self.x + self.vx * dt, self.y + self.vy * dt

    def step(self, now: float) -> None:
        """Day trang thai toi thoi diem now bang mo hinh van toc khong doi."""
        if not self.initialized or self.last_time is None:
            self.last_time = now
            return
        dt = max(0.0, now - self.last_time)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.last_time = now

    def update(self, mx: float, my: float, now: float) -> None:
        """Cap nhat bang mot phep do moi (mx, my) trong odom."""
        if not self.initialized or self.last_time is None:
            self.x, self.y = float(mx), float(my)
            self.vx = self.vy = 0.0
            self.initialized = True
            self.last_time = now
            return

        dt = max(1e-3, now - self.last_time)
        px = self.x + self.vx * dt
        py = self.y + self.vy * dt

        rx = float(mx) - px
        ry = float(my) - py

        self.x = px + self.alpha * rx
        self.y = py + self.alpha * ry
        self.vx += (self.beta / dt) * rx
        self.vy += (self.beta / dt) * ry

        sp = math.hypot(self.vx, self.vy)
        if sp > self.max_speed and sp > 1e-6:
            k = self.max_speed / sp
            self.vx *= k
            self.vy *= k

        self.last_time = now
