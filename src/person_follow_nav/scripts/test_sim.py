"""Kiem thu offline: gia lap rclpy de import duoc node, roi chay mo phong 2D."""
import math
import sys
import types
import os

import numpy as np

# ── Stub cac module ROS ─────────────────────────────────────────────────────
def _mk(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _Pub:
    def publish(self, *a, **k):
        pass


class _Param:
    def __init__(self, v):
        self.value = v


class _FakeNode:
    def __init__(self, name):
        self._p = {}

    def declare_parameter(self, n, v, descriptor=None):
        self._p[n] = v

    def get_parameter(self, n):
        return _Param(self._p[n])

    def create_subscription(self, *a, **k):
        return None

    def create_publisher(self, *a, **k):
        return _Pub()

    def create_service(self, *a, **k):
        return None

    def create_timer(self, *a, **k):
        return None

    def get_logger(self):
        class L:
            def info(self, *a, **k):
                pass
            def warn(self, *a, **k):
                pass
            def warning(self, *a, **k):
                pass
            def error(self, *a, **k):
                pass
        return L()

    def get_clock(self):
        class C:
            def now(self):
                class T:
                    def to_msg(self):
                        return None
                return T()
        return C()

    def destroy_node(self):
        return True


class _Vec:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Twist:
    def __init__(self):
        self.linear = _Vec()
        self.angular = _Vec()


class _Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Color:
    def __init__(self, r=0.0, g=0.0, b=0.0, a=1.0):
        self.r, self.g, self.b, self.a = r, g, b, a


class _Str:
    def __init__(self, data=""):
        self.data = data


class _Marker:
    ADD = 0
    DELETE = 2
    SPHERE = 2
    CYLINDER = 3
    LINE_STRIP = 4

    def __init__(self):
        self.header = types.SimpleNamespace(frame_id="", stamp=None)
        self.pose = types.SimpleNamespace(
            position=_Point(), orientation=types.SimpleNamespace(w=1.0)
        )
        self.scale = _Vec()
        self.color = _Color()
        self.points = []


class _MarkerArray:
    def __init__(self):
        self.markers = []


_mk("rclpy", init=lambda **k: None, shutdown=lambda: None, spin=lambda n: None, ok=lambda: True)
_mk("rclpy.node", Node=_FakeNode)
_mk("rclpy.qos", qos_profile_sensor_data=None)
_mk("rcl_interfaces")
_mk("rcl_interfaces.msg", ParameterDescriptor=lambda **k: None)
_mk("geometry_msgs")
_mk("geometry_msgs.msg", Twist=_Twist, Point=_Point)
_mk("nav_msgs")
_mk("nav_msgs.msg", Odometry=object)
_mk("sensor_msgs")
_mk("sensor_msgs.msg", LaserScan=object)
_mk("std_msgs")
_mk("std_msgs.msg", String=_Str, Float32=object, ColorRGBA=_Color)
_mk("std_srvs")
_mk("std_srvs.srv", Trigger=types.SimpleNamespace(Request=object, Response=object))
_mk("visualization_msgs")
_mk("visualization_msgs.msg", Marker=_Marker, MarkerArray=_MarkerArray)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from person_follow_nav.follow_planner_node import FollowPlannerNode  # noqa: E402
from person_follow_nav.geometry import (  # noqa: E402
    AlphaBeta2D, rect_clearance, scan_to_base_points, segment_blocked, wrap_pi,
)

# ════════════════════════════════════════════════════════════════════════════
# TEST 1 — geometry: doi goc lidar
# ════════════════════════════════════════════════════════════════════════════
print("=" * 70)
print("TEST 1 — scan_to_base_points: goc lidar -> base_link")
print("=" * 70)

# Gia lap sc_mini: 360 tia, angle_min=0, angle_max=2pi
n = 360
inc = 2 * math.pi / n
ranges = np.full(n, 5.0)
# vat can o goc lidar 90 do (theo SYSTEM_CONTEXT = mui xe)
ranges[90] = 0.8
# vat can o goc lidar 180 do (phai la ben TRAI xe neu offset=-90)
ranges[180] = 1.5

pts, bear, rng, ldeg = scan_to_base_points(
    ranges, 0.0, inc, 0.1, 10.0,
    lidar_x=0.0, lidar_y=0.0,
    lidar_yaw_offset_rad=math.radians(-90.0), lidar_angle_sign=1.0,
    max_use_range=8.0,
)
i08 = int(np.argmin(np.abs(rng - 0.8)))
i15 = int(np.argmin(np.abs(rng - 1.5)))
b08 = math.degrees(bear[i08])
b15 = math.degrees(bear[i15])
print(f"  vat can lidar 90deg  -> base_link {b08:+7.2f} deg  (ky vong ~0, truoc mat)")
print(f"  vat can lidar 180deg -> base_link {b15:+7.2f} deg  (ky vong ~+90, ben trai)")
ok1 = abs(b08) < 1.0 and abs(b15 - 90.0) < 1.0
print(f"  => {'DAT' if ok1 else 'LOI'}")

# ════════════════════════════════════════════════════════════════════════════
# TEST 2 — rect_clearance: footprint chu nhat
# ════════════════════════════════════════════════════════════════════════════
print()
print("=" * 70)
print("TEST 2 — rect_clearance: xe 0.60 ngang x 0.47 doc")
print("=" * 70)
F, R, HW = 0.14, 0.33, 0.30
cases = [
    (0.50, 0.00, 0.36, "diem truoc mui 0.50m"),
    (0.00, 0.40, 0.10, "diem ben canh 0.40m"),
    (-0.50, 0.00, 0.17, "diem SAU duoi 0.50m"),
    (0.10, 0.10, 0.00, "diem BEN TRONG xe"),
]
ok2 = True
for x, y, expect, name in cases:
    got = float(rect_clearance(np.array([x]), np.array([y]), F, R, HW)[0])
    good = abs(got - expect) < 0.01
    ok2 &= good
    print(f"  {name:28s} -> {got:.3f}m (ky vong {expect:.3f})  {'DAT' if good else 'LOI'}")
print(f"  => {'DAT' if ok2 else 'LOI'}")

# So sanh voi mo hinh hinh tron
r_circ = max(math.hypot(F, HW), math.hypot(R, HW))
print(f"\n  [So sanh] Ban kinh ngoai tiep = {r_circ:.3f}m  (quyet dinh boi goc SAU)")
print(f"  Khe rong 0.85m: mo hinh HINH TRON can {2*r_circ:.2f}m -> TUONG LA KHONG LOT")
print(f"                  mo hinh CHU NHAT can {2*HW:.2f}m     -> LOT DUOC (du {0.85-2*HW:.2f}m)")

# ════════════════════════════════════════════════════════════════════════════
# TEST 3 — segment_blocked
# ════════════════════════════════════════════════════════════════════════════
print()
print("=" * 70)
print("TEST 3 — segment_blocked: phat hien nguoi chen vao giua duong")
print("=" * 70)
obs_clear = np.array([[1.0, 2.0], [-1.0, 0.5]])
obs_block = np.array([[1.0, 0.05], [2.0, 3.0]])
b1, d1 = segment_blocked(obs_clear, 2.0, 0.0, 0.31)
b2, d2 = segment_blocked(obs_block, 2.0, 0.0, 0.31)
print(f"  duong trong        -> bi_chan={b1}  (ky vong False)")
print(f"  co nguoi tai 1.0m  -> bi_chan={b2} tai {d2:.2f}m  (ky vong True, ~1.0m)")
ok3 = (not b1) and b2 and abs(d2 - 1.0) < 0.05
print(f"  => {'DAT' if ok3 else 'LOI'}")

# ════════════════════════════════════════════════════════════════════════════
# TEST 4 — AlphaBeta2D: du doan khi bi che
# ════════════════════════════════════════════════════════════════════════════
print()
print("=" * 70)
print("TEST 4 — AlphaBeta2D: nguoi di 1.0 m/s, bi che 1.5s")
print("=" * 70)
f = AlphaBeta2D(alpha=0.55, beta=0.10, max_speed=1.8)
t = 0.0
for k in range(40):          # 2s @ 20Hz, nguoi di theo +x voi 1.0 m/s
    t += 0.05
    f.update(1.0 * t, 0.0, t)
print(f"  Sau 2s theo doi: vi tri=({f.x:.2f}, {f.y:.2f})  van toc={f.vx:.2f} m/s (ky vong ~1.0)")
px, py = f.predict(t + 1.5)
err = abs(px - 1.0 * (t + 1.5))
print(f"  Du doan them 1.5s bi che: x={px:.2f}  that={1.0*(t+1.5):.2f}  sai so={err:.2f}m")
ok4 = abs(f.vx - 1.0) < 0.15 and err < 0.30
print(f"  => {'DAT' if ok4 else 'LOI'}")

# ════════════════════════════════════════════════════════════════════════════
# TEST 5 — MO PHONG DAY DU: xe bam nguoi, co nguoi chen ngang
# ════════════════════════════════════════════════════════════════════════════
print()
print("=" * 70)
print("TEST 5 — MO PHONG: bam nguoi + co nguoi la chen vao giua")
print("=" * 70)


def make_planner():
    p = FollowPlannerNode.__new__(FollowPlannerNode)
    _FakeNode.__init__(p, "x")
    FollowPlannerNode._declare_params(p)
    FollowPlannerNode._read_params(p)
    p.enabled = True
    p.state = "FOLLOW"
    p.prev_state = "FOLLOW"
    p.state_since = 0.0
    p.cur_v = 0.0
    p.cur_w = 0.0
    p.avoid_side = 0
    p.avoid_side_time = 0.0
    p.blocked_since = 0.0
    p.clear_since = 0.0
    p.obstacles = np.zeros((0, 2))
    p.last_heading = 0.0
    p.t_arr = np.arange(1, p.horizon_steps + 1, dtype=np.float64) * p.horizon_dt
    return p


P = make_planner()

# The gioi: nguoi muc tieu di thang tren truc x tu (3.0, 0) ra xa voi 0.35 m/s
# Mot nguoi LA dung yen tai (2.0, 0.0) -> chen dung giua duong
# Mot buc tuong doc theo y = +1.1 va y = -1.1 (hanh lang rong 2.2m)
rx, ry, ryaw = 0.0, 0.0, 0.0
person = np.array([3.0, 0.0])
person_v = np.array([0.15, 0.0])
intruder = np.array([2.0, 0.0])

dt = 1.0 / 15.0
log = []
min_gap_to_intruder = 9.9
reached_behind = False

for step in range(420):          # 28 giay
    tsim = step * dt
    person = person + person_v * dt

    # Xay dung "quet lidar" trong base_link: nguoi la + 2 buc tuong + muc tieu
    obs = []
    # nguoi la: vong tron ban kinh 0.20m
    for a in np.linspace(0, 2 * math.pi, 18, endpoint=False):
        obs.append(intruder + 0.20 * np.array([math.cos(a), math.sin(a)]))
    # muc tieu cung la vat can vat ly
    for a in np.linspace(0, 2 * math.pi, 14, endpoint=False):
        obs.append(person + 0.20 * np.array([math.cos(a), math.sin(a)]))
    # hai buc tuong
    for xw in np.arange(-1.0, 6.0, 0.10):
        obs.append(np.array([xw, 1.10]))
        obs.append(np.array([xw, -1.10]))
    obs = np.array(obs)

    # doi sang base_link
    d = obs - np.array([rx, ry])
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    base_obs = np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]], axis=1)
    # chi giu diem trong tam lidar 5m
    base_obs = base_obs[np.hypot(base_obs[:, 0], base_obs[:, 1]) < 5.0]
    P.obstacles = base_obs

    # muc tieu trong base_link
    dt_p = person - np.array([rx, ry])
    tx = c * dt_p[0] - s * dt_p[1]
    ty = s * dt_p[0] + c * dt_p[1]
    dist = math.hypot(tx, ty)
    bearing = math.atan2(ty, tx)

    goal_r = max(0.0, dist - P.follow_distance)
    gx = goal_r * math.cos(bearing)
    gy = goal_r * math.sin(bearing)

    corridor = P.half_width * P.block_corridor_scale + P.margin_hard
    blocked, _ = segment_blocked(P.obstacles, gx, gy, corridor)
    if blocked:
        P.clear_since = 0.0
        if P.blocked_since == 0.0:
            P.blocked_since = tsim
        if tsim - P.blocked_since >= P.block_enter_sec:
            if P.avoid_side == 0:
                P.avoid_side = P._choose_avoid_side(bearing)
                P.avoid_side_time = tsim
            P.state = "AVOID"
    else:
        P.blocked_since = 0.0
        if P.clear_since == 0.0:
            P.clear_since = tsim
        if P.state == "AVOID" and (tsim - P.clear_since) >= P.block_exit_sec:
            P.avoid_side = 0
            P.state = "FOLLOW"

    if goal_r <= P.dist_deadband:
        v_cmd, w_cmd = 0.0, float(np.clip(1.5 * bearing, -0.5, 0.5))
        clearance = None
    else:
        phi, reach, _ = P._choose_heading(bearing, goal_r, bearing)
        if phi is None:
            v_cmd, w_cmd, clearance = 0.0, 0.35 * (P.avoid_side or 1), None
            P.state = "BLOCKED"
        else:
            sr = min(reach, goal_r)
            res = P._dwa((sr * math.cos(phi), sr * math.sin(phi)), (tx, ty), dt)
            if res is None:
                v_cmd, w_cmd, clearance = 0.0, 0.35 * (P.avoid_side or 1), None
                P.state = "BLOCKED"
            else:
                v_cmd, w_cmd, clearance = res

    # gioi han gia toc (giong _emit nhung khong publish)
    v_cmd = float(np.clip(v_cmd, P.cur_v - P.accel_lin * dt, P.cur_v + P.accel_lin * dt))
    w_cmd = float(np.clip(w_cmd, P.cur_w - P.accel_ang * dt, P.cur_w + P.accel_ang * dt))
    P.cur_v, P.cur_w = v_cmd, w_cmd

    # tich phan chuyen dong xe
    rx += v_cmd * math.cos(ryaw) * dt
    ry += v_cmd * math.sin(ryaw) * dt
    ryaw = wrap_pi(ryaw + w_cmd * dt)

    gap = math.hypot(rx - intruder[0], ry - intruder[1]) - 0.20
    min_gap_to_intruder = min(min_gap_to_intruder, gap)
    if rx > intruder[0] + 0.1:
        reached_behind = True

    if step % 30 == 0:
        log.append(
            (tsim, rx, ry, math.degrees(ryaw), dist, math.degrees(bearing),
             v_cmd, w_cmd, P.state, P.avoid_side)
        )

print("   t(s)   xe_x   xe_y  yaw   d_nguoi  goc    v      w     trang thai")
for (t_, x_, y_, yw, dd, bb, vv, ww, st, sd) in log:
    print(f"  {t_:5.1f} {x_:6.2f} {y_:6.2f} {yw:5.0f}  {dd:6.2f} {bb:6.1f} "
          f"{vv:6.3f} {ww:6.3f}  {st}{'' if sd == 0 else ('/trai' if sd > 0 else '/phai')}")

final_d = math.hypot(person[0] - rx, person[1] - ry)
print()
print(f"  Khoang cach cuoi toi nguoi: {final_d:.2f}m  (muc tieu {P.follow_distance:.2f}m)")
print(f"  Khoang cach gan nhat toi nguoi chen: {min_gap_to_intruder:.3f}m  "
      f"(gioi han cung {P.margin_hard:.2f}m)")
print(f"  Da vuot qua duoc nguoi chen: {reached_behind}")
ok5 = reached_behind and min_gap_to_intruder > 0.05 and final_d < 1.8
print(f"  => {'DAT' if ok5 else 'LOI'}")

# ════════════════════════════════════════════════════════════════════════════
# TEST 6 — Khe hep: xe 0.48m phai chui qua khe 0.75m
# ════════════════════════════════════════════════════════════════════════════
print()
print("=" * 70)
print("TEST 6 — KHE HEP: xe rong 0.60m chui khe 0.85m")
print("=" * 70)

P2 = make_planner()
rx, ry, ryaw = 0.0, 0.0, 0.0
gap_half = 0.425                  # khe rong 0.85m
target = np.array([3.5, 0.0])

obs_w = []
for yy in np.arange(gap_half, 2.0, 0.06):
    obs_w.append([1.8, yy])
    obs_w.append([1.9, yy])
    obs_w.append([-yy + 0.0, 0.0])  # placeholder bo qua
obs_w = [o for o in obs_w if o[1] != 0.0]
for yy in np.arange(gap_half, 2.0, 0.06):
    obs_w.append([1.8, -yy])
    obs_w.append([1.9, -yy])
wall = np.array(obs_w)

min_clear_seen = 9.9
passed = False
for step in range(500):
    d = wall - np.array([rx, ry])
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    base_obs = np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]], axis=1)
    P2.obstacles = base_obs[np.hypot(base_obs[:, 0], base_obs[:, 1]) < 5.0]

    dp = target - np.array([rx, ry])
    tx = c * dp[0] - s * dp[1]
    ty = s * dp[0] + c * dp[1]
    dist = math.hypot(tx, ty)
    bearing = math.atan2(ty, tx)
    goal_r = max(0.0, dist - P2.follow_distance)
    if goal_r < 0.05:
        break
    gx, gy = goal_r * math.cos(bearing), goal_r * math.sin(bearing)

    phi, reach, _ = P2._choose_heading(bearing, goal_r, bearing)
    if phi is None:
        v_cmd, w_cmd = 0.0, 0.3
    else:
        sr = min(reach, goal_r)
        res = P2._dwa((sr * math.cos(phi), sr * math.sin(phi)), (tx, ty), dt)
        if res is None:
            v_cmd, w_cmd = 0.0, 0.3
        else:
            v_cmd, w_cmd, cl = res
            min_clear_seen = min(min_clear_seen, cl)

    v_cmd = float(np.clip(v_cmd, P2.cur_v - P2.accel_lin * dt, P2.cur_v + P2.accel_lin * dt))
    w_cmd = float(np.clip(w_cmd, P2.cur_w - P2.accel_ang * dt, P2.cur_w + P2.accel_ang * dt))
    P2.cur_v, P2.cur_w = v_cmd, w_cmd
    rx += v_cmd * math.cos(ryaw) * dt
    ry += v_cmd * math.sin(ryaw) * dt
    ryaw = wrap_pi(ryaw + w_cmd * dt)
    if rx > 2.2:
        passed = True
        break

print(f"  Chui qua khe: {passed}   vi tri cuoi=({rx:.2f}, {ry:.2f})  sau {step} buoc")
print(f"  Do thoang nho nhat khi qua khe: {min_clear_seen:.3f}m")
print(f"  (margin_soft={P2.margin_soft:.2f}m la mong muon, margin_hard={P2.margin_hard:.2f}m la tuyet doi)")
ok6 = passed and min_clear_seen > 0.0
print(f"  => {'DAT' if ok6 else 'LOI'}")

print()
print("=" * 70)
allok = all([ok1, ok2, ok3, ok4, ok5, ok6])
print(f"TONG KET: {'TAT CA DAT' if allok else 'CO TEST LOI'}")
print("=" * 70)
