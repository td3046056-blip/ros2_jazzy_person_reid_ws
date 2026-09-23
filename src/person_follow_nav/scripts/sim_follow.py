"""
sim_follow.py — mo phong VONG KIN bam nguoi, khong can ROS.

Chay dung code THAT cua target_tracker_node + follow_planner_node (tham so tu
config/follow_nav.yaml), ghep voi:
  - xe vi sai bam lenh /cmd_vel (tre bac nhat 0.1 s)
  - lidar gia 10 Hz: 2 chan nguoi dang buoc + tuong phong + than xe (lidar 270-290)
  - camera gia 8 Hz, tre 150 ms, chi thay nguoi trong ±CAM_HALF do va khong bi tuong che

Dung de do truoc/sau khi sua thuat toan bam nguoi. KHONG thay the test tren xe that.

CACH DUNG
---------
  cd src/person_follow_nav/scripts
  python3 sim_follow.py thang                 # in bang chi tiet + dong chi so
  python3 sim_follow.py -q thang re_trai_gat  # chi in dong chi so

  Bien moi truong:
    CAM_HALF=25        camera chi nhan ra nguoi trong ±25 do (mac dinh 31). Ngoai doi
                       ReID thuong mat nguoi truoc mep khung.
    CORNER=1           them tuong y=+1.0 tu x=-1 toi x=CORNER_X (mac dinh 3.0) — dung cho goc_tuong
    CORRIDOR=1         them hai vach doc hai ben dan vao cua (CORRIDOR_W, mac dinh 0.9 m)
    SIDE_DOOR="x0:rong:y"  tuong DOC theo y=const co o cua — nguoi di thang roi RE vao cua
    START="x:y:yaw_do"     tu the xuat phat cua xe (de thu rieng dong tac chui cua)
    HIDE_LEGS_DEG=25   lidar khong thay chan nguoi khi lech qua goc nay (ep lidar_track that bai)
    PKG_DIR=...        chay voi ban package khac (vd. ban cu lay tu git) de so sanh

  Vi du da dung khi sua 17/09 (xem CLAUDE.md muc 7.2):
    CAM_HALF=25 python3 sim_follow.py -q thang re_trai_gat re_phai_gat ne_ngang_nhanh cheo_thoat
    CAM_HALF=25 CORNER=1 python3 sim_follow.py -q goc_tuong
    CAM_HALF=25 HIDE_LEGS_DEG=25 python3 sim_follow.py -q thoat_khung
    CAM_HALF=25 python3 sim_follow.py -q chan_giua cat_ngang chan_sat   # nguoi thu hai (22/09)
    CAM_HALF=25 CAM_OCCLUDE=1 python3 sim_follow.py -q cat_ngang_gan    # di ngang che camera
    CAM_HALF=25 python3 sim_follow.py -q sau_ne_re                      # ne xong nguoi re gat
    CAM_HALF=25 DOOR=3.2:0.81 python3 sim_follow.py -q qua_cua qua_cua_lech qua_cua_cheo  # cua that 0.81 m
    CAM_HALF=25 DOOR=3.2:0.81 CORRIDOR=1 python3 sim_follow.py -q qua_cua_cheo   # co hanh lang dan vao
    CAM_HALF=25 SIDE_DOOR=3.0:0.81:-0.9 python3 sim_follow.py -q cua_ben   # re vao cua ben hong
    CAM_HALF=25 SIDE_DOOR=3.0:0.81:-0.9 python3 sim_follow.py -q cua_ben_dung  # re vao roi dung lai
    # Thu RIENG dong tac chui cua (nguoi dung yen ben kia, xe xuat phat lech truc/lech goc):
    CAM_HALF=25 SIDE_DOOR=3.0:0.81:-0.9 START="3.1:0.0:-75" python3 sim_follow.py -q cua_ben_yen

Moi lan chay deu in "ho_nho_nhat_voi_tuong": khoang ho THAT giua footprint chu nhat cua xe
va tuong (<= 0 la da cham). Co DOOR/SIDE_DOOR thi in them do thoang va goc lech truc cua
DUNG LUC XE DANG TRONG KHUNG CUA — do la con so quyet dinh xe co ca vao khung cua khong.
    CAM_OCCLUDE=1 ...  nguoi thu hai che ca camera khi dung tren tia nhin toi muc tieu

  Moi kich ban mat 1-3 phut. Chi so: |goc| = lech giua mui xe va nguoi THAT (do),
  ngoai_khung = % thoi gian nguoi ngoai ±CAM_HALF, doi_chieu_w = so lan lenh xoay doi
  chieu (cang nhieu cang lac), SEARCH = co vao che do tim nguoi khong.
"""
import json
import math
import os
import sys
import types

import numpy as np

ORIG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.environ.get("PKG_DIR", ORIG)

# stub ROS lay tu test_sim.py (phan truoc sys.path.insert)
_stub = open(os.path.join(ORIG, "scripts/test_sim.py")).read().split("sys.path.insert")[0]
exec(compile(_stub, "stubs", "exec"))
sys.path.insert(0, PKG)

import yaml  # noqa: E402
import person_follow_nav.target_tracker_node as TT  # noqa: E402
import person_follow_nav.follow_planner_node as FP  # noqa: E402
from person_follow_nav import geometry as G  # noqa: E402

CLK = types.SimpleNamespace(t=1000.0)
_ft = types.SimpleNamespace(time=lambda: CLK.t, monotonic=lambda: CLK.t, sleep=lambda s: None)
TT.time = _ft
FP.time = _ft

CFG = yaml.safe_load(open(os.path.join(PKG, "config/follow_nav.yaml")))
RNG = np.random.default_rng(1)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def make_nodes():
    tr = TT.TargetTrackerNode()
    tr._p.update(CFG["target_tracker_node"]["ros__parameters"])
    tr._read_params()
    tr.filter = G.AlphaBeta2D(alpha=tr.ab_alpha, beta=tr.ab_beta, max_speed=tr.max_person_speed)
    pl = FP.FollowPlannerNode()
    pl._p.update(CFG["follow_planner_node"]["ros__parameters"])
    pl._read_params()
    pl.t_arr = np.arange(1, pl.horizon_steps + 1, dtype=np.float64) * pl.horizon_dt
    pl.enabled = True
    return tr, pl


# ── The gioi ──────────────────────────────────────────────────────────────
WALLS = []  # doan thang (x1,y1,x2,y2)
def room(x0, y0, x1, y1):
    WALLS.extend([(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)])
room(-4.0, -7.0, 14.0, 7.0)
if os.environ.get("CORNER"):
    WALLS.append((-1.0, 1.0, float(os.environ.get("CORNER_X", "3.0")), 1.0))
DOOR = None
if os.environ.get("DOOR"):
    # DOOR="x:rong" — tuong ngang qua x, chua mot cua rong `rong` o giua (y = 0)
    _dx, _dw = (float(v) for v in os.environ["DOOR"].split(":"))
    WALLS.extend([(_dx, -6.0, _dx, -_dw / 2), (_dx, _dw / 2, _dx, 6.0)])
    if os.environ.get("CORRIDOR"):
        # HANH_LANG=1: them hai vach doc hai ben dan vao cua (giong luc di RA cua that:
        # hai ben co vat chan doc nen xe bi ep thang hang truoc khi toi cua)
        _cw = float(os.environ.get("CORRIDOR_W", "0.9")) / 2
        WALLS.extend([(_dx - 2.0, _cw, _dx, _cw), (_dx - 2.0, -_cw, _dx, -_cw)])
    DOOR = (_dx, _dw)
SIDE_DOOR = None
if os.environ.get("SIDE_DOOR"):
    # SIDE_DOOR="x0:rong:y" — tuong DOC theo y=const, co o cua tu x0 den x0+rong.
    # Giong cua that cua nguoi dung: dang di thang roi RE vao cua ben hong.
    _sx, _sw, _sy = (float(v) for v in os.environ["SIDE_DOOR"].split(":"))
    WALLS.extend([(-2.0, _sy, _sx, _sy), (_sx + _sw, _sy, 9.0, _sy)])
    SIDE_DOOR = (_sx, _sw, _sy)
WALLS_A = np.array(WALLS)


def _wall_points(step=0.02):
    """Roi rac hoa tuong thanh diem — de do khoang ho THAT cua footprint chu nhat."""
    out = []
    for (x1, y1, x2, y2) in WALLS_A:
        L = math.hypot(x2 - x1, y2 - y1)
        k = max(2, int(L / step) + 1)
        u = np.linspace(0.0, 1.0, k)
        out.append(np.stack([x1 + (x2 - x1) * u, y1 + (y2 - y1) * u], axis=1))
    return np.concatenate(out) if out else np.zeros((0, 2))


WALL_PTS = _wall_points()


def wall_clearance(rx, ry, yaw, F, R, HW):
    """Khoang ho nho nhat tu footprint chu nhat toi tuong. <= 0 la DA CHAM."""
    if WALL_PTS.shape[0] == 0:
        return 10.0
    dx = WALL_PTS[:, 0] - rx
    dy = WALL_PTS[:, 1] - ry
    near = (np.abs(dx) < 1.5) & (np.abs(dy) < 1.5)
    if not np.any(near):
        return 10.0
    dx, dy = dx[near], dy[near]
    c, s = math.cos(yaw), math.sin(yaw)
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return float(np.min(G.rect_clearance(lx, ly, F, R, HW)))


def raycast(ox, oy, ang, circles):
    """ang: mang goc the gioi. Tra ve khoang cach (inf neu khong trung)."""
    dx, dy = np.cos(ang), np.sin(ang)
    best = np.full(ang.shape, np.inf)
    for (x1, y1, x2, y2) in WALLS_A:
        ex, ey = x2 - x1, y2 - y1
        den = dx * (-ey) - dy * (-ex)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x1 - ox) * (-ey) - (y1 - oy) * (-ex)) / den
            u = (dx * (y1 - oy) - dy * (x1 - ox)) / den
        ok = (np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(ok & (t < best), t, best)
    for (cx, cy, r) in circles:
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        with np.errstate(invalid="ignore"):
            t = -b - np.sqrt(disc)
        ok = (disc >= 0) & (t > 0)
        best = np.where(ok & (t < best), t, best)
    return best


class Person:
    def __init__(self, path):
        self.path = path          # ham t -> (x, y, heading, speed)

    def legs(self, t):
        x, y, h, sp = self.path(t)
        swing = 0.10 * math.sin(2 * math.pi * 1.0 * t) if sp > 0.05 else 0.0
        c, s = math.cos(h), math.sin(h)
        # hai chan cach nhau 0.22 m ngang, dung/xoay truoc sau khi di
        return [(x + c * swing - s * 0.11, y + s * swing + c * 0.11, 0.06),
                (x - c * swing + s * 0.11, y - s * swing - c * 0.11, 0.06)]


def run(name, path, T, report_every=0.5, verbose=True, intruder=None):
    tr, pl = make_nodes()
    person = Person(path)
    other = Person(intruder) if intruder else None
    _st = os.environ.get("START")
    if _st:
        _sx, _sy, _syaw = (float(v) for v in _st.split(":"))
        rob = dict(x=_sx, y=_sy, yaw=math.radians(_syaw), v=0.0, w=0.0, cv=0.0, cw=0.0)
    else:
        rob = dict(x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0, cv=0.0, cw=0.0)

    def cmd_cb(msg):
        rob["cv"], rob["cw"] = msg.linear.x, msg.angular.z
    pl.pub_cmd.publish = cmd_cb
    last = {}
    def tgt_cb(msg):
        last["t"] = json.loads(msg.data)
        pl._target_cb(msg)
    tr.pub_target.publish = tgt_cb
    status = {}
    pl.pub_status.publish = lambda m: status.update(json.loads(m.data))

    cam_queue = []   # (deliver_time, payload)
    dt = 1.0 / 600.0
    t0 = CLK.t
    rows = []
    hit = dict(clear=10.0, t=0.0, x=0.0, y=0.0, yaw=0.0)
    gate = dict(clear=10.0, yaw_err=0.0, n=0)
    n = int(T / dt)
    for k in range(n):
        CLK.t = t0 + k * dt
        t = k * dt
        # dong hoc xe: bam lenh voi tre bac nhat 0.1 s
        a = dt / 0.10
        rob["v"] += a * (rob["cv"] - rob["v"])
        rob["w"] += a * (rob["cw"] - rob["w"])
        rob["x"] += rob["v"] * math.cos(rob["yaw"]) * dt
        rob["y"] += rob["v"] * math.sin(rob["yaw"]) * dt
        rob["yaw"] = wrap(rob["yaw"] + rob["w"] * dt)

        if k % 10 == 0:      # do khoang ho voi tuong o 60 Hz
            wc = wall_clearance(rob["x"], rob["y"], rob["yaw"], pl.front_len, pl.rear_len, pl.half_width)
            if wc < hit["clear"]:
                hit.update(clear=wc, t=t, x=rob["x"], y=rob["y"], yaw=math.degrees(rob["yaw"]))
            # Rieng LUC DANG TRONG KHUNG CUA: do khoang ho va goc lech so voi phap tuyen cua
            if SIDE_DOOR is not None:
                _sx, _sw, _sy = SIDE_DOOR
                if abs(rob["y"] - _sy) < 0.28 and _sx - 0.15 < rob["x"] < _sx + _sw + 0.15:
                    gate.update(clear=min(gate["clear"], wc), n=gate["n"] + 1,
                                yaw_err=max(gate["yaw_err"],
                                            abs(math.degrees(wrap(rob["yaw"] + math.pi / 2)))))
            if DOOR is not None:
                _dx, _dw = DOOR
                if abs(rob["x"] - _dx) < 0.28 and abs(rob["y"]) < _dw / 2 + 0.15:
                    gate.update(clear=min(gate["clear"], wc), n=gate["n"] + 1,
                                yaw_err=max(gate["yaw_err"], abs(math.degrees(wrap(rob["yaw"])))))

        px, py, ph, psp = person.path(t)
        ipos = intruder(t) if intruder else None     # nguoi thu hai (x, y, huong, toc do) hoac None
        rel_b = wrap(math.atan2(py - rob["y"], px - rob["x"]) - rob["yaw"])
        rel_d = math.hypot(px - rob["x"], py - rob["y"])

        if k % 12 == 0:      # odom 50 Hz
            od = types.SimpleNamespace(pose=types.SimpleNamespace(pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=rob["x"], y=rob["y"], z=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(rob["yaw"] / 2), w=math.cos(rob["yaw"] / 2)))))
            tr._odom_cb(od)
            pl._odom_cb(od)
        if k % 60 == 0:      # lidar 10 Hz
            lx = rob["x"] + 0.10 * math.cos(rob["yaw"])
            ly = rob["y"] + 0.10 * math.sin(rob["yaw"])
            world = rob["yaw"] + np.radians(np.arange(360.0) - 90.0)
            hide = os.environ.get("HIDE_LEGS_DEG") and abs(rel_b) > math.radians(float(os.environ["HIDE_LEGS_DEG"])) and t > 3.0
            circles = [] if hide else person.legs(t)
            if ipos is not None:
                circles = circles + other.legs(t)
            rr = raycast(lx, ly, world, circles)
            rr = rr + RNG.normal(0, 0.01, rr.shape)
            rr[270:290] = 0.128                      # than xe
            rr[rr > 10.0] = np.inf
            scan = types.SimpleNamespace(ranges=rr.tolist(), angle_min=0.0,
                                         angle_increment=math.radians(1.0), range_min=0.10, range_max=10.0)
            tr._scan_cb(scan)
            pl._scan_cb(scan)
        if k % 75 == 0:      # camera 8 Hz, chup luc nay, giao sau 150 ms
            blk = raycast(rob["x"], rob["y"], np.array([math.atan2(py - rob["y"], px - rob["x"])]), [])[0] < rel_d - 0.05
            if ipos is not None and os.environ.get("CAM_OCCLUDE"):
                # nguoi thu hai che camera neu dung gan tia nhin toi muc tieu
                ux, uy = (px - rob["x"]) / rel_d, (py - rob["y"]) / rel_d
                qx, qy = ipos[0] - rob["x"], ipos[1] - rob["y"]
                along = qx * ux + qy * uy
                blk = blk or (0.0 < along < rel_d and abs(-qx * uy + qy * ux) < 0.25)
            found = (not blk) and abs(rel_b) <= math.radians(float(os.environ.get("CAM_HALF", "31"))) and 0.4 < rel_d < 8.0
            ang = -math.degrees(rel_b) + RNG.normal(0, 0.8)
            h = 420.0 / max(0.3, rel_d)
            payload = {"camera_angle_deg": ang if found else None, "bbox": [0, 0, 60, h] if found else None,
                       "target_found": found, "identity_ready": True, "status": "locked" if found else "lost",
                       "ts": CLK.t}
            cam_queue.append((CLK.t + 0.15, payload))
        while cam_queue and cam_queue[0][0] <= CLK.t:
            tr._cam_cb(_Str(json.dumps(cam_queue.pop(0)[1])))
        if k % 30 == 0:      # tracker 20 Hz
            tr._update()
        if k % 40 == 0:      # planner 15 Hz
            pl._control()

        if k % int(report_every / dt) == 0:
            tj = last.get("t", {})
            eb = tj.get("bearing_deg")
            rows.append(dict(t=t, rx=rob["x"], ry=rob["y"], yaw=math.degrees(rob["yaw"]), d=rel_d,
                             b=math.degrees(rel_b), eb=eb, src=tj.get("source"), st=status.get("state"),
                             note=status.get("note"),
                             v=rob["cv"], w=rob["cw"], spd=tj.get("speed"), px=px, py=py,
                             ex=tj.get("odom_x"), ey=tj.get("odom_y"),
                             ix=None if ipos is None else ipos[0], iy=None if ipos is None else ipos[1],
                             rgap=None if ipos is None else math.hypot(ipos[0] - rob["x"], ipos[1] - rob["y"])))
    if verbose:
        print(f"\n=== {name} ===")
        print("   t    xe_x  xe_y   yaw | d_that goc_that goc_uoc | nguon         trang_thai |   v     w    v_nguoi_uoc"
              + (" | sai_uoc  xe->ng2" if intruder else ""))
        for r in rows:
            eb = "  -  " if r["eb"] is None else f"{r['eb']:6.1f}"
            extra = ""
            if intruder:
                err = "  -  " if r["ex"] is None else f"{math.hypot(r['ex'] - r['px'], r['ey'] - r['py']):5.2f}"
                extra = f" | {err}  " + ("  -  " if r["rgap"] is None else f"{r['rgap']:5.2f}")
            print(f"{r['t']:5.1f} {r['rx']:5.2f} {r['ry']:5.2f} {r['yaw']:6.1f} | {r['d']:5.2f} {r['b']:7.1f} {eb} | "
                  f"{str(r['src']):13s} {str(r['st']):9s} | {r['v']:5.2f} {r['w']:5.2f}  {r['spd']}{extra}"
                  f" | {str(r['note'])[:44]}")
    hit["gate"] = gate
    return rows, hit


# ── Kich ban ──────────────────────────────────────────────────────────────
def straight(t, v=0.20, x0=1.6):
    return (x0 + v * t, 0.0, 0.0, v)


def turn(t, side=+1, v=0.20, x0=1.6, t1=5.0, R=1.2):
    """Di thang t1 giay, re 90 do ban kinh R, roi di thang."""
    if t < t1:
        return (x0 + v * t, 0.0, 0.0, v)
    ang_len = (math.pi / 2) * R / v
    xs = x0 + v * t1
    if t < t1 + ang_len:
        th = (t - t1) * v / R
        return (xs + R * math.sin(th), side * R * (1 - math.cos(th)), side * th, v)
    s = (t - t1 - ang_len) * v
    return (xs + R, side * (R + s), side * math.pi / 2, v)


def sidestep(t, v_side=1.0, dur=1.5, t1=4.0, x0=1.5):
    """Dung truoc xe, roi buoc nhanh sang ngang (TRAI) roi dung han."""
    if t < t1:
        return (x0, 0.0, math.pi / 2, 0.0)
    s = min(t - t1, dur) * v_side
    sp = v_side if t - t1 < dur else 0.0
    return (x0, s, math.pi / 2, sp)


def diag(t, v=0.8, t1=3.0, dur=2.5, x0=1.4, ang=math.radians(60)):
    """Dung truoc xe roi di cheo 60 do sang trai 0.8 m/s trong 2.5 s roi dung."""
    if t < t1:
        return (x0, 0.0, ang, 0.0)
    s = min(t - t1, dur) * v
    return (x0 + s * math.cos(ang), s * math.sin(ang), ang, v if t - t1 < dur else 0.0)


def corner(t, v=0.5, x0=1.4, xt=3.4, R=0.3, walk=5.0):
    """Di thang +x toi x=xt, re TRAI 90 do (ban kinh R) vao sau goc tuong, di tiep +y roi dung."""
    t1 = (xt - x0) / v
    if t < t1:
        return (x0 + v * t, 0.0, 0.0, v)
    al = (math.pi / 2) * R / v
    if t < t1 + al:
        th = (t - t1) * v / R
        return (xt + R * math.sin(th), R * (1 - math.cos(th)), th, v)
    s = min(t - t1 - al, walk) * v
    return (xt + R, R + s, math.pi / 2, v if t - t1 - al < walk else 0.0)


def blocker(t, x=2.2, t_in=5.0, stay=6.0, v=1.0, y0=1.6):
    """Nguoi thu hai: buoc tu ben TRAI vao dung chan tai (x, 0) `stay` giay roi buoc ra ben PHAI."""
    tw = y0 / v
    if t < t_in:
        return None
    if t < t_in + tw:
        return (x, y0 - v * (t - t_in), -math.pi / 2, v)
    if t < t_in + tw + stay:
        return (x, 0.0, -math.pi / 2, 0.0)
    if t < t_in + 2 * tw + stay:
        return (x, -v * (t - t_in - tw - stay), -math.pi / 2, v)
    return None


def straight_then_turn(t, v1=0.20, x0=1.6, t1=12.0, v2=0.35, R=0.5, side=+1):
    """Di thang cham toi t1 roi re gat 90 do (ban kinh R) nhanh hon, di tiep."""
    if t < t1:
        return (x0 + v1 * t, 0.0, 0.0, v1)
    xs = x0 + v1 * t1
    al = (math.pi / 2) * R / v2
    if t < t1 + al:
        th = (t - t1) * v2 / R
        return (xs + R * math.sin(th), side * R * (1 - math.cos(th)), side * th, v2)
    s_ = (t - t1 - al) * v2
    return (xs + R, side * (R + s_), side * math.pi / 2, v2)


def side_door(t, v=0.25, x0=1.4, y0=0.0, x_turn=3.4, y_end=-2.6, R=0.45):
    """Di thang +x theo hanh lang, den x_turn thi RE PHAI (ban kinh R) qua o cua ben hong."""
    d1 = x_turn - R - x0
    if v * t < d1:
        return (x0 + v * t, y0, 0.0, v)
    al = (math.pi / 2) * R
    s_ = v * t - d1
    if s_ < al:
        th = s_ / R
        return (x_turn - R + R * math.sin(th), y0 - R * (1 - math.cos(th)), -th, v)
    yy = max(y_end, y0 - R - (s_ - al))
    return (x_turn, yy, -math.pi / 2, v if yy > y_end else 0.0)


def door_diag(t, v=0.25, x0=1.4, y0=-0.55, x_align=2.9, x_end=6.0):
    """Di CHEO tu (x0, y0) toi tam cua (x_align, 0) roi di thang qua cua."""
    dx, dy = x_align - x0, -y0
    leg = math.hypot(dx, dy)
    if v * t < leg:
        k = v * t / leg
        return (x0 + k * dx, y0 + k * dy, math.atan2(dy, dx), v)
    x = min(x_align + (v * t - leg), x_end)
    return (x, 0.0, 0.0, v if x < x_end else 0.0)


def through_door(t, v=0.25, x0=1.6, y=0.0, x_end=6.0):
    """Di thang qua cua (DOOR) theo duong y, toi x_end thi dung."""
    x = min(x0 + v * t, x_end)
    return (x, y, 0.0, v if x < x_end else 0.0)


SCEN = {
    "thang": (lambda t: straight(t), 24.0),
    "re_trai": (lambda t: turn(t, +1), 30.0),
    "re_phai": (lambda t: turn(t, -1), 30.0),
    "ne_ngang_nhanh": (lambda t: sidestep(t, 1.0, 1.5), 16.0),
    "ne_ngang_vua": (lambda t: sidestep(t, 0.6, 2.0), 16.0),
    "re_trai_gat": (lambda t: turn(t, +1, v=0.5, x0=1.4, t1=2.0, R=0.6), 14.0),
    "re_phai_gat": (lambda t: turn(t, -1, v=0.5, x0=1.4, t1=2.0, R=0.6), 14.0),
    "cheo_thoat": (lambda t: diag(t), 14.0),
    "goc_tuong": (lambda t: corner(t), 22.0),
    "thoat_khung": (lambda t: sidestep(t, 1.2, 1.2, t1=4.0, x0=1.5), 16.0),
    "goc_tuong_rong": (lambda t: corner(t, v=0.35, x0=1.4, xt=3.2, R=0.6, walk=4.0), 26.0),
    # nguoi thu hai buoc vao giua xe va muc tieu dang di thang (4.3/4.4)
    "chan_giua": (lambda t: straight(t), 20.0, lambda t: blocker(t)),
    # nguoi thu hai di cat ngang khong dung lai (4.3)
    "cat_ngang": (lambda t: straight(t), 16.0, lambda t: blocker(t, stay=0.0)),
    # nguoi thu hai dung sat truoc muc tieu (0.4 m) — vung cong chan nhay khong phan biet duoc
    "chan_sat": (lambda t: straight(t, v=0.0, x0=2.6), 16.0, lambda t: blocker(t, x=2.2, t_in=4.0, stay=5.0)),
    # 22/09 — nguoi thu hai di cat ngang SAT truoc muc tieu, cham (nen chay kem CAM_OCCLUDE=1)
    "cat_ngang_gan": (lambda t: straight(t), 18.0, lambda t: blocker(t, x=2.55, t_in=5.0, stay=0.0, v=0.6)),
    # vong qua nguoi thu hai xong, muc tieu re trai gat va nhanh hon
    "sau_ne_re": (lambda t: straight_then_turn(t), 26.0, lambda t: blocker(t)),
    # qua khung cua (chay kem DOOR=3.2:0.9)
    "qua_cua": (lambda t: through_door(t), 26.0),
    "qua_cua_lech": (lambda t: through_door(t, y=0.22), 26.0),
    # 23/09 — nguoi di CHEO toi cua roi qua cua: xe toi cua o the khong thang hang
    "qua_cua_cheo": (lambda t: door_diag(t), 30.0),
    # 23/09 — nguoi di thang roi RE PHAI vao o cua ben hong (chay kem SIDE_DOOR=3.0:0.81:-0.9)
    "cua_ben": (lambda t: side_door(t), 34.0),
    # 23/09 lan 2 — nguoi di CHAM hon xe nen xe bam kip, camera con thay nguoi khi vao cua:
    # dung kich ban that cua nguoi dung (nguon camera+lidar, trang thai AVOID o khung cua)
    "cua_ben_cham": (lambda t: side_door(t, v=0.18), 40.0),
    # Nguoi re vao cua roi DUNG LAI phia trong: camera van thay nguoi qua o cua nen xe o
    # nhanh chui khe (nguon camera+lidar, trang thai AVOID) — dung nhu nguoi dung quan sat.
    "cua_ben_dung": (lambda t: side_door(t, v=0.16, y_end=-2.2), 45.0),
    # Nguoi DUNG YEN ben kia cua — de thu rieng dong tac chui cua tu nhieu tu the xuat phat
    # (dung kem START="x:y:yaw" va SIDE_DOOR). Khong lan voi chuyen mat nguoi.
    "cua_ben_yen": (lambda t: (3.405, -2.4, -math.pi / 2, 0.0), 30.0),
}


def metrics(name, rows, hit):
    half = float(os.environ.get("CAM_HALF", "31"))
    t = np.array([r["t"] for r in rows]); b = np.array([r["b"] for r in rows])
    yaw = np.array([r["yaw"] for r in rows]); d = np.array([r["d"] for r in rows])
    w = np.array([r["w"] for r in rows]); src = [r["src"] for r in rows]; st = [r["st"] for r in rows]
    m = t > 2.0
    sgn = np.sign(np.where(np.abs(w) < 1e-3, 0.0, w))
    nz = sgn[sgn != 0]
    flips = int(np.sum(nz[1:] != nz[:-1])) if nz.size > 1 else 0
    cam = np.array([s is not None and s.startswith("camera") for s in src])
    pred = np.array([s == "predicted" for s in src])
    # lan mat camera dai nhat
    run_, best = 0.0, 0.0
    for c in cam[m]:
        run_ = 0.0 if c else run_ + 0.1
        best = max(best, run_)
    print(f"{name:15s} |goc|tb={np.mean(np.abs(b[m])):5.1f} max={np.max(np.abs(b[m])):5.1f} "
          f"ngoai_khung={100*np.mean(np.abs(b[m])>half):4.0f}% camera={100*np.mean(cam[m]):4.0f}% "
          f"predicted={100*np.mean(pred[m]):4.0f}% mat_cam_dai_nhat={best:4.1f}s "
          f"doi_chieu_w={flips:3d} d_cuoi={d[-1]:4.2f} yaw_cuoi={yaw[-1]:6.1f} goc_cuoi={b[-1]:6.1f} "
          f"SEARCH={'co' if 'SEARCH' in st else 'khong'} tt_cuoi={st[-1]}")
    tag = "CHAM TUONG" if hit["clear"] <= 0.0 else ("sat mep" if hit["clear"] < 0.03 else "khong cham")
    print(f"{'':15s} ho_nho_nhat_voi_tuong={hit['clear']:5.2f} m ({tag}) luc t={hit['t']:4.1f}s "
          f"tai ({hit['x']:.2f},{hit['y']:.2f}) yaw={hit['yaw']:6.1f}")
    g_ = hit["gate"]
    if g_["n"]:
        gtag = "CHAM" if g_["clear"] <= 0.0 else ("sat mep" if g_["clear"] < 0.03 else "ok")
        print(f"{'':15s} LUC TRONG KHUNG CUA: ho={g_['clear']:5.2f} m ({gtag}), "
              f"lech truc cua toi da={g_['yaw_err']:4.1f} do")
    if SIDE_DOOR is not None:
        sx, sw, sy = SIDE_DOOR
        posts = [(sx, sy), (sx + sw, sy)]
        pgap = min(math.hypot(r["rx"] - px_, r["ry"] - py_) for r in rows for (px_, py_) in posts)
        perr = [math.hypot(r["ex"] - r["px"], r["ey"] - r["py"]) for r in rows if r["ex"] is not None]
        y_min = min(r["ry"] for r in rows)
        qua = y_min < sy - 0.15      # tam xe da qua han mat cua
        print(f"{'':15s} qua cua ben: {'CO' if qua else 'KHONG'} (xe_y xa nhat={y_min:.2f}, can < {sy - 0.15:.2f}), "
              f"tam_xe_gan_thanh_cua_nhat={pgap:.2f} m, sai_uoc_max={max(perr) if perr else float('nan'):.2f} m")
    if DOOR is not None:
        dx, dw = DOOR
        jambs = [(dx, dw / 2), (dx, -dw / 2)]
        jgap = min(math.hypot(r["rx"] - jx, r["ry"] - jy) for r in rows for (jx, jy) in jambs)
        jerr = [math.hypot(r["ex"] - r["px"], r["ey"] - r["py"]) for r in rows if r["ex"] is not None]
        print(f"{'':15s} qua cua: {'CO' if rows[-1]['rx'] > dx + 0.4 else 'KHONG'} (xe_x cuoi={rows[-1]['rx']:.2f}), "
              f"tam_xe_gan_thanh_cua_nhat={jgap:.2f} m, sai_uoc_max={max(jerr) if jerr else float('nan'):.2f} m")
    occ = [r for r in rows if r["ix"] is not None and r["ex"] is not None]
    if occ:
        err = [math.hypot(r["ex"] - r["px"], r["ey"] - r["py"]) for r in occ]
        wrong = [math.hypot(r["ex"] - r["ix"], r["ey"] - r["iy"]) < math.hypot(r["ex"] - r["px"], r["ey"] - r["py"])
                 for r in occ]
        print(f"{'':15s} khi co nguoi thu hai: sai_uoc_max={max(err):4.2f} m, uoc_gan_ng2_hon={100*np.mean(wrong):4.0f}%, "
              f"xe_gan_ng2_nhat={min(r['rgap'] for r in occ):4.2f} m (tam xe->tam nguoi)")


if __name__ == "__main__":
    args = sys.argv[1:]
    quiet = "-q" in args
    names = [a for a in args if a != "-q"] or list(SCEN)
    for nm in names:
        f, T = SCEN[nm][0], SCEN[nm][1]
        intr = SCEN[nm][2] if len(SCEN[nm]) > 2 else None
        rows, hit = run(nm, f, T, report_every=0.1, verbose=not quiet, intruder=intr)
        metrics(nm, rows, hit)
