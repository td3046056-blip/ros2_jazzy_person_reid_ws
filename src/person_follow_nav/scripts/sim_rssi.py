#!/usr/bin/env python3
"""
sim_rssi.py — mo phong OFFLINE thuat toan RSSI de TIM LAI nguoi khi mat camera (30/09).
Khong can ROS, khong dung toi tracker/planner: chon thuat toan va do hieu qua truoc khi viet node.

MO HINH — khop so do that 30/09 (CLAUDE.md muc 8):
  * 3 board trong base_link: A (0.12, 0) dau xe; B (-0.20, -0.24) sau PHAI; C (-0.20, +0.24) sau TRAI
    (tam giac can canh 40/40/48 cm)
  * Bong than xe theo goc nguoi nhin tu xe (noi suy tuyen tinh giua cac moc 45 do):
      - truoc +-45 do: B va C NHU NHAU (do: C-B o trai 45 ~ phai 45)
      - +-90 do: C-B = +-12 dB (do bang gia do)
      - sau lung 135..180: CHUA DO -> hai bien the SAU=manh / SAU=yeu
  * Offset board A -6.9 / B +1.4 / C +5.4 dB (do tren ban); nguoi deo lam C lech -5 dB (do)
  * Suy hao -62 - 24 log10(d) dBm (khop -63..-66 dBm o 1.5 m, nguoi deo sau that lung)
  * Phan xa: truong Rician theo vi tri anten so voi nguoi, RIENG cho 3 kenh quang ba (2402/2426/2480 MHz):
    tu nhien co ca ba hien tuong da do — kenh lech nhau hon 10 dB, dung yen thi sai so "dong bang",
    xoay/di thi fading doi
  * Scanner doi kenh moi 50 ms, ~20 mau/s/board, mat mau duoi nguong nghe (~ -95 dBm)
  * Nguoi quay mat ve xe: A -19 dB, B/C -7 dB (do)

THUAT TOAN SO SANH (cung mot luong du lieu):
  tinh     C-B trong 1 s (tru offset ban), nguong 8 dB -> quay 90 do sang ben do; khong thi coi o truoc
  hai      xoay do: pha song hai bac nhat theo yaw (rssi_rotate.py), huong nhin board mac dinh
  hai_cal  nhu tren, huong nhin board do tu MOT lan hieu chinh o CHO KHAC (rssi_rotate.py --calib)
  mau      xoay do: so khop CA HINH DANG mau lay tu lan hieu chinh do — dung duoc ca khi xoay thieu vong

CACH DUNG
  python3 sim_rssi.py            # ca hai phan
  python3 sim_rssi.py huong      # chi uoc luong huong (mo vong)
  python3 sim_rssi.py vongkin    # chi vong kin "do - di"
  N=100 python3 sim_rssi.py      # so lan thu moi dieu kien (mac dinh 200)
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rssi_rotate import (ALPHA_DEG, build_template, est_template,  # noqa: E402
                         est_template_moving, estimate as estimate_harmonic)

BOARDS = ("A", "B", "C")
POS = {"A": (0.12, 0.0), "B": (-0.20, -0.24), "C": (-0.20, 0.24)}
OFF = {"A": -6.9, "B": 1.4, "C": 5.4}
BIAS_C = -5.0
FACING = {"A": -19.0, "B": -7.0, "C": -7.0}
FREQ = (2402e6, 2426e6, 2480e6)
FLOOR = -95.0
HZ = 20.0
CH_PHASE = {"A": 0, "B": 1, "C": 2}

# Moc bong than xe (dB) tai goc nguoi 0,45,90,135,180,-135,-90,-45 do (base_link, trai duong)
_ANCH = np.radians([0, 45, 90, 135, 180, 225, 270, 315, 360])
_TAB = {
    "manh": {"A": [5, 3, -2, -6, -8, -6, -2, 3], "C": [0, 0, 6, 7, 4, -6, -6, 0]},
    "yeu":  {"A": [5, 3, -2, -6, -8, -6, -2, 3], "C": [0, 0, 6, 3, 0, -3, -6, 0]},
}

CAM_HALF = math.radians(31.0)
MARGINS = (0.05, 0.10, 0.20)
V_DRIVE = 0.22
W_TURN = 0.8


@dataclass
class Cfg:
    d: float = 2.5            # khoang cach nguoi (m)
    w: float = 0.6            # toc do xoay do (rad/s)
    sweep: float = 360.0      # goc xoay do (do)
    walk: float = 0.0         # nguoi di vong quanh xe (m/s) trong luc xoay
    facing: bool = False      # nguoi quay mat ve xe
    K: float = 3.0            # he so Rician (nho = phan xa manh hon)
    rear: str = "manh"        # bong than xe phia sau: manh / yeu


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def gain(board: str, th: np.ndarray, rear: str) -> np.ndarray:
    tab = _TAB[rear]
    if board == "B":                      # doi xung voi C qua truc doc xe
        vals, th = tab["C"], -th
    else:
        vals = tab[board]
    return np.interp(np.mod(th, 2.0 * np.pi), _ANCH, vals + vals[:1])


class Field:
    """Truong phan xa Rician theo vector (anten - nguoi), rieng tung kenh quang ba."""

    def __init__(self, rng, K: float, n: int = 16) -> None:
        u = rng.uniform(0.0, 2.0 * np.pi, n)
        self.ux, self.uy = np.cos(u), np.sin(u)
        a = rng.rayleigh(1.0, n)
        self.a = a / np.sqrt((a ** 2).sum())
        self.ph = rng.uniform(0.0, 2.0 * np.pi, (3, n))
        self.los_ph = rng.uniform(0.0, 2.0 * np.pi, 3)
        self.k = 2.0 * np.pi * np.array(FREQ) / 3e8
        self.K = K

    def db(self, dx, dy, ch):
        k = self.k[ch]
        los = math.sqrt(self.K / (self.K + 1.0)) * np.exp(1j * (k * np.hypot(dx, dy) + self.los_ph[ch]))
        proj = dx[:, None] * self.ux[None, :] + dy[:, None] * self.uy[None, :]
        sc = math.sqrt(1.0 / (self.K + 1.0)) * (self.a[None, :] * np.exp(1j * (k[:, None] * proj + self.ph[ch]))).sum(1)
        return 20.0 * np.log10(np.abs(los + sc) + 1e-3)


def rx(board, t, rxp, ryp, ryaw, px, py, field, cfg, rng):
    """Mau RSSI board nhan duoc tai cac thoi diem t (mang), tra ve (t giu lai, rssi)."""
    bx, by = POS[board]
    c, s = np.cos(ryaw), np.sin(ryaw)
    ax, ay = rxp + c * bx - s * by, ryp + s * bx + c * by
    dxr, dyr = px - rxp, py - ryp
    d = np.maximum(0.3, np.hypot(dxr, dyr))
    th = np.arctan2(dyr, dxr) - ryaw
    ch = (np.floor(t / 0.05).astype(int) + CH_PHASE[board]) % 3
    r = (-62.0 - 24.0 * np.log10(d) + gain(board, th, cfg.rear) + OFF[board]
         + (BIAS_C if board == "C" else 0.0) + (FACING[board] if cfg.facing else 0.0)
         + field.db(ax - px, ay - py, ch) + rng.normal(0.0, 2.0, len(t)))
    keep = rng.random(len(t)) < np.clip((r - (FLOOR - 2.0)) / 4.0, 0.0, 1.0)
    return t[keep], np.round(r[keep])


# ─────────────────────────────── uoc luong ───────────────────────────────

def est_static(t, sid, rs, t0, t1) -> Optional[float]:
    """Huong (base_link) theo C-B khi xe dung yen: +90 / -90 / 0."""
    m = {}
    for s in ("B", "C"):
        v = rs[(sid == s) & (t >= t0) & (t < t1)]
        if len(v) < 3:
            return None
        m[s] = v.mean() - OFF[s]
    idx = m["C"] - m["B"]
    return math.pi / 2 if idx >= 8.0 else (-math.pi / 2 if idx <= -8.0 else 0.0)


def sweep_samples(rng, field, cfg, t0, t1, pose_fn, person_fn):
    """Sinh mau 3 board trong [t0, t1): pose_fn(t) -> (x, y, yaw), person_fn(t) -> (px, py)."""
    T, S, R = [], [], []
    for s in BOARDS:
        n = rng.poisson(HZ * (t1 - t0))
        t = np.sort(rng.uniform(t0, t1, n))
        if n == 0:
            continue
        x, y, yaw = pose_fn(t)
        px, py = person_fn(t)
        tk, rk = rx(s, t, x, y, yaw, px, py, field, cfg, rng)
        T.append(tk); S.append(np.full(len(tk), s)); R.append(rk)
    if not T:
        return np.zeros(0), np.zeros(0, dtype="<U1"), np.zeros(0)
    t = np.concatenate(T); o = np.argsort(t)
    return t[o], np.concatenate(S)[o], np.concatenate(R)[o]


def calibrate(rng, cfg: Cfg):
    """Mot lan hieu chinh o cho KHAC (truong phan xa khac): nguoi dung truoc mui 1.5 m, xe xoay 2 vong 0.4 rad/s."""
    field = Field(rng, cfg.K)
    ccfg = replace(cfg, walk=0.0, facing=False)
    w, T = 0.4, 4.0 * np.pi / 0.4
    t, sid, rs = sweep_samples(rng, field, ccfg, 0.0, T,
                               lambda tt: (np.zeros_like(tt), np.zeros_like(tt), w * tt),
                               lambda tt: (np.full_like(tt, 1.5), np.zeros_like(tt)))
    t_o = np.linspace(0.0, T, 2000)
    tmpl = build_template(t, sid, rs, t_o, w * t_o, 0.0)
    res = estimate_harmonic(t, sid, rs, t_o, w * t_o, {"A": 0.0, "B": 0.0, "C": 0.0})
    alpha = {k: math.degrees(wrap(0.0 - v["raw_arg"])) for k, v in res["boards"].items()} if res else dict(ALPHA_DEG)
    return tmpl, alpha


# ───────────────────────── phan 1: uoc luong huong ─────────────────────────

def trial_dir(rng, cfg: Cfg, cal) -> Dict[str, Tuple[Optional[float], float]]:
    field = Field(rng, cfg.K)
    phi0 = rng.uniform(-np.pi, np.pi)
    omega_p = (cfg.walk / cfg.d) * (1 if rng.random() < 0.5 else -1)
    person = lambda tt: (cfg.d * np.cos(phi0 + omega_p * tt), cfg.d * np.sin(phi0 + omega_p * tt))
    T = math.radians(cfg.sweep) / cfg.w
    pose = lambda tt: (np.zeros_like(tt), np.zeros_like(tt), np.where(tt < 1.0, 0.0, cfg.w * (tt - 1.0)))
    t, sid, rs = sweep_samples(rng, field, cfg, 0.0, 1.0 + T, pose, person)
    out = {}
    # tinh: 1 s dau, xe chua xoay (yaw 0)
    e = est_static(t, sid, rs, 0.0, 1.0)
    out["tinh"] = (None if e is None else math.degrees(abs(wrap(e - (phi0 + omega_p * 1.0)))), 1.0)
    truth = phi0 + omega_p * (1.0 + T)
    t_o = np.linspace(1.0, 1.0 + T, 1500)
    y_o = cfg.w * (t_o - 1.0)
    tmpl, alpha = cal
    for name, a in (("hai", ALPHA_DEG), ("hai_cal", alpha)):
        r = estimate_harmonic(t, sid, rs, t_o, y_o, a)
        out[name] = (None if r is None else math.degrees(abs(wrap(r["phi"] - truth))), T)
    e = est_template(t, sid, rs, t_o, y_o, tmpl)
    out["mau"] = (None if e is None else math.degrees(abs(wrap(e - truth))), T)
    e = est_template_moving(t, sid, rs, t_o, y_o, tmpl, 1.0 + T)
    out["mau_dong"] = (None if e is None else math.degrees(abs(wrap(e - truth))), T)
    for mg in MARGINS:
        e = est_template_moving(t, sid, rs, t_o, y_o, tmpl, 1.0 + T, margin=mg)
        out[f"kh{int(mg * 100)}"] = (None if e is None else math.degrees(abs(wrap(e - truth))), T)
    return out


def run_dir(n: int) -> None:
    rng = np.random.default_rng(3)
    base = Cfg()
    cases = [
        ("mac dinh: 2.5 m, xoay 0.6 rad/s 1 vong", base),
        ("nguoi gan 1.5 m", replace(base, d=1.5)),
        ("nguoi xa 4 m", replace(base, d=4.0)),
        ("nguoi xa 6 m", replace(base, d=6.0)),
        ("xoay cham 0.4 rad/s", replace(base, w=0.4)),
        ("xoay nhanh 0.9 rad/s", replace(base, w=0.9)),
        ("xoay 2 vong", replace(base, sweep=720.0)),
        ("chi xoay 270 do (thieu cho)", replace(base, sweep=270.0)),
        ("chi xoay 180 do (thieu cho)", replace(base, sweep=180.0)),
        ("nguoi di vong 0.4 m/s luc xoay", replace(base, walk=0.4)),
        ("nguoi quay mat ve xe", replace(base, facing=True)),
        ("phan xa manh (K=1)", replace(base, K=1.0)),
        ("bong sau lung yeu (chua do)", replace(base, rear="yeu")),
        ("xau nhat: 4 m, K=1, sau yeu, di 0.4", replace(base, d=4.0, K=1.0, rear="yeu", walk=0.4)),
    ]
    algs = ("tinh", "hai_cal", "mau", "mau_dong") + tuple(f"kh{int(m * 100)}" for m in MARGINS)
    cals = {}
    print(f"\n{'':38s}" + "".join(f"{a:>14s}" for a in algs))
    print(f"{'dieu kien':38s}" + "".join(f"{'<=30do|TVsai':>14s}" for _ in algs))
    for label, cfg in cases:
        key = (cfg.K, cfg.rear)
        if key not in cals:
            cals[key] = [calibrate(rng, cfg) for _ in range(5)]
        res = {a: [] for a in algs}
        tsec = {a: 0.0 for a in algs}
        for i in range(n):
            o = trial_dir(rng, cfg, cals[key][i % 5])
            for a in algs:
                res[a].append(180.0 if o[a][0] is None else o[a][0])
                tsec[a] = o[a][1]
        s = f"{label:38s}"
        for a in algs:
            e = np.array(res[a])
            s += f"{np.mean(e <= 30) * 100:7.0f}%|{np.median(e):5.1f}"
        print(s)
    print("  (trong30 = ti le sai <= 30 do: quay ve huong do thi nguoi nam trong khung camera +-31 do)")
    print("  (tinh mat 1 s; xoay do 1 vong o 0.6 rad/s mat ~10.5 s)")


# ───────────────────────── phan 2: vong kin "do - di" ─────────────────────────

D_CAM = float(os.environ.get("D_CAM", "2.0"))   # camera chi thay nguoi trong D_CAM m (xa hon bi che) — GIA DINH
T_MAX = 60.0


class Loop:
    """Xe dong hoc don gian (khong vat can), nguoi an, camera thay khi |goc| <= 31 do va <= D_CAM."""

    def __init__(self, rng, cfg: Cfg, wander: bool) -> None:
        self.rng, self.cfg = rng, cfg
        self.field = Field(rng, cfg.K)
        a = rng.uniform(-np.pi, np.pi)
        r = rng.uniform(2.5, 6.0)
        # quy dao nguoi luoi 0.05 s trong T_MAX
        n = int(T_MAX / 0.05) + 2
        self.tg = np.arange(n) * 0.05
        px, py = np.empty(n), np.empty(n)
        px[0], py[0] = r * math.cos(a), r * math.sin(a)
        h = rng.uniform(-np.pi, np.pi)
        v = 0.3 if wander else 0.0
        for i in range(1, n):
            h += rng.normal(0.0, 0.5 * math.sqrt(0.05))
            nx, ny = px[i - 1] + v * 0.05 * math.cos(h), py[i - 1] + v * 0.05 * math.sin(h)
            if math.hypot(nx, ny) > 8.0:
                h += math.pi
                nx, ny = px[i - 1], py[i - 1]
            px[i], py[i] = nx, ny
        self.px, self.py = px, py
        self.x = self.y = 0.0
        self.yaw = 0.0          # unwrapped
        self.t = 0.0
        self.logs: List[Tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        self.odom: List[Tuple[np.ndarray, np.ndarray]] = []
        self.found: Optional[float] = None

    def person(self, t):
        return np.interp(t, self.tg, self.px), np.interp(t, self.tg, self.py)

    def _run(self, dur, w, v) -> bool:
        """Chay mot doan (xoay w hoac di thang v) trong dur giay. True neu camera thay nguoi."""
        dur = min(dur, T_MAX - self.t)
        if dur <= 0:
            return True
        t0, x0, y0, yaw0 = self.t, self.x, self.y, self.yaw
        pose = lambda tt: (x0 + v * (tt - t0) * math.cos(yaw0), y0 + v * (tt - t0) * math.sin(yaw0), yaw0 + w * (tt - t0))
        tg = t0 + np.arange(0.0, dur, 0.05)
        xg, yg, yawg = pose(tg)
        pxg, pyg = self.person(tg)
        rel = wrap(np.arctan2(pyg - yg, pxg - xg) - yawg)
        vis = (np.abs(rel) <= CAM_HALF) & (np.hypot(pxg - xg, pyg - yg) <= D_CAM)
        run = 0
        for i, vv in enumerate(vis):          # can thay lien tuc 0.3 s
            run = run + 1 if vv else 0
            if run >= 6:
                self.found = float(tg[i])
                dur = tg[i] - t0
                break
        t, sid, rs = sweep_samples(self.rng, self.field, self.cfg, t0, t0 + dur, pose, self.person)
        self.logs.append((t, sid, rs))
        self.odom.append((tg, yawg))
        self.t = t0 + dur
        self.x, self.y, self.yaw = x0 + v * dur * math.cos(yaw0), y0 + v * dur * math.sin(yaw0), yaw0 + w * dur
        return self.found is not None or self.t >= T_MAX

    def rotate(self, ang, w) -> bool:
        return self._run(abs(ang) / w, math.copysign(w, ang), 0.0) if abs(ang) > 1e-3 else False

    def drive(self, dist) -> bool:
        return self._run(dist / V_DRIVE, 0.0, V_DRIVE)

    def wait(self, sec) -> bool:
        return self._run(sec, 0.0, 0.0)

    def segment(self, t0):
        t = np.concatenate([l[0] for l in self.logs]); sid = np.concatenate([l[1] for l in self.logs])
        rs = np.concatenate([l[2] for l in self.logs])
        to = np.concatenate([o[0] for o in self.odom]); yo = np.concatenate([o[1] for o in self.odom])
        m, mo = t >= t0, to >= t0
        return t[m], sid[m], rs[m], to[mo], yo[mo]


def strat_none(L: Loop, cal) -> None:
    # SEARCH hien tai sau khi toi cho thay cuoi: quet qua lai 0.35 rad/s, doi chieu moi 3.5 s, toi da 12 s
    for k in range(4):
        if L.rotate((1 if k % 2 == 0 else -1) * 0.35 * min(3.5, 12.0 - 3.5 * k), 0.35):
            return


def strat_spin(L: Loop, cal) -> None:
    # chi xoay 1 vong cho camera nhin 360 do, khong RSSI
    L.rotate(2 * math.pi, 0.6)


def strat_static(L: Loop, cal) -> None:
    while L.t < T_MAX:
        t0 = L.t
        if L.wait(1.0):
            return
        t, sid, rs, _, _ = L.segment(t0)
        e = est_static(t, sid, rs, t0, t0 + 1.0)
        if L.rotate(0.0 if e is None else e, W_TURN):
            return
        if L.drive(1.5):
            return


def strat_rotdf(L: Loop, cal, which: str, w: float = 0.6, step: float = 1.5, sweep: float = 360.0) -> None:
    tmpl, alpha = cal
    k = 0
    while L.t < T_MAX:
        t0 = L.t
        # xoay thieu vong thi doi chieu moi chu ky de lan luot nhin ca hai phia
        if L.rotate((1 if k % 2 == 0 else -1) * math.radians(sweep), w):
            return
        k += 1
        t, sid, rs, to, yo = L.segment(t0)
        if which == "mau":
            phi = est_template(t, sid, rs, to, yo, tmpl)
        elif which == "mau_dong":
            phi = est_template_moving(t, sid, rs, to, yo, tmpl, L.t)
        elif which.startswith("kh"):
            phi = est_template_moving(t, sid, rs, to, yo, tmpl, L.t, margin=int(which[2:]) / 100.0)
        else:
            r = estimate_harmonic(t, sid, rs, to, yo, alpha)
            phi = None if r is None else r["phi"]
        if phi is not None and L.rotate(float(wrap(phi - L.yaw)), W_TURN):
            return
        if L.drive(step):
            return


def run_loop(n: int) -> None:
    rng = np.random.default_rng(11)
    cfg = Cfg()
    cals = [calibrate(rng, cfg) for _ in range(5)]
    strats = [
        ("SEARCH hien tai (quet qua lai 12 s)", strat_none),
        ("chi xoay 1 vong cho camera nhin", strat_spin),
        ("RSSI tinh C-B (quay 90 do) + di 1.5 m", strat_static),
        ("mau 0.6 rad/s + di 1.5 m", lambda L, c: strat_rotdf(L, c, "mau")),
        ("mau 0.9 rad/s + di 3.0 m", lambda L, c: strat_rotdf(L, c, "mau", w=0.9, step=3.0)),
        ("mau_dong 0.9 rad/s, 270 do + di 2.0 m", lambda L, c: strat_rotdf(L, c, "mau_dong", w=0.9, step=2.0, sweep=270.0)),
        ("kh5 0.9 rad/s + di 2.0 m", lambda L, c: strat_rotdf(L, c, "kh5", w=0.9, step=2.0)),
        ("kh5 0.9 rad/s + di 3.0 m", lambda L, c: strat_rotdf(L, c, "kh5", w=0.9, step=3.0)),
        ("kh5 0.9 rad/s, 270 do + di 2.0 m", lambda L, c: strat_rotdf(L, c, "kh5", w=0.9, step=2.0, sweep=270.0)),
    ]
    if os.environ.get("GON"):
        strats = [strats[i] for i in (0, 1, 8)]
    print(f"\nVong kin: nguoi an cach 2.5-6 m huong bat ky; camera chi thay trong {D_CAM:.0f} m va +-31 do; toi da {T_MAX:.0f} s")
    print(f"{'chien luoc':42s}{'nguoi dung yen':>26s}{'nguoi di 0.3 m/s':>26s}")
    print(f"{'':42s}{'tim thay | TV thoi gian':>26s}{'tim thay | TV thoi gian':>26s}")
    for label, fn in strats:
        s = f"{label:42s}"
        for wander in (False, True):
            times = []
            for i in range(n):
                L = Loop(np.random.default_rng(1000 * i + (7 if wander else 3)), cfg, wander)
                fn(L, cals[i % 5])
                times.append(L.found if L.found is not None else np.inf)
            tt = np.array(times)
            ok = np.isfinite(tt)
            med = np.median(tt[ok]) if ok.any() else float("nan")
            s += f"{ok.mean() * 100:14.0f}% | {med:5.1f} s"
        print(s)


def run_check() -> None:
    """Mo hinh co tai tao dung so do that 30/09 khong (xe dung yen, ghi 40 s)?"""
    rng = np.random.default_rng(5)
    cases = [  # (nhan, goc do, khoang cach, co nguoi deo (lech C), so do C-B that)
        ("nguoi truoc 1.5 m", 0, 1.5, True, -3.7), ("nguoi TRAI 45", 45, 1.5, True, -5.7),
        ("nguoi PHAI 45", -45, 1.5, True, -5.1), ("nguoi truoc 3 m", 0, 3.0, True, -5.2),
        ("gia do TRAI 90", 90, 1.5, False, 11.6), ("gia do PHAI 90", -90, 1.5, False, -13.1),
        ("gia do truoc", 0, 1.5, False, 0.2)]
    global BIAS_C
    print(f"\n{'vi tri':20s}{'C-B that':>10s}{'C-B mo phong (TB +- lech giua cac lan)':>42s}{'lech chuan mau':>18s}")
    for label, deg, d, worn, real in cases:
        saved = BIAS_C
        BIAS_C = saved if worn else 0.0
        idx, sd = [], []
        for _ in range(40):
            field = Field(rng, 3.0)
            a = math.radians(deg)
            t, sid, rs = sweep_samples(rng, field, Cfg(), 0.0, 40.0,
                                       lambda tt: (np.zeros_like(tt), np.zeros_like(tt), np.zeros_like(tt)),
                                       lambda tt: (np.full_like(tt, d * math.cos(a)), np.full_like(tt, d * math.sin(a))))
            mb, mc = rs[sid == "B"].mean() - OFF["B"], rs[sid == "C"].mean() - OFF["C"]
            idx.append(mc - mb)
            sd.append(np.mean([rs[sid == s].std() for s in BOARDS]))
        BIAS_C = saved
        print(f"{label:20s}{real:+10.1f}{np.mean(idx):+26.1f} +- {np.std(idx):4.1f}{np.mean(sd):17.1f} dB")
    print("  so do that: lech chuan mau 1.5-7 dB; hai lan do cung cho lech nhau ~5 dB")


if __name__ == "__main__":
    n = int(os.environ.get("N", "200"))
    what = sys.argv[1:] or ["huong", "vongkin"]
    if "kiemtra" in what:
        run_check()
    if "huong" in what:
        run_dir(n)
    if "vongkin" in what:
        run_loop(max(50, n // 2))
