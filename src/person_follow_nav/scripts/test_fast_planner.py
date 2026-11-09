#!/usr/bin/env python3
"""(09/11, mini PC) Ban NHANH cua cac phan planner phai GIONG TUNG BIT ban goc:
  _known_free (gop moi tu the / vong quet trong vai phep numpy; Q > 2000 diem di vong cu), _known_free_grid (luoi 4 cm
  cua _esc_fields: khung bao tung tu the + vong quet gop theo tu the / theo tia), _esc_fields (them truong CO VIEN distp /
  kfp), tim thoat ket (_esc_search_gen: child_poses gop 12 dong tac, min tung dong tac bang reduceat, cung mu chi tinh o
  diem chua biet trong, nut mang san (Hv, err); _esc_clear / _esc_kfree doc truong co vien; _esc_hlook kep bang
  maximum / minimum), _nav_plan (nho phan luoi + Dijkstra theo dau vao; khoang cach chi o o lay mau; do thi CSR dung
  thang — _nav_graph).
So voi BAN SAO NGUYEN VAN code goc (ref_* ben duoi, trich tu nhanh gop-0711; ban tham chieu chay voi CAC HAM CON GOC
gan tam vao node — dung_ban_goc) tren canh bay ngau nhien (tui U, hanh lang cut, chu L, lon xon), trang thai node gan
nhu luc chay that (vet xe co doan dung yen, lich su quet mat 40 % tia).
Chay:  cd ~/ros2_ws/ros2_jazzy_person_reid_ws/src/person_follow_nav/scripts && python3 test_fast_planner.py   (=> DAT)
Chi can numpy + scipy + yaml, khong can ROS. SO_CANH=n: so canh (mac dinh 36).
Luu y: so sanh tung bit dua vao viec numpy cho CUNG ket qua cho cung gia tri bat ke vi tri phan tu trong mang (do 09/11
tren i5-5200U, numpy 1.26.4 AVX2: dung). May khac (vd. laptop AVX-512) phai chay lai test nay truoc khi dung ban nhanh.
"""
from __future__ import annotations

import heapq
import math
import os
import sys
import types
from collections import deque
from time import perf_counter as _real_perf

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
# stub ROS lay tu test_sim.py (phan truoc sys.path.insert) — nhu sim_follow.py
_stub = open(os.path.join(HERE, "test_sim.py")).read().split("sys.path.insert")[0]
exec(compile(_stub, "stubs", "exec"))
sys.path.insert(0, os.path.join(HERE, ".."))
import person_follow_nav.follow_planner_node as FP  # noqa: E402

CFG = yaml.safe_load(open(os.path.join(HERE, "..", "config", "follow_nav.yaml")))
FAKE_T = [1000.0]
time = types.SimpleNamespace(time=lambda: FAKE_T[0], monotonic=lambda: FAKE_T[0], sleep=lambda s: None)
FP.time = time                     # node dung gio gia (nhu sim_follow)
_PC = [0.0]


def _perf():                       # dong ho CPU gia: tim thoat ket khong phu thuoc may nhanh / cham (nhu sim 'dong_ho_gia')
    _PC[0] += 1e-6
    return _PC[0]


FP._perf = _perf
wrap_pi_arr = FP.wrap_pi_arr
wrap_pi = FP.wrap_pi
_sp_dijkstra, _csr_matrix, _edt = FP._sp_dijkstra, FP._csr_matrix, FP._edt
try:
    from scipy.sparse import csr_matrix as _csr
except Exception:                  # pragma: no cover
    _csr = None


# ══════════════════════════════════════════════════════════════════════════════════════════════════════════════
# BAN SAO NGUYEN VAN code goc (nhanh gop-0711 = tag ban-tot-nhat-0711 + toi uu 13/10) — chi doi ten ham
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════════

def ref_known_free(self, Q: np.ndarray, trail_sec: float = 30.0) -> np.ndarray:
    """Diem Q (khung odom) da biet la TRONG: than xe tung o do (trail_sec, thuong 30 s; thoat ket dung
    escape_trail_sec), hoac lidar da nhin xuyen qua (20 s)."""
    now = time.time()
    ok = np.zeros(Q.shape[0], bool)
    m_ = 0.02
    for (t, x, y, yaw) in self.pose_trail:
        if now - t > trail_sec:
            continue
        c, s_ = math.cos(yaw), math.sin(yaw)
        dx, dy = Q[:, 0] - x, Q[:, 1] - y
        lx, ly = c * dx + s_ * dy, -s_ * dx + c * dy
        ok |= (lx < self.front_len - m_) & (lx > -self.rear_len + m_) & (np.abs(ly) < self.half_width - m_)
        if ok.all():
            return ok
    for h in self.scan_hist:
        if now - h[0] > 20.0:
            continue
        c, s_ = math.cos(h[3]), math.sin(h[3])
        dx, dy = Q[:, 0] - h[1], Q[:, 1] - h[2]
        bx, by = c * dx + s_ * dy, -s_ * dx + c * dy
        rm, rb, okb = self._beam_lookup(bx, by, h[4:], long_free=True)
        ok |= okb & (rb > rm + 0.08)
        if ok.all():
            return ok
    return ok


def ref_esc_search_gen(self, f: Dict[str, Any], hf: Optional[Dict[str, Any]], max_exp: int,
                    face: Optional[Tuple[float, float, float, float, float]] = None, t_scale: float = 1.0):
    """A* tren (x, y, huong) voi cac dong tac co ban. GENERATOR (06/11): moi lan next() tim escape_slice_exp buoc
    roi yield None (xe dung yen, nhip dieu khien sau tim tiep); xong thi StopIteration.value = danh sach dong tac
    [(loai, v, w, T)] da gop, [] neu tu the hien tai da dat, None neu khong tim thay. Truoc 06/11 tim mot mach voi
    tran 0.25 s THOI GIAN THAT: may ban (laptop chay YOLO, mini PC cham ~3 lan, 28 mo phong song song) thi tim duoc
    it buoc hon han -> bay sau khong ra ke hoach (mo phong tui U 0.90 m sau 1.3 m: chay rieng DAT, chay chung LOI).
    Nay tran theo SO BUOC (max_exp) + tong thoi gian escape_max_sec * t_scale chia nhieu nhip.

    face = (px, py, d_lo, d_hi, tol) (06/11): DICH la QUAY MAT ve diem (px, py) (khung base luc lap) lech <= tol, cach
    diem do trong [d_lo, d_hi] — xe da toi noi ma xoay tai cho khong duoc (duoi vuong tuong). Khong dung ban do."""
    pose0 = (self.robot_x, self.robot_y, self.robot_yaw)    # khung lap ke hoach (co dinh suot cac nhip)
    prims = self._esc_prims()
    T_tpl, rear, n_per = self._esc_template()
    thr = self.margin_hard + 0.012                  # luoi 1 cm sai toi ~1 cm: chat hon mot chut
    floor_ = 0.02 + 0.012
    a_tol = math.radians(25.0)
    if face is not None:
        fx_, fy_, d_lo, d_hi, a_tol = face

        def look(X, Y, TH):
            X, Y, TH = np.asarray(X, float), np.asarray(Y, float), np.asarray(TH, float)
            dd_ = np.hypot(fx_ - X, fy_ - Y)
            return (np.maximum(0.0, d_lo - dd_) + np.maximum(0.0, dd_ - d_hi),
                    np.abs(wrap_pi_arr(np.arctan2(fy_ - Y, fx_ - X) - TH)))
    else:
        def look(X, Y, TH):
            return self._esc_hlook(hf, X, Y, TH, pose0)
    # mau thoi gian cua tung dong tac
    samp = []
    for (kind, v, w, T, cst) in prims:
        ns = 3 if kind != "F" else 4
        samp.append((np.arange(1, ns + 1) / ns) * T)

    def child_poses(x, y, th):
        Xs, Ys, THs, owner = [], [], [], []
        for j, (kind, v, w, T, cst) in enumerate(prims):
            t = samp[j]
            if abs(w) < 1e-9:
                X = x + v * t * math.cos(th)
                Y = y + v * t * math.sin(th)
                TH = np.full(t.shape, th)
            else:
                TH = th + w * t
                X = x + (v / w) * (np.sin(TH) - math.sin(th))
                Y = y - (v / w) * (np.cos(TH) - math.cos(th))
            Xs.append(X)
            Ys.append(Y)
            THs.append(TH)
            owner.append(np.full(t.shape, j))
        return np.concatenate(Xs), np.concatenate(Ys), np.concatenate(THs), np.concatenate(owner)

    def h_of(Hv, err, H0):
        return 0.06 * (err / self._ESC_ROT) + 0.5 * np.maximum(0.0, Hv - H0)

    def is_goal(x, y, th, cn, H0):
        if cn < thr:
            return False
        Hv, err = look(np.array([x]), np.array([y]), np.array([th]))
        if face is not None:
            return bool(Hv[0] <= 1e-6 and err[0] <= a_tol)
        if not np.isfinite(Hv[0]) or Hv[0] > H0 + 0.10 or err[0] > a_tol:
            return False
        dd = np.array([0.10, 0.20, 0.30])
        cl, _px, _py = self._esc_clear(f, x + dd * math.cos(th), y + dd * math.sin(th), np.full(3, th))
        return bool(cl.min() >= thr)

    c_s, _px, _py = self._esc_clear(f, np.zeros(1), np.zeros(1), np.zeros(1))
    c0 = float(c_s[0])
    H0a, e0a = look(np.zeros(1), np.zeros(1), np.zeros(1))
    H0 = float(H0a[0])
    if not np.isfinite(H0):
        return None
    if is_goal(0.0, 0.0, 0.0, c0, H0):
        return []
    # node: (x, y, th, g, parent, prim, huong tinh tien +1/-1/0, do thoang)
    nodes = [(0.0, 0.0, 0.0, 0.0, -1, -1, 0, c0)]
    heap = [(float(h_of(np.array([H0]), e0a, H0)[0]), 0)]
    closed: Dict[Tuple[int, int, int], float] = {}
    k_th = math.radians(7.5)
    goal_i = -1
    t_total = self.escape_max_sec * t_scale         # tran tong thoi gian (gioi han chinh: max_exp)
    t_used = 0.0
    t_s0 = _perf()
    _it = 0
    for _it in range(max_exp):
        if not heap:
            break
        # moi nhip dung escape_slice_exp buoc (TAT DINH: mo phong lap lai duoc, laptop / mini PC cung mot ke hoach,
        # chi khac thoi gian cho); may cham bat thuong (mot nhip > 0.2 s) thi nhuong som
        if (_it % self.escape_slice_exp) == self.escape_slice_exp - 1 or ((_it & 15) == 15 and _perf() - t_s0 > 0.2):
            t_used += _perf() - t_s0
            if t_used > t_total:
                break                               # het tong thoi gian — lan sau tim lai tu tu the moi
            self.esc_exp = _it + 1
            yield None                              # nhuong nhip dieu khien (xe dung yen), nhip sau tim tiep
            t_s0 = _perf()
        _fs, ni = heapq.heappop(heap)
        x, y, th, gc, _par, _pi, dr, cn = nodes[ni]
        key = (int(round(x / 0.025)), int(round(y / 0.025)), int(round(th / k_th)) % 48)
        if key in closed and closed[key] <= gc + 1e-9:
            continue
        closed[key] = gc
        if ni != 0 and is_goal(x, y, th, cn, H0):
            goal_i = ni
            break
        if len(nodes) > 40 * max_exp:
            break
        X, Y, TH, own = child_poses(x, y, th)
        cl, PX, PY = self._esc_clear(f, X, Y, TH)
        # vien footprint roi vao cung MU (nhin tu tu the luc lap ke hoach) ngoai than xe luc dau: phai da biet trong
        per = slice(0, n_per)
        bl = self._esc_blind(PX[:, per], PY[:, per])
        okb = ~bl | self._esc_kfree(f, PX[:, per], PY[:, per])
        blind_ok = okb.all(axis=1)
        keep, lasts = [], []
        for j, (kind, v, w, T, cst) in enumerate(prims):
            sel = own == j
            mn = float(cl[sel].min())
            if not (mn >= thr or (cn < thr and mn >= max(floor_, cn - 0.005))):
                continue
            if not bool(blind_ok[sel].all()):
                continue
            if kind == "B":
                rx_, ry_ = PX[sel][:, rear], PY[sel][:, rear]
                if not bool(self._esc_kfree(f, rx_, ry_).all()):
                    continue
            keep.append(j)
            lasts.append(int(np.where(sel)[0][-1]))
        if not keep:
            continue
        li = np.array(lasts)
        Hv, err = look(X[li], Y[li], TH[li])
        hh = h_of(Hv, err, H0)
        for q, j in enumerate(keep):
            if not np.isfinite(Hv[q]):
                continue
            kind, v, w, T, cst = prims[j]
            last = lasts[q]
            nd = 1 if kind == "F" else (-1 if kind == "B" else dr)
            g2 = gc + cst + (0.10 if (kind != "R" and dr != 0 and nd != dr) else 0.0)
            g2 += 0.5 * max(0.0, float(Hv[q]) - H0)          # di vao sau hon trong ngo cut thi dat
            nodes.append((float(X[last]), float(Y[last]), float(TH[last]), g2, ni, j, nd, float(cl[last])))
            heapq.heappush(heap, (g2 + 1.5 * float(hh[q]), len(nodes) - 1))
    self.esc_exp = _it + 1 if max_exp > 0 else 0          # so buoc tim (de log / chinh gioi han CPU)
    if goal_i < 0:
        return None
    seq = []
    i = goal_i
    while i > 0:
        seq.append(nodes[i][5])
        i = nodes[i][4]
    seq.reverse()
    out = []
    for j in seq:
        kind, v, w, T, _c = prims[j]
        if out and out[-1][0] == kind and abs(out[-1][1] - v) < 1e-9 and abs(out[-1][2] - w) < 1e-9:
            out[-1] = (kind, v, w, out[-1][3] + T)
        else:
            out.append((kind, v, w, T))
    return out


def ref_esc_fields(self, half: float = 1.5, trail_sec: float = 30.0) -> Optional[Dict[str, Any]]:
    """Truong khoang cach toi vat (luoi 1 cm) + luoi 'da biet la trong' (4 cm) quanh xe, khung base hien tai."""
    if _edt is None:
        return None
    res, pad = 0.01, 0.40
    L = half + pad
    parts = [a for a in (self.obstacles_full, self.mem_occl, self.obstacles) if a is not None and a.shape[0]]
    P = np.vstack(parts) if parts else np.zeros((0, 2))
    n = int(math.ceil(2.0 * L / res))
    occ = np.zeros((n, n), dtype=bool)
    if P.shape[0]:
        ci = np.floor((P[:, 0] + L) / res).astype(np.int64)
        ri = np.floor((P[:, 1] + L) / res).astype(np.int64)
        ok = (ci >= 0) & (ci < n) & (ri >= 0) & (ri < n)
        occ[ri[ok], ci[ok]] = True
    dist = (_edt(~occ) * res).astype(np.float32) if occ.any() else np.full((n, n), 9.0, dtype=np.float32)
    kres = 0.04
    nk = int(math.ceil(2.0 * half / kres))
    kc = -half + (np.arange(nk) + 0.5) * kres
    KX, KY = np.meshgrid(kc, kc)
    if self.rev_known_free:
        c, s_ = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        Q = np.stack([self.robot_x + c * KX.ravel() - s_ * KY.ravel(),
                      self.robot_y + s_ * KX.ravel() + c * KY.ravel()], axis=1)
        kf = self._known_free(Q, trail_sec).reshape(nk, nk)
        # an toan hon: o "da biet trong" chi khi ca 4 o ke cung trong (tam o 4 cm, diem mau co the lech 2.8 cm)
        ke = kf.copy()
        ke[1:, :] &= kf[:-1, :]
        ke[:-1, :] &= kf[1:, :]
        ke[:, 1:] &= kf[:, :-1]
        ke[:, :-1] &= kf[:, 1:]
        kf = ke
    else:
        kf = np.ones((nk, nk), dtype=bool)
    return {"L": L, "res": res, "n": n, "dist": dist, "half": half, "kres": kres, "nk": nk, "kf": kf}


def ref_esc_clear(self, f: Dict[str, Any], X: np.ndarray, Y: np.ndarray, TH: np.ndarray):
    """Do thoang (xap xi luoi 1 cm) cua footprint o cac tu the (X, Y, TH) — khung base luc lap ke hoach."""
    T, _rear, _np = self._esc_template()
    c, s_ = np.cos(TH)[:, None], np.sin(TH)[:, None]
    PX = X[:, None] + c * T[None, :, 0] - s_ * T[None, :, 1]
    PY = Y[:, None] + s_ * T[None, :, 0] + c * T[None, :, 1]
    n, L, res = f["n"], f["L"], f["res"]
    ci = np.floor((PX + L) / res).astype(np.int64)
    ri = np.floor((PY + L) / res).astype(np.int64)
    out = (ci < 0) | (ci >= n) | (ri < 0) | (ri >= n)
    d = f["dist"][np.clip(ri, 0, n - 1), np.clip(ci, 0, n - 1)]
    d = np.where(out, 0.0, d)                       # ra ngoai cua so = khong cho di (khong biet gi o do)
    return d.min(axis=1), PX, PY


def ref_esc_kfree(self, f: Dict[str, Any], PX: np.ndarray, PY: np.ndarray) -> np.ndarray:
    kres, nk, half = f["kres"], f["nk"], f["half"]
    ci = np.floor((PX + half) / kres).astype(np.int64)
    ri = np.floor((PY + half) / kres).astype(np.int64)
    out = (ci < 0) | (ci >= nk) | (ri < 0) | (ri >= nk)
    v = f["kf"][np.clip(ri, 0, nk - 1), np.clip(ci, 0, nk - 1)] & ~out
    m_ = 0.01
    ins = (PX < self.front_len - m_) & (PX > -self.rear_len + m_) & (np.abs(PY) < self.half_width - m_)
    return v | ins


def ref_esc_blind(self, PX: np.ndarray, PY: np.ndarray) -> np.ndarray:
    """Diem nam trong cung MU cua lidar nhin tu tu the luc lap ke hoach (sau duoi xe)."""
    m = np.zeros(PX.shape, dtype=bool)
    if not self.blind_sectors:
        return m
    a = (np.arctan2(PY - self.lidar_y, PX - self.lidar_x) - self.lidar_yaw_offset_rad) * self.lidar_angle_sign
    deg = np.degrees(a) % 360.0
    for lo, hi in self.blind_sectors:
        m |= (deg >= lo) & (deg <= hi)
    return m


def ref_esc_hlook(self, hf: Dict[str, Any], X: np.ndarray, Y: np.ndarray, TH: np.ndarray,
                  pose: Optional[Tuple[float, float, float]] = None):
    """(con bao xa toi dich, lech mui so voi huong xuong doc) cho tu the (X, Y, TH) khung base cua `pose` (tu the
    luc LAP ke hoach — tim chia nhieu nhip nen khong dung tu the hien tai; mac dinh = tu the hien tai)."""
    g = hf["g"]
    px, py, pyaw = g["pose"]
    rx, ry, ryaw = pose if pose is not None else (self.robot_x, self.robot_y, self.robot_yaw)
    c, s_ = math.cos(ryaw), math.sin(ryaw)
    OX = rx + c * X - s_ * Y
    OY = ry + s_ * X + c * Y
    dx, dy = OX - px, OY - py
    cp, sp = math.cos(pyaw), math.sin(pyaw)
    BX, BY = cp * dx + sp * dy, -sp * dx + cp * dy
    ci = np.clip(np.round((BX - g["x0"]) / g["res"] - 0.5).astype(np.int64), 0, g["nxc"] - 1)
    ri = np.clip(np.round((BY + g["ey"]) / g["res"] - 0.5).astype(np.int64), 0, g["nyc"] - 1)
    err = np.abs(wrap_pi_arr(TH + (ryaw - pyaw) - hf["DIR"][ri, ci]))
    return hf["H"][ri, ci], err


# cac ham GOC gan tam vao node khi chay ban tham chieu (ref_esc_search_gen / ref_esc_fields goi self._esc_clear ...)
REF_METHODS = {"_known_free": ref_known_free, "_esc_clear": ref_esc_clear, "_esc_kfree": ref_esc_kfree,
               "_esc_blind": ref_esc_blind, "_esc_hlook": ref_esc_hlook}


class dung_ban_goc:
    """with dung_ban_goc(pl): ... — trong khoi nay node dung cac ham GOC (REF_METHODS) thay cho ban nhanh."""

    def __init__(self, pl):
        self.pl = pl

    def __enter__(self):
        for k, fn in REF_METHODS.items():
            setattr(self.pl, k, types.MethodType(fn, self.pl))
        return self.pl

    def __exit__(self, *a):
        for k in REF_METHODS:
            delattr(self.pl, k)


def fields_same(fa, fb):
    """_esc_fields ban nhanh (fa) vs ban goc (fb): moi khoa cu giong tung bit + 'distp' / 'kfp' = ban goc + vien 0."""
    if fa is None or fb is None:
        return fa is None and fb is None
    if set(fa) != set(fb) | {"distp", "kfp"}:
        return False
    dp, kp = fa["distp"], fa["kfp"]
    ok = (dp.dtype == np.float32 and same(dp[1:-1, 1:-1], fb["dist"]) and kp.dtype == bool
          and same(kp[1:-1, 1:-1], fb["kf"]))
    for a_ in (dp, kp):
        ok = ok and not (a_[0].any() or a_[-1].any() or a_[:, 0].any() or a_[:, -1].any())
    return ok and all(same(fa[k], fb[k]) for k in fb)


def ref_gap_refine(self, g: Dict[str, float],
                target_xy: Optional[Tuple[float, float]] = None) -> Optional[Dict[str, float]]:
    """Dat lai khe vao MAT CAT HEP NHAT cua loi di quanh khe vua chon (14/10).

    _gap_target lay hai diem lidar KE NHAU THEO GOC. O cua that, cho do thuong KHONG phai cho
    hep nhat: tuong day, khung cua nho ra, rem buoc gon, canh cua mo ap mot ben bien loi di thanh
    mot "duong ham" ngan, va hai thanh cua o mieng cua nam o goc nhin rat cheo nen khong ke nhau
    theo goc. Xe that 14/10 (bag follow_1014_153520, cua bep 0.80 m): khe chon duoc la cho mo
    0.95 m PHIA TRONG bep (tuong trai trong bep <-> canh cua), lech 7-8 do so voi cua that ->
    xe "da vao truc" (lech 0-1 cm) theo truc sai, that ra lech trai 4 cm + nghieng 8 do -> goc
    truoc trai cach thanh cua 4-5 cm -> xoay qua xoay lai mai; lan sau khe nhay qua lai giua cua
    that va mot cap diem cheo -> xe tien lui mai.

    Cach tinh: thu cac huong chui quanh huong cu (+-16 do, buoc 2 do). Voi moi huong, chieu cac
    diem lidar tu 0.7 m truoc khe toi 0.3 m sau khe (ke ca canh than xe) len truc ngang, lay
    khoang TRONG LIEN TUC chua tam khe cu = be rong duong THANG lot qua that. Chon huong rong
    nhat, phat nhe khi lech huong cu / truc nhip truoc (1-2 mm moi do) de khong nhay vi nhieu. Tam
    khe moi = giua khoang trong do, dat o mat cat hep dau tien xe gap. Duoi 2*(half_width +
    gap_cross_margin) + 2 cm (xe khong the lot) hoac khong tinh duoc thi tra None.
    """
    P = self.obstacles_full if self.obstacles_full.shape[0] else self.obstacles
    if P.shape[0] < 2:
        return None
    if target_xy is not None:
        # Bo chan nguoi dang bam (nguoi vua buoc qua cua thi chan ho khong duoc lam lech khe). Vi tri du
        # doan troi toi sat thanh cua thi xoa luon thanh cua -> khe "no" ra: bat o dieu kien be rong ben
        # duoi (mat cat hep nhat khong the rong hon khe cu).
        d_t = np.hypot(P[:, 0] - target_xy[0], P[:, 1] - target_xy[1])
        P = P[d_t > min(0.35, self.target_clear_r)]
    mx, my, psi0 = g["mx"], g["my"], g["psi"]
    lat_w = 0.5 * g["gw"] + 0.35
    cands = [psi0 + math.radians(float(d)) for d in range(-16, 17, 2)]
    # Truc nhip truoc (khung odom) — chi khi cung mot khe (tam gan, huong gan)
    prev = None
    now_ = time.time()
    gox, goy = self._odom_of(mx, my)          # tam khe CU (khung odom) — de nhan ra "cung mot khe"
    if self.gap_axis_prev is not None and now_ - self.gap_axis_prev[3] < 1.0:
        pa, pox, poy, _t = self.gap_axis_prev
        pr = wrap_pi(pa - self.robot_yaw)
        if math.hypot(gox - pox, goy - poy) < 0.5 and abs(wrap_pi(pr - psi0)) < math.radians(20.0):
            prev = pr
            cands.append(pr)
    best = None
    for th in cands:
        c, s_ = math.cos(th), math.sin(th)
        S = P[:, 0] * c + P[:, 1] * s_
        L = -P[:, 0] * s_ + P[:, 1] * c
        sm = mx * c + my * s_
        lm = -mx * s_ + my * c
        win = ((S >= max(sm - 0.70, -self.rear_len - 0.05)) & (S <= sm + 0.30)
               & (np.abs(L - lm) <= lat_w))
        if not np.any(win):
            continue
        Lw, Sw = L[win], S[win]
        lft = Lw > lm
        if not lft.any() or lft.all():
            continue                           # mot ben khong co gi chan: khong phai loi di
        l_l = float(np.min(Lw[lft]))
        l_r = float(np.max(Lw[~lft]))
        w = l_l - l_r
        # Phat lech: 1 mm moi do so voi huong khe cu, 2 mm moi do so voi truc nhip truoc (chong nhay vi
        # nhieu). Phai NHO hon do doc be rong theo goc cua cua that: tuong day 12 cm chi hep them ~2 mm
        # moi do nghieng — phat 4 mm/do thi khong bao gio xoay duoc truc ve phap tuyen that.
        dev = 0.057 * abs(wrap_pi(th - psi0)) + (0.115 * abs(wrap_pi(th - prev)) if prev is not None else 0.0)
        score = w - dev
        if best is None or score > best[0]:
            # Mat cat hep DAU TIEN xe gap: s nho nhat cua MOI diem sat mep khoang trong (trong 3 cm ra
            # ngoai), khong chi diem chan mep. Tuong day: goc ngoai va goc trong thanh cua cung toa do
            # ngang, lay diem nao la ngau nhien -> 'along' nhay 12 cm moi nhip, cham nguong lui (mo phong
            # cua ben tuong day 14/10: xe lui - tien mai cach mat cua 0.5 m).
            e_l = (Lw >= l_l - 1e-3) & (Lw <= l_l + 0.03)
            e_r = (Lw <= l_r + 1e-3) & (Lw >= l_r - 0.03)
            # Do SAU loi di: thanh chan mep chay dai bao nhieu doc truc (tuong song song truc = hanh lang).
            # Dai 10 cm (khong phai 3 cm): truc lech tuong vai do thi tuong dai 1 m chi con ~0.3 m trong dai 3 cm.
            d_l = (Lw >= l_l - 1e-3) & (Lw <= l_l + 0.10)
            d_r = (Lw <= l_r + 1e-3) & (Lw >= l_r - 0.10)
            depth = max(float(np.ptp(Sw[d_l])), float(np.ptp(Sw[d_r])))
            best = (score, th, w, 0.5 * (l_l + l_r), float(np.min(Sw[e_l | e_r])), depth)
    # Mat cat hep nhat KHONG THE rong hon khe cu (hai mep cu nam trong cua so): rong hon la da mat diem
    # (bi che / bi loai nham) -> khong tin.
    if (best is None or best[2] < 2.0 * (self.half_width + self.gap_cross_margin) + 0.02
            or best[2] > min(g["gw"] + 0.03, self.gap_wp_max_w)):
        return None
    _sc, th, w, cl, s_star, depth = best
    dx, dy = math.cos(th), math.sin(th)
    mx2, my2 = s_star * dx - cl * dy, s_star * dy + cl * dx
    self.gap_axis_prev = (wrap_pi(th + self.robot_yaw), gox, goy, now_)
    return {"mx": mx2, "my": my2, "nx": -dx, "ny": -dy, "gw": w,
            "psi": wrap_pi(th), "along": s_star, "lat": abs(cl),
            # HANH LANG rong (thanh chan chay dai doc truc, vd. giua thung va tuong): khong phai khung cua,
            # DWA / ban do tu di duoc. Dong tac chui khe o day thi 'along' tinh tu dau cua so (tuong chay
            # suot) -> luat lui ve truc kich hoat -> lui - dung mai (mo phong 14/10: vong qua thung bi chan
            # ben trai, sang phai thi ket o mieng hanh lang 1.1 m). Cua hep van chui (rong < 0.95 m).
            "corridor": bool(w > 0.95 and depth > 0.45)}


def ref_nav_plan(self, tx: float, ty: float, moving: bool,
              standoff: Optional[float] = None, prefer_side: int = 0,
              back: float = 1.0) -> Optional[Dict[str, Any]]:
    """Tim duong tren luoi quanh xe (khung base_link) toi CHO DUNG TOT NHAT gan nguoi.

    Vi sao: tang chon khe chi thu duong THANG 1.6 / 1.1 / 0.7 / 0.45 m. O xa khong huong nao lot
    thi no lui ve tam do ngan, luc do khe hep giua hai thung (hep hon xe) cung trong "thoang" ->
    xe chon nham ben roi ket (nguoi dung bao 07/10: nguoi dung sau thung 50x30 cm, thung thu hai
    cach 50 cm mot ben, ben kia trong — xe di vao ben co thung). Diem dich (cach nguoi 1 m) lai
    nam TRONG thung -> DWA dung im mai.

    Cach lam: o luoi bi chan neu tam xe dat o do se cach vat < nav_lethal_m; gan vat hon
    half_width + margin_hard (dung muc DWA doi) thi rat dat, them vung phat nav_soft_m. Dijkstra
    tu o cua xe. Chon o dich theo J = quang duong + nav_far_weight * (khoang cach toi nguoi vuot
    follow_distance): dung truoc thung cach nguoi 1.4 m (J ~ 1.6) re hon vong 2 m de toi 1.0 m;
    nguoi o xa hon nua thi vong. O tot nhat ngay duoi xe -> 'hold' (dung cho, nhin nguoi).
    """
    res = self.nav_res if _sp_dijkstra is not None else max(self.nav_res, 0.08)
    dist = math.hypot(tx, ty)
    # back: luoi nhin bao xa SAU xe (thuong 1.0 m; bay sau 06/11: escape_far_back_m — xem _nav_get)
    x0, x1 = -float(back), float(np.clip(dist + 1.2, 2.0, 4.5))
    ey = float(np.clip(dist + 1.0, 1.8, 3.5))
    nxc = int(math.ceil((x1 - x0) / res))
    nyc = int(math.ceil(2.0 * ey / res))
    xs = x0 + (np.arange(nxc) + 0.5) * res
    ys = -ey + (np.arange(nyc) + 0.5) * res
    GX, GY = np.meshgrid(xs.astype(np.float32), ys.astype(np.float32))      # (ny, nx)

    # 03/11: diem DAY DU (vong quet + noi tia ke nhau + bo nho vat can) — chi vong quet hien tai thi tuong nhin
    # cheo con lo hong va vung mu sau duoi xe trong tron.
    P = self.obstacles_dense if self.obstacles_dense.shape[0] else self.obstacles
    if P.shape[0] > 0:
        keep = np.hypot(P[:, 0] - tx, P[:, 1] - ty) > self.nav_person_clear
        Q = P[keep].astype(np.float32)
    else:
        Q = np.zeros((0, 2), dtype=np.float32)
    if _edt is not None:
        # Bien doi khoang cach tren LUOI MIN 3 lan (o 2 cm): nhanh, khong phu thuoc so diem (diem day du co the
        # vai tram), sai so ~1 cm — cua 0.81 m chi con dai 0.21 m cho tam xe di qua, luoi 6 cm sai 3 cm la khep
        # mat cua. Mo rong 0.6 m quanh luoi de diem sat bien van tinh.
        f3 = 3
        rf = res / f3
        pad = int(math.ceil(0.6 / rf))
        nyf, nxf = nyc * f3 + 2 * pad, nxc * f3 + 2 * pad
        occ = np.zeros((nyf, nxf), dtype=bool)
        if Q.shape[0]:
            ci_ = np.floor((Q[:, 0] - x0) / rf).astype(np.int64) + pad
            ri_ = np.floor((Q[:, 1] + ey) / rf).astype(np.int64) + pad
            ok_ = (ci_ >= 0) & (ci_ < nxf) & (ri_ >= 0) & (ri_ < nyf)
            occ[ri_[ok_], ci_[ok_]] = True
        if occ.any():
            # tam o luoi 6 cm = tam o min giua khoi 3x3
            dobs = (_edt(~occ) * rf)[pad + 1:pad + nyc * f3:f3, pad + 1:pad + nxc * f3:f3].astype(np.float32)
        else:
            dobs = np.full(GX.shape, np.inf, dtype=np.float32)
    else:
        d2 = np.full(GX.shape, np.inf, dtype=np.float32)
        for i in range(0, Q.shape[0], 48):
            q = Q[i:i + 48]
            dd = (GX[..., None] - q[:, 0]) ** 2 + (GY[..., None] - q[:, 1]) ** 2
            d2 = np.minimum(d2, dd.min(axis=2))
        dobs = np.sqrt(d2)
    r_in = self.half_width + self.margin_hard
    lethal = dobs < self.nav_lethal
    cost = 1.0 + self.nav_soft_cost * np.clip((r_in + self.nav_soft - dobs) / self.nav_soft, 0.0, 1.5) ** 2

    ri = int(np.clip(round((0.0 + ey) / res - 0.5), 0, nyc - 1))
    ci = int(np.clip(round((0.0 - x0) / res - 0.5), 0, nxc - 1))
    d0 = float(dobs[ri, ci])
    if d0 < self.nav_lethal:
        # Xe DA o sat vat hon nguong: cho di qua cac o quanh xe khong gan vat hon cho dang dung, de
        # duong di thoat ra duoc (khong thi moi o quanh xe deu cam -> khong co duong).
        relax = (np.hypot(GX, GY) < 0.35) & (dobs >= d0 - 0.01)
        if Q.shape[0]:
            # 05/11: chi cac o KHONG tien ve phia vat gan nhat (ra xa / di ngang). Truoc day xe dung o mieng khe
            # 0.4 m (chan ghe ngay truoc mui) thi cac o TRONG khe cung "khong gan vat hon" -> duong ban do di xuyen
            # khe chi nguoi lot, xe tien-lui mai o do (mo phong tui hep 05/11).
            qn = Q[int(np.argmin(np.hypot(Q[:, 0], Q[:, 1])))]
            qd = math.hypot(float(qn[0]), float(qn[1]))
            if qd > 1e-3:
                relax &= (GX * float(qn[0]) + GY * float(qn[1])) / qd <= 0.03
        lethal &= ~relax
    lethal[ri, ci] = False                    # xe dang dung o day — luon cho xuat phat
    if prefer_side != 0 and dist > 0.3:
        # Giu ben vong da chon: o ben KIA duong thang xe -> nguoi dat gap 3
        lat = (tx * GY - ty * GX) / dist
        cost = cost * np.where(prefer_side * lat < -0.15, 3.0, 1.0)
    N = nxc * nyc
    start = ri * nxc + ci
    flat_cost = cost.ravel()
    free = ~lethal.ravel()

    if _sp_dijkstra is not None:
        idx = np.arange(N).reshape(nyc, nxc)
        us, vs, ws = [], [], []
        for dy, dx, L in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0))):
            ya, yb = max(0, -dy), nyc - max(0, dy)
            xa, xb = max(0, -dx), nxc - max(0, dx)
            u = idx[ya:yb, xa:xb].ravel()
            v = idx[ya + dy:yb + dy, xa + dx:xb + dx].ravel()
            ok = free[u] & free[v]
            u, v = u[ok], v[ok]
            us.append(u)
            vs.append(v)
            ws.append(L * res * 0.5 * (flat_cost[u] + flat_cost[v]))
        G = _csr_matrix((np.concatenate(ws), (np.concatenate(us), np.concatenate(vs))), shape=(N, N))
        D, pred = _sp_dijkstra(G, directed=False, indices=start, return_predecessors=True)
    else:
        D = np.full(N, np.inf)
        pred = np.full(N, -9999, dtype=np.int64)
        D[start] = 0.0
        hp = [(0.0, start)]
        nb = ((0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
              (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142))
        while hp:
            du, u = heapq.heappop(hp)
            if du > D[u]:
                continue
            uy, ux = divmod(u, nxc)
            for dy, dx, L in nb:
                vy, vx = uy + dy, ux + dx
                if 0 <= vy < nyc and 0 <= vx < nxc:
                    v = vy * nxc + vx
                    if free[v]:
                        nd = du + L * res * 0.5 * (flat_cost[u] + flat_cost[v])
                        if nd < D[v]:
                            D[v] = nd
                            pred[v] = u
                            heapq.heappush(hp, (nd, v))

    dp = np.hypot(GX - tx, GY - ty).ravel()
    kfar = self.nav_far_w_moving if moving else self.nav_far_w
    fd = self.follow_distance if standoff is None else float(standoff)
    cand = np.isfinite(D) & free & (dp >= fd - 0.3)
    if not np.any(cand):
        return None
    J = np.where(cand, D + kfar * np.maximum(0.0, dp - fd)
                 + 4.0 * np.maximum(0.0, fd - 0.1 - dp), np.inf)
    k = int(np.argmin(J))
    path = []
    cur = k
    for _ in range(N):
        cy_, cx_ = divmod(cur, nxc)
        path.append((float(xs[cx_]), float(ys[cy_])))
        if cur == start:
            break
        cur = int(pred[cur])
        if cur < 0:
            return None
    path.reverse()
    path[0] = (0.0, 0.0)
    tgt_xy = path[-1]
    hold = math.hypot(*tgt_xy) < self.nav_hold_r
    out = {"path": path, "target": tgt_xy, "hold": hold, "cost": float(D[k]), "J": float(J[k]),
           "person_d": float(dp[k])}
    if _sp_dijkstra is not None:
        # 05/11: giu luoi + do thi de bo THOAT KET tinh "con bao xa toi dich" tu MOI o (Dijkstra nguoc tu dich)
        out["grid"] = {"G": G, "k": k, "nxc": nxc, "nyc": nyc, "res": res, "x0": x0, "ey": ey,
                       "pose": (self.robot_x, self.robot_y, self.robot_yaw)}
    return out




# ══════════════════════════════════════════════════════════════════════════════════════════════════════════════
# So sanh tren canh BAY ngau nhien (tui U / hanh lang / vat lon xon), trang thai node gan nhu luc chay that
# ══════════════════════════════════════════════════════════════════════════════════════════════════════════════
def _segs_points(segs, rng, step=0.03, noise=0.008):
    out = []
    for (x1, y1, x2, y2) in segs:
        L = math.hypot(x2 - x1, y2 - y1)
        k = max(2, int(L / step))
        u = np.linspace(0.0, 1.0, k)
        out.append(np.stack([x1 + u * (x2 - x1), y1 + u * (y2 - y1)], 1))
    P = np.vstack(out)
    return P + rng.normal(0.0, noise, P.shape)


def scene(rng):
    """Canh khung base (xe o goc, mui +x): tui U / hanh lang cut / chu L + vai thung. Tra ve diem vat can, nguoi."""
    W = rng.uniform(0.72, 1.3)
    d = rng.uniform(0.4, 2.4)
    kind = int(rng.integers(0, 4))
    segs = []
    if kind in (0, 1):            # tui U / hanh lang cut: xe da di vao tu phia SAU, phia truoc la vach
        segs += [(-d, W / 2, 0.45, W / 2), (-d, -W / 2, 0.45, -W / 2), (0.45, -W / 2, 0.45, W / 2)]
        if kind == 1:
            segs += [(-d - 1.5, W / 2 + 0.5, -d, W / 2), (-d - 1.5, -W / 2 - 0.5, -d, -W / 2)]
    elif kind == 2:               # chu L sat vach
        segs += [(-1.5, -0.38, 1.2, -0.38), (0.30 + rng.uniform(0, 0.3), -0.38, 0.30 + rng.uniform(0, 0.3), 1.5)]
    else:                         # lon xon
        for _ in range(int(rng.integers(3, 7))):
            cx, cy = rng.uniform(-1.5, 2.5), rng.uniform(-1.8, 1.8)
            if math.hypot(cx, cy) < 0.7:
                continue
            a, b = rng.uniform(0.15, 0.5), rng.uniform(0.15, 0.5)
            segs += [(cx - a, cy - b, cx + a, cy - b), (cx + a, cy - b, cx + a, cy + b), (cx + a, cy + b, cx - a, cy + b),
                     (cx - a, cy + b, cx - a, cy - b)]
    for _ in range(int(rng.integers(0, 3))):
        cx, cy = rng.uniform(-2.0, 3.0), rng.uniform(-2.0, 2.0)
        if math.hypot(cx, cy) > 0.8:
            segs += [(cx - 0.2, cy, cx + 0.2, cy), (cx, cy - 0.2, cx, cy + 0.2)]
    P = _segs_points(segs, rng)
    P = P[np.hypot(P[:, 0], P[:, 1]) < 4.5]
    a = rng.uniform(-math.pi, math.pi)
    r = rng.uniform(1.2, 3.2)
    return P, (r * math.cos(a), r * math.sin(a))


def gap_scene(rng):
    """Canh CUA (khung base, xe o goc): tuong vuong goc truc cua, cua rong 0.70-1.10 m, tuong day 2-15 cm, them vai
    vat; khe uoc luong (mx, my, psi, gw) lech chut so voi that; nguoi o ben kia cua (doi khi sat thanh cua)."""
    W = rng.uniform(0.70, 1.10)
    d = rng.uniform(0.6, 2.0)
    y0 = rng.uniform(-0.5, 0.5)
    psi = rng.uniform(-0.5, 0.5)                 # huong truc cua (xuyen qua cua, tu phia xe)
    tw = rng.uniform(0.02, 0.15)
    ux, uy = math.cos(psi), math.sin(psi)        # doc truc
    vx, vy = -uy, ux                             # doc mat cua
    cx, cy = d * math.cos(math.atan2(y0, d)), y0
    segs = []
    for sg in (1.0, -1.0):
        ax, ay = cx + sg * 0.5 * W * vx, cy + sg * 0.5 * W * vy
        bx, by = ax + sg * 1.5 * vx, ay + sg * 1.5 * vy
        for off in (0.0, tw):                    # hai mat tuong (day tw)
            segs.append((ax + off * ux, ay + off * uy, bx + off * ux, by + off * uy))
        segs.append((ax, ay, ax + tw * ux, ay + tw * uy))   # mep thanh cua
    for _ in range(int(rng.integers(0, 3))):     # do dac truoc / sau cua
        ox, oy = cx + rng.uniform(-0.8, 1.2) * ux + rng.uniform(-1.0, 1.0) * vx, cy + rng.uniform(-0.8, 1.2) * uy
        a = rng.uniform(0.1, 0.3)
        segs += [(ox - a, oy - a, ox + a, oy - a), (ox + a, oy - a, ox + a, oy + a)]
    P = _segs_points(segs, rng, step=0.02, noise=float(rng.choice([0.0, 0.005, 0.01])))
    g = {"mx": cx + rng.uniform(-0.05, 0.05), "my": cy + rng.uniform(-0.05, 0.05),
         "psi": psi + math.radians(rng.uniform(-8.0, 8.0)), "gw": W + rng.uniform(-0.03, 0.06)}
    if rng.random() < 0.3:                       # nguoi sat thanh cua
        tg = (cx + 0.1 * ux + 0.5 * W * vx, cy + 0.1 * uy + 0.5 * W * vy)
    else:
        k = rng.uniform(0.4, 1.5)
        tg = (cx + k * ux, cy + k * uy)
    return P, g, tg


def set_state(pl, rng, P):
    """Gan trang thai node: tu the odom, tap diem (nhu _scan_cb), vet xe (co doan dung yen trung tu the), lich su quet."""
    rx, ry, ryaw = rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi)
    pl.robot_x, pl.robot_y, pl.robot_yaw = rx, ry, ryaw
    d = np.hypot(P[:, 0], P[:, 1])
    o = np.argsort(d)
    pl.obstacles = P[o[:160]]
    pl.obstacles_full = P[d < 3.0]
    pl.obstacles_dense = P if rng.random() > 0.1 else np.zeros((0, 2))
    pl.obstacles_gap = pl.obstacles_dense
    pl.mem_occl = P[o[-20:]] if rng.random() > 0.5 else np.zeros((0, 2))
    now = FAKE_T[0]
    c, s_ = math.cos(ryaw), math.sin(ryaw)
    trail = []
    n_back = int(rng.integers(0, 120))
    for i in range(n_back, 0, -1):                       # xe di lui ve qua khu: moi 4 cm
        bx = -0.04 * i
        by = rng.normal(0, 0.01)
        trail.append((now - 0.08 * i - rng.uniform(0, 40) * (i > 80), rx + c * bx - s_ * by, ry + s_ * bx + c * by,
                      ryaw + rng.normal(0, 0.02)))
    for k in range(int(rng.integers(0, 200))):          # dung yen: trung tu the
        trail.append((now - 0.25 * k, rx, ry, ryaw))
    pl.pose_trail = deque(trail, maxlen=600)
    scans = []
    lx0, ly0 = pl.lidar_x, pl.lidar_y
    for k in range(int(rng.integers(0, 48))):
        sx, sy, syaw = rx - 0.04 * k * c, ry - 0.04 * k * s_, ryaw + rng.normal(0, 0.03)
        # diem P (khung base hien tai) -> odom -> khung base cua vong quet -> goc lidar
        ox, oy = rx + c * P[:, 0] - s_ * P[:, 1], ry + s_ * P[:, 0] + c * P[:, 1]
        cc, ss = math.cos(syaw), math.sin(syaw)
        bx, by = cc * (ox - sx) + ss * (oy - sy), -ss * (ox - sx) + cc * (oy - sy)
        lx, ly = bx - lx0, by - ly0
        a_las = (np.arctan2(ly, lx) - pl.lidar_yaw_offset_rad) * pl.lidar_angle_sign
        idx = np.mod(np.round(a_las / math.radians(1.0)).astype(np.int64), 360)
        ranges = np.full(360, np.inf)
        np.minimum.at(ranges, idx, np.hypot(lx, ly))
        drop = rng.random(360) < 0.4                       # SC-Mini mat ~40 % tia
        ranges[drop] = np.where(rng.random(int(drop.sum())) < 0.5, np.inf, np.nan)
        scans.append((now - 0.5 * k - (rng.uniform(0, 30) if k > 30 else 0.0), sx, sy, syaw, ranges, 0.0,
                      math.radians(1.0), 0.02, 8.0))
    pl.scan_hist = deque(scans, maxlen=48)


def same(a, b, path="x"):
    """So sanh SAU, dung tung bit (float qua ==, mang qua dtype + array_equal co NaN, csr qua data/indices/indptr)."""
    if _csr is not None and isinstance(a, _csr):
        return (isinstance(b, _csr) and a.shape == b.shape and same(a.data, b.data) and same(a.indices, b.indices)
                and same(a.indptr, b.indptr))
    if isinstance(a, np.ndarray):
        return isinstance(b, np.ndarray) and a.dtype == b.dtype and a.shape == b.shape and (
            np.array_equal(a.view(np.uint8), b.view(np.uint8)) if a.size else True)
    if isinstance(a, dict):
        return isinstance(b, dict) and a.keys() == b.keys() and all(same(a[k], b[k], f"{path}.{k}") for k in a)
    if isinstance(a, (list, tuple)):
        return type(a) is type(b) and len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if isinstance(a, float):
        return isinstance(b, float) and (a == b or (a != a and b != b)) and math.copysign(1, a) == math.copysign(1, b)
    return type(a) is type(b) and a == b


def main():
    pl = FP.FollowPlannerNode()
    pl._p.update(CFG["follow_planner_node"]["ros__parameters"])
    pl._read_params()
    rng = np.random.default_rng(2026)
    n_kf = n_nav = n_fld = n_srch = n_gap = n_dwa = n_gap_ok = 0
    bad = []
    t0 = _real_perf()
    for sc in range(int(os.environ.get("SO_CANH", "36"))):
        P, (tx, ty) = scene(rng)
        set_state(pl, rng, P)
        pl._nav_memo = None
        # 1) _known_free: 6 diem sau duoi (nhu _reverse_ok), luoi 4 cm (nhu _esc_fields), diem rai rac
        c, s_ = math.cos(pl.robot_yaw), math.sin(pl.robot_yaw)
        Qs = []
        grids = []
        R6 = np.array([(-0.33 - rng.uniform(0, 0.2), rng.uniform(-0.24, 0.24)) for _ in range(6)])
        Qs.append(np.stack([pl.robot_x + c * R6[:, 0] - s_ * R6[:, 1], pl.robot_y + s_ * R6[:, 0] + c * R6[:, 1]], 1))
        for half in (1.5, 2.6):
            kc = -half + (np.arange(int(math.ceil(2 * half / 0.04))) + 0.5) * 0.04
            KX, KY = np.meshgrid(kc, kc)
            Qs.append(np.stack([pl.robot_x + c * KX.ravel() - s_ * KY.ravel(), pl.robot_y + s_ * KX.ravel() + c * KY.ravel()], 1))
            grids.append((Qs[-1], kc))
        Qs.append(np.stack([pl.robot_x + rng.uniform(-3, 3, 300), pl.robot_y + rng.uniform(-3, 3, 300)], 1))
        Qs.append(np.stack([pl.robot_x + rng.uniform(-3, 3, 2600), pl.robot_y + rng.uniform(-3, 3, 2600)], 1))
        Qs.append(np.zeros((0, 2)))
        for Q in Qs:
            for ts in (30.0, 90.0):
                n_kf += 1
                if not same(pl._known_free(Q, ts), ref_known_free(pl, Q, ts)):
                    bad.append(f"canh {sc}: _known_free Q {Q.shape} trail {ts}")
        # _known_free_grid (luoi 4 cm cua _esc_fields) == _known_free goc tren cung luoi. Them: CHI than xe (bo lich su
        # quet; vet thua 1/7 de o sot o mep khung bao lo ra), CHI vong quet (bo vet) — hai phan khong che lan nhau
        tr0, sh0 = pl.pose_trail, pl.scan_hist
        variants = [("du", tr0, sh0), ("than_xe", deque(list(tr0)[::7], maxlen=600), deque(maxlen=48)),
                    ("vong_quet", deque(maxlen=600), sh0)]
        for nm_v, tr_v, sh_v in variants:
            pl.pose_trail, pl.scan_hist = tr_v, sh_v
            for Q, kc in grids:
                for ts in (30.0, 90.0):
                    n_kf += 1
                    if not same(pl._known_free_grid(Q, kc, 0.04, ts), ref_known_free(pl, Q, ts)):
                        bad.append(f"canh {sc}: _known_free_grid ({nm_v}) Q {Q.shape} trail {ts}")
        pl.pose_trail, pl.scan_hist = tr0, sh0
        # 2) _nav_plan: chuoi goi nhu trong mot nhip (duong bam / duong tim khe / luoi xa / giu ben) — co trung dau vao
        calls = [(False, None, 0, 1.0), (True, 0.0, 0, 1.0), (False, None, 0, pl.esc_far_back), (True, 0.0, 0, pl.esc_far_back),
                 (False, None, 1, 1.0), (False, None, -1, 1.0), (True, None, 0, 1.0), (False, None, 0, 1.0)]
        navs = []
        for (mv, so, ps, bk) in calls:
            n_nav += 1
            a = pl._nav_plan(tx, ty, mv, standoff=so, prefer_side=ps, back=bk)
            b = ref_nav_plan(pl, tx, ty, mv, standoff=so, prefer_side=ps, back=bk)
            if not same(a, b):
                bad.append(f"canh {sc}: _nav_plan {(mv, so, ps, bk)}")
            navs.append(a)
        P2 = pl.obstacles_dense.copy()                     # mang MOI cung noi dung -> nho dem phai tinh lai, van ra dung so
        pl.obstacles_dense = P2 if P2.shape[0] else pl.obstacles_dense
        n_nav += 1
        if not same(pl._nav_plan(tx, ty, False), ref_nav_plan(pl, tx, ty, False)):
            bad.append(f"canh {sc}: _nav_plan sau khi doi mang")
        # 3) _esc_fields ban nhanh == ban sao goc (ref_esc_fields voi cac ham goc): khoa cu tung bit + vien dung
        for half, ts in ((1.5, pl.esc_trail_sec), (pl.esc_far_back, pl.esc_trail_sec), (1.5, 30.0)):
            n_fld += 1
            fa = pl._esc_fields(half=half, trail_sec=ts)
            with dung_ban_goc(pl):
                fb = ref_esc_fields(pl, half=half, trail_sec=ts)
            if not fields_same(fa, fb):
                bad.append(f"canh {sc}: _esc_fields half {half}")
        # 4) tim thoat ket: ban moi (_esc_search) vs generator goc chay toi het — cung ke hoach, cung so buoc
        nav = navs[0]
        jobs = []
        if nav is not None and "grid" in nav:
            hf = pl._esc_hfield(nav["grid"])
            if hf is not None:
                jobs.append((1.5, hf, None, 1.0))
                if sc % 3 == 0:
                    jobs.append((pl.esc_far_back, hf, None, 2.0))
        d_ = math.hypot(tx, ty)
        jobs.append((1.5, None, (tx, ty, 0.75, max(d_, pl.follow_distance) + 0.5, math.radians(12.0)), 1.0))
        # them: 'da biet trong' IT (chi 12 tu the cuoi, khong lich su quet) -> kiem cung mu / lui loai dong tac thuong xuyen
        if nav is not None and "grid" in nav and jobs and jobs[0][1] is not None:
            jobs.append((1.5, jobs[0][1], None, 1.0, "it_biet"))
        for job in jobs:
            half, hf, face, tsc = job[:4]
            tr0, sh0 = pl.pose_trail, pl.scan_hist
            if len(job) > 4:
                pl.pose_trail, pl.scan_hist = deque(list(tr0)[-12:], maxlen=600), deque(maxlen=48)
            f = pl._esc_fields(half=half, trail_sec=pl.esc_trail_sec)
            if f is None:
                pl.pose_trail, pl.scan_hist = tr0, sh0
                continue
            mx = pl.escape_max_exp * (2 if tsc > 1.0 else 1)
            n_srch += 1
            a = pl._esc_search(f, hf, mx, face=face, t_scale=tsc)
            ea = pl.esc_exp
            # ban GOC tron ven: truong tu ref_esc_fields, ham con goc (_esc_clear / _kfree / _blind / _hlook)
            with dung_ban_goc(pl):
                fr = ref_esc_fields(pl, half=half, trail_sec=pl.esc_trail_sec)
                gen = ref_esc_search_gen(pl, fr, hf, mx, face=face, t_scale=tsc)
                try:
                    while True:
                        next(gen)
                except StopIteration as si:
                    b = si.value
            eb = pl.esc_exp
            pl.pose_trail, pl.scan_hist = tr0, sh0
            if not same(a, b) or ea != eb:
                bad.append(f"canh {sc}: tim thoat ket half {half} face {face is not None} {job[4:]}: {a} / {b}, "
                           f"buoc {ea} / {eb}")
        # 5) _gap_refine (lam lai khe): canh CUA ngau nhien (+ do dac) va canh bay o tren; ket qua VA trang thai
        #    gap_axis_prev (ham ghi lai truc khe) phai giong het
        for k in range(8):
            Pg, g, tg = gap_scene(rng) if k < 6 else (P, {"mx": rng.uniform(0.3, 1.5), "my": rng.uniform(-0.6, 0.6),
                                                         "psi": rng.uniform(-0.6, 0.6), "gw": rng.uniform(0.7, 1.1)},
                                                     None)
            pl.obstacles_full = Pg
            for prev_ in (None, "gan"):
                if prev_ is None:
                    pv = None
                else:                                   # truc nhip truoc gan khe nay (khung odom), 0.4 s truoc
                    ox_, oy_ = pl._odom_of(g["mx"], g["my"])
                    pv = (wrap_pi(g["psi"] + pl.robot_yaw + rng.uniform(-0.2, 0.2)), ox_ + rng.uniform(-0.2, 0.2),
                          oy_ + rng.uniform(-0.2, 0.2), FAKE_T[0] - 0.4)
                for tgt in (None, tg):
                    n_gap += 1
                    pl.gap_axis_prev = pv
                    a = pl._gap_refine(dict(g), tgt)
                    sa = pl.gap_axis_prev
                    pl.gap_axis_prev = pv
                    b = ref_gap_refine(pl, dict(g), tgt)
                    sb = pl.gap_axis_prev
                    n_gap_ok += b is not None
                    if not same(a, b) or not same(sa, sb):
                        bad.append(f"canh {sc}: _gap_refine k {k} prev {prev_} tgt {tgt is not None}: {a} / {b}")
        pl.obstacles_full = P[np.hypot(P[:, 0], P[:, 1]) < 3.0]
        # 6) DWA dung tap diem BO DIEM TRUNG (_obst_uniq): rect_clearance_min giong tung bit tap goc (co diem trung)
        Pd = np.vstack([pl.obstacles, pl.obstacles[rng.integers(0, max(1, pl.obstacles.shape[0]), 40)]]) \
            if pl.obstacles.shape[0] else pl.obstacles
        pl.obstacles = Pd
        if Pd.shape[0]:
            n_dwa += 1
            nT, kT = 60, 12
            Vv, Ww = rng.uniform(-0.1, 0.4, nT), rng.uniform(-1.0, 1.0, nT)
            tt = np.arange(1, kT + 1) * 0.10
            thT = Ww[:, None] * tt[None, :]
            XT = (Vv / Ww)[:, None] * np.sin(thT)
            YT = (Vv / Ww)[:, None] * (1.0 - np.cos(thT))
            a = FP.rect_clearance_min(XT, YT, thT, pl._obst_uniq(), pl.front_len, pl.rear_len, pl.half_width)
            b = FP.rect_clearance_min(XT, YT, thT, Pd, pl.front_len, pl.rear_len, pl.half_width)
            if not same(a, b) or pl._obst_uniq().shape[0] >= Pd.shape[0]:
                bad.append(f"canh {sc}: DWA tap diem bo trung")
    dt = _real_perf() - t0
    print(f"  _known_free  : {n_kf} lan so (6 diem / luoi 4 cm / rai rac / rong; vet 30 s + 90 s)")
    print(f"  _nav_plan    : {n_nav} lan so (duong bam, tim khe, luoi xa, giu ben, mang moi)")
    print(f"  _esc_fields  : {n_fld} lan so (truong 1 cm + luoi da biet trong 4 cm)")
    print(f"  tim thoat ket: {n_srch} lan so (ban do + quay mat ve nguoi; ke hoach + so buoc tim)")
    print(f"  _gap_refine  : {n_gap} lan so ({n_gap_ok} ra khe; cua ngau nhien + do dac / canh bay; co / khong truc nhip "
          f"truoc, chan nguoi)")
    print(f"  DWA          : {n_dwa} lan so (tap diem bo trung vs tap goc)")
    for b_ in bad[:10]:
        print("  KHAC: " + b_)
    print(f"  ({dt:.0f} s)")
    print("=> DAT" if not bad else f"=> LOI ({len(bad)} cho khac)")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
