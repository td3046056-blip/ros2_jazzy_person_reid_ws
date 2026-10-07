"""
rssi_df.py — tim huong nguoi deo beacon bang RSSI khi XE XOAY TAI CHO ("xoay do huong").

Thuan numpy, KHONG phu thuoc ROS: dung chung cho rssi_bearing_node, scripts/rssi_rotate.py (do tren xe)
va scripts/sim_rssi.py (mo phong) — mot ma duy nhat cho ca ba.

Y TUONG
-------
Xe dung yen thi 3 board KHONG phan biet duoc nguoi o trai 45 hay phai 45 do (do 30/09). Khi xe xoay, moi
board lan luot quay mat ve phia nguoi: muc tin hieu cua no len xuong theo yaw. So khop duong cong do voi
MAU hinh dang lay tu mot lan hieu chinh (nguoi dung o huong da biet) -> ra huong nguoi trong khung ODOM.
Offset tung board, do lech do nguoi deo beacon = hang so cong them -> khong anh huong (mau va du lieu deu
tru trung binh). Do tren xe 01/10 (11 lan, beacon nhin thang xe): 11/11 sai <= 30 do, lap lai +-1..7 do.

DIEU KIEN DE TIN
----------------
  - xe da xoay >= ~250 do tai cho trong vai giay gan day (xoay 180 do: sai toi 44 do)
  - mau khop du tot VA khong mo ho: corr >= 0.45 & margin >= 0.10, hoac corr >= 0.35 & margin >= 0.25
  - beacon nhin thang ve xe (bi than nguoi che thi kem han — vong do 1)
  - xe chua di chuyen khoi cho xoay (di chuyen thi huong nguoi doi -> RotationDF tu xoa du lieu cu)
"""

from __future__ import annotations

import json
import math
import os
from collections import deque
from typing import Dict, Optional

import numpy as np

# Huong "nhin" cua tung board trong base_link (trai duong): board manh nhat khi nguoi o huong nay.
# Mac dinh theo vi tri lap (tam giac can canh 40/40/48 cm): A dau xe 0, B sau PHAI -114, C sau TRAI
# +114. Chay mot lan voi nguoi o huong da biet roi dung --calib de do lai tren xe that.
ALPHA_DEG = {"A": 0.0, "B": -114.0, "C": 114.0}

BOARDS = ("A", "B", "C")


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def estimate(t_r: np.ndarray, sid: np.ndarray, rssi: np.ndarray,
             t_o: np.ndarray, yaw_u: np.ndarray, alpha_deg: Dict[str, float],
             nbins: int = 36) -> Optional[dict]:
    """Huong nguoi (rad, khung odom) tu mot doan xe xoay >= 1 vong.

    Moi board: trung binh dB theo o yaw (10 do/o, chong lech do toc do xoay khong deu), roi lay
    hai bac nhat Z_i = mean[(m - mean m) e^{j yaw}]. Neu board manh nhat khi nguoi o huong
    alpha_i (base_link) thi arg(Z_i) = phi - alpha_i. Cong cac board: Z = sum Z_i e^{j alpha_i}.
    """
    keep = (t_r >= t_o[0]) & (t_r <= t_o[-1])
    yaw = np.interp(t_r[keep], t_o, yaw_u)
    psi = (yaw + math.pi) % (2.0 * math.pi) - math.pi
    b = np.clip(((psi + math.pi) / (2.0 * math.pi) * nbins).astype(int), 0, nbins - 1)
    centers = -math.pi + (np.arange(nbins) + 0.5) * 2.0 * math.pi / nbins
    Z = 0j
    boards = {}
    for s, a in alpha_deg.items():
        m = sid[keep] == s
        if m.sum() < nbins:
            continue
        sums = np.bincount(b[m], weights=rssi[keep][m], minlength=nbins)
        cnt = np.bincount(b[m], minlength=nbins)
        ok = cnt > 0
        if ok.mean() < 0.9:                     # board nay thieu o (xoay chua du vong / tin hieu duoi nguong
            continue                            # nghe) -> bo RIENG board nay; truoc day bo ca uoc luong
        mb = sums[ok] / cnt[ok]
        zi = np.mean((mb - mb.mean()) * np.exp(1j * centers[ok]))
        boards[s] = {"amp_db": 2.0 * abs(zi), "phi": wrap(float(np.angle(zi)) + math.radians(a)),
                     "raw_arg": float(np.angle(zi))}
        Z += zi * np.exp(1j * math.radians(a))
    if not boards:
        return None
    w = np.array([v["amp_db"] for v in boards.values()])
    ph = np.array([v["phi"] for v in boards.values()])
    agree = abs(np.sum(w * np.exp(1j * ph))) / max(1e-9, w.sum())   # 1 = cac board cung chi mot huong
    return {"phi": float(np.angle(Z)), "amp_db": 2.0 * abs(Z), "agree": float(agree), "boards": boards}


def build_template(t_r, sid, rssi, t_o, yaw_u, truth: float, nbins: int = 36) -> dict:
    """MAU hinh dang tung board theo goc nguoi nhin tu xe (0..2pi), tu mot lan xoay BIET huong nguoi
    (truth, khung odom). Mau da tru trung binh -> offset board / lech do nguoi deo khong anh huong."""
    keep = (t_r >= t_o[0]) & (t_r <= t_o[-1])
    yaw = np.interp(t_r[keep], t_o, yaw_u)
    th = np.mod(truth - yaw, 2.0 * math.pi)
    b = (th / (2.0 * math.pi) * nbins).astype(int) % nbins
    ctr = (np.arange(nbins) + 0.5) * 2.0 * math.pi / nbins
    tmpl = {}
    for s in BOARDS:
        m = sid[keep] == s
        cnt = np.bincount(b[m], minlength=nbins)
        ok = cnt > 0
        if ok.sum() < nbins // 2:
            continue
        v = np.bincount(b[m], weights=rssi[keep][m], minlength=nbins)[ok] / cnt[ok]
        full = np.interp(ctr, np.concatenate([ctr[ok] - 2.0 * math.pi, ctr[ok], ctr[ok] + 2.0 * math.pi]), np.tile(v, 3))
        tmpl[s] = (ctr, full - full.mean())
    return tmpl


def save_template(tmpl: dict, path: str, alpha_deg: Optional[dict] = None) -> None:
    out = {s: [round(float(x), 3) for x in v[1]] for s, v in tmpl.items()}
    if alpha_deg:
        out["alpha_deg"] = alpha_deg       # huong nhin tung board, de khoi phai go --alpha
    json.dump(out, open(path, "w"), indent=1)


def load_alpha(path: str) -> Optional[dict]:
    if not path or not os.path.exists(path):
        return None
    a = json.load(open(path)).get("alpha_deg")
    return {k: float(v) for k, v in a.items()} if a else None


def load_template(path: str) -> Optional[dict]:
    if not path or not os.path.exists(path):
        return None
    raw = json.load(open(path))
    out = {}
    for s, vals in raw.items():
        if s not in BOARDS:
            continue
        n = len(vals)
        out[s] = ((np.arange(n) + 0.5) * 2.0 * math.pi / n, np.array(vals, float))
    return out


def est_template(t, sid, rs, t_o, yaw_u, tmpl, nbins=36) -> Optional[float]:
    """So khop hinh dang mau (tu lan hieu chinh) voi song do duoc theo yaw — duoc ca khi xoay thieu vong."""
    keep = (t >= t_o[0]) & (t <= t_o[-1])
    psi = wrap(np.interp(t[keep], t_o, yaw_u))
    b = np.clip(((psi + np.pi) / (2.0 * np.pi) * nbins).astype(int), 0, nbins - 1)
    centers = -np.pi + (np.arange(nbins) + 0.5) * 2.0 * np.pi / nbins
    phis = np.radians(np.arange(-180.0, 180.0, 5.0))
    score = np.zeros(len(phis))
    used = 0
    for s in BOARDS:
        if s not in tmpl:
            continue
        m = sid[keep] == s
        cnt = np.bincount(b[m], minlength=nbins)
        ok = cnt > 0
        if ok.sum() < 6:
            continue
        x = np.bincount(b[m], weights=rs[keep][m], minlength=nbins)[ok] / cnt[ok]
        x = x - x.mean()
        th = np.mod(phis[:, None] - centers[ok][None, :], 2.0 * np.pi)
        T = np.interp(th, np.append(tmpl[s][0], 2.0 * np.pi), np.append(tmpl[s][1], tmpl[s][1][0]))
        T = T - T.mean(1, keepdims=True)
        score += (T * x[None, :]).sum(1) / max(1e-6, float(np.sqrt((x ** 2).sum())))
        used += 1
    return float(phis[np.argmax(score)]) if used else None


def est_template_moving(t, sid, rs, t_o, yaw_u, tmpl, t_end, margin: Optional[float] = None) -> Optional[float]:
    """Nhu est_template nhung cho phep nguoi DI trong luc xoay: tim ca huong luc ket thuc (phi) lan toc do
    goc cua nguoi (omega). Goc nguoi nhin tu xe tai mau t: phi + omega (t - t_end) - yaw(t). Khop tung mau."""
    keep = (t >= t_o[0]) & (t <= t_o[-1])
    tk, sk, rk = t[keep], sid[keep], rs[keep]
    yaw = np.interp(tk, t_o, yaw_u)
    phis = np.radians(np.arange(-180.0, 180.0, 5.0))
    omegas = np.arange(-0.30, 0.301, 0.05)
    score = np.zeros((len(omegas), len(phis)))
    used = 0
    for s in BOARDS:
        if s not in tmpl:
            continue
        m = sk == s
        if m.sum() < 20:
            continue
        x = rk[m] - rk[m].mean()
        dt = tk[m] - t_end
        th = np.mod(phis[None, :, None] + omegas[:, None, None] * dt[None, None, :] - yaw[m][None, None, :], 2.0 * np.pi)
        T = np.interp(th, np.append(tmpl[s][0], 2.0 * np.pi), np.append(tmpl[s][1], tmpl[s][1][0]))
        T = T - T.mean(2, keepdims=True)
        score += (T * x[None, None, :]).sum(2) / max(1e-6, float(np.sqrt((x ** 2).sum())))
        used += 1
    if not used:
        return None
    i, j = np.unravel_index(np.argmax(score), score.shape)
    if margin is not None:
        # Ket hop: chi tin mo hinh "nguoi dang di" khi no khop TOT HON han mo hinh dung yen (omega = 0);
        # khong thi them mot an so chi lam tang sai so khi nguoi dung yen
        i0 = int(np.argmin(np.abs(omegas)))
        j0 = int(np.argmax(score[i0]))
        if score[i, j] < score[i0, j0] + margin * abs(score[i0, j0]):
            return float(phis[j0])
    return float(phis[j])


def match_template(t, sid, rs, t_o, yaw_u, tmpl, nbins: int = 36) -> Optional[dict]:
    """Nhu est_template nhung tra ve ca CHAT LUONG khop de biet co nen tin hay khong:
      phi     huong nguoi (rad, khung odom) — cung cach chon voi est_template
      corr    he so tuong quan giua song do duoc va mau tai phi (TB co trong so theo board), 1 = khop hoan hao
      margin  diem tot nhat hon diem tot nhat o CHO KHAC (lech >= 40 do) bao nhieu phan — nho = mo ho
      cover   ti le o yaw co du lieu (TB cac board dung duoc)
    """
    keep = (t >= t_o[0]) & (t <= t_o[-1])
    if keep.sum() < 10:
        return None
    psi = wrap(np.interp(t[keep], t_o, yaw_u))
    b = np.clip(((psi + np.pi) / (2.0 * np.pi) * nbins).astype(int), 0, nbins - 1)
    centers = -np.pi + (np.arange(nbins) + 0.5) * 2.0 * np.pi / nbins
    phis = np.radians(np.arange(-180.0, 180.0, 5.0))
    score = np.zeros(len(phis))
    parts = []
    for s in BOARDS:
        if s not in tmpl:
            continue
        m = sid[keep] == s
        cnt = np.bincount(b[m], minlength=nbins)
        ok = cnt > 0
        if ok.sum() < 6:
            continue
        x = np.bincount(b[m], weights=rs[keep][m], minlength=nbins)[ok] / cnt[ok]
        x = x - x.mean()
        nx = max(1e-6, float(np.sqrt((x ** 2).sum())))
        th = np.mod(phis[:, None] - centers[ok][None, :], 2.0 * np.pi)
        T = np.interp(th, np.append(tmpl[s][0], 2.0 * np.pi), np.append(tmpl[s][1], tmpl[s][1][0]))
        T = T - T.mean(1, keepdims=True)
        sc = (T * x[None, :]).sum(1) / nx
        score += sc
        parts.append((sc, np.sqrt((T ** 2).sum(1)), nx, float(ok.mean())))
    if not parts:
        return None
    j = int(np.argmax(score))
    w = np.array([p[2] for p in parts])
    corr = float(sum(p[2] * (p[0][j] / max(1e-6, p[1][j])) for p in parts) / w.sum())
    far = np.abs(wrap(phis - phis[j])) >= math.radians(40.0)
    margin = float((score[j] - score[far].max()) / max(1e-6, abs(score[j])))
    return {"phi": float(phis[j]), "corr": corr, "margin": margin, "cover": float(np.mean([p[3] for p in parts])),
            "n_boards": len(parts)}


class RotationDF:
    """Gom mau RSSI + odom theo thoi gian thuc va uoc luong huong nguoi tu lan XOAY TAI CHO gan day.

    Dung:  df.add_odom(t, x, y, yaw); df.add_sample(t, "A", -63); r = df.estimate(now)
    Du lieu chi giu trong window_sec va bi XOA khi xe di chuyen qua reset_dist_m (dung o cho khac thi huong
    nguoi nhin tu xe da doi). r["valid"] chi True khi da xoay du min_sweep_deg va mau khop du tot.
    """

    def __init__(self, tmpl: dict, window_sec: float = 14.0, min_sweep_deg: float = 250.0,
                 reset_dist_m: float = 0.30, min_corr: float = 0.45, min_margin: float = 0.10,
                 relax_corr: float = 0.35, relax_margin: float = 0.25) -> None:
        self.tmpl = tmpl
        self.window_sec = float(window_sec)
        self.min_sweep = math.radians(float(min_sweep_deg))
        self.reset_dist = float(reset_dist_m)
        self.min_corr = float(min_corr)
        self.min_margin = float(min_margin)
        # Khop hoi kem (corr >= relax_corr) nhung KHONG mo ho (margin >= relax_margin) thi van tin. 182 cua so quet
        # cat tu 14 lan do that: them 13 cua so duoc nhan, khong them lan nao sai > 45 do (01/10).
        self.relax_corr = float(relax_corr)
        self.relax_margin = float(relax_margin)
        self.odom: deque = deque()            # (t, yaw lien tuc)
        self.samples: deque = deque()         # (t, board, rssi)
        self._yaw_prev: Optional[float] = None
        self._yaw_u = 0.0
        self._anchor: Optional[tuple] = None
        self.last_sample_t: Optional[float] = None
        self.n_reset = 0

    def reset(self) -> None:
        self.odom.clear()
        self.samples.clear()
        self._anchor = None
        self.n_reset += 1

    def add_odom(self, t: float, x: float, y: float, yaw: float) -> None:
        if self._yaw_prev is not None:
            self._yaw_u += float(wrap(yaw - self._yaw_prev))
        else:
            self._yaw_u = float(yaw)
        self._yaw_prev = yaw
        if self._anchor is None:
            self._anchor = (x, y)
        elif math.hypot(x - self._anchor[0], y - self._anchor[1]) > self.reset_dist:
            self.reset()                      # xe da doi cho: mau cu khong con dung
            self._anchor = (x, y)
        self.odom.append((t, self._yaw_u))
        self._prune(t)

    def add_sample(self, t: float, board: str, rssi: float) -> None:
        self.samples.append((t, board, float(rssi)))
        self.last_sample_t = t

    def _prune(self, now: float) -> None:
        lim = now - self.window_sec
        while self.odom and self.odom[0][0] < lim:
            self.odom.popleft()
        while self.samples and self.samples[0][0] < lim:
            self.samples.popleft()

    def estimate(self, now: float) -> dict:
        out = {"valid": False, "reason": "", "swept_deg": 0.0, "n_samples": len(self.samples),
               "beacon_age": None if self.last_sample_t is None else now - self.last_sample_t, "levels": {}}
        self._prune(now)
        recent = [(b, r) for t, b, r in self.samples if t >= now - 2.0]
        for s in BOARDS:
            v = [r for b, r in recent if b == s]
            if v:
                out["levels"][s] = sum(v) / len(v)
        if len(self.odom) < 5 or len(self.samples) < 30:
            out["reason"] = "chua du du lieu"
            return out
        t_o = np.array([o[0] for o in self.odom])
        y_o = np.array([o[1] for o in self.odom])
        swept = float(y_o.max() - y_o.min())
        out["swept_deg"] = math.degrees(swept)
        t = np.array([s[0] for s in self.samples])
        sid = np.array([s[1] for s in self.samples])
        rs = np.array([s[2] for s in self.samples])
        res = match_template(t, sid, rs, t_o, y_o, self.tmpl)
        if res is None:
            out["reason"] = "khong khop duoc mau"
            return out
        out.update({"bearing_odom": res["phi"], "corr": res["corr"], "margin": res["margin"],
                    "cover": res["cover"], "yaw_now": float(wrap(y_o[-1]))})
        out["bearing_base"] = float(wrap(res["phi"] - y_o[-1]))
        good = res["corr"] >= self.min_corr and res["margin"] >= self.min_margin
        clear = res["corr"] >= self.relax_corr and res["margin"] >= self.relax_margin
        if swept < self.min_sweep:
            out["reason"] = "xoay chua du"
        elif good or clear:
            out["valid"] = True
        elif res["corr"] < self.min_corr:
            out["reason"] = "khop mau kem"
        else:
            out["reason"] = "mo ho (hai huong khop gan bang nhau)"
        return out
