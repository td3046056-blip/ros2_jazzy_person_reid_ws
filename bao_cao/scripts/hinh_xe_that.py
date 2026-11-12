#!/usr/bin/env python3
"""Hinh tu BAG CHAY THAT (07/11): ban do phong dung tu LiDAR + quy dao xe + vi tri nguoi + thoat ket; dien bien theo
thoi gian (khoang cach, trang thai, lenh v/w, nguon tracker).

Can: ~/bao_cao_cache/<bag>.pkl (tao bang src/person_follow_nav/scripts/dump_bag.py, can ROS).
    python3 hinh_xe_that.py
"""
import json
import math
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kieu_hinh import *  # noqa: F401,F403,E402
from kieu_hinh import CACHE, PKG_NAV, RONG, luu, ve_xe, bang_mau_trang_thai  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

sys.path.insert(0, PKG_NAV)
from person_follow_nav import geometry as G  # noqa: E402

BLIND = [[246.0, 294.0]]


def doc(bag):
    D = pickle.load(open(os.path.join(CACHE, bag + ".pkl"), "rb"))
    t0 = D["/odom"][0][0]
    od = D["/odom"]
    T = np.array([t - t0 for t, _ in od])
    X = np.array([o["x"] for _, o in od])
    Y = np.array([o["y"] for _, o in od])
    YAW = np.unwrap(np.array([G.yaw_from_quaternion(o["qx"], o["qy"], o["qz"], o["qw"]) for _, o in od]))
    ps = [(t - t0, json.loads(s)) for t, s in D["/follow/planner_status"]]
    tg = [(t - t0, json.loads(s)) for t, s in D["/follow/target"]]
    cmd = [(t - t0, c["v"], c["w"]) for t, c in D["/cmd_vel"]]
    sc = [(t - t0, s) for t, s in D["/scan"]]
    return dict(T=T, X=X, Y=Y, YAW=YAW, ps=ps, tg=tg, cmd=cmd, sc=sc)


def pose_at(B, t):
    i = int(np.clip(np.searchsorted(B["T"], t), 1, len(B["T"]) - 1))
    a = (t - B["T"][i - 1]) / max(1e-6, B["T"][i] - B["T"][i - 1])
    return (B["X"][i - 1] + a * (B["X"][i] - B["X"][i - 1]), B["Y"][i - 1] + a * (B["Y"][i] - B["Y"][i - 1]),
            B["YAW"][i - 1] + a * (B["YAW"][i] - B["YAW"][i - 1]))


def diem_quet(B, t_lo, t_hi, buoc=2):
    """Diem LiDAR (khung odom) cua cac vong quet trong [t_lo, t_hi], da loc than xe nhu planner."""
    out = []
    for k, (t, s) in enumerate(B["sc"]):
        if t < t_lo or t > t_hi or k % buoc:
            continue
        pts, bear, rng, ldeg = G.scan_to_base_points(np.array(s["ranges"], dtype=float), s["angle_min"],
                                                    s["angle_increment"], s["range_min"], s["range_max"],
                                                    0.10, 0.0, math.radians(-90.0), 1.0, 5.0)
        if pts.shape[0] == 0:
            continue
        keep = G.self_filter_mask(pts, ldeg, F_LEN, R_LEN, HW, 0.03, BLIND)
        pts = pts[keep]
        x, y, th = pose_at(B, t)
        c, s_ = math.cos(th), math.sin(th)
        out.append(np.stack([x + c * pts[:, 0] - s_ * pts[:, 1], y + s_ * pts[:, 0] + c * pts[:, 1]], axis=1))
    return np.concatenate(out) if out else np.zeros((0, 2))


def ve_ban_do(ax, P, res=0.03, nguong=4):
    """Luoi dem diem -> o co it nhat `nguong` lan cham = vat (giong ban do chiem cho)."""
    if P.shape[0] == 0:
        return
    x0, y0 = P.min(axis=0) - 0.2
    x1, y1 = P.max(axis=0) + 0.2
    nx, ny = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
    H = np.zeros((ny, nx))
    ix = ((P[:, 0] - x0) / res).astype(int)
    iy = ((P[:, 1] - y0) / res).astype(int)
    np.add.at(H, (iy, ix), 1)
    M = np.ma.masked_less(H, nguong)
    ax.imshow(np.log1p(M), origin="lower", extent=(x0, x1, y0, y1), cmap="Greys", vmin=0, vmax=np.log1p(60),
              interpolation="nearest", zorder=1)


def nhan_trang_thai(B, ts):
    tps = np.array([t for t, _ in B["ps"]])
    sts = [s.get("state") if s.get("enabled") else "IDLE" for _, s in B["ps"]]
    idx = np.clip(np.searchsorted(tps, ts), 0, len(sts) - 1)
    return [sts[i] for i in idx]


def cum_thoat_ket(B, t_lo, t_hi):
    ev, last = [], -99.0
    for t, s in B["ps"]:
        n = s.get("note") or ""
        if t_lo <= t <= t_hi and n.startswith("thoat ket"):
            if t - last > 4.0:
                ev.append(t)
            last = t
    return ev


def hinh_ban_do(bag, t_lo, t_hi, ten, tieu_de, buoc_xe=4.0):
    B = doc(bag)
    P = diem_quet(B, t_lo, t_hi)
    fig, ax = plt.subplots(figsize=(RONG, RONG * 0.78))
    ve_ban_do(ax, P)
    m = (B["T"] >= t_lo) & (B["T"] <= t_hi)
    T, X, Y, YAW = B["T"][m], B["X"][m], B["Y"][m], B["YAW"][m]
    st = nhan_trang_thai(B, T)
    cm = bang_mau_trang_thai()
    seg = np.stack([np.stack([X[:-1], Y[:-1]], 1), np.stack([X[1:], Y[1:]], 1)], axis=1)
    ax.add_collection(LineCollection(seg, colors=[cm.get(s, "#999") for s in st[:-1]], lw=1.6, zorder=4))
    # vi tri nguoi tracker uoc luong (chi luc camera thay)
    pp = np.array([[s["odom_x"], s["odom_y"]] for t, s in B["tg"]
                   if t_lo <= t <= t_hi and s.get("valid") and str(s.get("source", "")).startswith("camera")
                   and s.get("odom_x") is not None])
    if pp.shape[0]:
        ax.scatter(pp[::5, 0], pp[::5, 1], s=3, c=MAU_NGUOI, alpha=0.35, lw=0, zorder=3,
                   label="người (tracker ước lượng)")
    t_last = -1e9
    for t, x, y, th in zip(T, X, Y, YAW):
        if t - t_last >= buoc_xe:
            ve_xe(ax, x, y, th, mau="#d95f02", alpha=0.10, lw=0.5, mui=False)
            t_last = t
    for k, te in enumerate(cum_thoat_ket(B, t_lo, t_hi), 1):
        x, y, _ = pose_at(B, te)
        ax.plot(x, y, marker="*", ms=11, mfc="#d62728", mec="white", mew=0.6, zorder=6)
        ax.annotate(str(k), (x, y), xytext=(4, 4), textcoords="offset points", fontsize=7, color="#d62728",
                    zorder=7)
    ax.plot(X[0], Y[0], "o", ms=6, mfc="white", mec="k", zorder=6, label="điểm bắt đầu")
    ax.plot([], [], "*", ms=9, mfc="#d62728", mec="white", label="bắt đầu thoát kẹt")
    for s in ("FOLLOW", "AVOID", "ARRIVED", "SEARCH", "OCCLUDED"):
        if s in st:
            ax.plot([], [], color=cm[s], lw=2.5, label=s)
    ax.set_aspect("equal")
    ax.set_xlim(min(X.min(), pp[:, 0].min() if pp.shape[0] else X.min()) - 1.2,
                max(X.max(), pp[:, 0].max() if pp.shape[0] else X.max()) + 1.2)
    ax.set_ylim(min(Y.min(), pp[:, 1].min() if pp.shape[0] else Y.min()) - 1.2,
                max(Y.max(), pp[:, 1].max() if pp.shape[0] else Y.max()) + 1.2)
    ax.set_xlabel("x (m, khung odom)")
    ax.set_ylabel("y (m)")
    ax.set_title(tieu_de)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False)
    dist = float(np.sum(np.hypot(np.diff(X), np.diff(Y))))
    ax.text(0.01, 0.01, f"{t_hi - t_lo:.0f} s, xe đi {dist:.1f} m", transform=ax.transAxes, fontsize=7,
            va="bottom", bbox=dict(fc="white", ec="none", alpha=0.7))
    return luu(fig, ten)


def hinh_dien_bien(bag, t_lo, t_hi, ten, tieu_de):
    B = doc(bag)
    cm = bang_mau_trang_thai()
    fig, axs = plt.subplots(4, 1, figsize=(RONG, 5.6), sharex=True,
                            gridspec_kw=dict(height_ratios=[2.2, 0.55, 2.0, 0.55], hspace=0.12))
    tg = [(t, s) for t, s in B["tg"] if t_lo <= t <= t_hi]
    td = np.array([t for t, s in tg if s.get("valid") and s.get("distance_m") is not None])
    dd = np.array([s["distance_m"] for t, s in tg if s.get("valid") and s.get("distance_m") is not None])
    axs[0].plot(td, dd, color=MAU_NGUOI, lw=1.0, label="khoảng cách tới người (tracker)")
    axs[0].axhline(1.0, color="k", ls="--", lw=0.8, label="khoảng cách bám mong muốn 1.0 m")
    axs[0].set_ylabel("khoảng cách (m)")
    axs[0].set_ylim(0.5, max(4.0, float(dd.max()) + 0.9) if dd.size else 4.0)
    # dai trang thai planner
    ps = [(t, s.get("state") if s.get("enabled") else "IDLE") for t, s in B["ps"] if t_lo <= t <= t_hi]
    for (t, s), (t2, _) in zip(ps[:-1], ps[1:]):
        axs[1].axvspan(t, t2, color=cm.get(s, "#999"), lw=0)
    axs[1].set_yticks([])
    axs[1].set_ylabel("trạng thái", rotation=0, ha="right", va="center")
    axs[1].grid(False)
    for s in ("FOLLOW", "AVOID", "ARRIVED", "SEARCH", "OCCLUDED", "BLOCKED"):
        if any(x == s for _, x in ps):
            axs[1].plot([], [], color=cm[s], lw=5, label=s)
    axs[1].legend(loc="center left", bbox_to_anchor=(1.005, 0.5), ncol=1, frameon=False, fontsize=6.5)
    cmd = np.array([c for c in B["cmd"] if t_lo <= c[0] <= t_hi])
    axs[2].plot(cmd[:, 0], cmd[:, 1], color="#1f77b4", lw=0.9, label="v (m/s)")
    axs[2].plot(cmd[:, 0], cmd[:, 2], color="#d95f02", lw=0.7, alpha=0.8, label="ω (rad/s)")
    axs[2].axhline(0, color="k", lw=0.5)
    axs[2].set_ylabel("lệnh /cmd_vel")
    axs[2].legend(loc="upper left", ncol=2, frameon=True, framealpha=0.85, edgecolor="none", fontsize=7)
    src_c = {"camera+lidar": "#1b9e77", "camera+bbox": "#66a61e", "lidar_track": "#e6ab02", "predicted": "#e7298a",
             "rssi_bearing": "#7570b3"}
    so = [(t, s.get("source") if s.get("valid") else "mất") for t, s in tg]
    for (t, s), (t2, _) in zip(so[:-1], so[1:]):
        axs[3].axvspan(t, t2, color=src_c.get(s, "#dddddd"), lw=0)
    axs[3].set_yticks([])
    axs[3].set_ylabel("nguồn", rotation=0, ha="right", va="center")
    axs[3].grid(False)
    for s, c in src_c.items():
        if any(x == s for _, x in so):
            axs[3].plot([], [], color=c, lw=5, label=s)
    axs[3].plot([], [], color="#dddddd", lw=5, label="mất người")
    axs[3].legend(loc="center left", bbox_to_anchor=(1.005, 0.5), ncol=1, frameon=False, fontsize=6.5)
    for te in cum_thoat_ket(B, t_lo, t_hi):
        for a in axs[:3]:
            a.axvline(te, color="#d62728", lw=0.8, ls=":")
    axs[0].plot([], [], color="#d62728", ls=":", lw=0.8, label="bắt đầu thoát kẹt")
    axs[0].legend(loc="upper left", ncol=3, frameon=True, framealpha=0.85, edgecolor="none", fontsize=7)
    axs[3].set_xlabel("thời gian (s, tính từ đầu bag)")
    axs[0].set_title(tieu_de)
    axs[3].set_xlim(t_lo, t_hi)
    return luu(fig, ten)


if __name__ == "__main__":
    hinh_ban_do("follow_1107_104423", 330, 450, "xe_that_ban_do_1107_104423",
                "Chạy thật 07/11: bản đồ LiDAR, quỹ đạo xe và vị trí người (330–450 s)", buoc_xe=6.0)
    hinh_ban_do("follow_1107_112843", 150, 240, "xe_that_ban_do_1107_112843",
                "Chạy thật 07/11 (bag follow_1107_112843, 150–240 s)", buoc_xe=6.0)
    hinh_dien_bien("follow_1107_104423", 330, 450, "xe_that_dien_bien_1107_104423",
                   "Diễn biến khi bám người qua chỗ hẹp (chạy thật 07/11, 330–450 s)")
