#!/usr/bin/env python3
"""Hinh he thong + hieu chinh: hinh hoc xe, toc do driver, loc than xe tren vong quet that, mo hinh goc camera.
So lieu do lay tu CLAUDE.md muc 2-3 va CALIBRATION.md (ghi kem trong tung ham)."""
import math
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kieu_hinh import *  # noqa: F401,F403,E402
from kieu_hinh import CACHE, PKG_NAV, RONG, NUA, luu, footprint  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Circle, Wedge  # noqa: E402

sys.path.insert(0, PKG_NAV)
from person_follow_nav import geometry as G  # noqa: E402


def hinh_hoc_xe():
    """Footprint chu nhat, base_link o truc banh TRUOC (tam quay), ban kinh quet goc truoc / sau, cung mu LiDAR,
    goc nhin camera. So lieu: CLAUDE.md muc 2."""
    fig, ax = plt.subplots(figsize=(RONG * 0.62, RONG * 0.62))
    # vong quet khi xoay tai cho
    for r, ls, lb in ((math.hypot(F_LEN, HW), ":", "bán kính góc trước 0,331 m"),
                      (math.hypot(R_LEN, HW), "--", "bán kính góc sau 0,446 m"),
                      (0.47, "-", "rotate_radius 0,47 m")):
        ax.add_patch(Circle((0, 0), r, fill=False, ls=ls, lw=0.9, ec="#7570b3"))
        ax.plot([], [], ls=ls, color="#7570b3", lw=0.9, label=lb)
    # cung mu LiDAR [246, 294] do khung lidar -> base = lidar - 90 -> [156, 204] do, goc tai LiDAR (x = 0.10)
    ax.add_patch(Wedge((LIDAR_X, 0), 0.62, 156, 204, fc="#d62728", alpha=0.13, ec="none"))
    ax.plot([], [], color="#d62728", alpha=0.35, lw=6, label="cung mù LiDAR (bỏ tia thân xe)")
    # goc nhin camera +-57.2 do (mat ca 114.4 do)
    ax.add_patch(Wedge((0.05, 0), 0.75, -57.2, 57.2, fc="#2ca02c", alpha=0.08, ec="#2ca02c", lw=0.6, ls="--"))
    ax.plot([], [], color="#2ca02c", alpha=0.3, lw=6, label="góc nhìn camera 114,4°")
    # than xe
    ax.add_patch(Polygon(footprint(0, 0, 0), closed=True, fc="#d95f02", alpha=0.25, ec="#d95f02", lw=1.2))
    # banh xe
    for sy in (1, -1):
        ax.add_patch(Polygon([(-0.063, sy * 0.17), (0.063, sy * 0.17), (0.063, sy * 0.23), (-0.063, sy * 0.23)],
                             fc="#333333", ec="none"))
    ax.plot(0, 0, "k+", ms=10, mew=1.5)
    ax.annotate("base_link = tâm quay\n(trục bánh trước)", (0, 0), xytext=(-0.42, -0.62), fontsize=7,
                arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.plot(LIDAR_X, 0, "o", ms=6, mfc="#1f77b4", mec="white")
    ax.annotate("LiDAR (x = 0,10 m)", (LIDAR_X, 0), xytext=(0.22, 0.40), fontsize=7,
                arrowprops=dict(arrowstyle="-", lw=0.6))
    # kich thuoc
    def kich(x1, y1, x2, y2, t, dx=0, dy=0):
        ax.annotate("", (x1, y1), (x2, y2), arrowprops=dict(arrowstyle="<->", lw=0.7))
        ax.text((x1 + x2) / 2 + dx, (y1 + y2) / 2 + dy, t, fontsize=7, ha="center", va="center",
                bbox=dict(fc="white", ec="none", pad=0.5))
    kich(0, -0.36, F_LEN, -0.36, "0,14", dy=-0.035)
    kich(-R_LEN, -0.36, 0, -0.36, "0,33", dy=-0.035)
    kich(-0.42, -HW, -0.42, HW, "0,60", dx=-0.05)
    ax.annotate("mũi xe", (F_LEN, 0.0), xytext=(0.30, -0.12), fontsize=7, arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.set_aspect("equal")
    ax.set_xlim(-0.75, 0.85)
    ax.set_ylim(-0.75, 0.75)
    ax.set_xlabel("x (m) — hướng mũi xe")
    ax.set_ylabel("y (m) — bên trái")
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False, fontsize=7)
    ax.set_title("Hình học xe BW-DR03 dùng trong planner")
    return luu(fig, "he_thong_hinh_hoc_xe")


def toc_do_driver():
    """measure_speed.py (CALIBRATION.md): 13 % -> 0.107, 29 % -> 0.238, 58 % -> 0.475 m/s; xoay 43 % -> 1.763 rad/s.
    max_linear = 60 * 0.475 / 58 = 0.49; max_angular = 60 * 1.763 / 43 = 2.46. Kiem chung 17/09: lenh 0.22 m/s ->
    thuoc 64.5 cm / odom 64.0 cm (0.213 m/s); lenh xoay 0.8 rad/s -> that 135 do / odom 130.9 do."""
    p = np.array([13, 29, 58])
    v = np.array([0.107, 0.238, 0.475])
    k = float(np.sum(p * v) / np.sum(p * p))
    fig, axs = plt.subplots(1, 2, figsize=(RONG, 2.6))
    ax = axs[0]
    xx = np.linspace(0, 62, 50)
    ax.plot(xx, k * xx, color="#1f77b4", lw=1, label=f"khớp qua gốc: {1000 * k:.2f} mm/s mỗi %")
    ax.plot(p, v, "o", color="#d95f02", ms=6, label="đo (measure_speed.py)")
    ax.plot(60, 0.49, "s", color="k", ms=5)
    ax.annotate("max_percent 60 %\n→ max_linear 0,49 m/s", (59, 0.49), xytext=(2, 0.465), fontsize=7,
                va="center", arrowprops=dict(arrowstyle="->", lw=0.6))
    ax.set_xlabel("độ rộng xung PWM (%)")
    ax.set_ylabel("tốc độ thật (m/s)")
    ax.set_title("(a) Tốc độ tiến theo % PWM", fontsize=9)
    ax.legend(fontsize=6.8, frameon=False, loc="lower right")
    ax.set_xlim(0, 65)
    ax.set_ylim(0, 0.56)
    ax = axs[1]
    lab = ["tiến 0,22 m/s", "xoay 0,80 rad/s"]
    cmd = [0.22, 0.80]
    that = [0.213, 0.80 * 135 / 135]     # thuoc / goc that do bang mat
    odom = [0.640 / 0.645 * 0.213, 0.80 * 130.9 / 135]
    x = np.arange(2)
    ax.bar(x - 0.25, [1.0, 1.0], 0.25, color="#cccccc", label="lệnh")
    ax.bar(x, [that[0] / cmd[0], that[1] / cmd[1]], 0.25, color="#d95f02", label="đo thật (thước / góc)")
    ax.bar(x + 0.25, [odom[0] / cmd[0], odom[1] / cmd[1]], 0.25, color="#1f77b4", label="odom")
    for i, (a, b) in enumerate(zip(that, odom)):
        ax.text(i, a / cmd[i] + 0.01, f"{a / cmd[i]:.2f}", ha="center", fontsize=6.5)
        ax.text(i + 0.25, b / cmd[i] + 0.01, f"{b / cmd[i]:.2f}", ha="center", fontsize=6.5)
    ax.set_xticks(x)
    ax.set_xticklabels(lab)
    ax.set_ylim(0.8, 1.12)
    ax.set_ylabel("tỉ lệ so với lệnh")
    ax.set_title("(b) Kiểm chứng sau khi sửa (17/09)", fontsize=9)
    ax.legend(fontsize=6.6, frameon=False, loc="upper center", ncol=3, handlelength=1.2, columnspacing=0.8)
    fig.tight_layout()
    return luu(fig, "hieu_chinh_toc_do_driver")


def loc_than_xe(bag="follow_1107_104423", t_max=5.0):
    """Vong quet THAT luc xe dung yen dau bag: diem giu lai / bi loai vi nam trong footprint / trong cung mu."""
    D = pickle.load(open(os.path.join(CACHE, bag + ".pkl"), "rb"))
    t0 = D["/odom"][0][0]
    keep_p, fp_p, bl_p = [], [], []
    n = 0
    for t, s in D["/scan"]:
        if t - t0 > t_max:
            break
        n += 1
        pts, bear, rng, ldeg = G.scan_to_base_points(np.array(s["ranges"], dtype=float), s["angle_min"],
                                                    s["angle_increment"], s["range_min"], s["range_max"],
                                                    0.10, 0.0, math.radians(-90.0), 1.0, 3.0)
        keep = G.self_filter_mask(pts, ldeg, F_LEN, R_LEN, HW, 0.03, [[246.0, 294.0]])
        blind = (ldeg >= 246.0) & (ldeg <= 294.0)
        keep_p.append(pts[keep])
        bl_p.append(pts[~keep & blind])
        fp_p.append(pts[~keep & ~blind])
    K, FP, BL = (np.concatenate(a) if a else np.zeros((0, 2)) for a in (keep_p, fp_p, bl_p))
    fig, ax = plt.subplots(figsize=(RONG * 0.62, RONG * 0.62))
    ax.add_patch(Wedge((LIDAR_X, 0), 1.3, 156, 204, fc="#d62728", alpha=0.08, ec="none"))
    ax.add_patch(Polygon(footprint(0, 0, 0), closed=True, fc="#d95f02", alpha=0.15, ec="#d95f02", lw=1.0))
    ax.scatter(K[:, 0], K[:, 1], s=1.5, c="#333333", lw=0, label=f"giữ lại: {K.shape[0]} điểm (vật cản)")
    ax.scatter(BL[:, 0], BL[:, 1], s=5, c="#d62728", lw=0, label=f"bỏ: trong cung mù [246°, 294°] ({BL.shape[0]})")
    ax.scatter(FP[:, 0], FP[:, 1], s=5, c="#1f77b4", lw=0, label=f"bỏ: nằm trong footprint ({FP.shape[0]})")
    ax.plot(LIDAR_X, 0, "^", ms=5, mfc="#2ca02c", mec="white")
    # khung phong to quanh than xe
    ins = ax.inset_axes([1.06, 0.0, 0.62, 0.62])
    ins.add_patch(Polygon(footprint(0, 0, 0), closed=True, fc="#d95f02", alpha=0.15, ec="#d95f02", lw=1.0))
    ins.add_patch(Wedge((LIDAR_X, 0), 0.5, 156, 204, fc="#d62728", alpha=0.08, ec="none"))
    ins.scatter(K[:, 0], K[:, 1], s=3, c="#333333", lw=0)
    ins.scatter(BL[:, 0], BL[:, 1], s=9, c="#d62728", lw=0)
    ins.scatter(FP[:, 0], FP[:, 1], s=9, c="#1f77b4", lw=0)
    ins.plot(LIDAR_X, 0, "^", ms=6, mfc="#2ca02c", mec="white")
    ins.annotate("LiDAR", (LIDAR_X, 0), xytext=(0.12, 0.15), fontsize=6.5, arrowprops=dict(arrowstyle="-", lw=0.5))
    ins.annotate("tia đập thân xe\n(cách LiDAR ~0,13 m)", (-0.03, 0.0), xytext=(-0.38, -0.27), fontsize=6.5,
                 arrowprops=dict(arrowstyle="-", lw=0.5))
    ins.set_xlim(-0.40, 0.25)
    ins.set_ylim(-0.33, 0.33)
    ins.set_aspect("equal")
    ins.tick_params(labelsize=6)
    ins.set_title("phóng to quanh thân xe", fontsize=7)
    ax.indicate_inset_zoom(ins, edgecolor="#777777")
    ax.set_aspect("equal")
    ax.set_xlim(-1.3, 1.3)
    ax.set_ylim(-1.3, 1.3)
    ax.set_xlabel("x (m, base_link)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Lọc thân xe trên {n} vòng quét thật (xe đứng yên)", fontsize=9)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False, fontsize=7, markerscale=3)
    return luu(fig, "hieu_chinh_loc_than_xe")


def mo_hinh_camera():
    """FOV do 30/09 bang thuoc 5 vach (camera cach tuong 1.00 m): vach 3/7 cach tam 54 cm -> 28.4 do o x = 160 px,
    vach 1/9 cach tam 152.5 cm -> 56.7 do o mep anh (320 px). Mo hinh mat ca deu: f = 324 px, k1 = -0.01077.
    Mo hinh cu: pinhole FOV 62 do."""
    x = np.linspace(0, 320, 200)
    f, k1 = 324.0, -0.01077
    # giai nguoc r = f * th * (1 + k1 th^2) (mo hinh mat ca OpenCV, he so bac 1)
    th = x / f
    for _ in range(20):
        th = th - (f * th * (1 + k1 * th ** 2) - x) / (f * (1 + 3 * k1 * th ** 2))
    fp = 320 / math.tan(math.radians(31))
    fig, ax = plt.subplots(figsize=(NUA * 1.25, 2.6))
    ax.plot(x, np.degrees(th), color="#d95f02", lw=1.4, label="mắt cá đều (f = 324 px) — dùng hiện tại")
    ax.plot(x, np.degrees(np.arctan(x / fp)), color="#8c8c8c", lw=1.2, ls="--", label="pinhole FOV 62° — cấu hình cũ")
    ax.plot([0, 160, 320], [0, math.degrees(math.atan(0.54)), math.degrees(math.atan(1.525))], "o", color="#1f77b4",
            ms=6, label="đo bằng thước 5 vạch (30/09)")
    ax.annotate("cũ báo 15,5°\nthật 28,4°", (160, 15.5), xytext=(190, 6), fontsize=7,
                arrowprops=dict(arrowstyle="->", lw=0.6))
    ax.set_xlabel("khoảng cách tới tâm ảnh (px, ảnh 640×480)")
    ax.set_ylabel("góc ngang (độ)")
    ax.set_title("Mô hình góc camera KINGSEN", fontsize=9)
    ax.legend(fontsize=6.6, frameon=False, loc="upper left")
    ax.set_xlim(0, 330)
    ax.set_ylim(0, 62)
    return luu(fig, "hieu_chinh_mo_hinh_camera")


if __name__ == "__main__":
    hinh_hoc_xe()
    toc_do_driver()
    loc_than_xe()
    mo_hinh_camera()
