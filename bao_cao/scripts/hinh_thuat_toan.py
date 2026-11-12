#!/usr/bin/env python3
"""Hinh minh hoa thuat toan, goi THANG code planner that (follow_planner_node):
  - bo nho goc tuong doi (code cu) vs bo nho khung odom (code moi)  — giong test_memory.py
  - quat quy dao DWA voi footprint chu nhat (lenh chon lay tu _dwa that)
  - ban do luoi + duong Dijkstra (_nav_plan that), truong chi phi tinh theo dung cong thuc trong _nav_plan
  - ke hoach thoat ket A* ghi duoc trong mo phong tui chu U (bao_cao/du_lieu/mo_phong/bay_U__0711.json)
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kieu_hinh import *  # noqa: F401,F403,E402
from kieu_hinh import SCRIPTS_NAV, DU_LIEU, RONG, luu, ve_xe, footprint  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

for k in [k for k in os.environ if k in ("DOOR", "SIDE_DOOR", "TALLWALL", "LOWBOX", "OBOX", "CORNER", "PKG_DIR")]:
    del os.environ[k]
sys.path.insert(0, SCRIPTS_NAV)
import sim_follow as S  # noqa: E402
from person_follow_nav import geometry as G  # noqa: E402


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def hinh_bo_nho():
    """Kich ban test_memory.py: nguoi dung yen cach 2 m, camera mat hinh, xe ne sang trai quay 35 do trong 1.2 s."""
    d, b0 = 2.0, 0.0
    px, py = d, 0.0
    w = math.radians(35) / 1.2
    rx = ry = yaw = 0.0
    v = 0.18
    T, true_b, old_b, poses = [], [], [], []
    for k in range(13):                     # dung nhu test_memory.py: 0 .. 1.2 s
        t = k * 0.1
        tb = wrap(math.atan2(py - ry, px - rx) - yaw)
        T.append(t)
        true_b.append(math.degrees(tb))
        old_b.append(math.degrees(b0))
        poses.append((rx, ry, yaw))
        rx += v * math.cos(yaw) * 0.1
        ry += v * math.sin(yaw) * 0.1
        yaw = wrap(yaw + w * 0.1)
    fig, axs = plt.subplots(1, 2, figsize=(RONG, 2.7), gridspec_kw=dict(width_ratios=[1.15, 1]))
    ax = axs[0]
    ax.plot(px, py, "o", ms=8, mfc=MAU_NGUOI, mec="white", zorder=5)
    ax.text(px + 0.05, py + 0.12, "người (đứng yên)", fontsize=7)
    for k in (0, 6, 12):
        x, y, th = poses[k]
        ve_xe(ax, x, y, th, mau="#d95f02", alpha=0.2, lw=0.6)
        # code cu: nho goc tuong doi 0 do -> tuong nguoi o thang mui xe
        ax.plot([x, x + 2.0 * math.cos(th)], [y, y + 2.0 * math.sin(th)], color="#8c8c8c", ls="--", lw=0.9)
        ax.plot([x, px], [y, py], color="#1b9e77", lw=0.9)
    ax.plot([p[0] for p in poses], [p[1] for p in poses], color="#d95f02", lw=1.2)
    ax.plot([], [], color="#8c8c8c", ls="--", label="code cũ: nhớ góc tương đối (0°)")
    ax.plot([], [], color="#1b9e77", label="code mới: nhớ vị trí trong khung odom")
    ax.set_aspect("equal")
    ax.set_xlim(-0.4, 2.3)
    ax.set_ylim(-0.4, 1.1)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(fontsize=6.8, frameon=False, loc="upper left")
    ax.set_title("(a) Xe né sang trái khi camera mất người", fontsize=9)
    ax = axs[1]
    err_old = [abs(wrap(math.radians(o - t))) * 180 / math.pi for o, t in zip(old_b, true_b)]
    ax.plot(T, err_old, color="#8c8c8c", lw=1.4, label="cũ")
    ax.plot(T, [0.0] * len(T), color="#1b9e77", lw=1.4, label="mới (odom)")
    ax.axhline(31, color="#d62728", ls=":", lw=0.8)
    ax.text(0.02, 27.8, "nửa FOV cấu hình cũ (31°)", fontsize=6.5, color="#d62728")
    ax.set_xlabel("thời gian (s)")
    ax.set_ylabel("sai số góc tới người (độ)")
    ax.set_title(f"(b) Sai số lớn nhất: cũ {max(err_old):.1f}°, mới 0°", fontsize=9)
    ax.legend(fontsize=6.8, frameon=False, loc="lower right", bbox_to_anchor=(1.0, 0.06))
    fig.tight_layout()
    return luu(fig, "thuat_toan_bo_nho_odom")


def canh_hop(boxes, step=0.03):
    """Diem tren canh cac hop (x0, y0, x1, y1) — vat can nhu LiDAR thay (khung base_link)."""
    P = []
    for (x0, y0, x1, y1) in boxes:
        for (a, b, c, d) in ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)):
            n = max(2, int(math.hypot(c - a, d - b) / step))
            u = np.linspace(0, 1, n)
            P.append(np.stack([a + (c - a) * u, b + (d - b) * u], 1))
    return np.concatenate(P)


def tao_planner(P, v0=0.15, w0=0.0):
    tr, pl = S.make_nodes()
    pl.obstacles = P.astype(np.float64)
    pl.obstacles_full = P.astype(np.float64)
    pl.obstacles_dense = P.astype(np.float64)
    pl.robot_x = pl.robot_y = pl.robot_yaw = 0.0
    pl.cur_v, pl.cur_w = v0, w0
    return pl


def hinh_dwa():
    """Hai tang cua planner THAT: _choose_heading (chon khe, nhin xa 1.6 m) cho dich phu, _dwa chon quy dao.
    Thung chan truoc mui, mot chan ghe sau-phai: xoay gap sang trai thi duoi xe (0.33 m) van vao chan ghe.
    Quat quy dao ve lai dung cong thuc trong _dwa, loc bang dung _admissible."""
    boxes = [(0.55, -0.35, 0.85, 0.15), (-0.40, -0.46, -0.34, -0.40)]
    P = canh_hop(boxes, step=0.02)
    goal = (1.30, 0.05)
    person = (2.30, 0.05)
    pl = tao_planner(P, v0=0.12, w0=0.0)
    goal_b, goal_r = math.atan2(goal[1], goal[0]), math.hypot(*goal)
    dist = math.hypot(*person)
    chk_r = max(goal_r, dist - pl.target_clear_r)
    phi, reach, _ = pl._choose_heading(goal_b, chk_r, math.atan2(person[1], person[0]))
    sub_r = min(reach, goal_r)
    sub = (sub_r * math.cos(phi), sub_r * math.sin(phi))
    res = pl._dwa(sub, person, 1.0 / 15.0, fov_active=True)
    dv = pl.accel_lin * pl.sample_window
    dw = pl.accel_ang * pl.sample_window
    v_lo, v_hi = max(0.0, pl.cur_v - pl.accel_lin * pl.sample_window), min(pl.v_max, pl.cur_v + dv)
    w_lo, w_hi = max(-pl.w_max, pl.cur_w - dw), min(pl.w_max, pl.cur_w + dw)
    vs = np.unique(np.concatenate([np.linspace(v_lo, v_hi, pl.n_v), [0.0]]))
    ws = np.linspace(w_lo, w_hi, pl.n_w)
    t = pl.t_arr

    def quy_dao(V, W):
        th = W * t
        if abs(W) < 1e-4:
            return V * t, 0 * t, th
        return V / W * np.sin(th), V / W * (1 - np.cos(th)), th

    fig, ax = plt.subplots(figsize=(RONG * 0.62, RONG * 0.52))
    for (x0, y0, x1, y1) in boxes:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=MAU_VAT_THAP, alpha=0.35, ec=MAU_VAT_THAP))
    ax.scatter(P[:, 0], P[:, 1], s=1.5, c="#333333", zorder=3)
    n_ok = n_bad = 0
    for V in vs:
        for W in ws:
            X, Y, th = quy_dao(V, W)
            dx = P[None, :, 0] - X[:, None]
            dy = P[None, :, 1] - Y[:, None]
            c, s_ = np.cos(th)[:, None], np.sin(th)[:, None]
            clr_t = G.rect_clearance(c * dx + s_ * dy, -s_ * dx + c * dy, pl.front_len, pl.rear_len,
                                     pl.half_width).min(axis=1)
            ok = bool(pl._admissible(np.array([clr_t.min()]), np.array([clr_t[-1]]))[0])
            n_ok += ok
            n_bad += not ok
            ax.plot(np.r_[0, X], np.r_[0, Y], color="#1b9e77" if ok else "#d62728", lw=0.6,
                    alpha=0.7 if ok else 0.45, zorder=4)
            ax.plot(X[-1], Y[-1], ".", ms=1.5, color="#1b9e77" if ok else "#d62728", zorder=4)
    ve_xe(ax, 0, 0, 0, mau="#1f77b4", alpha=0.06, lw=0.9)
    if res is not None:
        X, Y, th = quy_dao(res[0], res[1])
        for k in (5, 11):
            ax.add_patch(Polygon(footprint(X[k], Y[k], th[k]), closed=True, fill=False, ec="#d95f02", lw=0.8,
                                 ls=(0, (3, 2)), zorder=5))
        ax.plot(np.r_[0, X], np.r_[0, Y], color="#d95f02", lw=2.4, zorder=5)
    la = 0.75
    ax.plot([0, la * math.cos(phi)], [0, la * math.sin(phi)], color="#7570b3", ls="--", lw=1.2, zorder=6,
            label=f"tầng chọn khe: hướng ngắm {math.degrees(phi):+.0f}°")
    ax.text(0.90, 0.20, "→ đích, người\n(thẳng trước,\n1,3 / 2,3 m)", fontsize=6.5, color="#333333")
    ax.text(0.62, -0.45, "thùng", fontsize=7, color="#7a5517")
    ax.text(-0.50, -0.60, "chân ghế", fontsize=7, color="#7a5517")
    ax.plot([], [], color="#1b9e77", label=f"quỹ đạo hợp lệ ({n_ok})")
    ax.plot([], [], color="#d62728", label=f"bị loại: footprint hở ≤ 6 cm ({n_bad})")
    if res is not None:
        ax.plot([], [], color="#d95f02", lw=2.4,
                label=f"được chọn: v = {res[0]:.2f} m/s, ω = {res[1]:+.2f} rad/s")
    ax.set_aspect("equal")
    ax.set_xlim(-0.6, 1.25)
    ax.set_ylim(-0.7, 0.95)
    ax.set_xlabel("x (m, base_link)")
    ax.set_ylabel("y (m)")
    ax.legend(fontsize=6.5, frameon=True, framealpha=0.92, edgecolor="none", loc="upper left")
    ax.set_title("Chọn khe + DWA: quỹ đạo thử 1,2 s, kiểm tra footprint chữ nhật", fontsize=9)
    fig.tight_layout()
    return luu(fig, "thuat_toan_dwa_quat_quy_dao")


def hinh_ban_do_luoi():
    """Nguoi sau thung thap, thung thu hai cach 0.5 m ben trai (canh 07/10). Duong do _nav_plan that tinh."""
    boxes = [(1.30, -0.25, 1.60, 0.25), (1.30, 0.75, 1.60, 1.05)]
    P = canh_hop(boxes)
    person = (2.4, 0.0)
    pl = tao_planner(P, v0=0.0)
    plan = pl._nav_plan(person[0], person[1], False)
    g = plan["grid"]
    res, x0, ey, nxc, nyc = g["res"], g["x0"], g["ey"], g["nxc"], g["nyc"]
    xs = x0 + (np.arange(nxc) + 0.5) * res
    ys = -ey + (np.arange(nyc) + 0.5) * res
    GX, GY = np.meshgrid(xs, ys)
    Q = P[np.hypot(P[:, 0] - person[0], P[:, 1] - person[1]) > pl.nav_person_clear]
    dobs = np.sqrt(((GX[..., None] - Q[:, 0]) ** 2 + (GY[..., None] - Q[:, 1]) ** 2).min(axis=2))
    r_in = pl.half_width + pl.margin_hard
    cost = 1.0 + pl.nav_soft_cost * np.clip((r_in + pl.nav_soft - dobs) / pl.nav_soft, 0.0, 1.5) ** 2
    cost = np.where(dobs < pl.nav_lethal, np.nan, cost)
    fig, ax = plt.subplots(figsize=(RONG * 0.72, RONG * 0.52))
    im = ax.imshow(cost, origin="lower", extent=(xs[0] - res / 2, xs[-1] + res / 2, ys[0] - res / 2, ys[-1] + res / 2),
                   cmap="YlOrBr", vmin=1, vmax=1 + pl.nav_soft_cost * 2.25, alpha=0.85)
    lm = np.where(dobs < pl.nav_lethal, 1.0, np.nan)
    ax.imshow(lm, origin="lower", extent=(xs[0] - res / 2, xs[-1] + res / 2, ys[0] - res / 2, ys[-1] + res / 2),
              cmap="Greys", vmin=0, vmax=1.4, alpha=0.75)
    cb = fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02)
    cb.set_label("chi phí mỗi ô", fontsize=7)
    cb.ax.tick_params(labelsize=6)
    for (bx0, by0, bx1, by1) in boxes:
        ax.add_patch(Rectangle((bx0, by0), bx1 - bx0, by1 - by0, fc=MAU_VAT_THAP, alpha=0.9, ec="k", lw=0.6))
    path = np.array(plan["path"])
    ax.plot(path[:, 0], path[:, 1], color="#1f77b4", lw=2.0, label="đường Dijkstra (_nav_plan)")
    ax.plot(*plan["target"], "s", ms=6, mfc="#1f77b4", mec="white", label="chỗ đứng tốt nhất")
    ve_xe(ax, 0, 0, 0, mau="#d95f02", alpha=0.35, lw=0.8)
    ax.plot(*person, "o", ms=8, mfc=MAU_NGUOI, mec="white", zorder=6)
    ax.text(person[0] + 0.08, person[1] + 0.08, "người", fontsize=7)
    ax.add_patch(Rectangle((0, 0), 0, 0, fc="#888888", alpha=0.75, label="ô cấm (tâm xe cách vật < 0,30 m)"))
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, 3.0)
    ax.set_ylim(-1.2, 1.6)
    ax.set_xlabel("x (m, base_link)")
    ax.set_ylabel("y (m)")
    ax.legend(fontsize=6.8, frameon=True, framealpha=0.9, edgecolor="none", loc="upper left")
    ax.set_title(f"Bản đồ lưới {100 * res:.0f} cm: khe 0,5 m giữa hai thùng bị cấm (xe rộng 0,6 m),\n"
                 "Dijkstra chọn đường vòng bên trống", fontsize=9)
    return luu(fig, "thuat_toan_ban_do_luoi_dijkstra")


def hinh_thoat_ket():
    """Ke hoach thoat ket A* (x, y, huong) THAT ghi trong mo phong tui chu U sau 1.3 m (ban cuoi), ve tung buoc."""
    d = json.load(open(os.path.join(DU_LIEU, "mo_phong", "bay_U__0711.json")))
    e = d["esc"][0]
    ten = {"R": "xoay tại chỗ", "F": "tiến", "B": "lùi"}
    truoc = [n for n in d["nav"] if n["t"] <= e["t"]]
    pa = np.array(truoc[-1]["path"]) if truoc else None
    T_plan = sum(p[3] for p in e["plan"])
    R = [r for r in d["rows"] if e["t"] - 0.05 <= r["t"] <= e["t"] + T_plan + 0.5]
    n = len(e["plan"])
    fig, axs = plt.subplots(2, (n + 1) // 2, figsize=(RONG, RONG * 0.62), sharex=True, sharey=True)
    axs = axs.ravel()

    def nen(ax):
        for w in d["walls"]:
            ax.plot([w[0], w[2]], [w[1], w[3]], color=MAU_TUONG, lw=2.0)
        for o in d["oboxes"]:
            cx, cy, L, W, a, h = o
            c, s_ = math.cos(a), math.sin(a)
            poly = [(cx + c * ex - s_ * ey, cy + s_ * ex + c * ey)
                    for (ex, ey) in ((L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2))]
            ax.add_patch(Polygon(poly, fc=MAU_VAT_THAP, alpha=0.45, ec=MAU_VAT_THAP))

    def xe(ax, p, mau, ls, lw):
        ax.add_patch(Polygon(footprint(*p), closed=True, fill=False, ec=mau, lw=lw, ls=ls, zorder=5))
        ax.annotate("", xy=(p[0] + 0.22 * math.cos(p[2]), p[1] + 0.22 * math.sin(p[2])), xytext=(p[0], p[1]),
                    arrowprops=dict(arrowstyle="-|>", lw=lw, color=mau, mutation_scale=7), zorder=6)

    for k, ax in enumerate(axs[:n]):
        kind, v, w, T = e["plan"][k]
        if kind == "R":
            mo = f"{ten[kind]} {abs(math.degrees(w * T)):.0f}°"
        else:
            mo = f"{ten[kind]} {abs(100 * v * T):.0f} cm" + (" (thẳng)" if abs(w) < 1e-3 else " (theo cung)")
        nen(ax)
        if k == 0 and pa is not None:
            ax.plot(pa[:, 0], pa[:, 1], color="#1b9e77", lw=1.2, ls="--")
        xe(ax, e["poses"][0], "#1f77b4", "-", 0.7) if k == 0 else xe(ax, e["poses"][k], "#999999", "-", 0.7)
        xe(ax, e["poses"][k + 1], "#d62728", "--", 1.1)
        if k == n - 1:
            ax.plot([r["rx"] for r in R], [r["ry"] for r in R], color="k", lw=1.0, zorder=7)
        ax.set_title(f"Bước {k + 1}: {mo}", fontsize=8.5)
        ax.set_aspect("equal")
    for ax in axs[n:]:
        ax.axis("off")
    axs[0].set_xlim(-0.55, 1.45)
    axs[0].set_ylim(-0.62, 0.62)
    for ax in axs:
        ax.tick_params(labelsize=7)
    for ax in axs[(len(axs) // 2):]:
        ax.set_xlabel("x (m)", fontsize=8)
    for ax in axs[::max(1, (n + 1) // 2)]:
        ax.set_ylabel("y (m)", fontsize=8)
    h = [plt.Line2D([], [], color="#1f77b4", lw=0.9, label="tư thế lúc lập kế hoạch"),
         plt.Line2D([], [], color="#999999", lw=0.9, label="tư thế trước bước"),
         plt.Line2D([], [], color="#d62728", lw=1.1, ls="--", label="tư thế sau bước (kế hoạch)"),
         plt.Line2D([], [], color="#1b9e77", lw=1.2, ls="--", label="đường bản đồ (đi ra phía sau xe)"),
         plt.Line2D([], [], color="k", lw=1.0, label=f"tâm xe thật khi chạy cả kế hoạch ({T_plan:.0f} s)")]
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    fig.legend(handles=h, loc="lower center", ncol=3, fontsize=6.8, frameon=False)
    return luu(fig, "thuat_toan_ke_hoach_thoat_ket")


if __name__ == "__main__":
    hinh_bo_nho()
    hinh_dwa()
    hinh_ban_do_luoi()
    hinh_thoat_ket()
