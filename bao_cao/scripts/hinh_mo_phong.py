#!/usr/bin/env python3
"""Hinh so sanh TRUOC / SAU tu mo phong vong kin (planner + tracker THAT, sim_follow.py).

Can chay truoc: python3 mo_phong_hinh.py  (tao bao_cao/du_lieu/mo_phong/*.json)
    python3 hinh_mo_phong.py            # ve het
    python3 hinh_mo_phong.py bay_U      # vai ca
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kieu_hinh import *  # noqa: F401,F403,E402
from kieu_hinh import DU_LIEU, RONG, luu, ve_xe, ve_tuong, ve_hop, footprint, bang_mau_trang_thai  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

MP = os.path.join(DU_LIEU, "mo_phong")

TEN_PB = {
    "6ddbe62": "Bản 22/09 (trước khi sửa)",
    "bc3dcdf": "Bản 26/09 (trước khi sửa)",
    "799d643": "Bản 23/09 (trước khi sửa)",
    "6d0aa54": "Bản 07/10 (trước bản đồ lưới)",
    "2a979a8": "Bản 09/10 (trước khi sửa cửa bếp)",
    "d271164": "Bản 14/10 (trước khi sửa)",
    "0311": "Bản 03/11 (trước thoát kẹt)",
    "0611": "Bản 05/11 (trước sửa chỗ hẹp)",
    "0711": "Bản cuối 07/11",
}

# ca -> (tieu de hinh, [phien ban trai, phai], ten file)
CA = {
    "cua_giua": ("Xe đang ở miệng cửa bên hông 0,81 m, tường dày 12 cm, nhiễu LiDAR 2 cm", ["bc3dcdf", "0711"],
                 "mp_cua_giua"),
    "cua_ben": ("Người rẽ vào cửa bên hông 0,81 m (tường dày 12 cm)", ["799d643", "0711"], "mp_cua_ben_hong"),
    "vat_thap": ("Người đứng sau thùng thấp, thùng thứ hai cách 0,5 m bên trái", ["6d0aa54", "0711"],
                 "mp_vat_thap"),
    "u_trap": ("Người đứng sau vật hình chữ U", ["6d0aa54", "0711"], "mp_vat_chu_U"),
    "cua_bep": ("Cửa bếp 0,80 m dựng lại từ bag chạy thật 14/10", ["2a979a8", "0711"], "mp_cua_bep"),
    "tu_lanh": ("Qua cửa có tủ lạnh sát sau tường trái, người rẽ trái ngay", ["d271164", "0711"], "mp_tu_lanh"),
    "tui_hep": ("Túi hẹp: khe 0,4 m chỉ người lọt, bên trái trống", ["0311", "0711"], "mp_tui_hep"),
    "bay_U": ("Xe đang ở trong túi chữ U sâu 1,3 m, người đi xa", ["0611", "0711"], "mp_bay_U_sau"),
    "bay_goc": ("Mũi xe chúi vào góc tường, người đứng trong phòng", ["0611", "0711"], "mp_bay_goc_tuong"),
    "bay_khe": ("Khe 0,60 m trong vách cao (hẹp hơn xe), người lách qua rồi đi xa", ["0311", "0711"],
                "mp_bay_khe_060"),
}


def doc(ca, ver):
    p = os.path.join(MP, f"{ca}__{ver}.json")
    return json.load(open(p)) if os.path.exists(p) else None


def obox_poly(cx, cy, L, W, a):
    c, s = math.cos(a), math.sin(a)
    return [(cx + c * ex - s * ey, cy + s * ex + c * ey)
            for (ex, ey) in ((L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2))]


def chi_so(d):
    R = d["rows"]
    cam = 100.0 * np.mean([str(r["src"] or "").startswith("camera") for r in R if r["t"] > 2.0])
    return dict(d=R[-1]["d"], tt=R[-1]["st"], ho=d["hit"].get("clear", float("nan")), cam=cam,
                lui=d["mot"].get("back", 0.0), sw=d["mot"].get("sw", 0.0), ket=d["mot"].get("stuck", 0.0))


def khung(datas):
    xs, ys = [], []
    for d in datas:
        for r in d["rows"]:
            xs += [r["rx"], r["px"]]
            ys += [r["ry"], r["py"]]
            if r["ix"] is not None:
                xs.append(r["ix"])
                ys.append(r["iy"])
        for b in d["low_boxes"]:
            xs += [b[0], b[2]]
            ys += [b[1], b[3]]
        for o in d["oboxes"]:
            for (x, y) in obox_poly(*o[:5]):
                xs.append(x)
                ys.append(y)
    pad = 0.7
    return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad


def ve_the_gioi(ax, d):
    ve_tuong(ax, d["walls"])
    for b in d["low_boxes"]:
        x0, y0, x1, y1, h = b
        cao = h >= d["cam_h"]
        ve_hop(ax, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], mau="#555555" if cao else MAU_VAT_THAP,
               alpha=0.55 if cao else 0.35)
    for o in d["oboxes"]:
        cao = o[5] >= d["cam_h"]
        ve_hop(ax, obox_poly(*o[:5]), mau="#555555" if cao else MAU_VAT_THAP, alpha=0.55 if cao else 0.35)


def ve_mot(ax, d, mau, buoc=3.0, ke_hoach=True):
    ve_the_gioi(ax, d)
    R = d["rows"]
    X = np.array([r["rx"] for r in R])
    Y = np.array([r["ry"] for r in R])
    PX = np.array([r["px"] for r in R])
    PY = np.array([r["py"] for r in R])
    ax.plot(PX, PY, color=MAU_NGUOI, lw=1.0, ls="--", zorder=3)
    ax.plot(PX[-1], PY[-1], "o", ms=7, mfc=MAU_NGUOI, mec="white", zorder=7)
    if any(r["ix"] is not None for r in R):
        IX = [r["ix"] for r in R if r["ix"] is not None]
        IY = [r["iy"] for r in R if r["iy"] is not None]
        ax.plot(IX, IY, color=MAU_NGUOI2, lw=1.0, ls="--", zorder=3)
        ax.plot(IX[-1], IY[-1], "o", ms=6, mfc=MAU_NGUOI2, mec="white", zorder=7)
    t_last = -1e9
    for r in R:
        if r["t"] - t_last >= buoc:
            ve_xe(ax, r["rx"], r["ry"], math.radians(r["yaw"]), mau=mau, alpha=0.07, lw=0.4, mui=False)
            t_last = r["t"]
    ax.plot(X, Y, color=mau, lw=1.6, zorder=5)
    r0 = R[0]
    ax.add_patch(plt.Polygon(footprint(r0["rx"], r0["ry"], math.radians(r0["yaw"])), closed=True, fill=False,
                             ec="k", ls="--", lw=0.8, zorder=6))
    ve_xe(ax, R[-1]["rx"], R[-1]["ry"], math.radians(R[-1]["yaw"]), mau=mau, alpha=0.45, lw=1.0, z=6)
    if ke_hoach:
        for e in d["esc"][:4]:
            P = np.array(e["poses"])
            ax.plot(P[:, 0], P[:, 1], color="#d62728", lw=0.9, ls=":", zorder=6)
            ax.add_patch(plt.Polygon(footprint(*P[-1]), closed=True, fill=False, ec="#d62728", lw=0.7, ls=":",
                                     zorder=6))
    if d["hit"].get("clear", 1.0) <= 0.0:
        ax.plot(d["hit"]["x"], d["hit"]["y"], "x", color="red", ms=10, mew=2, zorder=8)
    m = chi_so(d)
    ho = "không gặp vật cản" if m["ho"] >= 5.0 else f"hở nhỏ nhất {100 * m['ho']:.0f} cm"
    txt = (f"cuối: cách người {m['d']:.2f} m ({m['tt']}) · {ho}\n"
           f"camera thấy người {m['cam']:.0f}% · đứng im lâu nhất {m['ket']:.0f} s")
    ax.text(0.5, 1.012, txt, transform=ax.transAxes, va="bottom", ha="center", fontsize=6.6, color="#333333")


def tieu_de_tren(fig, tieu_de):
    """Dat tieu de chung NGAY TREN chu cao nhat cua cac khung (tieu de khung + dong so lieu)."""
    fig.canvas.draw()
    r = fig.canvas.get_renderer()
    top = max(t.get_window_extent(r).y1 for ax in fig.axes for t in [ax.title] + list(ax.texts))
    y = fig.transFigure.inverted().transform((0, top))[1]
    fig.suptitle(tieu_de, fontsize=10, y=y + 0.02, va="bottom")


def hinh_so_sanh(ca, tieu_de, vers, ten):
    ds = [doc(ca, v) for v in vers]
    if any(d is None for d in ds):
        print("thieu du lieu", ca)
        return
    x0, x1, y0, y1 = khung(ds)
    w_h = (x1 - x0) / max(0.1, (y1 - y0))
    n = len(ds)
    h = min(4.6, max(2.4, RONG / n / w_h + 0.6))
    muc_tieu = 0.85 if n == 2 else 0.6          # canh cao/hep (cua bep): mo rong x cho day o, khong de khoang trong giua
    if w_h < muc_tieu:
        them = 0.5 * ((y1 - y0) * muc_tieu - (x1 - x0))
        x0, x1 = x0 - them, x1 + them
    fig, axs = plt.subplots(1, n, figsize=(RONG, h), sharex=True, sharey=True)
    axs = np.atleast_1d(axs)
    for k, (ax, d, v) in enumerate(zip(axs, ds, vers)):
        ve_mot(ax, d, MAU_BAN.get("moi" if v == "0711" else "cu"))
        ax.set_title(f"({'abcd'[k]}) {TEN_PB.get(v, v)}", fontsize=9, pad=24)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
    axs[0].set_ylabel("y (m)")
    # chu thich chung
    h_ = [plt.Line2D([], [], color="#8c8c8c", lw=1.6), plt.Line2D([], [], color="#d95f02", lw=1.6),
          plt.Line2D([], [], color=MAU_NGUOI, lw=1, ls="--", marker="o", ms=5, mec="white"),
          plt.Line2D([], [], color="k", lw=0.8, ls="--"), plt.Line2D([], [], color=MAU_TUONG, lw=1.6)]
    lb = ["quỹ đạo xe (bản trước)", "quỹ đạo xe (bản sau)", "người (đường đi, vị trí cuối)",
          "tư thế xuất phát", "tường / vách cao"]
    d0 = ds[0]
    hs = [b[4] for b in d0["low_boxes"]] + [o[5] for o in d0["oboxes"]]
    if any(h < d0["cam_h"] for h in hs):
        h_.append(plt.Rectangle((0, 0), 1, 1, fc=MAU_VAT_THAP, alpha=0.35))
        lb.append("vật thấp (camera nhìn qua)")
    if any(h >= d0["cam_h"] for h in hs):
        h_.append(plt.Rectangle((0, 0), 1, 1, fc="#555555", alpha=0.55))
        lb.append("vật cao (che camera)")
    if any(d["esc"] for d in ds):
        h_.append(plt.Line2D([], [], color="#d62728", lw=0.9, ls=":"))
        lb.append("kế hoạch thoát kẹt")
    if any(any(r["ix"] is not None for r in d["rows"]) for d in ds):
        h_.append(plt.Line2D([], [], color=MAU_NGUOI2, lw=1, ls="--"))
        lb.append("người thứ hai")
    fig.tight_layout()
    tieu_de_tren(fig, tieu_de)
    fig.legend(h_, lb, loc="upper center", bbox_to_anchor=(0.5, 0.0), ncol=4, frameon=False, fontsize=7)
    return luu(fig, ten)


def hinh_dien_bien_mp(ca, ver, ten, tieu_de):
    d = doc(ca, ver)
    R = d["rows"]
    t = np.array([r["t"] for r in R])
    cm = bang_mau_trang_thai()
    fig, axs = plt.subplots(3, 1, figsize=(RONG, 4.2), sharex=True,
                            gridspec_kw=dict(height_ratios=[2, 0.5, 1.8], hspace=0.12))
    axs[0].plot(t, [r["d"] for r in R], color=MAU_NGUOI, lw=1.1, label="khoảng cách\ntới người")
    axs[0].axhline(1.0, color="k", ls="--", lw=0.8, label="khoảng cách\nbám 1,0 m")
    axs[0].set_ylabel("khoảng cách (m)")
    axs[0].legend(loc="center left", bbox_to_anchor=(1.005, 0.5), frameon=False, fontsize=6.5)
    # cat bot doan cuoi dung yen: toi lan doi trang thai cuoi + 10 s
    t_doi = [r1["t"] for r0, r1 in zip(R[:-1], R[1:]) if r1["st"] != r0["st"]]
    t_het = min(t[-1], (t_doi[-1] if t_doi else t[-1]) + 10.0)
    for r0, r1 in zip(R[:-1], R[1:]):
        axs[1].axvspan(r0["t"], r1["t"], color=cm.get(r0["st"], "#999"), lw=0)
    axs[1].set_yticks([])
    axs[1].grid(False)
    axs[1].set_ylabel("trạng thái", rotation=0, ha="right", va="center")
    for s in ("FOLLOW", "AVOID", "ARRIVED", "SEARCH", "BLOCKED", "IDLE"):
        if any(r["st"] == s for r in R):
            axs[1].plot([], [], color=cm[s], lw=5, label=s)
    axs[1].legend(loc="center left", bbox_to_anchor=(1.005, 0.5), frameon=False, fontsize=6.5)
    axs[2].plot(t, [r["v"] for r in R], color="#1f77b4", lw=1.0, label="v (m/s)")
    axs[2].plot(t, [r["w"] for r in R], color="#d95f02", lw=0.8, label="ω (rad/s)")
    axs[2].axhline(0, color="k", lw=0.5)
    axs[2].set_ylabel("lệnh")
    axs[2].legend(loc="center left", bbox_to_anchor=(1.005, 0.5), frameon=False, fontsize=6.5)
    for e in d["esc"]:
        for a in axs:
            a.axvline(e["t"], color="#d62728", ls=":", lw=0.9)
    axs[2].set_xlabel("thời gian mô phỏng (s)")
    axs[2].set_xlim(-1.0, t_het)
    if d["esc"]:
        axs[0].text(d["esc"][0]["t"] + 0.6, axs[0].get_ylim()[1] * 0.93, "thoát kẹt A*", color="#d62728",
                    fontsize=7, va="top")
    axs[0].set_title(tieu_de)
    return luu(fig, ten)


def hinh_chen():
    """Nguoi thu hai buoc vao chen giua: vi tri tracker uoc luong so voi vi tri that."""
    d = doc("chen", "0711")
    if d is None:
        return
    R = d["rows"]
    fig, axs = plt.subplots(1, 2, figsize=(RONG, 2.9), gridspec_kw=dict(width_ratios=[1.25, 1]))
    ax = axs[0]
    ve_mot(ax, d, MAU_BAN["moi"], ke_hoach=False)
    E = np.array([[r["ex"], r["ey"]] for r in R if r["ex"] is not None])
    ax.scatter(E[::4, 0], E[::4, 1], s=4, c="#1f77b4", zorder=4, label="vị trí người tracker ước lượng")
    x0, x1, y0, y1 = khung([d])
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("(a) Quỹ đạo (bản cuối)", fontsize=9, pad=24)
    ax.plot([], [], color=MAU_NGUOI2, ls="--", marker="o", ms=4, label="người thứ hai")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), fontsize=6.5, frameon=False, ncol=2)
    ax2 = axs[1]
    t = [r["t"] for r in R if r["ex"] is not None]
    err = [math.hypot(r["ex"] - r["px"], r["ey"] - r["py"]) for r in R if r["ex"] is not None]
    err2 = [math.hypot(r["ex"] - r["ix"], r["ey"] - r["iy"]) if r["ix"] is not None else np.nan
            for r in R if r["ex"] is not None]
    ax2.plot(t, err, color="#1f77b4", lw=1.0, label="sai số so với người đang bám")
    ax2.plot(t, err2, color=MAU_NGUOI2, lw=1.0, label="khoảng cách ước lượng – người thứ hai")
    ax2.set_xlabel("thời gian (s)")
    ax2.set_ylabel("m")
    ax2.set_title("(b) Ước lượng vị trí người của tracker", fontsize=9)
    ax2.legend(fontsize=6.5, frameon=False)
    fig.tight_layout()
    return luu(fig, "mp_nguoi_thu_hai_chen")


if __name__ == "__main__":
    chon = sys.argv[1:]
    for ca, (td, vers, ten) in CA.items():
        if not chon or ca in chon:
            hinh_so_sanh(ca, td, vers, ten)
    if not chon or "dien_bien" in chon:
        hinh_dien_bien_mp("bay_U", "0711", "mp_dien_bien_bay_U", "Diễn biến khi xe tự thoát khỏi túi chữ U sâu (mô phỏng, bản cuối)")
    if not chon or "chen" in chon:
        hinh_chen()
