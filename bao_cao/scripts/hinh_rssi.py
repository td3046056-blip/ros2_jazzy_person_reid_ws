#!/usr/bin/env python3
"""Hinh RSSI "xoay do huong" tu DU LIEU DO THAT tren xe (rssi_logs/q2_*, 01/10):
  (a) muc tin hieu 3 board theo goc nguoi nhin tu xe — lan hieu chinh q2_cal000 (nguoi truoc mui 2 m)
  (b) diem so khop mau theo huong thu — mot lan do (q2_p090_1)
  (c) sai so huong cua 11 lan do (so khop mau, ca vong xoay) — dung ham rssi_df.est_template cua node that
"""
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kieu_hinh import *  # noqa: F401,F403,E402
from kieu_hinh import WS, PKG_NAV, RONG, luu  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, PKG_NAV)
from person_follow_nav import rssi_df as DF  # noqa: E402

LOG = os.path.join(WS, "rssi_logs")
MAU_BOARD = {"A": "#1b9e77", "B": "#d95f02", "C": "#7570b3"}
TEN_BOARD = {"A": "A (đầu xe)", "B": "B (sau-phải)", "C": "C (sau-trái)"}


def doc(path):
    rows = list(csv.DictReader(open(path, newline="")))
    meta = json.load(open(os.path.splitext(path)[0] + ".json"))
    r = [x for x in rows if x["kind"] == "R"]
    o = [x for x in rows if x["kind"] == "O"]
    t_r = np.array([float(x["t_pc"]) for x in r])
    sid = np.array([x["id"] for x in r])
    rs = np.array([float(x["rssi"]) for x in r])
    t_o = np.array([float(x["t_pc"]) for x in o])
    yaw = np.unwrap(np.array([float(x["yaw"]) for x in o]))
    sel = (t_o >= meta.get("t_rot_start", t_o[0])) & (t_o <= meta.get("t_rot_end", t_o[-1]))
    t_o, yaw = t_o[sel], yaw[sel]
    deg = meta["truth_deg"] if "truth_deg" in meta else meta.get("person_deg", 0.0)
    truth = DF.wrap(yaw[0] + math.radians(float(deg)))
    return t_r, sid, rs, t_o, yaw, truth, meta


def diem_so(t, sid, rs, t_o, yaw_u, tmpl, nbins=36):
    """Dung cong thuc trong rssi_df.est_template, tra ve CA duong diem so (de ve)."""
    keep = (t >= t_o[0]) & (t <= t_o[-1])
    psi = DF.wrap(np.interp(t[keep], t_o, yaw_u))
    b = np.clip(((psi + np.pi) / (2.0 * np.pi) * nbins).astype(int), 0, nbins - 1)
    centers = -np.pi + (np.arange(nbins) + 0.5) * 2.0 * np.pi / nbins
    phis = np.radians(np.arange(-180.0, 180.0, 5.0))
    score = np.zeros(len(phis))
    for s in DF.BOARDS:
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
    return phis, score


def main():
    tmpl = DF.load_template(os.path.join(WS, "src", "person_follow_nav", "config", "rssi_template.json"))
    fig, axs = plt.subplots(1, 3, figsize=(RONG, 2.45), gridspec_kw=dict(width_ratios=[1.25, 1, 1]))

    # (a) mau anten tu lan hieu chinh
    t_r, sid, rs, t_o, yaw, truth, meta = doc(os.path.join(LOG, "q2_cal000.csv"))
    keep = (t_r >= t_o[0]) & (t_r <= t_o[-1])
    th = np.degrees(DF.wrap(truth - np.interp(t_r[keep], t_o, yaw)))
    ax = axs[0]
    for s in ("A", "B", "C"):
        m = sid[keep] == s
        ax.scatter(th[m], rs[keep][m], s=1.2, color=MAU_BOARD[s], alpha=0.18, lw=0)
        e = np.arange(-180, 181, 10)
        k = np.digitize(th[m], e) - 1
        mu = [rs[keep][m][k == i].mean() if np.any(k == i) else np.nan for i in range(len(e) - 1)]
        ax.plot(e[:-1] + 5, mu, color=MAU_BOARD[s], lw=1.6, label=TEN_BOARD[s])
    ax.set_xlim(-180, 180)
    ax.set_xticks([-180, -90, 0, 90, 180])
    ax.set_xlabel("góc người – mũi xe (độ)")
    ax.set_ylabel("RSSI (dBm)")
    ax.set_title(f"(a) Hiệu chỉnh: xoay {math.degrees(abs(yaw[-1] - yaw[0])) / 360:.0f} vòng,\nngười trước mũi 2 m",
                 fontsize=8.5)
    ax.legend(fontsize=6.3, frameon=False, loc="lower center", ncol=1)

    # (b) diem so khop mau cua mot lan do
    t_r, sid, rs, t_o, yaw, truth, meta = doc(os.path.join(LOG, "q2_p090_1.csv"))
    phis, sc = diem_so(t_r, sid, rs, t_o, yaw, tmpl)
    rel = np.degrees(DF.wrap(phis - yaw[0]))
    o = np.argsort(rel)
    ax = axs[1]
    ax.plot(rel[o], sc[o], color="#333333", lw=1.2)
    est = math.degrees(DF.wrap(phis[int(np.argmax(sc))] - yaw[0]))
    tru = math.degrees(DF.wrap(truth - yaw[0]))
    ax.axvline(tru, color=MAU_NGUOI, lw=1.2, ls="--", label=f"hướng thật {tru:+.0f}°")
    ax.axvline(est, color="#d62728", lw=1.2, label=f"ước lượng {est:+.0f}°")
    ax.set_xlim(-180, 180)
    ax.set_xticks([-180, -90, 0, 90, 180])
    ax.set_xlabel("hướng thử (độ)")
    ax.set_ylabel("điểm khớp mẫu")
    ax.set_title(f"(b) Khớp mẫu một lần đo: thật {tru:+.0f}°\n(nét đứt), ước lượng {est:+.0f}° (đỏ)", fontsize=8.5)

    # (c) sai so 11 lan do
    files = sorted(f for f in glob.glob(os.path.join(LOG, "q2_*.csv")) if "cal" not in f)
    names, errs = [], []
    for f in files:
        t_r, sid, rs, t_o, yaw, truth, meta = doc(f)
        phi = DF.est_template(t_r, sid, rs, t_o, yaw, tmpl)
        names.append(os.path.basename(f)[3:-4])
        errs.append(math.degrees(DF.wrap(phi - truth)))
    errs = np.array(errs)
    ax = axs[2]
    y = np.arange(len(errs))
    ax.axvspan(-30, 30, color="#1b9e77", alpha=0.10, lw=0)
    ax.barh(y, errs, color=["#1b9e77" if abs(e) <= 30 else "#d62728" for e in errs], height=0.6)
    ax.axvline(0, color="k", lw=0.6)
    ax.set_yticks(y)
    nhan = [("+" if n[0] == "p" else "−") + f"{int(n[1:4])}°, lần {n[-1]}" for n in names]
    ax.set_yticklabels(nhan, fontsize=6.3)
    ax.invert_yaxis()
    ax.set_xlim(-45, 45)
    ax.set_xlabel("sai số hướng (độ)")
    tb = f"{np.mean(np.abs(errs)):.1f}".replace(".", ",")
    ax.set_title(f"(c) 11 lần đo: {int(np.sum(np.abs(errs) <= 30))}/{len(errs)} trong ±30°,\nsai số TB {tb}°", fontsize=8.5)
    fig.tight_layout(w_pad=0.6)
    print("sai so tung lan:", dict(zip(names, np.round(errs, 1))))
    return luu(fig, "rssi_xoay_do_huong")


if __name__ == "__main__":
    main()
