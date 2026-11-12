#!/usr/bin/env python3
"""Bieu do tong hop tu SO LIEU DA GHI trong CLAUDE.md (muc 8, 7.2) — khong chay lai mo phong:
  - hoi quy toan bo 3 ban tot nhat (03/11, 05/11, 07/11), 3 mam, 612 kich ban co tieu chi x 3 = 1836 luot
  - tien do qua tung dot sua (moi dot so ban truoc / ban sau tren bo hoi quy cua dot do)
  - camera / ReID (7.2-AG, 7.2-AP)
So lieu duoc ghi thang o day; tong theo nhom duoc kiem lai khop tong trong CLAUDE.md (1212 / 1523 / 1658).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kieu_hinh import *  # noqa: F401,F403,E402
from kieu_hinh import RONG, MAU_BAN, luu  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

# nhom -> (nhan, so luot toi da, [ban 03/11, ban 05/11, ban 07/11]) — CLAUDE.md muc 8 (06/11 toi) va (05/11)
NHOM = [
    ("Cửa 0,81 m: gần giữa (nhiễu 1–2 cm) + lưới 15 tư thế", 270, [270, 270, 270]),
    ("Xe đặt sẵn sát khung cửa", 36, [35, 35, 35]),
    ("Hồi quy gốc (bám thẳng, rẽ, cửa trước/bên hông…)", 30, [28, 28, 28]),
    ("Người thứ hai chen vào", 63, [54, 54, 54]),
    ("Người đứng sau vật thấp", 39, [39, 39, 39]),
    ("Kịch bản khó (chữ U, ghế dài, 3 thùng…)", 51, [44, 44, 44]),
    ("Camera ngửa, cấu hình sai", 18, [16, 16, 16]),
    ("Tìm lại người bằng RSSI", 6, [6, 6, 6]),
    ("Cửa bếp dựng từ bag thật 14/10", 57, [53, 54, 54]),
    ("Vòng qua vật, bị chặn giữa chừng", 15, [13, 15, 15]),
    ("Cửa + tủ lạnh / ghế sau tường", 30, [28, 27, 26]),
    ("Thùng xốp đặt nghiêng", 45, [44, 44, 45]),
    ("Góc kẹt", 33, [24, 24, 24]),
    ("Vật lộn xộn", 72, [70, 72, 72]),
    ("Vật lộn xộn ngẫu nhiên (khó)", 60, [38, 39, 39]),
    ("Tốc độ thích ứng (cả xe chở nặng)", 39, [33, 33, 33]),
    ("Túi hẹp (khe chỉ người lọt)", 126, [24, 121, 123]),
    ("Bẫy: túi chữ U", 288, [86, 164, 254]),
    ("Bẫy: hành lang cụt 2 m", 36, [0, 0, 4]),
    ("Bẫy: chữ L sát vách", 240, [162, 221, 227]),
    ("Bẫy: góc cụt chữ V", 54, [42, 44, 47]),
    ("Bẫy: khe 0,45–0,70 m", 132, [81, 130, 130]),
    ("Bẫy: góc tường cao", 96, [22, 43, 73]),
]
TONG = [1212, 1523, 1658]
TEN3 = ["Bản 03/11", "Bản 05/11 (túi hẹp)", "Bản 07/11 (chỗ hẹp, cuối)"]
KHOA = ["0311", "0611", "0711"]


def kiem_tong():
    tong_max = sum(n for _, n, _ in NHOM)
    assert tong_max == 1836, tong_max
    for i in range(3):
        s = sum(v[i] for _, _, v in NHOM)
        assert s == TONG[i], (i, s)


def hinh_hoi_quy_3_ban():
    kiem_tong()
    n = len(NHOM)
    fig, ax = plt.subplots(figsize=(RONG, 6.3))
    y = np.arange(n)[::-1]
    for i in range(3):
        p = [100.0 * v[i] / m for _, m, v in NHOM]
        ax.scatter(p, y, s=[16, 16, 30][i], color=MAU_BAN[KHOA[i]], zorder=3 + i, label=f"{TEN3[i]}: "
                   f"{TONG[i]}/1836 ({100.0 * TONG[i] / 1836:.1f}%)".replace(".", ","),
                   marker=["s", "D", "o"][i], edgecolor="white", linewidth=0.4)
    for yy, (_, m, v) in zip(y, NHOM):
        lo, hi = min(v) / m * 100, max(v) / m * 100
        ax.plot([lo, hi], [yy, yy], color="#bbbbbb", lw=1.0, zorder=1)
        ax.text(101.5, yy, f"{v[2]}/{m}", va="center", fontsize=6.5, color=MAU_BAN["0711"])
    ax.set_yticks(y)
    ax.set_yticklabels([t for t, _, _ in NHOM], fontsize=7)
    ax.axhline(y[16] - 0.5, color="#888888", lw=0.6, ls=":")
    ax.set_xlim(0, 112)
    ax.set_xticks(range(0, 101, 20))
    ax.set_xlabel("tỉ lệ lượt đạt (%) — 3 mầm ngẫu nhiên mỗi kịch bản")
    ax.legend(fontsize=7, frameon=False, loc="upper center", bbox_to_anchor=(0.42, -0.07), ncol=1)
    ax.set_title("Hồi quy vòng kín 612 kịch bản × 3 mầm: không lượt nào chạm vật ở cả 3 bản", fontsize=9)
    ax.grid(axis="y", lw=0)
    fig.tight_layout()
    return luu(fig, "hoi_quy_3_ban_theo_nhom")


# dot sua -> (bo hoi quy, ban truoc, ban sau, so luot) — CLAUDE.md muc 8
DOT = [
    ("07/10\nbản đồ lưới,\nvật thấp", "165 kịch bản", 131, 159, 165),
    ("14/10\ncửa bếp", "195 kịch bản", 174, 189, 195),
    ("03/11\nbộ nhớ vật cản,\ntủ lạnh, góc kẹt", "288 kịch bản\n(mầm 1)", 249, 268, 288),
    ("05/11\ntúi hẹp,\nthoát kẹt A*", "330 × 3 mầm", 819, 922, 990),
    ("07/11\nchỗ hẹp\n(6 loại bẫy)", "612 × 3 mầm", 1523, 1658, 1836),
]


def hinh_tien_do():
    fig, ax = plt.subplots(figsize=(RONG, 2.7))
    x = np.arange(len(DOT))
    for i, (ten, bo, a, b, n) in enumerate(DOT):
        pa, pb = 100.0 * a / n, 100.0 * b / n
        ax.plot([i, i], [pa, pb], color="#bbbbbb", lw=2, zorder=1)
        ax.scatter([i], [pa], color=MAU_BAN["cu"], s=30, zorder=3, marker="s")
        ax.scatter([i], [pb], color=MAU_BAN["moi"], s=40, zorder=4)
        ax.text(i + 0.09, pa, f"{a}/{n}", fontsize=6.5, va="center", color="#555555")
        ax.text(i + 0.09, pb, f"{b}/{n}", fontsize=6.5, va="center", color=MAU_BAN["moi"])
        ax.text(i, pa - 2.5, bo, fontsize=6.3, ha="center", va="top", color="#555555")
    ax.scatter([], [], color=MAU_BAN["cu"], marker="s", label="bản trước đợt sửa")
    ax.scatter([], [], color=MAU_BAN["moi"], label="bản sau đợt sửa")
    ax.set_xticks(x)
    ax.set_xticklabels([d[0] for d in DOT], fontsize=7)
    ax.set_ylim(50, 102)
    ax.set_xlim(-0.4, len(DOT) - 0.3)
    ax.set_ylabel("tỉ lệ đạt (%)")
    ax.legend(fontsize=7, frameon=False, loc="lower right")
    ax.set_title("Mỗi đợt sửa: bộ hồi quy được mở rộng thêm kịch bản khó, bản mới luôn đạt cao hơn bản trước",
                 fontsize=8.5)
    fig.tight_layout()
    return luu(fig, "hoi_quy_tien_do_cac_dot")


def hinh_reid():
    """7.2-AG (30/09: loi nap anh BGR) va 7.2-AP (09/10: OSNet x0.5 + YOLO26n onnxruntime)."""
    muc = [
        ("(a) Rank-1 Market-1501\nsửa lỗi nạp BGR→RGB", "%", 39.8, 75.5),
        ("(b) Khoá nhầm người lạ\nkhi chủ vắng", "% số khung", 8.6, 0.1),
        ("(c) Thời gian xử lý\nmỗi khung (2 lõi)", "ms", 70.6, 21.0),
        ("(d) Bám đúng chủ\nkhi đi tự do", "% số khung", 47.5, 59.4),
    ]
    fig, axs = plt.subplots(1, 4, figsize=(RONG, 2.55))
    for ax, (tit, dv, a, b) in zip(axs, muc):
        bars = ax.bar([0, 1], [a, b], color=[MAU_BAN["cu"], MAU_BAN["moi"]], width=0.62)
        for r, v in zip(bars, (a, b)):
            ax.text(r.get_x() + r.get_width() / 2, v, f"{v:g}".replace(".", ","), ha="center", va="bottom",
                    fontsize=7)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(["trước", "sau"], fontsize=7)
        ax.set_ylabel(dv, fontsize=7)
        ax.set_ylim(0, max(a, b) * 1.22)
        ax.set_title(tit, fontsize=7.5)
        ax.tick_params(axis="y", labelsize=6.5)
        ax.grid(axis="x", lw=0)
    fig.tight_layout(w_pad=1.0, rect=(0, 0.1, 1, 1))
    fig.text(0.5, 0.01, "(a) mạng ReID của DeepSORT, trước/sau sửa lỗi (30/09). (b)–(d) dữ liệu quay thật 09/10: "
             "trước = ckpt.t7 + YOLOv5n (PyTorch),\nsau = OSNet x0.5 + YOLO26n 416 (onnxruntime).",
             ha="center", va="bottom", fontsize=6.3, color="#444444")
    return luu(fig, "camera_reid_truoc_sau")


if __name__ == "__main__":
    hinh_hoi_quy_3_ban()
    hinh_tien_do()
    hinh_reid()
