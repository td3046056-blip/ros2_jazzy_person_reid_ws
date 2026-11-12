"""Kieu hinh dung chung cho hinh bao cao khoa luan (font tieng Viet, 300 dpi, kho A4).

Moi script ve hinh import file nay: from kieu_hinh import *
"""
import os
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Circle, Rectangle, Wedge  # noqa: E402,F401
import numpy as np  # noqa: E402

GOC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))      # .../bao_cao
WS = os.path.dirname(GOC)                                               # workspace
HINH = os.path.join(GOC, "hinh")
DU_LIEU = os.path.join(GOC, "du_lieu")
SCRIPTS_NAV = os.path.join(WS, "src", "person_follow_nav", "scripts")
PKG_NAV = os.path.join(WS, "src", "person_follow_nav")
CACHE = os.path.expanduser("~/bao_cao_cache")                           # du lieu lon (bag -> pickle), ngoai repo

# Kich thuoc: be ngang vung chu A4 le 2.5 + 2 cm ~ 16.5 cm
RONG = 6.4      # inch, hinh rong toan trang
NUA = 3.15      # inch, hinh nua trang

plt.rcParams.update({
    "font.family": "DejaVu Sans",          # co du dau tieng Viet
    "font.size": 9,
    "axes.titlesize": 10,
    "axes.labelsize": 9,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "figure.dpi": 110,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.04,
    "axes.grid": True,
    "grid.alpha": 0.25,
    "grid.linewidth": 0.5,
    "axes.spines.top": False,
    "axes.spines.right": False,
})

# Mau co dinh cho cac ban (de do mu mau van phan biet duoc)
MAU_BAN = {"0311": "#8c8c8c", "0611": "#1f77b4", "0711": "#d95f02", "cu": "#8c8c8c", "moi": "#d95f02"}
TEN_BAN = {"0311": "Bản 03/11", "0611": "Bản 05/11", "0711": "Bản 07/11 (cuối)"}
MAU_TUONG = "#333333"
MAU_VAT_THAP = "#a6761d"
MAU_NGUOI = "#2ca02c"
MAU_NGUOI2 = "#9467bd"

# Hinh hoc xe (muc 2 CLAUDE.md) — base_link o truc banh truoc
F_LEN, R_LEN, HW = 0.14, 0.33, 0.30
LIDAR_X = 0.10


def luu(fig, ten):
    """Luu PNG 300 dpi vao bao_cao/hinh/<ten>.png."""
    os.makedirs(HINH, exist_ok=True)
    duong = os.path.join(HINH, ten + ".png")
    fig.savefig(duong)
    plt.close(fig)
    print("da luu", duong)
    return duong


def footprint(x, y, yaw):
    """4 goc footprint chu nhat cua xe (khung the gioi)."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(F_LEN, HW), (F_LEN, -HW), (-R_LEN, -HW), (-R_LEN, HW)]
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in pts]


def ve_xe(ax, x, y, yaw, mau="#d95f02", alpha=0.25, lw=0.8, mui=True, z=3):
    ax.add_patch(Polygon(footprint(x, y, yaw), closed=True, fc=mau, ec=mau, alpha=alpha, lw=lw, zorder=z))
    ax.add_patch(Polygon(footprint(x, y, yaw), closed=True, fill=False, ec=mau, alpha=min(1.0, alpha * 3), lw=lw,
                         zorder=z))
    if mui:     # mui ten chi huong mui xe
        ax.annotate("", xy=(x + 0.22 * math.cos(yaw), y + 0.22 * math.sin(yaw)), xytext=(x, y),
                    arrowprops=dict(arrowstyle="-|>", color=mau, lw=0.8, mutation_scale=7), zorder=z + 1)


def ve_tuong(ax, walls, lw=1.6, mau=MAU_TUONG, nhan=None):
    first = True
    for (x1, y1, x2, y2) in walls:
        ax.plot([x1, x2], [y1, y2], color=mau, lw=lw, solid_capstyle="butt", zorder=2,
                label=nhan if (first and nhan) else None)
        first = False


def ve_hop(ax, poly, mau=MAU_VAT_THAP, alpha=0.35, nhan=None):
    ax.add_patch(Polygon(poly, closed=True, fc=mau, ec=mau, alpha=alpha, lw=0.8, zorder=2, label=nhan))


def bang_mau_trang_thai():
    return {"FOLLOW": "#1b9e77", "ARRIVED": "#66a61e", "AVOID": "#e6ab02", "SEARCH": "#7570b3",
            "OCCLUDED": "#a6761d", "BLOCKED": "#e7298a", "IDLE": "#666666", "ESTOP": "#d62728", None: "#cccccc"}
