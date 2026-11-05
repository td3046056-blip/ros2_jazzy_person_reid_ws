#!/usr/bin/env python3
"""
regress_follow.py — bo HOI QUY vong kin bam nguoi (07/10): chay sim_follow.py tren ~190 kich ban song song,
in bang DAT/LOI theo nhom; co the so voi mot ban cu lay tu git (--base). Khong can ROS, chi numpy + scipy.

Nhom kich ban:
  regress    17 kich ban cu (thang, re gat, ne ngang, cheo, nguoi thu hai, goc tuong, cua truoc, cua ben hong...)
  door_near  xe da o gan giua cua 0.81 m tuong day 12 cm, 30 tu the x nhieu lidar 1 cm va 2 cm
  door_grid  chui cua ben hong tu xa, 15 tu the x tuong mong / day
  stuck      xe da lo sat khung cua 3-4.5 cm (12 tu the)
  chen       nguoi thu hai buoc vao chen (24 ca)
  turn       xoay theo nguoi di nhanh
  lowbox     nguoi dung sau thung thap 50x30 cm (canh nguoi dung bao 07/10), co / khong thung thu hai
  hard       chu U, ghe dai, thung lon xon, nguoi di vong thung, camera ngua sai goc, nguoi cao / thap, nap sau tu
  pitch      camera ngua / cui sai 12-18 do so voi camera_pitch_deg (xe that 08/10: chi xoay, khong di toi)
  rssi       nguoi khuat sau vach co cua, co / khong RSSI
  vong_chan  xe dang vong qua vat can chinh thi bi chan giua chung (nguoi thu hai / vat khuat) -> vong ben kia
  cua_bep    cua bep THAT 0.80 m tu bag 14/10 (canh cua mo ap ben phai, cho mo gia 0.95 m phia trong), co / khong
             tuong hanh lang trai; nguoi re vao cua, di thang vao cua, dung yen trong bep (5 tu the xuat phat)
  tui_hep    (05/11) vat chinh + ke de ngang + ghe noi tiep: tui rong 0.85-1.1 m khep lai con khe 0.4-0.5 m chi nguoi
             lot; nguoi lach qua khe, xe phai vong ben trai vat chinh (da vao tui thi tu quay dau ra, khong cham)

CACH DUNG (trong src/person_follow_nav/scripts):
  python3 regress_follow.py                        # tat ca nhom, ban hien tai (~10 phut tren 32 nhan)
  python3 regress_follow.py --base 6d0aa54         # so voi ban cu (git archive vao thu muc tam)
  python3 regress_follow.py --groups lowbox,hard   # chi vai nhom
  python3 regress_follow.py --jobs 8 --json kq.json

DAT nghia la: cua / cua_bep -> qua cua va khong cham tuong; vat thap / kho -> ket thuc ARRIVED/FOLLOW cach nguoi 0.8-1.4 m va
khong cham; chen -> uoc luong khong gan nguoi thu hai hon > 10 % thoi gian va ho voi chan ho luc ho dung yen
>= 5 cm (3 ca "di:2.2:8.0:*" nguoi thu hai buoc thang vao cho xe dang dung — ban nao cung cham, khong tinh).
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
JOBS = []


def J(group, label, scen, **env):
    JOBS.append(dict(group=group, label=label, scen=scen, env={k: str(v) for k, v in env.items()}))


# ── Kich ban ─────────────────────────────────────────────────────────────
for sc in ("thang", "re_trai_gat", "re_phai_gat", "ne_ngang_nhanh", "cheo_thoat", "chan_giua", "cat_ngang",
           "chan_sat", "sau_ne_re"):
    J("regress", sc, sc, CAM_HALF=25)
J("regress", "goc_tuong", "goc_tuong", CAM_HALF=25, CORNER=1)
J("regress", "thoat_khung", "thoat_khung", CAM_HALF=25, HIDE_LEGS_DEG=25)
J("regress", "cat_ngang_gan", "cat_ngang_gan", CAM_HALF=25, CAM_OCCLUDE=1)
for sc in ("qua_cua", "qua_cua_lech", "qua_cua_cheo"):
    J("regress", sc, sc, CAM_HALF=25, DOOR="3.2:0.81")
J("regress", "qua_cua_cheo+HL", "qua_cua_cheo", CAM_HALF=25, DOOR="3.2:0.81", CORRIDOR=1)
for sc in ("cua_ben", "cua_ben_cham", "cua_ben_dung"):
    J("regress", sc, sc, CAM_HALF=25, SIDE_DOOR="3.0:0.81:-0.9")
    J("regress", sc + "+day", sc, CAM_HALF=55, SIDE_DOOR="3.0:0.81:-0.9", WALL_T=0.12)
J("regress", "chan_giua55", "chan_giua", CAM_HALF=55)
J("regress", "sau_ne_re55", "sau_ne_re", CAM_HALF=55)

for nz in (0.01, 0.02):
    for x in (3.365, 3.385, 3.405, 3.425, 3.445):
        for y in (-0.62, -0.80):
            for yaw in (-95, -90, -85):
                J(f"door_near_{nz}", f"{x}:{y}:{yaw}", "cua_ben_yen", CAM_HALF=31, LIDAR_NOISE=nz, WALL_T=0.12,
                  SIDE_DOOR="3.0:0.81:-0.9", START=f"{x}:{y}:{yaw}", START_TRAIL=0.8)

for wt in (0, 0.12):
    for x in (2.8, 3.1, 3.4, 3.7, 4.0):
        for yaw in (-105, -90, -75):
            J(f"door_grid_wt{wt}", f"{x}:0.0:{yaw}", "cua_ben_yen", CAM_HALF=31, WALL_T=wt,
              SIDE_DOOR="3.0:0.81:-0.9", START=f"{x}:0.0:{yaw}", START_TRAIL=0.8)

for st in ("3.300:-0.790:-105.0", "3.480:-0.840:-105.0", "3.290:-0.770:-100.0", "3.480:-0.840:-100.0",
           "3.300:-0.750:-95.0", "3.480:-0.840:-95.0", "3.340:-0.840:-85.0", "3.460:-0.790:-85.0",
           "3.330:-0.840:-80.0", "3.450:-0.800:-80.0", "3.340:-0.840:-75.0", "3.460:-0.810:-75.0"):
    J("stuck", st, "cua_ben_yen", CAM_HALF=31, SIDE_DOOR="3.0:0.81:-0.9", START=st, START_TRAIL=0.8)

for tgt in ("dung", "di"):
    for x in (2.2, 2.45):
        for tin in (5.0, 8.0):
            for ys in (-0.15, 0.0, 0.15):
                J("chen", f"{tgt}:{x}:{tin}:{ys}", "chen", CAM_HALF=25, CAM_OCCLUDE=1, TGT=tgt,
                  CHEN=f"{x}:{tin}:6.0:{ys}")

for sc in ("vong_trai", "vong_phai", "vong_nhanh_trai", "vong_nhanh_phai", "vong_nguoi_trai", "vong_nguoi_phai"):
    J("turn", sc, sc, CAM_HALF=25)

LB_L = "1.30:-0.25:1.60:0.25;1.30:0.75:1.60:1.05"      # thung 2 ben TRAI, khe 0.5 m (xe khong lot)
LB_L8 = "1.30:-0.25:1.60:0.25;1.30:1.05:1.60:1.35"     # thung 2 ben TRAI, khe 0.8 m (sat nut)
LB_R = "1.30:-0.25:1.60:0.25;1.30:-1.05:1.60:-0.75"    # thung 2 ben PHAI, khe 0.5 m
for lb, nm in ((LB_L, "L5"), (LB_L8, "L8"), (LB_R, "R5")):
    for px in (2.4, 3.2):
        for st in ("0:0:0", "0:0.35:5" if nm != "R5" else "0:-0.35:-5"):
            J("lowbox", f"{nm}:px{px}:{st}", "sau_vat_thap", CAM_HALF=55, LOWBOX=lb, PX=px, START=st)
J("lowbox", "don:px2.6:py0.3", "sau_vat_thap", CAM_HALF=55, LOWBOX="1.30:-0.25:1.60:0.25", PX=2.6, PY=0.3)

U = "1.6:-0.6:1.8:0.6;1.0:-0.6:1.8:-0.4;1.0:0.4:1.8:0.6"
BENCH = "1.3:-0.7:1.6:0.7"
CL = "1.0:0.3:1.3:0.6;1.6:-0.5:1.9:-0.2;2.2:0.1:2.5:0.4"
WP_HOC = "1.7:-0.9;1.7:-0.9;2.4:-0.9;2.4:0.0"
for lab, sc, kw in (
        ("U_trap:px2.6", "sau_vat_thap", dict(LOWBOX=U, PX=2.6)),
        ("U_trap:px3.2", "sau_vat_thap", dict(LOWBOX=U, PX=3.2)),
        ("ghe_dai:px2.2", "sau_vat_thap", dict(LOWBOX=BENCH, PX=2.2)),
        ("ghe_dai:px2.6", "sau_vat_thap", dict(LOWBOX=BENCH, PX=2.6)),
        ("ghe_dai:px3.0", "sau_vat_thap", dict(LOWBOX=BENCH, PX=3.0)),
        ("lon_xon:px3.4", "sau_vat_thap", dict(LOWBOX=CL, PX=3.4)),
        ("lon_xon:px3.4:py-0.5", "sau_vat_thap", dict(LOWBOX=CL, PX=3.4, PY=-0.5)),
        ("L5:khe_thang", "sau_vat_thap", dict(LOWBOX=LB_L, PX=2.4, START="0:0.5:0")),
        ("L8:khe_thang", "sau_vat_thap", dict(LOWBOX=LB_L8, PX=3.2, START="0:0.65:0")),
        ("di_qua_thung", "duong_di", dict(LOWBOX=LB_L, WP="2.4:0;4.0:-1.0", WPV=0.4, WPT=30)),
        ("di_vong_thung", "duong_di", dict(LOWBOX=LB_L, WP="1.0:-0.8;2.2:-0.8;2.4:0;3.5:0.3", WPV=0.35, WPT=35)),
        ("ngua12_sai3", "sau_vat_thap", dict(LOWBOX=LB_L, PX=2.4, CAM_PITCH=12, CAM_PITCH_ERR=3)),
        ("ngua12_sai-3", "sau_vat_thap", dict(LOWBOX=LB_L, PX=2.4, CAM_PITCH=12, CAM_PITCH_ERR=-3)),
        ("nguoi_cao1.9", "sau_vat_thap", dict(LOWBOX=LB_L, PX=2.4, PERSON_H=1.9)),
        ("nguoi_thap1.5", "sau_vat_thap", dict(LOWBOX=LB_L, PX=2.4, PERSON_H=1.5)),
        # dung cho thoang vai giay (tracker hoc chieu cao / sai lech goc ngua) roi moi ra sau thung
        ("nguoi_cao1.9_hoc", "duong_di", dict(LOWBOX=LB_L, PERSON_H=1.9, WP=WP_HOC, WPV=0.25, WPT=35)),
        ("ngua12_sai3_hoc", "duong_di", dict(LOWBOX=LB_L, CAM_PITCH=12, CAM_PITCH_ERR=3, WP=WP_HOC, WPV=0.25, WPT=35))):
    J("hard", lab, sc, CAM_HALF=55, **kw)

# Camera NGUA that 18 do ma camera_pitch_deg = 0 (xe that 08/10: tracker bao nguoi cach 0.5-0.8 m trong khi that
# 2-3 m -> xe chi xoay tai cho, khong di toi). Tracker phai tu do sai lech bang LiDAR (_check_geometry) roi bam.
for sc in ("thang", "re_trai", "vong_trai"):
    J("pitch", f"{sc}:ngua18_sai-18", sc, CAM_HALF=55, CAM_PITCH=18, CAM_PITCH_ERR=-18)
J("pitch", "thang:ngua18_sai-12", "thang", CAM_HALF=55, CAM_PITCH=18, CAM_PITCH_ERR=-12)
J("pitch", "thang:cui12_sai12", "thang", CAM_HALF=55, CAM_PITCH=-12, CAM_PITCH_ERR=12)
J("pitch", "vat_thap_hoc:ngua18_sai-18", "duong_di", CAM_HALF=55, LOWBOX=LB_L, CAM_PITCH=18, CAM_PITCH_ERR=-18,
  WP=WP_HOC, WPV=0.25, WPT=35)

for r_ in ("", "1"):
    J("rssi", f"an_sau_vach:RSSI={r_ or '0'}", "an_sau_vach", CAM_HALF=55, RSSI=r_,
      TALLWALL="3.0:-6:3.0:1.5;3.0:2.4:3.0:6")
    J("rssi", f"sau_tu:RSSI={r_ or '0'}", "duong_di", CAM_HALF=55, RSSI=r_, TALLWALL="2.6:-0.6:2.6:0.6",
      WP="1.6:0;2.2:0.9;3.1:0.9;3.1:0.0", WPV=0.6, WPT=50)

# Cua bep THAT (bag follow_1014_153520, 14/10): cua 0.80 m giua hai thanh, mat hanh lang y = 1.10..1.22, canh cua mo
# ap ben phai sau vao bep, mat trong ben trai lui ra -> cho mo 0.95 m PHIA TRONG (khe gia ma planner cu chon), tuong
# hanh lang trai cach thanh cua trai 0.2 m (rem buoc gon). Xe that ket ca 2 lan vao bep tu ben phai.
KITCHEN = ("-0.78:1.22:14.0:1.22;-0.78:1.22:-0.78:1.30;-0.78:1.30:-0.83:1.30;-0.83:1.30:-0.83:2.10;"
           "-0.78:1.34:0.50:1.34;-1.80:1.10:-1.60:1.10;-1.60:1.10:-1.60:1.32;-1.60:1.32:-1.70:1.62;"
           "-1.70:1.62:-2.60:1.62;-2.60:1.62:-2.60:3.90;-2.60:3.90:0.50:3.90;0.50:3.90:0.50:1.34;"
           "-1.30:2.95:-0.75:2.95;-1.80:1.10:-4.00:1.10")
K_LEFT = ";-1.80:1.10:-1.80:-0.90"                   # tuong hanh lang trai (bo di = khong co vat can hai ben)
K_PASS = "-1.80:1.45:-0.70:1.45"                     # vach ngang qua o cua (phia trong)
for wl, wn in ((K_LEFT, "tuong_trai"), ("", "trong")):
    for nz in (0.01, 0.02):
        # nguoi di tu truoc mat xe, re trai vao cua (dung duong di trong bag), dung trong bep
        J("cua_bep", f"{wn}:re_trai:nz{nz}", "duong_di", CAM_HALF=55, TALLWALL=KITCHEN + wl, PASS_LINE=K_PASS,
          LIDAR_NOISE=nz, START="2.4:0.0:180", WP="1.2:0.30;-0.6:0.35;-1.15:0.75;-1.20:1.70;-1.55:2.60;-1.75:3.40",
          WPV=0.30, WPT=60)
        # nguoi di thang tu xa vao cua
        J("cua_bep", f"{wn}:thang:nz{nz}", "duong_di", CAM_HALF=55, TALLWALL=KITCHEN + wl, PASS_LINE=K_PASS,
          LIDAR_NOISE=nz, START="-1.10:-1.40:90", WP="-1.15:-0.30;-1.20:1.70;-1.55:2.60;-1.75:3.40", WPV=0.30,
          WPT=55)
    # nguoi DUNG YEN trong bep, xe xuat phat o nhieu tu the truoc cua (lech truc / lech goc) — thu rieng dong tac chui
    for st in ("-0.40:0.20:150", "-0.80:0.45:120", "-1.20:0.30:90", "-1.35:0.45:75", "-0.95:0.70:105"):
        J("cua_bep", f"{wn}:yen:{st}", "sau_vat_thap", CAM_HALF=55, TALLWALL=KITCHEN + wl, PASS_LINE=K_PASS,
          LIDAR_NOISE=0.01, PX=-1.60, PY=2.90, START=st, START_TRAIL=0.8)
    if not wl:
        # khong co tuong trai: nguoi di tu phia TRAI vong vao cua (doi xung voi bag)
        J("cua_bep", f"{wn}:re_phai", "duong_di", CAM_HALF=55, TALLWALL=KITCHEN, PASS_LINE=K_PASS,
          START="-3.40:0.10:0", WP="-2.40:0.30;-1.60:0.35;-1.25:0.75;-1.20:1.70;-1.55:2.60;-1.75:3.40",
          WPV=0.30, WPT=60)

# 14/10 — xe dang VONG qua vat can chinh (nguoi dung sau no) thi bi CHAN giua chung o ben dang vong: nguoi thu hai buoc
# vao dung chan hanh lang (CHAN), hoac vat thap khuat sau vat chinh. Xe phai quay lai, vong ben kia toi nguoi.
VC_WALLS = "0.0:1.5:4.5:1.5;0.0:-1.6:4.5:-1.6"          # tuong hai ben: hanh lang trai 1.0 m, phai 1.1 m
VC_BOX = "1.3:-0.5:1.7:0.5"
for lab, kw in (
        ("chan_trai", dict(TALLWALL=VC_WALLS, PX=3.3, PY=0.3, CHAN="1.5:4.0:40:1.0:1.45")),
        ("chan_phai", dict(TALLWALL=VC_WALLS, PX=3.3, PY=-0.3, CHAN="1.5:4.0:40:-1.05:-1.55")),
        ("chan_trai_muon", dict(TALLWALL=VC_WALLS, PX=3.3, PY=0.3, CHAN="1.9:6.5:40:1.0:1.45")),
        ("chan_trai_khong_tuong", dict(PX=3.3, PY=0.3, CHAN="1.5:4.0:40:1.0:1.6")),
        ("khuat_sau_trai", dict(TALLWALL=VC_WALLS, PX=3.3, PY=0.3, LOWBOX=VC_BOX + ";2.0:0.45:2.4:1.5"))):
    box = kw.pop("LOWBOX", VC_BOX)
    J("vong_chan", lab, "vong_chan", CAM_HALF=55, LOWBOX=box, T_END=45, **kw)

# 03/11 — Cua 0.80 m tuong day 10 cm, TU LANH (cao, che ca camera) dat sat sau tuong ben TRAI cua; nguoi qua cua roi
# RE TRAI ngay truoc mat tu lanh (xe that 15/10: xe vua qua cua thi xoay / tien lui mai du xung quanh trong). Doi chung:
# GHE (4 chan, thap) thay cho tu lanh, khong co gi, tu lanh ben PHAI + nguoi re phai, nguoi di thang.
TL_DOOR = dict(DOOR="3.2:0.80", WALL_T=0.10, PASS_LINE="3.3:-3:3.3:3")
TL_FRIDGE_L = "3.63:0.83:0.65:0.75:0:1.8"
TL_FRIDGE_R = "3.63:-0.83:0.65:0.75:0:1.8"
TL_CHAIR_L = ";".join(f"{x}:{y}:0.03:0.03:0:0.45" for x in (3.34, 3.76) for y in (0.49, 0.91))
TL_WP_L = "1.6:0;3.3:0;4.25:0.05;4.35:1.3;4.35:1.9"
TL_WP_R = "1.6:0;3.3:0;4.25:-0.05;4.35:-1.3;4.35:-1.9"
for lab, ob, wp in (("tu_lanh_trai:re_trai", TL_FRIDGE_L, TL_WP_L), ("ghe_trai:re_trai", TL_CHAIR_L, TL_WP_L),
                    ("trong:re_trai", "", TL_WP_L), ("tu_lanh_phai:re_phai", TL_FRIDGE_R, TL_WP_R),
                    ("tu_lanh_trai:thang", TL_FRIDGE_L, "1.6:0;3.3:0;5.4:0")):
    for v in (0.17, 0.22):
        J("tu_lanh", f"{lab}:v{v}", "duong_di", CAM_HALF=55, OBOX=ob, WP=wp, WPV=v, WPT=55, **TL_DOOR)

# 03/11 — Nguoi dung sau THUNG XOP 50x35 cm cao 35 cm dat NGHIENG (xe that 15/10: luc ne duoc, luc lai thang toi mep
# thung). Camera cao 0.70 m nhin qua thung. Them ca co ghe ben canh thung.
for ang in (0, 25, -30, 60):
    for st in ("0:0:0", "0:0.35:5", "0:-0.30:-5"):
        J("thung_xop", f"goc{ang}:{st}", "sau_vat_thap", CAM_HALF=55, OBOX=f"1.6:0.0:0.50:0.35:{ang}:0.30",
          PX=2.3, PY=0.05, START=st)
XOP_GHE = "1.6:0.0:0.50:0.35:25:0.30;" + ";".join(f"{x}:{y}:0.03:0.03:0:0.45" for x in (1.45, 1.87)
                                                for y in (0.55, 0.97))
for st in ("0:0:0", "0:0.4:0"):
    J("thung_xop", f"goc25+ghe:{st}", "sau_vat_thap", CAM_HALF=55, OBOX=XOP_GHE, PX=2.3, PY=0.05, START=st)
J("thung_xop", "goc25:nguoi_di_toi", "duong_di", CAM_HALF=55, OBOX="1.6:0.0:0.50:0.35:25:0.30",
  WP="1.2:-0.9;2.3:-0.9;2.3:0.05", WPV=0.25, WPT=35)

# 03/11 — GOC KET: thung xop nghieng phia truoc-trai + vat dai ben phai (sofa) + vat THAP phia SAU nam trong vung mu
# lidar (xe that 15/10: xoay trai roi lui, dung vat phia sau). Tieu chi chinh: KHONG CHAM (ke ca phia sau).
GK = "2.35:0.10:0.50:0.35:35:0.30;2.05:-0.85:2.3:0.45:0:0.9;0.55:-0.10:0.40:0.45:0:0.30"
for st in ("1.30:-0.15:10", "1.45:-0.20:25", "1.25:-0.25:-10"):
    for lab, (px, py) in (("nguoi_trai", (3.1, 1.1)), ("nguoi_truoc", (3.3, 0.6)), ("nguoi_trai_xa", (2.4, 1.7))):
        # xe vua tien vao goc 0.15 m (phia sau xa hon la vat thap trong vung mu — xe KHONG duoc lui vao do)
        J("goc_ket", f"{lab}:{st}", "sau_vat_thap", CAM_HALF=55, OBOX=GK, PX=px, PY=py, START=st, START_TRAIL=0.15)
# nguoi dan xe vao goc roi vong sang trai (xe tu di vao; vat thap nam BEN PHAI duong vao — xe thay no luc di ngang,
# luc da vao goc thi no o sau-phai, trong vung mu)
GK2 = "2.35:0.10:0.50:0.35:35:0.30;2.05:-0.85:2.3:0.45:0:0.9;0.75:-0.42:0.40:0.40:0:0.30"
J("goc_ket", "dan_vao_goc", "duong_di", CAM_HALF=55, OBOX=GK2, START="-0.3:0.15:0",
  WP="0.9:0.15;1.9:-0.25;1.9:-0.25;2.0:0.75;3.0:1.15", WPV=0.22, WPT=60)
J("goc_ket", "dan_vao_goc_cham", "duong_di", CAM_HALF=55, OBOX=GK2, START="-0.3:0.15:0",
  WP="0.9:0.15;1.9:-0.25;1.9:-0.25;1.9:-0.25;2.0:0.75;3.0:1.15", WPV=0.15, WPT=70)


def _pocket(w=0.85, gap=0.40, chair="chan", m_h=0.45, m_len=0.6, mirror=False, stop="xa"):
    """TUI HEP (05/11, xe that 05/11 bag 101558): vat chinh M phia truoc; ben PHAI M la ke de ngang (tua tuong) va ghe
    noi tiep ke nam lui ve sau M — loi giua M va ke rong `w` (xe vao duoc) roi khep lai con `gap` (chi nguoi lot) o
    giua M va ghe. Ben TRAI M trong. Nguoi di vao tui, lach qua khe hep roi di tiep sau M. Xe phai vong ben trai M;
    neu da lo vao tui thi phai tu quay dau ra (khong cham, khong lui vao cho chua biet).
    Tra ve dict env (OBOX, TALLWALL, WP) — toa do: xe o goc, M o x 1.9..1.9+m_len, mat phai M o y = -0.15."""
    s = -1.0 if mirror else 1.0
    my0 = -0.15                                   # mat phai M
    mx0, mx1 = 1.9, 1.9 + m_len
    sy1 = my0 - w                                 # mat trai ke
    sy0 = sy1 - 0.30
    cy1 = my0 - gap                               # mat trai ghe
    cx0, cx1 = mx1 - 0.05, mx1 + 0.35
    obs = [f"{0.5 * (mx0 + mx1):.3f}:{s * 0.20:.3f}:{m_len:.2f}:0.70:0:{m_h:.2f}",
           f"1.75:{s * 0.5 * (sy0 + sy1):.3f}:1.00:0.30:0:1.20"]
    if chair == "chan":
        obs += [f"{x:.3f}:{s * y:.3f}:0.03:0.03:0:0.45" for x in (cx0 + 0.02, cx1 - 0.02) for y in (cy1 - 0.02, cy1 - 0.38)]
    else:
        obs.append(f"{0.5 * (cx0 + cx1):.3f}:{s * (cy1 - 0.20):.3f}:0.40:0.40:0:0.45")
    wall_y = sy0 - 0.05
    walls = f"-1.5:{s * wall_y:.3f}:4.6:{s * wall_y:.3f};-1.5:{s * 2.0:.3f}:4.6:{s * 2.0:.3f};4.6:{s * wall_y:.3f}:4.6:{s * 2.0:.3f}"
    cy = 0.5 * (my0 + sy1)                        # giua tui
    end = (3.9, 1.2) if stop == "xa" else (3.2, my0 - 0.5 * gap - 0.05)
    wp = [(1.2, cy), (1.9, cy), (mx1 - 0.02, my0 - 0.5 * gap), (3.1, my0 - 0.5 * gap - 0.05)]
    if stop == "xa":
        wp += [(3.6, 0.3), end]
    return dict(OBOX=";".join(obs), TALLWALL=walls, WP=";".join(f"{x:.2f}:{s * y:.2f}" for x, y in wp)), cy * s


for (lab, kw) in (("w0.85", {}), ("w0.95", dict(w=0.95)), ("w1.10", dict(w=1.10)), ("khe0.5", dict(gap=0.50)),
                  ("ghe_khoi", dict(chair="khoi")), ("M_cao", dict(m_h=1.0)), ("M_ngan", dict(m_len=0.40)),
                  ("doi_xung", dict(mirror=True)), ("dung_sau_khe", dict(stop="gan"))):
    env, cy = _pocket(**kw)
    for st_lab, st in (("thang_tui", f"0:{cy:.2f}:0"), ("xa_tui", f"-1.0:{cy:.2f}:0"), ("giua", "0:0.20:0"),
                       ("ben_trai", f"0.3:{0.9 * (-1 if kw.get('mirror') else 1):.2f}:{-20 * (-1 if kw.get('mirror') else 1)}")):
        for v in ((0.30,) if lab != "w0.85" else (0.25, 0.35)):
            J("tui_hep", f"{lab}:{st_lab}:v{v}", "duong_di", CAM_HALF=55, START=st, WPV=v, WPT=round(6.5 / v + 45), **env)
# nhieu lidar 2 cm + doi mam (canh hon loan)
for sd in (2, 3):
    env, cy = _pocket()
    J("tui_hep", f"w0.85:thang_tui:nz2:seed{sd}", "duong_di", CAM_HALF=55, START=f"0:{cy:.2f}:0", WPV=0.30, WPT=70,
      LIDAR_NOISE=0.02, SEED=sd, **env)


def _clutter(seed, kmin=5, kmax=9, clear=0.45, sep=0.55, v=0.25):
    """Canh LON XON ngau nhien (03/11): kmin..kmax-1 vat hinh chu nhat xoay ngau nhien (thap / cao) trong phong rong
    4.8 m; nguoi di qua theo duong co do thoang >= clear (xe 0.60 m can 0.36) roi dung. Tra ve dict env hoac None."""
    import numpy as np
    from scipy import ndimage
    rng = np.random.default_rng(1000 + seed + (0 if kmin == 5 else 50000))
    res, x0, x1, y0, y1 = 0.05, -1.0, 7.0, -2.4, 2.4
    nx, ny = int((x1 - x0) / res), int((y1 - y0) / res)
    for _try in range(60):
        k = int(rng.integers(kmin, kmax))
        boxes = []
        tries = 0
        while len(boxes) < k and tries < 2000:
            tries += 1
            cx, cy = rng.uniform(1.4, 5.2), rng.uniform(-1.9, 1.9)
            if any(np.hypot(cx - b[0], cy - b[1]) < sep for b in boxes):
                continue
            L, W = rng.uniform(0.30, 0.80), rng.uniform(0.25, 0.55)
            h = rng.uniform(0.30, 0.50) if rng.random() < 0.7 else rng.uniform(1.0, 1.8)
            boxes.append((cx, cy, L, W, rng.uniform(0.0, 180.0), h))
        occ = np.zeros((ny, nx), bool)
        X, Y = np.meshgrid(x0 + (np.arange(nx) + 0.5) * res, y0 + (np.arange(ny) + 0.5) * res)
        for (cx, cy, L, W, a, _h) in boxes:
            c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
            lx, ly = c * (X - cx) + s * (Y - cy), -s * (X - cx) + c * (Y - cy)
            occ |= (np.abs(lx) <= L / 2) & (np.abs(ly) <= W / 2)
        occ[0, :] = occ[-1, :] = True                      # tuong y = +-2.4
        clr = ndimage.distance_transform_edt(~occ) * res
        free = clr >= clear
        sx, sy = int((1.5 - x0) / res), int((0.0 - y0) / res)
        gy_ = float(rng.uniform(-1.2, 1.2))
        gx, gy = int((6.2 - x0) / res), int((gy_ - y0) / res)
        if not (free[sy, sx] and free[gy, gx]):
            continue
        # Dijkstra 8 huong tren o thoang (gia uu tien do thoang lon: nguoi di giua khe)
        import heapq
        D = np.full((ny, nx), np.inf)
        P = -np.ones((ny, nx, 2), int)
        D[sy, sx] = 0.0
        hp = [(0.0, sy, sx)]
        while hp:
            du, uy, ux = heapq.heappop(hp)
            if du > D[uy, ux]:
                continue
            if (uy, ux) == (gy, gx):
                break
            for dy, dx in ((0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                vy, vx = uy + dy, ux + dx
                if 0 <= vy < ny and 0 <= vx < nx and free[vy, vx]:
                    nd = du + np.hypot(dy, dx) * (1.0 + 0.6 / max(clr[vy, vx], 0.3))
                    if nd < D[vy, vx]:
                        D[vy, vx] = nd
                        P[vy, vx] = (uy, ux)
                        heapq.heappush(hp, (nd, vy, vx))
        if not np.isfinite(D[gy, gx]):
            continue
        path = [(gy, gx)]
        while path[-1] != (sy, sx):
            path.append(tuple(P[path[-1]]))
        path = path[::-1]
        pts = [(x0 + (p[1] + 0.5) * res, y0 + (p[0] + 0.5) * res) for p in path[::8]] + \
              [(x0 + (gx + 0.5) * res, y0 + (gy + 0.5) * res)]
        length = sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        return dict(OBOX=";".join(f"{b[0]:.2f}:{b[1]:.2f}:{b[2]:.2f}:{b[3]:.2f}:{b[4]:.0f}:{b[5]:.2f}" for b in boxes),
                    TALLWALL="-1.0:2.4:8.0:2.4;-1.0:-2.4:8.0:-2.4",
                    WP=";".join(f"{x:.2f}:{y:.2f}" for x, y in pts), WPV=v, WPT=round(length / v + 30.0))
    return None


for seed in range(1, 25):
    _env = _clutter(seed)
    if _env:
        J("lon_xon", f"seed{seed}", "duong_di", CAM_HALF=55, **_env)
# KHO hon: 10-13 vat, sat nhau hon, duong nguoi di chi du cho xe (do thoang 0.40 m: du 4 cm moi ben), nguoi di 0.3 m/s
for seed in range(1, 21):
    _env = _clutter(seed, kmin=10, kmax=14, clear=0.40, sep=0.45, v=0.30)
    if _env:
        J("lon_xon_kho", f"seed{seed}", "duong_di", CAM_HALF=55, **_env)

# 03/11 — TOC DO: nguoi di thang nhanh / cham, dung dot ngot, re, co vat ben duong; xe CHO NANG (dap ung cham hon:
# ROBOT_TAU 0.35 s, chi dat 85 % toc do lenh). Do: khoang cach trung binh / lon nhat luc nguoi dang di, khong cham.
for v in (0.15, 0.25, 0.35, 0.45):
    J("toc_do", f"thang:v{v}", "duong_di", CAM_HALF=55, WP="1.6:0;14.0:0", WPV=v, WPT=round(min(40.0, 9.5 / v)))
    J("toc_do", f"thang:v{v}:nang", "duong_di", CAM_HALF=55, WP="1.6:0;14.0:0", WPV=v, WPT=round(min(40.0, 9.5 / v)),
      ROBOT_TAU=0.35, ROBOT_GAIN=0.85)
for v in (0.3, 0.45):
    J("toc_do", f"dung_dot_ngot:v{v}", "duong_di", CAM_HALF=55, WP="1.6:0;6.5:0", WPV=v, WPT=round(4.9 / v + 12))
    J("toc_do", f"dung_dot_ngot:v{v}:nang", "duong_di", CAM_HALF=55, WP="1.6:0;6.5:0", WPV=v, WPT=round(4.9 / v + 12),
      ROBOT_TAU=0.35, ROBOT_GAIN=0.85)
    J("toc_do", f"re_trai:v{v}", "duong_di", CAM_HALF=55, WP="1.6:0;5.0:0;6.5:1.5;6.5:5.0", WPV=v,
      WPT=round(8.3 / v + 10))
    # thung thap sat ben duong nguoi di: xe phai cham lai khi di ngang
    J("toc_do", f"vat_ben_duong:v{v}", "duong_di", CAM_HALF=55, WP="1.6:0;9.0:0", WPV=v, WPT=round(7.4 / v + 8),
      OBOX="4.5:-0.75:0.6:0.4:0:0.30;6.5:0.80:0.5:0.4:20:0.30")

# ── Doc ket qua tu dong chi so cua sim_follow.py ─────────────────────────
NUM = r"(-?[0-9.]+|nan|inf)"
PATS = [(r"camera=\s*" + NUM + "%", "cam", float), (r"mat_cam_dai_nhat=\s*" + NUM + "s", "mat_cam", float),
        (r"d_cuoi=\s*" + NUM, "d", float), (r"tt_cuoi=(\S+)", "tt", str),
        (r"ho_nho_nhat_voi_tuong=\s*" + NUM, "ho", float), (r"dung im trong cua lau nhat=\s*" + NUM, "stall", float),
        (r"qua cua ben: (\S+)", "qua_ben", str), (r"qua cua: (\S+)", "qua", str),
        (r"uoc_gan_ng2_hon=\s*" + NUM, "nham", float), (r"luc_ng2_dung_yen=\s*" + NUM, "ho_ng2", float),
        (r"ngang vat ben (TRAI|PHAI|khong toi)", "ben", str), (r"qua vach: (\S+)", "qua_vach", str),
        (r"lui_tong=\s*" + NUM, "lui", float), (r"doi_tien_lui=\s*" + NUM, "sw", float),
        (r"dung_im_xa_lau_nhat=\s*" + NUM, "ket", float), (r"kc_tb=\s*" + NUM, "dtb", float),
        (r"kc_max=\s*" + NUM, "dmax", float)]


def parse(out):
    d = {}
    for pat, key, conv in PATS:
        m = re.search(pat, out)
        if m:
            try:
                d[key] = conv(m.group(1))
            except ValueError:
                d[key] = m.group(1)
    return d


def run(pkg_dir, job):
    env = dict(os.environ)
    env.update(job["env"])
    env.pop("PKG_DIR", None)
    if pkg_dir:
        env["PKG_DIR"] = pkg_dir
    t0 = time.time()
    p = subprocess.run([sys.executable, os.path.join(HERE, "sim_follow.py"), "-q", job["scen"]], cwd=HERE, env=env,
                       capture_output=True, text=True, timeout=3600)
    r = parse(p.stdout)
    r["sec"] = round(time.time() - t0, 1)
    if p.returncode != 0:
        r["err"] = p.stderr[-400:]
    return r


def passed(group, label, x):
    """True/False theo tieu chi nhom; None = chi de xem (khong cham diem)."""
    if not x or "err" in x:
        return False
    if group.startswith("door") or group == "stuck":
        return x.get("qua_ben") == "CO" and x.get("ho", 0.0) > 0.0
    if group == "cua_bep":
        return x.get("qua_vach") == "CO" and x.get("ho", 0.0) > 0.0
    if group == "tu_lanh":
        # qua cua, toi gan nguoi, khong cham, khong tien-lui / dung im lau (xe that 15/10)
        return (x.get("qua_vach") == "CO" and x.get("ho", 0.0) > 0.0 and x.get("d", 9.0) <= 1.7
                and x.get("sw", 99) <= 6 and x.get("ket", 99.0) <= 8.0)
    if group == "thung_xop":
        return (x.get("tt") in ("ARRIVED", "FOLLOW") and 0.8 <= x.get("d", 9.0) <= 1.45 and x.get("ho", 0.0) > 0.0
                and x.get("lui", 9.0) <= 0.5)
    if group in ("goc_ket", "lon_xon", "lon_xon_kho"):
        return x.get("ho", 0.0) > 0.0 and x.get("d", 9.0) <= 1.7
    if group == "tui_hep":
        # khong cham; ket thuc gan nguoi (vong ben trong / hoac dung cho tot nhat sau khe khi nguoi dung ngay sau khe);
        # khong dung im lau khi nguoi con xa
        return (x.get("ho", 0.0) > 0.0 and x.get("tt") in ("ARRIVED", "FOLLOW") and x.get("d", 9.0) <= 1.6
                and x.get("ket", 99.0) <= 10.0)
    if group == "toc_do":
        # Khong cham; nguoi dung lai thi xe toi gan (<= 1.8 m); nguoi dang di ma xe DU SUC theo (toc do nguoi <= 90 %
        # toc do toi da xe dat duoc: 0.40 m/s, cho nang 85 %) thi khong tut lai qua 2.6 m. Nguoi nhanh hon suc xe va
        # khong dung (thang:v0.45) thi chi xem.
        if x.get("ho", 0.0) <= 0.0:
            return False
        v = float(re.search(r"v([0-9.]+)", label).group(1))
        v_lim = 0.9 * 0.40 * (0.85 if label.endswith(":nang") else 1.0)
        if label.startswith("thang:") and v > v_lim:
            return None
        ok = x.get("d", 9.0) <= 1.8
        if v <= v_lim:
            ok = ok and x.get("dmax", 9.0) <= 2.6
        return ok
    if group in ("lowbox", "hard", "vong_chan") or (group == "rssi" and label.endswith("RSSI=1")):
        return x.get("tt") in ("ARRIVED", "FOLLOW") and 0.8 <= x.get("d", 9.0) <= 1.4 and x.get("ho", 0.0) > 0.0
    if group == "pitch":
        # nguoi van dang di o cuoi kich ban -> xe bam sau 1.2-1.6 m (camera dung goc: 1.30)
        hi = 1.4 if label.startswith("vat_thap") else 1.7
        return x.get("tt") in ("ARRIVED", "FOLLOW") and 0.8 <= x.get("d", 9.0) <= hi and x.get("ho", 0.0) > 0.0
    if group == "chen":
        if label.startswith("di:2.2:8.0"):
            return None
        return x.get("nham", 0.0) <= 10.0 and x.get("ho_ng2", 9.0) >= 0.05
    if group == "regress" and (x.get("qua") or x.get("qua_ben")):
        return (x.get("qua") or x.get("qua_ben")) == "CO" and x.get("ho", 0.0) > 0.0
    return None


def cell(x):
    if not x or "err" in x:
        return "LOI CHAY".ljust(42)
    side = (x.get("ben") or x.get("qua_ben") or x.get("qua") or "")[:1]
    return (f"{x.get('cam', 0):3.0f}% {x.get('mat_cam', 0):4.1f}s d{x.get('d', 0):4.2f} {str(x.get('tt'))[:7]:7s} "
            f"h{x.get('ho', 0):5.2f} {side} L{x.get('lui', 0):3.1f}/{x.get('sw', 0):.0f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--groups", default="", help="vd. lowbox,hard (mac dinh: tat ca)")
    ap.add_argument("--base", default="", help="git ref de so (vd. 6d0aa54); rong = chi chay ban hien tai")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--json", default="", help="ghi ket qua tho ra file")
    a = ap.parse_args()
    groups = [g for g in a.groups.split(",") if g]
    jobs = [j for j in JOBS if not groups or any(j["group"].startswith(g) for g in groups)]
    # REGRESS_NEW_PKG=<thu muc package da chep> : chay "moi" tren BAN CHUP (sua code trong luc dang chay khong lam lan ket qua)
    vers = [("moi", os.environ.get("REGRESS_NEW_PKG") or None)]
    tmp = None
    if a.base:
        top = subprocess.run(["git", "rev-parse", "--show-toplevel"], cwd=HERE, capture_output=True,
                             text=True, check=True).stdout.strip()
        tmp = tempfile.mkdtemp(prefix="regress_base_")
        rel = os.path.relpath(os.path.dirname(HERE), top)
        arch = subprocess.run(["git", "archive", a.base, rel], cwd=top, capture_output=True, check=True).stdout
        subprocess.run(["tar", "-x", "-C", tmp], input=arch, check=True)
        vers.insert(0, (a.base, os.path.join(tmp, rel)))
    print(f"{len(jobs)} kich ban x {len(vers)} ban, {a.jobs} tien trinh song song ...", flush=True)
    res = {}
    try:
        with ThreadPoolExecutor(max_workers=a.jobs) as ex:
            futs = {ex.submit(run, pkg, j): (name, j) for (name, pkg) in vers for j in jobs}
            for f in futs:
                name, j = futs[f]
                res.setdefault((j["group"], j["label"]), {})[name] = f.result()
    finally:
        if tmp:
            shutil.rmtree(tmp, ignore_errors=True)
    names = [n for n, _ in vers]
    total = {n: [0, 0] for n in names}
    cur = None
    for (g, lab), v in res.items():
        gg = g if not g.startswith(("door", "stuck")) else g
        if gg != cur:
            cur = gg
            print(f"\n=== {gg} ===")
            print(f"  {'kich ban':24s} | " + " | ".join(f"{n:^42s}" for n in names))
        marks = []
        for n in names:
            ok = passed(g, lab, v.get(n))
            if ok is not None:
                total[n][0] += int(ok)
                total[n][1] += 1
            marks.append(("DAT " if ok else "LOI ") if ok is not None else "    ")
        print(f"  {lab[:24]:24s} | " + " | ".join(m + cell(v.get(n))[:46] for m, n in zip(marks, names)))
    print("\nTONG (cac kich ban co tieu chi): " + ", ".join(f"{n}: {t[0]}/{t[1]} DAT" for n, t in total.items()))
    if a.json:
        json.dump({f"{g}|{lab}": v for (g, lab), v in res.items()}, open(a.json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
