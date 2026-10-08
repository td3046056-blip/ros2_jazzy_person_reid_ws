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

CACH DUNG (trong src/person_follow_nav/scripts):
  python3 regress_follow.py                        # tat ca nhom, ban hien tai (~10 phut tren 32 nhan)
  python3 regress_follow.py --base 6d0aa54         # so voi ban cu (git archive vao thu muc tam)
  python3 regress_follow.py --groups lowbox,hard   # chi vai nhom
  python3 regress_follow.py --jobs 8 --json kq.json

DAT nghia la: cua -> qua cua va khong cham tuong; vat thap / kho -> ket thuc ARRIVED/FOLLOW cach nguoi 0.8-1.4 m va
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
                  SIDE_DOOR="3.0:0.81:-0.9", START=f"{x}:{y}:{yaw}")

for wt in (0, 0.12):
    for x in (2.8, 3.1, 3.4, 3.7, 4.0):
        for yaw in (-105, -90, -75):
            J(f"door_grid_wt{wt}", f"{x}:0.0:{yaw}", "cua_ben_yen", CAM_HALF=31, WALL_T=wt,
              SIDE_DOOR="3.0:0.81:-0.9", START=f"{x}:0.0:{yaw}")

for st in ("3.300:-0.790:-105.0", "3.480:-0.840:-105.0", "3.290:-0.770:-100.0", "3.480:-0.840:-100.0",
           "3.300:-0.750:-95.0", "3.480:-0.840:-95.0", "3.340:-0.840:-85.0", "3.460:-0.790:-85.0",
           "3.330:-0.840:-80.0", "3.450:-0.800:-80.0", "3.340:-0.840:-75.0", "3.460:-0.810:-75.0"):
    J("stuck", st, "cua_ben_yen", CAM_HALF=31, SIDE_DOOR="3.0:0.81:-0.9", START=st)

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

# ── Doc ket qua tu dong chi so cua sim_follow.py ─────────────────────────
NUM = r"(-?[0-9.]+|nan|inf)"
PATS = [(r"camera=\s*" + NUM + "%", "cam", float), (r"mat_cam_dai_nhat=\s*" + NUM + "s", "mat_cam", float),
        (r"d_cuoi=\s*" + NUM, "d", float), (r"tt_cuoi=(\S+)", "tt", str),
        (r"ho_nho_nhat_voi_tuong=\s*" + NUM, "ho", float), (r"dung im trong cua lau nhat=\s*" + NUM, "stall", float),
        (r"qua cua ben: (\S+)", "qua_ben", str), (r"qua cua: (\S+)", "qua", str),
        (r"uoc_gan_ng2_hon=\s*" + NUM, "nham", float), (r"luc_ng2_dung_yen=\s*" + NUM, "ho_ng2", float),
        (r"ngang vat ben (TRAI|PHAI|khong toi)", "ben", str)]


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
    if group in ("lowbox", "hard") or (group == "rssi" and label.endswith("RSSI=1")):
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
            f"h{x.get('ho', 0):5.2f} {side}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--groups", default="", help="vd. lowbox,hard (mac dinh: tat ca)")
    ap.add_argument("--base", default="", help="git ref de so (vd. 6d0aa54); rong = chi chay ban hien tai")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--json", default="", help="ghi ket qua tho ra file")
    a = ap.parse_args()
    groups = [g for g in a.groups.split(",") if g]
    jobs = [j for j in JOBS if not groups or any(j["group"].startswith(g) for g in groups)]
    vers = [("moi", None)]
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
        print(f"  {lab[:24]:24s} | " + " | ".join(m + cell(v.get(n))[:38] for m, n in zip(marks, names)))
    print("\nTONG (cac kich ban co tieu chi): " + ", ".join(f"{n}: {t[0]}/{t[1]} DAT" for n, t in total.items()))
    if a.json:
        json.dump({f"{g}|{lab}": v for (g, lab), v in res.items()}, open(a.json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
