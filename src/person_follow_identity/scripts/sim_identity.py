#!/usr/bin/env python3
"""
sim_identity.py — mo phong dam dong bang anh nguoi THAT de so sanh logic khoa nguoi.

Khong can ROS, khong can camera. Can bo anh Market-1501 (chi dung de thu, khong dua vao repo):
    https://huggingface.co/datasets/aveocr/Market-1501-v15.09.15.zip  (152 MB, giai nen)

Cach lam:
  - Khung 640x480 ghep tu anh cac nguoi (bounding_box_test — ID KHONG dung de huan luyen
    mang ReID) theo hinh hoc camera KINGSEN: FOV ngang --hfov (do duoc 107.9 do), camera cao
    --cam-h (m, tren xe 0.34), phep chieu mat ca deu nhu ong that. Nguoi gan xe khong vua khung
    doc -> bi cat nhu that.
    (Bang ket qua 30/09 trong README chay voi gia dinh cu: FOV 62 do, cao 0.6 m.)
  - Ve xa truoc gan sau nen nguoi dung truoc che nguoi dung sau.
  - DeepSORT mo phong: can n_init khung lien tiep moi co ID, mat qua max_age khung thi ID moi
    khi quay lai, hai nguoi cat nhau (IoU > 0.3) roi tach ra thi TRAO ID voi xac suat --p-switch.
  - TargetIdentityManager chay THAT tren khung do, feature ReID tinh bang mang that.
  - Nguoi gay nham: 2 nguoi GIONG muc tieu nhat (theo feature) + 2 nguoi ngau nhien.

Kich ban (sau 32 s enroll: muc tieu di tu 2.6 m lai 1.1 m roi lui ra, mot nguoi di xa phia sau):
  - 60 s bam: muc tieu di qua lai 1.1-2.4 m, 4 nguoi khac di cat ngang truoc/sau.
  - t=20-24 s: muc tieu ra khoi khung 4 s (nguoi khac van o do) roi quay lai.
  - t=40-43 s: mot nguoi GIONG muc tieu dung chan ngay truoc muc tieu 3 s roi di.

Chi so (trong 60 s bam):
  nham_s   : so giay bao muc tieu la NGUOI KHAC (nguy hiem nhat: xe bam nham)
  thieu_%  : ti le khung thay ro muc tieu ma khong bao
  lai_s    : thoi gian tu luc muc tieu quay lai khung (co ID) den luc bao dung
  sau_che_s: thoi gian tu luc nguoi chan di khoi den luc bao dung

Chay:
  python3 sim_identity.py --market ~/Market-1501-v15.09.15 --runs 12
  python3 sim_identity.py --market ... --variants old,new --fps 8
"""
from __future__ import annotations

import os

# Moi tien trinh con 1-2 luong: khong thi BLAS/OpenCV moi tien trinh mo ca chuc luong -> qua tai
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse
import glob
import importlib
import inspect
import math
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass
import multiprocessing as mp
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TRACE = bool(os.environ.get("SIM_TRACE"))
WS_SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(WS_SRC, "person_reid_tracker"))
sys.path.insert(0, os.path.join(WS_SRC, "person_follow_identity"))

W, H = 640, 480
# KINGSEN o 640x480 do bang thuoc 30/09: FOV ngang 114.4 do, ong "mat ca deu" (goc ti le voi
# pixel, f = 324 px). Doi bang --hfov / --projection.
PROJ = "equidistant"
FX = (W / 2) / math.radians(114.4 / 2)
FY = FX  # pixel vuong
CKPT = os.path.join(WS_SRC, "person_reid_tracker", "model_assets", "ckpt.t7")
KINGSEN_YAML = os.path.join(WS_SRC, "person_follow_robot", "config", "identity_lock_kingsen.yaml")


# ─────────────────────────────────────────────────────────────────────────────
# Mang ReID + tien xu ly (BGR nhu code cu, hoac RGB nhu code moi)
# ─────────────────────────────────────────────────────────────────────────────
_NET = None


def _net():
    global _NET
    if _NET is None:
        import torch
        from tracking_module.deep_sort.deep.model import Net
        torch.set_num_threads(2)
        cv2.setNumThreads(1)
        net = Net(reid=True)
        st = torch.load(CKPT, map_location="cpu", weights_only=False)
        net.load_state_dict(st["net_dict"] if "net_dict" in st else st)
        _NET = net.eval()
    return _NET


MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def embed(crops: List[np.ndarray], rgb: bool) -> np.ndarray:
    import torch
    arr = []
    for im in crops:
        if rgb:
            im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        x = cv2.resize(im.astype(np.float32) / 255.0, (64, 128))
        arr.append(((x - MEAN) / STD).transpose(2, 0, 1))
    with torch.inference_mode():
        return _net()(torch.from_numpy(np.stack(arr))).numpy()


def make_extractor(rgb: bool):
    def extract(frame, bboxes):
        out: List[Optional[np.ndarray]] = [None] * len(bboxes)
        crops, idx = [], []
        for i, (x1, y1, x2, y2) in enumerate(bboxes):
            if x2 > x1 + 2 and y2 > y1 + 2:
                crops.append(frame[y1:y2, x1:x2])
                idx.append(i)
        if crops:
            for i, f in zip(idx, embed(crops, rgb)):
                out[i] = f.astype(np.float32)
        return out
    return extract


# ─────────────────────────────────────────────────────────────────────────────
# Du lieu
# ─────────────────────────────────────────────────────────────────────────────
def load_market(root: str) -> Dict[int, List[str]]:
    by_id: Dict[int, List[str]] = {}
    for p in sorted(glob.glob(os.path.join(root, "bounding_box_test", "*.jpg"))):
        m = re.match(r"(-?\d+)_c\d", os.path.basename(p))
        pid = int(m.group(1)) if m else -1
        if pid > 0:
            by_id.setdefault(pid, []).append(p)
    return {k: v for k, v in by_id.items() if len(v) >= 12}


# ─────────────────────────────────────────────────────────────────────────────
# Canh
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Person:
    key: str
    pid: int
    imgs: List[np.ndarray]
    height_m: float
    x: float = 0.0
    z: float = 3.0
    present: bool = True
    img_i: int = 0


PITCH = 0.0  # goc ngua len cua camera (rad)


def _proj(a: float) -> float:
    """Goc (rad) so voi truc quang -> do lech pixel: pinhole f*tan(a), mat ca deu f*a."""
    return math.tan(a) if PROJ == "pinhole" else a


def person_rect(p: Person, cam_h: float) -> Tuple[int, int, int, int]:
    # Camera ngua len PITCH: diem o do cao y, xa z co goc ngang (elevation) atan((y-cam_h)/z) - PITCH
    top = H / 2 - FY * _proj(math.atan((p.height_m - cam_h) / p.z) - PITCH)
    bot = H / 2 - FY * _proj(math.atan(-cam_h / p.z) - PITCH)
    cx = W / 2 + FX * _proj(math.atan2(p.x, p.z))
    hw = 0.25 * (bot - top)
    return int(round(cx - hw)), int(round(top)), int(round(cx + hw)), int(round(bot))


def make_background(rng) -> np.ndarray:
    bg = rng.integers(60, 200, size=(12, 16, 3)).astype(np.uint8)
    bg = cv2.resize(bg, (W, H), interpolation=cv2.INTER_CUBIC)
    bg[int(H * 0.62):] = (bg[int(H * 0.62):] * 0.6).astype(np.uint8)  # san
    return cv2.GaussianBlur(bg, (0, 0), 3)


def render(bg, people: List[Person], cam_h: float, rng=None):
    frame = bg.copy()
    rects = {}
    order = sorted([p for p in people if p.present and p.z > 0.3], key=lambda p: -p.z)
    for p in order:
        x1, y1, x2, y2 = person_rect(p, cam_h)
        rects[p.key] = (x1, y1, x2, y2)
        img = p.imgs[p.img_i % len(p.imgs)]
        rw, rh = max(2, x2 - x1), max(2, y2 - y1)
        if rw * rh > 4 * W * H:
            continue
        spr = cv2.resize(img, (rw, rh), interpolation=cv2.INTER_LINEAR)
        if rng is not None:
            # Khung video that khac nhau tung chut (anh sang, mo, dang) — anh Market thi co dinh
            gain, bias = rng.uniform(0.9, 1.1), rng.uniform(-8, 8)
            spr = np.clip(spr.astype(np.float32) * gain + bias, 0, 255).astype(np.uint8)
        cx1, cy1, cx2, cy2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)
        if cx2 <= cx1 or cy2 <= cy1:
            continue
        frame[cy1:cy2, cx1:cx2] = spr[cy1 - y1:cy2 - y1, cx1 - x1:cx2 - x1]
    # Ti le nhin thay: phan trong khung khong bi nguoi gan hon che
    vis = {}
    for i, p in enumerate(order):
        x1, y1, x2, y2 = rects[p.key]
        cx1, cy1, cx2, cy2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)
        if cx2 <= cx1 or cy2 <= cy1:
            vis[p.key] = 0.0
            continue
        mask = np.ones((cy2 - cy1, cx2 - cx1), bool)
        for q in order[i + 1:]:
            qx1, qy1, qx2, qy2 = rects[q.key]
            ix1, iy1, ix2, iy2 = max(cx1, qx1), max(cy1, qy1), min(cx2, qx2), min(cy2, qy2)
            if ix2 > ix1 and iy2 > iy1:
                mask[iy1 - cy1:iy2 - cy1, ix1 - cx1:ix2 - cx1] = False
        vis[p.key] = float(mask.mean())
    return frame, rects, vis


def iou(a, b) -> float:
    ix1, iy1, ix2, iy2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


class FakeDeepSort:
    """Giu ID khi con thay; ID moi sau max_age; trao ID khi hai nguoi cat nhau voi xs p_switch."""

    def __init__(self, rng, n_init=4, max_age=30, p_switch=0.3):
        self.rng, self.n_init, self.max_age, self.p_switch = rng, n_init, max_age, p_switch
        self.next_id = 1
        self.tid: Dict[str, Optional[int]] = {}
        self.hits: Dict[str, int] = {}
        self.miss: Dict[str, int] = {}
        self.entangled: set = set()

    def step(self, rects, vis, ok_det):
        from person_reid_tracker.types import TrackCandidate
        det = {k: r for k, r in rects.items() if ok_det.get(k)}
        for k in list(self.tid):
            if k not in det:
                self.miss[k] = self.miss.get(k, 0) + 1
                self.hits[k] = 0
                if self.miss[k] > self.max_age:
                    self.tid[k] = None
        for k in det:
            self.miss[k] = 0
            self.hits[k] = self.hits.get(k, 0) + 1
            if self.tid.get(k) is None and self.hits[k] >= self.n_init:
                self.tid[k] = self.next_id
                self.next_id += 1
        keys = [k for k in det if self.tid.get(k) is not None]
        for i, a in enumerate(keys):
            for b in keys[i + 1:]:
                pair = tuple(sorted((a, b)))
                o = iou(det[a], det[b])
                if o > 0.3:
                    self.entangled.add(pair)
                elif pair in self.entangled and o < 0.05:
                    self.entangled.discard(pair)
                    if self.rng.random() < self.p_switch:
                        self.tid[a], self.tid[b] = self.tid[b], self.tid[a]
        tracks, owner = [], {}
        for k in keys:
            x1, y1, x2, y2 = det[k]
            j = self.rng.integers(-2, 3, size=4)
            bb = (max(0, x1 + j[0]), max(0, y1 + j[1]), min(W - 1, x2 + j[2]), min(H - 1, y2 + j[3]))
            if bb[2] - bb[0] < 8 or bb[3] - bb[1] < 16:
                continue
            tracks.append(TrackCandidate(track_id=int(self.tid[k]), bbox=bb, conf=0.8))
            owner[int(self.tid[k])] = k
        return tracks, owner


# ─────────────────────────────────────────────────────────────────────────────
# He thong can so sanh
# ─────────────────────────────────────────────────────────────────────────────
def _git_show(rev: str, rel: str) -> str:
    return subprocess.check_output(["git", "-C", WS_SRC, "show", f"{rev}:src/{rel}"], text=True)


def load_old_package(rev: str) -> str:
    """Chep identity_lock_manager/gallery/gait o commit `rev` ra goi tam 'old_pfi'."""
    d = tempfile.mkdtemp(prefix="old_pfi_")
    pkg = os.path.join(d, "old_pfi")
    os.makedirs(pkg)
    open(os.path.join(pkg, "__init__.py"), "w").close()
    for f in ["identity_lock_manager.py", "identity_gallery.py", "gait_identity.py"]:
        with open(os.path.join(pkg, f), "w") as fh:
            fh.write(_git_show(rev, f"person_follow_identity/person_follow_identity/{f}"))
    with open(os.path.join(d, "old_kingsen.yaml"), "w") as fh:
        fh.write(_git_show(rev, "person_follow_robot/config/identity_lock_kingsen.yaml"))
    return d


def build_manager(module, yaml_path: str):
    import yaml
    params = yaml.safe_load(open(yaml_path))["identity_lock_node"]["ros__parameters"]
    params = dict(params)
    params["select_mode"] = params.get("initial_select_mode", "center_largest")
    sig = inspect.signature(module.TargetIdentityManager.__init__)
    kw = {k: v for k, v in params.items() if k in sig.parameters}
    return module.TargetIdentityManager(**kw)


# ─────────────────────────────────────────────────────────────────────────────
# Mot lan chay
# ─────────────────────────────────────────────────────────────────────────────
def run_one(args_tuple):
    seed, variant, fps, market, cam_h, p_switch, old_dir, ids_all, feats_mean, n_look, hfov, pitch_deg, proj = args_tuple
    global FX, FY, PITCH, PROJ
    PROJ = proj
    FX = FY = (W / 2) / (math.tan(math.radians(hfov / 2)) if proj == "pinhole" else math.radians(hfov / 2))
    PITCH = math.radians(pitch_deg)
    rng = np.random.default_rng(seed)
    if variant.startswith("old"):
        sys.path.insert(0, old_dir)
        mod = importlib.import_module("old_pfi.identity_lock_manager")
        yaml_path = os.path.join(old_dir, "old_kingsen.yaml")
    else:
        mod = importlib.import_module("person_follow_identity.identity_lock_manager")
        yaml_path = KINGSEN_YAML
    rgb = variant != "old"  # "old" = code cu nguyen ban (BGR); "old+rgb" = logic cu + sua RGB
    mgr = build_manager(mod, yaml_path)
    extract = make_extractor(rgb)

    # Chon nguoi: muc tieu + 2 nguoi giong nhat + 2 ngau nhien + 1 nguoi di xa luc enroll
    tgt = int(ids_all[rng.integers(len(ids_all))])
    sims = feats_mean @ feats_mean[ids_all.index(tgt)]
    order = [ids_all[i] for i in np.argsort(-sims) if ids_all[i] != tgt]
    look = order[:n_look]
    rest = [i for i in ids_all if i != tgt and i not in look]
    rnd = [int(x) for x in rng.choice(rest, 5 - n_look, replace=False)]
    look = look + rnd[:2 - n_look] if n_look < 2 else look
    rnd = rnd[2 - n_look:] if n_look < 2 else rnd

    def mk(key, pid):
        paths = rng.permutation(market[pid])
        return Person(key, pid, [cv2.imread(p) for p in paths], float(rng.uniform(1.58, 1.85)))

    T = mk("T", tgt)
    # Anh enroll va anh luc bam TACH RIENG: luc bam muc tieu xuat hien voi dang/goc nhin chua hoc
    half = max(4, len(T.imgs) // 2)
    t_enroll_imgs, t_test_imgs = T.imgs[:half], T.imgs[half:] or T.imgs[:half]
    T.imgs = t_enroll_imgs
    L1, L2, R1, R2, BG = mk("L1", look[0]), mk("L2", look[1]), mk("R1", rnd[0]), mk("R2", rnd[1]), mk("BG", rnd[2])
    people = [T, L1, L2, R1, R2, BG]
    bg = make_background(rng)
    ds = FakeDeepSort(rng, p_switch=p_switch)
    dt = 1.0 / fps

    # Quy dao nguoi khac: di ngang qua lai o do sau co dinh, lech pha ngau nhien
    paths = {}
    for p, zr in [(L1, (1.4, 2.8)), (L2, (0.9, 2.0)), (R1, (1.6, 3.5)), (R2, (1.0, 2.6))]:
        paths[p.key] = (float(rng.uniform(*zr)), float(rng.uniform(0.35, 0.9)), float(rng.uniform(0, 2 * math.pi)), float(rng.uniform(1.4, 2.2)))

    stats = dict(visible=0, correct=0, wrong=0, reacq=None, after_block=None, drops=0, enroll_s=0.0)
    reentered_at = None
    block_end = None
    prev_correct = False
    mgr.start_enrollment(now=0.0)

    def place_others(tb):
        for p in (L1, L2, R1, R2):
            z, speed, phase, half = paths[p.key]
            p.present = True
            p.z = z
            p.x = half * math.sin(phase + speed * tb / half)

    # --- Enroll: muc tieu di tu 2.6 m lai 1.1 m roi lui ra (chu ky 30 s), mot nguoi di xa phia sau.
    # Chay toi khi xong (toi da 60 s); qua 31 s ma van dang enroll thi goi finish nhu nguoi van hanh.
    for p in (L1, L2, R1, R2):
        p.present = False
    i = 0
    while True:
        t = i * dt
        T.z = 2.6 - 1.5 * abs(math.sin(math.pi * t / 30.0))
        T.x = 0.12 * math.sin(2 * math.pi * t / 7.0)
        BG.z, BG.x = 4.5, -2.2 + 4.4 * ((t / 9.0) % 1.0)
        for p in people:
            if rng.random() < 0.5:  # nguoi di chuyen: dang/goc nhin doi moi khung
                p.img_i += 1
        frame, rects, vis = render(bg, people, cam_h, rng)
        ok_det = {k: (vis[k] >= 0.35 and rects[k][3] - rects[k][1] >= 40 and rng.random() > 0.03) for k in rects}
        tracks, owner = ds.step(rects, vis, ok_det)
        mgr.update(frame, tracks, extract, now=t)
        i += 1
        if getattr(mgr, "enrolling", False) and t >= 31.0:
            mgr.finish_enrollment()
        if (not getattr(mgr, "enrolling", False) and mgr.identity_ready) or t >= 60.0:
            break
    t0 = i * dt
    stats["enroll_s"] = t0
    T.imgs = t_test_imgs
    last_status = ""
    BG.present = False

    for j in range(int(60.0 * fps)):
        tb = j * dt
        t = t0 + tb
        T.z = 1.75 + 0.65 * math.sin(2 * math.pi * tb / 17.0)
        T.x = 0.35 * math.sin(2 * math.pi * tb / 11.0)
        if 20.0 <= tb < 24.0:
            T.x = 3.0 * T.z  # ra khoi khung ngang
        place_others(tb)
        if 39.2 <= tb < 43.0:
            # L1 di bo toi dung chan ngay truoc muc tieu (0.8 s) roi dung 3 s
            bz = max(0.7, T.z - 0.45)
            bx = T.x * bz / T.z + 0.08
            k = min(1.0, (tb - 39.2) / 0.8)
            L1.z = (1 - k) * L1.z + k * bz
            L1.x = (1 - k) * L1.x + k * bx
        for p in people:
            if rng.random() < 0.5:  # nguoi di chuyen: dang/goc nhin doi moi khung
                p.img_i += 1
        frame, rects, vis = render(bg, people, cam_h, rng)
        ok_det = {k: (vis[k] >= 0.35 and rects[k][3] - rects[k][1] >= 40 and rng.random() > 0.03) for k in rects}
        tracks, owner = ds.step(rects, vis, ok_det)
        res = mgr.update(frame, tracks, extract, now=t)
        reported = None
        if res.target is not None and not str(res.status).startswith("ENROLL"):
            reported = owner.get(int(res.target.track_id))
        t_vis = vis.get("T", 0) >= 0.5 and any(v == "T" for v in owner.values())
        if t_vis:
            stats["visible"] += 1
        if reported == "T":
            stats["correct"] += 1
        elif reported is not None:
            stats["wrong"] += 1
        if prev_correct and reported != "T" and t_vis and vis.get("T", 0) > 0.9:
            stats["drops"] += 1
        if TRACE and (reported not in (None, "T") or res.status != last_status):
            ids = {k: tid for tid, k in owner.items()}
            print(f"[{variant} s{seed}] tb={tb:5.2f} bao={reported} status={res.status} id_T={ids.get('T')} "
                  f"visT={vis.get('T', 0):.2f} ids={ids} | {res.reason[:150]}")
        last_status = res.status
        prev_correct = reported == "T"
        if tb >= 24.0 and reentered_at is None and any(v == "T" for v in owner.values()):
            reentered_at = tb
        if reentered_at is not None and stats["reacq"] is None and reported == "T":
            stats["reacq"] = tb - reentered_at
        if tb >= 43.0 and block_end is None:
            block_end = tb
        if block_end is not None and stats["after_block"] is None and reported == "T":
            stats["after_block"] = tb - block_end
    for k in ("reacq", "after_block"):
        if stats[k] is None:
            stats[k] = 99.0
    stats["wrong_s"] = stats["wrong"] / fps
    stats["miss"] = 1.0 - stats["correct"] / max(1, stats["visible"])
    stats["enrolled"] = bool(mgr.identity_ready)
    return variant, seed, stats


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--market", required=True, help="thu muc Market-1501-v15.09.15")
    ap.add_argument("--runs", type=int, default=12)
    ap.add_argument("--variants", default="old,old+rgb,new")
    ap.add_argument("--fps", default="8", help="vd 8 hoac 8,15")
    ap.add_argument("--cam-h", type=float, default=0.34, help="do cao camera (m); tren xe 34 cm (30/09), se nang ~70 cm")
    ap.add_argument("--hfov", type=float, default=114.4, help="FOV ngang (do); KINGSEN 640x480 do duoc 114.4")
    ap.add_argument("--projection", default="equidistant", choices=["equidistant", "pinhole"],
                    help="KINGSEN la mat ca deu (goc ti le voi pixel)")
    ap.add_argument("--cam-pitch", type=float, default=0.0, help="goc ngua len cua camera (do)")
    ap.add_argument("--p-switch", type=float, default=0.3, help="xac suat DeepSORT trao ID khi hai nguoi cat nhau")
    ap.add_argument("--old-rev", default="HEAD", help="commit chua code cu")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--verbose", action="store_true", help="in ket qua tung lan chay")
    ap.add_argument("--lookalikes", type=int, default=2, choices=[0, 1, 2],
                    help="so nguoi gay nham chon la nguoi GIONG muc tieu nhat (con lai ngau nhien)")
    a = ap.parse_args()

    market = load_market(os.path.expanduser(a.market))
    ids_all = sorted(market)
    # Feature trung binh moi nguoi (mang RGB) de chon nguoi "giong nhat"
    cache = os.path.join(tempfile.gettempdir(), f"sim_identity_means_{len(ids_all)}.npy")
    if os.path.exists(cache):
        means = np.load(cache)
    else:
        means = []
        for pid in ids_all:
            f = embed([cv2.imread(p) for p in market[pid][:8]], rgb=True)
            f = f.mean(0)
            means.append(f / np.linalg.norm(f))
        means = np.stack(means)
        np.save(cache, means)
    old_dir = load_old_package(a.old_rev)

    jobs = []
    for fps in [float(x) for x in a.fps.split(",")]:
        for v in a.variants.split(","):
            for s in range(a.runs):
                jobs.append((1000 + s, v, fps, market, a.cam_h, a.p_switch, old_dir, ids_all, means, a.lookalikes, a.hfov, a.cam_pitch, a.projection))
    # spawn: tien trinh cha da dung torch (OpenMP) -> fork de treo tien trinh con
    if a.jobs <= 1:
        out = [run_one(j) for j in jobs]
    else:
        with mp.get_context("spawn").Pool(a.jobs) as pool:
            out = pool.map(run_one, jobs)

    print(f"\ncam_h={a.cam_h} m, ngua {a.cam_pitch} do, FOV ngang {a.hfov} do ({a.projection}), p_switch={a.p_switch}, nguoi giong={a.lookalikes}/4, {a.runs} lan/bien the (cung bo nguoi, cung kich ban)\n")
    print(f"{'bien the':10s} {'fps':>4s} {'nham_s TB':>9s} {'nham_s max':>10s} {'lan co nham':>11s} {'thieu_%':>8s} "
          f"{'lai_s TV':>8s} {'sau_che_s TV':>12s} {'rot khoa':>8s}")
    for fps in [float(x) for x in a.fps.split(",")]:
        for v in a.variants.split(","):
            rows = [st for (vv, s, st), j in zip(out, jobs) if vv == v and j[2] == fps]
            w = np.array([r["wrong_s"] for r in rows])
            print(f"{v:10s} {fps:4.0f} {w.mean():9.2f} {w.max():10.2f} {int((w > 0).sum()):>6d}/{len(rows):<4d} "
                  f"{100 * np.mean([r['miss'] for r in rows]):8.1f} {np.median([r['reacq'] for r in rows]):8.2f} "
                  f"{np.median([r['after_block'] for r in rows]):12.2f} {np.mean([r['drops'] for r in rows]):8.1f}"
                  f"   (enroll TB {np.mean([r['enroll_s'] for r in rows]):.0f} s, xong {sum(r['enrolled'] for r in rows)}/{len(rows)})")
    if a.verbose:
        for (v, sd, st), j in zip(out, jobs):
            print(f"  {v:8s} fps={j[2]:.0f} seed={sd} enroll={st['enrolled']} nham_s={st['wrong_s']:.2f} "
                  f"thieu={100 * st['miss']:.0f}% lai={st['reacq']:.2f} sau_che={st['after_block']:.2f} rot={st['drops']}")
    print("\nnham_s: giay bao NHAM nguoi / 60 s; thieu_%: khung thay ro muc tieu ma khong bao; "
          "lai_s/sau_che_s: trung vi (99 = khong bao gio nhan lai); rot khoa: so lan mat khoa khi muc tieu thay ro >90%")


if __name__ == "__main__":
    main()
