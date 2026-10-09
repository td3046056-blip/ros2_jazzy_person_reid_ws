#!/usr/bin/env python3
"""eval_reid_data.py — so cac mang ReID tren du lieu THAT ghi bang record_reid_data.py (09/10).

Mo phong dung cach identity_lock_manager cham diem: kho anh chu lay tu pha "A_dangky" (moi 3 khung
1 mau, toi da 160 — nhu enroll), diem = trung binh 5 do giong cao nhat (topk_mean, k = 5). So:
  - KHONG biet nguoi la:  diem voi kho chu cua A (thu tren pha A_thu + A trong pha hai nguoi)
                          so voi cua B (B_thu + B trong pha hai nguoi)  -> AUC, TPR khi FPR 1% / 0.1%
  - CO kho nguoi la B (hoc tu pha B_dangky, nhu gallery am): diem A - diem B -> AUC ...
  - tach theo dieu kien: hai nguoi cung khung, nguoi bi cat mep anh, nguoi o xa (bbox thap)
Mang: deepsort (ckpt.t7 dang dung) + OSNet hoc tren MSMT17 (trong ~/.cache/reid_models/*.pth,
ma nguon torchreid trong third_party). Chay bang Python cua ROS (torch), khong can onnxruntime.

  python3 eval_reid_data.py                    # phien moi nhat trong ~/reid_data
  python3 eval_reid_data.py ~/reid_data/<phien> --models deepsort,osnet_x0_25
Dac trung duoc luu lai trong <phien>/emb_<mang>.npz (chay lai nhanh).
"""
import argparse
import importlib.util
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import cv2
import numpy as np
import torch

WS = Path(__file__).resolve().parents[3]
TR = WS / "third_party" / "deep-person-reid-master" / "torchreid" / "models"
WDIR = Path.home() / ".cache" / "reid_models"
ALL = ["deepsort", "osnet_x0_25", "osnet_x0_5", "osnet_x0_75", "osnet_x1_0", "osnet_ain_x1_0"]
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def load_osnet(name):
    modname = "osnet_ain" if name.startswith("osnet_ain") else "osnet"
    spec = importlib.util.spec_from_file_location("tr_" + modname, TR / f"{modname}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    m = getattr(mod, name)(num_classes=1000, pretrained=False, loss="softmax")
    sd = torch.load(WDIR / f"{name}.pth", map_location="cpu", weights_only=False)
    sd = sd.get("state_dict", sd)
    sd = {(k[7:] if k.startswith("module.") else k): v for k, v in sd.items()}
    sd = {k: v for k, v in sd.items() if not k.startswith("classifier")}
    m.load_state_dict(sd, strict=False)
    return m.eval()


class Embedder:
    def __init__(self, name):
        self.name = name
        if name == "deepsort":
            sys.path.insert(0, str(WS / "src" / "person_reid_tracker"))
            from tracking_module.deep_sort.deep.feature_extractor import Extractor
            self.ext = Extractor(str(WS / "src" / "person_reid_tracker" / "model_assets" / "ckpt.t7"), use_cuda=False)
        else:
            self.net = load_osnet(name)

    def __call__(self, crops):
        if self.name == "deepsort":
            f = self.ext(crops)
        else:
            x = np.empty((len(crops), 3, 256, 128), np.float32)
            for i, c in enumerate(crops):
                im = cv2.resize(c, (128, 256))[:, :, ::-1].astype(np.float32) / 255.0
                x[i] = ((im - MEAN) / STD).transpose(2, 0, 1)
            with torch.no_grad():
                f = self.net(torch.from_numpy(x)).numpy()
        f = np.asarray(f, np.float32)
        return f / np.maximum(1e-9, np.linalg.norm(f, axis=1, keepdims=True))


def load_items(sess, min_conf, min_area):
    items = []
    for line in open(sess / "meta.jsonl"):
        m = json.loads(line)
        for d in m["dets"]:
            x1, y1, x2, y2 = d["bbox"]
            if d["conf"] < min_conf or d["label"] not in ("A", "B") or (x2 - x1) * (y2 - y1) < min_area:
                continue
            items.append({"i": m["i"], "file": m["file"], "phase": m["phase"], "label": d["label"], "bbox": d["bbox"],
                          "tid": d["tid"], "cut": y1 <= 2 or y2 >= 477, "edge": x1 <= 2 or x2 >= 637,
                          "far": (y2 - y1) < 150, "pair": m["phase"].startswith("AB_")})
    return items


def embed_all(sess, name, items, batch=32):
    cache = sess / f"emb_{name}.npz"
    keys = np.array([f"{it['i']}:{it['bbox']}" for it in items])
    if cache.exists():
        z = np.load(cache, allow_pickle=False)
        if len(z["keys"]) == len(keys) and (z["keys"] == keys).all():
            return z["emb"], float(z["ms"])
    emb = Embedder(name)
    out, crops, t_total = [], [], 0.0
    imgs = {}
    for it in items:
        if it["file"] not in imgs:
            imgs = {it["file"]: cv2.imread(str(sess / "frames" / it["file"]))}
        x1, y1, x2, y2 = it["bbox"]
        crops.append(imgs[it["file"]][y1:y2, x1:x2])
        if len(crops) == batch:
            t = time.perf_counter(); out.append(emb(crops)); t_total += time.perf_counter() - t
            crops = []
    if crops:
        t = time.perf_counter(); out.append(emb(crops)); t_total += time.perf_counter() - t
    E = np.concatenate(out) if out else np.zeros((0, 1), np.float32)
    ms = 1000 * t_total / max(1, len(items))
    np.savez(cache, emb=E, keys=keys, ms=ms)
    return E, ms


def topk_mean(S, k=5):
    k = min(k, S.shape[1])
    return np.sort(S, axis=1)[:, -k:].mean(axis=1)


def roc_stats(pos, neg):
    """AUC, TPR khi FPR 1% / 0.1%, d'."""
    pos, neg = np.asarray(pos), np.asarray(neg)
    if len(pos) == 0 or len(neg) == 0:
        return None
    allv = np.concatenate([pos, neg])
    ranks = allv.argsort().argsort() + 1.0
    auc = (ranks[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg))
    t1 = np.quantile(neg, 0.99)
    t01 = np.quantile(neg, 0.999)
    dprime = (pos.mean() - neg.mean()) / np.sqrt(0.5 * (pos.var() + neg.var()) + 1e-12)
    return auc, float((pos > t1).mean()), float((pos > t01).mean()), dprime


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", nargs="?", default=None)
    ap.add_argument("--models", default=",".join(ALL))
    ap.add_argument("--min-conf", type=float, default=0.5)
    ap.add_argument("--min-area", type=float, default=1800)
    ap.add_argument("--threads", type=int, default=2)
    args = ap.parse_args()
    torch.set_num_threads(args.threads)
    root = Path.home() / "reid_data"
    sess = Path(args.session).expanduser() if args.session else sorted(p for p in root.iterdir() if p.is_dir())[-1]
    items = load_items(sess, args.min_conf, args.min_area)
    lab = np.array([it["label"] for it in items])
    ph = np.array([it["phase"] for it in items])
    print(f"Phien {sess.name}: {len(items)} crop co nhan (A {int((lab == 'A').sum())}, B {int((lab == 'B').sum())})")
    gA = np.where(ph == "A_dangky")[0][::3][:160]
    gB = np.where(ph == "B_dangky")[0][::3][:160]
    pA = np.where((lab == "A") & (ph != "A_dangky"))[0]
    pB = np.where((lab == "B") & (ph != "B_dangky"))[0]
    print(f"Kho chu A {len(gA)} mau, kho nguoi la B {len(gB)} mau; thu: A {len(pA)}, B {len(pB)}")
    cond = {"hai nguoi cung khung": np.array([it["pair"] for it in items]),
            "bi cat mep tren/duoi": np.array([it["cut"] for it in items]),
            "o xa (bbox < 150 px)": np.array([it["far"] for it in items])}
    print(f"\n{'mang':16s} {'ms/crop':>7s} | {'chi kho chu: AUC  TPR@1%  TPR@.1%   d':>36s} | {'chu - nguoi la: AUC  TPR@1%  TPR@.1%':>38s} | A~chu p10  B~chu p90")
    res = {}
    for name in args.models.split(","):
        E, ms = embed_all(sess, name, items)
        sA = topk_mean(E[pA] @ E[gA].T)
        sB = topk_mean(E[pB] @ E[gA].T)
        mA = sA - topk_mean(E[pA] @ E[gB].T)
        mB = sB - topk_mean(E[pB] @ E[gB].T)
        r1, r2 = roc_stats(sA, sB), roc_stats(mA, mB)
        res[name] = (sA, sB, mA, mB)
        print(f"{name:16s} {ms:7.1f} | {r1[0]:16.4f} {r1[1]:7.3f} {r1[2]:7.3f} {r1[3]:5.2f} | {r2[0]:17.4f} {r2[1]:7.3f} {r2[2]:7.3f}  | "
              f"{np.quantile(sA, 0.10):8.3f} {np.quantile(sB, 0.90):9.3f}")
    print("\nTheo dieu kien (AUC 'chi kho chu' / 'chu - nguoi la'):")
    for cname, cm in cond.items():
        line = f"  {cname:24s}"
        for name, (sA, sB, mA, mB) in res.items():
            a, b = cm[pA], cm[pB]
            if a.sum() < 5 or b.sum() < 5:
                line += f" {name}: it mau |"
                continue
            r1, r2 = roc_stats(sA[a], sB[b]), roc_stats(mA[a], mB[b])
            line += f" {name.replace('osnet_', '')}: {r1[0]:.3f}/{r2[0]:.3f} |"
        print(line)


if __name__ == "__main__":
    main()
