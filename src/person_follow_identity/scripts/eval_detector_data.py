#!/usr/bin/env python3
"""eval_detector_data.py — so detector (YOLOv5n dang dung / YOLO26n) tren du lieu THAT (09/10).

Moi pha cua record_reid_data.py biet truoc SO NGUOI trong khung: nen = 0, A/B = 1, hai nguoi = 2.
Dem so nguoi moi detector tim duoc tren tung khung -> ti le khung dung so nguoi, khung SOT nguoi
(it hon), khung BAO NHAM (nhieu hon); rieng pha "_thu" (sat mep, rat gan, xa, nup mot phan, quay
lung) la cho detector yeu lo ra. Chay bang onnxruntime (venv ~/.venvs/yolo hoac sau khi cai):

  ~/.venvs/yolo/bin/python eval_detector_data.py [~/reid_data/<phien>] [--threads 2]
Model ONNX: ~/.cache/yolo_models/{yolov5n,yolo26n}_{640x480,416x320}.onnx
"""
import argparse
import json
import os
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

YDIR = Path.home() / ".cache" / "yolo_models"
EXPECT = {"none": 0, "A": 1, "B": 1, "A_left": 2, "A_right": 2}
CONFIGS = [("yolov5n_640x480", 0.40), ("yolov5n_416x320", 0.40), ("yolo26n_640x480", 0.40),
           ("yolo26n_640x480", 0.30), ("yolo26n_416x320", 0.40), ("yolo26n_416x320", 0.30)]


def letterbox(img, w, h):
    s = min(w / img.shape[1], h / img.shape[0])
    nw, nh = int(round(img.shape[1] * s)), int(round(img.shape[0] * s))
    px, py = (w - nw) // 2, (h - nh) // 2
    out = np.full((h, w, 3), 114, np.uint8)
    out[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    x = out[:, :, ::-1].astype(np.float32).transpose(2, 0, 1)[None] / 255.0
    return np.ascontiguousarray(x), s, px, py


class Det:
    def __init__(self, name, threads):
        o = ort.SessionOptions()
        o.intra_op_num_threads = threads
        o.inter_op_num_threads = 1
        self.s = ort.InferenceSession(str(YDIR / f"{name}.onnx"), o, providers=["CPUExecutionProvider"])
        self.inp = self.s.get_inputs()[0].name
        _, _, self.h, self.w = self.s.get_inputs()[0].shape
        self.v5 = name.startswith("yolov5")

    def __call__(self, img, conf, iou=0.5):
        x, s, px, py = letterbox(img, self.w, self.h)
        y = self.s.run(None, {self.inp: x})[0][0]
        if self.v5:                       # (N, 85): cx cy w h obj cls...
            score = y[:, 4] * y[:, 5]
            box = y[:, :4]
        else:                             # (84, N): cx cy w h cls...
            y = y.T
            score = y[:, 4]
            box = y[:, :4]
        k = score >= conf
        box, score = box[k], score[k]
        if len(box) == 0:
            return []
        xywh = np.stack([(box[:, 0] - box[:, 2] / 2 - px) / s, (box[:, 1] - box[:, 3] / 2 - py) / s,
                         box[:, 2] / s, box[:, 3] / s], 1)
        idx = cv2.dnn.NMSBoxes(xywh.tolist(), score.tolist(), conf, iou)
        return [(xywh[i], float(score[i])) for i in np.array(idx).reshape(-1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", nargs="?", default=None)
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--every", type=int, default=2, help="lay 1 khung moi N khung (mac dinh 2)")
    args = ap.parse_args()
    root = Path.home() / "reid_data"
    sess = Path(args.session).expanduser() if args.session else sorted(p for p in root.iterdir() if p.is_dir())[-1]
    frames = [json.loads(l) for l in open(sess / "meta.jsonl")]
    frames = [f for f in frames if f["rule"] in EXPECT][::args.every]
    print(f"Phien {sess.name}: {len(frames)} khung co so nguoi biet truoc (lay 1/{args.every})")
    dets = {}
    for name, conf in CONFIGS:
        if name not in dets:
            dets[name] = Det(name, args.threads)
    print(f"\n{'detector':24s} {'ms':>5s} | {'dung so nguoi':>13s} {'SOT':>6s} {'NHAM':>6s} | {'pha kho (_thu): dung  SOT':>27s} | {'nen: nham':>9s} | conf TV")
    for name, conf in CONFIGS:
        d = dets[name]
        st = defaultdict(lambda: [0, 0, 0, 0])   # nhom -> [n, dung, sot, nham]
        confs, ts = [], []
        for f in frames:
            img = cv2.imread(str(sess / "frames" / f["file"]))
            t = time.perf_counter()
            out = d(img, conf)
            ts.append(time.perf_counter() - t)
            n = len(out)
            confs += [c for _, c in out]
            exp = EXPECT[f["rule"]]
            for g in ("tat ca", "thu" if f["phase"].endswith("_thu") else None, "nen" if exp == 0 else None):
                if g is None:
                    continue
                s = st[g]
                s[0] += 1
                s[1] += n == exp
                s[2] += n < exp
                s[3] += n > exp
        a, h, z = st["tat ca"], st["thu"], st["nen"]
        pct = lambda x, n: 100.0 * x / max(1, n)  # noqa: E731
        print(f"{name + ' @' + str(conf):24s} {1000 * np.median(ts):5.1f} | {pct(a[1], a[0]):12.1f}% {pct(a[2], a[0]):5.1f}% {pct(a[3], a[0]):5.1f}% |"
              f" {pct(h[1], h[0]):19.1f}% {pct(h[2], h[0]):5.1f}% | {pct(z[3], z[0]):8.1f}% | {np.mean(confs) if confs else 0:.2f}")


if __name__ == "__main__":
    main()
