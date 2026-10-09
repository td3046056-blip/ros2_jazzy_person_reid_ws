#!/usr/bin/env python3
"""replay_identity.py — chay lai TOAN BO logic camera (detector + DeepSORT + identity_lock_manager)
tren du lieu ghi bang record_reid_data.py, theo dung dau thoi gian da ghi (09/10).

Enroll A trong pha "A_dangky" (nhu bam /person_reid/start_enroll), roi do KHOA NHAM:
  - pha chi co B (B_dangky, B_thu): target_found = KHOA NHAM (chu khong co trong khung)
  - pha hai nguoi (AB_A_trai / AB_A_phai): bbox muc tieu nam ben cua B = KHOA NHAM
  - pha chi co A: target_found = dung (ti le = do bam duoc)
So cau hinh cu / moi bang --set (ghi de tham so identity_lock_kingsen.yaml) hoac --override file.yaml.

  python3 replay_identity.py [~/reid_data/<phien>]                          # cau hinh dang dung
  python3 replay_identity.py --set detector_onnx=yolo26n_416x320.onnx --set reid_onnx=osnet_x0_25.onnx \\
        --override osnet_thresholds.yaml --name moi
"""
import argparse
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import cv2
import numpy as np
import yaml

from person_follow_identity.core import IdentityFollowCore

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_reid_data import color_label  # noqa: E402

HERE = Path(__file__).resolve().parent
CFG = HERE.parents[1] / "person_follow_robot" / "config" / "identity_lock_kingsen.yaml"


def parse_value(v: str):
    try:
        return yaml.safe_load(v)
    except Exception:
        return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", nargs="?", default=None)
    ap.add_argument("--set", action="append", default=[], help="key=value ghi de tham so")
    ap.add_argument("--override", default=None, help="file yaml {key: value} ghi de tham so")
    ap.add_argument("--name", default="cau_hinh")
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--log", default=None, help="ghi tung khung (jsonl) de xem lai")
    ap.add_argument("--judge", default="color", choices=["color", "phase"],
                    help="color = cham bbox muc tieu theo mau ao/quan (A ao sang quan sam), khong ro thi theo pha")
    args = ap.parse_args()
    import torch
    torch.set_num_threads(args.threads)

    root = Path.home() / "reid_data"
    sess = Path(args.session).expanduser() if args.session else sorted(p for p in root.iterdir() if p.is_dir())[-1]
    params = dict(yaml.safe_load(open(CFG))["identity_lock_node"]["ros__parameters"])
    if args.override:
        params.update(yaml.safe_load(open(args.override)) or {})
    for kv in args.set:
        k, v = kv.split("=", 1)
        params[k] = parse_value(v)
    params["ort_threads"] = args.threads
    core = IdentityFollowCore(params)

    frames = [json.loads(l) for l in open(sess / "meta.jsonl")]
    st = defaultdict(lambda: {"n": 0, "found": 0, "wrong": 0, "right": 0, "episodes": 0, "wrong_streak": 0.0})
    enrolled = False
    in_wrong, wrong_t0, wrong_last, wrong_phase = False, 0.0, 0.0, None
    prev_phase = None
    logf = open(args.log, "w") if args.log else None
    for f in frames:
        t, ph, rule = f["t"], f["phase"], f["rule"]
        if ph == "A_dangky" and prev_phase != "A_dangky":
            core.identity.start_enrollment(now=t)
        if prev_phase == "A_dangky" and ph != "A_dangky" and core.identity.enrolling:
            core.identity.finish_enrollment()           # ban --short: du mau nhung chua du 30 s
        prev_phase = ph
        img = cv2.imread(str(sess / "frames" / f["file"]))
        _, payload, result, tracks = core.process_frame(img, capture_ts=t, draw_debug=False)
        found = bool(payload.get("target_found"))
        bbox = payload.get("bbox")
        verdict = "-"
        if in_wrong and ph != wrong_phase:            # dot nham khong keo qua ranh gioi pha (co dem nguoc giua pha)
            st[wrong_phase]["wrong_streak"] = max(st[wrong_phase]["wrong_streak"], wrong_last - wrong_t0)
            in_wrong = False
        judged = rule in ("A", "B", "none", "A_left", "A_right") or (rule == "free" and args.judge == "color")
        if judged and core.identity.identity_ready and ph != "A_dangky":
            s = st[ph]
            s["n"] += 1
            s["found"] += found
            wrong = False
            col = "?"
            if found and bbox and args.judge == "color":
                col = color_label(cv2.cvtColor(img, cv2.COLOR_BGR2LAB), [int(v) for v in bbox])
            if found and col in ("A", "B"):
                wrong = col == "B"
                s["right"] += not wrong
            elif found and rule in ("B", "none"):
                wrong = True
            elif found and rule in ("A_left", "A_right") and bbox:
                cx = 0.5 * (bbox[0] + bbox[2])
                if abs(cx - img.shape[1] / 2) >= 40:
                    on_left = cx < img.shape[1] / 2
                    wrong = on_left != (rule == "A_left")
                    s["right"] += not wrong
            elif found and rule == "A":
                s["right"] += 1
            s["wrong"] += wrong
            if wrong and not in_wrong:
                in_wrong, wrong_t0, wrong_phase = True, t, ph
                s["episodes"] += 1
            if wrong:
                wrong_last = t
            if in_wrong and not wrong:
                st[ph]["wrong_streak"] = max(st[ph]["wrong_streak"], wrong_last - wrong_t0)
                in_wrong = False
            verdict = "NHAM" if wrong else ("dung" if found else "khong thay")
        if logf:
            logf.write(json.dumps({"t": t, "phase": ph, "status": payload.get("status"), "found": found, "bbox": bbox,
                                   "verdict": verdict, "reason": (payload.get("reason") or "")[:160]}) + "\n")
    if logf:
        logf.close()
    print(f"\n=== {args.name}: {sess.name} | detector={params.get('detector_onnx') or 'torch yolov5n'} "
          f"reid={params.get('reid_onnx') or 'ckpt.t7'} ===")
    print(f"{'pha':12s} {'khung':>6s} {'thay chu':>9s} {'KHOA NHAM':>10s} {'so lan':>7s} {'lau nhat':>9s}")
    tot_wrong = tot_n = 0
    for ph in ("B_dangky", "A_thu", "B_thu", "AB_A_trai", "AB_A_phai", "AB_tudo", "nen"):
        if ph not in st:
            continue
        s = st[ph]
        tot_wrong += s["wrong"]
        tot_n += s["n"]
        print(f"{ph:12s} {s['n']:6d} {100 * s['found'] / max(1, s['n']):8.1f}% {100 * s['wrong'] / max(1, s['n']):9.1f}% "
              f"{s['episodes']:7d} {s['wrong_streak']:8.1f}s")
    print(f"TONG khoa nham: {tot_wrong}/{tot_n} khung ({100 * tot_wrong / max(1, tot_n):.1f}%)")


if __name__ == "__main__":
    sys.exit(main())
