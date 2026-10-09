#!/usr/bin/env python3
"""record_reid_data.py — ghi du lieu THAT tu camera KINGSEN de chon / chinh mang ReID (09/10).

Khong can ROS. KHONG chay cung luc voi identity node / run_full.sh (camera chi mo duoc mot noi).
Hai nguoi: A = chu (nguoi se enroll de xe bam), B = nguoi thu hai. Script tu chay lan luot cac
PHA co dem nguoc, hien huong dan tren cua so camera va doc "pha 1, pha 2..." qua loa (spd-say).

  python3 record_reid_data.py              # ~5 phut, luu vao ~/reid_data/<ngay_gio>/
  python3 record_reid_data.py --short      # ban ngan ~2.5 phut (thu nhanh)
  python3 record_reid_data.py --no-window  # khong mo cua so (chi in huong dan ra terminal)

Phim (khi cua so dang chon): SPACE = sang pha sau ngay, P = tam dung / tiep, Q hoac ESC = dung va luu.

Luu: frames/000123.jpg (moi khung xu ly, ~15 Hz nhu node), meta.jsonl (moi khung: pha, cac bbox
nguoi + track id DeepSORT + conf + nhan A/B neu biet), session.json (lich pha, cau hinh camera).
Du lieu nam NGOAI workspace -> hook sao luu khong day anh nguoi len GitHub.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")   # nhu node.py: BLAS 1 luong, khong tranh CPU voi torch

import cv2
import numpy as np
import yaml

from person_follow_identity.core import parse_camera_source
from person_follow_identity.node import LatestFrameGrabber
from person_reid_tracker.path_utils import resolve_path
from person_reid_tracker.yolo_deepsort_pipeline import YoloDeepSortPipeline

HERE = Path(__file__).resolve().parent
CFG = HERE.parents[1] / "person_follow_robot" / "config" / "identity_lock_kingsen.yaml"

# (ten, giay, nhan, huong dan). Nhan: none = khong ai; A / B = chi mot nguoi; A_left / A_right = ca hai,
# A dung ben trai / phai khung (gan nhan theo ben); free = ca hai di tu do (khong gan nhan, dung de chay
# lai toan bo logic khoa nguoi).
PHASES = [
    ("nen", 10, "none", ["KHONG AI trong khung hinh", "(ghi nen: kiem tra bao nham)"]),
    ("A_dangky", 45, "A", ["CHI NGUOI A (chu) trong khung", "di tu ~3 m lai gan ~0.8 m roi lui ra",
                           "xoay trai / phai: truoc, nghieng, sau lung"]),
    ("B_dangky", 45, "B", ["CHI NGUOI B trong khung", "di tu ~3 m lai gan ~0.8 m roi lui ra",
                           "xoay trai / phai: truoc, nghieng, sau lung"]),
    ("A_thu", 30, "A", ["CHI NGUOI A", "sat mep trai / phai, rat gan (mat dau / chan), xa 3-4 m",
                        "nup mot phan sau ghe / cua, quay lung"]),
    ("B_thu", 30, "B", ["CHI NGUOI B", "sat mep trai / phai, rat gan (mat dau / chan), xa 3-4 m",
                        "nup mot phan sau ghe / cua, quay lung"]),
    ("AB_A_trai", 30, "A_left", ["CA HAI: A ben TRAI khung, B ben PHAI", "di toi / lui, xoay nguoi, KHONG doi ben"]),
    ("AB_A_phai", 30, "A_right", ["CA HAI: A ben PHAI khung, B ben TRAI", "di toi / lui, xoay nguoi, KHONG doi ben"]),
    ("AB_tudo", 45, "free", ["CA HAI di tu do: di cheo qua nhau, che nhau,", "ra khoi khung roi quay lai"]),
]
COUNTDOWN_SEC = 6.0
EXPECT = {"none": 0, "A": 1, "B": 1, "A_left": 2, "A_right": 2, "free": None}


def say(text: str, enabled: bool) -> None:
    if not enabled or shutil.which("spd-say") is None:
        return
    try:
        subprocess.Popen(["spd-say", "-l", "vi", text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        pass


def side_label(rule: str, bbox, width: int) -> str:
    """Nhan theo pha: A/B; pha ca hai theo ben trai/phai (bo vung giua +-40 px de tranh nham)."""
    if rule in ("A", "B"):
        return rule
    if rule in ("A_left", "A_right"):
        cx = 0.5 * (bbox[0] + bbox[2])
        if abs(cx - width / 2) < 40:
            return "?"
        left = cx < width / 2
        return "A" if left == (rule == "A_left") else "B"
    return "?"


def draw(frame, tracks, title, lines, remain, rec, paused, width_scale):
    img = frame.copy()
    for t in tracks:
        x1, y1, x2, y2 = t.bbox
        col = (0, 200, 0) if t.conf > 0 else (0, 160, 255)
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        cv2.putText(img, f"id {t.track_id} {t.conf:.2f}", (x1 + 2, max(14, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    cv2.line(img, (img.shape[1] // 2, 0), (img.shape[1] // 2, img.shape[0]), (80, 80, 80), 1)
    if width_scale != 1.0:
        img = cv2.resize(img, None, fx=width_scale, fy=width_scale)
    h = img.shape[0]
    cv2.rectangle(img, (0, 0), (img.shape[1], 40 + 30 * len(lines)), (0, 0, 0), -1)
    cv2.putText(img, title, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    for i, ln in enumerate(lines):
        cv2.putText(img, ln, (10, 62 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(img, f"{remain:4.0f}s", (img.shape[1] - 120, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
    if rec:
        cv2.circle(img, (img.shape[1] - 30, 30), 12, (0, 0, 255), -1)
    if paused:
        cv2.putText(img, "TAM DUNG (P de tiep)", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    return img


def main() -> int:
    try:
        return run()
    except KeyboardInterrupt:
        print("\nCtrl-C: da dung (khung da ghi van con trong thu muc phien)")
        return 130


def run() -> int:
    sys.stdout.reconfigure(line_buffering=True)
    ap = argparse.ArgumentParser(description="Ghi du lieu ReID that tu camera (2 nguoi A/B, theo pha)")
    ap.add_argument("--out", default=str(Path.home() / "reid_data"), help="thu muc goc (mac dinh ~/reid_data)")
    ap.add_argument("--short", action="store_true", help="moi pha ngan mot nua (~2.5 phut)")
    ap.add_argument("--no-window", action="store_true")
    ap.add_argument("--no-voice", action="store_true")
    ap.add_argument("--scale", type=float, default=1.5, help="phong to cua so cho de nhin tu xa (mac dinh 1.5)")
    ap.add_argument("--hz", type=float, default=15.0, help="tan so xu ly / luu khung (nhu node: 15)")
    ap.add_argument("--no-wait", action="store_true", help="khong cho SPACE, bat dau ngay sau 3 s")
    args = ap.parse_args()
    voice = not args.no_voice

    p = yaml.safe_load(open(CFG))["identity_lock_node"]["ros__parameters"]
    source = parse_camera_source(p["camera_source"])
    dev = f"/dev/video{source}" if isinstance(source, int) else str(source)
    if not os.path.exists(dev):
        print(f"Khong co camera {dev}")
        return 1
    if shutil.which("fuser") and subprocess.run(["fuser", "-s", os.path.realpath(dev)]).returncode == 0:
        print(f"Camera {dev} dang bi tien trinh khac giu (identity node / run_full.sh dang chay?) — tat truoc.")
        return 1

    ctrls = str(p.get("camera_v4l2_controls", "")).strip()
    if ctrls and shutil.which("v4l2-ctl"):
        subprocess.run(["v4l2-ctl", "-d", dev, "-c", ctrls], capture_output=True, timeout=5)
    cap = cv2.VideoCapture(source)
    fourcc = str(p.get("camera_fourcc", "")).strip()
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*(fourcc + "    ")[:4]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(p["frame_width"]))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(p["frame_height"]))
    cap.set(cv2.CAP_PROP_FPS, float(p.get("camera_fps", 25.0)))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print(f"Khong mo duoc camera {dev}")
        return 1
    mode = str(p.get("camera_exposure_mode", "auto")).lower()
    exp_ms = float(p.get("camera_exposure_ms", 30.0))
    if mode in ("fixed_fps", "manual"):
        if not (cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) and cap.set(cv2.CAP_PROP_EXPOSURE, exp_ms * 10.0)):
            mode = "auto"
    if mode == "auto":
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
    grab = LatestFrameGrabber(cap, exposure_mode=mode, exposure_ms=exp_ms,
                              exposure_min_ms=float(p.get("camera_exposure_min_ms", 1.0)),
                              exposure_max_ms=float(p.get("camera_exposure_max_ms", 30.0)),
                              exposure_step_ms=float(p.get("camera_exposure_step_ms", 10.0)),
                              brightness_target=float(p.get("camera_brightness_target", 120.0)))

    print("Nap YOLOv5n + DeepSORT (nhu node)...")
    pipe = YoloDeepSortPipeline(
        model_weights=resolve_path(str(p.get("model_weights", "")), "yolov5n.pt"),
        deepsort_ckpt=resolve_path(str(p.get("deepsort_ckpt", "")), "ckpt.t7"),
        device="cpu", use_cuda=False, img_size=int(p.get("img_size", 640)),
        conf_thres=float(p.get("det_conf_thres", 0.40)), iou_thres=float(p.get("det_iou_thres", 0.50)),
        person_class_id=int(p.get("person_class_id", 0)), deepsort_max_age=int(p.get("deepsort_max_age", 30)),
        deepsort_n_init=int(p.get("deepsort_n_init", 4)), deepsort_max_dist=float(p.get("deepsort_max_dist", 0.18)),
        rect_inference=bool(p.get("rect_inference", True)))

    sess = Path(args.out).expanduser() / time.strftime("%Y%m%d_%H%M%S")
    (sess / "frames").mkdir(parents=True, exist_ok=True)
    phases = [(n, (s / 2.0 if args.short else float(s)), r, ln) for n, s, r, ln in PHASES]
    meta = open(sess / "meta.jsonl", "w", encoding="utf-8")
    info = {"camera": dev, "fourcc": fourcc, "exposure_mode": mode, "hz": args.hz, "phases": [], "start": time.time()}
    win = None if args.no_window else "Ghi du lieu ReID (SPACE sang pha, P tam dung, Q dung)"
    if win:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    print(f"\nLuu vao {sess}")
    # Dung cho: hai nguoi vao vi tri; bam SPACE trong cua so (hoac Enter neu --no-window) moi bat dau.
    # Pha 1 la 'nen' (khong ai trong khung) + 6 s dem nguoc: du thoi gian bam xong roi ra khoi khung.
    wait_key = not args.no_wait
    if wait_key:
        print("SAN SANG — bam SPACE trong cua so camera de bat dau (Q de thoat)" if win else "SAN SANG — bam Enter de bat dau")
        say("San sang. Bam phim cach de bat dau", voice)
    t_wait = time.time()
    while True:
        frame, _, _ = grab.latest()
        if win:
            if frame is not None:
                cv2.imshow(win, draw(frame, [], "SAN SANG: bam SPACE de bat dau" if wait_key else "CHO CAMERA...",
                                     ["Q = thoat"] if wait_key else [], 0, False, False, args.scale))
            k = cv2.waitKey(30) & 0xFF
            if k in (ord("q"), 27):
                grab.stop(); cap.release(); cv2.destroyAllWindows()
                meta.close(); shutil.rmtree(sess, ignore_errors=True)
                print("Thoat, chua ghi gi.")
                return 0
            if (wait_key and k == ord(" ")) or (not wait_key and time.time() - t_wait > 3.0):
                break
        else:
            if wait_key:
                input()
            else:
                time.sleep(3.0)
            break

    idx = 0
    last_seq = -1
    stats = {}
    stop = False
    paused = False
    pause_t0 = 0.0
    period = 1.0 / max(1.0, args.hz)
    for pi, (name, dur, rule, lines) in enumerate(phases):
        if stop:
            break
        title_cd = f"CHUAN BI pha {pi + 1}/{len(phases)}: {name}"
        print(f"\n=== {title_cd} ({dur:.0f} s) ===")
        for ln in lines:
            print("   " + ln)
        say(f"Chuan bi pha {pi + 1}", voice)
        t_cd = time.time()
        skip = False
        while time.time() - t_cd < COUNTDOWN_SEC and not skip and not stop:
            frame, _, _ = grab.latest()
            if win and frame is not None:
                cv2.imshow(win, draw(frame, [], title_cd, lines, COUNTDOWN_SEC - (time.time() - t_cd), False, False, args.scale))
                k = cv2.waitKey(30) & 0xFF
                skip = k == ord(" ")
                stop = k in (ord("q"), 27)
            else:
                time.sleep(0.05)
        if stop:
            break
        say("Bat dau", voice)
        t0 = time.time()
        ph = {"name": name, "rule": rule, "start": t0, "frames": 0, "frames_expected": 0, "dets": 0}
        paused_total = 0.0
        t_last = 0.0
        while not stop:
            now = time.time()
            elapsed = now - t0 - paused_total
            if elapsed >= dur:
                break
            frame, stamp, seq = grab.latest()
            if frame is None or seq == last_seq or (now - t_last) < period or paused:
                if win and frame is not None and paused:
                    cv2.imshow(win, draw(frame, [], f"pha {pi + 1}: {name}", lines, dur - elapsed, False, True, args.scale))
                k = (cv2.waitKey(5) & 0xFF) if win else 255
                if k == ord("p"):
                    if paused:
                        paused_total += time.time() - pause_t0
                    else:
                        pause_t0 = time.time()
                    paused = not paused
                elif k == ord(" "):
                    break
                elif k in (ord("q"), 27):
                    stop = True
                if not win:
                    time.sleep(0.003)
                continue
            last_seq, t_last = seq, now
            tracks = pipe.detect_and_track(frame)
            fn = f"{idx:06d}.jpg"
            cv2.imwrite(str(sess / "frames" / fn), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            dets = [{"bbox": [int(v) for v in t.bbox], "tid": int(t.track_id), "conf": round(float(t.conf), 3),
                     "label": side_label(rule, t.bbox, frame.shape[1]) if t.conf > 0 else "?"} for t in tracks]
            meta.write(json.dumps({"i": idx, "file": fn, "t": round(stamp, 4), "phase": name, "rule": rule,
                                   "exp_ms": getattr(grab, "exposure_ms", None),
                                   "bright": round(float(getattr(grab, "brightness", float("nan"))), 1),
                                   "dets": dets}) + "\n")
            idx += 1
            n_real = sum(1 for d in dets if d["conf"] > 0)
            ph["frames"] += 1
            ph["dets"] += n_real
            if EXPECT[rule] is not None and n_real == EXPECT[rule]:
                ph["frames_expected"] += 1
            if win:
                cv2.imshow(win, draw(frame, tracks, f"pha {pi + 1}/{len(phases)}: {name}", lines, dur - elapsed, True, False, args.scale))
                k = cv2.waitKey(1) & 0xFF
                if k == ord(" "):
                    break
                if k == ord("p"):
                    pause_t0 = time.time()
                    paused = True
                if k in (ord("q"), 27):
                    stop = True
        ph["end"] = time.time()
        info["phases"].append(ph)
        stats[name] = ph
        exp = EXPECT[rule]
        print(f"    -> {ph['frames']} khung, {ph['dets']} nguoi phat hien"
              + (f", {ph['frames_expected']}/{ph['frames']} khung dung {exp} nguoi" if exp is not None else ""))
    say("Xong", voice)
    meta.close()
    info["end"] = time.time()
    info["n_frames"] = idx
    json.dump(info, open(sess / "session.json", "w"), indent=1)
    grab.stop()
    cap.release()
    if win:
        cv2.destroyAllWindows()
    size_mb = sum(f.stat().st_size for f in (sess / "frames").glob("*.jpg")) / 1e6
    print(f"\nXONG: {idx} khung ({size_mb:.0f} MB) -> {sess}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
