from __future__ import annotations

import argparse
import json
import time

import cv2

from .camera_reid_core import CameraReIDCore, parse_camera_source
from .path_utils import resolve_path


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Non-ROS camera test for YOLOv5 + DeepSORT + target ReID recovery")
    p.add_argument("--camera-source", default="0")
    p.add_argument("--model-weights", default="")
    p.add_argument("--deepsort-ckpt", default="")
    p.add_argument("--device", default="cpu", choices=["cpu", "cuda"])
    p.add_argument("--use-cuda", action="store_true")
    p.add_argument("--frame-width", type=int, default=640)
    p.add_argument("--frame-height", type=int, default=480)
    p.add_argument("--det-conf-thres", type=float, default=0.5)
    p.add_argument("--det-iou-thres", type=float, default=0.5)
    p.add_argument("--reid-threshold", type=float, default=0.72)
    p.add_argument("--max-lost-sec", type=float, default=5.0)
    p.add_argument("--draw-all-tracks", action="store_true")
    return p


def main() -> None:
    args = build_arg_parser().parse_args()
    params = vars(args)
    params["model_weights"] = resolve_path(params["model_weights"], "yolov5n.pt")
    params["deepsort_ckpt"] = resolve_path(params["deepsort_ckpt"], "ckpt.t7")
    params.setdefault("img_size", 640)
    params.setdefault("person_class_id", 0)
    params.setdefault("deepsort_max_age", 70)
    params.setdefault("deepsort_n_init", 3)
    params.setdefault("deepsort_max_dist", 0.2)
    params.setdefault("min_stable_frames", 3)
    params.setdefault("initial_select_mode", "largest_box")
    params.setdefault("reid_gallery_size", 30)
    params.setdefault("reid_margin", 0.06)
    params.setdefault("spatial_gate_px", 280.0)
    params.setdefault("update_gallery_every_n_frames", 3)
    params.setdefault("camera_fov_deg", 62.0)

    core = CameraReIDCore(params)
    cap = cv2.VideoCapture(parse_camera_source(args.camera_source))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.frame_height)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera source: {args.camera_source}")

    print("Press r to reset target, q to quit.")
    last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame")
            break
        debug, payload, _, _ = core.process_frame(frame)
        now = time.time()
        if now - last_print > 0.5:
            print(json.dumps(payload, ensure_ascii=False))
            last_print = now
        cv2.imshow("camera_reid_demo", debug)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):
            core.reset_target()
            print("Target reset")
        elif key == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
