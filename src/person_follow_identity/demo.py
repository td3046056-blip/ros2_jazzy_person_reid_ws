from __future__ import annotations

import argparse
import json
import time

import cv2

from .core import IdentityFollowCore, parse_camera_source


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Strict identity-lock camera demo with enrollment")
    p.add_argument("--camera-source", default="0")
    p.add_argument("--frame-width", type=int, default=640)
    p.add_argument("--frame-height", type=int, default=480)
    p.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda", "gpu"])
    p.add_argument("--use-cuda", action="store_true")
    p.add_argument("--enroll-seconds", type=float, default=30.0)
    p.add_argument("--enroll-min-samples", type=int, default=80)
    p.add_argument("--draw-all-tracks", action="store_true")
    p.add_argument("--auto-enroll-on-start", action="store_true")
    return p


def main() -> None:
    args = build_arg_parser().parse_args()
    params = vars(args)
    params.setdefault("show_window", True)
    core = IdentityFollowCore(params)
    print(core.device_note)

    cap = cv2.VideoCapture(parse_camera_source(args.camera_source))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.frame_height)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera source: {args.camera_source}")

    print("Keys: e=start enrollment, f=finish enrollment, r=reset identity, q=quit")
    last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Failed to read frame")
            break
        debug, payload, _, _ = core.process_frame(frame)
        now = time.time()
        if now - last_print > 0.5:
            print(json.dumps(payload, ensure_ascii=False))
            last_print = now
        cv2.imshow("identity_lock_demo", debug)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("e"):
            core.start_enrollment()
            print("Enrollment started")
        elif key == ord("f"):
            print(f"Finish enrollment: identity_ready={core.finish_enrollment()}")
        elif key == ord("r"):
            core.reset()
            print("Identity reset")
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
