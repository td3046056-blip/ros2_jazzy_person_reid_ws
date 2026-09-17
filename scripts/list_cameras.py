#!/usr/bin/env python3
from __future__ import annotations

import glob
import os
import subprocess
from pathlib import Path

import cv2


def run(cmd):
    try:
        return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT).strip()
    except Exception:
        return ""


print("=== /dev/v4l/by-id ===")
by_id = sorted(glob.glob("/dev/v4l/by-id/*"))
if by_id:
    for p in by_id:
        try:
            print(f"{p} -> {os.path.realpath(p)}")
        except Exception:
            print(p)
else:
    print("No /dev/v4l/by-id entries found")

print("\n=== v4l2-ctl --list-devices ===")
out = run(["v4l2-ctl", "--list-devices"])
print(out if out else "v4l2-ctl not installed or no output. Install with: sudo apt install v4l-utils")

print("\n=== OpenCV probe ===")
for dev in sorted(glob.glob("/dev/video*")):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    opened = cap.isOpened()
    ok = False
    shape = ""
    if opened:
        ok, frame = cap.read()
        if ok and frame is not None:
            shape = f" frame={frame.shape[1]}x{frame.shape[0]}"
    cap.release()
    print(f"{dev}: opened={opened} read={ok}{shape}")

print("\nUse the USB camera that shows opened=True and read=True.")
print("Example: ./scripts/run_usb_camera_reid.sh /dev/video2")
