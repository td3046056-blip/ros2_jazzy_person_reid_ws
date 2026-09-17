#!/usr/bin/env python3
from __future__ import annotations

import sys
import cv2

source = sys.argv[1] if len(sys.argv) > 1 else "0"
cap_source = int(source) if str(source).isdigit() else source
cap = cv2.VideoCapture(cap_source, cv2.CAP_V4L2 if not str(source).isdigit() else cv2.CAP_ANY)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print(f"ERROR: cannot open camera source {source}")
    sys.exit(1)

print(f"Opened camera source {source}. Press q to quit.")
while True:
    ok, frame = cap.read()
    if not ok or frame is None:
        print("ERROR: failed to read frame")
        break
    cv2.imshow("raw_usb_camera_test", frame)
    if (cv2.waitKey(1) & 0xFF) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
