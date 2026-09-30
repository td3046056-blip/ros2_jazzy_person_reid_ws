#!/usr/bin/env python3
"""
measure_fov.py — do FOV NGANG that cua camera bang thuoc day (khong can ban co).

Vi sao can: KINGSEN o 640x480 la anh CAT GIUA 1440x1080 cua cam bien (do bang so khop anh
30/09), nen FOV hep hon che do 1920x1080. Neu 62 do la thong so nha san xuat cho 1080p thi
o 640x480 chi con ~43-49 do -> camera_angle_deg bao lon hon that toi ~33% (nguoi o mep
khung that 24 do bi bao 31 do).

Cach do (2 phut):
  1. KHONG chay identity_lock_node cung luc (hai tien trinh khong mo chung camera duoc).
  2. Dat camera vuong goc voi mot buc tuong phang, cach tuong dung D met (vd 1.00 m, do tu
     mat ong kinh toi tuong).
  3. Chay script: cua so hien anh voi 2 vach do o mep trai/phai va 1 vach giua.
  4. Dan 2 mau giay (hoac dat 2 vat) len tuong sao cho chung vua cham 2 vach do.
  5. Do khoang cach W (m) giua 2 mau giay, nhap D va W khi script hoi (bam q de thoat cua so).

  python3 measure_fov.py --source /dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0
  python3 measure_fov.py --distance 1.0 --width 0.86      # chi tinh, khong mo camera

Ket qua: camera_fov_deg (dan vao identity_lock_kingsen.yaml) va FOV doc suy ra (diem anh vuong).
"""
from __future__ import annotations

import argparse
import math

import cv2


def report(d: float, w: float, width_px: int = 640, height_px: int = 480) -> None:
    hfov = 2.0 * math.degrees(math.atan(0.5 * w / d))
    fx = (0.5 * width_px) / math.tan(math.radians(0.5 * hfov))
    vfov = 2.0 * math.degrees(math.atan(0.5 * height_px / fx))
    print("=" * 60)
    print(f"D = {d:.3f} m, W = {w:.3f} m")
    print(f"FOV ngang = {hfov:.1f} do   (fx = {fx:.0f} px)")
    print(f"FOV doc   = {vfov:.1f} do   (diem anh vuong, {width_px}x{height_px})")
    print("Dan vao identity_lock_kingsen.yaml:")
    print(f"    camera_fov_deg: {hfov:.1f}")
    print("=" * 60)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="/dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0")
    ap.add_argument("--distance", type=float, default=None, help="D: camera toi tuong (m)")
    ap.add_argument("--width", type=float, default=None, help="W: khoang cach 2 mau giay (m)")
    a = ap.parse_args()

    if a.distance is not None and a.width is not None:
        report(a.distance, a.width)
        return

    src = int(a.source) if str(a.source).isdigit() else a.source
    cap = cv2.VideoCapture(src)
    # Giong node: MJPG 640x480 (che do khac co the cat khung khac -> FOV khac)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise SystemExit(f"Khong mo duoc camera {a.source}")
    print("Dan 2 mau giay len tuong cham 2 vach DO, roi bam q.")
    w_px, h_px = 640, 480
    while True:
        ok, img = cap.read()
        if not ok:
            continue
        h_px, w_px = img.shape[:2]
        cv2.line(img, (2, 0), (2, h_px - 1), (0, 0, 255), 2)
        cv2.line(img, (w_px - 3, 0), (w_px - 3, h_px - 1), (0, 0, 255), 2)
        cv2.line(img, (w_px // 2, 0), (w_px // 2, h_px - 1), (0, 255, 0), 1)
        cv2.putText(img, "vach do = mep khung; q = xong", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("measure_fov", img)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()
    d = float(input("D - khoang cach camera toi tuong (m): ").strip())
    w = float(input("W - khoang cach giua 2 mau giay (m): ").strip())
    report(d, w, w_px, h_px)


if __name__ == "__main__":
    main()
