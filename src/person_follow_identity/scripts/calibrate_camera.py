#!/usr/bin/env python3
"""
calibrate_camera.py — hieu chinh noi tai camera (ma tran K + he so meo) bang ban co.

Dung khi doi sang camera GOC RONG (90-120 do) hoac thay goc nguoi o mep khung sai.
Ket qua dan vao identity_lock_kingsen.yaml:
    camera_angle_model: "calibrated"
    camera_matrix: [...]
    dist_coeffs: [...]
    camera_fisheye: false|true
    undistort_frame: true      # tuy chon: khu meo ca khung truoc YOLO/ReID

Chuan bi: in ban co (mac dinh 9x6 GOC TRONG, o vuong 25 mm), dan phang len bia cung.
KHONG chay cung luc voi identity_lock_node (hai tien trinh khong mo chung camera duoc).

Chay voi camera:
    python3 calibrate_camera.py --source /dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0
      phim c: chup mau (can thay du ban co), phim k: tinh ket qua, phim q: thoat
    Chup 15-25 mau: gan/xa, nghieng, va DAC BIET o 4 goc + 2 mep trai/phai khung
    (meo nang nhat o mep — thieu mau o mep thi he so meo sai).

Chay voi thu muc anh da chup san:
    python3 calibrate_camera.py --images ~/calib_imgs --fisheye
"""
from __future__ import annotations

import argparse
import glob
import math
import os
from typing import List, Tuple

import cv2
import numpy as np


def find_corners(gray: np.ndarray, pattern: Tuple[int, int]):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)


def calibrate(obj_pts: List[np.ndarray], img_pts: List[np.ndarray], size: Tuple[int, int], fisheye: bool):
    if fisheye:
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        rms, K, D, _, _ = cv2.fisheye.calibrate(
            [o.reshape(-1, 1, 3) for o in obj_pts], [i.reshape(-1, 1, 2) for i in img_pts], size, K, D, flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
    else:
        rms, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    return rms, K, D


def report(rms: float, K: np.ndarray, D: np.ndarray, size: Tuple[int, int], fisheye: bool) -> None:
    w, h = size
    # FOV ngang THAT: goc cua hai diem mep trai/phai o giua doc, sau khi khu meo
    pts = np.asarray([[[0.0, h / 2.0]], [[w - 1.0, h / 2.0]]], dtype=np.float64)
    und = cv2.fisheye.undistortPoints(pts, K, D) if fisheye else cv2.undistortPoints(pts, K, D)
    fov = math.degrees(math.atan(abs(und[0, 0, 0])) + math.atan(abs(und[1, 0, 0])))
    print("\n" + "=" * 60)
    print(f"RMS reprojection error: {rms:.3f} px  (< 0.5 tot, > 1.0 nen chup lai)")
    print(f"FOV ngang do duoc: {fov:.1f} do  (so sanh voi camera_fov_deg dang dung)")
    print("Dan vao identity_lock_kingsen.yaml:")
    print('    camera_angle_model: "calibrated"')
    print("    camera_matrix: [" + ", ".join(f"{v:.4f}" for v in K.reshape(-1)) + "]")
    print("    dist_coeffs: [" + ", ".join(f"{v:.6f}" for v in D.reshape(-1)) + "]")
    print(f"    camera_fisheye: {'true' if fisheye else 'false'}")
    print(f"    camera_fov_deg: {fov:.1f}")
    print("=" * 60)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default=None, help="duong dan / chi so camera")
    ap.add_argument("--images", default=None, help="thu muc anh ban co da chup san")
    ap.add_argument("--cols", type=int, default=9, help="so goc trong theo chieu ngang")
    ap.add_argument("--rows", type=int, default=6, help="so goc trong theo chieu doc")
    ap.add_argument("--square", type=float, default=0.025, help="canh o vuong (m)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fisheye", action="store_true", help="mo hinh mat ca (camera > ~120 do)")
    a = ap.parse_args()

    pattern = (a.cols, a.rows)
    objp = np.zeros((a.cols * a.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
    obj_pts: List[np.ndarray] = []
    img_pts: List[np.ndarray] = []
    size = (a.width, a.height)

    if a.images:
        files = sorted(glob.glob(os.path.join(os.path.expanduser(a.images), "*.png")) + glob.glob(os.path.join(os.path.expanduser(a.images), "*.jpg")))
        for f in files:
            img = cv2.imread(f)
            if img is None:
                continue
            size = (img.shape[1], img.shape[0])
            c = find_corners(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), pattern)
            print(f"{os.path.basename(f)}: {'OK' if c is not None else 'khong thay ban co'}")
            if c is not None:
                obj_pts.append(objp.copy())
                img_pts.append(c)
    else:
        src = a.source if a.source is not None else "0"
        cap = cv2.VideoCapture(int(src) if str(src).isdigit() else src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
        if not cap.isOpened():
            raise SystemExit(f"Khong mo duoc camera {src}")
        print("c = chup mau, k = tinh ket qua, q = thoat")
        while True:
            ok, img = cap.read()
            if not ok:
                continue
            size = (img.shape[1], img.shape[0])
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            c = find_corners(gray, pattern)
            view = img.copy()
            if c is not None:
                cv2.drawChessboardCorners(view, pattern, c, True)
            for pts in img_pts:
                x, y, w, h = cv2.boundingRect(pts.astype(np.float32))
                cv2.rectangle(view, (x, y), (x + w, y + h), (0, 160, 0), 1)
            cv2.putText(view, f"mau: {len(img_pts)}  (c chup, k tinh, q thoat)", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow("calibrate_camera", view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("c") and c is not None:
                obj_pts.append(objp.copy())
                img_pts.append(c)
                print(f"da chup {len(img_pts)} mau")
            elif key == ord("k"):
                break
            elif key == ord("q"):
                cap.release()
                cv2.destroyAllWindows()
                return
        cap.release()
        cv2.destroyAllWindows()

    if len(img_pts) < 8:
        raise SystemExit(f"Chi co {len(img_pts)} mau hop le, can it nhat 8 (nen 15-25)")
    rms, K, D = calibrate(obj_pts, img_pts, size, a.fisheye)
    report(rms, K, D, size, a.fisheye)


if __name__ == "__main__":
    main()
