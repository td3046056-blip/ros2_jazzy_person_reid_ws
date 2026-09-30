#!/usr/bin/env python3
"""
measure_fov.py — do FOV NGANG va DO MEO cua camera bang thuoc day (khong can ban co).

Vi sao can: KINGSEN o 640x480 la anh CAT GIUA 1440x1080 cua cam bien (so khop anh 30/09).
Nguoi dung do 30/09: D = 1 m, hai mep cach nhau 2.75 m -> FOV ngang 107.9 do (khong phai 62).
Ong goc rong thuong meo thung: chi biet FOV o mep thi goc cua nguoi o GIUA-khung van co the
lech toi ~7 do (x = 160 px: 27 do neu ong "mat ca", 34.5 do neu khong meo). Do them cac vach
giua de biet dung.

Cach do (5 phut):
  1. KHONG chay identity_lock_node cung luc (hai tien trinh khong mo chung camera duoc).
  2. Dat camera vuong goc voi tuong phang, cach tuong D met (vd 1.00 m).
  3. Chay script: cua so hien 9 vach danh so 1..9 (5 = giua, 1 va 9 = mep).
  4. Dan giay len tuong NGANG TAM camera, moi mau giay cham dung 1 vach (it nhat vach 1, 3, 5,
     7, 9; du 9 vach thi tot nhat). Bam q.
  5. Keo thuoc tu mau giay o vach 1 sang phai, doc vi tri (m) cac mau giay con lai, nhap vao.

  python3 measure_fov.py                                   # mo camera, huong dan tung buoc
  python3 measure_fov.py --distance 1.0 --width 2.75       # chi 2 mep: in FOV
  python3 measure_fov.py --distance 1.0 --marks 0,0.52,0.93,1.20,1.375,1.55,1.82,2.23,2.75
                                                           # vi tri 9 mau giay tinh tu vach 1 (bo trong = khong do)

Ket qua: FOV ngang, bang goc tung vach, va tham so dan vao identity_lock_kingsen.yaml.
"""
from __future__ import annotations

import argparse
import math
from typing import List, Optional

import numpy as np

W_PX, H_PX = 640, 480
U = [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0]   # vi tri 9 vach (chuan hoa theo nua be ngang)


def line_x(u: float) -> float:
    # Vach 1 va 9 ve o cot 2 va 637 (mep that cua anh)
    return min(W_PX - 3.0, max(2.0, 0.5 * W_PX * (1.0 + u)))


def report_two_edges(d: float, w: float) -> None:
    hfov = 2.0 * math.degrees(math.atan(0.5 * w / d))
    fx = (0.5 * W_PX) / math.tan(math.radians(0.5 * hfov))
    print("=" * 64)
    print(f"D = {d:.3f} m, W = {w:.3f} m -> FOV ngang = {hfov:.1f} do")
    print(f"Neu ong KHONG meo (pinhole): fx = {fx:.0f} px, FOV doc = {2 * math.degrees(math.atan(0.5 * H_PX / fx)):.1f} do")
    print(f"Neu ong 'mat ca' (goc ti le voi pixel): FOV doc = {hfov * H_PX / W_PX:.1f} do")
    print("Dan vao identity_lock_kingsen.yaml:")
    print(f"    camera_fov_deg: {hfov:.1f}")
    print("Ong goc rong: do them cac vach giua (--marks) de biet do meo.")
    print("=" * 64)


def fit_marks(d: float, pos: List[Optional[float]]) -> None:
    """Khop mo hinh mat ca OpenCV (Kannala-Brandt) tren hang giua: x - cx = fx*th*(1 + k1 th^2 + k2 th^4)."""
    from scipy.optimize import least_squares

    if pos[4] is None:
        raise SystemExit("Can mau giay o vach 5 (giua)")
    idx = [i for i, p in enumerate(pos) if p is not None]
    xs = np.array([line_x(U[i]) - 0.5 * W_PX for i in idx])
    offs = np.array([pos[i] - pos[4] for i in idx])
    # Camera co the hoi xoay so voi tuong (psi): offset = D*(tan(th + psi) - tan(psi))
    def angles(psi: float) -> np.ndarray:
        return np.arctan(offs / d + math.tan(psi)) - psi

    n_side = len({abs(U[i]) for i in idx if U[i] != 0.0})
    use_k = n_side >= 2  # can it nhat 2 muc |u| khac nhau moi uoc luong duoc do meo

    def resid(p: np.ndarray) -> np.ndarray:
        fx, psi = p[0], p[1]
        k1, k2 = (p[2], p[3]) if use_k and len(p) > 3 else ((p[2], 0.0) if use_k else (0.0, 0.0))
        th = angles(psi)
        return fx * th * (1.0 + k1 * th ** 2 + k2 * th ** 4) - xs

    p0 = [300.0, 0.0] + ([0.0, 0.0] if (use_k and n_side >= 3) else ([0.0] if use_k else []))
    sol = least_squares(resid, p0)
    fx, psi = sol.x[0], sol.x[1]
    k1 = sol.x[2] if use_k else 0.0
    k2 = sol.x[3] if (use_k and len(sol.x) > 3) else 0.0
    th = angles(psi)

    # Goc cua mep anh theo mo hinh da khop (giai nguoc x = 320 px)
    def theta_at(x: float) -> float:
        lo, hi = 0.0, 1.55
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if fx * mid * (1 + k1 * mid ** 2 + k2 * mid ** 4) < x:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    th_edge = theta_at(0.5 * W_PX)
    th_vert = theta_at(0.5 * H_PX)
    f_pin = (0.5 * W_PX) / math.tan(th_edge)
    print("=" * 76)
    print(f"D = {d:.3f} m, {len(idx)} vach, camera lech tuong {math.degrees(psi):+.1f} do, sai so khop {np.sqrt(np.mean(sol.fun ** 2)):.1f} px")
    print(f"{'vach':>4} {'x px':>6} {'do duoc':>8} {'mo hinh':>8} {'pinhole':>8} {'tuyen tinh':>10}  (do)")
    for j, i in enumerate(idx):
        x = xs[j]
        model = math.degrees(math.copysign(theta_at(abs(x)), x)) if x != 0 else 0.0
        pin = math.degrees(math.atan(x / f_pin))
        lin = math.degrees(th_edge) * x / (0.5 * W_PX)
        print(f"{i + 1:>4} {x + 0.5 * W_PX:6.0f} {math.degrees(th[j]):8.1f} {model:8.1f} {pin:8.1f} {lin:10.1f}")
    print(f"FOV ngang {2 * math.degrees(th_edge):.1f} do, FOV doc (hang giua) {2 * math.degrees(th_vert):.1f} do")
    if not use_k:
        print("Chi co vach o mep: chua biet do meo — dan them vach 3 va 7 (hoac du 9 vach).")
    print("Dan vao identity_lock_kingsen.yaml:")
    print(f"    camera_fov_deg: {2 * math.degrees(th_edge):.1f}")
    print('    camera_angle_model: "calibrated"')
    print(f"    camera_matrix: [{fx:.2f}, 0.0, {0.5 * W_PX:.1f}, 0.0, {fx:.2f}, {0.5 * H_PX:.1f}, 0.0, 0.0, 1.0]")
    print(f"    dist_coeffs: [{k1:.5f}, {k2:.5f}, 0.0, 0.0]")
    print("    camera_fisheye: true")
    print("(Gia dinh tam anh o giua khung va diem anh vuong.)")
    print("=" * 76)


def parse_marks(text: str) -> List[Optional[float]]:
    vals = [t.strip() for t in text.split(",")]
    if len(vals) != 9:
        raise SystemExit("--marks can dung 9 gia tri (bo trong vach khong do), vd 0,,0.93,,1.375,,1.82,,2.75")
    return [float(v) if v else None for v in vals]


def run_camera(source: str) -> None:
    import cv2

    src = int(source) if str(source).isdigit() else source
    cap = cv2.VideoCapture(src)
    # Giong node: MJPG 640x480 (che do khac cat khung khac -> FOV khac)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W_PX)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H_PX)
    if not cap.isOpened():
        raise SystemExit(f"Khong mo duoc camera {source}")
    print("Dan giay len tuong ngang tam camera, moi mau cham 1 vach (it nhat 1,3,5,7,9). Bam q khi xong.")
    while True:
        ok, img = cap.read()
        if not ok:
            continue
        for k, u in enumerate(U):
            x = int(round(line_x(u)))
            color = (0, 0, 255) if abs(u) == 1.0 else ((0, 255, 0) if u == 0.0 else (0, 220, 255))
            cv2.line(img, (x, 0), (x, H_PX - 1), color, 2 if abs(u) == 1.0 else 1)
            cv2.putText(img, str(k + 1), (min(W_PX - 16, max(4, x - 6)), 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.line(img, (0, H_PX // 2), (W_PX - 1, H_PX // 2), (255, 255, 255), 1)
        cv2.putText(img, "q = xong", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("measure_fov", img)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()
    d = float(input("D - khoang cach camera toi tuong (m): ").strip())
    print("Vi tri tung mau giay tinh tu mau o vach 1 (m); vach khong dan thi bam Enter:")
    pos: List[Optional[float]] = [0.0]
    for k in range(2, 10):
        t = input(f"  vach {k}: ").strip()
        pos.append(float(t) if t else None)
    if sum(p is not None for p in pos) <= 2 or pos[4] is None:
        if pos[8] is None:
            raise SystemExit("Can it nhat mau giay o vach 9")
        report_two_edges(d, pos[8])
    else:
        fit_marks(d, pos)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="/dev/v4l/by-id/usb-Generic_KINGSEN_CAMERA_200901010001-video-index0")
    ap.add_argument("--distance", type=float, default=None, help="D: camera toi tuong (m)")
    ap.add_argument("--width", type=float, default=None, help="W: khoang cach 2 mau giay o 2 mep (m)")
    ap.add_argument("--marks", default=None, help="9 vi tri mau giay tinh tu vach 1 (m), cach nhau dau phay")
    a = ap.parse_args()
    if a.distance is not None and a.marks:
        fit_marks(a.distance, parse_marks(a.marks))
    elif a.distance is not None and a.width is not None:
        report_two_edges(a.distance, a.width)
    else:
        run_camera(a.source)


if __name__ == "__main__":
    main()
