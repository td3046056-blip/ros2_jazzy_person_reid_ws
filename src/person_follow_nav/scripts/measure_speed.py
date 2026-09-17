#!/usr/bin/env python3
"""
measure_speed.py — do toc do THAT cua xe ung voi mot lenh /cmd_vel co dinh.

Dung de hieu chinh max_linear / max_angular cua driver BW-DR03.

VI SAO CAN DO
-------------
Driver doi /cmd_vel sang % PWM mo vong: pwm = v / max_linear * max_percent.
Neu max_linear khong phai toc do THAT o max_percent thi xe chay khac lenh.
Truoc day launch dung 0.226 (so calib o max_percent 30) voi max_percent 60
-> xe chay nhanh gap 2.16 lan lenh. Da do va sua thanh 0.49 / 2.46.

Hang so DRIVER_* ben duoi PHAI TRUNG voi launch dang chay. Sau khi doi launch,
chay lai: ti le that/lenh phai ~0.95-1.0 (driver cat phan le % nen hoi thap).

CACH DUNG
---------
  Terminal 1 (CHI lidar + driver, KHONG co planner):
    ros2 launch person_follow_nav calibrate.launch.py

  Terminal 2:
    # di thang — can >= 2.5m trong truoc mui xe
    python3 measure_speed.py 0.22
    python3 measure_speed.py 0.11
    python3 measure_speed.py 0.05

    # xoay tai cho — duoi xe vang ban kinh 0.45m, don trong xung quanh
    python3 measure_speed.py 0.0 --w 0.8

    # lenh NHO NHAT planner gui (min_move_linear / min_move_angular) -> 5% PWM.
    # Banh phai quay. Neu v/w ~0 thi phai tang min_move_* trong follow_nav.yaml.
    python3 measure_speed.py 0.035
    python3 measure_speed.py 0.0 --w 0.10

  Dung khan cap: Ctrl-C (tu gui lenh dung truoc khi thoat)

KIEM CHUNG BANG THUOC
---------------------
Toc do tu /odom phu thuoc encoder_ppr (750) va wheel_radius. Da kiem chung
16/09: thuoc 1.50 m / odom 1.503 m. Khi do lai: dan bang dinh danh dau tam
truc banh truoc truoc khi chay, do quang duong that sau khi xe dung han,
so voi dong "tong quang duong odom" in ra cuoi.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# Gia tri driver dang dat trong launch cua person_follow_nav — phai trung
DRIVER_MAX_LINEAR = 0.49
DRIVER_MAX_ANGULAR = 2.46
DRIVER_MAX_PERCENT = 60
DRIVER_DEADBAND = 0.02

PUB_HZ = 15.0


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def driver_pwm(v: float, w: float) -> tuple[int, int]:
    """% PWM co dau (phai, trai) y het cmd_vel_callback cua decoded_serial_node. 0 = dung."""
    if abs(v) < DRIVER_DEADBAND and abs(w) < DRIVER_DEADBAND:
        return 0, 0
    x_norm = max(-1.0, min(1.0, v / DRIVER_MAX_LINEAR))
    z_norm = max(-1.0, min(1.0, w / DRIVER_MAX_ANGULAR))
    right = x_norm + z_norm
    left = x_norm - z_norm
    m = max(abs(right), abs(left), 1.0)
    out = []
    for val in (right / m, left / m):
        if abs(val) < DRIVER_DEADBAND:
            out.append(0)
        else:
            pct = max(5, min(DRIVER_MAX_PERCENT, int(abs(val) * DRIVER_MAX_PERCENT)))
            out.append(pct if val > 0 else -pct)
    return out[0], out[1]


def fmt_pwm(pct: int) -> str:
    return "%+d%%" % pct if pct else "dung"


class SpeedMeter(Node):
    def __init__(self) -> None:
        super().__init__("measure_speed")
        self.samples: list[tuple[float, float, float, float]] = []   # (t, x, y, yaw)
        self.create_subscription(Odometry, "/odom", self._on_odom, 50)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.samples.append((time.monotonic(), p.x, p.y, yaw_of(msg)))

    def spin_for(self, sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, end - time.monotonic()))


def travel(samples: list[tuple[float, float, float, float]]) -> tuple[float, float]:
    """Quang duong tien (chieu len huong ban dau) va goc quay cong don."""
    _, x0, y0, yaw0 = samples[0]
    _, x1, y1, _ = samples[-1]
    fwd = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
    dyaw = 0.0
    for a, b in zip(samples, samples[1:]):
        dyaw += math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
    return fwd, dyaw


def main() -> int:
    ap = argparse.ArgumentParser(description="Do toc do that cua xe BW-DR03")
    ap.add_argument("v", type=float, help="lenh linear.x (m/s)")
    ap.add_argument("--w", type=float, default=0.0, help="lenh angular.z (rad/s)")
    ap.add_argument("--sec", type=float, default=3.0, help="thoi gian gui lenh (s)")
    ap.add_argument("--settle", type=float, default=0.8,
                    help="bo qua giai doan tang toc dau tien (s)")
    args = ap.parse_args()

    if abs(args.v) > 0.25 or abs(args.w) > 1.2:
        print("Lenh qua lon (|v| <= 0.25, |w| <= 1.2). Khong chay.")
        return 1
    if args.sec < args.settle + 1.0:
        print("--sec phai >= --settle + 1.0 de co du mau.")
        return 1

    # Tat signal handler cua rclpy: Ctrl-C phai de context con song
    # thi moi gui duoc lenh dung.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SpeedMeter()
    pub = None
    stop = Twist()
    try:
        node.spin_for(1.5)   # cho discovery + /odom

        # Chi MOT nguon duoc ghi /cmd_vel — neu planner dang chay thi dung lai
        n_pub = node.count_publishers("/cmd_vel")
        if n_pub > 0:
            print("Dang co %d node publish /cmd_vel (follow_planner?)." % n_pub)
            print("Dung launch khac, chi chay: ros2 launch person_follow_nav calibrate.launch.py")
            return 1
        if not node.samples:
            node.spin_for(3.0)
        if not node.samples:
            print("Khong nhan duoc /odom. Driver xe co dang chay khong?")
            return 1

        pwm_r, pwm_l = driver_pwm(args.v, args.w)
        if pwm_r == 0 and pwm_l == 0:
            print("Lenh nam trong deadband cua driver — xe se khong chay. Khong do.")
            return 1
        pub = node.create_publisher(Twist, "/cmd_vel", 10)
        print("Lenh: v=%.3f m/s  w=%.3f rad/s  ->  driver gui PWM phai %s, trai %s"
              % (args.v, args.w, fmt_pwm(pwm_r), fmt_pwm(pwm_l)))
        for k in (3, 2, 1):
            print("  xe chay sau %d s ... (Ctrl-C de huy)" % k)
            node.spin_for(1.0)

        cmd = Twist()
        cmd.linear.x = args.v
        cmd.angular.z = args.w
        node.samples = node.samples[-1:]      # giu 1 mau ngay truoc khi chay
        t_start = time.monotonic()
        while time.monotonic() - t_start < args.sec:
            pub.publish(cmd)
            node.spin_for(1.0 / PUB_HZ)
        t_cmd_end = time.monotonic()

        # Dung va cho xe dung han de tinh ca quang duong troi
        t_stop = time.monotonic()
        while time.monotonic() - t_stop < 1.5:
            pub.publish(stop)
            node.spin_for(1.0 / PUB_HZ)
    except KeyboardInterrupt:
        if pub is not None:
            for _ in range(5):
                pub.publish(stop)
                time.sleep(0.05)
        print("\nDa huy — da gui lenh dung.")
        return 1
    finally:
        if pub is not None:
            pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

    steady = [s for s in node.samples if t_start + args.settle <= s[0] <= t_cmd_end]
    if len(steady) < 5:
        print("Qua it mau /odom trong cua so on dinh (%d). Kiem tra: ros2 topic hz /odom" % len(steady))
        return 1

    dt = steady[-1][0] - steady[0][0]
    fwd, dyaw = travel(steady)
    v_real = fwd / dt
    w_real = dyaw / dt
    total_fwd, total_yaw = travel(node.samples)

    print()
    print("=" * 64)
    print(" KET QUA (tu /odom, cua so on dinh %.2f s, bo %.1f s dau)" % (dt, args.settle))
    print("=" * 64)
    print("  Lenh           : v=%.3f m/s   w=%.3f rad/s" % (args.v, args.w))
    print("  Thuc te (odom) : v=%.3f m/s   w=%.3f rad/s" % (v_real, w_real))
    # Goi y tinh theo % PWM driver THUC SU gui (da cat phan le), chi khi di
    # thang thuan hoac xoay thuan — lenh tron thi hai banh khac % nen khong suy ra duoc.
    pct = (abs(pwm_r) + abs(pwm_l)) / 2.0
    if abs(args.w) < 1e-3:
        print("  Ti le v that/lenh = %.2f" % (v_real / args.v))
        print("  -> max_linear khop thuc te (neu odom dung) = %d * %.3f / %.0f = %.3f"
              "   (dang dung %.3f)"
              % (DRIVER_MAX_PERCENT, abs(v_real), pct,
                 DRIVER_MAX_PERCENT * abs(v_real) / pct, DRIVER_MAX_LINEAR))
    elif abs(args.v) < 1e-3:
        print("  Ti le w that/lenh = %.2f" % (w_real / args.w))
        print("  -> max_angular khop thuc te (neu odom dung) = %d * %.3f / %.0f = %.3f"
              "   (dang dung %.2f)"
              % (DRIVER_MAX_PERCENT, abs(w_real), pct,
                 DRIVER_MAX_PERCENT * abs(w_real) / pct, DRIVER_MAX_ANGULAR))
    print("  Tong quang duong odom (ke ca troi sau khi dung): %.3f m, quay %.1f do"
          % (total_fwd, math.degrees(total_yaw)))
    print("  => Do bang thuoc quang duong that va so voi con so tren.")
    print("=" * 64)
    return 0


if __name__ == "__main__":
    sys.exit(main())
