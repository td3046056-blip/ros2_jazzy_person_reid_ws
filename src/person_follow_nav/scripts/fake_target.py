#!/usr/bin/env python3
"""
fake_target.py — gia lap "nguoi" de test ne vat can ma khong can camera.

Thay cho lenh `ros2 topic pub` dai dong va de go nham.

CACH DUNG
---------
  # nguoi dung yen cach 1.2m thang truoc mat (xe se nhich 0.2m roi dung)
  python3 fake_target.py 1.2

  # cach 2.5m, lech trai 20 do
  python3 fake_target.py 2.5 20

  # cach 2.5m, va in trang thai planner moi 0.5s
  python3 fake_target.py 2.5 --watch

  # dung: Ctrl-C  (tu goi /follow/stop truoc khi thoat)

LUU Y QUAN TRONG
----------------
Muc tieu gia nay o KHUNG base_link, nghia la no LUON cach xe dung khoang do
du xe di bao xa. Xe se duoi mai khong bao gio toi noi.

  - Dat 1.2m  -> goal_r = 1.2 - 1.0 = 0.2m -> xe nhich 20cm roi DUNG. An toan.
  - Dat 2.5m  -> goal_r = 1.5m -> xe chay LIEN TUC cho den khi gap vat can.
                 Dung cho test ne vat can, nhung phai san sang bam /follow/stop.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class FakeTarget(Node):

    def __init__(self, dist: float, bearing_deg: float, watch: bool) -> None:
        super().__init__("fake_target")
        self.dist = float(dist)
        self.bearing = math.radians(float(bearing_deg))
        self.watch = watch
        self.pub = self.create_publisher(String, "/follow/target", 10)
        self.status = None
        if watch:
            self.create_subscription(String, "/follow/planner_status", self._st, 10)
            self.create_timer(0.5, self._print)
        self.create_timer(0.1, self._tick)
        self.stop_cli = self.create_client(Trigger, "/follow/stop")
        self.n = 0

        goal_r = max(0.0, self.dist - 1.0)
        print()
        print("=" * 68)
        print(f"  Muc tieu gia: {self.dist:.2f} m, goc {bearing_deg:+.0f} do")
        print(f"  Khoang cach bam 1.00 m  =>  xe se tien {goal_r:.2f} m")
        if goal_r > 0.5:
            print()
            print("  [!] Muc tieu o khung base_link nen no LUON cach xe 2.50m.")
            print("      Xe se chay LIEN TUC. San sang bam Ctrl-C.")
        print()
        print("  Bat bam:  ros2 service call /follow/enable std_srvs/srv/Trigger {}")
        print("  Dung:     Ctrl-C  (tu goi /follow/stop)")
        print("=" * 68)

    def _tick(self) -> None:
        d = self.dist
        b = self.bearing
        payload = {
            "stamp": time.time(),
            "status": "ok",
            "valid": True,
            "source": "fake",
            "measured_this_tick": True,
            "confidence": 1.0,
            "age_since_fix_sec": 0.0,
            "base_x": round(d * math.cos(b), 4),
            "base_y": round(d * math.sin(b), 4),
            "distance_m": round(d, 3),
            "bearing_rad": round(b, 4),
            "bearing_deg": round(math.degrees(b), 2),
            "in_camera_fov": abs(math.degrees(b)) <= 31.0,
            "speed": 0.0,
            "vx": 0.0,
            "vy": 0.0,
        }
        self.pub.publish(String(data=json.dumps(payload)))
        self.n += 1

    def _st(self, msg: String) -> None:
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass

    def _print(self) -> None:
        s = self.status
        if not s:
            print("  (chua nhan /follow/planner_status — planner chua chay?)")
            return
        print(f"  [{s.get('state','?'):8s}] {s.get('note',''):32s} "
              f"v={s.get('cmd_v',0):+.3f} w={s.get('cmd_w',0):+.3f}  "
              f"thoang={s.get('front_clearance_m','-')}  "
              f"huong={s.get('chosen_heading_deg','-')}  "
              f"n_obs={s.get('n_obstacles','-')}")

    def stop_robot(self) -> None:
        try:
            if self.stop_cli.wait_for_service(timeout_sec=1.0):
                self.stop_cli.call_async(Trigger.Request())
                rclpy.spin_once(self, timeout_sec=0.5)
                print("\n  Da goi /follow/stop.")
            else:
                print("\n  Khong goi duoc /follow/stop — hay tu bam nut dung.")
        except Exception:
            pass


def main() -> None:
    ap = argparse.ArgumentParser(description="Gia lap muc tieu de test ne vat can")
    ap.add_argument("distance", type=float, nargs="?", default=1.2,
                    help="khoang cach toi muc tieu gia, met (mac dinh 1.2)")
    ap.add_argument("bearing", type=float, nargs="?", default=0.0,
                    help="goc, do. Duong = ben TRAI xe (mac dinh 0)")
    ap.add_argument("--watch", action="store_true",
                    help="in trang thai planner moi 0.5 giay")
    a = ap.parse_args()

    rclpy.init()
    node = FakeTarget(a.distance, a.bearing, a.watch)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
