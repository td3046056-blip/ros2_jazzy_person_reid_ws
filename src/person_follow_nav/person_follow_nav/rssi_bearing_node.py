"""
rssi_bearing_node.py
====================
Uoc luong HUONG nguoi deo beacon tu mau RSSI tho + odom, bang cach "xoay do huong" (xem rssi_df.py).

Node nay CHI LANG NGHE: khong bao gio ghi /cmd_vel. No chi bao valid=true khi xe da tu xoay tai cho du
(min_sweep_deg) trong window_sec gan day va song do duoc khop mau du tot. Ai can huong (planner / script
thu) thi tu cho xe xoay roi doc ket qua o day.

KHONG dung de bam khi camera con thay nguoi (camera+LiDAR chinh xac +-1 do; RSSI +-15 do va can ~6-8 s xoay).
Dung de TIM LAI nguoi khi mat han.

Topics subscribed:
  /rssi/raw   std_msgs/String (JSON)  tu rssi_scanner_node
  /odom       nav_msgs/Odometry

Service:
  /rssi/reset    std_srvs/Trigger — xoa mau cu truoc mot lan xoay do moi

Topics published:
  /rssi/bearing  std_msgs/String (JSON), publish_hz:
      stamp, valid, reason,
      bearing_odom_rad                  huong nguoi trong khung odom (dung cai nay de quay xe)
      bearing_base_rad, bearing_base_deg  huong so voi mui xe LUC NAY (trai duong)
      swept_deg, corr, margin, cover, n_samples
      levels {A,B,C: dBm trung binh 2 s gan day}, beacon_age_sec, beacon_ok
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Dict

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .geometry import yaw_from_quaternion
from .rssi_df import RotationDF, load_template


class RssiBearingNode(Node):

    def __init__(self) -> None:
        super().__init__("rssi_bearing_node")
        self._declare_params()
        self._read_params()

        tmpl = load_template(self.template_file)
        if not tmpl:
            self.get_logger().error(
                f"Khong doc duoc file mau '{self.template_file}' — chay scripts/rssi_rotate.py --calib de tao. "
                "Node van chay nhung luon bao valid=false.")
        self.df = RotationDF(tmpl or {}, window_sec=self.window_sec, min_sweep_deg=self.min_sweep_deg,
                             reset_dist_m=self.reset_dist_m, min_corr=self.min_corr, min_margin=self.min_margin,
                             relax_corr=self.relax_corr, relax_margin=self.relax_margin)
        self.have_tmpl = bool(tmpl)
        self.last_valid = None

        self.pub = self.create_publisher(String, self.bearing_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 50)
        self.create_subscription(String, self.raw_topic, self._raw_cb, 50)
        self.create_service(Trigger, "/rssi/reset", self._reset_srv)
        self.create_timer(1.0 / max(0.5, self.publish_hz), self._tick)
        self.get_logger().info(
            f"rssi_bearing: mau {sorted(tmpl) if tmpl else '(khong co)'}, can xoay >= {self.min_sweep_deg:.0f} do "
            f"trong {self.window_sec:.0f} s, corr >= {self.min_corr} & margin >= {self.min_margin} "
            f"(hoac corr >= {self.relax_corr} & margin >= {self.relax_margin})")

    # ─────────────────────────────────────────────────────────────────────
    def _declare_params(self) -> None:
        d: Dict[str, Any] = {
            "raw_topic": "/rssi/raw",
            "odom_topic": "/odom",
            "bearing_topic": "/rssi/bearing",
            "template_file": "",
            # Do 01/10: xoay 180 do sai toi 44-61 do, 270 do sai <= 28 do, du vong <= 24 do
            "window_sec": 14.0,
            "min_sweep_deg": 250.0,
            "reset_dist_m": 0.30,
            # 14 lan do that: lan hop le co corr 0.46-0.74, margin 0.13-0.47; lan sai 46 do co margin 0.04
            "min_corr": 0.45,
            "min_margin": 0.10,
            # Khop hoi kem nhung KHONG mo ho thi van tin (182 cua so quet that: +13 cua so, khong them lan nao sai > 45 do)
            "relax_corr": 0.35,
            "relax_margin": 0.25,
            "publish_hz": 5.0,
            "beacon_timeout_sec": 3.0,
        }
        dyn = ParameterDescriptor(dynamic_typing=True)
        for k, v in d.items():
            self.declare_parameter(k, v, dyn)

    def _read_params(self) -> None:
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.raw_topic = str(g("raw_topic"))
        self.odom_topic = str(g("odom_topic"))
        self.bearing_topic = str(g("bearing_topic"))
        self.template_file = os.path.expanduser(str(g("template_file")))
        self.window_sec = float(g("window_sec"))
        self.min_sweep_deg = float(g("min_sweep_deg"))
        self.reset_dist_m = float(g("reset_dist_m"))
        self.min_corr = float(g("min_corr"))
        self.min_margin = float(g("min_margin"))
        self.relax_corr = float(g("relax_corr"))
        self.relax_margin = float(g("relax_margin"))
        self.publish_hz = float(g("publish_hz"))
        self.beacon_timeout = float(g("beacon_timeout_sec"))

    # ─────────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.df.add_odom(time.time(), float(p.x), float(p.y), yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _raw_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        for board, rssi, t in data.get("samples", []):
            self.df.add_sample(float(t), str(board), float(rssi))

    def _reset_srv(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """Bat dau mot lan do MOI: xoa mau cu (nguoi co the da di cho khac tu lan xoay truoc)."""
        self.df.reset()
        res.success = True
        res.message = "da xoa mau RSSI cu"
        return res

    def _tick(self) -> None:
        now = time.time()
        r = self.df.estimate(now)
        if not self.have_tmpl:
            r["valid"] = False
            r["reason"] = "khong co file mau"
        age = r.get("beacon_age")
        out: Dict[str, Any] = {
            "stamp": round(now, 3),
            "valid": bool(r["valid"]),
            "reason": r.get("reason", ""),
            "swept_deg": round(r.get("swept_deg", 0.0), 1),
            "n_samples": int(r.get("n_samples", 0)),
            "levels": {k: round(v, 1) for k, v in r.get("levels", {}).items()},
            "beacon_age_sec": None if age is None else round(age, 2),
            "beacon_ok": age is not None and age <= self.beacon_timeout,
        }
        if "bearing_odom" in r:
            out.update({
                "bearing_odom_rad": round(r["bearing_odom"], 4),
                "bearing_base_rad": round(r["bearing_base"], 4),
                "bearing_base_deg": round(math.degrees(r["bearing_base"]), 1),
                "corr": round(r["corr"], 3),
                "margin": round(r["margin"], 3),
                "cover": round(r["cover"], 2),
            })
        self.pub.publish(String(data=json.dumps(out)))
        if out["valid"] != self.last_valid:
            self.last_valid = out["valid"]
            if out["valid"]:
                self.get_logger().info(
                    f"huong nguoi: {out['bearing_base_deg']:+.0f} do so voi mui xe (da xoay {out['swept_deg']:.0f} do, "
                    f"corr {out['corr']:.2f}, margin {out['margin']:.2f})")
            else:
                self.get_logger().info(f"chua co huong hop le: {out['reason']}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RssiBearingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):     # Ctrl-C truc tiep / launch dung node
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
