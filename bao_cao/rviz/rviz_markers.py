#!/usr/bin/env python3
"""Doi /follow/target va /follow/planner_status (JSON trong bag) thanh Marker cho RViz2 — CHI DOC topic,
khong ghi /cmd_vel. Dung cung xem_bag.sh (phat lai bag trong mien ROS rieng).

Xuat /bao_cao/markers (MarkerArray):
  - footprint chu nhat cua xe (base_link): F 0.14, R 0.33, HW 0.30 (muc 2 CLAUDE.md), mau theo trang thai planner
  - nguoi (khung odom, tu tracker): mau theo nguon (camera+lidar / camera+bbox / lidar_track / predicted)
  - vet xe trong khung odom (tu /odom)
  - chu: trang thai + note + khoang cach + lenh (v, w)
"""
import json
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

F_LEN, R_LEN, HW = 0.14, 0.33, 0.30

MAU_TT = {
    "FOLLOW": (0.11, 0.62, 0.47), "AVOID": (0.90, 0.67, 0.01), "ARRIVED": (0.40, 0.65, 0.12),
    "SEARCH": (0.46, 0.44, 0.70), "OCCLUDED": (0.65, 0.46, 0.11), "BLOCKED": (0.84, 0.15, 0.16),
    "IDLE": (0.45, 0.45, 0.45), "ESTOP": (0.84, 0.15, 0.16),
}
MAU_NGUON = {
    "camera+lidar": (0.17, 0.63, 0.17), "camera+bbox": (0.55, 0.80, 0.25), "lidar_track": (0.12, 0.47, 0.71),
    "predicted": (0.60, 0.60, 0.60), "rssi_bearing": (0.58, 0.40, 0.74),
}


def mau(rgb, a=1.0):
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


class RvizMarkers(Node):
    def __init__(self):
        super().__init__("bao_cao_rviz_markers")
        self.pub = self.create_publisher(MarkerArray, "/bao_cao/markers", 10)
        self.create_subscription(String, "/follow/target", self.cb_target, 10)
        self.create_subscription(String, "/follow/planner_status", self.cb_status, 10)
        self.create_subscription(Odometry, "/odom", self.cb_odom, 50)
        self.target = {}
        self.status = {}
        self.trail = []
        self.last_trail = None
        self.create_timer(0.1, self.publish)

    def cb_target(self, msg):
        try:
            self.target = json.loads(msg.data)
        except ValueError:
            pass

    def cb_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_trail is None or math.hypot(p.x - self.last_trail[0], p.y - self.last_trail[1]) > 0.03:
            self.trail.append((p.x, p.y))
            self.last_trail = (p.x, p.y)
            if len(self.trail) > 4000:
                self.trail = self.trail[-4000:]

    def mk(self, ns, mid, typ, frame):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = rclpy.time.Time().to_msg()       # 0 = dung TF moi nhat
        m.ns, m.id, m.type, m.action = ns, mid, typ, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish(self):
        arr = MarkerArray()
        st = str(self.status.get("state", "IDLE"))
        # footprint xe
        fp = self.mk("xe", 0, Marker.LINE_STRIP, "base_link")
        fp.scale.x = 0.025
        fp.color = mau(MAU_TT.get(st, (0.5, 0.5, 0.5)))
        for (x, y) in ((F_LEN, HW), (F_LEN, -HW), (-R_LEN, -HW), (-R_LEN, HW), (F_LEN, HW)):
            fp.points.append(Point(x=x, y=y, z=0.0))
        arr.markers.append(fp)
        mui = self.mk("xe", 1, Marker.ARROW, "base_link")
        mui.scale.x, mui.scale.y, mui.scale.z = 0.03, 0.06, 0.06
        mui.color = fp.color
        mui.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=0.30, y=0.0, z=0.0)]
        arr.markers.append(mui)
        # vet xe
        if len(self.trail) > 1:
            tr = self.mk("vet", 0, Marker.LINE_STRIP, "odom")
            tr.scale.x = 0.02
            tr.color = mau((0.85, 0.37, 0.01), 0.8)
            tr.points = [Point(x=x, y=y, z=0.0) for (x, y) in self.trail]
            arr.markers.append(tr)
        # nguoi
        t = self.target
        if t.get("valid") and t.get("odom_x") is not None:
            src = str(t.get("source", ""))
            ng = self.mk("nguoi", 0, Marker.CYLINDER, "odom")
            ng.pose.position.x, ng.pose.position.y, ng.pose.position.z = float(t["odom_x"]), float(t["odom_y"]), 0.4
            ng.scale.x = ng.scale.y = 0.30
            ng.scale.z = 0.8
            ng.color = mau(MAU_NGUON.get(src, (0.6, 0.6, 0.6)), 0.85)
            arr.markers.append(ng)
        else:
            de = self.mk("nguoi", 0, Marker.CYLINDER, "odom")
            de.action = Marker.DELETE
            arr.markers.append(de)
        # chu
        tx = self.mk("chu", 0, Marker.TEXT_VIEW_FACING, "base_link")
        tx.pose.position.x, tx.pose.position.z = -0.1, 0.9
        tx.scale.z = 0.16
        tx.color = mau((0.1, 0.1, 0.1))
        d = t.get("distance_m")
        tx.text = (f"{st}  {self.status.get('note', '')}\n"
                   f"nguon: {t.get('source', '-')}  d = {d:.2f} m" if isinstance(d, (int, float)) else
                   f"{st}  {self.status.get('note', '')}")
        tx.text += f"\nv = {float(self.status.get('cmd_v', 0.0)):+.2f}  w = {float(self.status.get('cmd_w', 0.0)):+.2f}"
        arr.markers.append(tx)
        self.pub.publish(arr)


def main():
    rclpy.init()
    n = RvizMarkers()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
