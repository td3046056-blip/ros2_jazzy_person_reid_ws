"""
target_tracker_node.py
======================
Node 1/2 cua he thong bam nguoi + ne vat can.

NHIEM VU: tra loi duy nhat mot cau hoi — "NGUOI DANG O DAU TRONG KHUNG odom?"
va tra loi duoc CA KHI CAMERA KHONG NHIN THAY.

Vi sao phai co node nay (van de cot loi cua ban hien tai):
--------------------------------------------------------
follow_controller.py hien tai luu `last_valid_payload` roi dung lai goc
`camera_angle_deg` cu trong 1.5s khi mat target. NHUNG goc do la goc TUONG DOI
voi than xe. Ngay khi xe quay de ne vat can, goc do SAI hoan toan — xe quay
trai 20 do thi nguoi da lech sang phai 20 do so voi luc do, ma controller van
tuong nguoi con o cho cu. Do la ly do xe khong the vua ne vua nho duoc nguoi.

Cach sua: quy doi vi tri nguoi ve khung odom (khung co dinh voi mat dat).
Khi xe quay/di chuyen, goc tuong doi toi diem da nho TU DONG cap nhat dung,
vi ta lay odom hien tai tru di.

NGUON DU LIEU (uu tien giam dan):
  1. Camera ReID  -> goc chinh xac (+-1 do), nhung KHOANG CACH tu bbox rat nhieu
  2. LiDAR        -> khoang cach chinh xac (+-2cm) tai goc camera bao
     => GHEP 2 CAI: goc tu camera, khoang cach tu lidar. Day la nang cap lon nhat.
  3. LiDAR tracking -> khi camera bi che nhung chan nguoi con thay duoc
  4. Du doan alpha-beta -> khi bi che hoan toan (toi da ~2.5s)
  5. RSSI bearing -> khi mat lau, sua lai HUONG cho du doan khoi troi

Subscribe:
  /person_reid/target   std_msgs/String (JSON tu person_follow_identity)
  /scan                 sensor_msgs/LaserScan
  /odom                 nav_msgs/Odometry
  /rssi/angle_deg       std_msgs/Float32
  /rssi/confidence      std_msgs/Float32

Publish:
  /follow/target        std_msgs/String (JSON) — dau vao cho follow_planner_node
  /follow/target_marker visualization_msgs/Marker — xem trong RViz
"""

from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from .geometry import (
    AlphaBeta2D,
    Cluster,
    cluster_points,
    scan_to_base_points,
    self_filter_mask,
    wrap_pi,
    yaw_from_quaternion,
)


class TargetTrackerNode(Node):

    def __init__(self) -> None:
        super().__init__("target_tracker_node")
        self._declare_params()
        self._read_params()

        # ── Trang thai ────────────────────────────────────────────────────
        self.filter = AlphaBeta2D(
            alpha=self.ab_alpha, beta=self.ab_beta, max_speed=self.max_person_speed
        )
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_ok = False
        self.yaw_history: List[tuple] = []          # [(t, yaw)] de bu tre camera

        self.scan_msg: Optional[LaserScan] = None
        self.scan_time = 0.0
        self.scan_pts = np.zeros((0, 2))
        self.scan_bear = np.zeros(0)
        self.scan_rng = np.zeros(0)

        self.cam_payload: Optional[Dict[str, Any]] = None
        self.cam_time = 0.0

        self.rssi_angle_deg = 0.0
        self.rssi_conf = 0.0
        self.rssi_time = 0.0

        self.last_camera_fix_time = 0.0      # lan cuoi co goc camera that
        self.last_lidar_fix_time = 0.0       # lan cuoi lidar bam duoc cum nguoi
        self.confidence = 0.0
        self.source = "none"
        self.tracking = False
        self.enabled_lidar_track = True

        # ── IO ────────────────────────────────────────────────────────────
        self.create_subscription(String, self.target_topic_in, self._cam_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.create_subscription(Float32, self.rssi_angle_topic, self._rssi_angle_cb, 10)
        self.create_subscription(Float32, self.rssi_conf_topic, self._rssi_conf_cb, 10)

        self.pub_target = self.create_publisher(String, self.target_topic_out, 10)
        self.pub_marker = self.create_publisher(Marker, self.marker_topic, 5)

        self.create_service(Trigger, "/follow/reset_tracker", self._reset_cb)

        self.create_timer(1.0 / max(1.0, self.update_hz), self._update)
        self.last_log = 0.0

        self.get_logger().info(
            "target_tracker_node san sang | lidar_yaw_offset=%.1f deg sign=%.0f | "
            "fuse_lidar_distance=%s | predict_max=%.1fs"
            % (
                math.degrees(self.lidar_yaw_offset_rad),
                self.lidar_angle_sign,
                self.use_lidar_distance,
                self.predict_max_sec,
            )
        )

    # ─────────────────────────────────────────────────────────────────────
    # Tham so
    # ─────────────────────────────────────────────────────────────────────

    def _declare_params(self) -> None:
        d: Dict[str, Any] = {
            # Topic
            "target_topic_in": "/person_reid/target",
            "target_topic_out": "/follow/target",
            "marker_topic": "/follow/target_marker",
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "rssi_angle_topic": "/rssi/angle_deg",
            "rssi_conf_topic": "/rssi/confidence",
            "odom_frame": "odom",
            "base_frame": "base_link",

            # Hinh hoc lidar — PHAI HIEU CHINH, xem scripts/calibrate_lidar_front.py
            "lidar_yaw_offset_deg": -90.0,
            "lidar_angle_sign": 1.0,
            "lidar_x": 0.10,
            "lidar_y": 0.0,
            "lidar_max_use_range": 8.0,
            "self_filter_enabled": True,
            "self_filter_margin": 0.03,
            "blind_sectors_deg": [0.0],
            "front_len": 0.14,
            "rear_len": 0.33,
            "half_width": 0.30,

            # Camera
            "camera_angle_sign": -1.0,     # +cam_deg (phai khung hinh) -> -yaw base_link
            "camera_fov_deg": 62.0,
            "require_identity_ready": True,
            "camera_msg_timeout_sec": 0.6,
            "compensate_camera_latency": True,
            "max_camera_latency_sec": 0.6,

            # Ghep lidar de lay khoang cach
            "use_lidar_distance": True,
            "assoc_window_deg": 10.0,        # cung goc quanh huong camera de tim nguoi
            "cluster_gap_m": 0.18,
            "person_width_min_m": 0.08,
            "person_width_max_m": 0.90,
            "person_range_min_m": 0.30,
            "person_range_max_m": 6.0,
            "lidar_distance_max_jump_m": 0.60,

            # Fallback bbox (chi dung khi lidar khong thay)
            "bbox_fallback_enabled": True,
            "bbox_height_at_1m_px": 420.0,

            # Bam bang lidar khi camera mat (chan nguoi con thay)
            "lidar_only_track_enabled": True,
            "lidar_only_assoc_radius_m": 0.55,
            "lidar_only_max_sec": 3.0,

            # Du doan khi bi che hoan toan
            "predict_max_sec": 2.5,
            "ab_alpha": 0.55,
            "ab_beta": 0.10,
            "max_person_speed": 1.8,

            # RSSI
            "rssi_enabled": True,
            "rssi_angle_sign": -1.0,
            "rssi_min_confidence": 0.20,
            "rssi_timeout_sec": 2.0,
            "rssi_takeover_after_sec": 1.2,
            "rssi_assumed_distance_m": 1.6,
            "rssi_bearing_blend": 0.35,

            "update_hz": 20.0,
            "publish_marker": True,
            "log_period_sec": 1.0,
        }
        # ROS 2 phan biet nghiem ngat INTEGER voi DOUBLE. Khai bao mac dinh 0.22
        # (DOUBLE) ma go `-p v_max:=1` (INTEGER) la ROS nem InvalidParameterTypeException.
        # dynamic_typing cho phep ca hai; code doc ra van ep bang float()/int().
        dyn = ParameterDescriptor(dynamic_typing=True)
        for k, v in d.items():
            self.declare_parameter(k, v, dyn)

    def _read_params(self) -> None:
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.target_topic_in = str(g("target_topic_in"))
        self.target_topic_out = str(g("target_topic_out"))
        self.marker_topic = str(g("marker_topic"))
        self.scan_topic = str(g("scan_topic"))
        self.odom_topic = str(g("odom_topic"))
        self.rssi_angle_topic = str(g("rssi_angle_topic"))
        self.rssi_conf_topic = str(g("rssi_conf_topic"))
        self.odom_frame = str(g("odom_frame"))
        self.base_frame = str(g("base_frame"))

        self.lidar_yaw_offset_rad = math.radians(float(g("lidar_yaw_offset_deg")))
        self.lidar_angle_sign = float(g("lidar_angle_sign"))
        self.lidar_x = float(g("lidar_x"))
        self.lidar_y = float(g("lidar_y"))
        self.lidar_max_use_range = float(g("lidar_max_use_range"))
        self.self_filter = bool(g("self_filter_enabled"))
        self.self_margin = float(g("self_filter_margin"))
        # ROS 2 KHONG ho tro mang long nhau lam tham so — chi co
        # int64[], double[], string[]... Nen khai bao PHANG theo cap:
        #   [246.0, 294.0, 168.0, 197.0]  =  hai cung goc 246-294 va 168-197
        bs = list(g("blind_sectors_deg") or [])
        self.blind_sectors = [
            [float(bs[i]), float(bs[i + 1])] for i in range(0, len(bs) - 1, 2)
            if float(bs[i]) >= 0.0 and float(bs[i + 1]) > float(bs[i]) - 360.0
        ]
        self.front_len = float(g("front_len"))
        self.rear_len = float(g("rear_len"))
        self.half_width = float(g("half_width"))

        self.camera_angle_sign = float(g("camera_angle_sign"))
        self.camera_fov_rad = math.radians(float(g("camera_fov_deg")))
        self.require_identity_ready = bool(g("require_identity_ready"))
        self.camera_msg_timeout = float(g("camera_msg_timeout_sec"))
        self.compensate_latency = bool(g("compensate_camera_latency"))
        self.max_latency = float(g("max_camera_latency_sec"))

        self.use_lidar_distance = bool(g("use_lidar_distance"))
        self.assoc_window_rad = math.radians(float(g("assoc_window_deg")))
        self.cluster_gap_m = float(g("cluster_gap_m"))
        self.person_w_min = float(g("person_width_min_m"))
        self.person_w_max = float(g("person_width_max_m"))
        self.person_r_min = float(g("person_range_min_m"))
        self.person_r_max = float(g("person_range_max_m"))
        self.max_jump = float(g("lidar_distance_max_jump_m"))

        self.bbox_fallback = bool(g("bbox_fallback_enabled"))
        self.bbox_h_1m = float(g("bbox_height_at_1m_px"))

        self.lidar_only_enabled = bool(g("lidar_only_track_enabled"))
        self.lidar_only_radius = float(g("lidar_only_assoc_radius_m"))
        self.lidar_only_max_sec = float(g("lidar_only_max_sec"))

        self.predict_max_sec = float(g("predict_max_sec"))
        self.ab_alpha = float(g("ab_alpha"))
        self.ab_beta = float(g("ab_beta"))
        self.max_person_speed = float(g("max_person_speed"))

        self.rssi_enabled = bool(g("rssi_enabled"))
        self.rssi_angle_sign = float(g("rssi_angle_sign"))
        self.rssi_min_conf = float(g("rssi_min_confidence"))
        self.rssi_timeout = float(g("rssi_timeout_sec"))
        self.rssi_takeover_after = float(g("rssi_takeover_after_sec"))
        self.rssi_assumed_dist = float(g("rssi_assumed_distance_m"))
        self.rssi_blend = float(g("rssi_bearing_blend"))

        self.update_hz = float(g("update_hz"))
        self.publish_marker = bool(g("publish_marker"))
        self.log_period = float(g("log_period_sec"))

    # ─────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────

    def _cam_cb(self, msg: String) -> None:
        try:
            self.cam_payload = json.loads(msg.data)
            self.cam_time = time.time()
        except Exception as exc:
            self.get_logger().warning("JSON /person_reid/target hong: %s" % exc)

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan_msg = msg
        self.scan_time = time.time()
        pts, bear, rng, ldeg = scan_to_base_points(
            np.asarray(msg.ranges, dtype=np.float64),
            float(msg.angle_min),
            float(msg.angle_increment),
            float(msg.range_min),
            float(msg.range_max),
            self.lidar_x,
            self.lidar_y,
            self.lidar_yaw_offset_rad,
            self.lidar_angle_sign,
            self.lidar_max_use_range,
        )
        if self.self_filter and pts.shape[0]:
            keep = self_filter_mask(
                pts, ldeg, self.front_len, self.rear_len, self.half_width,
                self.self_margin, self.blind_sectors,
            )
            pts, bear, rng = pts[keep], bear[keep], rng[keep]
        self.scan_pts, self.scan_bear, self.scan_rng = pts, bear, rng

    def _odom_cb(self, msg: Odometry) -> None:
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.robot_yaw = yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        self.odom_ok = True

        now = time.time()
        self.yaw_history.append((now, self.robot_yaw))
        cutoff = now - 1.5
        while self.yaw_history and self.yaw_history[0][0] < cutoff:
            self.yaw_history.pop(0)

    def _rssi_angle_cb(self, msg: Float32) -> None:
        self.rssi_angle_deg = float(msg.data)
        self.rssi_time = time.time()

    def _rssi_conf_cb(self, msg: Float32) -> None:
        self.rssi_conf = float(msg.data)

    def _reset_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.filter.reset()
        self.tracking = False
        self.confidence = 0.0
        self.last_camera_fix_time = 0.0
        self.last_lidar_fix_time = 0.0
        res.success = True
        res.message = "Da xoa bo nho vi tri muc tieu"
        self.get_logger().info(res.message)
        return res

    # ─────────────────────────────────────────────────────────────────────
    # Cong cu
    # ─────────────────────────────────────────────────────────────────────

    def _yaw_at(self, t: float) -> float:
        """Yaw cua xe tai thoi diem t (noi suy tu lich su odom)."""
        if not self.yaw_history:
            return self.robot_yaw
        if t >= self.yaw_history[-1][0]:
            return self.yaw_history[-1][1]
        if t <= self.yaw_history[0][0]:
            return self.yaw_history[0][1]
        for i in range(len(self.yaw_history) - 1, 0, -1):
            t1, y1 = self.yaw_history[i]
            t0, y0 = self.yaw_history[i - 1]
            if t0 <= t <= t1:
                if t1 - t0 < 1e-6:
                    return y1
                k = (t - t0) / (t1 - t0)
                return y0 + k * wrap_pi(y1 - y0)
        return self.robot_yaw

    def _camera_bearing(self, payload: Dict[str, Any], now: float) -> Optional[float]:
        """Goc toi nguoi trong base_link HIEN TAI, da bu do tre xu ly anh.

        YOLO+DeepSORT+ReID tre khoang 80-250ms. Neu xe dang quay 0.5 rad/s thi
        250ms = 7 do sai lech — du de xe bam lech ra ngoai khung hinh.
        """
        a = payload.get("camera_angle_deg")
        if a is None:
            return None
        try:
            bearing = self.camera_angle_sign * math.radians(float(a))
        except Exception:
            return None

        if not self.compensate_latency:
            return wrap_pi(bearing)

        ts = payload.get("ts")
        if ts is None:
            return wrap_pi(bearing)
        lat = now - float(ts)
        if lat <= 0.0 or lat > self.max_latency:
            return wrap_pi(bearing)

        yaw_then = self._yaw_at(float(ts))
        delta_yaw = wrap_pi(self.robot_yaw - yaw_then)
        return wrap_pi(bearing - delta_yaw)

    def _bbox_distance(self, payload: Dict[str, Any]) -> Optional[float]:
        bbox = payload.get("bbox")
        if not isinstance(bbox, list) or len(bbox) != 4:
            return None
        try:
            h = abs(float(bbox[3]) - float(bbox[1]))
        except Exception:
            return None
        if h < 5.0 or self.bbox_h_1m <= 1.0:
            return None
        d = self.bbox_h_1m / h
        return d if 0.3 <= d <= 8.0 else None

    def _pick_person_cluster(
        self, bearing: float, expect_range: Optional[float]
    ) -> Optional[Cluster]:
        """Tim cum lidar ung voi nguoi tai huong `bearing`."""
        if self.scan_pts.shape[0] == 0:
            return None
        clusters = cluster_points(
            self.scan_pts, self.scan_bear, self.scan_rng,
            bearing, self.assoc_window_rad,
            gap_threshold_m=self.cluster_gap_m, min_points=2,
        )
        best: Optional[Cluster] = None
        best_cost = float("inf")
        for c in clusters:
            if not (self.person_r_min <= c.range_m <= self.person_r_max):
                continue
            if not (self.person_w_min <= c.width_m <= self.person_w_max):
                continue
            cost = abs(wrap_pi(c.bearing - bearing))
            if expect_range is not None:
                cost += 0.6 * min(2.0, abs(c.range_m - expect_range))
            if cost < best_cost:
                best_cost = cost
                best = c
        return best

    def _nearest_cluster_to(self, tx: float, ty: float) -> Optional[Cluster]:
        """Cum lidar gan diem du doan nhat (dung khi camera mat nhung chan con thay)."""
        if self.scan_pts.shape[0] == 0:
            return None
        bearing = math.atan2(ty, tx)
        rng = math.hypot(tx, ty)
        if rng < 0.15:
            return None
        window = max(self.assoc_window_rad, math.atan2(self.lidar_only_radius, max(0.3, rng)))
        clusters = cluster_points(
            self.scan_pts, self.scan_bear, self.scan_rng,
            bearing, window, gap_threshold_m=self.cluster_gap_m, min_points=2,
        )
        best, best_d = None, self.lidar_only_radius
        for c in clusters:
            if not (self.person_w_min <= c.width_m <= self.person_w_max):
                continue
            d = math.hypot(c.cx - tx, c.cy - ty)
            if d < best_d:
                best_d = d
                best = c
        return best

    def _to_odom(self, bx: float, by: float) -> tuple:
        cy, sy = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        return (self.robot_x + cy * bx - sy * by, self.robot_y + sy * bx + cy * by)

    def _to_base(self, ox: float, oy: float) -> tuple:
        dx = ox - self.robot_x
        dy = oy - self.robot_y
        cy, sy = math.cos(-self.robot_yaw), math.sin(-self.robot_yaw)
        return (cy * dx - sy * dy, sy * dx + cy * dy)

    # ─────────────────────────────────────────────────────────────────────
    # Vong cap nhat chinh
    # ─────────────────────────────────────────────────────────────────────

    def _update(self) -> None:
        now = time.time()
        if not self.odom_ok:
            self._publish(now, "wait_odom", None)
            return

        scan_fresh = (now - self.scan_time) < 0.5
        measured = False
        source = "none"
        meas_quality = 0.0

        # ── 1. CAMERA (+ LIDAR) ──────────────────────────────────────────
        payload = self.cam_payload or {}
        cam_fresh = (now - self.cam_time) < self.camera_msg_timeout
        cam_found = bool(payload.get("target_found", False))
        ident_ok = bool(payload.get("identity_ready", False)) or not self.require_identity_ready

        if cam_fresh and cam_found and ident_ok:
            bearing = self._camera_bearing(payload, now)
            if bearing is not None:
                dist: Optional[float] = None
                expect = None
                if self.filter.initialized:
                    px, py = self.filter.predict(now)
                    bx, by = self._to_base(px, py)
                    expect = math.hypot(bx, by)

                if self.use_lidar_distance and scan_fresh:
                    c = self._pick_person_cluster(bearing, expect)
                    if c is not None:
                        dist = c.range_m
                        # dung luon bearing cua cum — chinh xac hon ca camera o gan
                        bearing = c.bearing
                        source = "camera+lidar"
                        meas_quality = 1.0

                if dist is None and self.bbox_fallback:
                    dist = self._bbox_distance(payload)
                    if dist is not None:
                        source = "camera+bbox"
                        meas_quality = 0.55

                if dist is not None:
                    bx = dist * math.cos(bearing)
                    by = dist * math.sin(bearing)
                    ox, oy = self._to_odom(bx, by)
                    self.filter.update(ox, oy, now)
                    self.last_camera_fix_time = now
                    if "lidar" in source:
                        self.last_lidar_fix_time = now
                    measured = True

        # ── 2. LIDAR-ONLY (camera bi che, chan nguoi con thay) ───────────
        if not measured and self.lidar_only_enabled and scan_fresh and self.filter.initialized:
            age_cam = now - self.last_camera_fix_time
            if age_cam <= self.lidar_only_max_sec:
                px, py = self.filter.predict(now)
                bx, by = self._to_base(px, py)
                c = self._nearest_cluster_to(bx, by)
                if c is not None:
                    ox, oy = self._to_odom(c.cx, c.cy)
                    self.filter.update(ox, oy, now)
                    self.last_lidar_fix_time = now
                    source = "lidar_track"
                    meas_quality = 0.75
                    measured = True

        # ── 3. RSSI sua huong khi mat lau ────────────────────────────────
        rssi_fresh = self.rssi_enabled and (now - self.rssi_time) < self.rssi_timeout
        rssi_ok = rssi_fresh and self.rssi_conf >= self.rssi_min_conf
        if not measured and rssi_ok and self.filter.initialized:
            if (now - self.last_camera_fix_time) >= self.rssi_takeover_after:
                rb = self.rssi_angle_sign * math.radians(self.rssi_angle_deg)
                px, py = self.filter.predict(now)
                bx, by = self._to_base(px, py)
                cur_b = math.atan2(by, bx)
                cur_r = max(0.5, math.hypot(bx, by))
                new_b = cur_b + self.rssi_blend * wrap_pi(rb - cur_b)
                ox, oy = self._to_odom(cur_r * math.cos(new_b), cur_r * math.sin(new_b))
                self.filter.update(ox, oy, now)
                source = "rssi_bearing"
                meas_quality = 0.30
                measured = True

        # ── 4. Du doan thuan tuy ─────────────────────────────────────────
        if not measured:
            self.filter.step(now)

        # ── Tinh do tin cay ──────────────────────────────────────────────
        if not self.filter.initialized:
            self.tracking = False
            self.confidence = 0.0
            self._publish(now, "no_target_yet", None)
            return

        age_fix = now - max(self.last_camera_fix_time, self.last_lidar_fix_time)
        if measured:
            self.confidence = max(self.confidence, meas_quality)
            self.confidence = 0.65 * self.confidence + 0.35 * meas_quality
        else:
            decay = max(0.0, 1.0 - age_fix / max(0.2, self.predict_max_sec))
            self.confidence = min(self.confidence, decay)

        if age_fix > (self.predict_max_sec + self.lidar_only_max_sec):
            self.tracking = False
            self.filter.reset()
            self._publish(now, "target_lost", None)
            return

        self.tracking = True
        self.source = source if measured else "predicted"
        self._publish(now, "ok", self.source, age_fix=age_fix, measured=measured)

    # ─────────────────────────────────────────────────────────────────────
    # Xuat du lieu
    # ─────────────────────────────────────────────────────────────────────

    def _publish(
        self,
        now: float,
        status: str,
        source: Optional[str],
        age_fix: float = 999.0,
        measured: bool = False,
    ) -> None:
        out: Dict[str, Any] = {
            "stamp": now,
            "status": status,
            "valid": bool(self.tracking and self.filter.initialized),
            "source": source or "none",
            "measured_this_tick": bool(measured),
            "confidence": round(float(self.confidence), 3),
            "age_since_fix_sec": round(float(age_fix), 3) if age_fix < 900 else None,
            "odom_frame": self.odom_frame,
            "robot": {
                "x": round(self.robot_x, 4),
                "y": round(self.robot_y, 4),
                "yaw": round(self.robot_yaw, 4),
            },
        }

        if self.tracking and self.filter.initialized:
            tx, ty = self.filter.predict(now)
            bx, by = self._to_base(tx, ty)
            bearing = math.atan2(by, bx)
            dist = math.hypot(bx, by)
            in_fov = abs(bearing) <= (self.camera_fov_rad * 0.5)
            out.update({
                "odom_x": round(tx, 4),
                "odom_y": round(ty, 4),
                "vx": round(self.filter.vx, 3),
                "vy": round(self.filter.vy, 3),
                "speed": round(math.hypot(self.filter.vx, self.filter.vy), 3),
                "base_x": round(bx, 4),
                "base_y": round(by, 4),
                "distance_m": round(dist, 3),
                "bearing_rad": round(bearing, 4),
                "bearing_deg": round(math.degrees(bearing), 2),
                "in_camera_fov": bool(in_fov),
            })
        else:
            out.update({
                "odom_x": None, "odom_y": None, "vx": 0.0, "vy": 0.0, "speed": 0.0,
                "base_x": None, "base_y": None, "distance_m": None,
                "bearing_rad": None, "bearing_deg": None, "in_camera_fov": False,
            })

        self.pub_target.publish(String(data=json.dumps(out, ensure_ascii=False)))

        if self.publish_marker:
            self._publish_marker(out)

        if now - self.last_log > self.log_period:
            if out["valid"]:
                self.get_logger().info(
                    "muc tieu: d=%.2fm goc=%.1fdeg nguon=%s conf=%.2f v=%.2fm/s %s"
                    % (
                        out["distance_m"], out["bearing_deg"], out["source"],
                        out["confidence"], out["speed"],
                        "" if out["in_camera_fov"] else "[NGOAI KHUNG HINH]",
                    )
                )
            else:
                self.get_logger().info("muc tieu: %s" % status)
            self.last_log = now

    def _publish_marker(self, out: Dict[str, Any]) -> None:
        m = Marker()
        m.header.frame_id = self.odom_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "follow_target"
        m.id = 0
        m.type = Marker.CYLINDER
        if not out["valid"]:
            m.action = Marker.DELETE
            self.pub_marker.publish(m)
            return
        m.action = Marker.ADD
        m.pose.position = Point(x=float(out["odom_x"]), y=float(out["odom_y"]), z=0.5)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.35
        m.scale.y = 0.35
        m.scale.z = 1.0
        c = float(out["confidence"])
        m.color = ColorRGBA(r=float(1.0 - c), g=float(c), b=0.2, a=0.75)
        self.pub_marker.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
