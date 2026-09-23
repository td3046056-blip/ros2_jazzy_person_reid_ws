"""
follow_planner_node.py
======================
Node 2/2. La NGUON DUY NHAT publish /cmd_vel — day la diem quan trong nhat.

Hien tai trong workspace cua ban co HAI node cung ghi /cmd_vel
(person_follow_controller va rssi_follow_node). Hai node cung ghi mot topic thi
ROS khong "tron" lenh — xe nhan xen ke lenh cua ca hai, ket qua la giat cuc.
Chinh comment v1.9 trong follow_controller.py cua ban da mo ta hien tuong nay.
Node nay thay the ca hai.

THUAT TOAN: DWA (Dynamic Window Approach) co cai bien cho bai toan bam nguoi.

  1. Lay muc tieu tu /follow/target (da o khung odom, song duoc qua luc bi che)
  2. Dat GOAL khong phai tai nguoi ma LUI LAI follow_distance_m trước nguoi
     => xe tu dong dung dung khoang cach, va KHONG can loai nguoi ra khoi
        danh sach vat can. Day la cach sua sach se cho Bug#4 trong ghi chu cua ban
        ("exclude target khien xe lao vao vat can phia sau nguoi").
  3. Sinh ~100 cap (v, w) trong cua so gia toc cho phep
  4. Mo phong 1.2s cho tung cap, kiem tra va cham bang FOOTPRINT CHU NHAT THAT
     (46 x 57 cm), khong phai hinh tron => chui duoc khe hep hon nhieu
  5. Cham diem: tien toi dich + do thoang + giu nguoi trong khung hinh camera
     + muot (phat thay doi dot ngot) + giu huong ne da chon
  6. Chon diem cao nhat, gioi han gia toc, publish

BIEN PHAP CHO CAMERA CO DINH (khong xoay duoc):
  - cost_fov: phat nhung quy dao lam nguoi ra khoi +-24 do => xe uu tien ne ve
    phia van "liec" thay nguoi
  - Khi bat buoc phai ne sang ben lam mat nguoi, bo nho odom giu vi tri nguoi,
    xe ne xong tu dong quay lai dung huong

CHO KHONG GIAN HEP:
  - Hai nguong: margin_soft (mong muon, vd 0.22m) va margin_hard (tuyet doi,
    vd 0.07m). Cho thoang xe giu 22cm; hanh lang hep xe tu dong ep xuong 7cm
    vi luc keo cua muc tieu thang luc day cua vat can. KHONG can doi mode.
  - Toc do tu dong giam theo do thoang => cho hep di cham, cho rong di nhanh.

Subscribe:  /follow/target (String JSON), /scan (LaserScan), /odom (Odometry)
Publish:    /cmd_vel (Twist), /follow/planner_status (String JSON),
            /follow/debug_markers (MarkerArray)
Services:   /follow/enable, /follow/disable, /follow/stop
"""

from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import (
    downsample_polar,
    self_filter_mask,
    rect_clearance,
    scan_to_base_points,
    segment_blocked,
    wrap_pi,
    yaw_from_quaternion,
)

# Cac trang thai cua may trang thai
S_IDLE = "IDLE"
S_FOLLOW = "FOLLOW"
S_AVOID = "AVOID"
S_OCCLUDED = "OCCLUDED"
S_SEARCH = "SEARCH"
S_ARRIVED = "ARRIVED"
S_BLOCKED = "BLOCKED"
S_ESTOP = "ESTOP"


class FollowPlannerNode(Node):

    def __init__(self) -> None:
        super().__init__("follow_planner_node")
        self._declare_params()
        self._read_params()

        self.enabled = bool(self.start_enabled)
        self.state = S_IDLE
        self.prev_state = S_IDLE
        self.state_since = time.time()

        self.cur_v = 0.0
        self.cur_w = 0.0

        self.target: Dict[str, Any] = {}
        self.target_time = 0.0
        self.last_valid_time = 0.0          # lan cuoi co muc tieu HOP LE (0 = chua/het quet)
        self.occl_turning = False           # dang xoay ve huong nho cuoi (co tre)
        self.last_target_odom: Optional[Tuple[float, float]] = None   # cho thay nguoi lan cuoi
        self.search_goto = False            # SEARCH dang o pha lai toi cho do
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.scan_pts = np.zeros((0, 2))
        self.scan_time = 0.0
        self.obstacles = np.zeros((0, 2))
        self.odom_ok = False
        self.robot_yaw = 0.0

        self.avoid_side = 0                 # -1 phai, +1 trai, 0 chua chon
        self.avoid_side_time = 0.0
        self.blocked_since = 0.0
        self.clear_since = 0.0
        self.search_dir = 1.0
        self.last_heading = 0.0        # huong khe da chon lan truoc (chong do du)
        self.search_start = 0.0
        self.last_log = 0.0
        self.last_seen_bearing = 0.0

        # Mang thoi gian mo phong (tinh truoc cho nhanh)
        self.t_arr = np.arange(1, self.horizon_steps + 1, dtype=np.float64) * self.horizon_dt

        self.create_subscription(String, self.target_topic, self._target_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)

        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.pub_markers = self.create_publisher(MarkerArray, self.markers_topic, 5)

        self.create_service(Trigger, "/follow/enable", self._enable_cb)
        self.create_service(Trigger, "/follow/disable", self._disable_cb)
        self.create_service(Trigger, "/follow/stop", self._stop_cb)

        self.create_timer(1.0 / max(1.0, self.control_hz), self._control)

        self.get_logger().info(
            "follow_planner_node san sang | footprint %.2fx%.2fm | v_max=%.2f w_max=%.2f | "
            "khoang cach bam=%.2fm | margin mem=%.2f cung=%.2f | enabled=%s"
            % (
                self.front_len + self.rear_len, self.half_width * 2.0,
                self.v_max, self.w_max, self.follow_distance,
                self.margin_soft, self.margin_hard, self.enabled,
            )
        )
        self.get_logger().info(
            "Bat bam: ros2 service call /follow/enable std_srvs/srv/Trigger {}"
        )

    # ─────────────────────────────────────────────────────────────────────
    # Tham so
    # ─────────────────────────────────────────────────────────────────────

    def _declare_params(self) -> None:
        d: Dict[str, Any] = {
            "target_topic": "/follow/target",
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "cmd_vel_topic": "/cmd_vel",
            "status_topic": "/follow/planner_status",
            "markers_topic": "/follow/debug_markers",
            "base_frame": "base_link",

            "start_enabled": False,
            "control_hz": 15.0,

            # Hinh hoc lidar (phai trung voi target_tracker_node)
            "lidar_yaw_offset_deg": -90.0,
            "lidar_angle_sign": 1.0,
            "lidar_x": 0.10,
            "lidar_y": 0.0,
            "lidar_max_use_range": 5.0,
            "scan_timeout_sec": 0.5,
            "obstacle_bucket_deg": 2.0,
            "max_obstacle_points": 160,
            # Loc tia dap vao than xe. TAT = xe co the ket vinh vien o BLOCKED.
            # Chay `calibrate_lidar --ros-args -p mode:=self_scan` de lay so.
            "self_filter_enabled": True,
            "self_filter_margin": 0.03,
            "blind_sectors_deg": [0.0],

            # Footprint that cua BW-DR03: 46cm ngang x 57cm doc
            "front_len": 0.14,          # tu tam base_link toi mui (+ them 1.5cm du)
            "rear_len": 0.33,           # tu tam base_link toi duoi
            "half_width": 0.30,         # nua be ngang
            "margin_soft": 0.15,        # khoang cach MONG MUON toi vat can
            "margin_hard": 0.06,        # khoang cach TUYET DOI khong duoc pham
            "rotate_radius": 0.47,      # ban kinh can thoang de xoay tai cho

            # Gioi han dong hoc — toc do THAT (driver da hieu chinh max_linear 0.49)
            "v_max": 0.22,
            "v_min": -0.06,             # chi dung khi thoat ket
            "w_max": 0.80,
            "accel_lin": 0.35,
            "accel_ang": 1.60,
            "min_move_linear": 0.035,   # duoi muc nay banh xe khong quay (deadband driver)
            "min_move_angular": 0.10,

            # Lay mau DWA
            "n_samples_v": 7,
            "n_samples_w": 17,
            "horizon_steps": 12,
            "horizon_dt": 0.10,
            # Cua so lay mau van toc = accel * sample_window_sec.
            # KHONG duoc de bang 1/control_hz: khi do cua so chi rong
            # 0.35*0.067 = 0.023 m/s, loi ich tien len trong 1.2s chi la 2.8cm,
            # nho hon chi phi "muot" => DWA luon chon v=0 va xe DUNG YEN VINH VIEN.
            # Gioi han gia toc that van duoc ap o khau _emit().
            "sample_window_sec": 0.5,

            # Trong so ham chi phi
            "w_goal": 2.4,
            "w_heading": 1.0,
            "w_clear": 2.2,
            "w_speed": 0.55,
            "w_smooth": 0.70,
            "w_fov": 1.6,
            "w_side": 0.9,

            # Hanh vi bam
            "follow_distance_m": 1.00,
            "distance_deadband_m": 0.12,
            "bearing_deadband_deg": 4.0,
            "max_follow_distance_m": 6.0,

            # Camera co dinh
            "camera_fov_deg": 62.0,
            "fov_keep_deg": 22.0,       # co gang giu nguoi trong +-22 do
            "fov_cost_only_when_visible": True,
            # Camera khong thay nguoi (lidar_track/predicted) va nguoi lech hon goc nay ->
            # xoay tai cho ve huong nho cuoi truoc, xuong duoi mot nua goc nay moi tien toi.
            # 0 = tat.
            "occluded_turn_deg": 15.0,
            # Kiem tra duong bi chan toi TAN CHO NGUOI (tru ban kinh nay quanh ho), khong
            # chi toi dich. Dich cach nguoi follow_distance nen nguoi thu hai dung chen
            # thuong dung dung tai dich -> kiem tra toi dich thoi thi khong thay chan.
            "target_clear_radius_m": 0.45,

            # Chuyen trang thai
            # Tang chon khe (VFH) chay truoc DWA
            "probe_distances": [1.6, 1.1, 0.7, 0.45],
            "min_speed_scale": 0.25,

            "block_corridor_scale": 1.15,   # hanh lang kiem tra = half_width * scale
            "block_enter_sec": 0.25,
            "block_exit_sec": 0.60,
            "avoid_side_hold_sec": 2.0,
            "stuck_time_sec": 2.5,
            "search_after_sec": 1.5,
            # Mat nguoi -> truoc khi xoay quet, LAI TOI cho thay nguoi lan cuoi (khung
            # odom) toi da bay nhieu giay. Nguoi re khuat sau goc tuong thi phai toi goc
            # moi nhin thay duoc. 0 = tat (cho search_after_sec roi xoay tai cho).
            "search_goto_max_sec": 8.0,
            "search_w": 0.35,
            "search_max_sec": 12.0,
            "min_confidence": 0.15,
            "target_msg_timeout_sec": 0.8,

            "publish_markers": True,
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

        self.target_topic = str(g("target_topic"))
        self.scan_topic = str(g("scan_topic"))
        self.odom_topic = str(g("odom_topic"))
        self.cmd_vel_topic = str(g("cmd_vel_topic"))
        self.status_topic = str(g("status_topic"))
        self.markers_topic = str(g("markers_topic"))
        self.base_frame = str(g("base_frame"))

        self.start_enabled = bool(g("start_enabled"))
        self.control_hz = float(g("control_hz"))

        self.lidar_yaw_offset_rad = math.radians(float(g("lidar_yaw_offset_deg")))
        self.lidar_angle_sign = float(g("lidar_angle_sign"))
        self.lidar_x = float(g("lidar_x"))
        self.lidar_y = float(g("lidar_y"))
        self.lidar_max_use_range = float(g("lidar_max_use_range"))
        self.scan_timeout = float(g("scan_timeout_sec"))
        self.bucket_deg = float(g("obstacle_bucket_deg"))
        self.max_obs_pts = int(g("max_obstacle_points"))
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
        self.margin_soft = float(g("margin_soft"))
        self.margin_hard = float(g("margin_hard"))
        self.rotate_radius = float(g("rotate_radius"))

        self.v_max = float(g("v_max"))
        self.v_min = float(g("v_min"))
        self.w_max = float(g("w_max"))
        self.accel_lin = float(g("accel_lin"))
        self.accel_ang = float(g("accel_ang"))
        self.min_move_v = float(g("min_move_linear"))
        self.min_move_w = float(g("min_move_angular"))

        self.n_v = max(3, int(g("n_samples_v")))
        self.n_w = max(5, int(g("n_samples_w")))
        self.horizon_steps = max(4, int(g("horizon_steps")))
        self.horizon_dt = float(g("horizon_dt"))
        self.sample_window = max(0.05, float(g("sample_window_sec")))
        self.horizon_T = self.horizon_steps * self.horizon_dt

        self.w_goal = float(g("w_goal"))
        self.w_heading = float(g("w_heading"))
        self.w_clear = float(g("w_clear"))
        self.w_speed = float(g("w_speed"))
        self.w_smooth = float(g("w_smooth"))
        self.w_fov = float(g("w_fov"))
        self.w_side = float(g("w_side"))

        self.follow_distance = float(g("follow_distance_m"))
        self.dist_deadband = float(g("distance_deadband_m"))
        self.bearing_deadband = math.radians(float(g("bearing_deadband_deg")))
        self.max_follow_dist = float(g("max_follow_distance_m"))

        self.camera_fov_rad = math.radians(float(g("camera_fov_deg")))
        self.fov_keep_rad = math.radians(float(g("fov_keep_deg")))
        self.fov_only_visible = bool(g("fov_cost_only_when_visible"))
        self.occluded_turn_rad = math.radians(float(g("occluded_turn_deg")))
        self.target_clear_r = float(g("target_clear_radius_m"))

        self.probe_distances = [float(x) for x in g("probe_distances")]
        self.min_speed_scale = float(g("min_speed_scale"))
        self.block_corridor_scale = float(g("block_corridor_scale"))
        self.block_enter_sec = float(g("block_enter_sec"))
        self.block_exit_sec = float(g("block_exit_sec"))
        self.avoid_hold = float(g("avoid_side_hold_sec"))
        self.stuck_time = float(g("stuck_time_sec"))
        self.search_after = float(g("search_after_sec"))
        self.search_goto_max = float(g("search_goto_max_sec"))
        self.search_w = float(g("search_w"))
        self.search_max = float(g("search_max_sec"))
        self.min_conf = float(g("min_confidence"))
        self.target_timeout = float(g("target_msg_timeout_sec"))

        self.publish_markers = bool(g("publish_markers"))
        self.log_period = float(g("log_period_sec"))

    # ─────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────

    def _target_cb(self, msg: String) -> None:
        try:
            self.target = json.loads(msg.data)
            self.target_time = time.time()
        except Exception:
            pass

    def _scan_cb(self, msg: LaserScan) -> None:
        pts, bear, rng, ldeg = scan_to_base_points(
            np.asarray(msg.ranges, dtype=np.float64),
            float(msg.angle_min), float(msg.angle_increment),
            float(msg.range_min), float(msg.range_max),
            self.lidar_x, self.lidar_y,
            self.lidar_yaw_offset_rad, self.lidar_angle_sign,
            self.lidar_max_use_range,
        )
        if self.self_filter and pts.shape[0]:
            keep = self_filter_mask(
                pts, ldeg, self.front_len, self.rear_len, self.half_width,
                self.self_margin, self.blind_sectors,
            )
            n_drop = int(pts.shape[0] - keep.sum())
            if n_drop and (time.time() - getattr(self, "_last_sf_log", 0.0)) > 10.0:
                self._last_sf_log = time.time()
                self.get_logger().info("self-filter: bo %d/%d tia dap vao than xe"
                                       % (n_drop, pts.shape[0]))
            pts, bear, rng = pts[keep], bear[keep], rng[keep]
        self.scan_pts = pts
        obs = downsample_polar(pts, bear, rng, self.bucket_deg, keep_all_within_m=1.0)
        if obs.shape[0] > self.max_obs_pts:
            d = np.hypot(obs[:, 0], obs[:, 1])
            keep = np.argsort(d)[: self.max_obs_pts]
            obs = obs[keep]
        self.obstacles = obs
        self.scan_time = time.time()

    def _odom_cb(self, msg: Odometry) -> None:
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.robot_yaw = yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        self.odom_ok = True

    def _enable_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.enabled = True
        # Bat lai la bat dau moi: khong lai toi cho nguoi cua lan bam truoc
        self.last_valid_time = 0.0
        self.last_target_odom = None
        res.success = True
        res.message = "Bam nguoi: BAT"
        self.get_logger().info(res.message)
        return res

    def _disable_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.enabled = False
        self._hard_stop()
        res.success = True
        res.message = "Bam nguoi: TAT"
        self.get_logger().info(res.message)
        return res

    def _stop_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.enabled = False
        self._hard_stop()
        res.success = True
        res.message = "DUNG KHAN CAP"
        self.get_logger().warn(res.message)
        return res

    # ─────────────────────────────────────────────────────────────────────
    # Lenh dieu khien
    # ─────────────────────────────────────────────────────────────────────

    def _hard_stop(self) -> None:
        self.cur_v = 0.0
        self.cur_w = 0.0
        t = Twist()
        for _ in range(3):
            self.pub_cmd.publish(t)

    def _emit(self, v: float, w: float, dt: float) -> Tuple[float, float]:
        """Gioi han gia toc + bu deadband driver roi publish."""
        dv = self.accel_lin * dt
        dw = self.accel_ang * dt
        v = float(np.clip(v, self.cur_v - dv, self.cur_v + dv))
        w = float(np.clip(w, self.cur_w - dw, self.cur_w + dw))
        v = float(np.clip(v, self.v_min, self.v_max))
        w = float(np.clip(w, -self.w_max, self.w_max))

        out_v, out_w = v, w
        # BW-DR03 co deadband 0.02 + PWM toi thieu: lenh qua nho = banh khong quay,
        # xe se "ri roi dung" nhin rat giat. Nang len nguong toi thieu.
        if 1e-4 < abs(out_v) < self.min_move_v:
            out_v = math.copysign(self.min_move_v, out_v)
        if 1e-4 < abs(out_w) < self.min_move_w:
            out_w = math.copysign(self.min_move_w, out_w)
        if abs(out_v) < 1e-4:
            out_v = 0.0
        if abs(out_w) < 1e-4:
            out_w = 0.0

        msg = Twist()
        msg.linear.x = out_v
        msg.angular.z = out_w
        self.pub_cmd.publish(msg)

        self.cur_v = v
        self.cur_w = w
        return out_v, out_w

    # ─────────────────────────────────────────────────────────────────────
    # Nhan thuc
    # ─────────────────────────────────────────────────────────────────────

    def _scan_fresh(self, now: float) -> bool:
        return (now - self.scan_time) < self.scan_timeout

    def _min_obstacle_range(self) -> float:
        if self.obstacles.shape[0] == 0:
            return float("inf")
        return float(np.min(np.hypot(self.obstacles[:, 0], self.obstacles[:, 1])))

    def _front_clearance(self) -> float:
        """Do thoang truoc mat theo footprint (dung de bao cao / dung khan)."""
        if self.obstacles.shape[0] == 0:
            return float("inf")
        c = rect_clearance(
            self.obstacles[:, 0], self.obstacles[:, 1],
            self.front_len, self.rear_len, self.half_width,
        )
        front = self.obstacles[:, 0] > 0.0
        if not np.any(front):
            return float("inf")
        return float(np.min(c[front]))

    def _can_rotate_in_place(self) -> bool:
        if self.obstacles.shape[0] == 0:
            return True
        r = np.hypot(self.obstacles[:, 0], self.obstacles[:, 1])
        return bool(np.min(r) > (self.rotate_radius + self.margin_hard))

    def _choose_heading(
        self, goal_bearing: float, goal_dist: float, target_bearing: float
    ) -> Tuple[Optional[float], float, bool]:
        """Tang CHON KHE (kieu VFH+) dat TRUOC DWA.

        Tai sao can tang nay: DWA thuan tuy chi nhin truoc 1.2s (~0.26m o toc do
        cua xe ban). Khi co nguoi dung chan giua duong, DWA thay "di thang thi
        dung, re thi cung khong gan dich hon" => no chon dung yen. Do la bay
        cuc tieu dia phuong kinh dien cua DWA.

        Tang nay quet cac huong tu -100 den +100 do, kiem tra huong nao di duoc
        MOT DOAN DAI (tuy chon 1.5m) ma khong cham footprint, roi chon huong
        re nhat. DWA sau do chi con viec lai cho muot toi huong da chon.

        Returns: (goc_chon, khoang_cach_probe, co_di_thang_duoc_khong)
        """
        corridor = self.half_width + self.margin_hard
        # Buoc 2 do chu khong phai 4 do: sai so 4 do o tam 1.6m = 11cm lech ngang,
        # bang gan het le an toan khi chui khe hep.
        cand = np.radians(np.arange(-100.0, 100.1, 2.0))

        direct_free = False
        for probe in self.probe_distances:
            reach = min(probe, max(0.35, goal_dist))
            free: list = []
            for phi in cand:
                gx = reach * math.cos(phi)
                gy = reach * math.sin(phi)
                blocked, _ = segment_blocked(self.obstacles, gx, gy, corridor)
                if not blocked:
                    free.append(float(phi))
            if not free:
                continue

            fa = np.array(free)
            if np.min(np.abs(wrap_pi_arr(fa - goal_bearing))) < math.radians(5.0):
                direct_free = True

            # Gom cac huong tu do lien tuc thanh KHE, roi nham vao giua khe.
            # Neu chi chon huong tu do GAN DICH NHAT thi xe bam mep khe: o khe
            # rong khong sao, nhung khe hep thi le an toan bi an het va xe ket.
            step_rad = math.radians(2.0) * 1.5
            groups, cur = [], [fa[0]]
            for a in fa[1:]:
                if a - cur[-1] <= step_rad:
                    cur.append(a)
                else:
                    groups.append(cur); cur = [a]
            groups.append(cur)
            aims = []
            for gp in groups:
                lo, hi = gp[0], gp[-1]
                half = (hi - lo) / 2.0
                # khe rong -> bam sat huong dich; khe hep -> ve giua khe
                pad = min(half, max(0.15 * half, math.radians(8.0)))
                aims.append(float(np.clip(goal_bearing, lo + pad, hi - pad)))
            fa = np.array(aims)

            # Chi phi cho tung huong tu do
            c_goal = np.abs(wrap_pi_arr(fa - goal_bearing)) / math.pi
            c_prev = np.abs(wrap_pi_arr(fa - self.last_heading)) / math.pi
            # Giu nguoi trong khung hinh camera co dinh
            c_fov = np.clip(
                (np.abs(wrap_pi_arr(fa - target_bearing)) - self.fov_keep_rad)
                / max(1e-3, math.pi - self.fov_keep_rad), 0.0, 1.0,
            )
            if self.avoid_side != 0:
                c_side = np.where(np.sign(fa) == -self.avoid_side, 0.5, 0.0)
            else:
                c_side = np.zeros(fa.shape[0])

            cost = 2.2 * c_goal + 0.45 * c_prev + 0.8 * c_fov + 0.7 * c_side
            best = int(np.argmin(cost))
            phi = float(fa[best])
            self.last_heading = phi
            return phi, reach, direct_free

        return None, 0.0, False

    def _gap_side(self, goal_bearing: float, reach_dist: float, target_bearing: float) -> int:
        """Ben ne theo KHE DI DUOC that su: +1 trai, -1 phai, 0 = khong ro.

        Chay tang chon khe khi CHUA thien vi ben nao, xem huong tot nhat lech ve phia nao so
        voi huong dich. Tang chon khe kiem tra hanh lang rong bang xe nen biet khe cua o dau.
        _choose_avoid_side chi so khoang trong trung binh hai ben — o khung cua hai ben gan
        bang nhau, no roi vao nhanh "theo dau goc toi nguoi", tuc tung dong xu.
        """
        saved_side, saved_prev = self.avoid_side, self.last_heading
        self.avoid_side = 0
        phi, _reach, _direct = self._choose_heading(goal_bearing, reach_dist, target_bearing)
        self.avoid_side, self.last_heading = saved_side, saved_prev
        if phi is None:
            return 0
        d = wrap_pi(phi - goal_bearing)
        if abs(d) < math.radians(2.0):
            return 0
        return 1 if d > 0 else -1

    def _choose_avoid_side(self, target_bearing: float) -> int:
        """Chon ne TRAI (+1) hay PHAI (-1).

        Tieu chi, theo thu tu quan trong:
          1. Ben nao co khe du rong cho xe 46cm chui qua
          2. Ben nao giu duoc nguoi trong khung hinh camera lau hon
          3. Ben nao lech it hon so voi huong nguoi
        """
        if self.obstacles.shape[0] == 0:
            return 1 if target_bearing >= 0 else -1

        bear = np.arctan2(self.obstacles[:, 1], self.obstacles[:, 0])
        rng = np.hypot(self.obstacles[:, 0], self.obstacles[:, 1])

        def side_score(sign: int) -> float:
            lo, hi = (0.12, 1.30) if sign > 0 else (-1.30, -0.12)
            sel = (bear >= lo) & (bear <= hi)
            if not np.any(sel):
                free = 4.0
            else:
                free = float(np.mean(np.sort(rng[sel])[: max(1, int(sel.sum() * 0.3))]))
            # thuong cho ben giup nguoi o gan tam khung hinh hon
            fov_bonus = 0.8 * math.cos(wrap_pi(target_bearing - sign * 0.45))
            return free + fov_bonus

        left = side_score(+1)
        right = side_score(-1)
        if abs(left - right) < 0.12:
            return 1 if target_bearing >= 0 else -1
        return 1 if left > right else -1

    # ─────────────────────────────────────────────────────────────────────
    # DWA
    # ─────────────────────────────────────────────────────────────────────

    def _dwa(
        self,
        goal_xy: Tuple[float, float],
        target_xy: Optional[Tuple[float, float]],
        dt_ctrl: float,
        allow_reverse: bool = False,
        fov_active: bool = True,
    ) -> Optional[Tuple[float, float, float]]:
        """Tra ve (v, w, clearance) tot nhat, hoac None neu khong co lua chon an toan."""
        gx, gy = goal_xy

        # 1. Cua so dong hoc — rong bang accel * sample_window, KHONG phai accel * dt_ctrl.
        #    Xem ghi chu o tham so sample_window_sec: cua so 1-tick lam xe dung yen.
        dv = self.accel_lin * self.sample_window
        dw = self.accel_ang * self.sample_window
        v_lo = max(self.v_min if allow_reverse else 0.0, self.cur_v - dv)
        v_hi = min(self.v_max, self.cur_v + dv)
        w_lo = max(-self.w_max, self.cur_w - dw)
        w_hi = min(self.w_max, self.cur_w + dw)
        if v_hi < v_lo:
            v_hi = v_lo

        vs = np.unique(np.concatenate([np.linspace(v_lo, v_hi, self.n_v), [0.0]]))
        ws = np.linspace(w_lo, w_hi, self.n_w)
        V, W = np.meshgrid(vs, ws, indexing="ij")
        V = V.ravel()
        W = W.ravel()
        n = V.shape[0]

        # 2. Mo phong quy dao (mo hinh xe 2 banh vi sai)
        t = self.t_arr
        theta = W[:, None] * t[None, :]                       # (N,K)
        small = np.abs(W) < 1e-4
        Wsafe = np.where(small, 1e-4, W)
        R = V / Wsafe
        X = np.where(small[:, None], V[:, None] * t[None, :], R[:, None] * np.sin(theta))
        Y = np.where(small[:, None], 0.0, R[:, None] * (1.0 - np.cos(theta)))

        # 3. Do thoang theo footprint chu nhat that
        P = self.obstacles
        if P.shape[0] > 0:
            dx = P[None, None, :, 0] - X[:, :, None]
            dy = P[None, None, :, 1] - Y[:, :, None]
            c = np.cos(theta)[:, :, None]
            s = np.sin(theta)[:, :, None]
            lx = c * dx + s * dy
            ly = -s * dx + c * dy
            clear = rect_clearance(lx, ly, self.front_len, self.rear_len, self.half_width)
            min_clear = clear.min(axis=(1, 2))                # (N,)
        else:
            min_clear = np.full(n, 10.0)

        admissible = min_clear > self.margin_hard
        if not np.any(admissible):
            return None

        # 4. Ham chi phi (cang nho cang tot)
        xe = X[:, -1]
        ye = Y[:, -1]
        te = theta[:, -1]

        d_start = math.hypot(gx, gy)
        d_end = np.hypot(gx - xe, gy - ye)
        d_start_pre, d_end_pre = d_start, d_end
        # Chi phi dich = "khong tien duoc bao nhieu so voi muc tien toi da co the".
        # Chuan hoa theo quang duong toi da (v_max * horizon) chu KHONG theo
        # khoang cach con lai — neu chuan hoa theo khoang cach con lai thi dich
        # cang xa tin hieu cang yeu, DWA se ngoi im.
        max_prog = max(1e-3, self.v_max * self.horizon_T)
        c_goal = np.clip(1.0 - (d_start - d_end) / max_prog, 0.0, 2.0)

        ang_to_goal = np.arctan2(gy - ye, gx - xe)
        c_head = np.abs(wrap_pi_arr(ang_to_goal - te)) / math.pi

        # ── MARGIN THICH NGHI — chia khoa cho "toi uu khong gian hep" ────────
        # O cho rong: soft = 0.22m, xe giu khoang cach thoai mai voi tuong.
        # Trong khe hep chi cho phep 0.13m: neu van doi 0.22m thi di qua khe luon
        # bi phat, con "queo ra cho rong" thi khong bi phat => xe se queo di roi
        # dung truoc khe. Nen ta ha soft xuong muc TOT NHAT MA CAC QUY DAO CO
        # TIEN VE DICH dat duoc.
        #
        # CHU Y quan trong: chi lay mau trong nhom CO TIEN VE DICH. Neu lay ca
        # nhung quy dao queo ra cho rong (thoang 0.4m) thi nguong tham chieu bi
        # keo len 0.4m, khe hep 0.13m van bi phat nang, va xe van dung truoc khe.
        # Lay mau tu nhom TOP 10% quy dao TOI GAN DICH PHU NHAT, roi lay do
        # thoang TOT NHAT trong nhom do. Cau hoi dung phai la: "neu toi that su
        # di vao khe nay, nhieu nhat toi duoc bao nhieu cho trong?"
        #
        # Loc theo "co tien ve dich" thoi thi KHONG DU: quy dao queo 20 do van
        # tien duoc 97% quang duong nhung do thoang cao hon han, nen no keo
        # nguong tham chieu len 0.22m. Luc do di thang qua khe 0.90m (thoang
        # 0.15m) bi phat 0.55 con queo tranh bi phat 0. Xe queo, lech tam khe,
        # roi ket. Day dung la loi da tim thay khi mo phong.
        d_sort = np.where(admissible, d_end_pre, np.inf)
        n_adm = int(np.count_nonzero(admissible))
        if n_adm > 0:
            k = max(1, int(math.ceil(0.10 * n_adm)))
            top = np.argsort(d_sort)[:k]
            c_ref = float(np.max(min_clear[top]))
        else:
            c_ref = self.margin_soft
        soft = float(np.clip(c_ref, self.margin_hard + 0.02, self.margin_soft))
        c_clear = np.clip((soft - min_clear) / max(1e-3, soft - self.margin_hard), 0.0, 1.0) ** 2

        # Toc do MONG MUON giam dan theo do thoang: cho hep di cham, cho rong
        # di nhanh. Dung sai lech |V - v_pref| chu KHONG dung mot vach phat
        # cung — vach cung tao ra hanh vi sai: "queo ra cho rong de duoc chay
        # nhanh" re hon "di cham va chui qua khe", nen xe khong bao gio vao khe.
        scale = np.clip(
            (min_clear - self.margin_hard) / max(1e-3, self.margin_soft - self.margin_hard),
            self.min_speed_scale, 1.0,
        )
        v_pref = self.v_max * scale
        c_speed = np.clip(np.abs(V - v_pref) / max(1e-3, self.v_max), 0.0, 1.0)

        # Bac hai: thay doi nho thi gan nhu mien phi, thay doi dot ngot thi rat dat.
        # => xe di muot ma van dam bao phan ung nhanh khi can.
        c_smooth = 0.5 * (
            (np.abs(V - self.cur_v) / max(1e-3, dv)) ** 2
            + (np.abs(W - self.cur_w) / max(1e-3, dw)) ** 2
        )

        if target_xy is not None and fov_active:
            tx, ty = target_xy
            tb = wrap_pi_arr(np.arctan2(ty - ye, tx - xe) - te)
            c_fov = np.clip((np.abs(tb) - self.fov_keep_rad) / max(1e-3, math.pi - self.fov_keep_rad), 0.0, 1.0)
        else:
            c_fov = np.zeros(n)

        if self.avoid_side != 0:
            c_side = np.where(np.sign(W) == -self.avoid_side, np.abs(W) / max(1e-3, self.w_max), 0.0)
        else:
            c_side = np.zeros(n)

        cost = (
            self.w_goal * c_goal
            + self.w_heading * c_head
            + self.w_clear * c_clear
            + self.w_speed * c_speed
            + self.w_smooth * c_smooth
            + self.w_fov * c_fov
            + self.w_side * c_side
            # Uu tien DI THANG khi hoa. Canh rat quan trong: trong canh doi xung
            # hoan hao (chui khe thang truoc mat), chi phi cua w=+0.4 va w=-0.4
            # bang nhau TUNG BIT. np.argmin luc do luon tra ve chi so NHO HON,
            # tuc luon quay PHAI. Xe lech dan sang phai, mat le an toan, roi ket
            # truoc khe. Them mot luong cuc nho de hoa thi chon w gan 0 nhat.
            + 1e-3 * np.abs(W) / max(1e-3, self.w_max)
        )
        cost = np.where(admissible, cost, np.inf)

        # 5. KHONG can them rang buoc phanh.
        #    min_clear la do thoang NHO NHAT quanh footprint, phan lon la thoang
        #    NGANG khi chui khe — ma phanh thi khong giup gi cho thoang ngang.
        #    Ap dieu kien phanh len no khien xe tu choi moi khe hep, du no lot.
        #    Dieu kien du va dung: quy dao khong cham gi trong ca 1.2s mo phong.
        #    Kiem chung: thoi gian phanh = v_max/accel = 0.22/0.35 = 0.63s < 1.2s,
        #    quang duong phanh = v^2/(2a) = 0.069m < quang duong mo phong 0.26m.
        best = int(np.argmin(cost))
        if not np.isfinite(cost[best]):
            return None
        return float(V[best]), float(W[best]), float(min_clear[best])

    # ─────────────────────────────────────────────────────────────────────
    # Vong dieu khien
    # ─────────────────────────────────────────────────────────────────────

    def _set_state(self, s: str) -> None:
        if s != self.state:
            self.prev_state = self.state
            self.state = s
            self.state_since = time.time()

    def _control(self) -> None:
        now = time.time()
        dt = 1.0 / max(1.0, self.control_hz)

        status: Dict[str, Any] = {"stamp": now, "enabled": self.enabled}

        if not self.enabled:
            self._set_state(S_IDLE)
            self.cur_v = self.cur_w = 0.0
            self.pub_cmd.publish(Twist())
            self._report(status, 0.0, 0.0, "disabled", now)
            return

        scan_ok = self._scan_fresh(now)
        if not scan_ok:
            self._set_state(S_ESTOP)
            self._hard_stop()
            self._report(status, 0.0, 0.0, "mat du lieu /scan — dung xe", now)
            return

        tgt = self.target if (now - self.target_time) < self.target_timeout else {}
        valid = bool(tgt.get("valid", False)) and float(tgt.get("confidence", 0.0)) >= self.min_conf

        # ── Khong co muc tieu: tim kiem hoac dung ────────────────────────
        if not valid:
            # Tinh tu lan cuoi co muc tieu HOP LE, khong dung target_time: tracker gui
            # /follow/target 20 Hz ca khi valid=false nen target_time luon moi va xe
            # KHONG BAO GIO vao SEARCH. last_valid_time = 0 -> chua thay ai / da quet xong.
            lost_for = now - self.last_valid_time
            if self.state != S_SEARCH and self.last_valid_time > 0.0:
                use_goto = self.search_goto_max > 0.0 and self.last_target_odom is not None
                if use_goto or lost_for > self.search_after:
                    self._set_state(S_SEARCH)
                    self.search_start = now
                    self.search_goto = use_goto
                    self.occl_turning = False
                    self.avoid_side = 0
                    self.search_dir = 1.0 if self.last_seen_bearing >= 0 else -1.0
            if self.state == S_SEARCH and self.search_goto:
                # Pha 1: lai toi cach cho thay nguoi lan cuoi mot follow_distance. Dung
                # sat hon thi canh tuong/goc ban thuong trong tam duoi xe -> khong xoay quet duoc.
                ox, oy = self.last_target_odom
                c, s = math.cos(-self.robot_yaw), math.sin(-self.robot_yaw)
                dx, dy = ox - self.robot_x, oy - self.robot_y
                bx, by = c * dx - s * dy, s * dx + c * dy
                gb = math.atan2(by, bx)
                gr = math.hypot(bx, by) - self.follow_distance
                if gr > self.dist_deadband and (now - self.search_start) < self.search_goto_max:
                    # Chi xoay tai cho khi dich lech hon 60 do: DWA tu lai duoc trong ±100 do.
                    # Nguong thap (nhu occluded_turn_deg) khien xe di doc tuong cu dung-xoay-di.
                    turn_thr = math.radians(60.0) * (0.5 if self.occl_turning else 1.0)
                    if abs(gb) > turn_thr and self._can_rotate_in_place():
                        self.occl_turning = True
                        w_cmd = float(np.clip(1.5 * gb, -self.w_max * 0.6, self.w_max * 0.6))
                        v, w = self._emit(0.0, w_cmd, dt)
                        self._report(status, v, w, "mat nguoi — quay ve cho thay lan cuoi", now)
                        return
                    self.occl_turning = False
                    phi, reach, _direct = self._choose_heading(gb, gr, gb)
                    if phi is not None:
                        sub_r = min(reach, gr)
                        res = self._dwa((sub_r * math.cos(phi), sub_r * math.sin(phi)), None, dt,
                                        allow_reverse=False, fov_active=False)
                        if res is not None:
                            v, w = self._emit(res[0], res[1], dt)
                            self._report(status, v, w, "mat nguoi — lai toi cho thay lan cuoi",
                                         now, clearance=res[2], chosen_heading=phi)
                            return
                # Toi noi, het gio, hoac bi chan -> pha 2: xoay tai cho quet tim
                self.search_goto = False
                self.occl_turning = False
                self.search_start = now
            if self.state == S_SEARCH:
                if (now - self.search_start) > self.search_max or not self._can_rotate_in_place():
                    self._set_state(S_IDLE)
                    self.last_valid_time = 0.0      # khong quet lai cho toi khi thay nguoi
                    self._emit(0.0, 0.0, dt)
                    self._report(status, 0.0, 0.0, "khong tim thay nguoi — dung", now)
                    return
                # doi chieu quet moi 3.5s de queo ca hai ben
                phase = int((now - self.search_start) / 3.5)
                d = self.search_dir * (1.0 if phase % 2 == 0 else -1.0)
                v, w = self._emit(0.0, d * self.search_w, dt)
                self._report(status, v, w, "dang quet tim nguoi", now)
                return
            self._emit(0.0, 0.0, dt)
            self._report(status, 0.0, 0.0, "cho tin hieu muc tieu", now)
            return

        # ── Co muc tieu ──────────────────────────────────────────────────
        tx = float(tgt.get("base_x", 0.0))
        ty = float(tgt.get("base_y", 0.0))
        dist = float(tgt.get("distance_m", 0.0))
        bearing = float(tgt.get("bearing_rad", 0.0))
        source = str(tgt.get("source", "none"))
        predicted = source in ("predicted", "rssi_bearing")
        self.last_seen_bearing = bearing
        self.last_valid_time = now
        self.search_goto = False
        if tgt.get("odom_x") is not None and tgt.get("odom_y") is not None:
            self.last_target_odom = (float(tgt["odom_x"]), float(tgt["odom_y"]))

        if dist > self.max_follow_dist:
            self._emit(0.0, 0.0, dt)
            self._report(status, 0.0, 0.0, "nguoi qua xa (%.1fm)" % dist, now)
            return

        # GOAL = diem cach nguoi follow_distance ve phia xe.
        # Nho vay khong can loai nguoi khoi danh sach vat can (sua Bug#4).
        goal_r = max(0.0, dist - self.follow_distance)
        # Vung chet huong: lech duoi bearing_deadband thi coi dich nam thang truoc mui.
        # Goc lay tu cum chan nguoi rung vai do moi buoc chan; khong co vung chet thi DWA
        # be lai +-0.1 rad/s lien tuc va xe lac qua lai thay vi bam thang.
        goal_b = bearing if abs(bearing) > self.bearing_deadband else 0.0
        gx = goal_r * math.cos(goal_b)
        gy = goal_r * math.sin(goal_b)

        # Tam do cho TANG CHON KHE: toi cach nguoi target_clear_radius_m (bo qua chan chinh ho).
        # Nguoi thu hai buoc vao giua thuong dung ngay diem dich (cach nguoi 1 m) -> chi do toi
        # dich thi huong thang van "thoang", xe di thang toi ho.
        chk_r = max(goal_r, dist - self.target_clear_r)

        # ── Duong thang toi dich co bi chan khong? ───────────────────────
        # CHI toi dich (khong toi chk_r): do toi tan cho nguoi thi di qua khung cua hep, tia
        # nhin toi nguoi sat thanh cua -> vao AVOID, khoa nham ben tuong -> xe ep vao thanh cua.
        # Tang chon khe (do toi chk_r) du de lai vong nguoi thu hai ma khong khoa ben.
        corridor = self.half_width * self.block_corridor_scale + self.margin_hard
        blocked, _block_dist = segment_blocked(self.obstacles, gx, gy, corridor)

        # Khe vua du rong cho xe di THANG (vd. khung cua 0.81 m) thi KHONG vao AVOID. Hanh lang
        # kiem tra AVOID rong hon thuc te (half_width * 1.15 + margin_hard = 0.405 moi ben) nen o
        # cua hep no LUON bao bi chan du xe dang thang hang va thua suc lot: xe be ra mot ben roi
        # ket o mep cua. Tang chon khe van nham giua khe, DWA van giu margin_hard.
        if blocked:
            fit, _d = segment_blocked(self.obstacles, gx, gy, self.half_width + self.margin_hard)
            if not fit:
                blocked = False

        if blocked:
            self.clear_since = 0.0
            if self.blocked_since == 0.0:
                self.blocked_since = now
        else:
            self.blocked_since = 0.0
            if self.clear_since == 0.0:
                self.clear_since = now

        want_avoid = blocked and (now - self.blocked_since) >= self.block_enter_sec
        can_exit_avoid = (not blocked) and self.clear_since > 0 and (now - self.clear_since) >= self.block_exit_sec

        # ── Chon trang thai ──────────────────────────────────────────────
        if want_avoid:
            # Chon ben theo khe di duoc (_gap_side); khong ro thi moi dung khoang trong hai ben.
            # Het avoid_side_hold_sec thi chon LAI ca khi van bi chan — truoc day khoa chet ben da
            # chon suot luc bi chan: o khung cua lo chon ben tuong la ep vao thanh cua mai.
            if self.avoid_side == 0 or (now - self.avoid_side_time) > self.avoid_hold:
                side = self._gap_side(goal_b, chk_r, bearing)
                if side == 0:
                    side = self.avoid_side if self.avoid_side != 0 else self._choose_avoid_side(bearing)
                self.avoid_side = side
                self.avoid_side_time = now
            self._set_state(S_AVOID)
        elif can_exit_avoid and self.state == S_AVOID:
            self.avoid_side = 0
            self._set_state(S_FOLLOW)
        elif predicted:
            self._set_state(S_OCCLUDED)
        elif goal_r <= self.dist_deadband and abs(bearing) <= self.bearing_deadband:
            self._set_state(S_ARRIVED)
        elif self.state not in (S_AVOID,):
            self._set_state(S_FOLLOW)

        # Roi AVOID bang duong khac nhanh can_exit_avoid (OCCLUDED khi xoay ve phia nguoi,
        # SEARCH, BLOCKED...) thi avoid_side van con -> c_side tiep tuc PHAT quay ve phia da ne:
        # ne xong nguoi re ve phia do la xe xoay rat cham. Duong da thoang thi xoa.
        if self.state != S_AVOID and not blocked:
            self.avoid_side = 0

        # ── NGUOI RA KHOI CAMERA: xoay ve huong nho cuoi truoc ───────────
        # Camera co dinh nen phai QUAY XE moi thay lai nguoi. Camera khong thay (chi con
        # lidar_track hoac du doan) ma nguoi lech qua occluded_turn_deg -> xoay tai cho ve
        # phia do (co tre: quay toi duoi mot nua nguong), roi moi tien toi. Goc du doan
        # thuong tre hon goc that ~10 do nen nguong dat thap hon nua FOV. Chi xoay khi
        # du cho cho duoi xe.
        camera_blind = not source.startswith("camera")
        turn_thr = self.occluded_turn_rad * (0.5 if self.occl_turning else 1.0)
        if (camera_blind and self.occluded_turn_rad > 0.0 and abs(bearing) > turn_thr
                and self._can_rotate_in_place()):
            self.occl_turning = True
            self._set_state(S_OCCLUDED)
            w_cmd = float(np.clip(1.5 * bearing, -self.w_max * 0.6, self.w_max * 0.6))
            v, w = self._emit(0.0, w_cmd, dt)
            self._report(status, v, w, "nguoi ra khoi camera — xoay ve huong nho cuoi",
                         now, dist, bearing, source)
            return
        self.occl_turning = False

        # ── DA TOI DUNG KHOANG CACH: chi xoay nhe de giu nguoi giua khung ─
        if self.state == S_ARRIVED:
            v, w = self._emit(0.0, 0.0, dt)
            self._report(status, v, w, "giu khoang cach %.2fm" % dist, now, dist, bearing, source)
            return

        if goal_r <= self.dist_deadband:
            w_cmd = 0.0
            if abs(bearing) > self.bearing_deadband and self._can_rotate_in_place():
                w_cmd = float(np.clip(1.5 * bearing, -self.w_max * 0.6, self.w_max * 0.6))
            v, w = self._emit(0.0, w_cmd, dt)
            self._report(status, v, w, "canh huong tai cho", now, dist, bearing, source)
            return

        # ── TANG 1: chon huong di qua khe (VFH) ──────────────────────────
        # Do khe toi tan cho nguoi (chk_r) de huong thang bi chan boi nguoi thu hai thi
        # chon khe ben canh; dich phu ben duoi van gioi han trong goal_r.
        phi, reach, _direct = self._choose_heading(goal_b, chk_r, bearing)

        if phi is None:
            # Khong huong nao di duoc: xoay tai cho tim loi, hoac dung han
            if self._can_rotate_in_place():
                self._set_state(S_BLOCKED)
                d = self.avoid_side if self.avoid_side != 0 else (1.0 if bearing >= 0 else -1.0)
                v, w = self._emit(0.0, d * min(self.search_w, self.w_max * 0.5), dt)
                self._report(status, v, w, "bi chan — xoay tai cho tim loi", now, dist, bearing, source)
                return
            self._set_state(S_BLOCKED)
            v, w = self._emit(0.0, 0.0, dt)
            self._report(status, v, w, "ket hoan toan — dung cho", now, dist, bearing, source)
            return

        # Dich phu: diem tren huong da chon. DWA chi con viec lai cho muot toi do.
        sub_r = min(reach, goal_r)
        sgx = sub_r * math.cos(phi)
        sgy = sub_r * math.sin(phi)

        # ── TANG 2: DWA lam muot + dam bao an toan ───────────────────────
        res = self._dwa((sgx, sgy), (tx, ty), dt, allow_reverse=False, fov_active=True)

        if res is None:
            if self._can_rotate_in_place():
                self._set_state(S_BLOCKED)
                d = self.avoid_side if self.avoid_side != 0 else (1.0 if phi >= 0 else -1.0)
                v, w = self._emit(0.0, d * min(self.search_w, self.w_max * 0.5), dt)
                self._report(status, v, w, "bi chan — xoay tai cho tim loi", now, dist, bearing, source)
                return
            res = self._dwa((sgx, sgy), (tx, ty), dt, allow_reverse=True, fov_active=False)
            if res is None:
                self._set_state(S_BLOCKED)
                v, w = self._emit(0.0, 0.0, dt)
                self._report(status, v, w, "ket hoan toan — dung cho", now, dist, bearing, source)
                return

        v_cmd, w_cmd, clearance = res
        v, w = self._emit(v_cmd, w_cmd, dt)

        note = {
            S_FOLLOW: "bam nguoi",
            S_AVOID: "ne vat can (%s)" % ("trai" if self.avoid_side > 0 else "phai"),
            S_OCCLUDED: "nguoi bi che — di theo du doan",
            S_BLOCKED: "bi chan",
        }.get(self.state, self.state)
        self._report(status, v, w, note, now, dist, bearing, source, clearance, phi)

        if self.publish_markers:
            self._publish_markers(sgx, sgy, tx, ty)

    # ─────────────────────────────────────────────────────────────────────
    # Bao cao
    # ─────────────────────────────────────────────────────────────────────

    def _report(
        self,
        status: Dict[str, Any],
        v: float,
        w: float,
        note: str,
        now: float,
        dist: Optional[float] = None,
        bearing: Optional[float] = None,
        source: str = "none",
        clearance: Optional[float] = None,
        chosen_heading: Optional[float] = None,
    ) -> None:
        status.update({
            "state": self.state,
            "note": note,
            "cmd_v": round(float(v), 4),
            "cmd_w": round(float(w), 4),
            "target_distance_m": None if dist is None else round(dist, 3),
            "target_bearing_deg": None if bearing is None else round(math.degrees(bearing), 2),
            "target_source": source,
            "avoid_side": int(self.avoid_side),
            "clearance_m": None if clearance is None else round(float(clearance), 3),
            "chosen_heading_deg": None if chosen_heading is None else round(math.degrees(chosen_heading), 1),
            "front_clearance_m": round(self._front_clearance(), 3),
            "n_obstacles": int(self.obstacles.shape[0]),
        })
        self.pub_status.publish(String(data=json.dumps(status, ensure_ascii=False)))

        if now - self.last_log > self.log_period:
            self.get_logger().info(
                "[%s] %s | d=%s goc=%s | v=%.3f w=%.3f | thoang=%s"
                % (
                    self.state, note,
                    "-" if dist is None else "%.2fm" % dist,
                    "-" if bearing is None else "%.0fdeg" % math.degrees(bearing),
                    v, w,
                    "-" if clearance is None else "%.2fm" % clearance,
                )
            )
            self.last_log = now

    def _publish_markers(self, gx: float, gy: float, tx: float, ty: float) -> None:
        arr = MarkerArray()

        g = Marker()
        g.header.frame_id = self.base_frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.ns = "follow_goal"
        g.id = 0
        g.type = Marker.SPHERE
        g.action = Marker.ADD
        g.pose.position = Point(x=float(gx), y=float(gy), z=0.1)
        g.pose.orientation.w = 1.0
        g.scale.x = g.scale.y = g.scale.z = 0.18
        g.color = ColorRGBA(r=0.1, g=0.9, b=0.9, a=0.9)
        arr.markers.append(g)

        line = Marker()
        line.header.frame_id = self.base_frame
        line.header.stamp = g.header.stamp
        line.ns = "follow_goal"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.03
        line.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8)
        line.points = [Point(x=0.0, y=0.0, z=0.1), Point(x=float(tx), y=float(ty), z=0.1)]
        arr.markers.append(line)

        self.pub_markers.publish(arr)

    def destroy_node(self) -> bool:
        try:
            self._hard_stop()
        except Exception:
            pass
        return super().destroy_node()


def wrap_pi_arr(a: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(a), np.cos(a))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowPlannerNode()
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
