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

import heapq

import numpy as np
import rclpy
try:                                       # tim duong tren luoi nhanh hon ~10 lan; khong co thi dung heapq
    from scipy.sparse import csr_matrix as _csr_matrix
    from scipy.sparse.csgraph import dijkstra as _sp_dijkstra
except Exception:                          # pragma: no cover
    _csr_matrix = None
    _sp_dijkstra = None
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
        self.gap_back_used = 0.0            # da lui bao nhieu met trong dong tac chui khe
        self.gap_back_time = 0.0            # lan cuoi lui (de cap lai han muc)
        self.gap_cross_time = 0.0           # lan cuoi o pha qua khe (tre dung sai truc)
        self.last_target_odom: Optional[Tuple[float, float]] = None   # cho thay nguoi lan cuoi
        self.search_goto = False            # SEARCH dang o pha lai toi cho do
        self.nav_rotating = False           # bo bam duong ban do dang xoay tai cho ve huong duong
        self.nav_side = 0                   # ben vong da cam ket (+1 trai / -1 phai)
        self.nav_side_t = 0.0
        self.nav_look = (0.0, 0.0)
        self.nav_behind_t = 0.0             # luc bat dau thay duong di nam phia sau xe
        self.goal_bad_since = 0.0           # luc bat dau thay dich cham le an toan
        self.nav_fight_t = 0.0              # luc DWA bat dau queo nguoc huong duong ban do
        self.nav_follow_t = 0.0             # lan goi _nav_follow gan nhat
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
        self.rssi_msg: Optional[Dict[str, Any]] = None
        self.rssi_msg_time = 0.0
        self.search_phase: Optional[str] = None
        self.search_phase_t = 0.0
        self.spin_acc = 0.0
        self.spin_last_yaw = 0.0
        self.spin_tries = 0
        self.rssi_goal_odom: Optional[Tuple[float, float]] = None
        self.rssi_aligned_t = 0.0
        self.rssi_bearing_odom = 0.0
        if self.rssi_search:
            self.create_subscription(String, self.rssi_topic, self._rssi_cb, 10)
            self.rssi_reset_client = self.create_client(Trigger, self.rssi_reset_srv)
        else:
            self.rssi_reset_client = None

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
            "w_max": 1.00,
            "accel_lin": 0.35,
            "accel_ang": 2.40,
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
            # Giu nguoi giua khung hinh SUOT quy dao (xem _dwa). 0 = tat (nhu truoc 29/09).
            "w_center": 0.8,
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
            # Duong thang khong lot -> tim khe kieu khung cua rong tu 0.72 m den muc nay, ngam
            # vao truc vuong goc cua khe de canh xe roi chui qua. 0 = tat.
            "gap_waypoint_max_width_m": 1.20,
            # Canh truc khe truoc khi chui: xe coi nhu "da tren truc" khi lech duoi muc nay,
            # luc do neu mui xe con lech qua gap_align_deg va con cach mat khe duoi
            # gap_align_range_m thi DUNG LAI XOAY cho vuong roi moi tien.
            # Khe phai mo ra it nhat ngan nay goc nhin tu xe — chan "khe gia" do tia
            # lidar quet doc tuong cheo tao ra (cua 0.81 m o 4.6 m moi con 10 do).
            "gap_min_span_deg": 10.0,
            # Toc do khi chui khe / lui ra khoi khe. Cham de con sua duoc lech ngang.
            "gap_cross_speed": 0.12,
            # Chi lam dong tac chui khe khi mat khe da o trong tam nay. Xa hon thi de
            # tang chon khe + DWA lai nhu binh thuong.
            "gap_engage_range_m": 1.60,
            # Quang duong lui toi da khi canh lai truc khe. Lidar KHONG thay thang phia
            # sau (blind_sectors_deg) nen chi lui dung doan vua di qua.
            "gap_back_max_m": 0.40,
            # Khoang ho TOI THIEU footprint-vat khi dang chui khe (thay margin_hard trong
            # dong tac nay). Xem giai thich o follow_nav.yaml.
            "gap_cross_margin_m": 0.03,
            # BAN DO LUOI CUC BO + TIM DUONG (07/10). Chi chay khi duong thang toi dich bi chan.
            # Thay cho tang chon khe o cho: chon dung ben vong (khe hep hon xe tu dong bi dong), biet
            # khi nao dung cho la tot nhat (dich nam trong vat), dong tac chui cua chi chay khi duong
            # di that su qua khe do. Xem _nav_plan.
            "nav_plan_enabled": True,
            "nav_grid_res_m": 0.06,
            "nav_lethal_m": 0.30,          # tam xe khong bao gio vao gan vat hon muc nay (= half_width)
            "nav_soft_m": 0.25,            # vung phat them ngoai half_width + margin_hard
            "nav_soft_cost": 3.0,
            "nav_far_weight": 4.0,         # moi met con xa nguoi hon follow_distance = ngan nay met duong
            "nav_far_weight_moving": 8.0,  # nguoi dang di (> 0.25 m/s): vong qua som hon
            "nav_person_clear_m": 0.35,    # bo diem lidar quanh nguoi khi lap ban do (chan chinh ho)
            "nav_lookahead_m": 1.2,
            "nav_hold_radius_m": 0.15,     # o tot nhat cach xe duoi muc nay -> da o cho tot nhat, dung cho
            "nav_replan_sec": 0.2,
            "nav_turn_first_deg": 75.0,    # duong lech hon -> xoay tai cho roi moi di (xem _nav_follow)
            "nav_fov_keep_deg": 50.0,      # luc di vong chi phat khi nguoi sap ra khoi khung hinh
            # RSSI trong SEARCH (07/10): khi mat nguoi va 2 node RSSI dang chay (beacon_ok), xe XOAY DO
            # tai cho de /rssi/bearing (rssi_bearing_node) ra huong beacon, quay camera ve do cho
            # camera/ReID nhan lai, chua thay thi di ve huong do rssi_go_dist_m roi do lai. Khong co
            # RSSI -> quet nhu cu. RSSI KHONG dung luc dang bam (sai ~15 do, can ~8 s xoay).
            "rssi_search_enabled": True,
            "rssi_bearing_topic": "/rssi/bearing",
            "rssi_reset_service": "/rssi/reset",
            "rssi_timeout_sec": 2.0,
            "rssi_spin_w": 0.9,              # do tren xe 01/10 o 0.82-0.9 rad/s (that 0.82)
            "rssi_spin_max_deg": 760.0,      # nhu rssi_seek.py
            "rssi_face_hold_sec": 1.5,       # quay ve huong beacon roi dung cho camera nhan lai
            "rssi_go_dist_m": 1.5,
            "rssi_search_max_sec": 90.0,
            "gap_axis_tol_m": 0.12,
            "gap_align_deg": 10.0,
            "gap_align_range_m": 0.90,
            "w_gap_align": 3.0,

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
        self.w_center = float(g("w_center"))
        self.occluded_turn_rad = math.radians(float(g("occluded_turn_deg")))
        self.target_clear_r = float(g("target_clear_radius_m"))
        self.gap_wp_max_w = float(g("gap_waypoint_max_width_m"))
        self.gap_min_span_rad = math.radians(float(g("gap_min_span_deg")))
        self.gap_cross_speed = float(g("gap_cross_speed"))
        self.gap_engage_range = float(g("gap_engage_range_m"))
        self.gap_back_max = float(g("gap_back_max_m"))
        self.gap_cross_margin = float(g("gap_cross_margin_m"))
        self.nav_enabled = bool(g("nav_plan_enabled"))
        self.nav_res = float(g("nav_grid_res_m"))
        self.nav_lethal = float(g("nav_lethal_m"))
        self.nav_soft = float(g("nav_soft_m"))
        self.nav_soft_cost = float(g("nav_soft_cost"))
        self.nav_far_w = float(g("nav_far_weight"))
        self.nav_far_w_moving = float(g("nav_far_weight_moving"))
        self.nav_person_clear = float(g("nav_person_clear_m"))
        self.nav_lookahead = float(g("nav_lookahead_m"))
        self.nav_hold_r = float(g("nav_hold_radius_m"))
        self.nav_replan = float(g("nav_replan_sec"))
        self.nav_turn_first = math.radians(float(g("nav_turn_first_deg")))
        self.nav_fov_keep = math.radians(float(g("nav_fov_keep_deg")))
        self.nav_cache: Optional[Dict[str, Any]] = None
        self.rssi_search = bool(g("rssi_search_enabled"))
        self.rssi_topic = str(g("rssi_bearing_topic"))
        self.rssi_reset_srv = str(g("rssi_reset_service"))
        self.rssi_timeout = float(g("rssi_timeout_sec"))
        self.rssi_spin_w = float(g("rssi_spin_w"))
        self.rssi_spin_max = math.radians(float(g("rssi_spin_max_deg")))
        self.rssi_face_hold = float(g("rssi_face_hold_sec"))
        self.rssi_go_dist = float(g("rssi_go_dist_m"))
        self.rssi_search_max = float(g("rssi_search_max_sec"))
        self.gap_axis_tol = float(g("gap_axis_tol_m"))
        self.gap_align_rad = math.radians(float(g("gap_align_deg")))
        self.gap_align_range = float(g("gap_align_range_m"))
        self.w_gap_align = float(g("w_gap_align"))

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

    # ─────────────────────────────────────────────────────────────────────
    # SEARCH sau pha "di toi" (07/10): quay mat -> (co RSSI) xoay do -> nhin huong beacon -> di toi
    # huong do; (khong RSSI) quet qua lai theo goc xoay duoc that.
    # ─────────────────────────────────────────────────────────────────────
    def _search_next(self, now: float, phase: str) -> None:
        self.search_phase = phase
        self.search_phase_t = now
        if phase == "spin":
            self.spin_acc = 0.0
            self.spin_last_yaw = self.robot_yaw
            self._rssi_request_reset()

    def _search_give_up(self, now: float, dt: float, status: Dict[str, Any], why: str) -> None:
        self._set_state(S_IDLE)
        self.search_phase = None
        self.last_valid_time = 0.0      # khong quet lai cho toi khi thay nguoi
        self._emit(0.0, 0.0, dt)
        self._report(status, 0.0, 0.0, "khong tim thay nguoi — dung (%s)" % why, now)

    def _turn_toward(self, now: float, dt: float, status: Dict[str, Any], err: float, note: str) -> bool:
        """Xoay tai cho ve goc err (base_link) neu quet footprint an toan. True = da ra lenh."""
        if self._can_turn(err):
            w_cmd = float(np.clip(2.2 * err, -self.w_max * 0.9, self.w_max * 0.9))
            v, w = self._emit(0.0, w_cmd, dt)
            self._report(status, v, w, note, now)
            return True
        return False

    def _search_step(self, now: float, dt: float, status: Dict[str, Any]) -> None:
        rssi_ok = self._rssi_available(now)
        limit = self.rssi_search_max if rssi_ok else self.search_max
        if (now - self.search_start) > limit:
            self._search_give_up(now, dt, status, "het %.0f s" % limit)
            return
        ph = self.search_phase
        T = now - self.search_phase_t

        if ph == "face":
            # Camera 114 do: quay mat ve cho thay nguoi lan cuoi thuong la thay lai ngay. Truoc 07/10 pha
            # nay khong co: toi noi ma mui lech 37 do, roi quet doi trong ca vong 360 do -> canh thung
            # thi bo cuoc (IDLE) du nguoi ngay do.
            if self.last_target_odom is not None and T < 4.0:
                bx, by = self._base_of(*self.last_target_odom)
                err = math.atan2(by, bx)
                if abs(err) > math.radians(8.0) and \
                        self._turn_toward(now, dt, status, err, "mat nguoi — quay mat ve cho thay lan cuoi"):
                    return
            if T < 0.8:                          # dung yen chut cho camera/ReID kip nhan lai
                v, w = self._emit(0.0, 0.0, dt)
                self._report(status, v, w, "mat nguoi — nhin ve cho thay lan cuoi", now)
                return
            self._search_next(now, "spin" if rssi_ok else "scan")
            ph, T = self.search_phase, 0.0

        if ph == "spin":
            if not rssi_ok:
                self._search_next(now, "scan")
                return
            m = self.rssi_msg or {}
            if (m.get("valid") and m.get("bearing_odom_rad") is not None
                    and self.rssi_msg_time > self.search_phase_t + 0.5 and self.spin_acc > math.radians(200.0)):
                self.rssi_goal_odom = None
                self.rssi_bearing_odom = float(m["bearing_odom_rad"])
                self.rssi_aligned_t = 0.0
                self.get_logger().info("RSSI: huong beacon %+.0f do so voi mui xe (corr %.2f)" % (
                    math.degrees(wrap_pi(self.rssi_bearing_odom - self.robot_yaw)), float(m.get("corr", 0.0))))
                self._search_next(now, "rssi_face")
                return
            if self.spin_acc > self.rssi_spin_max or T > 18.0:
                # xoay het muc ma khong khop duoc -> quet thuong mot luot roi thu lai
                self.spin_tries += 1
                self._search_next(now, "scan" if self.spin_tries >= 2 else "spin")
                return
            if not self._can_rotate_in_place():
                # xoay do can trong ca vong (duoi xe quet 0.47 m) -> nhich ra cho thoang truoc
                self._search_next(now, "open")
                return
            self.spin_acc += abs(wrap_pi(self.robot_yaw - self.spin_last_yaw))
            self.spin_last_yaw = self.robot_yaw
            w_spin = min(self.rssi_spin_w, self.w_max) * (1.0 if self.search_dir >= 0 else -1.0)
            v, w = self._emit(0.0, w_spin, dt)
            self._report(status, v, w, "mat nguoi — xoay do huong beacon (%.0f do)" % math.degrees(self.spin_acc), now)
            return

        if ph == "rssi_face":
            err = wrap_pi(self.rssi_bearing_odom - self.robot_yaw)
            if abs(err) > math.radians(6.0) and T < 5.0:
                if self._turn_toward(now, dt, status, err, "mat nguoi — quay camera ve huong beacon"):
                    return
            if self.rssi_aligned_t == 0.0:
                self.rssi_aligned_t = now
            if now - self.rssi_aligned_t < self.rssi_face_hold:
                v, w = self._emit(0.0, 0.0, dt)
                self._report(status, v, w, "mat nguoi — nhin huong beacon, cho camera nhan lai", now)
                return
            d = self.rssi_go_dist
            self.rssi_goal_odom = (self.robot_x + d * math.cos(self.rssi_bearing_odom),
                                   self.robot_y + d * math.sin(self.rssi_bearing_odom))
            self._search_next(now, "rssi_go")
            return

        if ph in ("rssi_go", "open"):
            if ph == "open":
                # Huong thoang nhat (tang chon khe, dich 1 m ve huong do) — de co cho xoay do
                if self.rssi_goal_odom is None or T < dt * 1.5:
                    phi, reach, _ = self._choose_heading(0.0, 1.0, 0.0)
                    if phi is None:
                        self._search_next(now, "scan")
                        return
                    dd = min(0.6, reach)
                    self.rssi_goal_odom = self._odom_of(dd * math.cos(phi), dd * math.sin(phi))
            gx_, gy_ = self._base_of(*self.rssi_goal_odom)
            if math.hypot(gx_, gy_) < 0.25 or T > 10.0 or (ph == "open" and self._can_rotate_in_place()):
                self.rssi_goal_odom = None
                self._search_next(now, "spin")
                return
            look = (gx_, gy_)
            blk, _bd = segment_blocked(self.obstacles, gx_, gy_, self.half_width + self.margin_hard)
            if blk and self.nav_enabled:
                nav = self._nav_get(now, gx_, gy_, False)
                if nav is None or nav["hold"]:
                    self.rssi_goal_odom = None
                    self._search_next(now, "spin")
                    return
                res = self._nav_follow(nav, gx_, gy_, dt, None)
                look = self.nav_look
            else:
                res = self._dwa(look, None, dt, allow_reverse=False, fov_active=False)
            if res is None:
                self.rssi_goal_odom = None
                self._search_next(now, "scan")
                return
            v, w = self._emit(res[0], res[1], dt)
            note = "mat nguoi — di ve huong beacon" if ph == "rssi_go" else "mat nguoi — nhich ra cho thoang de xoay do"
            self._report(status, v, w, note, now, clearance=res[-1], chosen_heading=math.atan2(look[1], look[0]))
            return

        # ph == "scan": quet qua lai, doi chieu moi 3.5 s; xoay theo goc QUET DUOC THAT (_can_turn) chu
        # khong doi trong ca vong 360 do (13.14: canh goc tuong / thung la bo cuoc ngay)
        phase = int((now - self.search_phase_t) / 3.5)
        d = self.search_dir * (1.0 if phase % 2 == 0 else -1.0)
        for dd in (d, -d):
            if self._can_turn(dd * 0.6):
                v, w = self._emit(0.0, dd * self.search_w, dt)
                self._report(status, v, w, "dang quet tim nguoi", now)
                return
        self._search_give_up(now, dt, status, "khong du cho xoay")

    def _rssi_cb(self, msg: String) -> None:
        try:
            self.rssi_msg = json.loads(msg.data)
            self.rssi_msg_time = time.time()
        except Exception:
            pass

    def _rssi_available(self, now: float) -> bool:
        m = self.rssi_msg
        return (self.rssi_search and m is not None and (now - self.rssi_msg_time) < self.rssi_timeout
                and bool(m.get("beacon_ok", False)))

    def _rssi_request_reset(self) -> None:
        """Xoa mau RSSI cu truoc mot lan xoay do moi (nguoi co the da di cho khac)."""
        cli = self.rssi_reset_client
        try:
            if cli is not None and cli.service_is_ready():
                cli.call_async(Trigger.Request())
        except Exception:
            pass

    def _enable_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.enabled = True
        # Bat lai la bat dau moi: khong lai toi cho nguoi cua lan bam truoc
        self.last_valid_time = 0.0
        self.last_target_odom = None
        # 13.34: dang o SEARCH pha "di toi" ma goi /follow/enable thi nhip sau doc last_target_odom = None
        # -> TypeError -> node CHET. Xoa luon pha tim.
        self.search_goto = False
        self.search_phase = None
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

    def _gap_target(self, goal_bearing: float, goal_dist: float,
                    target_xy: Optional[Tuple[float, float]] = None
                    ) -> Optional[Dict[str, float]]:
        """Tim KHE HEP kieu khung cua nam giua xe va nguoi. None = khong co khe nao hop.

        Tra ve hinh hoc cua khe trong base_link:
          mx, my : tam khe
          nx, ny : phap tuyen khe, huong TU khe VE PHIA xe
          psi    : huong chui qua khe — mui xe phai quay ve day truoc khi chui
          along  : khoang cach tu xe toi mat khe
          lat    : xe dang lech khoi truc giua khe bao nhieu

        Xe KHONG the di cheo qua khe hep: dai kiem tra rong 2*(half_width+margin_hard)=0.72 m,
        di cheo goc th thi no cat mat cua thanh doan 0.72/cos(th) — o cua 0.81 m chi lech 27 do
        la khong huong nao lot (xem CLAUDE.md muc 9). Vi vay phai TOI NGANG TAM CUA theo phuong
        vuong goc roi moi chui qua, thay vi cu ngam thang vao nguoi o ben kia cua.

        Tim hai diem lidar ke nhau theo goc, cach nhau tu 0.72 m den gap_waypoint_max_width_m
        (khe rong hon nua thi la cho trong, khong can canh truc).
        """
        P = self.obstacles
        if self.gap_wp_max_w <= 0.0 or P.shape[0] < 2:
            return None
        need = 2.0 * (self.half_width + self.margin_hard)
        rng = np.hypot(P[:, 0], P[:, 1])
        sel = (rng < max(2.5, goal_dist + 1.0)) & (P[:, 0] > -0.2)
        if int(np.count_nonzero(sel)) < 2:
            return None
        Q = P[sel]
        if target_xy is not None:
            # Bo chan NGUOI DANG BAM ra khoi danh sach mep khe. Khong bo thi khe
            # "giua chan nguoi va thanh cua" rong dung co 0.8 m se duoc chon, xe
            # canh theo mot truc sai hoan toan roi lao vao tuong (loi 7.2-AB).
            d_t = np.hypot(Q[:, 0] - target_xy[0], Q[:, 1] - target_xy[1])
            Q = Q[d_t > min(0.35, self.target_clear_r)]
            if Q.shape[0] < 2:
                return None
        Q = Q[np.argsort(np.arctan2(Q[:, 1], Q[:, 0]))]

        best, best_cost = None, float("inf")
        for i in range(Q.shape[0] - 1):
            A, B = Q[i], Q[i + 1]
            w = float(math.hypot(B[0] - A[0], B[1] - A[1]))
            if not (need + 0.02 <= w <= self.gap_wp_max_w):
                continue
            mx, my = 0.5 * (A[0] + B[0]), 0.5 * (A[1] + B[1])
            m_r = math.hypot(mx, my)
            # Khe phai nam GIUA xe va nguoi, va khong lech huong nguoi qua nhieu.
            # KHONG loai khe sat xe: xe da dung trong khung cua (ket o do) thi tam khe
            # cach tam xe chi vai cm — day chinh la luc can dong tac nhat.
            if m_r > goal_dist + self.follow_distance:
                continue
            in_gap = m_r < 0.4        # xe dang dung trong khe: huong toi tam khe vo nghia
            # ── LOAI KHE GIA ────────────────────────────────────────────────
            # Tia lidar quet DOC theo tuong o goc rat cheo: hai tia lien nhau roi
            # cach nhau ca met TREN MAT TUONG, nhin ra nhu mot khe rong 0.8 m.
            # Xe bam vao "khe" do roi lao vao tuong canh cua (loi 7.2-AB).
            # Khe that phai: (1) mo ra mot goc du lon nhin tu xe, (2) nam NGANG
            # tia nhin chu khong doc theo no.
            span = abs(wrap_pi(math.atan2(B[1], B[0]) - math.atan2(A[1], A[0])))
            if span < self.gap_min_span_rad:
                continue
            uxh, uyh = (B[0] - A[0]) / w, (B[1] - A[1]) / w
            if not in_gap and abs(uxh * mx + uyh * my) / m_r > 0.87:   # lech < 30 do so voi tia nhin
                continue
            d_ang = 0.0 if in_gap else abs(wrap_pi(math.atan2(my, mx) - goal_bearing))
            if d_ang > math.radians(80.0):
                continue
            if target_xy is not None and not in_gap:
                # Xe va nguoi phai o HAI PHIA cua khe — khong thi day khong phai cho can chui
                # (vd nguoi dung ngay trong khe, hay khe nam ben canh). Thieu kiem tra nay
                # thi chieu khe tinh theo nguoi co the quay nguoc 180 do: xe quay lung lai
                # nguoi o cua truoc 0.81 m (mo phong 26/09: camera 100% -> 11%).
                cnx, cny = -uyh, uxh
                s_r = -(cnx * mx + cny * my)
                s_t = cnx * (target_xy[0] - mx) + cny * (target_xy[1] - my)
                if s_r * s_t >= 0.0:
                    continue
            cost = d_ang + 0.3 * m_r
            if cost < best_cost:
                best_cost, best = cost, (A, B, mx, my)
        if best is None:
            return None

        A, B, mx, my = best
        ux, uy = B[0] - A[0], B[1] - A[1]
        un = max(1e-6, math.hypot(ux, uy))
        ux, uy = ux / un, uy / un                      # doc theo mat khe
        nx, ny = -uy, ux                               # phap tuyen cua khe
        # Phai huong TU khe VE PHIA xe, tuc QUAY LUNG voi nguoi. Xet theo NGUOI chu khong
        # theo xe: xe dang dung trong khung cua thi tich vo huong voi tam khe gan 0, dau
        # doi lien tuc giua cac nhip -> huong chui qua khe lat 180 do.
        if target_xy is not None:
            if nx * (target_xy[0] - mx) + ny * (target_xy[1] - my) > 0.0:
                nx, ny = -nx, -ny
        elif nx * mx + ny * my > 0.0:
            nx, ny = -nx, -ny
        along = -(nx * mx + ny * my)                   # khoang cach tu xe toi mat khe
        lat = abs(ux * mx + uy * my)                   # xe lech khoi truc giua khe bao nhieu
        psi = math.atan2(-ny, -nx)                     # huong chui qua khe
        return {"mx": mx, "my": my, "nx": nx, "ny": ny, "gw": float(math.hypot(ux * un, uy * un)),
                "psi": psi, "along": along, "lat": lat}

    def _pose_clearance(self, px: float, py: float, yaw: float) -> float:
        """Do thoang footprint chu nhat neu xe dung o tu the (px, py, yaw) — khung base_link hien tai."""
        P = self.obstacles
        if P.shape[0] == 0:
            return 10.0
        dx, dy = P[:, 0] - px, P[:, 1] - py
        c, s_ = math.cos(yaw), math.sin(yaw)
        return float(np.min(rect_clearance(c * dx + s_ * dy, -s_ * dx + c * dy,
                                           self.front_len, self.rear_len, self.half_width)))

    def _clear_now(self) -> float:
        """Do thoang cua footprint o tu the HIEN TAI (chua di dau)."""
        P = self.obstacles
        if P.shape[0] == 0:
            return 10.0
        return float(np.min(rect_clearance(P[:, 0], P[:, 1],
                                           self.front_len, self.rear_len, self.half_width)))

    def _admissible(self, min_clear, end_clear):
        """Quy dao nao duoc phep chay.

        Binh thuong: thoang suot quy dao > margin_hard.

        LUAT THOAT KET: xe DA lo sat vat duoi margin_hard (nhieu lidar, truot banh,
        lidar chi 10 Hz, min_move keo lenh quay nho len 0.10 rad/s...) thi luat tren
        loai MOI quy dao — ke ca quy dao xoay RA XA khung cua. Xe dung im mai o
        BLOCKED du chi can xoay nhe ve phia rong la qua (nguoi dung xoay tay xe
        tai cho mot chut thi di duoc, 23/09). Nen luc da ket, cho phep quy dao:
          - khong luc nao gan vat hon hien tai (tru 5 mm sai so), va
          - ket thuc xa vat hon hien tai it nhat 1 cm, va
          - khong bao gio cham (> 1 cm).
        Dung yen van an toan hon lao vao; luat nay chi mo them duong THOAT.
        """
        ok = min_clear > self.margin_hard
        c0 = self._clear_now()
        if c0 <= self.margin_hard:
            ok = ok | ((min_clear >= c0 - 0.005) & (end_clear >= c0 + 0.01) & (min_clear > 0.01))
        return ok

    def _arc_clearance(self, v: float, w: float, steps: Optional[int] = None) -> Tuple[float, float]:
        """Do thoang nho nhat cua footprint chu nhat khi chay dung MOT lenh (v, w).

        Dung cho dong tac chui khe: lenh do bo dieu khien hinh hoc sinh ra chu khong
        phai DWA chon, nen van phai qua dung bo loc va cham do truoc khi cho chay.
        """
        P = self.obstacles
        if P.shape[0] == 0:
            return 10.0, 10.0
        t = self.t_arr if steps is None else self.t_arr[:max(1, int(steps))]
        th = w * t
        if abs(w) < 1e-4:
            X, Y = v * t, np.zeros_like(t)
        else:
            R = v / w
            X, Y = R * np.sin(th), R * (1.0 - np.cos(th))
        dx = P[None, :, 0] - X[:, None]
        dy = P[None, :, 1] - Y[:, None]
        c, s_ = np.cos(th)[:, None], np.sin(th)[:, None]
        lx = c * dx + s_ * dy
        ly = -s_ * dx + c * dy
        clr_t = rect_clearance(lx, ly, self.front_len, self.rear_len, self.half_width).min(axis=1)
        return float(clr_t.min()), float(clr_t[-1])

    def _can_turn(self, angle: float) -> bool:
        """Xoay tai cho DUNG goc `angle` (rad) co an toan khong — quet footprint chu nhat that.

        Thay cho _can_rotate_in_place() o nhung cho xoay VE MOT HUONG CU THE. Luat cu doi
        trong CA VONG 360 do ban kinh 0.53 m: dung trong/ngay sau khung cua 0.81 m thi
        thanh cua chi cach ~0.4 m nen luat do LUON sai — xe khong the xoay 30 do ve phia
        nguoi du hoan toan an toan (nguoi dung buoc sang phai de xe xoay theo ma xe dung
        im, 23/09). Kiem tra lai moi nhip nen chi can quet du goc con lai.
        """
        T = max(1e-3, self.horizon_T)
        return self._arc_ok(0.0, float(np.clip(angle, -math.pi, math.pi)) / T)

    def _turn_dir_ok(self, d: float, w_abs: float) -> float:
        """Chieu xoay tim loi: thu chieu d truoc, khong duoc thi chieu nguoc. 0 = ca hai deu cham."""
        for dd in (d, -d):
            if self._arc_ok(0.0, dd * w_abs):
                return dd
        return 0.0

    # ─────────────────────────────────────────────────────────────────────
    # Ban do luoi cuc bo + tim duong (07/10)
    # ─────────────────────────────────────────────────────────────────────
    def _nav_plan(self, tx: float, ty: float, moving: bool,
                  standoff: Optional[float] = None, prefer_side: int = 0) -> Optional[Dict[str, Any]]:
        """Tim duong tren luoi quanh xe (khung base_link) toi CHO DUNG TOT NHAT gan nguoi.

        Vi sao: tang chon khe chi thu duong THANG 1.6 / 1.1 / 0.7 / 0.45 m. O xa khong huong nao lot
        thi no lui ve tam do ngan, luc do khe hep giua hai thung (hep hon xe) cung trong "thoang" ->
        xe chon nham ben roi ket (nguoi dung bao 07/10: nguoi dung sau thung 50x30 cm, thung thu hai
        cach 50 cm mot ben, ben kia trong — xe di vao ben co thung). Diem dich (cach nguoi 1 m) lai
        nam TRONG thung -> DWA dung im mai.

        Cach lam: o luoi bi chan neu tam xe dat o do se cach vat < nav_lethal_m; gan vat hon
        half_width + margin_hard (dung muc DWA doi) thi rat dat, them vung phat nav_soft_m. Dijkstra
        tu o cua xe. Chon o dich theo J = quang duong + nav_far_weight * (khoang cach toi nguoi vuot
        follow_distance): dung truoc thung cach nguoi 1.4 m (J ~ 1.6) re hon vong 2 m de toi 1.0 m;
        nguoi o xa hon nua thi vong. O tot nhat ngay duoi xe -> 'hold' (dung cho, nhin nguoi).
        """
        res = self.nav_res if _sp_dijkstra is not None else max(self.nav_res, 0.08)
        dist = math.hypot(tx, ty)
        x0, x1 = -1.0, float(np.clip(dist + 1.2, 2.0, 4.5))
        ey = float(np.clip(dist + 1.0, 1.8, 3.5))
        nxc = int(math.ceil((x1 - x0) / res))
        nyc = int(math.ceil(2.0 * ey / res))
        xs = x0 + (np.arange(nxc) + 0.5) * res
        ys = -ey + (np.arange(nyc) + 0.5) * res
        GX, GY = np.meshgrid(xs.astype(np.float32), ys.astype(np.float32))      # (ny, nx)

        P = self.obstacles
        if P.shape[0] > 0:
            keep = np.hypot(P[:, 0] - tx, P[:, 1] - ty) > self.nav_person_clear
            Q = P[keep].astype(np.float32)
        else:
            Q = np.zeros((0, 2), dtype=np.float32)
        d2 = np.full(GX.shape, np.inf, dtype=np.float32)
        for i in range(0, Q.shape[0], 48):
            q = Q[i:i + 48]
            dd = (GX[..., None] - q[:, 0]) ** 2 + (GY[..., None] - q[:, 1]) ** 2
            d2 = np.minimum(d2, dd.min(axis=2))
        dobs = np.sqrt(d2)
        r_in = self.half_width + self.margin_hard
        lethal = dobs < self.nav_lethal
        cost = 1.0 + self.nav_soft_cost * np.clip((r_in + self.nav_soft - dobs) / self.nav_soft, 0.0, 1.5) ** 2

        ri = int(np.clip(round((0.0 + ey) / res - 0.5), 0, nyc - 1))
        ci = int(np.clip(round((0.0 - x0) / res - 0.5), 0, nxc - 1))
        d0 = float(dobs[ri, ci])
        if d0 < self.nav_lethal:
            # Xe DA o sat vat hon nguong: cho di qua cac o quanh xe khong gan vat hon cho dang dung, de
            # duong di thoat ra duoc (khong thi moi o quanh xe deu cam -> khong co duong).
            lethal &= ~((np.hypot(GX, GY) < 0.35) & (dobs >= d0 - 0.01))
        lethal[ri, ci] = False                    # xe dang dung o day — luon cho xuat phat
        if prefer_side != 0 and dist > 0.3:
            # Giu ben vong da chon: o ben KIA duong thang xe -> nguoi dat gap 3
            lat = (tx * GY - ty * GX) / dist
            cost = cost * np.where(prefer_side * lat < -0.15, 3.0, 1.0)
        N = nxc * nyc
        start = ri * nxc + ci
        flat_cost = cost.ravel()
        free = ~lethal.ravel()

        if _sp_dijkstra is not None:
            idx = np.arange(N).reshape(nyc, nxc)
            us, vs, ws = [], [], []
            for dy, dx, L in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0))):
                ya, yb = max(0, -dy), nyc - max(0, dy)
                xa, xb = max(0, -dx), nxc - max(0, dx)
                u = idx[ya:yb, xa:xb].ravel()
                v = idx[ya + dy:yb + dy, xa + dx:xb + dx].ravel()
                ok = free[u] & free[v]
                u, v = u[ok], v[ok]
                us.append(u)
                vs.append(v)
                ws.append(L * res * 0.5 * (flat_cost[u] + flat_cost[v]))
            G = _csr_matrix((np.concatenate(ws), (np.concatenate(us), np.concatenate(vs))), shape=(N, N))
            D, pred = _sp_dijkstra(G, directed=False, indices=start, return_predecessors=True)
        else:
            D = np.full(N, np.inf)
            pred = np.full(N, -9999, dtype=np.int64)
            D[start] = 0.0
            hp = [(0.0, start)]
            nb = ((0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
                  (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142))
            while hp:
                du, u = heapq.heappop(hp)
                if du > D[u]:
                    continue
                uy, ux = divmod(u, nxc)
                for dy, dx, L in nb:
                    vy, vx = uy + dy, ux + dx
                    if 0 <= vy < nyc and 0 <= vx < nxc:
                        v = vy * nxc + vx
                        if free[v]:
                            nd = du + L * res * 0.5 * (flat_cost[u] + flat_cost[v])
                            if nd < D[v]:
                                D[v] = nd
                                pred[v] = u
                                heapq.heappush(hp, (nd, v))

        dp = np.hypot(GX - tx, GY - ty).ravel()
        kfar = self.nav_far_w_moving if moving else self.nav_far_w
        fd = self.follow_distance if standoff is None else float(standoff)
        cand = np.isfinite(D) & free & (dp >= fd - 0.3)
        if not np.any(cand):
            return None
        J = np.where(cand, D + kfar * np.maximum(0.0, dp - fd)
                     + 4.0 * np.maximum(0.0, fd - 0.1 - dp), np.inf)
        k = int(np.argmin(J))
        path = []
        cur = k
        for _ in range(N):
            cy_, cx_ = divmod(cur, nxc)
            path.append((float(xs[cx_]), float(ys[cy_])))
            if cur == start:
                break
            cur = int(pred[cur])
            if cur < 0:
                return None
        path.reverse()
        path[0] = (0.0, 0.0)
        tgt_xy = path[-1]
        hold = math.hypot(*tgt_xy) < self.nav_hold_r
        return {"path": path, "target": tgt_xy, "hold": hold, "cost": float(D[k]), "J": float(J[k]),
                "person_d": float(dp[k])}

    def _nav_lookahead(self, path, tx: float, ty: float) -> Tuple[float, float]:
        """Diem xa nhat tren duong (<= nav_lookahead_m) ma doan thang tu xe toi do khong cham vat."""
        P = self.obstacles
        if P.shape[0] > 0:
            P = P[np.hypot(P[:, 0] - tx, P[:, 1] - ty) > self.nav_person_clear]
        best = path[min(len(path) - 1, 4)]
        acc = 0.0
        for i in range(1, len(path)):
            acc += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if acc > self.nav_lookahead:
                break
            # hanh lang rong half_width + margin_soft: diem ngam khong "cat goc" sat vat (xe lai mui
            # vao goc thung roi het duong: tien hay xoay phai deu dua mep truoc vao goc)
            blk, _d = segment_blocked(P, path[i][0], path[i][1], self.half_width + self.margin_soft)
            if not blk:
                best = path[i]
        return best

    def _nav_get(self, now: float, tx: float, ty: float, moving: bool) -> Optional[Dict[str, Any]]:
        """Ke hoach duong di, lap lai moi nav_replan_sec (luu trong khung odom, doi ve base moi nhip)."""
        c = self.nav_cache
        if c is not None and (now - c["t"]) < self.nav_replan and c["tgt_odom"] is not None:
            ox, oy = self._odom_of(tx, ty)
            if math.hypot(ox - c["tgt_odom"][0], oy - c["tgt_odom"][1]) < 0.3:
                path = [self._base_of(px, py) for (px, py) in c["path_odom"]]
                path[0] = (0.0, 0.0)
                out = dict(c["plan"])
                out["path"] = path
                return out
        plan = self._nav_plan(tx, ty, moving)
        if plan is None:
            self.nav_cache = None
            return None
        # GIU BEN VONG: vat doi xung (ghe dai thang truoc nguoi) thi hai ben re ngang nhau, moi lan lap lai
        # duong lai doi ben -> xe xoay qua xoay lai tai cho mai (mo phong 07/10). Chi doi ben khi ben kia
        # re hon ro (J nho hon 1.25 lan + 0.5 m); khong lap duong qua 3 s thi bo cam ket.
        s_new = self._nav_side_of(plan, tx, ty)
        if self.nav_side != 0 and s_new == -self.nav_side and (now - self.nav_side_t) < 3.0:
            alt = self._nav_plan(tx, ty, moving, prefer_side=self.nav_side)
            if (alt is not None and not alt["hold"] and self._nav_side_of(alt, tx, ty) == self.nav_side
                    and alt["J"] <= 1.25 * plan["J"] + 0.5):
                plan, s_new = alt, self.nav_side
        if s_new != 0:
            self.nav_side = s_new
        self.nav_side_t = now
        self.nav_cache = {"t": now, "plan": plan, "tgt_odom": self._odom_of(tx, ty),
                          "path_odom": [self._odom_of(px, py) for (px, py) in plan["path"]]}
        return plan

    @staticmethod
    def _nav_side_of(plan: Dict[str, Any], tx: float, ty: float) -> int:
        """Ben vong cua duong di so voi duong thang xe -> nguoi: +1 trai, -1 phai, 0 gan nhu thang."""
        d = math.hypot(tx, ty)
        if d < 1e-3 or not plan["path"]:
            return 0
        lat = [(tx * py - ty * px) / d for (px, py) in plan["path"]]
        m = max(lat, key=abs)
        return 0 if abs(m) < 0.15 else (1 if m > 0 else -1)

    def _nav_follow(self, nav: Dict[str, Any], tx: float, ty: float, dt: float,
                    fov_target: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float, str, float]]:
        """Bam duong ban do. Tra ve (v, w, ghi chu, do thoang) hoac None.

        DWA nham diem ngam tren duong la du khi duong chi lech vua phai — no lai cung, giu duoc nguoi
        trong camera. Nhung khi duong re gat (vong qua mieng chu U, quanh dau ghe dai) DWA cat goc: mui
        xe lech cheo ~45 do, goc truoc (0.33 m tu tam) cham mat vat, xe bo 0.04 m/s roi dung han; xoay
        tai cho thi goc truoc quet vao vat, ma DWA khong lui -> dung mai (mo phong 07/10). Nen:
          - diem ngam o PHIA SAU (> 110 do): lui theo cung toi do (han muc lui chung voi dong tac chui
            khe — lidar mu phia sau), khong duoc thi xoay ve phia do;
          - lech hon nav_turn_first_deg (75 do, tre xuong 25 do): XOAY TAI CHO cho dung huong roi moi
            di. Nguong 60 do thi luc vong qua nguoi thu hai dung chen xe cung xoay han, mat camera 3-4 s;
            75 do thi khong, ma van gioi han duoc ca chu U;
          - con lai: DWA voi chi phi khung hinh noi ra nav_fov_keep_deg (50 do — chi phat khi nguoi sap
            ra khoi khung, khong keo mui xe ve phia vat dang chan nhu nguong 22 do);
          - DWA dung im (v ~ 0): xoay ve huong duong, khong duoc thi lui.
        """
        look = self._nav_lookahead(nav["path"], tx, ty)
        la = math.atan2(look[1], look[0])
        self.nav_look = look
        t_call = time.time()
        if t_call - self.nav_follow_t > 0.5:
            # lan dau / sau mot luc khong dung ban do: bo cac bo dem cu (khong thi vua quay lai da bi coi la ket)
            self.nav_rotating = False
            self.nav_fight_t = 0.0
            self.nav_behind_t = 0.0
        self.nav_follow_t = t_call

        def back() -> Optional[Tuple[float, float]]:
            vb = -self.gap_cross_speed
            if abs(la) > math.pi / 2:
                # diem ngam phia sau: lui theo cung toi do
                L2 = look[0] ** 2 + look[1] ** 2
                wb = float(np.clip(vb * 2.0 * look[1] / max(L2, 1e-4), -self.w_max, self.w_max))
            else:
                # diem ngam phia truoc ma xe ket (khong tien, khong xoay duoc — goc mui ti vao vat): lui ra
                # mot doan ngan, vua lui vua xoay mui dan ve phia diem ngam, cho co cho xoay
                wb = float(np.clip(1.5 * la, -0.5, 0.5))
            for w_try in (wb, 0.5 * wb, 0.0):
                if self._arc_ok(vb, w_try) and self._back_take():
                    return vb, w_try
            return None

        def rot() -> Optional[Tuple[float, float]]:
            if self._can_turn(la):
                return 0.0, float(np.clip(2.2 * la, -self.w_max * 0.9, self.w_max * 0.9))
            return None

        thr = math.radians(25.0) if self.nav_rotating else self.nav_turn_first
        if abs(la) > math.radians(110.0):
            # Duong bat dau bang doan LUI: xe da vao sat vat, hoac nguoi thu hai buoc toi sat mui xe. Con
            # thay nguoi thi DUNG CHO 2 s truoc (vat la nguoi thi ho thuong buoc di; lui+xoay ngay thi xe
            # quay lung voi nguoi dang bam, mo phong 07/10), roi xoay tai cho ve huong duong; khong xoay
            # duoc moi lui (lidar mu phia sau).
            now_ = time.time()
            if self.nav_behind_t == 0.0:
                self.nav_behind_t = now_
            if fov_target is not None and now_ - self.nav_behind_t < 2.0:
                return 0.0, 0.0, "ban do: duong di nam phia sau — dung cho", 0.0
            out = rot() or back()
            if out is not None:
                self.nav_rotating = out[0] == 0.0
                return out[0], out[1], ("ban do: lui ra" if out[0] < 0 else "ban do: xoay ve huong duong di"), 0.0
        else:
            self.nav_behind_t = 0.0
            if abs(la) > thr:
                out = rot()
                if out is not None:
                    self.nav_rotating = True
                    return out[0], out[1], "ban do: xoay ve huong duong di", 0.0
        self.nav_rotating = False
        res = self._dwa(look, fov_target, dt, allow_reverse=False, fov_active=fov_target is not None,
                        fov_keep=self.nav_fov_keep)
        side = "trai" if la > 0 else "phai"
        # DWA quay NGUOC huong duong di (duong sang phai ma DWA queo trai) = no khong bam duoc duong: thuong
        # la xoay ve phia duong thi goc mui quet vao vat, nen DWA chon queo nguoc lai — cu the xe chui dan vao
        # goc ket giua vat va huong nguoi (mo phong 07/10, nguoi di vong qua thung). Keo dai 0.8 s thi coi nhu
        # ket (thoang qua thi bo qua: nguoi thu hai buoc sat mui xe, xoay/lui ngay lam xe sat chan ho hon).
        now_ = time.time()
        if res is not None and abs(la) > math.radians(25.0) and res[1] * la < 0.0 and abs(res[1]) > 0.15:
            if self.nav_fight_t == 0.0:
                self.nav_fight_t = now_
        else:
            self.nav_fight_t = 0.0
        fights = self.nav_fight_t > 0.0 and now_ - self.nav_fight_t >= 0.8
        if res is not None and res[0] > 0.02 and not fights:
            return res[0], res[1], "di vong theo ban do (%s, %.1fm duong)" % (side, nav["cost"]), res[2]
        out = rot()
        if out is not None:
            self.nav_rotating = True
            return out[0], out[1], "ban do: xoay ve huong duong di", 0.0
        out = back()
        if out is not None:
            return out[0], out[1], "ban do: ket — lui ra cho de xoay", 0.0
        if res is not None:
            return res[0], res[1], "di vong theo ban do (%s, %.1fm duong)" % (side, nav["cost"]), res[2]
        return None

    def _odom_of(self, bx: float, by: float) -> Tuple[float, float]:
        c, s_ = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        return self.robot_x + c * bx - s_ * by, self.robot_y + s_ * bx + c * by

    def _base_of(self, ox: float, oy: float) -> Tuple[float, float]:
        dx, dy = ox - self.robot_x, oy - self.robot_y
        c, s_ = math.cos(-self.robot_yaw), math.sin(-self.robot_yaw)
        return c * dx - s_ * dy, s_ * dx + c * dy

    @staticmethod
    def _path_near(plan: Optional[Dict[str, Any]], mx: float, my: float, r: float = 0.35) -> bool:
        if plan is None:
            return True
        return any(math.hypot(px - mx, py - my) < r for (px, py) in plan["path"])

    def _arc_ok(self, v: float, w: float) -> bool:
        mn, en = self._arc_clearance(v, w)
        return bool(self._admissible(np.array([mn]), np.array([en]))[0])

    def _gap_maneuver(self, g: Dict[str, float]) -> Optional[Tuple[float, float, str]]:
        """Dong tac CHUI KHUNG CUA — dieu khien hinh hoc, khong phai DWA.

        Vi sao khong de DWA lam: cua 0.81 m, xe rong 0.60 m, duoi dai 0.33 m. Lech
        14 do la het le an toan. DWA nhin truoc 1.2 s (0.26 m) voi 7 thanh phan chi
        phi giang nhau (dich, huong, thoang, toc do, muot, khung hinh, ben ne) khong
        giu noi do chinh xac do — do la luc xe ep vao mot ben khung cua.

        Ba pha, dung thu tu:
          1. Con lech truc ma da qua gan mat khe -> LUI RA. Khong lui thi tu nhot minh:
             tuong ngay truoc mui, goc truoc quet 0.33 m nen het cho xoay.
          2. Con lech truc -> truot doc mat khe toi diem dung cho tren truc.
          3. Da vao truc -> canh mui theo phap tuyen khe roi chui, vua chui vua sua
             lech ngang.
        Moi lenh deu phai qua _arc_clearance truoc khi cho chay.
        """
        mx, my, nx, ny = g["mx"], g["my"], g["nx"], g["ny"]
        psi, along, lat = g["psi"], g["along"], g["lat"]
        stand = self.front_len + self.margin_soft + 0.25
        if along > self.gap_engage_range:
            # Khe con xa: de tang chon khe + DWA lai nhu binh thuong. Vao dong tac nay
            # tu xa thi xe bo chi phi giu khung hinh va chay cham suot doan duong dai.
            return None

        # DA SAT KHUNG CUA (duoi margin_hard): xoay NHE tai cho ve phia lam xe xa khung
        # cua hon — dung dieu nguoi dung lam bang tay khi xe ket (23/09). Uu tien hon lui:
        # xoay nhe chi dua duoi xe ra ngang vai cm, con lui thi di thang vao cung sau
        # ma lidar khong nhin thay (blind_sectors_deg).
        c0 = self._clear_now()
        if c0 <= self.margin_hard:
            best = None
            for w_try in (0.15, -0.15, 0.30, -0.30):
                mn, en = self._arc_clearance(0.0, w_try)
                if not bool(self._admissible(np.array([mn]), np.array([en]))[0]):
                    continue
                # cung nhu nhau thi chon chieu dua mui ve phap tuyen khe
                score = en + (0.005 if w_try * psi > 0.0 else 0.0)
                if best is None or score > best[1]:
                    best = (w_try, score)
            if best is not None:
                return 0.0, best[0], "chui khe hep — sat khung cua %.0fcm, xoay nhe ve phia rong" % (
                    100.0 * c0)

        # Dung sai "da vao truc" tinh theo KHE THAT: cua 0.81 m, xe 0.60 m -> du 10.5 cm moi
        # ben, tru khoang ho toi thieu -> chi con vai cm. Dung sai co dinh 12 cm (cu) cho xe
        # lech 7.5 cm vao pha chui -> vat ly khong lot -> dung im giua cua (29/09).
        # Tinh bang margin_hard (hinh hoc that), KHONG bang gap_cross_margin: nguong do chi
        # bu nhieu do cua lidar. Dung chung thi dung sai noi ra, xe co chui khi con lech 6 cm
        # la vat ly khong lot (mo phong 29/09: nhieu lan dung im hon).
        tol = min(self.gap_axis_tol,
                  max(0.02, 0.5 * (g["gw"] - 2.0 * self.half_width) - self.margin_hard - 0.01))
        now_ = time.time()
        if now_ - self.gap_cross_time < 1.0:
            tol += 0.03                            # tre: dang chui thi khong nhay ve pha canh truc
        backing = (now_ - self.gap_back_time < 0.5) and along < stand + 0.05 and lat > 0.5 * tol
        if (lat > tol and along < stand - 0.05) or backing:
            # Da qua gan mat khe ma con lech truc -> LUI VE TRUC roi vao lai.
            # Lui THEO CUNG toi mot diem tren truc khe phia sau (pure-pursuit cho chieu lui),
            # khong lui thang: lui thang thi van lech nguyen do, con qua it cho de vua tien
            # vua nan -> xe lai co chui, lai lui, het han muc (mo phong 29/09). Da bat dau lui
            # thi lui cho du toi cho dung (along >= stand) moi tien lai.
            # GIOI HAN quang lui: cung sau xe nam trong blind_sectors_deg nen lidar KHONG
            # THAY gi thang phia sau. Chi lui lai dung doan vua di qua.
            rx, ry = mx + nx * (stand + 0.10), my + ny * (stand + 0.10)
            kappa = 2.0 * ry / max(1e-3, rx * rx + ry * ry)
            v_b = -self.gap_cross_speed
            w_b = float(np.clip(v_b * kappa, -0.40, 0.40))
            if abs(w_b) < self.min_move_w and abs(w_b) > 0.02:
                w_b = math.copysign(self.min_move_w, w_b)
            for w_try in (w_b, 0.0):
                if self._arc_ok(v_b, w_try) and self._back_take():
                    return v_b, w_try, "chui khe hep — lui ve truc khe (lech %.2fm)" % lat
            v_des, head_des, turn_thr = 0.0, psi, math.radians(35.0)
            note = "chui khe hep — het han lui, canh goc tai cho"
        elif lat > tol:
            # Ngam co NHIN TRUOC (pure-pursuit): diem ngam nam tren truc khe nhung o phia
            # truoc xe mot doan look, chu khong ngang hong xe. Ngam ngang hong thi lech
            # truc 13 cm cung ra lenh quay 90 do de "truot ngang" -> xe quay vong tai cho.
            look = min(0.9, max(0.45, along - stand + 0.45))
            aim = max(stand, along - look)
            ax, ay = mx + nx * aim, my + ny * aim
            head_des = math.atan2(ay, ax)
            v_des, turn_thr = min(self.v_max, 0.8 * math.hypot(ax, ay)), math.radians(35.0)
            note = "chui khe hep — vao truc giua khe (lech %.2fm)" % lat
        else:
            self.gap_cross_time = now_
            return self._gap_cross(g, c0)

        err = wrap_pi(head_des)          # mui xe = 0 trong base_link
        # Vung chet goc: khong co thi cac lenh quay ti hon bi min_move_angular keo thanh
        # +-0.10 rad/s doi chieu lien tuc, xe lac qua lai khi dang chui khe.
        w_des = 0.0 if abs(err) < math.radians(2.0) else float(
            np.clip(2.0 * err, -self.w_max * 0.8, self.w_max * 0.8))
        if v_des > 0.0:
            # Lech nhieu thi xoay tai cho truoc, khong di cheo. Giam dan theo cos(lech)
            # chu khong cat dot ngot — cat dot ngot lam xe dung-di-dung lien tuc.
            v_des = 0.0 if abs(err) > turn_thr else v_des * math.cos(err)
        for v_try in (v_des, 0.5 * v_des, 0.0):
            if self._arc_ok(v_try, w_des):
                return v_try, w_des, note
        # Khong tien/xoay duoc (vd goc xe da chia qua thanh cua, xoay vuong thi ep goc
        # xuong tuong ben canh): LUI-XOAY nhu lui xe vao chuong — lui nhe cho goc xe nhac
        # khoi tuong, vua lui vua be mui ve truc khe. Dung im o day thi ket mai.
        for w_try in (w_des, 0.5 * w_des, 0.0):
            if self._arc_ok(-self.gap_cross_speed, w_try) and self._back_take():
                return -self.gap_cross_speed, w_try, note + " — lui xoay de canh lai"
        if c0 > self.margin_hard:
            return 0.0, 0.0, note + " — dung cho"
        return None

    def _gap_cross(self, g: Dict[str, float], c0: float) -> Optional[Tuple[float, float, str]]:
        """Pha QUA KHE (xe da vao truc): giu mui theo phap tuyen khe, vua di vua sua lech ngang.

        Luat cu (29/09 nguoi dung bao xe dung im giua cua 0.81 m, "lech -3..-7 do"):
        huong = psi + 1.2*lech_ngang. Xe lech tam VA lech goc thi hai so hang nguoc dau,
        cong lai con 2 do -> roi vao vung chet -> w = 0; tien thi duoi xe sat khung ->
        v = 0 -> dung im mai. Bay gio, theo thu tu, lay lenh DAU TIEN an toan:
          1. di toi theo luat tren (vung chet chi 1 do, lenh quay nho nang len 0.10)
          2. xoay tai cho ve huong do (xe vuong mat cua la thoang nhat)
          3. xoay nhe ve phia lam xe thoang hon
          4. lui-xoay de canh lai truc
        Chi dung im khi ca bon deu khong an toan.
        """
        mx, my, psi, along = g["mx"], g["my"], g["psi"], g["along"]
        e_left = mx * (-math.sin(psi)) + my * math.cos(psi)
        head_des = wrap_pi(psi + float(np.clip(1.2 * e_left, -0.35, 0.35)))
        # Chi bo cham khi DA SAT mat khe; cham ca doan duong dai thi nguoi di mat
        v_c = self.gap_cross_speed if along <= 0.75 else self.v_max
        steps = max(1, int(round(0.6 / max(1e-3, self.horizon_dt))))
        err = head_des
        if abs(err) <= math.radians(1.0):
            w_des = 0.0
        else:
            w_des = float(np.clip(2.0 * err, -self.w_max * 0.8, self.w_max * 0.8))
            if abs(w_des) < self.min_move_w:
                w_des = math.copysign(self.min_move_w, w_des)
        tag = "(lech %.0f do)" % math.degrees(psi)

        def ok(v, w):
            mn, en = self._arc_clearance(v, w, steps)
            return bool(self._admissible_cross(mn, en, c0))

        if abs(err) < math.radians(12.0):
            v_des = v_c * math.cos(err)
            # Uu tien DI THANG neu thang lot: be lai trong khe lam duoi xe (0.33 m) vang ra,
            # an mat vai cm dung o cho khong con cm nao. Lech ngang con lai nho thi cu thang.
            for v_try, w_try in ((v_des, w_des), (v_des, 0.0), (0.5 * v_des, w_des), (0.5 * v_des, 0.0)):
                if ok(v_try, w_try):
                    return v_try, w_try, "chui khe hep — qua khe theo truc " + tag
        if w_des != 0.0 and ok(0.0, w_des):
            return 0.0, w_des, "chui khe hep — xoay vuong mat cua " + tag
        # Khong tien, khong xoay vuong duoc: LUI VE TRUC mot doan cho co cho vao lai. Dat
        # TRUOC "xoay nhe cho thoang": o mieng cua, xoay cho thoang la quay mui TRANH thanh
        # cua gan -> lech khoi truc toi 18 do roi lai xoay ve, lac mai (mo phong 29/09).
        stand = self.front_len + self.margin_soft + 0.25
        rx, ry = mx + g["nx"] * (stand + 0.10), my + g["ny"] * (stand + 0.10)
        kappa = 2.0 * ry / max(1e-3, rx * rx + ry * ry)
        w_back = float(np.clip(-self.gap_cross_speed * kappa, -0.40, 0.40))
        for w_try in (w_back, 0.0):
            if ok(-self.gap_cross_speed, w_try) and self._back_take():
                return -self.gap_cross_speed, w_try, "chui khe hep — lui ve truc de vao lai " + tag
        # Da sat duoi nguong: xoay nhe ve phia lam xe thoang hon (go ket)
        if c0 <= self.gap_cross_margin:
            rot = None
            for w_try in (0.15, -0.15, 0.30, -0.30):
                mn, en = self._arc_clearance(0.0, w_try, steps)
                if en > c0 + 0.005 and bool(self._admissible_cross(mn, en, c0)):
                    sc = en + (0.005 if w_try * head_des > 0.0 else 0.0)
                    if rot is None or sc > rot[0]:
                        rot = (sc, w_try)
            if rot is not None:
                return 0.0, rot[1], "chui khe hep — sat khung cua %.0fcm, xoay nhe cho thoang " % (
                    100.0 * c0) + tag
        return None

    def _admissible_cross(self, mn: float, en: float, c0: float) -> bool:
        """Luat an toan khi DANG CHUI KHE: nhu _admissible nhung nguong cung la
        gap_cross_margin_m (xem config) thay cho margin_hard."""
        if mn > self.gap_cross_margin:
            return True
        return c0 <= self.gap_cross_margin and mn >= c0 - 0.005 and en >= c0 + 0.01 and mn > 0.01

    def _back_take(self) -> bool:
        """Con han muc lui khong (va tru di mot nhip). Lidar KHONG nhin thang phia sau
        (blind_sectors_deg) nen chi cho lui toi da gap_back_max_m moi lan chui khe."""
        now_ = time.time()
        if now_ - self.gap_back_time > 1.0:
            self.gap_back_used = 0.0              # roi xa lan lui truoc -> cap lai han muc
        if self.gap_back_used >= self.gap_back_max:
            return False
        self.gap_back_time = now_
        self.gap_back_used += self.gap_cross_speed / max(1.0, self.control_hz)
        return True

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
        head_ref: Optional[float] = None,
        head_weight: Optional[float] = None,
        v_cap: Optional[float] = None,
        center_active: bool = False,
        fov_keep: Optional[float] = None,
    ) -> Optional[Tuple[float, float, float]]:
        """Tra ve (v, w, clearance) tot nhat, hoac None neu khong co lua chon an toan."""
        gx, gy = goal_xy

        # 1. Cua so dong hoc — rong bang accel * sample_window, KHONG phai accel * dt_ctrl.
        #    Xem ghi chu o tham so sample_window_sec: cua so 1-tick lam xe dung yen.
        dv = self.accel_lin * self.sample_window
        dw = self.accel_ang * self.sample_window
        v_lo = max(self.v_min if allow_reverse else 0.0, self.cur_v - dv)
        v_hi = min(self.v_max, self.cur_v + dv)
        if v_cap is not None:
            # Chan toc do tien: v_cap=0 => chi con xoay tai cho, nhung VAN qua
            # bo loc va cham cua DWA (footprint chu nhat quay quanh tam).
            v_hi = min(v_hi, v_cap)
            v_lo = min(v_lo, v_hi)
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
            clr_t = clear.min(axis=2)                         # (N,K) tung buoc
            min_clear = clr_t.min(axis=1)                     # (N,)
            end_clear = clr_t[:, -1]
        else:
            min_clear = np.full(n, 10.0)
            end_clear = min_clear

        admissible = self._admissible(min_clear, end_clear)
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

        # head_ref: huong mui xe MONG MUON o cuoi quy dao. Binh thuong la huong
        # toi dich; khi chui khe hep thi la PHAP TUYEN cua khe — xe phai vuong goc
        # voi mat cua truoc khi qua, khong thi duoi xe (0.33 m) quet ra ngoai khe.
        if head_ref is None:
            ang_to_goal = np.arctan2(gy - ye, gx - xe)
        else:
            ang_to_goal = float(head_ref)
        c_head = np.abs(wrap_pi_arr(ang_to_goal - te)) / math.pi
        w_head = self.w_heading if head_weight is None else float(head_weight)

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
            fk = self.fov_keep_rad if fov_keep is None else float(fov_keep)
            c_fov = np.clip((np.abs(tb) - fk) / max(1e-3, math.pi - fk), 0.0, 1.0)
            # GIU NGUOI GIUA KHUNG HINH SUOT QUY DAO, khong chi o diem cuoi. c_head/c_fov chi xet tu
            # the CUOI (sau 1.2 s) nen DWA chon quay tu tu cho vua het 1.2 s (mo phong: w = 0.28
            # rad/s du duoc phep 1.0); nguoi di tiep nen xe tre ~15 do. Nguoi dung thay xe xoay
            # theo "hoi cham" (29/09). Vung chet bearing_deadband de di thang khong lac.
            # CHI khi duong toi nguoi thoang (center_active): luc NE, keo mui ve phia nguoi chinh
            # la lao thang vao nguoi/vat chen giua (test_sim test 5: xe dung sat nguoi chen).
            if self.w_center > 0.0 and center_active:
                tb_t = np.abs(wrap_pi_arr(np.arctan2(ty - Y, tx - X) - theta))      # (N,K)
                c_center = np.clip(np.mean(np.maximum(tb_t - self.bearing_deadband, 0.0), axis=1)
                                   / max(1e-3, self.fov_keep_rad), 0.0, 2.0)
            else:
                c_center = np.zeros(n)
        else:
            c_fov = np.zeros(n)
            c_center = np.zeros(n)

        if self.avoid_side != 0:
            c_side = np.where(np.sign(W) == -self.avoid_side, np.abs(W) / max(1e-3, self.w_max), 0.0)
        else:
            c_side = np.zeros(n)

        cost = (
            self.w_goal * c_goal
            + w_head * c_head
            + self.w_clear * c_clear
            + self.w_speed * c_speed
            + self.w_smooth * c_smooth
            + self.w_fov * c_fov
            + self.w_center * c_center
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
                bx, by = self._base_of(ox, oy)
                gb = math.atan2(by, bx)
                gr = math.hypot(bx, by) - self.follow_distance
                # Tuong chan giua xe va cho thay nguoi lan cuoi (cua ben hong): chui qua
                # cua, dung dung lai quet tai cho — quet o ben nay tuong thi khong bao gio
                # thay lai nguoi.
                if (now - self.search_start) < self.search_goto_max:
                    cor = self.half_width + self.margin_hard
                    blk, _bd = segment_blocked(self.obstacles, bx, by, cor)
                    if blk:
                        g2 = self._gap_target(gb, math.hypot(bx, by), (bx, by))
                        if g2 is not None:
                            out2 = self._gap_maneuver(g2)
                            if out2 is not None:
                                v, w = self._emit(out2[0], out2[1], dt)
                                self._report(status, v, w, "mat nguoi — " + out2[2], now)
                                return
                # Duong toi CHO NGUOI bi chan (nguoi thu hai dung chen che mat nguoi dang bam):
                # diem dich (lui follow_distance) roi dung cho ho dung -> truoc day xe bo toi sat
                # chan ho roi dung quay tim (nguoi dung bao 29/09). Coi nhu chua toi noi, va do
                # huong di toi tan cho nguoi (nhu chk_r cua che do bam) de VONG QUA ho.
                p_r = math.hypot(bx, by)
                p_blk, _pbd = segment_blocked(self.obstacles, bx, by, self.half_width + self.margin_hard)
                if (gr > self.dist_deadband or (p_blk and p_r > 0.6)) and \
                        (now - self.search_start) < self.search_goto_max:
                    # Duong bi chan -> ban do luoi (07/10): chon dung ben vong, dich trong vat thi dung o
                    # cho tot nhat thay vi dung im truoc vat. Dat TRUOC luat "lech > 60 do thi xoay ve phia
                    # nguoi": dang vong qua vat ma mui lech khoi cho nguoi la chuyen phai co — xoay ve thi
                    # pha hong duong vong, mui chuc vao vat (mo phong chu U 07/10).
                    if self.nav_enabled and p_blk:
                        nav = self._nav_get(now, bx, by, False)
                        if nav is not None and not nav["hold"]:
                            out = self._nav_follow(nav, bx, by, dt, None)
                            if out is not None:
                                v, w = self._emit(out[0], out[1], dt)
                                self._report(status, v, w, "mat nguoi — lai toi cho thay lan cuoi (" + out[2] + ")",
                                             now, clearance=out[3],
                                             chosen_heading=math.atan2(self.nav_look[1], self.nav_look[0]))
                                return
                        if nav is not None and nav["hold"]:
                            gr = -1.0          # da o cho gan nhat co the -> sang pha quay mat / do
                    # Chi xoay tai cho khi dich lech hon 60 do: DWA tu lai duoc trong ±100 do.
                    # Nguong thap (nhu occluded_turn_deg) khien xe di doc tuong cu dung-xoay-di.
                    turn_thr = math.radians(60.0) * (0.5 if self.occl_turning else 1.0)
                    if gr > -1.0 and abs(gb) > turn_thr and self._can_turn(gb):
                        self.occl_turning = True
                        w_cmd = float(np.clip(2.2 * gb, -self.w_max * 0.9, self.w_max * 0.9))
                        v, w = self._emit(0.0, w_cmd, dt)
                        self._report(status, v, w, "mat nguoi — quay ve cho thay lan cuoi", now)
                        return
                    self.occl_turning = False
                    chk = max(gr, p_r - self.target_clear_r) if p_blk else gr
                    phi, reach, _direct = self._choose_heading(gb, chk, gb) if gr > -1.0 else (None, 0.0, False)
                    if phi is not None:
                        sub_r = min(reach, max(gr, 0.4) if p_blk else gr)
                        res = self._dwa((sub_r * math.cos(phi), sub_r * math.sin(phi)), None, dt,
                                        allow_reverse=False, fov_active=False)
                        if res is not None:
                            v, w = self._emit(res[0], res[1], dt)
                            self._report(status, v, w, "mat nguoi — lai toi cho thay lan cuoi",
                                         now, clearance=res[2], chosen_heading=phi)
                            return
                # Toi noi, het gio, hoac bi chan -> pha 2: quay mat ve cho thay lan cuoi roi do / quet
                self.search_goto = False
                self.occl_turning = False
                self.search_start = now
                self._search_next(now, "face")
            if self.state == S_SEARCH:
                if self.search_phase is None:
                    self._search_next(now, "face")
                self._search_step(now, dt, status)
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
        self.search_phase = None
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

        # ── BAN DO LUOI + TIM DUONG khi duong thang toi dich bi chan (07/10) ─
        # Hoac khi DICH cham le an toan (xe dung o dich, quay mat ve nguoi, thi footprint sat vat hon
        # margin_hard): vd. dich ngay truoc thung thap ma nguoi dung sau — duong thang toi dich "thoang"
        # nhung DWA khong bao gio toi duoc -> truoc day dung im mai o FOLLOW, v = 0 (13.12; mo phong 07/10
        # camera ngua sai 3 do). Ban do se chon cho dung tot nhat (thuong la 'hold' ngay do). Phai keo dai
        # 1.5 s: nguoi thu hai buoc vao dung ngay diem dich roi di ngay thi xe cho nhu cu, giu camera.
        nav = None
        spd = float(tgt.get("speed") or 0.0)
        if (not blocked and goal_r > self.dist_deadband
                and self._pose_clearance(gx, gy, bearing) <= self.margin_hard):
            if self.goal_bad_since == 0.0:
                self.goal_bad_since = now
        else:
            self.goal_bad_since = 0.0
        goal_bad = self.goal_bad_since > 0.0 and (now - self.goal_bad_since) >= 1.5
        if self.nav_enabled and (blocked or goal_bad):
            nav = self._nav_get(now, tx, ty, spd > 0.25)
        else:
            self.nav_cache = None

        # ── CHUI KHE HEP (khung cua) ─────────────────────────────────────
        # Chi khi duong thang toi dich KHONG lot. Dat TRUOC nhanh "xoay ve huong nho
        # cuoi": nguoi re vao cua ben hong thi tuong che camera ngay, ma neu xe dung
        # xoay tai cho tim nguoi thi khong bao gio toi duoc cua. Da biet nguoi o dau
        # (khung odom) thi cu chui qua cua roi tinh tiep.
        # Chan giua xe va NGUOI cung tinh, khong chi chan toi diem dich: nguoi vua
        # buoc qua cua thi diem dich (lui lai follow_distance) con nam BEN NAY tuong,
        # duong toi no thoang, xe tuong da toi noi va dung lai truoc tuong.
        blocked_person, _bpd = segment_blocked(self.obstacles, tx, ty, corridor)
        # Chi chui khe khi: chua toi khoang cach bam (hoac nguoi DANG DI — nguoi buoc qua cua cham thi xe
        # da o sat 1 m ma van phai canh truc cua ngay, cho ho xa ra moi canh thi ho da khuat sau tuong),
        # ban do khong bao "dung cho la tot nhat", va duong di (neu co) DI QUA khe do. Truoc 07/10 khe
        # nao 0.74-1.2 m giua xe va nguoi cung bi chui — ke ca khe giua hai thung khi ben kia trong (log
        # nguoi dung: "vao truc giua khe" roi "ket hoan toan"), va khe giua hai mon do khi xe da toi noi
        # va nguoi dung yen (lac khi ARRIVED, 13.37).
        if ((blocked or blocked_person) and (goal_r > self.dist_deadband or spd > 0.15)
                and not (nav is not None and nav["hold"])):
            gap = self._gap_target(goal_b, max(goal_r, dist), (tx, ty))
            if gap is not None and not self._path_near(nav, gap["mx"], gap["my"]):
                gap = None
            if gap is not None:
                out = self._gap_maneuver(gap)
                if out is not None:
                    v_c, w_c, note = out
                    self.avoid_side = 0    # c_side se keo xe lech khoi truc khe
                    self.occl_turning = False
                    v, w = self._emit(v_c, w_c, dt)
                    self._report(status, v, w, note, now, dist, bearing, source)
                    if self.publish_markers:
                        self._publish_markers(gap["mx"], gap["my"], tx, ty)
                    return

        # ── NGUOI RA KHOI CAMERA: xoay ve huong nho cuoi truoc ───────────
        # Camera co dinh nen phai QUAY XE moi thay lai nguoi. Camera khong thay (chi con
        # lidar_track hoac du doan) ma nguoi lech qua occluded_turn_deg -> xoay tai cho ve
        # phia do (co tre: quay toi duoi mot nua nguong), roi moi tien toi. Goc du doan
        # thuong tre hon goc that ~10 do nen nguong dat thap hon nua FOV. Chi xoay khi
        # du cho cho duoi xe.
        camera_blind = not source.startswith("camera")
        turn_thr = self.occluded_turn_rad * (0.5 if self.occl_turning else 1.0)
        detouring = self.state == S_AVOID or (nav is not None and not nav["hold"])
        if (camera_blind and self.occluded_turn_rad > 0.0 and abs(bearing) > turn_thr
                and not detouring and self._can_turn(bearing)):
            self.occl_turning = True
            self._set_state(S_OCCLUDED)
            # He so 2.2 va tran 0.9*w_max (truoc 1.5 va 0.6*w_max = 0.48 rad/s): nguoi dung thay xe
            # xoay theo nguoi "hoi cham" so voi toc do di binh thuong (29/09). Mo phong buoc ngang
            # nhanh: mat camera 0.9 -> 0.3 s.
            w_cmd = float(np.clip(2.2 * bearing, -self.w_max * 0.9, self.w_max * 0.9))
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
            if abs(bearing) > self.bearing_deadband:
                if self._can_turn(bearing):
                    w_cmd = float(np.clip(2.2 * bearing, -self.w_max * 0.9, self.w_max * 0.9))
                else:
                    # Xoay tai cho se quet DUOI xe (0.33 m) vao vat — thuong la vua qua cua,
                    # duoi con nam giua hai thanh cua. Nhich toi theo cung ve phia nguoi cho
                    # duoi ra khoi khung cua roi moi xoay, thay vi dung im nhin nguoi di mat.
                    res = self._dwa((0.3 * math.cos(bearing), 0.3 * math.sin(bearing)), (tx, ty), dt,
                                    allow_reverse=False, fov_active=True)
                    if res is not None and res[0] > 0.0:
                        v, w = self._emit(res[0], res[1], dt)
                        self._report(status, v, w, "duoi xe vuong — nhich toi de xoay ve phia nguoi",
                                     now, dist, bearing, source, res[2])
                        return
            v, w = self._emit(0.0, w_cmd, dt)
            self._report(status, v, w, "canh huong tai cho", now, dist, bearing, source)
            return

        # ── DI THEO BAN DO (07/10) ───────────────────────────────────────
        if nav is not None:
            self.avoid_side = 0                    # ben ne do duong di quyet dinh, c_side khong keo lai
            if nav["hold"]:
                # Cho tot nhat la ngay day (vd. nguoi dung sau vat thap: vong qua de gan them vai chuc
                # cm phai di 2 m) -> dung, chi canh huong de camera giu nguoi giua khung.
                w_cmd = 0.0
                if abs(bearing) > self.bearing_deadband and self._can_turn(bearing):
                    w_cmd = float(np.clip(2.2 * bearing, -self.w_max * 0.9, self.w_max * 0.9))
                self._set_state(S_ARRIVED)
                v, w = self._emit(0.0, w_cmd, dt)
                self._report(status, v, w, "vat chan giua — dung o cho tot nhat, cach nguoi %.2fm" % dist,
                             now, dist, bearing, source)
                return
            # VAN giu chi phi khung hinh (c_fov, noi ra nav_fov_keep_deg): tat di thi xe vong qua nguoi thu
            # hai dung chen voi mui lech toi 60 do, mat camera 3.6 s (mo phong chan_giua, sau_ne_re; bat
            # lai: 0 s). c_center van tat (chi khi thoang). Chi tiet: _nav_follow.
            out = self._nav_follow(nav, tx, ty, dt, (tx, ty))
            if out is not None:
                v, w = self._emit(out[0], out[1], dt)
                self._report(status, v, w, out[2], now, dist, bearing, source, out[3],
                             math.atan2(self.nav_look[1], self.nav_look[0]))
                if self.publish_markers:
                    self._publish_markers(self.nav_look[0], self.nav_look[1], tx, ty)
                return

        # ── TANG 1: chon huong di qua khe (VFH) ──────────────────────────
        # Do khe toi tan cho nguoi (chk_r) de huong thang bi chan boi nguoi thu hai thi
        # chon khe ben canh; dich phu ben duoi van gioi han trong goal_r.
        phi, reach, _direct = self._choose_heading(goal_b, chk_r, bearing)

        if phi is None:
            # Khong huong nao di duoc: xoay tai cho tim loi, hoac dung han
            w_rot = min(self.search_w, self.w_max * 0.5)
            d = self._turn_dir_ok(
                self.avoid_side if self.avoid_side != 0 else (1.0 if bearing >= 0 else -1.0), w_rot)
            if d != 0.0:
                self._set_state(S_BLOCKED)
                v, w = self._emit(0.0, d * w_rot, dt)
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
        res = self._dwa((sgx, sgy), (tx, ty), dt, allow_reverse=False, fov_active=True,
                        center_active=(not blocked and self.state != S_AVOID))

        if res is None:
            w_rot = min(self.search_w, self.w_max * 0.5)
            d = self._turn_dir_ok(
                self.avoid_side if self.avoid_side != 0 else (1.0 if phi >= 0 else -1.0), w_rot)
            if d != 0.0:
                self._set_state(S_BLOCKED)
                v, w = self._emit(0.0, d * w_rot, dt)
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
