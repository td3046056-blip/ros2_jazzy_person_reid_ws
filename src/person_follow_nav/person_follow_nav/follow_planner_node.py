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
from collections import deque
from typing import Any, Dict, Optional, Tuple

import heapq
from time import perf_counter as _perf   # do thoi gian THAT cho gioi han CPU (mo phong thay module time)

import numpy as np
import rclpy
try:                                       # tim duong tren luoi nhanh hon ~10 lan; khong co thi dung heapq
    from scipy.sparse import csr_matrix as _csr_matrix
    from scipy.sparse.csgraph import dijkstra as _sp_dijkstra
except Exception:                          # pragma: no cover
    _csr_matrix = None
    _sp_dijkstra = None
try:                                       # khoang cach toi vat tren luoi ban do (03/11)
    from scipy.ndimage import distance_transform_edt as _edt
except Exception:                          # pragma: no cover
    _edt = None
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
        self.gap_axis_prev: Optional[Tuple[float, float, float, float]] = None   # truc khe nhip truoc (odom)
        self.gap_seen_t = 0.0               # lan cuoi THAY khe truc tiep (khong tinh lan dung lai khe da nho)
        self.gap_none_since = 0.0           # lan cuoi dong tac chui khe CO lenh (hoac khong co khe) — de dung cho ngan
        self.gap_mem: Optional[Dict[str, float]] = None   # khe da chon gan nhat (odom) — dung lai o mieng cua
        self.last_target_odom: Optional[Tuple[float, float]] = None   # cho thay nguoi lan cuoi
        self.search_goto = False            # SEARCH dang o pha lai toi cho do
        self.search_goto_lim = 0.0          # han thoi gian pha di toi (keo dai theo do dai duong ban do, 05/11)
        self.nav_rotating = False           # bo bam duong ban do dang xoay tai cho ve huong duong
        self.nav_side = 0                   # ben vong da cam ket (+1 trai / -1 phai)
        self.nav_side_t = 0.0
        self.nav_look = (0.0, 0.0)
        self.nav_behind_t = 0.0             # luc bat dau thay duong di nam phia sau xe
        self.goal_bad_since = 0.0           # luc bat dau thay dich cham le an toan
        self.nav_fight_t = 0.0              # luc DWA bat dau queo nguoc huong duong ban do
        self.nav_follow_t = 0.0             # lan goi _nav_follow gan nhat
        self.esc: Optional[Dict[str, Any]] = None   # ke hoach THOAT KET dang chay (05/11)
        self.esc_try_t = 0.0                # lan cuoi thu lap ke hoach thoat ket (gioi han CPU)
        self.prog_hist: deque = deque(maxlen=400)   # (t, x, y) — phat hien xe dung mot cho qua lau
        self.stuck_reset_t = 0.0            # lan cuoi xe DUOC PHEP dung (toi noi / ban do bao dung cho)
        self.nav_ext_t = 0.0                # lan cuoi ban do phai nhin XA sau xe moi thay loi ra (bay sau, 06/11)
        self.nav_ext_try_t = 0.0            # lan cuoi thu lap ban do nhin xa (gioi han CPU)
        self.creep_acc = 0.0                # quang da "nhich toi de xoay" trong dot hien tai (06/11)
        self.creep_t = 0.0
        self.face_since = 0.0               # luc bat dau: da toi noi, nguoi lech nhieu, khong xoay tai cho duoc
        self.retrace_used = 0.0             # quang da "lui theo vet" o bay sau (06/11)
        self.retrace_t = 0.0
        self.esc_fail_t = 0.0               # lan cuoi tim thoat ket KHONG ra ke hoach
        self.esc_fail_ext = False
        self.esc_retreat_t = 0.0            # tim khong ra luc dang sat vat -> lui ngan theo vet (06/11)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.scan_pts = np.zeros((0, 2))
        self.scan_time = 0.0
        self.obstacles = np.zeros((0, 2))
        # 03/11: BO NHO VAT CAN (khung odom, o luoi mem_cell) — vat da thay roi lot vao vung mu sau duoi xe /
        # tia quet song song mat tuong van duoc tinh (xe that 15/10: xoay roi lui, dung vat phia sau).
        self.mem_xy = np.zeros((0, 2))
        self.mem_t = np.zeros(0)
        self.mem_n = np.zeros(0)                # so lan cham da gop vao o (trung binh)
        # Diem dung cho BAN DO / TIM KHE: obstacles + diem noi lien hai tia ke nhau (tuong o goc cheo) + bo nho xa hon
        self.obstacles_dense = np.zeros((0, 2))
        self.obstacles_gap = np.zeros((0, 2))
        self.obstacles_full = np.zeros((0, 2))   # nhu obstacles nhung KHONG cat max_obstacle_points (do khe, phia truoc)
        self.mem_occl = np.zeros((0, 2))
        self.mem_all_base = np.zeros((0, 2))
        self.scan_hist: deque = deque(maxlen=48)   # (t, x, y, yaw, ranges, a0, ainc) moi 0.5 s — da thay TRONG o dau
        self.pose_trail: deque = deque(maxlen=600)  # (t, x, y, yaw) — cho than xe da di qua la cho trong
        self.tgt_trail: deque = deque(maxlen=240)   # (t, x, y) nguoi dang bam (khung odom) — khong nho chan nguoi
        self.back_upd_t = 0.0                       # lan cuoi cap nhat han muc lui (hoan dan)
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
        self.cam_seen_t = 0.0               # lan cuoi CAMERA thay nguoi (nguon camera+...)
        self.open_tries = 0                 # SEARCH: so lan nhich ra cho thoang vi khong xoay duoc
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
            # TOC DO THICH UNG (03/11): dang bam nguoi o cho THOANG (camera thay nguoi, khong vat chan, phia truoc
            # trong >= adaptive_open_clear_m) thi toc do = toc do nguoi + gain * (khoang cach - khoang cach mong
            # muon), toi da v_max_open; khoang cach mong muon dai them theo toc do nguoi (headway). Gan vat / qua
            # cua / di vong / mat camera: van v_max nhu cu. false = nhu cu (luon toi v_max roi dung).
            "adaptive_speed": True,
            "v_max_open": 0.40,          # <= max_linear cua driver (0.49 o max_percent 60)
            "follow_speed_gain": 0.8,    # 1/s: lech 0.25 m khoang cach -> them 0.2 m/s
            "follow_headway_s": 0.6,     # nguoi di 0.4 m/s -> giu xa them 0.24 m (toi da +0.4 m)
            "adaptive_open_clear_m": 1.0,
            # ... va dai rong nay (m, giua la xe) tu hong xe toi 2 m phia truoc KHONG co vat nao (tru quanh nguoi): chay
            # nhanh men tuong / hanh lang / canh do dac thi nguoi re vao cua ben hong la xe khong kip phan ung (mo phong
            # cua ben camera +-25 do: 6/10 -> 10/10 khi tat che do nhanh).
            "adaptive_open_width_m": 2.0,
            # Gioi han GIAM toc (m/s^2) — lon hon accel_lin de phanh kip o toc do cao. Cho hang nang: do lai (13.49).
            "decel_lin": 0.60,
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
            # 03/11: 1.20 -> 0.95. Khe rong hon 0.95 m thi du thoang ~12 cm moi ben, ban do + DWA tu qua duoc; dong
            # tac chui khe (canh truc, lui ve truc, bo 0.12 m/s) o khe 1.0-1.2 m giua do dac lam xe lui / dung vo ich
            # (bag 15/10: khe 1.17 m giua goc tu lanh va do ben phai -> lui 2 s ngay sau khi qua cua).
            "gap_waypoint_max_width_m": 0.95,
            # 03/11: tim khe = cach 14/10 (cap tia lidar ke nhau, khe phai gan duong ban do) TRUOC; khong thay thi tim MAT
            # CAT HEP tren DUONG DI ban do toi nguoi. Cap tia khong thay cua khi xe da o MIENG cua (hai thanh cua hai ben
            # than xe) -> truoc day xe tien lui mai o mieng cua (mo phong cua co ghe); chi tim tren duong thi khong nhan ra
            # cua khi nguoi RE vao (duong cat cua cheo, hai thanh cua lech nhau ~0.4 m doc duong) -> xe bo qua cua, mat
            # nguoi (mo phong cua bep). Hai cach noi tiep: 84/90 dat (rieng tren duong 72, rieng cap tia 79). false = chi
            # cap tia nhu 14/10.
            "gap_on_path": True,
            # Be rong toi da mat cat cho cach tim tren duong (<= gap_waypoint_max_width_m). Phat lai bag 15/10 (vong ho):
            # mat cat 0.85-0.95 m giua do dac lam xe canh truc o cho ban 14/10 cu bam nguoi; nhung mo phong vong kin ha
            # xuong 0.85 thi cua bi di cheo (nguoi re vao) kem han: 83 -> 73/90. Giu 0.95; ha neu thuc dia thay xe chui
            # khe o loi rong khong can.
            "gap_path_max_width_m": 0.95,
            # Cap tia (_gap_target) khong thay khe thi NHO khe da thay: "crossing" = chi luc dang chui (03/11), "legacy" =
            # nhu 14/10 (2 s, tu lam moi — giu dong tac chui khe ca khi da het khe), "off". gap_at_mouth = luat 14/10
            # "o mieng cua tin khe hon ban do". Hai tuy chon nay de quay lai cach 14/10 khi can so sanh ngoai thuc dia.
            "gap_recall_mode": "crossing",
            "gap_at_mouth": False,
            # BO NHO VAT CAN (khung odom): giu diem da thay toi da ngan nay giay khi vong quet hien tai KHONG thay
            # cho do (vung mu sau duoi, tia khong ve, bi che); tia hien tai di xuyen qua thi xoa ngay. 0 = tat.
            "obstacle_memory_sec": 12.0,
            "obstacle_memory_radius_m": 3.0,
            # Noi hai tia ke nhau gan hon muc nay (tuong nhin cheo, tia thua) khi lap BAN DO / tim khe: xe rong 0.60 m
            # khong chui qua khe hep hon 0.72 m nen noi lai la dung — khong noi thi ban do ve duong XUYEN tuong.
            "fill_gap_max_m": 0.65,
            "fill_in_dwa": True,            # diem noi tia ke nhau (< 1.2 m) dua ca vao kiem tra va cham DWA
            # Han muc lui (gap_back_max_m) HOAN DAN ngan nay m/s khi khong lui (truoc: nghi 1 s la cap lai du 0.40 m ->
            # lui - tien - lui lien tuc, tong cong ca met vao vung mu phia sau). Mo phong 03/11 (177 kich ban cua / goc
            # ket / lon xon): 0.025 -> 166, 0.05 -> 168, 0.10 -> 167 dat; 0.025 hong 1 ca cua sat khung nhieu 2 cm.
            "back_refill_mps": 0.05,
            # Chi LUI vao cho DA BIET LA TRONG: than xe da tung o do (30 s) hoac lidar da nhin thay trong (20 s).
            "reverse_known_free": True,
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
            # THOAT KET bang chuoi dong tac (05/11): xe vao tui hep / goc ma duong ban do quay nguoc ra, xoay tai cho khong
            # duoc -> tim chuoi xoay 15 do / tien cung 12 cm / lui cung 8 cm (chi vao cho da biet la trong) toi tu the
            # thang hang voi duong ban do. Cung chay khi xe dung trong o vuong 25 cm qua escape_stuck_sec ma nguoi con
            # xa. false = nhu ban 03/11.
            "escape_enabled": True,
            "escape_stuck_sec": 6.0,
            "escape_max_expansions": 1500,
            # 06/11: TONG thoi gian moi lan tim, CHIA NHIP — moi nhip dieu khien tim escape_slice_exp buoc roi nhuong,
            # xe dung yen (van gui lenh deu, khong cham watchdog driver). Truoc 06/11: 0.25 s mot mach -> may ban (YOLO,
            # mini PC) tim duoc it buoc hon han. Gioi han chinh la escape_max_expansions.
            "escape_max_sec": 3.0,
            "escape_slice_exp": 150,
            # 06/11 — CHO HEP. Bay SAU (tui chu U, hanh lang cut): ban do thuong chi nhin 1 m sau xe -> loi ra xa hon thi
            # bao "dung o cho tot nhat" mai. Nguoi con xa ma ban do thuong bao dung cho -> lap lai voi luoi nhin toi
            # escape_far_back_m sau xe; thoat ket khi do tim trong cua so lon hon va lui duoc vao VET than xe da di qua
            # trong escape_trail_sec (lui 1.5-2 m o 0.06 m/s mat ~30 s, vet 30 s thuong het han giua chung).
            "escape_far_back_m": 2.6,
            "escape_trail_sec": 90.0,
            # "Duoi xe vuong — nhich toi de xoay ve phia nguoi": truoc 06/11 khong gioi han -> di doc tuong ma nguoi dung
            # gan tuong ben kia thi xe bo toi cach nguoi 0.43 m (mo phong goc tuong). Nay: khong nhich khi da cach nguoi
            # duoi creep_min_dist_m, moi dot toi da creep_max_m (du cho duoi 0.33 m ra khoi khung cua); het cach thi
            # nguoi lech hon face_escape_deg -> tim chuoi dong tac (lui / xoay / tien ngan) de QUAY MAT ve nguoi.
            "creep_min_dist_m": 0.80,
            "creep_max_m": 0.45,
            "face_escape_deg": 20.0,
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
            # 05/11: pha di toi duoc gia han theo do dai duong ban do / luc dang thoat ket, toi da search_goto_cap_sec.
            # 06/11: 30 -> 75 s — lui ra khoi hanh lang cut 2 m o 0.06 m/s da mat ~35 s, roi con vong qua vach
            "search_goto_cap_sec": 75.0,
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
        self.adaptive_speed = bool(g("adaptive_speed"))
        self.v_max_open = max(self.v_max, float(g("v_max_open")))
        self.follow_gain = float(g("follow_speed_gain"))
        self.follow_headway = float(g("follow_headway_s"))
        self.open_clear = float(g("adaptive_open_clear_m"))
        self.open_width = float(g("adaptive_open_width_m"))
        self.decel_lin = max(float(g("decel_lin")), float(g("accel_lin")))
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
        self.gap_on_path = bool(g("gap_on_path"))
        self.gap_path_max_w = float(g("gap_path_max_width_m"))
        self.gap_recall_mode = str(g("gap_recall_mode"))
        self.gap_at_mouth = bool(g("gap_at_mouth"))
        self.mem_sec = float(g("obstacle_memory_sec"))
        self.mem_radius = float(g("obstacle_memory_radius_m"))
        self.mem_cell = 0.05
        self.fill_max = float(g("fill_gap_max_m"))
        self.fill_in_dwa = bool(g("fill_in_dwa"))
        self.back_refill = float(g("back_refill_mps"))
        self.rev_known_free = bool(g("reverse_known_free"))
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
        self.escape_on = bool(g("escape_enabled"))
        self.escape_stuck = float(g("escape_stuck_sec"))
        self.escape_max_exp = max(100, int(g("escape_max_expansions")))
        self.escape_max_sec = max(0.02, float(g("escape_max_sec")))
        self.escape_slice_exp = int(np.clip(int(g("escape_slice_exp")), 16, 2000))
        self.esc_far_back = max(1.0, float(g("escape_far_back_m")))
        self.esc_trail_sec = max(30.0, float(g("escape_trail_sec")))
        self.creep_min_d = float(g("creep_min_dist_m"))
        self.creep_max = max(0.0, float(g("creep_max_m")))
        self.face_esc = math.radians(float(g("face_escape_deg")))
        self.nav_cache: Optional[Dict[str, Any]] = None
        self.nav_gap_cache: Optional[Dict[str, Any]] = None
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
        self.search_goto_cap = max(self.search_goto_max, float(g("search_goto_cap_sec")))
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
            return
        t = self.target
        src = str(t.get("source", ""))
        if t.get("valid") and t.get("odom_x") is not None and src.startswith(("camera", "lidar")):
            now = self.target_time
            if not self.tgt_trail or now - self.tgt_trail[-1][0] > 0.2:
                self.tgt_trail.append((now, float(t["odom_x"]), float(t["odom_y"])))

    def _scan_cb(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        pts, bear, rng, ldeg = scan_to_base_points(
            ranges,
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
            pts, bear, rng, ldeg = pts[keep], bear[keep], rng[keep], ldeg[keep]
        self.scan_pts = pts
        obs = downsample_polar(pts, bear, rng, self.bucket_deg, keep_all_within_m=1.0)
        if obs.shape[0] > self.max_obs_pts:
            d = np.hypot(obs[:, 0], obs[:, 1])
            keep = np.argsort(d)[: self.max_obs_pts]
            obs = obs[keep]
        now = time.time()
        scan = (ranges, float(msg.angle_min), float(msg.angle_increment),
                max(float(msg.range_min), 0.02), min(float(msg.range_max), self.lidar_max_use_range))
        mem = self._mem_update(pts, scan, now) if (self.mem_sec > 0.0 and self.odom_ok) else np.zeros((0, 2))
        fill = self._fill_points(pts, ldeg)
        far_mem = np.zeros((0, 2))
        # DWA / quet footprint: + diem noi hai tia ke nhau trong 1.2 m — lidar nam gan nhu TRUNG mat tuong (xe dung o
        # mieng cua) thi doan tuong sat goc khung khong co tia nao cham, goc truoc xe lua vao do (mo phong 03/11, xe sat
        # khung cua luc bat may: cham tuong). Diem noi nam tren doan thang giua hai tia that.
        fill_near = fill[np.hypot(fill[:, 0], fill[:, 1]) < 1.2] if fill.shape[0] else fill
        if fill_near.shape[0] and self.fill_in_dwa:
            obs = np.vstack([obs, fill_near])
        near = np.zeros((0, 2))
        if mem.shape[0]:
            # diem nho trong 2 m (toi da 150 diem gan nhat); xa hon chi dung cho ban do
            dm = np.hypot(mem[:, 0], mem[:, 1])
            near = mem[dm < 2.0]
            far_mem = mem[dm >= 2.0]
            if near.shape[0] > 150:
                near = near[np.argsort(np.hypot(near[:, 0], near[:, 1]))[:150]]
            self.obstacles = np.vstack([obs, near]) if near.shape[0] else obs
        else:
            self.obstacles = obs
        # Tap DAY DU (khong cat max_obstacle_points) cho DO KHE (_gap_refine) va "phia truoc con trong" (toc do thich
        # ung). Sat tuong thi diem gan (< 1 m) da qua 160 -> obstacles chi con diem trong ~0.7 m: thanh cua ben kia mat
        # khoi tap -> lam lai khe thay "mot ben trong" -> tra None (mo phong cua bep 03/11: truc khe nhay 87 <-> 114 do,
        # xe tien lui o mieng cua). DWA van dung tap da cat nhu cu (tai CPU khong doi).
        sp = self.scan_pts
        sp = sp[np.hypot(sp[:, 0], sp[:, 1]) < 3.0] if sp.shape[0] else sp
        parts_f = [a for a in (sp, fill_near if self.fill_in_dwa else np.zeros((0, 2)), near) if a.shape[0]]
        self.obstacles_full = np.vstack(parts_f) if parts_f else self.obstacles
        fill_far = fill[np.hypot(fill[:, 0], fill[:, 1]) >= 1.2] if fill.shape[0] else fill
        parts = [a for a in (self.obstacles, fill_far, far_mem) if a.shape[0]]
        # TIM KHE / mat cat loi di: bo nho da TRUNG BINH (it nhieu) neu du diem; luc moi bat (bo nho trong) thi vong
        # quet + noi tia + diem nho vung mu.
        mall = getattr(self, "mem_all_base", np.zeros((0, 2)))
        if mall.shape[0] >= 30:
            self.obstacles_gap = np.vstack([mall, mem]) if mem.shape[0] else mall
        else:
            self.obstacles_gap = np.vstack(parts) if parts else self.obstacles
        # BAN DO: them ca diem nho dang bi vat gan hon che (tuong sau do dac, mieng cua)
        occl = getattr(self, "mem_occl", np.zeros((0, 2)))
        self.obstacles_dense = np.vstack([self.obstacles_gap, occl]) if occl.shape[0] else self.obstacles_gap
        if self.odom_ok and (not self.scan_hist or now - self.scan_hist[-1][0] >= 0.5):
            self.scan_hist.append((now, self.robot_x, self.robot_y, self.robot_yaw) + scan)
        self.scan_time = now

    def _odom_cb(self, msg: Odometry) -> None:
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.robot_yaw = yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        self.odom_ok = True
        now = time.time()
        tr = self.pose_trail
        if (not tr or now - tr[-1][0] > 0.25 or math.hypot(self.robot_x - tr[-1][1], self.robot_y - tr[-1][2]) > 0.05
                or abs(wrap_pi(self.robot_yaw - tr[-1][3])) > 0.15):
            tr.append((now, self.robot_x, self.robot_y, self.robot_yaw))

    # ─────────────────────────────────────────────────────────────────────
    # Bo nho vat can + cho da biet la trong (03/11)
    # ─────────────────────────────────────────────────────────────────────
    def _beam_lookup(self, bx: np.ndarray, by: np.ndarray, scan,
                     long_free: bool = False) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Diem (khung base, tai tu the cua vong quet) -> (khoang cach tu lidar, khoang cach tia o huong do,
        tia hop le & khong nam trong cung mu).

        long_free: tia KHONG VE trong tam dung (inf / xa hon tam) coi la trong toi het tam — chi dung cho "da biet la
        trong" khi lui (phong rong, sau lung xe khong co gi trong 5 m thi tia khong ve; truoc day vi vay xe dung im 40 s
        canh thung vi khong dam lui, mo phong 03/11). KHONG dung de xoa bo nho: tia song song mat tuong o mieng cua
        cung khong ve."""
        ranges, a0, ainc, rmin, rmax = scan
        n = ranges.shape[0]
        lx, ly = bx - self.lidar_x, by - self.lidar_y
        rm = np.hypot(lx, ly)
        if n == 0 or abs(ainc) < 1e-9:
            return rm, np.full(rm.shape, np.nan), np.zeros(rm.shape, bool)
        a_las = (np.arctan2(ly, lx) - self.lidar_yaw_offset_rad) * self.lidar_angle_sign
        idx = np.mod(np.round((a_las - a0) / ainc).astype(np.int64), n)
        rb = ranges[idx]
        ok = np.isfinite(rb) & (rb > rmin) & (rb < rmax)
        if long_free:
            far = ~np.isnan(rb) & (rb >= rmax) & (rm < rmax - 0.1)
            rb = np.where(far, rmax, rb)
            ok = ok | far
        if self.blind_sectors:
            deg = np.degrees(a0 + idx * ainc) % 360.0
            for lo, hi in self.blind_sectors:
                ok &= ~((deg >= lo) & (deg <= hi))
        return rm, rb, ok

    def _mem_update(self, pts: np.ndarray, scan, now: float) -> np.ndarray:
        """Cap nhat bo nho vat can; tra ve cac diem NHO (khung base) ma vong quet nay KHONG thay.

        Giu: diem da thay trong obstacle_memory_sec giay, con trong ban kinh, khong nam trong than xe, khong gan
        cho nguoi dang bam vua dung (chan nguoi di chuyen). Xoa ngay: tia hien tai di XUYEN qua cho do (cho do gio
        trong). Diem dang thay o vong nay thi lay tu vong quet (khong lap lai)."""
        c, s_ = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        rx, ry = self.robot_x, self.robot_y
        cell = self.mem_cell
        M, T = self.mem_xy, self.mem_t
        rem = np.zeros((0, 2))
        self.mem_occl = np.zeros((0, 2))
        if M.shape[0]:
            dx, dy = M[:, 0] - rx, M[:, 1] - ry
            bx, by = c * dx + s_ * dy, -s_ * dx + c * dy
            keep = ((now - T) < self.mem_sec) & (np.hypot(bx, by) < self.mem_radius + 0.5)
            m_ = self.self_margin
            keep &= ~((bx < self.front_len - m_) & (bx > -self.rear_len + m_) & (np.abs(by) < self.half_width - m_))
            rm, rb, ok = self._beam_lookup(bx, by, scan)
            through = ok & (rb > rm + 0.10)                    # tia di xuyen qua -> cho do da trong
            # Diem GAN xe (< 0.6 m tu lidar): chi xoa khi MOI tia trong +-2 do deu di qua. Lidar nam gan trung mat tuong
            # (xe o mieng cua) thi tia luot sat dau tuong bay xa -> xoa mat goc khung cua da nho -> DWA be lai cham khung
            # (mo phong 03/11 xe sat khung cua luc bat may).
            near = through & (rm < 0.6)
            if np.any(near):
                ranges_, a0_, ainc_, rmin_, rmax_ = scan
                n_ = ranges_.shape[0]
                lxn, lyn = bx[near] - self.lidar_x, by[near] - self.lidar_y
                a_las = (np.arctan2(lyn, lxn) - self.lidar_yaw_offset_rad) * self.lidar_angle_sign
                i0 = np.round((a_las - a0_) / ainc_).astype(np.int64)
                k_ = max(1, int(round(math.radians(2.0) / max(1e-6, abs(ainc_)))))
                allp = np.ones(i0.shape[0], bool)
                for dk in range(-k_, k_ + 1):
                    rr = ranges_[np.mod(i0 + dk, n_)]
                    allp &= np.isfinite(rr) & (rr > rm[near] + 0.10)
                through[np.where(near)[0][~allp]] = False
            keep &= ~through
            # Hai loai diem nho dung lai: (1) cho vong quet nay KHONG co tia ve (vung mu sau duoi, tia song song mat
            # tuong khong ve) -> cho DWA / quet footprint / lui; (2) bi vat GAN HON che -> chi cho BAN DO (biet tuong
            # sau do dac). Cho tia dang thay vat thi vong quet hien tai la du — them diem nho cu (nhieu lidar tich lai)
            # cho DWA lam long cua tuong day "hep" di vai cm, cua 0.81 m thanh 0.71 m, xe dung giua cua (mo phong).
            seen = ok & (rb >= rm - 0.10)
            occl = ok & (rb < rm - 0.10)
            if self.tgt_trail:
                tr = np.array([(x, y) for (t, x, y) in self.tgt_trail if now - t < self.mem_sec])
                if tr.shape[0]:
                    dmin = np.min(np.hypot(M[:, None, 0] - tr[None, :, 0], M[:, None, 1] - tr[None, :, 1]), axis=1)
                    keep &= dmin > 0.45
            rem_mask = keep & ~seen & ~occl
            rem = np.stack([bx[rem_mask], by[rem_mask]], axis=1)
            om = keep & occl
            self.mem_occl = np.stack([bx[om], by[om]], axis=1)
            M, T, N = M[keep], T[keep], self.mem_n[keep]
        else:
            N = np.zeros(0)
        if pts.shape[0]:
            P = pts[np.hypot(pts[:, 0], pts[:, 1]) < self.mem_radius]
            ox, oy = rx + c * P[:, 0] - s_ * P[:, 1], ry + s_ * P[:, 0] + c * P[:, 1]
            new = np.stack([ox, oy], axis=1)
            M = np.vstack([M, new]) if M.shape[0] else new
            T = np.concatenate([T, np.full(new.shape[0], now)])
            N = np.concatenate([N, np.ones(new.shape[0])])
        if M.shape[0]:
            # Moi o luoi 5 cm giu TRUNG BINH cac lan cham (khong phai mot diem nhieu): nhieu lidar 2 cm tich lai thi
            # tuong o mieng cua "day" them vai cm, cua hep thanh khong lot (mo phong 03/11). Dem toi da 20 lan de van
            # theo kip khi vat xe dich cham.
            key = np.round(M / cell).astype(np.int64)
            k1 = key[:, 0] * 1000003 + key[:, 1]
            _u, inv = np.unique(k1, return_inverse=True)
            w_ = np.minimum(N, 20.0)
            sw = np.bincount(inv, weights=w_)
            M = np.stack([np.bincount(inv, weights=w_ * M[:, 0]) / sw, np.bincount(inv, weights=w_ * M[:, 1]) / sw],
                         axis=1)
            Tm = np.full(_u.shape[0], -np.inf)
            np.maximum.at(Tm, inv, T)
            T, N = Tm, np.minimum(sw, 20.0)
        self.mem_xy, self.mem_t, self.mem_n = M, T, N
        # Ban sao khung base cua TOAN BO bo nho (diem da TRUNG BINH nhieu lan cham, nhieu ~0.5 cm) — dung de DO KHE:
        # lay diem gan nhat trong ~15 tia nhieu 2 cm thi mep khe bi keo vao 3-4 cm moi ben, cua 0.81 m do ra 0.72 m ->
        # dung sai canh truc con 2 cm -> lui - canh mai o mieng cua (mo phong 03/11).
        if M.shape[0]:
            dx, dy = M[:, 0] - rx, M[:, 1] - ry
            bx, by = c * dx + s_ * dy, -s_ * dx + c * dy
            sel = (np.hypot(bx, by) < self.mem_radius) & (N >= 2.0)
            m_ = self.self_margin
            sel &= ~((bx < self.front_len - m_) & (bx > -self.rear_len + m_) & (np.abs(by) < self.half_width - m_))
            self.mem_all_base = np.stack([bx[sel], by[sel]], axis=1)
        else:
            self.mem_all_base = np.zeros((0, 2))
        return rem

    def _fill_points(self, pts: np.ndarray, ldeg: np.ndarray) -> np.ndarray:
        """Noi hai tia KE NHAU (lech <= 4 do) cach nhau < fill_gap_max_m bang cac diem cach 0.06 m.

        Tuong nhin cheo / o xa thi hai tia ke nhau cach nhau vai chuc cm: ban do (o cam 0.30 m quanh tung diem) thay
        khe giua chung va ve duong xuyen tuong (bag 14/10 o mieng cua bep). Xe rong 0.60 m khong qua duoc khe hep hon
        0.72 m nen noi lai khong mat duong nao that."""
        if self.fill_max <= 0.0 or pts.shape[0] < 2:
            return np.zeros((0, 2))
        A, B = pts[:-1], pts[1:]
        dd = np.hypot(B[:, 0] - A[:, 0], B[:, 1] - A[:, 1])
        dg = np.abs(((ldeg[1:] - ldeg[:-1]) + 180.0) % 360.0 - 180.0)
        sel = (dd > 0.09) & (dd < self.fill_max) & (dg <= 4.0) & (np.hypot(A[:, 0], A[:, 1]) < 4.0)
        out = []
        for a, b, L in zip(A[sel], B[sel], dd[sel]):
            k = int(L / 0.06)
            if k >= 1:
                u = (np.arange(1, k + 1) / (k + 1))[:, None]
                out.append(a + u * (b - a))
        return np.vstack(out) if out else np.zeros((0, 2))

    def _known_free(self, Q: np.ndarray, trail_sec: float = 30.0) -> np.ndarray:
        """Diem Q (khung odom) da biet la TRONG: than xe tung o do (trail_sec, thuong 30 s; thoat ket dung
        escape_trail_sec), hoac lidar da nhin xuyen qua (20 s)."""
        now = time.time()
        ok = np.zeros(Q.shape[0], bool)
        m_ = 0.02
        for (t, x, y, yaw) in self.pose_trail:
            if now - t > trail_sec:
                continue
            c, s_ = math.cos(yaw), math.sin(yaw)
            dx, dy = Q[:, 0] - x, Q[:, 1] - y
            lx, ly = c * dx + s_ * dy, -s_ * dx + c * dy
            ok |= (lx < self.front_len - m_) & (lx > -self.rear_len + m_) & (np.abs(ly) < self.half_width - m_)
            if ok.all():
                return ok
        for h in self.scan_hist:
            if now - h[0] > 20.0:
                continue
            c, s_ = math.cos(h[3]), math.sin(h[3])
            dx, dy = Q[:, 0] - h[1], Q[:, 1] - h[2]
            bx, by = c * dx + s_ * dy, -s_ * dx + c * dy
            rm, rb, okb = self._beam_lookup(bx, by, h[4:], long_free=True)
            ok |= okb & (rb > rm + 0.08)
            if ok.all():
                return ok
        return ok

    def _reverse_ok_many(self, V: np.ndarray, W: np.ndarray) -> np.ndarray:
        """Nhu _reverse_ok cho nhieu lenh mot luc (DWA cho phep lui)."""
        out = np.ones(V.shape[0], bool)
        if V.shape[0] == 0 or not self.rev_known_free:
            return out
        T = self.horizon_T
        P = []
        for v, w in zip(V, W):
            for t in (0.5 * T, T):
                th = w * t
                if abs(w) < 1e-4:
                    X, Y = v * t, 0.0
                else:
                    X, Y = (v / w) * math.sin(th), (v / w) * (1.0 - math.cos(th))
                c, s_ = math.cos(th), math.sin(th)
                for (ex, ey) in ((-self.rear_len, self._rev_hw), (-self.rear_len, 0.0),
                                 (-self.rear_len, -self._rev_hw)):
                    P.append((X + c * ex - s_ * ey, Y + s_ * ex + c * ey))
        P = np.array(P)
        inside = (P[:, 0] > -self.rear_len) & (np.abs(P[:, 1]) < self.half_width)
        c, s_ = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        Q = np.stack([self.robot_x + c * P[:, 0] - s_ * P[:, 1], self.robot_y + s_ * P[:, 0] + c * P[:, 1]], axis=1)
        good = (self._known_free(Q) | inside).reshape(V.shape[0], -1)
        return np.all(good, axis=1)

    @property
    def _rev_hw(self) -> float:
        """Nua be ngang phan duoi phai 'da biet la trong' khi lui (06/11). Cung MU cua lidar (+-24 do quanh thang sau, nhin
        tu lidar o x = +0.10) o mep duoi (x = -0.33) chi rong |y| <= 0.19 m; hai GOC duoi (0.30 m) nhin tu lidar o ~145 do —
        NGOAI cung mu, lidar van thay nen kiem tra va cham (_arc_clearance / truong khoang cach) da lo. Truoc 06/11 doi ca
        hai goc ngoai trong vet than xe (vet chi tinh |y| < 0.28 m) -> lui THANG theo dung vet xe vua vao chi duoc khi lidar
        con nho da 'nhin xuyen' cho goc (20 s) -> lui ra khoi tui / hanh lang cut dut giua chung, ke hoach thoat ket bi huy
        lien tuc (mo phong tui U 0.90 m). Nay: |y| <= half_width - 0.06 (= 0.24 m, phu tron cung mu)."""
        return self.half_width - 0.06

    def _reverse_ok(self, v: float, w: float, T: Optional[float] = None, trail_sec: float = 30.0) -> bool:
        """Lenh LUI (v < 0) chi duoc chay neu cung sau xe quet qua cho DA BIET LA TRONG (lidar mu thang phia sau)."""
        if v >= 0.0 or not self.rev_known_free:
            return True
        T = self.horizon_T if T is None else T
        pts = []
        for t in (0.5 * T, T):
            th = w * t
            if abs(w) < 1e-4:
                X, Y = v * t, 0.0
            else:
                X, Y = (v / w) * math.sin(th), (v / w) * (1.0 - math.cos(th))
            c, s_ = math.cos(th), math.sin(th)
            for (ex, ey) in ((-self.rear_len, self._rev_hw), (-self.rear_len, 0.0), (-self.rear_len, -self._rev_hw)):
                pts.append((X + c * ex - s_ * ey, Y + s_ * ex + c * ey))
        P = np.array(pts)
        # diem nam trong than xe hien tai thi hien nhien trong
        inside = (P[:, 0] > -self.rear_len) & (np.abs(P[:, 1]) < self.half_width)
        if inside.all():
            return True
        c, s_ = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        Q = np.stack([self.robot_x + c * P[:, 0] - s_ * P[:, 1], self.robot_y + s_ * P[:, 0] + c * P[:, 1]], axis=1)
        return bool(np.all(self._known_free(Q, trail_sec) | inside))

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
                    # uu tien huong ve cho thay nguoi lan cuoi (nhich ra thi cung lai gan nguoi hon)
                    gb0 = 0.0
                    if self.last_target_odom is not None:
                        lbx, lby = self._base_of(*self.last_target_odom)
                        gb0 = float(np.clip(math.atan2(lby, lbx), -math.radians(90.0), math.radians(90.0)))
                    phi, reach, _ = self._choose_heading(gb0, 1.0, gb0)
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
        if self.open_tries < 2:
            # 03/11: khong xoay duoc (duoi xe con trong khung cua, sat tu...) -> NHICH RA cho thoang roi quet tiep,
            # truoc day bo cuoc ngay (mo phong cua co tu lanh: xe dung trong cua, nguoi khuat sau tu lanh)
            self.open_tries += 1
            self.rssi_goal_odom = None
            self._search_next(now, "open")
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
        self.esc = None
        self.stuck_reset_t = time.time()
        res.success = True
        res.message = "Bam nguoi: BAT"
        self.get_logger().info(res.message)
        return res

    def _disable_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.enabled = False
        self.esc = None
        self._hard_stop()
        res.success = True
        res.message = "Bam nguoi: TAT"
        self.get_logger().info(res.message)
        return res

    def _stop_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.enabled = False
        self.esc = None
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

    def _emit(self, v: float, w: float, dt: float, v_top: Optional[float] = None) -> Tuple[float, float]:
        """Gioi han gia toc + bu deadband driver roi publish.

        03/11: GIAM toc theo decel_lin (> accel_lin) — o toc do thich ung 0.4 m/s phai phanh kip; v_top = tran toc do
        cua nhip nay (che do thoang), mac dinh v_max."""
        dv = self.accel_lin * dt
        dd = self.decel_lin * dt
        dw = self.accel_ang * dt
        if abs(self.cur_v) > self.v_max + 0.01 and abs(v) < abs(self.cur_v) and v * self.cur_v >= 0.0:
            # dang o toc do thich ung (> v_max) va giam toc: phanh theo decel_lin. Duoi v_max giu nhu cu.
            v = float(np.clip(v, self.cur_v - dd, self.cur_v + dd))
        else:
            v = float(np.clip(v, self.cur_v - dv, self.cur_v + dv))
        w = float(np.clip(w, self.cur_w - dw, self.cur_w + dw))
        v = float(np.clip(v, self.v_min, self.v_max if v_top is None else max(self.v_max, v_top)))
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

    def _person_radial_speed(self, tgt: Dict[str, Any], bearing: float) -> float:
        """Toc do nguoi theo huong xe -> nguoi (m/s, duong = di ra xa), tu vx, vy (khung odom) cua tracker."""
        try:
            vx, vy = float(tgt.get("vx") or 0.0), float(tgt.get("vy") or 0.0)
        except (TypeError, ValueError):
            return 0.0
        a = self.robot_yaw + bearing
        return vx * math.cos(a) + vy * math.sin(a)

    def _person_leaving(self, tgt: Dict[str, Any], bearing: float) -> bool:
        """Nguoi dang di RA XA (> 0.12 m/s) va camera thay ro -> dung lai o diem 1 m roi chay tiep la giat (dung - di)."""
        return (self.adaptive_speed and str(tgt.get("source", "")).startswith("camera")
                and self._person_radial_speed(tgt, bearing) > 0.12)

    def _free_ahead(self, max_d: float = 2.5) -> float:
        """Quang duong THANG phia truoc con trong (hanh lang half_width + margin_soft) — tap day du, khong cat."""
        P = self.obstacles_full if self.obstacles_full.shape[0] else self.obstacles
        if P.shape[0] == 0:
            return max_d
        hw = self.half_width + self.margin_soft
        sel = (P[:, 0] > self.front_len) & (np.abs(P[:, 1]) < hw)
        if not np.any(sel):
            return max_d
        return float(min(max_d, np.min(P[sel, 0]) - self.front_len))

    def _adaptive_speed(self, tgt: Dict[str, Any], dist: float, bearing: float, source: str,
                        blocked: bool) -> Tuple[Optional[float], Optional[float]]:
        """(v_top, v_ref) cho nhip nay — (None, None) = nhu cu (v_max, DWA tu chon).

        Chi khi: dang FOLLOW, camera thay nguoi, duong thang toi dich VA toi tan nguoi khong bi chan (nguoi re vao cua
        thi khung cua chan duong toi nguoi -> ve v_max nhu cu: chay nhanh toi gan cua som qua, lech truc, mo phong
        cua_ben 03/11), nguoi lech < 26 do, phia truoc thoang >= adaptive_open_clear_m. v_ref = toc do nguoi (huong ra xa) + gain * (d - d_mong_muon), d_mong_muon =
        follow_distance + headway * toc do nguoi (toi da +0.4 m). Tran: v_max_open va toc do con phanh kip truoc vat
        gan nhat phia truoc (decel_lin, chua 0.30 m)."""
        if not self.adaptive_speed or self.state != S_FOLLOW or blocked or not source.startswith("camera"):
            return None, None
        if abs(bearing) > math.radians(26.0):
            return None, None
        free = self._free_ahead()
        if free < self.open_clear:
            return None, None
        # cho THOANG that: dai rong open_width tu hong xe toi 2 m phia truoc khong co vat (tru chan nguoi)
        P = self.obstacles_full if self.obstacles_full.shape[0] else self.obstacles
        if P.shape[0]:
            px, py = dist * math.cos(bearing), dist * math.sin(bearing)
            band = (P[:, 0] > -self.rear_len) & (P[:, 0] < 2.0) & (np.abs(P[:, 1]) < 0.5 * self.open_width)
            band &= np.hypot(P[:, 0] - px, P[:, 1] - py) > self.target_clear_r
            if np.any(band):
                return None, None
        v_p = max(0.0, self._person_radial_speed(tgt, bearing))
        d_des = self.follow_distance + min(0.4, self.follow_headway * v_p)
        v_cap = min(self.v_max_open, math.sqrt(2.0 * self.decel_lin * max(0.0, free - 0.30)))
        v_ref = float(np.clip(v_p + self.follow_gain * (dist - d_des), 0.0, v_cap))
        return max(self.v_max, v_cap), v_ref

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
            return self._gap_recall(target_xy)

        A, B, mx, my = best
        self.gap_seen_t = time.time()
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
        g = {"mx": mx, "my": my, "nx": nx, "ny": ny, "gw": float(math.hypot(ux * un, uy * un)),
             "psi": psi, "along": along, "lat": lat}
        g2 = self._gap_refine(g, target_xy)
        if g2 is not None and g2.get("corridor"):
            return None
        return self._gap_store(g2 or g)

    def _gap_recall(self, target_xy: Optional[Tuple[float, float]],
                    hold_sec: Optional[float] = None) -> Optional[Dict[str, float]]:
        """Khong thay khe o nhip nay MA DANG CHUI DO: dung lai khe da thay (khung odom) cho toi khi duoi xe qua han.

        Xe toi MIENG cua thi hai thanh cua nam hai ben than xe, khong con ke nhau theo goc -> _gap_target khong ra
        khe nao giua luc dang chui (14/10). 03/11: CHI khi nhip truoc dang o pha qua khe, va khe da THAY TRUC TIEP
        trong 6 s qua (ban 14/10 nho khe 2 s va TU LAM MOI moi lan dung lai -> giu dong tac chui khe hang giay sau
        khi het khe, ca o khoang 1.17 m giua do dac: 97 % so nhip chui khe moi tang trong bag 15/10)."""
        m = self.gap_mem
        now_ = time.time()
        if m is None or self.gap_recall_mode == "off":
            return None
        crossing = now_ - self.gap_cross_time < 0.6
        if hold_sec is not None:
            # (tim khe tren duong di) vua thay < hold_sec, hoac dang chui va da thay trong 6 s
            if not (now_ - self.gap_seen_t < hold_sec or (crossing and now_ - self.gap_seen_t < 6.0)):
                return None
        elif self.gap_recall_mode == "legacy":
            if now_ - m["t"] > 2.0:
                return None
        elif not crossing or now_ - self.gap_seen_t > 6.0:
            return None
        mx, my = self._base_of(m["ox"], m["oy"])
        th = wrap_pi(m["axis"] - self.robot_yaw)
        dx, dy = math.cos(th), math.sin(th)
        along = dx * mx + dy * my
        if along < -(self.rear_len + 0.10) or along > self.gap_engage_range:
            return None
        if target_xy is not None and dx * (target_xy[0] - mx) + dy * (target_xy[1] - my) <= 0.0:
            return None
        g = {"mx": mx, "my": my, "nx": -dx, "ny": -dy, "gw": m["gw"], "psi": th, "along": along,
             "lat": abs(-dy * mx + dx * my)}
        g2 = self._gap_refine(g, target_xy)
        if g2 is not None and g2.get("corridor"):
            return None
        if g2 is None:
            if hold_sec is None or now_ - self.gap_seen_t >= hold_sec:
                return None
            # giu ngan (< 1 s tu lan thay that): lam lai khe khong ra (than xe nam giua hai thanh cua) -> dung khe da loc
            return g
        return self._gap_store(g2)

    def _gap_store(self, g: Dict[str, float]) -> Dict[str, float]:
        """Nho khe (khung odom) va LOC theo thoi gian: cua dung yen, con so do moi nhip nhay vai cm vi nhieu
        lidar (goc thanh cua luc bat duoc luc khong) -> 'along' nhay +-6 cm qua nguong lui / tien -> xe lui
        tien lien tuc truoc cua (mo phong cua bep 14/10). Cung mot khe (lech < 25 cm, < 15 do) thi trung binh
        truot; khe khac thi lay luon. Tra ve khe da loc, tinh lai theo tu the xe hien tai."""
        ox, oy = self._odom_of(g["mx"], g["my"])
        ax = wrap_pi(g["psi"] + self.robot_yaw)
        gw = g["gw"]
        m = self.gap_mem
        now_ = time.time()
        frozen = False
        if m is not None and now_ - m["t"] < 1.0 and now_ - self.gap_cross_time < 0.6:
            # 03/11: DANG CHUI -> giu TRUC va DO SAU cua khe toi khi duoi xe ra han; chi nhan phep do cua CUNG mat cat
            # (tam lech < 10 cm ca doc truc lan doc mat khe) de sua vi tri ngang va be rong. Truoc day moi nhip lam lai
            # khe ra mat cat SAU hon mot chut (ngay sau cua: giua tu lanh va do ben phai 0.84-0.95 m), van "cung khe" nen
            # tron dan vao: tam troi 0.3 m, truc 94 -> 72 do, xe dang qua cua dung xoay "vuong mat cua" theo truc moi
            # (bag 15/10 103136 luc 314-320 s — tien lui sau cua co tu lanh). Khong dong bang han: uoc luong dau co the
            # sai (mo phong xe dat san sat cua: 0.91 m cho cua 0.81 m) -> phai sua duoc be rong.
            mbx, mby = self._base_of(m["ox"], m["oy"])
            th_m = wrap_pi(m["axis"] - self.robot_yaw)
            if math.cos(th_m) * mbx + math.sin(th_m) * mby > -(self.rear_len + 0.10):
                frozen = True
                ca, sa = math.cos(m["axis"]), math.sin(m["axis"])
                d_dep = (ox - m["ox"]) * ca + (oy - m["oy"]) * sa
                d_lat = -(ox - m["ox"]) * sa + (oy - m["oy"]) * ca
                a = 0.25 if (abs(d_dep) <= 0.10 and abs(d_lat) <= 0.10) else 0.0
                ox, oy = m["ox"] - a * d_lat * sa, m["oy"] + a * d_lat * ca
                ax = m["axis"]
                gw = m["gw"] + a * (gw - m["gw"])
        if (not frozen and m is not None and now_ - m["t"] < 1.0 and math.hypot(ox - m["ox"], oy - m["oy"]) < 0.25
                and abs(wrap_pi(ax - m["axis"])) < math.radians(15.0)):
            a = 0.25
            ox, oy = m["ox"] + a * (ox - m["ox"]), m["oy"] + a * (oy - m["oy"])
            ax = wrap_pi(m["axis"] + a * wrap_pi(ax - m["axis"]))
            gw = m["gw"] + a * (gw - m["gw"])
        self.gap_mem = {"ox": ox, "oy": oy, "axis": ax, "gw": gw, "t": now_}
        mx, my = self._base_of(ox, oy)
        th = wrap_pi(ax - self.robot_yaw)
        dx, dy = math.cos(th), math.sin(th)
        return {"mx": mx, "my": my, "nx": -dx, "ny": -dy, "gw": gw, "psi": th,
                "along": dx * mx + dy * my, "lat": abs(-dy * mx + dx * my)}

    def _gap_on_path(self, nav: Optional[Dict[str, Any]],
                     target_xy: Optional[Tuple[float, float]]) -> Optional[Dict[str, float]]:
        """KHE HEP NAM TREN DUONG DI cua ban do (03/11) — thay cho cap diem lidar ke nhau theo goc (_gap_target).

        Vi sao: _gap_target lay hai diem ke nhau theo goc nhin tu xe, cach nhau 0.74-1.2 m. Diem ke nhau theo goc
        co the o hai vat khac nhau: goc khung cua PHAI <-> goc tu lanh sau tuong ben TRAI cho ra "khe" cheo 1.05 m
        lech truc 30-37 do, xen ke voi cua that -> xe canh theo truc sai roi tien lui (mo phong + bag 15/10 cua co
        tu lanh). Ban 14/10 con NHO khe 2 s va tu lam moi -> giu dong tac chui khe hang giay sau khi het khe, ca o
        khoang 1.17 m giua do dac (97 % so nhip chui khe moi tang trong bag 15/10).

        Cach lam: di doc duong ban do (toi gap_engage_range_m), o moi diem do be rong MAT CAT vuong goc voi duong
        (diem vat gan nhat ben trai + ben phai trong dai +-0.2 m doc duong). Mat cat hep hon gap_waypoint_max_width_m
        dau tien la khe can chui; lay cho hep nhat cua doan hep do (co hong). Truc khe = vuong goc doan noi hai mep,
        chieu theo huong duong di. O vi tri xe xet ca doan than xe (duoi con trong khe -> van giu dong tac qua khe
        cho toi khi duoi ra han, khong can nho khe). Khe nao khong nam tren duong di thi khong bao gio chui.
        """
        if nav is None or nav.get("hold") or self.gap_wp_max_w <= 0.0:
            return None
        Pa = np.asarray(nav["path"], dtype=np.float64)
        if Pa.shape[0] < 3:
            return None
        P = self.obstacles_gap if self.obstacles_gap.shape[0] else self.obstacles
        if target_xy is not None and P.shape[0]:
            P = P[np.hypot(P[:, 0] - target_xy[0], P[:, 1] - target_xy[1]) > min(0.35, self.target_clear_r)]
        if P.shape[0] < 2:
            return None
        cum = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(Pa[:, 0]), np.diff(Pa[:, 1])))])
        # Dang chui do (nhip truoc o pha qua khe) -> o vi tri xe xet ca doan THAN XE: duoi con trong khe thi van giu
        # dong tac qua khe. Khong dang chui thi KHONG: xe dung giua hai vat o hai ben sau lung (vd. vua qua cua, canh
        # tu lanh) cho ra "khe" tai cho xe dung, truc lech 60 do -> xe lui nguoc vao cua (mo phong 03/11).
        crossing_now = (time.time() - self.gap_cross_time) < 0.6
        # Xet het tam gap_engage_range + 0.3 m, ca khi cho dung cho (cach nguoi follow_distance) con truoc cua: nguoi
        # qua cua roi re ngay sau tuong thi xe PHAI qua cua moi thay lai nguoi (thu gioi han "chi doan xe se di" 03/11:
        # mo phong tu lanh re trai 6/6 -> 1-2/6).
        res = self._gap_on_path_scan(Pa, P, cum, crossing_now, self.gap_engage_range + 0.3)
        if res is None:
            return None
        w, i, il, ir, tdx, tdy = res
        if w < 2.0 * (self.half_width + self.gap_cross_margin) + 0.02:
            # 05/11: mat cat HEP HON XE (vd. khe 0.4 m giua vat chinh va ghe — chi nguoi lot; duong ban do di qua do nho
            # o "thoat ket" quanh xe) -> khong phai khe de chui. Truoc day tra ve khe 0.10-0.46 m, xe tien-lui mai.
            return None
        Lp, Rp = P[il], P[ir]
        ux, uy = Lp[0] - Rp[0], Lp[1] - Rp[1]
        un = max(1e-6, math.hypot(ux, uy))
        ux, uy = ux / un, uy / un
        cx_, cy_ = uy, -ux                          # vuong goc doan noi hai mep
        if cx_ * tdx + cy_ * tdy < 0.0:
            cx_, cy_ = -cx_, -cy_                   # chieu chui = chieu duong di
        mx, my = 0.5 * (Lp[0] + Rp[0]), 0.5 * (Lp[1] + Rp[1])
        nx, ny = -cx_, -cy_
        g = {"mx": mx, "my": my, "nx": nx, "ny": ny, "gw": float(un), "psi": math.atan2(cy_, cx_),
             "along": -(nx * mx + ny * my), "lat": abs(ux * mx + uy * my)}
        g2 = self._gap_refine(g, target_xy)
        if g2 is not None and g2.get("corridor"):
            return None
        g = g2 or g
        if g["along"] < -0.05 and not crossing_now:
            return None                         # khe da o sau tam xe ma khong dang chui: da qua / dung lech trong do
        self.gap_seen_t = time.time()
        return self._gap_store(g)

    def _gap_on_path_scan(self, Pa: np.ndarray, P: np.ndarray, cum: np.ndarray,
                          body: bool, s_max: float) -> Optional[Tuple[float, int, int, int, float, float]]:
        """Di doc duong Pa (toi quang s_max), tra ve mat cat HEP NHAT cua doan hep dau tien (be rong, chi so, mep trai,
        mep phai, tiep tuyen). body = o vi tri xe xet ca doan than xe (duoi xe con trong khe)."""
        run = []
        w_max = min(self.gap_wp_max_w, self.gap_path_max_w)
        for i in range(Pa.shape[0]):
            if cum[i] > s_max:
                break
            j0, j1 = max(0, i - 2), min(Pa.shape[0] - 1, i + 2)
            tdx, tdy = Pa[j1, 0] - Pa[j0, 0], Pa[j1, 1] - Pa[j0, 1]
            tn = math.hypot(tdx, tdy)
            if tn < 1e-6:
                continue
            tdx, tdy = tdx / tn, tdy / tn
            qx, qy = P[:, 0] - Pa[i, 0], P[:, 1] - Pa[i, 1]
            al = qx * tdx + qy * tdy
            la = -qx * tdy + qy * tdx
            lo = -(self.rear_len + 0.05) if (i == 0 and body) else -0.20
            win = (al >= lo) & (al <= 0.20) & (np.abs(la) <= 0.75)
            lm, rm_ = win & (la > 0.0), win & (la < 0.0)
            if not lm.any() or not rm_.any():
                if run:
                    break
                continue
            il = int(np.where(lm)[0][np.argmin(la[lm])])
            ir = int(np.where(rm_)[0][np.argmax(la[rm_])])
            w = float(la[il] - la[ir])
            if w < w_max:
                run.append((w, i, il, ir, tdx, tdy))
            elif run:
                break
        if not run:
            return None
        return min(run, key=lambda r: r[0])

    def _gap_refine(self, g: Dict[str, float],
                    target_xy: Optional[Tuple[float, float]] = None) -> Optional[Dict[str, float]]:
        """Dat lai khe vao MAT CAT HEP NHAT cua loi di quanh khe vua chon (14/10).

        _gap_target lay hai diem lidar KE NHAU THEO GOC. O cua that, cho do thuong KHONG phai cho
        hep nhat: tuong day, khung cua nho ra, rem buoc gon, canh cua mo ap mot ben bien loi di thanh
        mot "duong ham" ngan, va hai thanh cua o mieng cua nam o goc nhin rat cheo nen khong ke nhau
        theo goc. Xe that 14/10 (bag follow_1014_153520, cua bep 0.80 m): khe chon duoc la cho mo
        0.95 m PHIA TRONG bep (tuong trai trong bep <-> canh cua), lech 7-8 do so voi cua that ->
        xe "da vao truc" (lech 0-1 cm) theo truc sai, that ra lech trai 4 cm + nghieng 8 do -> goc
        truoc trai cach thanh cua 4-5 cm -> xoay qua xoay lai mai; lan sau khe nhay qua lai giua cua
        that va mot cap diem cheo -> xe tien lui mai.

        Cach tinh: thu cac huong chui quanh huong cu (+-16 do, buoc 2 do). Voi moi huong, chieu cac
        diem lidar tu 0.7 m truoc khe toi 0.3 m sau khe (ke ca canh than xe) len truc ngang, lay
        khoang TRONG LIEN TUC chua tam khe cu = be rong duong THANG lot qua that. Chon huong rong
        nhat, phat nhe khi lech huong cu / truc nhip truoc (1-2 mm moi do) de khong nhay vi nhieu. Tam
        khe moi = giua khoang trong do, dat o mat cat hep dau tien xe gap. Duoi 2*(half_width +
        gap_cross_margin) + 2 cm (xe khong the lot) hoac khong tinh duoc thi tra None.
        """
        P = self.obstacles_full if self.obstacles_full.shape[0] else self.obstacles
        if P.shape[0] < 2:
            return None
        if target_xy is not None:
            # Bo chan nguoi dang bam (nguoi vua buoc qua cua thi chan ho khong duoc lam lech khe). Vi tri du
            # doan troi toi sat thanh cua thi xoa luon thanh cua -> khe "no" ra: bat o dieu kien be rong ben
            # duoi (mat cat hep nhat khong the rong hon khe cu).
            d_t = np.hypot(P[:, 0] - target_xy[0], P[:, 1] - target_xy[1])
            P = P[d_t > min(0.35, self.target_clear_r)]
        mx, my, psi0 = g["mx"], g["my"], g["psi"]
        lat_w = 0.5 * g["gw"] + 0.35
        cands = [psi0 + math.radians(float(d)) for d in range(-16, 17, 2)]
        # Truc nhip truoc (khung odom) — chi khi cung mot khe (tam gan, huong gan)
        prev = None
        now_ = time.time()
        gox, goy = self._odom_of(mx, my)          # tam khe CU (khung odom) — de nhan ra "cung mot khe"
        if self.gap_axis_prev is not None and now_ - self.gap_axis_prev[3] < 1.0:
            pa, pox, poy, _t = self.gap_axis_prev
            pr = wrap_pi(pa - self.robot_yaw)
            if math.hypot(gox - pox, goy - poy) < 0.5 and abs(wrap_pi(pr - psi0)) < math.radians(20.0):
                prev = pr
                cands.append(pr)
        best = None
        for th in cands:
            c, s_ = math.cos(th), math.sin(th)
            S = P[:, 0] * c + P[:, 1] * s_
            L = -P[:, 0] * s_ + P[:, 1] * c
            sm = mx * c + my * s_
            lm = -mx * s_ + my * c
            win = ((S >= max(sm - 0.70, -self.rear_len - 0.05)) & (S <= sm + 0.30)
                   & (np.abs(L - lm) <= lat_w))
            if not np.any(win):
                continue
            Lw, Sw = L[win], S[win]
            lft = Lw > lm
            if not lft.any() or lft.all():
                continue                           # mot ben khong co gi chan: khong phai loi di
            l_l = float(np.min(Lw[lft]))
            l_r = float(np.max(Lw[~lft]))
            w = l_l - l_r
            # Phat lech: 1 mm moi do so voi huong khe cu, 2 mm moi do so voi truc nhip truoc (chong nhay vi
            # nhieu). Phai NHO hon do doc be rong theo goc cua cua that: tuong day 12 cm chi hep them ~2 mm
            # moi do nghieng — phat 4 mm/do thi khong bao gio xoay duoc truc ve phap tuyen that.
            dev = 0.057 * abs(wrap_pi(th - psi0)) + (0.115 * abs(wrap_pi(th - prev)) if prev is not None else 0.0)
            score = w - dev
            if best is None or score > best[0]:
                # Mat cat hep DAU TIEN xe gap: s nho nhat cua MOI diem sat mep khoang trong (trong 3 cm ra
                # ngoai), khong chi diem chan mep. Tuong day: goc ngoai va goc trong thanh cua cung toa do
                # ngang, lay diem nao la ngau nhien -> 'along' nhay 12 cm moi nhip, cham nguong lui (mo phong
                # cua ben tuong day 14/10: xe lui - tien mai cach mat cua 0.5 m).
                e_l = (Lw >= l_l - 1e-3) & (Lw <= l_l + 0.03)
                e_r = (Lw <= l_r + 1e-3) & (Lw >= l_r - 0.03)
                # Do SAU loi di: thanh chan mep chay dai bao nhieu doc truc (tuong song song truc = hanh lang).
                # Dai 10 cm (khong phai 3 cm): truc lech tuong vai do thi tuong dai 1 m chi con ~0.3 m trong dai 3 cm.
                d_l = (Lw >= l_l - 1e-3) & (Lw <= l_l + 0.10)
                d_r = (Lw <= l_r + 1e-3) & (Lw >= l_r - 0.10)
                depth = max(float(np.ptp(Sw[d_l])), float(np.ptp(Sw[d_r])))
                best = (score, th, w, 0.5 * (l_l + l_r), float(np.min(Sw[e_l | e_r])), depth)
        # Mat cat hep nhat KHONG THE rong hon khe cu (hai mep cu nam trong cua so): rong hon la da mat diem
        # (bi che / bi loai nham) -> khong tin.
        if (best is None or best[2] < 2.0 * (self.half_width + self.gap_cross_margin) + 0.02
                or best[2] > min(g["gw"] + 0.03, self.gap_wp_max_w)):
            return None
        _sc, th, w, cl, s_star, depth = best
        dx, dy = math.cos(th), math.sin(th)
        mx2, my2 = s_star * dx - cl * dy, s_star * dy + cl * dx
        self.gap_axis_prev = (wrap_pi(th + self.robot_yaw), gox, goy, now_)
        return {"mx": mx2, "my": my2, "nx": -dx, "ny": -dy, "gw": w,
                "psi": wrap_pi(th), "along": s_star, "lat": abs(cl),
                # HANH LANG rong (thanh chan chay dai doc truc, vd. giua thung va tuong): khong phai khung cua,
                # DWA / ban do tu di duoc. Dong tac chui khe o day thi 'along' tinh tu dau cua so (tuong chay
                # suot) -> luat lui ve truc kich hoat -> lui - dung mai (mo phong 14/10: vong qua thung bi chan
                # ben trai, sang phai thi ket o mieng hanh lang 1.1 m). Cua hep van chui (rong < 0.95 m).
                "corridor": bool(w > 0.95 and depth > 0.45)}

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
                  standoff: Optional[float] = None, prefer_side: int = 0,
                  back: float = 1.0) -> Optional[Dict[str, Any]]:
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
        # back: luoi nhin bao xa SAU xe (thuong 1.0 m; bay sau 06/11: escape_far_back_m — xem _nav_get)
        x0, x1 = -float(back), float(np.clip(dist + 1.2, 2.0, 4.5))
        ey = float(np.clip(dist + 1.0, 1.8, 3.5))
        nxc = int(math.ceil((x1 - x0) / res))
        nyc = int(math.ceil(2.0 * ey / res))
        xs = x0 + (np.arange(nxc) + 0.5) * res
        ys = -ey + (np.arange(nyc) + 0.5) * res
        GX, GY = np.meshgrid(xs.astype(np.float32), ys.astype(np.float32))      # (ny, nx)

        # 03/11: diem DAY DU (vong quet + noi tia ke nhau + bo nho vat can) — chi vong quet hien tai thi tuong nhin
        # cheo con lo hong va vung mu sau duoi xe trong tron.
        P = self.obstacles_dense if self.obstacles_dense.shape[0] else self.obstacles
        if P.shape[0] > 0:
            keep = np.hypot(P[:, 0] - tx, P[:, 1] - ty) > self.nav_person_clear
            Q = P[keep].astype(np.float32)
        else:
            Q = np.zeros((0, 2), dtype=np.float32)
        if _edt is not None:
            # Bien doi khoang cach tren LUOI MIN 3 lan (o 2 cm): nhanh, khong phu thuoc so diem (diem day du co the
            # vai tram), sai so ~1 cm — cua 0.81 m chi con dai 0.21 m cho tam xe di qua, luoi 6 cm sai 3 cm la khep
            # mat cua. Mo rong 0.6 m quanh luoi de diem sat bien van tinh.
            f3 = 3
            rf = res / f3
            pad = int(math.ceil(0.6 / rf))
            nyf, nxf = nyc * f3 + 2 * pad, nxc * f3 + 2 * pad
            occ = np.zeros((nyf, nxf), dtype=bool)
            if Q.shape[0]:
                ci_ = np.floor((Q[:, 0] - x0) / rf).astype(np.int64) + pad
                ri_ = np.floor((Q[:, 1] + ey) / rf).astype(np.int64) + pad
                ok_ = (ci_ >= 0) & (ci_ < nxf) & (ri_ >= 0) & (ri_ < nyf)
                occ[ri_[ok_], ci_[ok_]] = True
            if occ.any():
                # tam o luoi 6 cm = tam o min giua khoi 3x3
                dobs = (_edt(~occ) * rf)[pad + 1:pad + nyc * f3:f3, pad + 1:pad + nxc * f3:f3].astype(np.float32)
            else:
                dobs = np.full(GX.shape, np.inf, dtype=np.float32)
        else:
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
            relax = (np.hypot(GX, GY) < 0.35) & (dobs >= d0 - 0.01)
            if Q.shape[0]:
                # 05/11: chi cac o KHONG tien ve phia vat gan nhat (ra xa / di ngang). Truoc day xe dung o mieng khe
                # 0.4 m (chan ghe ngay truoc mui) thi cac o TRONG khe cung "khong gan vat hon" -> duong ban do di xuyen
                # khe chi nguoi lot, xe tien-lui mai o do (mo phong tui hep 05/11).
                qn = Q[int(np.argmin(np.hypot(Q[:, 0], Q[:, 1])))]
                qd = math.hypot(float(qn[0]), float(qn[1]))
                if qd > 1e-3:
                    relax &= (GX * float(qn[0]) + GY * float(qn[1])) / qd <= 0.03
            lethal &= ~relax
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
        out = {"path": path, "target": tgt_xy, "hold": hold, "cost": float(D[k]), "J": float(J[k]),
               "person_d": float(dp[k])}
        if _sp_dijkstra is not None:
            # 05/11: giu luoi + do thi de bo THOAT KET tinh "con bao xa toi dich" tu MOI o (Dijkstra nguoc tu dich)
            out["grid"] = {"G": G, "k": k, "nxc": nxc, "nyc": nyc, "res": res, "x0": x0, "ey": ey,
                           "pose": (self.robot_x, self.robot_y, self.robot_yaw)}
        return out

    def _nav_lookahead(self, path, tx: float, ty: float) -> Tuple[float, float]:
        """Diem xa nhat tren duong (<= nav_lookahead_m) ma doan thang tu xe toi do khong cham vat."""
        P = self.obstacles_dense if self.obstacles_dense.shape[0] else self.obstacles
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
        # 06/11 — BAY SAU (tui chu U, hanh lang cut): luoi thuong chi nhin 1 m sau xe -> xe vao sau hon ~0.8 m thi loi ra
        # (vong qua dau vach) nam ngoai luoi, ban do bao "dung o cho tot nhat" -> thoat ket va bo phat hien ket deu tat,
        # xe dung mai trong khi nguoi di xa (mo phong tui U sau 1.3 m / hanh lang cut 2 m: 0/44). Nguoi con xa ma ban do
        # thuong bao dung cho / khong co duong -> lap lai voi luoi nhin escape_far_back_m sau xe; tim ra loi ra re hon ro
        # thi dung, va giu luoi rong 4 s (dang lui ra thi khong nhay qua lai giua hai luoi).
        ext_recent = (now - self.nav_ext_t) < 4.0
        back = self.esc_far_back if ext_recent else 1.0
        plan = self._nav_plan(tx, ty, moving, back=back)
        if (not ext_recent and self.escape_on and (plan is None or plan["hold"])
                and math.hypot(tx, ty) > self.follow_distance + 0.6 and now - self.nav_ext_try_t > 1.0):
            self.nav_ext_try_t = now
            ext = self._nav_plan(tx, ty, moving, back=self.esc_far_back)
            if ext is not None and not ext["hold"] and (plan is None or ext["J"] < plan["J"] - 0.5):
                plan, back, ext_recent = ext, self.esc_far_back, True
                self.nav_ext_t = now
        if plan is None:
            self.nav_cache = None
            return None
        if ext_recent:
            plan["ext"] = True
            if any(px < -0.8 for (px, py) in plan["path"]):
                self.nav_ext_t = now               # duong van can doan xa sau xe -> giu luoi rong
        # GIU BEN VONG: vat doi xung (ghe dai thang truoc nguoi) thi hai ben re ngang nhau, moi lan lap lai
        # duong lai doi ben -> xe xoay qua xoay lai tai cho mai (mo phong 07/10). Chi doi ben khi ben kia
        # re hon ro (J nho hon 1.25 lan + 0.5 m); khong lap duong qua 3 s thi bo cam ket.
        s_new = self._nav_side_of(plan, tx, ty)
        if self.nav_side != 0 and s_new == -self.nav_side and (now - self.nav_side_t) < 3.0:
            alt = self._nav_plan(tx, ty, moving, prefer_side=self.nav_side, back=back)
            if alt is not None and ext_recent:
                alt["ext"] = True
            if (alt is not None and not alt["hold"] and self._nav_side_of(alt, tx, ty) == self.nav_side
                    and alt["J"] <= 1.25 * plan["J"] + 0.5):
                plan, s_new = alt, self.nav_side
        if s_new != 0:
            self.nav_side = s_new
        self.nav_side_t = now
        self.nav_cache = {"t": now, "plan": plan, "tgt_odom": self._odom_of(tx, ty),
                          "path_odom": [self._odom_of(px, py) for (px, py) in plan["path"]]}
        return plan

    def _nav_gap_path(self, now: float, tx: float, ty: float) -> Optional[Dict[str, Any]]:
        """Duong ban do toi TAN CHO NGUOI (cach 0.5 m) — chi de TIM KHE can chui (03/11).

        Duong bam binh thuong dung o cho tot nhat cach nguoi ~1 m: nguoi vua re vao cua ben hong thi cho do con o
        BEN NAY tuong -> duong khong qua cua -> khong thay khe, xe toi sat tuong lech truc 0.44 m roi moi thay
        (mo phong cua_ben). Duong toi tan nguoi thi qua cua ngay tu dau -> canh truc tu xa nhu truoc."""
        c = self.nav_gap_cache
        if c is not None and (now - c["t"]) < self.nav_replan:
            ox, oy = self._odom_of(tx, ty)
            if math.hypot(ox - c["tgt_odom"][0], oy - c["tgt_odom"][1]) < 0.3:
                path = [self._base_of(px, py) for (px, py) in c["path_odom"]]
                path[0] = (0.0, 0.0)
                out = dict(c["plan"])
                out["path"] = path
                return out
        # toi TAN cho nguoi (khong dung lai cach 1 m): nguoi o mieng cua thi duong van di qua cua -> thay khe som
        # 06/11: ban do chinh dang nhin xa sau xe (bay sau) -> duong tim khe cung vay (luoi 1 m khong thay loi ra, duong
        # tim khe lai dan vao sau trong tui, "xac nhan" nham khe gia giua hai vach tui)
        back = self.esc_far_back if (now - self.nav_ext_t) < 4.0 else 1.0
        plan = self._nav_plan(tx, ty, True, standoff=0.0, back=back)
        if plan is None:
            self.nav_gap_cache = None
            return None
        self.nav_gap_cache = {"t": now, "plan": plan, "tgt_odom": self._odom_of(tx, ty),
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

        ext = bool(nav.get("ext"))

        def back() -> Optional[Tuple[float, float]]:
            vb = max(-self.gap_cross_speed, self.v_min)       # toc do lui that -> kiem tra dung cung se chay
            if abs(la) > math.pi / 2:
                # diem ngam phia sau: lui theo cung toi do (do cong nhu ban 14/10)
                L2 = look[0] ** 2 + look[1] ** 2
                wb = float(np.clip(-self.gap_cross_speed * 2.0 * look[1] / max(L2, 1e-4), -self.w_max, self.w_max))
            else:
                # diem ngam phia truoc ma xe ket (khong tien, khong xoay duoc — goc mui ti vao vat): lui ra
                # mot doan ngan, vua lui vua xoay mui dan ve phia diem ngam, cho co cho xoay
                wb = float(np.clip(1.5 * la, -0.5, 0.5))
            # 06/11: duong ban do quay NGUOC ra sau (ngo cut: tui chu U, hanh lang cut) hoac ban do dang nhin xa sau xe ->
            # LUI THEO VET xe vua vao (vet 90 s, han muc rieng escape_far_back_m), roi moi toi han muc lui chung. Ket ma
            # diem ngam o PHIA TRUOC (goc ket, mieng cua) thi van chi han muc chung — chong lui-tien nhieu lan (bag 15/10).
            retr = ext or abs(la) > math.radians(110.0)
            for w_try in (wb, 0.5 * wb, 0.0):
                if retr and self._arc_ok(vb, w_try, trail_sec=self.esc_trail_sec) and self._retrace_take():
                    return vb, w_try
                if self._arc_ok(vb, w_try) and self._back_take():
                    return vb, w_try
            return None

        def rot() -> Optional[Tuple[float, float]]:
            if self._can_turn(la):
                return 0.0, float(np.clip(2.2 * la, -self.w_max * 0.9, self.w_max * 0.9))
            return None

        def esc(why: str) -> Optional[Tuple[float, float, str, float]]:
            # 05/11: xoay tai cho khong duoc -> TIM chuoi dong tac thoat ket (truoc khi lui mot doan "mu")
            if self._esc_start(nav, why):
                o = self._esc_step(dt)
                if o is not None:
                    return o[0], o[1], o[2], 0.0
            return None

        # 06/11: vua tim thoat ket KHONG ra ma xe dang sat vat (mui da chui vao mieng khe 0.70 m roi ban do doi sang
        # "vong", goc ket) -> trong 2.5 s chi xoay ve huong duong (neu quet duoc) hoac LUI (cung han muc / kiem tra cho
        # da biet trong nhu moi lan lui), khong de DWA tien lai: truoc day "lui 1 nhip - DWA tien 1 nhip" xen ke, xe chui
        # lai vao mieng khe, lan tim sau van tu tu the kho (can 7000-22000 buoc, tran 1500) -> dung giang co mai (mo
        # phong bay_khe 0.70 m tuong thap 0/3). Lui ra 10-15 cm thi lan tim sau thuong ra ngay.
        if time.time() - self.esc_retreat_t < 2.5:
            out = rot()
            if out is None:
                out = back()
            if out is not None:
                self.nav_rotating = out[0] == 0.0
                return out[0], out[1], ("ban do: ket — lui ra cho de xoay" if out[0] < 0
                                        else "ban do: xoay ve huong duong di"), 0.0
            self.esc_retreat_t = 0.0
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
            out = rot()
            if out is None:
                o4 = esc("duong ban do nam phia sau")
                if o4 is not None:
                    return o4
                out = back()
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
        # 06/11: nguong 0.15 -> 0.08 rad/s: chi phi giu khung hinh keo DWA queo CHAM (0.10-0.12) ve phia nguoi = ve
        # phia tuong, nguoc duong di doc tuong -> mui xe cham tuong, thoat ket, lai queo vao (mo phong bay_khe 0.60 m:
        # 10 lan thoat ket lien tiep, bo sot vi 0.12 < 0.15).
        now_ = time.time()
        if res is not None and abs(la) > math.radians(25.0) and res[1] * la < 0.0 and abs(res[1]) > 0.08:
            if self.nav_fight_t == 0.0:
                self.nav_fight_t = now_
        else:
            self.nav_fight_t = 0.0
        fights = self.nav_fight_t > 0.0 and now_ - self.nav_fight_t >= 0.8
        if fights and fov_target is not None:
            # quay nguoc duong da 0.8 s ma DANG giu khung hinh: thu lai DWA KHONG chi phi khung hinh (UU TIEN DUONG) —
            # di theo duong thi tam thoi mat camera (bo nho odom + tracker + SEARCH lo), con hon dam vao tuong mai
            res2 = self._dwa(look, fov_target, dt, allow_reverse=False, fov_active=False, fov_keep=self.nav_fov_keep)
            if res2 is not None and res2[0] > 0.02 and not (res2[1] * la < 0.0 and abs(res2[1]) > 0.08):
                return res2[0], res2[1], "di vong theo ban do (%s, %.1fm duong, uu tien duong)" % (side, nav["cost"]), \
                    res2[2]
        if res is not None and res[0] > 0.02 and not fights:
            return res[0], res[1], "di vong theo ban do (%s, %.1fm duong)" % (side, nav["cost"]), res[2]
        out = rot()
        if out is not None:
            self.nav_rotating = True
            return out[0], out[1], "ban do: xoay ve huong duong di", 0.0
        # 06/11 da thu va BO: "DWA khong tien duoc thi cho 0.6 s roi moi thoat ket / lui" — ke hoach thoat ket vua xong
        # ma DWA khong tien thi phai lap ke hoach moi NGAY (vd. lui cheo 8 cm cho thoang roi moi chay doc vat); cho thi DWA
        # bo sat vat 7-8 cm chi 0.04 m/s, mat nguoi (hoi quy 3 mam, nhom chu L: co cho 207/240, bo cho 227/240).
        o4 = esc("ban do ket, khong xoay duoc")
        if o4 is not None:
            return o4
        out = back()
        if out is not None:
            return out[0], out[1], "ban do: ket — lui ra cho de xoay", 0.0
        if res is not None:
            return res[0], res[1], "di vong theo ban do (%s, %.1fm duong)" % (side, nav["cost"]), res[2]
        return None

    # ─────────────────────────────────────────────────────────────────────
    # THOAT KET bang CHUOI DONG TAC (05/11)
    # ─────────────────────────────────────────────────────────────────────
    # Xe da vao cho chat (tui giua vat chinh va ke / ghe, goc) ma duong ban do quay NGUOC ra: xoay tai cho thi duoi
    # (0.33 m sau truc) quet vao vat, DWA chi nhin 1.2 s ve phia truoc, cac luat xoay / lui le te lui-tien tai cho mai
    # (bag 05/11 101558: 3 lan vao tui, 2 lan nguoi dung phai day tay ra). Bo nay TIM mot chuoi dong tac ngan nhu lai xe
    # quay dau trong ngo hep: xoay tai cho 15 do / tien theo cung 12 cm / lui theo cung 8 cm (chi vao cho DA BIET LA
    # TRONG), kiem tra footprint chu nhat that o tung tu the, toi tu the THANG HANG voi duong ban do va phia truoc thong.
    # Moi nhip khi chay van kiem tra lai bang _arc_clearance (vong quet moi nhat) — sai la dung, tim lai.
    _ESC_ROT = math.radians(15.0)

    def _esc_prims(self):
        """Dong tac co ban: (loai, v, w, T, chi_phi). R = xoay tai cho 15 do; F = tien 0.12 m; B = lui 0.08 m."""
        pr = getattr(self, "_esc_prim_list", None)
        if pr is None:
            pr = []
            wr = 0.5
            for sgn in (1.0, -1.0):
                pr.append(("R", 0.0, sgn * wr, self._ESC_ROT / wr, 0.06))
            vf = self.gap_cross_speed
            for k in (0.0, 2.0, -2.0, 4.0, -4.0):
                pr.append(("F", vf, vf * k, 0.12 / vf, 0.12))
            vb = max(-self.gap_cross_speed, self.v_min)
            for k in (0.0, 2.0, -2.0, 4.0, -4.0):
                pr.append(("B", vb, vb * k, 0.08 / abs(vb), 0.20))      # lui dat gap ~2.5 lan tien (lidar mu sau)
            self._esc_prim_list = pr
        return pr

    def _esc_template(self):
        """Diem mau footprint: vien moi 2 cm + trong long moi 6 cm (bat ca vat lot vao giua than xe)."""
        tp = getattr(self, "_esc_tpl", None)
        if tp is None:
            F, R, H = self.front_len, self.rear_len, self.half_width
            xs = np.arange(-R, F + 1e-9, 0.02)
            ys = np.arange(-H, H + 1e-9, 0.02)
            per = np.vstack([np.stack([xs, np.full_like(xs, H)], 1), np.stack([xs, np.full_like(xs, -H)], 1),
                             np.stack([np.full_like(ys, F), ys], 1), np.stack([np.full_like(ys, -R), ys], 1)])
            IX, IY = np.meshgrid(np.arange(-R + 0.05, F - 0.02, 0.06), np.arange(-H + 0.05, H - 0.02, 0.06))
            T = np.vstack([per, np.stack([IX.ravel(), IY.ravel()], 1)])
            # phan duoi: lui thi phai vao cho da biet la trong — TRU hai goc ngoai (|y| > _rev_hw, 06/11: xem _rev_hw)
            rear = (T[:, 0] <= -R + 0.081) & (np.abs(T[:, 1]) <= self._rev_hw + 1e-9)
            tp = (T, rear, per.shape[0])
            self._esc_tpl = tp
        return tp

    def _esc_fields(self, half: float = 1.5, trail_sec: float = 30.0) -> Optional[Dict[str, Any]]:
        """Truong khoang cach toi vat (luoi 1 cm) + luoi 'da biet la trong' (4 cm) quanh xe, khung base hien tai."""
        if _edt is None:
            return None
        res, pad = 0.01, 0.40
        L = half + pad
        parts = [a for a in (self.obstacles_full, self.mem_occl, self.obstacles) if a is not None and a.shape[0]]
        P = np.vstack(parts) if parts else np.zeros((0, 2))
        n = int(math.ceil(2.0 * L / res))
        occ = np.zeros((n, n), dtype=bool)
        if P.shape[0]:
            ci = np.floor((P[:, 0] + L) / res).astype(np.int64)
            ri = np.floor((P[:, 1] + L) / res).astype(np.int64)
            ok = (ci >= 0) & (ci < n) & (ri >= 0) & (ri < n)
            occ[ri[ok], ci[ok]] = True
        dist = (_edt(~occ) * res).astype(np.float32) if occ.any() else np.full((n, n), 9.0, dtype=np.float32)
        kres = 0.04
        nk = int(math.ceil(2.0 * half / kres))
        kc = -half + (np.arange(nk) + 0.5) * kres
        KX, KY = np.meshgrid(kc, kc)
        if self.rev_known_free:
            c, s_ = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
            Q = np.stack([self.robot_x + c * KX.ravel() - s_ * KY.ravel(),
                          self.robot_y + s_ * KX.ravel() + c * KY.ravel()], axis=1)
            kf = self._known_free(Q, trail_sec).reshape(nk, nk)
            # an toan hon: o "da biet trong" chi khi ca 4 o ke cung trong (tam o 4 cm, diem mau co the lech 2.8 cm)
            ke = kf.copy()
            ke[1:, :] &= kf[:-1, :]
            ke[:-1, :] &= kf[1:, :]
            ke[:, 1:] &= kf[:, :-1]
            ke[:, :-1] &= kf[:, 1:]
            kf = ke
        else:
            kf = np.ones((nk, nk), dtype=bool)
        return {"L": L, "res": res, "n": n, "dist": dist, "half": half, "kres": kres, "nk": nk, "kf": kf}

    def _esc_clear(self, f: Dict[str, Any], X: np.ndarray, Y: np.ndarray, TH: np.ndarray):
        """Do thoang (xap xi luoi 1 cm) cua footprint o cac tu the (X, Y, TH) — khung base luc lap ke hoach."""
        T, _rear, _np = self._esc_template()
        c, s_ = np.cos(TH)[:, None], np.sin(TH)[:, None]
        PX = X[:, None] + c * T[None, :, 0] - s_ * T[None, :, 1]
        PY = Y[:, None] + s_ * T[None, :, 0] + c * T[None, :, 1]
        n, L, res = f["n"], f["L"], f["res"]
        ci = np.floor((PX + L) / res).astype(np.int64)
        ri = np.floor((PY + L) / res).astype(np.int64)
        out = (ci < 0) | (ci >= n) | (ri < 0) | (ri >= n)
        d = f["dist"][np.clip(ri, 0, n - 1), np.clip(ci, 0, n - 1)]
        d = np.where(out, 0.0, d)                       # ra ngoai cua so = khong cho di (khong biet gi o do)
        return d.min(axis=1), PX, PY

    def _esc_kfree(self, f: Dict[str, Any], PX: np.ndarray, PY: np.ndarray) -> np.ndarray:
        kres, nk, half = f["kres"], f["nk"], f["half"]
        ci = np.floor((PX + half) / kres).astype(np.int64)
        ri = np.floor((PY + half) / kres).astype(np.int64)
        out = (ci < 0) | (ci >= nk) | (ri < 0) | (ri >= nk)
        v = f["kf"][np.clip(ri, 0, nk - 1), np.clip(ci, 0, nk - 1)] & ~out
        m_ = 0.01
        ins = (PX < self.front_len - m_) & (PX > -self.rear_len + m_) & (np.abs(PY) < self.half_width - m_)
        return v | ins

    def _esc_blind(self, PX: np.ndarray, PY: np.ndarray) -> np.ndarray:
        """Diem nam trong cung MU cua lidar nhin tu tu the luc lap ke hoach (sau duoi xe)."""
        m = np.zeros(PX.shape, dtype=bool)
        if not self.blind_sectors:
            return m
        a = (np.arctan2(PY - self.lidar_y, PX - self.lidar_x) - self.lidar_yaw_offset_rad) * self.lidar_angle_sign
        deg = np.degrees(a) % 360.0
        for lo, hi in self.blind_sectors:
            m |= (deg >= lo) & (deg <= hi)
        return m

    def _esc_hfield(self, grid: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Tu MOI o luoi: con bao xa toi dich (Dijkstra nguoc tu o dich cua ban do) + huong di xuong doc (theo
        duong ngan nhat 6 o ~0.36 m). O cam (tam xe qua sat vat) lay theo o tu do gan nhat + phat."""
        Dg, pred = _sp_dijkstra(grid["G"], directed=False, indices=grid["k"], return_predecessors=True)
        nyc, nxc = grid["nyc"], grid["nxc"]
        H = np.asarray(Dg, dtype=np.float64).reshape(nyc, nxc)
        fin = np.isfinite(H)
        if not fin.any():
            return None
        idx = np.arange(nyc * nxc)
        cur = idx.copy()
        for _ in range(6):
            nx_ = pred[cur]
            cur = np.where(nx_ >= 0, nx_, cur)
        cy, cx = np.divmod(idx, nxc)
        ly, lx = np.divmod(cur, nxc)
        DIR = np.arctan2((ly - cy).astype(np.float64), (lx - cx).astype(np.float64)).reshape(nyc, nxc)
        if not fin.all() and _edt is not None:
            dd, inds = _edt(~fin, return_indices=True)
            Hf = H[inds[0], inds[1]] + dd * grid["res"] * 2.0 + 0.10
            Df = DIR[inds[0], inds[1]]
            Hf[fin] = H[fin]
            Df[fin] = DIR[fin]
            H, DIR = Hf, Df
        return {"H": H, "DIR": DIR, "g": grid}

    def _esc_hlook(self, hf: Dict[str, Any], X: np.ndarray, Y: np.ndarray, TH: np.ndarray,
                   pose: Optional[Tuple[float, float, float]] = None):
        """(con bao xa toi dich, lech mui so voi huong xuong doc) cho tu the (X, Y, TH) khung base cua `pose` (tu the
        luc LAP ke hoach — tim chia nhieu nhip nen khong dung tu the hien tai; mac dinh = tu the hien tai)."""
        g = hf["g"]
        px, py, pyaw = g["pose"]
        rx, ry, ryaw = pose if pose is not None else (self.robot_x, self.robot_y, self.robot_yaw)
        c, s_ = math.cos(ryaw), math.sin(ryaw)
        OX = rx + c * X - s_ * Y
        OY = ry + s_ * X + c * Y
        dx, dy = OX - px, OY - py
        cp, sp = math.cos(pyaw), math.sin(pyaw)
        BX, BY = cp * dx + sp * dy, -sp * dx + cp * dy
        ci = np.clip(np.round((BX - g["x0"]) / g["res"] - 0.5).astype(np.int64), 0, g["nxc"] - 1)
        ri = np.clip(np.round((BY + g["ey"]) / g["res"] - 0.5).astype(np.int64), 0, g["nyc"] - 1)
        err = np.abs(wrap_pi_arr(TH + (ryaw - pyaw) - hf["DIR"][ri, ci]))
        return hf["H"][ri, ci], err

    def _esc_search(self, f: Dict[str, Any], hf: Optional[Dict[str, Any]], max_exp: int,
                    face: Optional[Tuple[float, float, float, float, float]] = None, t_scale: float = 1.0):
        """Chay _esc_search_gen mot mach toi xong (khong chia nhip) — de thu offline."""
        gen = self._esc_search_gen(f, hf, max_exp, face=face, t_scale=t_scale)
        try:
            while True:
                next(gen)
        except StopIteration as si:
            return si.value

    def _esc_search_gen(self, f: Dict[str, Any], hf: Optional[Dict[str, Any]], max_exp: int,
                        face: Optional[Tuple[float, float, float, float, float]] = None, t_scale: float = 1.0):
        """A* tren (x, y, huong) voi cac dong tac co ban. GENERATOR (06/11): moi lan next() tim escape_slice_exp buoc
        roi yield None (xe dung yen, nhip dieu khien sau tim tiep); xong thi StopIteration.value = danh sach dong tac
        [(loai, v, w, T)] da gop, [] neu tu the hien tai da dat, None neu khong tim thay. Truoc 06/11 tim mot mach voi
        tran 0.25 s THOI GIAN THAT: may ban (laptop chay YOLO, mini PC cham ~3 lan, 28 mo phong song song) thi tim duoc
        it buoc hon han -> bay sau khong ra ke hoach (mo phong tui U 0.90 m sau 1.3 m: chay rieng DAT, chay chung LOI).
        Nay tran theo SO BUOC (max_exp) + tong thoi gian escape_max_sec * t_scale chia nhieu nhip.

        face = (px, py, d_lo, d_hi, tol) (06/11): DICH la QUAY MAT ve diem (px, py) (khung base luc lap) lech <= tol, cach
        diem do trong [d_lo, d_hi] — xe da toi noi ma xoay tai cho khong duoc (duoi vuong tuong). Khong dung ban do."""
        pose0 = (self.robot_x, self.robot_y, self.robot_yaw)    # khung lap ke hoach (co dinh suot cac nhip)
        prims = self._esc_prims()
        T_tpl, rear, n_per = self._esc_template()
        thr = self.margin_hard + 0.012                  # luoi 1 cm sai toi ~1 cm: chat hon mot chut
        floor_ = 0.02 + 0.012
        a_tol = math.radians(25.0)
        if face is not None:
            fx_, fy_, d_lo, d_hi, a_tol = face

            def look(X, Y, TH):
                X, Y, TH = np.asarray(X, float), np.asarray(Y, float), np.asarray(TH, float)
                dd_ = np.hypot(fx_ - X, fy_ - Y)
                return (np.maximum(0.0, d_lo - dd_) + np.maximum(0.0, dd_ - d_hi),
                        np.abs(wrap_pi_arr(np.arctan2(fy_ - Y, fx_ - X) - TH)))
        else:
            def look(X, Y, TH):
                return self._esc_hlook(hf, X, Y, TH, pose0)
        # mau thoi gian cua tung dong tac
        samp = []
        for (kind, v, w, T, cst) in prims:
            ns = 3 if kind != "F" else 4
            samp.append((np.arange(1, ns + 1) / ns) * T)

        def child_poses(x, y, th):
            Xs, Ys, THs, owner = [], [], [], []
            for j, (kind, v, w, T, cst) in enumerate(prims):
                t = samp[j]
                if abs(w) < 1e-9:
                    X = x + v * t * math.cos(th)
                    Y = y + v * t * math.sin(th)
                    TH = np.full(t.shape, th)
                else:
                    TH = th + w * t
                    X = x + (v / w) * (np.sin(TH) - math.sin(th))
                    Y = y - (v / w) * (np.cos(TH) - math.cos(th))
                Xs.append(X)
                Ys.append(Y)
                THs.append(TH)
                owner.append(np.full(t.shape, j))
            return np.concatenate(Xs), np.concatenate(Ys), np.concatenate(THs), np.concatenate(owner)

        def h_of(Hv, err, H0):
            return 0.06 * (err / self._ESC_ROT) + 0.5 * np.maximum(0.0, Hv - H0)

        def is_goal(x, y, th, cn, H0):
            if cn < thr:
                return False
            Hv, err = look(np.array([x]), np.array([y]), np.array([th]))
            if face is not None:
                return bool(Hv[0] <= 1e-6 and err[0] <= a_tol)
            if not np.isfinite(Hv[0]) or Hv[0] > H0 + 0.10 or err[0] > a_tol:
                return False
            dd = np.array([0.10, 0.20, 0.30])
            cl, _px, _py = self._esc_clear(f, x + dd * math.cos(th), y + dd * math.sin(th), np.full(3, th))
            return bool(cl.min() >= thr)

        c_s, _px, _py = self._esc_clear(f, np.zeros(1), np.zeros(1), np.zeros(1))
        c0 = float(c_s[0])
        H0a, e0a = look(np.zeros(1), np.zeros(1), np.zeros(1))
        H0 = float(H0a[0])
        if not np.isfinite(H0):
            return None
        if is_goal(0.0, 0.0, 0.0, c0, H0):
            return []
        # node: (x, y, th, g, parent, prim, huong tinh tien +1/-1/0, do thoang)
        nodes = [(0.0, 0.0, 0.0, 0.0, -1, -1, 0, c0)]
        heap = [(float(h_of(np.array([H0]), e0a, H0)[0]), 0)]
        closed: Dict[Tuple[int, int, int], float] = {}
        k_th = math.radians(7.5)
        goal_i = -1
        t_total = self.escape_max_sec * t_scale         # tran tong thoi gian (gioi han chinh: max_exp)
        t_used = 0.0
        t_s0 = _perf()
        _it = 0
        for _it in range(max_exp):
            if not heap:
                break
            # moi nhip dung escape_slice_exp buoc (TAT DINH: mo phong lap lai duoc, laptop / mini PC cung mot ke hoach,
            # chi khac thoi gian cho); may cham bat thuong (mot nhip > 0.2 s) thi nhuong som
            if (_it % self.escape_slice_exp) == self.escape_slice_exp - 1 or ((_it & 15) == 15 and _perf() - t_s0 > 0.2):
                t_used += _perf() - t_s0
                if t_used > t_total:
                    break                               # het tong thoi gian — lan sau tim lai tu tu the moi
                self.esc_exp = _it + 1
                yield None                              # nhuong nhip dieu khien (xe dung yen), nhip sau tim tiep
                t_s0 = _perf()
            _fs, ni = heapq.heappop(heap)
            x, y, th, gc, _par, _pi, dr, cn = nodes[ni]
            key = (int(round(x / 0.025)), int(round(y / 0.025)), int(round(th / k_th)) % 48)
            if key in closed and closed[key] <= gc + 1e-9:
                continue
            closed[key] = gc
            if ni != 0 and is_goal(x, y, th, cn, H0):
                goal_i = ni
                break
            if len(nodes) > 40 * max_exp:
                break
            X, Y, TH, own = child_poses(x, y, th)
            cl, PX, PY = self._esc_clear(f, X, Y, TH)
            # vien footprint roi vao cung MU (nhin tu tu the luc lap ke hoach) ngoai than xe luc dau: phai da biet trong
            per = slice(0, n_per)
            bl = self._esc_blind(PX[:, per], PY[:, per])
            okb = ~bl | self._esc_kfree(f, PX[:, per], PY[:, per])
            blind_ok = okb.all(axis=1)
            keep, lasts = [], []
            for j, (kind, v, w, T, cst) in enumerate(prims):
                sel = own == j
                mn = float(cl[sel].min())
                if not (mn >= thr or (cn < thr and mn >= max(floor_, cn - 0.005))):
                    continue
                if not bool(blind_ok[sel].all()):
                    continue
                if kind == "B":
                    rx_, ry_ = PX[sel][:, rear], PY[sel][:, rear]
                    if not bool(self._esc_kfree(f, rx_, ry_).all()):
                        continue
                keep.append(j)
                lasts.append(int(np.where(sel)[0][-1]))
            if not keep:
                continue
            li = np.array(lasts)
            Hv, err = look(X[li], Y[li], TH[li])
            hh = h_of(Hv, err, H0)
            for q, j in enumerate(keep):
                if not np.isfinite(Hv[q]):
                    continue
                kind, v, w, T, cst = prims[j]
                last = lasts[q]
                nd = 1 if kind == "F" else (-1 if kind == "B" else dr)
                g2 = gc + cst + (0.10 if (kind != "R" and dr != 0 and nd != dr) else 0.0)
                g2 += 0.5 * max(0.0, float(Hv[q]) - H0)          # di vao sau hon trong ngo cut thi dat
                nodes.append((float(X[last]), float(Y[last]), float(TH[last]), g2, ni, j, nd, float(cl[last])))
                heapq.heappush(heap, (g2 + 1.5 * float(hh[q]), len(nodes) - 1))
        self.esc_exp = _it + 1 if max_exp > 0 else 0          # so buoc tim (de log / chinh gioi han CPU)
        if goal_i < 0:
            return None
        seq = []
        i = goal_i
        while i > 0:
            seq.append(nodes[i][5])
            i = nodes[i][4]
        seq.reverse()
        out = []
        for j in seq:
            kind, v, w, T, _c = prims[j]
            if out and out[-1][0] == kind and abs(out[-1][1] - v) < 1e-9 and abs(out[-1][2] - w) < 1e-9:
                out[-1] = (kind, v, w, out[-1][3] + T)
            else:
                out.append((kind, v, w, T))
        return out

    def _esc_start(self, nav: Optional[Dict[str, Any]], why: str) -> bool:
        """Bat dau TIM ke hoach thoat ket (06/11: chia nhip — _esc_step tim tiep moi nhip, xe dung yen; co ke hoach thi
        chay luon). True = da bat dau tim."""
        now = time.time()
        # >= 1 s tu luc BAT DAU lan tim truoc va >= 1 s tu luc lan tim truoc KET THUC khong ra ke hoach: tim chia nhip mat
        # ~0.7 s, chi dem tu luc bat dau thi gan nhu khong con nhip nao cho cach lui / xoay cu (lui 2-3 cm cho co cho xoay
        # — ban 05/11 tim mot nhip nen van co) -> xe dung im truoc khe 0.60 m mai (mo phong 06/11, bay_khe tuong cao)
        if (not self.escape_on or nav is None or nav.get("hold") or _sp_dijkstra is None or _edt is None
                or now - self.esc_try_t < 1.0 or now - self.esc_fail_t < 1.0):
            return False
        if nav.get("ext") and self.esc_fail_ext and now - self.esc_fail_t < 3.0:
            return False        # 06/11 bay sau: vua tim khong ra -> de xe lui theo vet them mot doan roi moi tim lai
        self.esc_try_t = now
        grid = nav.get("grid")
        if grid is None:
            return False
        hf = self._esc_hfield(grid)
        if hf is None:
            return False
        # 06/11: ban do dang nhin XA sau xe (bay sau) -> cua so tim cung rong theo, tim lau hon (xe dang dung yen)
        ext = bool(nav.get("ext"))
        f = self._esc_fields(half=(self.esc_far_back if ext else 1.5), trail_sec=self.esc_trail_sec)
        if f is None:
            return False
        # tim CHIA NHIP: _esc_step chay tiep moi nhip (xe dung yen) toi khi co ke hoach / het tran
        gen = self._esc_search_gen(f, hf, self.escape_max_exp * (2 if ext else 1), t_scale=(2.0 if ext else 1.0))
        self.esc = {"phase": "search", "gen": gen, "why": why, "mode": "nav", "t_plan": now, "ext": ext}
        return True

    def _esc_begin(self, plan, why: str, mode: str) -> None:
        now = time.time()
        self.esc = {"phase": "run", "prims": plan, "i": 0, "p0": (self.robot_x, self.robot_y, self.robot_yaw),
                    "t0": now, "acc": 0.0, "last": (self.robot_x, self.robot_y, self.robot_yaw), "t_plan": now,
                    "why": why, "mode": mode}
        self.get_logger().info("Thoat ket (%s, %d buoc tim): %s" % (why, getattr(self, "esc_exp", 0), " ".join(
            "%s%+.0f%s" % (k, math.degrees(w * T) if k == "R" else 100.0 * v * T, "do" if k == "R" else "cm")
            for (k, v, w, T) in plan)))

    def _esc_start_face(self, tx: float, ty: float, why: str) -> bool:
        """06/11: da toi noi (cach nguoi ~1 m) ma XOAY TAI CHO ve phia nguoi khong duoc (duoi 0.33 m vuong tuong / vat)
        va nhich toi thi chi sat nguoi them -> tim chuoi dong tac ngan (lui / xoay / tien) toi tu the QUAY MAT ve nguoi
        (lech <= 12 do), van cach nguoi 0.75 m tro len. Cung bo dong tac, cung kiem tra va cham / cho da biet la trong
        nhu thoat ket theo ban do."""
        now = time.time()
        if not self.escape_on or _edt is None or now - self.esc_try_t < 1.0 or now - self.esc_fail_t < 1.0:
            return False
        self.esc_try_t = now
        f = self._esc_fields(trail_sec=self.esc_trail_sec)
        if f is None:
            return False
        d = math.hypot(tx, ty)
        face = (tx, ty, 0.75, max(d, self.follow_distance) + 0.5, math.radians(12.0))
        gen = self._esc_search_gen(f, None, self.escape_max_exp, face=face)
        self.esc = {"phase": "search", "gen": gen, "why": why, "mode": "face", "t_plan": now}
        return True

    def _esc_step(self, dt: float) -> Optional[Tuple[float, float, str]]:
        """Chay tiep ke hoach thoat ket: (v, w, ghi chu) hoac None (xong / bo vi khong con an toan)."""
        e = self.esc
        if e is None:
            return None
        if e.get("phase") == "search":
            # 06/11: dang TIM (chia nhip) — tim tiep mot lat, chua xong thi dung yen cho nhip sau. Luc tim van tinh la DANG
            # bam ban do: khong thi _nav_follow tuong "lau roi moi dung ban do" (> 0.5 s), dat lai bo dem — lai "dung cho
            # 2 s" (duong phia sau, con thay nguoi) — va lan tim sau bat dau truoc khi kip lui / xoay
            if e.get("mode") == "nav":
                self.nav_follow_t = time.time()
            try:
                next(e["gen"])
                return 0.0, 0.0, "thoat ket — dang tim duong (%d buoc)" % getattr(self, "esc_exp", 0)
            except StopIteration as si:
                plan = si.value
            if not plan:
                self.esc = None
                self.esc_fail_t = time.time()
                self.esc_fail_ext = bool(e.get("ext"))
                if e.get("mode") == "nav" and self._clear_now() < self.margin_hard + 0.02:
                    self.esc_retreat_t = self.esc_fail_t       # xe dang sat vat: _nav_follow lui ngan truoc (xem do)
                return None
            self._esc_begin(plan, e["why"], e["mode"])
            e = self.esc
        now = time.time()
        lx_, ly_, la_ = e["last"]
        e["acc"] += (abs(wrap_pi(self.robot_yaw - la_)) if e["prims"][min(e["i"], len(e["prims"]) - 1)][0] == "R"
                     else math.hypot(self.robot_x - lx_, self.robot_y - ly_))
        e["last"] = (self.robot_x, self.robot_y, self.robot_yaw)
        while e["i"] < len(e["prims"]):
            kind, v, w, T = e["prims"][e["i"]]
            need = abs(w) * T if kind == "R" else abs(v) * T
            if e["acc"] >= 0.95 * need:
                e["i"] += 1
                e["acc"] = 0.0
                e["t0"] = now
                continue
            if now - e["t0"] > 2.5 * T + 2.0 or now - e["t_plan"] > 30.0:
                self.esc = None            # khong nhuc nhich duoc (truot banh / bi giu) -> bo
                return None
            T_rem = max(self.horizon_dt, T * (1.0 - e["acc"] / max(need, 1e-6)))
            # + 0.2 s: tron khi giam toc o cuoi dong tac (0.12 m/s -> ~2 cm)
            steps = max(1, int(math.ceil((T_rem + 0.2) / self.horizon_dt)))
            mn, _en = self._arc_clearance(v, w, steps)
            c0 = self._clear_now()
            ok = mn > self.margin_hard or (c0 <= self.margin_hard and mn >= c0 - 0.005 and mn > 0.015)
            if ok and v < 0.0:
                ok = self._reverse_ok(v, w, T_rem, trail_sec=self.esc_trail_sec)
            if not ok:
                self.esc = None
                return None
            name = {"R": "xoay %s" % ("trai" if w > 0 else "phai"), "F": "tien", "B": "lui"}[kind]
            note = "thoat ket — %s (%d/%d)" % (name, e["i"] + 1, len(e["prims"]))
            if kind == "R":
                return 0.0, w, note
            # giu DUNG do cong khi tang toc (v tang dan theo accel_lin): xe chay dung cung da kiem tra
            vn = float(np.clip(v, self.cur_v - self.accel_lin * dt, self.cur_v + self.accel_lin * dt))
            if vn * v <= 0.0:
                return 0.0, 0.0, note
            return vn, w * vn / v, note
        self.esc = None
        return None

    def _stuck(self, now: float, goal_r: float, nav: Optional[Dict[str, Any]]) -> bool:
        """Xe dung trong o vuong 25 cm suot escape_stuck_sec giay trong khi le ra phai di (nguoi con xa, ban do khong
        bao dung cho, khong dang chui qua khe)."""
        h = self.prog_hist
        h.append((now, self.robot_x, self.robot_y))
        while h and now - h[0][0] > self.escape_stuck + 1.0:
            h.popleft()
        if (goal_r <= 0.35 or self.state == S_ARRIVED or (nav is not None and nav["hold"])
                or now - self.gap_cross_time < 1.0):
            self.stuck_reset_t = now
            return False
        if now - self.stuck_reset_t < self.escape_stuck or now - h[0][0] < self.escape_stuck:
            return False
        xs = [p[1] for p in h if now - p[0] <= self.escape_stuck]
        ys = [p[2] for p in h if now - p[0] <= self.escape_stuck]
        return (max(xs) - min(xs)) < 0.25 and (max(ys) - min(ys)) < 0.25

    def _odom_of(self, bx: float, by: float) -> Tuple[float, float]:
        c, s_ = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        return self.robot_x + c * bx - s_ * by, self.robot_y + s_ * bx + c * by

    def _base_of(self, ox: float, oy: float) -> Tuple[float, float]:
        dx, dy = ox - self.robot_x, oy - self.robot_y
        c, s_ = math.cos(-self.robot_yaw), math.sin(-self.robot_yaw)
        return c * dx - s_ * dy, s_ * dx + c * dy

    def _gap_pair(self, goal_b: float, goal_r: float, dist: float, tx: float, ty: float,
                  nav: Optional[Dict[str, Any]], now: Optional[float] = None) -> Optional[Dict[str, float]]:
        """Khe theo cach 14/10: cap tia lidar ke nhau (_gap_target), phai gan duong ban do (neu co) — tru khi da o
        mieng cua (gap_at_mouth)."""
        saved = (self.gap_mem, self.gap_seen_t, self.gap_axis_prev)
        gap = self._gap_target(goal_b, max(goal_r, dist), (tx, ty))
        at_mouth = (self.gap_at_mouth and gap is not None
                    and gap["along"] < self.front_len + self.margin_soft + 0.35 and gap["lat"] < 0.25)
        if gap is not None and not at_mouth and not self._gap_path_ok(nav, gap):
            # 05/11: duong "dung cach nguoi 1 m" thuong NGAN (nguoi o gan -> dung truoc cua) — xet them duong toi TAN cho
            # nguoi (duong tim khe): khe nam tren duong do va dung chieu thi van nhan. Truoc day khe bi loai o day lai
            # duoc _gap_recall goi lai ngay sau (xem duoi) nen thuc te van dung — mo phong lon xon kho / cua bep can no.
            gp = (self._nav_gap_path(now, tx, ty)
                  if (now is not None and nav is not None and self.gap_on_path and self.nav_enabled) else None)
            if gp is None or not self._gap_path_ok(gp, gap):
                gap = None
                # Khe BI LOAI (khong nam tren duong nao / duong di qua no NGUOC chieu chui) thi KHONG duoc nho. Truoc day
                # _gap_target da kip ghi gap_mem + gap_seen_t -> ngay sau do _gap_recall(hold_sec=1.0) goi lai dung khe vua
                # bi loai: xe "vao truc giua khe" mot khe khong nam tren duong, dung im 50 s (bag 05/11 101558 luc
                # 222-271 s, lenh (0, 0) suot).
                self.gap_mem, self.gap_seen_t, self.gap_axis_prev = saved
        return gap

    @staticmethod
    def _path_backward(plan: Optional[Dict[str, Any]]) -> bool:
        """Duong ban do (khong phai 'dung cho') bat dau bang doan QUAY NGUOC ra sau xe (> 100 do)."""
        if plan is None or plan.get("hold") or not plan.get("path"):
            return False
        far = [p for p in plan["path"] if math.hypot(p[0], p[1]) >= 0.30]
        return bool(far) and abs(math.atan2(far[0][1], far[0][0])) > math.radians(100.0)

    @staticmethod
    def _path_near(plan: Optional[Dict[str, Any]], mx: float, my: float, r: float = 0.35) -> bool:
        if plan is None:
            return True
        return any(math.hypot(px - mx, py - my) < r for (px, py) in plan["path"])

    def _gap_path_ok(self, plan: Optional[Dict[str, Any]], g: Dict[str, float], need_near: bool = True) -> bool:
        """Khe co nam tren duong ban do VA duong di qua no CUNG chieu chui (psi) khong. Khong co ban do = khong loc.
        need_near=False: chi loai khi duong DI QUA khe NGUOC chieu (khe xa duong thi khong biet -> nhan) — dung trong
        SEARCH, vi tri nguoi du doan co the troi (nguoi re vao cua ben hong, du doan di thang).

        05/11: xe dung trong TUI HEP (giua vat chinh va ke), ban do bao di NGUOC ra mieng tui roi vong ben kia; cap tia
        mieng tui (vat chinh <-> ke) nam ngay tren duong do nen _path_near van nhan, nhung chieu chui tinh theo phia
        NGUOI (sau vat chinh) chi VAO sau trong tui -> dong tac chui khe keo xe vao ngo cut. Duong di qua khe nguoc
        chieu hon ~110 do thi bo khe (de ban do / thoat ket lo)."""
        if plan is None:
            return True
        mx, my = g["mx"], g["my"]
        path = plan["path"]
        if not path:
            return True
        if g["along"] < 0.05 and abs(g["psi"]) < math.radians(45.0):
            # Xe dang trong / vua qua mat khe, mui gan thang truc: dang chui — giu toi khi duoi ra, TRU KHI duong ban do
            # ngay tu cho xe da quay NGUOC lai (khe la ngo cut: nguoi lach qua khe hep chi nguoi lot, chan ghe luc nguoi
            # dung giua khe bi xoa cung vung quanh nguoi -> "khe" 0.8 m; 05/11 mo phong tui hep: xe tien-lui 68 lan).
            far = [p for p in path if math.hypot(p[0], p[1]) >= 0.30]
            if not far:
                return True
            return abs(wrap_pi(math.atan2(far[0][1], far[0][0]) - g["psi"])) < math.radians(100.0)
        d = [math.hypot(px - mx, py - my) for (px, py) in path]
        d = [math.hypot(px - mx, py - my) for (px, py) in path]
        i = int(np.argmin(d))
        if d[i] >= 0.35:
            return not need_near
        j0, j1 = max(0, i - 2), min(len(path) - 1, i + 2)
        tx_, ty_ = path[j1][0] - path[j0][0], path[j1][1] - path[j0][1]
        tn = math.hypot(tx_, ty_)
        if tn < 1e-6:
            return True
        return (tx_ * math.cos(g["psi"]) + ty_ * math.sin(g["psi"])) / tn > -0.35

    def _arc_ok(self, v: float, w: float, trail_sec: float = 30.0) -> bool:
        mn, en = self._arc_clearance(v, w)
        ok = bool(self._admissible(np.array([mn]), np.array([en]))[0])
        if not ok and v < 0.0:
            # LUI ra khi da sat vat (03/11): luat thoat ket doi cuoi quy dao thoang hon 1 cm — lui THANG ra khoi khe doc
            # thanh cua thi do thoang giu nguyen nen bi cam, chi con lui-xoay SAI chieu (xe lech truc them, mo phong xe
            # sat khung cua 4 cm). Lui vao cho DA BIET LA TRONG (kiem tra ngay duoi) chi can thoang khong giam.
            # Lui THEO CUNG ve truc khe: goc truoc nhich lai gan khung cua vai mm truoc khi ra xa — cho giam toi da 1 cm,
            # khong bao gio duoi 1.5 cm, va cuoi cung phai xa hon 1 cm (kiem tra tren DUNG cung se chay, toc do lui that).
            c0 = self._clear_now()
            ok = c0 <= self.margin_hard and ((mn >= c0 - 0.005 and en >= c0 - 0.005 and mn > 0.01)
                                             or (mn >= max(c0 - 0.01, 0.015) and en >= c0 + 0.01))
        if not ok:
            return False
        return self._reverse_ok(v, w, trail_sec=trail_sec)

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
        if along < 0.25 and abs(psi) > math.radians(60.0):
            # 05/11: "khe" NGAY CHO XE DUNG ma mui xe lech truc hon 60 do — khong phai dang chui cua (cua 0.81 m thi xe
            # nam trong mat cua lech qua ~40 do la cham khung). Do la xe dung trong loi di hep (tui giua vat chinh va
            # ke), duong ban do quay nguoc lai: mat cat ngay than xe bi coi la "cua" truc 130-165 do -> xoay vuong mat cua
            # khong duoc, lui ve truc (nguong 3 cm cua luc chui!) ve phia ke, cho... lap mai (bag 05/11 101558 339-400 s:
            # ho con 4 cm). De ban do + thoat ket quay dau xe.
            return None

        c0 = self._clear_now()

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
        crossing = lat <= tol and not backing
        if not crossing and not backing and along <= 0.0:
            # (03/11) Truc truoc DA QUA mat cat hep nhat, duoi con trong khung: di TIEP qua khe (_gap_cross tu kiem va cham,
            # tu lui ve truc neu that su ket). Lui ra canh lai luc nay chi day duoi nguoc vao khung cua (phat lai bag 15/10
            # 103136 luc 312-313 s: lech 5-6 cm so voi dung sai 5 cm -> "lui ve truc khe" giua cua).
            crossing = True
        tol_phys = 0.5 * (g["gw"] - 2.0 * self.half_width) - self.gap_cross_margin - 0.005
        mouth_try = False
        if not crossing and not backing and along < stand + 0.05 and lat <= tol_phys:
            # 03/11: gan mat khe, lech truc hon dung sai "an toan" nhung con trong gioi han VAT LY (khe - be ngang xe -
            # ho toi thieu luc chui): THU CHUI truoc neu (a) mui xe da o mieng khe, hoac (b) KHONG lui duoc (cho sau duoi
            # chua biet la trong / het han muc). _gap_cross tu kiem tra va cham tung lenh; khong co lenh an toan thi
            # moi lui ra canh lai nhu cu. Truoc day lui la cach duy nhat: xe da o mieng cua lech 6 cm (dung sai 4 cm)
            # lui ra 0.5 m roi lech them (mo phong xe sat khung cua).
            if along < self.front_len + 0.05 or not self._back_possible(psi):
                mouth_try = True
        if crossing or mouth_try:
            # DA VAO TRUC: pha qua khe lam TRUOC. No co luat rieng khi sat khung (nguong gap_cross_margin
            # 3 cm, lui ve truc, xoay cho thoang). Truoc 14/10 luat "sat khung <= margin_hard -> xoay nhe
            # ve phia rong" ben duoi chay truoc: xe da vuong mat cua, ho 4-5 cm (van du qua voi nguong
            # 3 cm) lai bi xoay ra, roi pha qua khe xoay vuong lai -> xoay qua xoay lai mai (xe that 14/10).
            self.gap_cross_time = now_
            out = self._gap_cross(g, c0)
            if out is not None:
                return out

        # DA SAT KHUNG CUA (duoi margin_hard): xoay NHE tai cho ve phia lam xe xa khung
        # cua hon — dung dieu nguoi dung lam bang tay khi xe ket (23/09). Uu tien hon lui:
        # xoay nhe chi dua duoi xe ra ngang vai cm, con lui thi di thang vao cung sau
        # ma lidar khong nhin thay (blind_sectors_deg). Khi da vao truc: chi khi pha qua khe
        # khong con lenh nao an toan.
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
        if crossing:
            return None

        # Lech truc NHIEU (> 0.25 m) ma da sat mat khe: lui theo cung khong keo ve truc duoc (|w| <= 0.4 o 0.06 m/s, cung
        # lui con bi luat "cho da biet la trong" chan) -> xe lui - tien mai roi mat nguoi (mo phong cua_ben 03/11). Khi
        # do di men theo mat khe toi truc (nhanh vao truc ben duoi) thay vi lui.
        if (((lat > tol and along < stand - 0.05) or backing) and (backing or self._back_possible(psi))
                and (backing or lat <= 0.25)):
            # Da qua gan mat khe ma con lech truc -> LUI VE TRUC roi vao lai.
            # Lui THEO CUNG toi mot diem tren truc khe phia sau (pure-pursuit cho chieu lui),
            # khong lui thang: lui thang thi van lech nguyen do, con qua it cho de vua tien
            # vua nan -> xe lai co chui, lai lui, het han muc (mo phong 29/09). Da bat dau lui
            # thi lui cho du toi cho dung (along >= stand) moi tien lai.
            # GIOI HAN quang lui: cung sau xe nam trong blind_sectors_deg nen lidar KHONG
            # THAY gi thang phia sau. Chi lui lai dung doan vua di qua.
            rx, ry = mx + nx * (stand + 0.10), my + ny * (stand + 0.10)
            kappa = 2.0 * ry / max(1e-3, rx * rx + ry * ry)
            # Toc do lui THAT v_min (-0.06; _emit chan lenh -0.12 cua ban 14/10 ve muc nay) — kiem tra va cham tren DUNG
            # cung se chay. Ban 14/10 kiem tra o -0.12 (cung thoai gap doi) ma chay o -0.06 (cung cong gap doi): bag 15/10
            # 104402 luc 275 s planner thay cung "ho 1.7-2.5 cm", xe chay cung cong hon, goc sau-trai quet vao vat.
            # Do cong giu HE SO cua ban 14/10 (w = 0.12 * kappa, tuc gap doi pure-pursuit): vao truc nhanh, it ton quang lui
            # (tinh dung pure-pursuit thi cung qua thoai, xe lui het han muc ma van lech truc — mo phong xe sat khung cua).
            v_b = max(-self.gap_cross_speed, self.v_min)
            w_b = float(np.clip(-self.gap_cross_speed * kappa, -0.40, 0.40))
            if abs(w_b) < self.min_move_w and abs(w_b) > 0.02:
                w_b = math.copysign(self.min_move_w, w_b)
            for w_try in (w_b, 0.0):
                if self._arc_ok(v_b, w_try) and self._back_take():
                    return v_b, w_try, "chui khe hep — lui ve truc khe (lech %.2fm)" % lat
            # Het han muc lui ma lech con trong gioi han VAT LY -> chui luon (03/11). Truoc day dung xoay tai cho cho han
            # muc hoan lai — mo phong: lech 5-6 cm (dung sai "an toan" 4-5 cm, vat ly 9 cm) dung cho 19 s roi moi chui.
            tol_phys = 0.5 * (g["gw"] - 2.0 * self.half_width) - self.gap_cross_margin - 0.005
            if lat <= tol_phys:
                self.gap_cross_time = now_
                out = self._gap_cross(g, c0)
                if out is not None:
                    return out
            v_des, head_des, turn_thr = 0.0, psi, math.radians(35.0)
            note = "chui khe hep — het han lui, canh goc tai cho"
        elif lat > tol:
            # (03/11) Gan mat khe ma KHONG lui duoc cung vao day: diem ngam o tam dung cho tren truc (sau xe theo
            # chieu doc truc) -> xe xoay ngang, di men theo tuong toi truc roi moi quay vao — nhu nguoi lai xe.
            # Ngam co NHIN TRUOC (pure-pursuit): diem ngam nam tren truc khe nhung o phia
            # truoc xe mot doan look, chu khong ngang hong xe. Ngam ngang hong thi lech
            # truc 13 cm cung ra lenh quay 90 do de "truot ngang" -> xe quay vong tai cho.
            look = min(0.9, max(0.45, along - stand + 0.45))
            aim = max(stand, along - look)
            ax, ay = mx + nx * aim, my + ny * aim
            head_des = math.atan2(ay, ax)
            v_des, turn_thr = min(self.v_max, 0.8 * math.hypot(ax, ay)), math.radians(35.0)
            note = "chui khe hep — vao truc giua khe (lech %.2fm)" % lat

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
            if v_try == 0.0 and w_des == 0.0:
                # 05/11: lenh RONG (0, 0) khong phai lenh: mui da huong dung diem ngam ma tien khong duoc -> truoc day
                # tra (0, 0) "vao truc giua khe" moi nhip, xe dung im mai (bag 05/11 101558 luc 226-271 s). Roi xuong
                # nhanh duoi (xa: nhuong ban do / DWA; gan: lui-xoay canh lai).
                continue
            if self._arc_ok(v_try, w_des):
                return v_try, w_des, note
        if along > stand + 0.05 and not backing:
            # (03/11) Con XA hon cho dung cho ma khong tien / xoay ve diem ngam duoc (vd. sat tuong ben kia, xoay thi
            # duoi quet tuong): de ban do + DWA dua xe lai gan nhu ban 14/10. Lui-xoay o day thi xe lui XA khoi cua
            # (mo phong cua bep co tuong trai: lui 0.6 m roi khong qua). Lui-xoay chi danh cho luc da toi mieng cua.
            return None
        # Khong tien/xoay duoc (vd goc xe da chia qua thanh cua, xoay vuong thi ep goc
        # xuong tuong ben canh): LUI-XOAY nhu lui xe vao chuong — lui nhe cho goc xe nhac
        # khoi tuong, vua lui vua be mui ve truc khe. Dung im o day thi ket mai.
        for w_try in (w_des, 0.5 * w_des, 0.0):
            v_bk = max(-self.gap_cross_speed, self.v_min)
            if self._arc_ok(v_bk, w_try) and self._back_take():
                return v_bk, w_try, note + " — lui xoay de canh lai"
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
            return bool(self._admissible_cross(mn, en, c0)) and self._reverse_ok(v, w, steps * self.horizon_dt)

        if abs(err) < math.radians(12.0):
            v_des = v_c * math.cos(err)
            # Uu tien DI THANG neu thang lot: be lai trong khe lam duoi xe (0.33 m) vang ra,
            # an mat vai cm dung o cho khong con cm nao. Lech ngang con lai nho thi cu thang.
            for v_try, w_try in ((v_des, w_des), (v_des, 0.0), (0.5 * v_des, w_des), (0.5 * v_des, 0.0)):
                if ok(v_try, w_try):
                    return v_try, w_try, "chui khe hep — qua khe theo truc " + tag
        # Xoay tai cho cho vuong mat cua: goc truoc quet NGANG ve phia khung cua — dung cho lidar do kem nhat (lidar gan
        # trung mat tuong o mieng cua, tia luot qua dau khung: do 9 cm khi that 5.9 cm). Dung le an toan thuong
        # (margin_hard, kem luat thoat ket) thay cho nguong 3 cm cua luc tien thang qua khe (03/11: mo phong xe dat san
        # sat khung cua, xoay vuong ngay -> cham khung 2/6 mam; ban 14/10 lui ra truoc vi uoc khe sai hon).
        if w_des != 0.0 and ok(0.0, w_des):
            mn_r, en_r = self._arc_clearance(0.0, w_des, steps)
            if bool(self._admissible(np.array([mn_r]), np.array([en_r]))[0]):
                return 0.0, w_des, "chui khe hep — xoay vuong mat cua " + tag
        # Khong tien, khong xoay vuong duoc: LUI VE TRUC mot doan cho co cho vao lai. Dat
        # TRUOC "xoay nhe cho thoang": o mieng cua, xoay cho thoang la quay mui TRANH thanh
        # cua gan -> lech khoi truc toi 18 do roi lai xoay ve, lac mai (mo phong 29/09).
        stand = self.front_len + self.margin_soft + 0.25
        rx, ry = mx + g["nx"] * (stand + 0.10), my + g["ny"] * (stand + 0.10)
        kappa = 2.0 * ry / max(1e-3, rx * rx + ry * ry)
        v_bk = max(-self.gap_cross_speed, self.v_min)            # toc do lui that; do cong nhu ban 14/10 (xem tren)
        w_back = float(np.clip(-self.gap_cross_speed * kappa, -0.40, 0.40))
        for w_try in (w_back, 0.0):
            if ok(v_bk, w_try) and self._back_take():
                return v_bk, w_try, "chui khe hep — lui ve truc de vao lai " + tag
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

    def _back_possible(self, psi: float = 0.0) -> bool:
        """Con lui duoc khong (han muc sau khi hoan dan + cho sau duoi da biet la trong) — KHONG tru han muc."""
        now_ = time.time()
        used = self.gap_back_used
        if now_ - self.gap_back_time > 0.3:
            used = max(0.0, used - self.back_refill * max(0.0, now_ - self.back_upd_t))
        if used >= self.gap_back_max - 1e-3:
            return False
        w_b = float(np.clip(-0.5 * psi, -0.4, 0.4))
        v_b = max(-self.gap_cross_speed, self.v_min)
        return self._arc_ok(v_b, w_b) or self._arc_ok(v_b, 0.0)

    def _back_take(self) -> bool:
        """Con han muc lui khong (va tru di mot nhip). Lidar KHONG nhin thang phia sau
        (blind_sectors_deg) nen tong quang lui bi gioi han.

        03/11: han muc HOAN DAN back_refill_mps khi khong lui. Truoc day nghi 1 s la cap lai du gap_back_max_m:
        xe lui - tien - lui lien tuc o goc ket thi moi lan lai co 0.40 m (bag 15/10: 6 lan lui trong 37 s), lui sau
        dan vao vung mu ma lidar khong thay -> dung vat phia sau."""
        now_ = time.time()
        dt_ = max(0.0, now_ - self.back_upd_t)
        self.back_upd_t = now_
        if now_ - self.gap_back_time > 0.3:      # khong dang lui -> hoan dan han muc
            self.gap_back_used = max(0.0, self.gap_back_used - self.back_refill * dt_)
        if self.gap_back_used >= self.gap_back_max:
            return False
        self.gap_back_time = now_
        self.gap_back_used += min(self.gap_cross_speed, -self.v_min) / max(1.0, self.control_hz)
        return True

    def _retrace_take(self) -> bool:
        """06/11 — LUI THEO VET o bay sau: han muc RIENG (escape_far_back_m moi dot), khong tru han muc lui chung.
        Chi dung khi ban do dang o che do nhin xa sau xe (tui chu U / hanh lang cut) va cung lui nam trong VET than xe
        da di qua (kiem tra truoc do bang _arc_ok(..., trail_sec=escape_trail_sec)). Han muc chung 0.40 m (hoan
        0.05 m/s) sinh ra de chan lui mu NHIEU LAN vao cho chua thay (bag 15/10) — lui ra theo dung duong xe vua vao thi
        khac: than xe da o do. Khong dung 5 s thi cap lai."""
        now_ = time.time()
        if now_ - self.retrace_t > 5.0:
            self.retrace_used = 0.0
        if self.retrace_used >= self.esc_far_back:
            return False
        self.retrace_t = now_
        self.retrace_used += min(self.gap_cross_speed, -self.v_min) / max(1.0, self.control_hz)
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
        v_top: Optional[float] = None,
        v_ref: Optional[float] = None,
    ) -> Optional[Tuple[float, float, float]]:
        """Tra ve (v, w, clearance) tot nhat, hoac None neu khong co lua chon an toan."""
        gx, gy = goal_xy

        # 1. Cua so dong hoc — rong bang accel * sample_window, KHONG phai accel * dt_ctrl.
        #    Xem ghi chu o tham so sample_window_sec: cua so 1-tick lam xe dung yen.
        dv = self.accel_lin * self.sample_window
        dw = self.accel_ang * self.sample_window
        vt = self.v_max if v_top is None else max(self.v_max, v_top)
        # giam toc duoc nhanh hon tang toc (decel_lin): tu 0.4 m/s phai co ngay cac quy dao cham de chon khi gap vat
        dec = self.decel_lin if self.cur_v > self.v_max + 0.01 else self.accel_lin
        v_lo = max(self.v_min if allow_reverse else 0.0, self.cur_v - dec * self.sample_window)
        v_hi = min(vt, self.cur_v + dv)
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
        if allow_reverse and self.rev_known_free and np.any(admissible & (V < 0.0)):
            # Lui chi vao cho DA BIET LA TRONG (lidar mu thang phia sau duoi xe)
            idx = np.where(admissible & (V < 0.0))[0]
            admissible[idx] = self._reverse_ok_many(V[idx], W[idx])
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
        # Che do thich ung (v_ref): tien toi dich chi duoc thuong toi muc v_ref (khong thuong chay nhanh hon nguoi) —
        # toc do do c_speed keo ve v_ref. Mac dinh nhu cu (v_max).
        max_prog = max(1e-3, (self.v_max if v_ref is None else max(0.02, v_ref)) * self.horizon_T)
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
        v_pref = (self.v_max if v_ref is None else v_ref) * scale
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
            if self.esc is not None and self.esc.get("mode") == "face":
                self.esc = None             # quay mat ve nguoi ma mat nguoi -> bo (SEARCH tu lo)
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
                    self.search_goto_lim = self.search_goto_max
                    self.occl_turning = False
                    self.avoid_side = 0
                    self.search_dir = 1.0 if self.last_seen_bearing >= 0 else -1.0
            if self.state == S_SEARCH and self.search_goto and self.esc is not None:
                # 05/11: dang thoat ket (bat dau trong pha di toi) -> lam tiep cho xong
                out_e = self._esc_step(dt)
                if out_e is not None:
                    # dang thoat ket la dang tien trien: gia han pha di toi (toi da 30 s) — het han ngay khi vua quay dau
                    # xong thi xe dung lai quet tai cho, khong di vong toi cho thay nguoi (mo phong tui hep vat cao)
                    self.search_goto_lim = max(self.search_goto_lim, min(self.search_goto_cap, (now - self.search_start) + 3.0))
                    v, w = self._emit(out_e[0], out_e[1], dt)
                    self._report(status, v, w, "mat nguoi — " + out_e[2], now)
                    return
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
                if (now - self.search_start) < self.search_goto_lim:
                    cor = self.half_width + self.margin_hard
                    blk, _bd = segment_blocked(self.obstacles, bx, by, cor)
                    if blk and self.nav_enabled and self._path_backward(self._nav_get(now, bx, by, False)):
                        blk = False         # 06/11: duong ban do quay nguoc ra sau (ngo cut) -> khong chui khe phia truoc
                    if blk:
                        saved_g = (self.gap_mem, self.gap_seen_t, self.gap_axis_prev)
                        g2 = self._gap_target(gb, math.hypot(bx, by), (bx, by))
                        gp2 = self._nav_gap_path(now, bx, by) if (self.gap_on_path and self.nav_enabled) else None
                        if g2 is not None and gp2 is not None and not self._gap_path_ok(gp2, g2, need_near=False):
                            # 05/11: duong toi cho thay cuoi di qua khe NGUOC chieu chui (xe trong tui, duong quay ra) thi
                            # bo, khong nho. Khe XA duong van nhan: vi tri du doan co the troi (nguoi re vao cua ben hong)
                            g2 = None
                            self.gap_mem, self.gap_seen_t, self.gap_axis_prev = saved_g
                        if g2 is None and gp2 is not None:
                            g2 = self._gap_on_path(gp2, (bx, by))
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
                        (now - self.search_start) < self.search_goto_lim:
                    # Duong bi chan -> ban do luoi (07/10): chon dung ben vong, dich trong vat thi dung o
                    # cho tot nhat thay vi dung im truoc vat. Dat TRUOC luat "lech > 60 do thi xoay ve phia
                    # nguoi": dang vong qua vat ma mui lech khoi cho nguoi la chuyen phai co — xoay ve thi
                    # pha hong duong vong, mui chuc vao vat (mo phong chu U 07/10).
                    if self.nav_enabled and p_blk:
                        nav = self._nav_get(now, bx, by, False)
                        if nav is not None and not nav["hold"]:
                            # 05/11: duong vong DAI (nguoi lach qua khe hep roi khuat sau vat cao, xe phai quay dau ra
                            # khoi tui roi vong ben kia) thi 8 s khong du — cho them theo do dai duong, toi da 30 s
                            self.search_goto_lim = max(self.search_goto_lim, min(
                                self.search_goto_cap, (now - self.search_start) + 2.0 + nav["cost"] / max(0.1, self.v_max)))
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
                self.esc = None
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
        if source.startswith("camera"):
            self.cam_seen_t = now
        # Camera KHONG thay nguoi > 0.6 s (bi vat CAO che: tu lanh, tu) thi ban do phai dua xe toi cho nhin thay duoc,
        # khong "dung o cho tot nhat" sau vat che (mo phong 03/11 cua co tu lanh: xe dung giua cua, nguoi di khuat).
        view_lost = (now - self.cam_seen_t) > 0.6
        self.search_goto = False
        self.search_phase = None
        self.open_tries = 0
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
        elif (goal_r <= self.dist_deadband and abs(bearing) <= self.bearing_deadband
              and not self._person_leaving(tgt, bearing)):
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
            nav = self._nav_get(now, tx, ty, spd > 0.25 or view_lost)
        else:
            self.nav_cache = None

        # ── THOAT KET (05/11) ────────────────────────────────────────────
        # Ke hoach dang chay thi lam tiep (truoc dong tac chui khe / xoay ve nguoi: no da tinh ca duong vong); toi noi /
        # ban do bao dung cho / het bi chan thi bo. Xe dung trong o vuong 25 cm qua escape_stuck_sec ma nguoi con xa
        # (khong phai luc ban do bao "dung o cho tot nhat") -> tu lap ke hoach (luoi an toan cho moi kieu ket khac).
        if self.esc is not None:
            if self.esc.get("mode") == "face":
                # 06/11 quay mat ve nguoi: xong khi da thang huong nguoi; nguoi di xa (bam binh thuong lo) hoac lai qua gan
                cancel = abs(bearing) <= self.bearing_deadband or goal_r > 0.6 or dist < 0.70
            else:
                cancel = goal_r <= self.dist_deadband or (nav is not None and nav["hold"])
            if cancel:
                self.esc = None
            else:
                out_e = self._esc_step(dt)
                if out_e is not None:
                    self.stuck_reset_t = now
                    v, w = self._emit(out_e[0], out_e[1], dt)
                    self._report(status, v, w, out_e[2], now, dist, bearing, source)
                    return
        elif self.escape_on and self._stuck(now, goal_r, nav):
            nav_s = nav if nav is not None else (self._nav_get(now, tx, ty, spd > 0.25 or view_lost)
                                                 if self.nav_enabled else None)
            if self._esc_start(nav_s, "dung im %.0f s, nguoi cach %.1f m" % (self.escape_stuck, dist)):
                out_e = self._esc_step(dt)
                if out_e is not None:
                    self.stuck_reset_t = now
                    v, w = self._emit(out_e[0], out_e[1], dt)
                    self._report(status, v, w, out_e[2], now, dist, bearing, source)
                    return

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
            if self.gap_on_path and self.nav_enabled:
                # 03/11: khe = cap tia ke nhau nhu 14/10 (_gap_pair) TRUOC, khong thay thi mat cat hep tren DUONG DI ban
                # do (_gap_on_path — thay ca khi xe da o mieng cua). Bo luat "o mieng cua thi tin khe hon ban do" va "nho
                # khe 2 s, tu lam moi" cua 14/10 (gap_at_mouth / gap_recall_mode): giu dong tac chui khe ca khi khe khong
                # nam tren duong di / da het khe (bag 15/10). Tuong nhin cheo o mieng cua gio co bo nho + noi tia nen ban
                # do khong con ve duong xuyen tuong. Chi dung ban do de TIM khe, khong doi nhanh di theo ban do ben duoi.
                gap = self._gap_pair(goal_b, goal_r, dist, tx, ty, nav, now)
                gpath = None
                if gap is None:
                    gpath = self._nav_gap_path(now, tx, ty)
                    gap = self._gap_on_path(gpath, (tx, ty))
                if gap is None:
                    # Nhip nay lo khong thay khe (o mieng cua, thanh cua luc thay luc khong) ma vua thay < 1 s truoc ->
                    # giu dong tac chui khe. Khong thi bo bam ban do chen vao giua: hai bo dieu khien lui-xoay nguoc
                    # chieu nhau, xe lech truc dan 0.07 -> 0.18 m (mo phong 03/11). Dang chui thi giu toi khi duoi ra.
                    gap = self._gap_recall((tx, ty), hold_sec=1.0)
                    # 05/11: khe nho lai cung phai nam tren duong ban do TOI TAN CHO NGUOI (duong tim khe) va dung chieu (xe
                    # da vao tui, duong quay nguoc ra mieng tui thi khong chui tiep vao trong). Khong dung duong "dung cach
                    # nguoi 1 m": nguoi o gan thi duong do ngan, dung truoc cua -> loai oan cua that (mo phong cua bep).
                    if gap is not None and not (self._gap_path_ok(gpath, gap)
                                                or (nav is not None and self._gap_path_ok(nav, gap))):
                        gap = None
            else:
                gap = self._gap_pair(goal_b, goal_r, dist, tx, ty, nav)
            if gap is not None:
                out = self._gap_maneuver(gap)
                stand_ = self.front_len + self.margin_soft + 0.25
                if out is None and gap["along"] < stand_ + 0.10 and now - self.gap_none_since < 2.0:
                    # Sat mep khe ma nhip nay dong tac chui khe CHUA co lenh an toan: DUNG cho nhip sau, khong nhuong cho
                    # bo bam ban do / xoay thoat — chung lui-xoay NGUOC chieu dong tac chui khe, xe lech truc them
                    # (mo phong 03/11 xe sat khung cua: lech 0.06 -> 0.15 m). Be tac qua 2 s moi de bo khac thu.
                    out = (0.0, 0.0, "chui khe hep — cho (chua co lenh an toan)")
                if out is not None:
                    if not out[2].endswith("(chua co lenh an toan)"):
                        self.gap_none_since = now
                    v_c, w_c, note = out
                    self.avoid_side = 0    # c_side se keo xe lech khoi truc khe
                    self.occl_turning = False
                    v, w = self._emit(v_c, w_c, dt)
                    self._report(status, v, w, note, now, dist, bearing, source)
                    if self.publish_markers:
                        self._publish_markers(gap["mx"], gap["my"], tx, ty)
                    return
            else:
                self.gap_none_since = now

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

        if goal_r <= self.dist_deadband and not self._person_leaving(tgt, bearing):
            w_cmd = 0.0
            if abs(bearing) > self.bearing_deadband:
                if self._can_turn(bearing):
                    w_cmd = float(np.clip(2.2 * bearing, -self.w_max * 0.9, self.w_max * 0.9))
                    self.creep_acc = 0.0
                    self.face_since = 0.0
                else:
                    # Xoay tai cho se quet DUOI xe (0.33 m) vao vat — thuong la vua qua cua,
                    # duoi con nam giua hai thanh cua. Nhich toi theo cung ve phia nguoi cho
                    # duoi ra khoi khung cua roi moi xoay, thay vi dung im nhin nguoi di mat.
                    # 06/11: CO GIOI HAN — khong nhich khi da cach nguoi duoi creep_min_dist_m, moi dot toi da
                    # creep_max_m. Truoc day di doc tuong (duoi vuong tuong ben kia) ma nguoi dung gan tuong thi
                    # xe bo toi cach nguoi 0.43 m, luot ngang qua nguoi roi mat camera (mo phong goc tuong).
                    if now - self.creep_t > 2.0:
                        self.creep_acc = 0.0
                    if dist > self.creep_min_d and self.creep_acc < self.creep_max:
                        res = self._dwa((0.3 * math.cos(bearing), 0.3 * math.sin(bearing)), (tx, ty), dt,
                                        allow_reverse=False, fov_active=True)
                        if res is not None and res[0] > 0.0:
                            v, w = self._emit(res[0], res[1], dt)
                            self.creep_acc += abs(v) * dt
                            self.creep_t = now
                            self._report(status, v, w, "duoi xe vuong — nhich toi de xoay ve phia nguoi",
                                         now, dist, bearing, source, res[2])
                            return
                    # Nhich toi khong duoc / khong nen nua ma nguoi lech nhieu (sap ra mep camera): tim chuoi dong
                    # tac ngan (lui / xoay / tien) de QUAY MAT ve nguoi — nhu lai xe quay dau trong ngo hep.
                    if self.escape_on and abs(bearing) > self.face_esc:
                        if self.face_since == 0.0:
                            self.face_since = now
                        if now - self.face_since >= 1.0 and self.esc is None and self._esc_start_face(
                                tx, ty, "nguoi lech %.0f do, xoay tai cho khong duoc" % math.degrees(bearing)):
                            out_e = self._esc_step(dt)
                            if out_e is not None:
                                v, w = self._emit(out_e[0], out_e[1], dt)
                                self._report(status, v, w, out_e[2], now, dist, bearing, source)
                                return
                    else:
                        self.face_since = 0.0
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
        # Toc do thich ung chi khi duong toi NGUOI thoang — xet toi cach nguoi target_clear_radius_m (chan nguoi luon nam
        # trong hanh lang toi tan tam nguoi, blocked_person vi vay gan nhu luon dung -> che do nhanh khong bao gio bat).
        # Khung cua nam giua xe va nguoi (nguoi re vao cua) van bi bat.
        r_ex = dist - self.target_clear_r
        blk_ex = (segment_blocked(self.obstacles, r_ex * tx / max(dist, 1e-6), r_ex * ty / max(dist, 1e-6),
                                  corridor)[0] if r_ex > 0.1 else False)
        v_top, v_ref = self._adaptive_speed(tgt, dist, bearing, source, blocked or blk_ex)
        if v_ref is not None:
            # dich phu: di tiep ve phia nguoi (toc do do v_ref quyet), khong dung o diem cach nguoi follow_distance
            sub_r = max(sub_r, min(reach, 0.6))
            sgx, sgy = sub_r * math.cos(phi), sub_r * math.sin(phi)
        res = self._dwa((sgx, sgy), (tx, ty), dt, allow_reverse=False, fov_active=True,
                        center_active=(not blocked and self.state != S_AVOID), v_top=v_top, v_ref=v_ref)

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
        v, w = self._emit(v_cmd, w_cmd, dt, v_top)

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
