"""
rssi_follow_node.py
====================
Subscribe /rssi/command và /rssi/confidence để điều khiển robot
BW-DR03 bằng /cmd_vel — chế độ RSSI độc lập (không cần camera).

Logic điều khiển:
  - DI_THANG   → Twist(linear.x = +linear_speed)
  - QUAY_PHAI  → Twist(angular.z = -angular_speed)
  - QUAY_TRAI  → Twist(angular.z = +angular_speed)
  - GIU_HUONG  → Twist() dừng
  - Timeout / confidence thấp → Twist() dừng

### SUA (khong co sonar, chi co LIDAR o mat truoc):
Logic khoảng cách nay lay tu LIDAR (/scan), quet 1 cung goc hep truoc mat
robot (lidar_front_center_deg ± front_window_deg), lay khoang cach GAN
NHAT trong cung do (co loc outlier don le). Nguyen tac nguong giu nguyen
y het ban sonar cu, chi doi nguon do:
  front_dist < emergency_stop_m  → DỪNG KHẨN CẤP (ưu tiên tuyệt đối)
  front_dist < target_distance_m → ở đúng khoảng cách → DỪNG tiến, giữ quay
  front_dist < slowdown_m        → giảm linear xuống slow_speed
  front_dist >= slowdown_m       → tiến bình thường

Topics subscribed:
  /rssi/command      (std_msgs/String)       — lệnh từ rssi_serial_node
  /rssi/confidence   (std_msgs/Float32)      — độ tin cậy
  /rssi/angle_deg    (std_msgs/Float32)      — góc tuyệt đối
  /scan              (sensor_msgs/LaserScan) — LIDAR, dùng để đo khoảng cách phía trước

Topics published:
  /cmd_vel                (geometry_msgs/Twist)
  /rssi_follow/status     (std_msgs/String)  — trạng thái JSON để debug

Services:
  /rssi_follow/enable   (std_srvs/Trigger)
  /rssi_follow/disable  (std_srvs/Trigger)
  /rssi_follow/stop     (std_srvs/Trigger)

Parameters:
  linear_speed        (float) — tốc độ tiến bình thường (m/s)   default: 0.15
  slow_speed          (float) — tốc độ tiến khi sắp đến (m/s)   default: 0.07
  angular_speed       (float) — tốc độ quay cố định (rad/s)     default: 0.40
  use_proportional    (bool)  — dùng kp_angular nhân góc         default: False
  kp_angular          (float) — hệ số khuếch đại góc             default: 0.018
  angle_deadband_deg  (float) — vùng chết góc (°) [KHÔNG dùng nữa cho quyết định
                                 quay/thẳng — xem angle_enter/exit_turn_deg bên dưới,
                                 giữ lại tham số này chỉ để tương thích ngược]  default: 15.0
  angle_enter_turn_deg (float) — [MỚI] phải lệch quá góc này mới BẮT ĐẦU quay
                                  (hysteresis — chống lắc trái-phải do nhiễu RSSI)
                                                                  default: 25.0
  angle_exit_turn_deg  (float) — [MỚI] phải về dưới góc này mới DỪNG quay
                                  (nhỏ hơn angle_enter_turn_deg — tạo vùng đệm)
                                                                  default: 8.0
  min_confidence      (float) — ngưỡng tin cậy tối thiểu         default: 0.10
  signal_timeout_sec  (float) — timeout mất RSSI (s)             default: 2.0
  control_hz          (float) — tần số điều khiển (Hz)           default: 10.0
  start_enabled       (bool)  — bật ngay khi khởi động           default: False
  cmd_vel_topic       (str)   — topic publish cmd_vel             default: /cmd_vel
  scan_topic          (str)   — topic LIDAR                       default: /scan
  lidar_front_center_deg (float) — góc 0° LIDAR ứng với hướng trước robot
                                 THẬT (⚠️ PHẢI TỰ HIỆU CHỈNH — xem ghi chú
                                 trong follow_person_lidar_controller.py:
                                 "Neu real frame bi xoay, doi thanh 180
                                 hoac -90/90")                     default: 0.0
  front_window_deg    (float) — nửa bề rộng cung quét phía trước (°) default: 20.0
  emergency_stop_m    (float) — dừng khẩn cấp nếu < X mét       default: 0.35
  target_distance_m   (float) — khoảng cách mục tiêu giữ (mét)  default: 0.80
  slowdown_m          (float) — bắt đầu giảm tốc khi < X mét    default: 1.20
  scan_timeout_sec    (float) — timeout LIDAR (s), bỏ qua nếu cũ  default: 0.5
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _angle_diff(a: float, b: float) -> float:
    """Hiệu góc ngắn nhất a-b (rad), xử lý wraparound đúng — copy cùng
    logic với follow_person_lidar_controller.py để nhất quán quy ước góc."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RssiFollowNode(Node):
    """Điều khiển robot theo hướng RSSI từ rssi_serial_node."""

    def __init__(self) -> None:
        super().__init__('rssi_follow_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('linear_speed',       0.15)
        self.declare_parameter('slow_speed',          0.07)
        self.declare_parameter('angular_speed',       0.40)
        self.declare_parameter('use_proportional',    False)
        self.declare_parameter('kp_angular',          0.018)
        self.declare_parameter('angle_deadband_deg',  15.0)
        # ### SUA (fix lac trai-phai): 2 nguong hysteresis thay the 1 nguong cu ###
        self.declare_parameter('angle_enter_turn_deg', 25.0)
        self.declare_parameter('angle_exit_turn_deg',   8.0)
        # ### SUA (fix lac o 2-3m): Bo loc lam muot goc RSSI truoc khi dua vao
        # hysteresis — EMA (Exponential Moving Average) voi alpha tu 0.0 den 1.0.
        # alpha = 1.0 => khong loc (tin ngay), alpha = 0.1 => loc cuc manh (cham).
        # Gia tri 0.25 la can bang tot: muot ma nhung van phan ung nhanh khi re.
        self.declare_parameter('angle_ema_alpha',      0.45)
        self.declare_parameter('min_confidence',      0.15)
        self.declare_parameter('signal_timeout_sec',  2.0)
        self.declare_parameter('control_hz',          10.0)
        self.declare_parameter('start_enabled',       False)
        self.declare_parameter('cmd_vel_topic',       '/cmd_vel')
        # ### SUA (khong co sonar, dung LIDAR /scan de do khoang cach truoc mat) ###
        self.declare_parameter('scan_topic',            '/scan')
        self.declare_parameter('lidar_front_center_deg', 0.0)
        self.declare_parameter('front_window_deg',      20.0)
        self.declare_parameter('emergency_stop_m',    0.35)
        self.declare_parameter('target_distance_m',   0.80)
        self.declare_parameter('slowdown_m',          1.20)
        self.declare_parameter('scan_timeout_sec',    0.5)

        self._linear_speed      = float(self.get_parameter('linear_speed').value)
        self._slow_speed        = float(self.get_parameter('slow_speed').value)
        self._angular_speed     = float(self.get_parameter('angular_speed').value)
        self._use_proportional  = bool(self.get_parameter('use_proportional').value)
        self._kp_angular        = float(self.get_parameter('kp_angular').value)
        self._deadband          = float(self.get_parameter('angle_deadband_deg').value)
        # ### SUA: doc 2 nguong hysteresis moi ###
        self._angle_enter       = float(self.get_parameter('angle_enter_turn_deg').value)
        self._angle_exit        = float(self.get_parameter('angle_exit_turn_deg').value)
        self._ema_alpha         = float(self.get_parameter('angle_ema_alpha').value)
        self._min_confidence    = float(self.get_parameter('min_confidence').value)
        self._signal_timeout    = float(self.get_parameter('signal_timeout_sec').value)
        self._control_hz        = float(self.get_parameter('control_hz').value)
        self._enabled           = bool(self.get_parameter('start_enabled').value)
        self._cmd_vel_topic     = str(self.get_parameter('cmd_vel_topic').value)
        # ### SUA: doc tham so LIDAR thay sonar ###
        self._scan_topic        = str(self.get_parameter('scan_topic').value)
        self._lidar_front_center = math.radians(
            float(self.get_parameter('lidar_front_center_deg').value))
        self._front_window      = math.radians(
            float(self.get_parameter('front_window_deg').value))
        self._emergency_stop_m  = float(self.get_parameter('emergency_stop_m').value)
        self._target_dist_m     = float(self.get_parameter('target_distance_m').value)
        self._slowdown_m        = float(self.get_parameter('slowdown_m').value)
        self._scan_timeout      = float(self.get_parameter('scan_timeout_sec').value)

        # ── State ────────────────────────────────────────────────────────
        self._last_cmd: Optional[str]   = None   # "DI_THANG" | "QUAY_PHAI" | ...
        # ### SUA: trang thai hysteresis rieng, KHONG bi lat theo tung goi tin RSSI ###
        self._turn_state: str           = 'NONE'  # 'NONE' | 'RIGHT' | 'LEFT'
        self._last_conf: float          = 0.0
        self._last_angle: float         = 0.0     # góc thô từ NodeMCU
        self._filtered_angle: float     = 0.0     # góc đã lọc EMA (dùng để điều khiển)
        self._last_rx_time: float       = 0.0
        self._last_log_time: float      = 0.0
        # LIDAR state (### SUA: thay cho sonar) ###
        self._last_scan: Optional[LaserScan] = None
        self._last_scan_time: float     = 0.0

        # ── Publishers / Subscribers / Services ─────────────────────────
        self._cmd_pub    = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._status_pub = self.create_publisher(String, '/rssi_follow/status', 10)

        self.create_subscription(String,  '/rssi/command',    self._cmd_cb,   10)
        self.create_subscription(Float32, '/rssi/confidence', self._conf_cb,  10)
        self.create_subscription(Float32, '/rssi/angle_deg',  self._angle_cb, 10)
        self.create_subscription(
            LaserScan, self._scan_topic, self._scan_cb,
            rclpy.qos.qos_profile_sensor_data)

        self.create_service(Trigger, '/rssi_follow/enable',  self._enable_cb)
        self.create_service(Trigger, '/rssi_follow/disable', self._disable_cb)
        self.create_service(Trigger, '/rssi_follow/stop',    self._stop_cb)

        # ── Control timer ────────────────────────────────────────────────
        period = 1.0 / max(1.0, self._control_hz)
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'rssi_follow_node ready | enabled={self._enabled} | '
            f'linear={self._linear_speed} m/s | angular={self._angular_speed} rad/s | '
            f'topic={self._cmd_vel_topic}')
        self.get_logger().info(
            f'LIDAR (thay sonar): emergency={self._emergency_stop_m}m | '
            f'target={self._target_dist_m}m | slowdown={self._slowdown_m}m | '
            f'scan_topic={self._scan_topic} | '
            f'front_center={math.degrees(self._lidar_front_center):.0f}° '
            f'±{math.degrees(self._front_window):.0f}° '
            f'(⚠️ HIỆU CHỈNH lidar_front_center_deg nếu robot không đi thẳng '
            f'đúng hướng người dù RSSI báo góc ~0°)')
        self.get_logger().info(
            f'### SUA: Hysteresis chong lac — enter={self._angle_enter}° '
            f'exit={self._angle_exit}° (thay the deadband cu {self._deadband}°)')
        self.get_logger().info(
            'Bật robot theo RSSI: ros2 service call /rssi_follow/enable std_srvs/srv/Trigger')

    # ────────────────────────────────────────────────────────────────────
    # Subscribers
    # ────────────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: String) -> None:
        self._last_cmd    = msg.data.strip()
        self._last_rx_time = time.time()

    def _conf_cb(self, msg: Float32) -> None:
        self._last_conf = float(msg.data)

    def _angle_cb(self, msg: Float32) -> None:
        raw = float(msg.data)
        self._last_angle = raw
        # ### SUA (fix lac o 2-3m): Loc EMA de lam muot goc truoc khi dieu khien.
        # Cong thuc: filtered = alpha * raw + (1-alpha) * filtered_cu
        # alpha nho => muot hon, phan ung cham hon (tot cho nhieu xa)
        # alpha lon => nhay hon, nhung de bi nhieu
        self._filtered_angle = (self._ema_alpha * raw
                                + (1.0 - self._ema_alpha) * self._filtered_angle)

    def _scan_cb(self, msg: LaserScan) -> None:
        """### SUA: nhận LaserScan thay vì mảng sonar — chỉ lưu message,
        việc trích khoảng cách phía trước thực hiện trong _front_distance()."""
        self._last_scan = msg
        self._last_scan_time = time.time()

    # ────────────────────────────────────────────────────────────────────
    # Services
    # ────────────────────────────────────────────────────────────────────

    def _enable_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._enabled = True
        res.success = True
        res.message = 'RSSI follow ENABLED'
        self.get_logger().info(res.message)
        return res

    def _disable_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._enabled = False
        self._turn_state = 'NONE'
        self._filtered_angle = 0.0
        self._publish_stop()
        res.success = True
        res.message = 'RSSI follow DISABLED'
        self.get_logger().info(res.message)
        return res

    def _stop_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._enabled = False
        self._turn_state = 'NONE'
        self._filtered_angle = 0.0
        self._publish_stop(force=True)
        res.success = True
        res.message = 'Emergency STOP — RSSI follow disabled'
        self.get_logger().warn(res.message)
        return res

    # ────────────────────────────────────────────────────────────────────
    # Control loop
    # ────────────────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        now = time.time()

        # Kiểm tra timeout tín hiệu RSSI
        signal_age = now - self._last_rx_time if self._last_rx_time > 0.0 else 999.0
        signal_ok  = (signal_age <= self._signal_timeout)

        # Kiểm tra độ tin cậy
        conf_ok = (self._last_conf >= self._min_confidence)

        # Trạng thái controller
        if not self._enabled:
            state = 'disabled'
            self._publish_stop()

        elif not signal_ok:
            state = f'signal_timeout_{signal_age:.1f}s'
            self._publish_stop()
            if now - self._last_log_time > 1.0:
                self.get_logger().warn(
                    f'Mất tín hiệu RSSI ({signal_age:.1f}s) — dừng robot!')
                self._last_log_time = now

        elif not conf_ok:
            state = f'low_confidence_{self._last_conf:.2f}'
            self._publish_stop()

        else:
            raw_cmd = self._last_cmd or 'GIU_HUONG'
            if raw_cmd == 'GIU_HUONG':
                cmd = 'GIU_HUONG'
            else:
                cmd = self._update_turn_state()
            state, twist = self._compute_motion(cmd, now)
            self._cmd_pub.publish(twist)

        # Publish status để debug
        scan_age = now - self._last_scan_time if self._last_scan_time > 0.0 else 999.0
        scan_ok  = scan_age <= self._scan_timeout
        front_dist_for_status = self._front_distance(now)
        status = {
            'enabled':         self._enabled,
            'state':           state,
            'signal_ok':       signal_ok,
            'signal_age_s':    round(signal_age, 2),
            'confidence':      round(self._last_conf, 3),
            'angle_deg':       round(self._last_angle, 1),
            'filtered_angle_deg': round(self._filtered_angle, 1),
            'last_cmd':        self._last_cmd,
            'front_dist_m':    round(front_dist_for_status, 3) if front_dist_for_status is not None else None,
            'scan_ok':         scan_ok,
            'target_dist_m':   self._target_dist_m,
            'emergency_stop_m': self._emergency_stop_m,
        }
        self._status_pub.publish(String(data=json.dumps(status)))

        # Log định kỳ
        if now - self._last_log_time > 1.0 and self._enabled and signal_ok and conf_ok:
            dist_str = f'{front_dist_for_status:.2f}m' if front_dist_for_status is not None else 'N/A'
            self.get_logger().info(
                f'state={state} | raw_angle={self._last_angle:.1f}° '
                f'filtered={self._filtered_angle:.1f}° '
                f'conf={self._last_conf:.2f} | cmd={self._last_cmd} | '
                f'front_dist={dist_str}')
            self._last_log_time = now

    # ────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────

    def _front_distance(self, now: float) -> Optional[float]:
        """Trích khoảng cách gần nhất trong 1 cung góc hẹp phía trước
        robot từ LIDAR."""
        return self._sector_min_distance(
            self._lidar_front_center, self._front_window, now)

    def _sector_min_distance(self, center_rad: float,
                             half_width_rad: float,
                             now: float) -> Optional[float]:
        """Khoảng cách gần nhất (robust) trong sector [center ± half_width].
        Bỏ 1 điểm min dị thường nếu đủ nhiều tia — tránh phantom brake."""
        scan = self._last_scan
        if scan is None or not scan.ranges:
            return None

        age = now - self._last_scan_time
        if age > self._scan_timeout:
            return None

        values = []
        for i, r in enumerate(scan.ranges):
            r = float(r)
            if not math.isfinite(r) or not (scan.range_min <= r <= scan.range_max):
                continue
            a = scan.angle_min + i * scan.angle_increment
            if abs(_angle_diff(a, center_rad)) > half_width_rad:
                continue
            values.append(r)

        if not values:
            return None

        values.sort()
        idx = 1 if len(values) >= 8 else 0
        return values[idx]

    def _compute_motion(self, cmd: str, now: float):
        """Tính Twist + state string, có xét khoảng cách phía trước (LIDAR).

        Thứ tự ưu tiên:
          1. Emergency stop (front_dist < emergency_stop_m)
          2. Đúng khoảng cách (front_dist < target_distance_m) → không tiến
          3. Vùng giảm tốc  (front_dist < slowdown_m)         → tiến chậm
          4. Bình thường                                       → tiến tốc độ đầy đủ
        """
        front_dist = self._front_distance(now)
        angular_z = self._calc_angular()

        # ── 1. EMERGENCY STOP ──────────────────────────────────────────
        if front_dist is not None and front_dist < self._emergency_stop_m:
            if now - self._last_log_time > 0.5:
                self.get_logger().warn(
                    f'EMERGENCY STOP: front_dist={front_dist:.2f}m < {self._emergency_stop_m}m!')
            return 'emergency_stop_lidar', Twist()

        # ── Angular (quay) — không phụ thuộc khoảng cách ─────────────
        if cmd in ('QUAY_PHAI', 'QUAY_TRAI'):
            t = Twist()
            t.angular.z = angular_z
            return f'turning_{cmd}', t

        if cmd == 'GIU_HUONG':
            return 'hold_heading', Twist()

        # ── cmd == 'DI_THANG': áp dụng logic khoảng cách LIDAR ───────
        if front_dist is not None:
            # 2. Đúng khoảng cách → dừng tiến, giữ thẳng hướng
            if front_dist <= self._target_dist_m:
                return f'hold_distance_{front_dist:.2f}m', Twist()

            # 3. Vùng giảm tốc → tiến chậm
            if front_dist < self._slowdown_m:
                t = Twist()
                t.linear.x = self._slow_speed
                return f'slowdown_{front_dist:.2f}m', t
        else:
            # ### SUA: canh bao ro rang khi KHONG co du lieu LIDAR — day la
            # tinh huong nguy hiem, robot se tien khong dung theo khoang cach
            if now - self._last_log_time > 1.0:
                self.get_logger().warn(
                    'KHONG CO DU LIEU LIDAR (/scan) — robot dang tien ma '
                    'KHONG co gioi han khoang cach nao! Kiem tra scan_topic.')

        # 4. Tiến bình thường (có LIDAR nhưng còn xa, hoặc thiếu LIDAR)
        t = Twist()
        t.linear.x = self._linear_speed
        state = 'moving_forward_NO_LIDAR' if front_dist is None else f'moving_forward_{front_dist:.2f}m'
        return state, t

    def _update_turn_state(self) -> str:
        """### SUA (fix lac trai-phai): Hysteresis kieu Schmitt trigger.

        Dung self._last_angle (lien tuc, do) thay vi tin chuoi cmd roi rac
        tu firmware. Hai nguong khac nhau cho viec BAT DAU va DUNG quay:

          - Dang DUNG YEN (NONE): phai lech qua angle_enter (vd 25 do) moi
            bat dau quay.
          - Dang QUAY (RIGHT/LEFT): phai ve duoi angle_exit (vd 8 do) moi
            duoc coi la da thang, DUNG quay.
          - Chi doi chieu quay (RIGHT<->LEFT) truc tiep neu goc vot han
            sang nguong doi dien — khong doi chieu vi mot cu nhieu nho.

        Nho vung dem rong + khong doi xung nay, nhieu RSSI dao dong quanh
        0 do se KHONG con kich hoat lat trai/phai lien tuc nhu truoc.
        """
        # ### SUA: dung goc DA LOC thay vi goc tho de tranh nhieu o 2-3m ###
        angle = self._filtered_angle

        if self._turn_state == 'NONE':
            if angle > self._angle_enter:
                self._turn_state = 'RIGHT'
            elif angle < -self._angle_enter:
                self._turn_state = 'LEFT'
        else:
            if abs(angle) < self._angle_exit:
                self._turn_state = 'NONE'
            elif self._turn_state == 'RIGHT' and angle < -self._angle_enter:
                self._turn_state = 'LEFT'
            elif self._turn_state == 'LEFT' and angle > self._angle_enter:
                self._turn_state = 'RIGHT'

        if self._turn_state == 'RIGHT':
            return 'QUAY_PHAI'
        if self._turn_state == 'LEFT':
            return 'QUAY_TRAI'
        return 'DI_THANG'

    def _calc_angular(self) -> float:
        """Tính angular.z từ trạng thái hysteresis đã khóa (không đọc lại
        cmd thô — self._turn_state được _update_turn_state() set trước đó
        trong cùng chu kỳ điều khiển)."""
        if self._turn_state == 'RIGHT':
            if self._use_proportional:
                return _clamp(
                    -self._kp_angular * self._filtered_angle,
                    -self._angular_speed, -0.05)
            return -self._angular_speed
        if self._turn_state == 'LEFT':
            if self._use_proportional:
                return _clamp(
                    -self._kp_angular * self._filtered_angle,
                    0.05, self._angular_speed)
            return +self._angular_speed
        return 0.0

    def _publish_stop(self, force: bool = False) -> None:
        stop = Twist()
        self._cmd_pub.publish(stop)
        if force:
            # Gửi 3 lần để chắc chắn robot nhận
            for _ in range(2):
                self._cmd_pub.publish(stop)

    # ────────────────────────────────────────────────────────────────────
    # Cleanup
    # ────────────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        try:
            self._publish_stop(force=True)
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RssiFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
