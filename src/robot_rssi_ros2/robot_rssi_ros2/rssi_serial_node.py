"""
rssi_serial_node.py
====================
Đọc dữ liệu từ NodeMCU Master qua cổng Serial USB, parse JSON và publish
lên ROS2 topics.

NodeMCU in ra Serial các dòng như:
  JSON:{"angle":12.3,"conf":0.28,"cmd":"DI_THANG","rssi_a":-55.0,...}

Node này chỉ quan tâm đến dòng bắt đầu bằng "JSON:" — các dòng debug
khác sẽ bị bỏ qua.

Topics published:
  /rssi/direction  (std_msgs/String)  — JSON đầy đủ (relay)
  /rssi/angle_deg  (std_msgs/Float32) — góc hướng tính được (độ)
  /rssi/confidence (std_msgs/Float32) — độ tin cậy [0, 1]
  /rssi/command    (std_msgs/String)  — "DI_THANG" | "QUAY_PHAI" | "QUAY_TRAI" | "GIU_HUONG"

Parameters:
  port          (str)   — cổng Serial, vd /dev/ttyUSB0   (default: /dev/ttyUSB0)
  baudrate      (int)   — tốc độ baud                     (default: 115200)
  signal_timeout_sec (float) — sau bao lâu không nhận thì báo mất tín hiệu (default: 3.0)
"""

from __future__ import annotations

import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    import serial
except ImportError:
    serial = None  # type: ignore


class RssiSerialNode(Node):
    """Đọc Serial từ NodeMCU, parse JSON, publish ROS2 topics."""

    def __init__(self) -> None:
        super().__init__('rssi_serial_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('signal_timeout_sec', 3.0)

        self._port = str(self.get_parameter('port').value)
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._timeout = float(self.get_parameter('signal_timeout_sec').value)

        # ── Publishers ───────────────────────────────────────────────────
        self._pub_dir    = self.create_publisher(String,  '/rssi/direction',  10)
        self._pub_angle  = self.create_publisher(Float32, '/rssi/angle_deg',  10)
        self._pub_conf   = self.create_publisher(Float32, '/rssi/confidence', 10)
        self._pub_cmd    = self.create_publisher(String,  '/rssi/command',    10)

        # ── State ────────────────────────────────────────────────────────
        self._last_rx_time: float = 0.0
        self._ser: Optional[object] = None  # serial.Serial instance

        # ── Watchdog timer: kiểm tra timeout mỗi 1s ─────────────────────
        self.create_timer(1.0, self._watchdog_cb)

        # ── Mở Serial & bắt đầu thread đọc ─────────────────────────────
        if serial is None:
            self.get_logger().error(
                'pyserial chưa cài! Chạy: pip install pyserial')
            return

        self._open_serial()
        if self._ser is not None:
            self._reader_thread = threading.Thread(
                target=self._read_loop, daemon=True)
            self._reader_thread.start()

    # ────────────────────────────────────────────────────────────────────
    # Serial
    # ────────────────────────────────────────────────────────────────────

    def _open_serial(self) -> None:
        try:
            self._ser = serial.Serial(          # type: ignore[union-attr]
                port=self._port,
                baudrate=self._baudrate,
                timeout=1.0,
            )
            self.get_logger().info(
                f'Đã mở Serial {self._port} @ {self._baudrate} baud')
        except Exception as exc:
            self.get_logger().error(
                f'Không mở được Serial {self._port}: {exc}\n'
                f'  → Kiểm tra cổng: ls /dev/ttyUSB* /dev/ttyACM*\n'
                f'  → Hoặc truyền tham số: ros2 run robot_rssi_ros2 '
                f'rssi_serial_node --ros-args -p port:=/dev/ttyACM0')
            self._ser = None

    def _read_loop(self) -> None:
        """Thread đọc liên tục từ Serial; parse dòng JSON."""
        ser = self._ser
        while rclpy.ok():
            try:
                # Tránh tích lũy buffer gây trễ (Serial Lag).
                # Mỗi dòng JSON dài khoảng 130 bytes. Nếu tích lũy > 250 bytes,
                # ta xóa sạch để luôn đọc dữ liệu thời gian thực mới nhất.
                if ser.in_waiting > 250:
                    ser.reset_input_buffer()

                raw = ser.readline()            # type: ignore[union-attr]
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='replace').strip()
            except Exception as exc:
                self.get_logger().warn(f'Serial read error: {exc}')
                time.sleep(0.1)
                continue

            # Chỉ xử lý dòng do firmware đánh dấu "JSON:"
            if not line.startswith('JSON:'):
                continue

            json_str = line[5:]  # bỏ prefix "JSON:"
            try:
                data = json.loads(json_str)
            except json.JSONDecodeError as exc:
                self.get_logger().warn(f'JSON parse error: {exc} | raw={json_str!r}')
                continue

            self._last_rx_time = time.time()
            self._publish(data, json_str)

    # ────────────────────────────────────────────────────────────────────
    # Publish
    # ────────────────────────────────────────────────────────────────────

    def _publish(self, data: dict, raw_json: str) -> None:
        angle   = float(data.get('angle', 0.0))
        conf    = float(data.get('conf',  0.0))
        cmd     = str(data.get('cmd', 'GIU_HUONG'))

        self._pub_dir.publish(String(data=raw_json))
        self._pub_angle.publish(Float32(data=angle))
        self._pub_conf.publish(Float32(data=conf))
        self._pub_cmd.publish(String(data=cmd))

        self.get_logger().debug(
            f'RSSI → angle={angle:.1f}° conf={conf:.2f} cmd={cmd}')

    # ────────────────────────────────────────────────────────────────────
    # Watchdog
    # ────────────────────────────────────────────────────────────────────

    def _watchdog_cb(self) -> None:
        if self._last_rx_time == 0.0:
            return  # chưa nhận được gì lần nào
        age = time.time() - self._last_rx_time
        if age > self._timeout:
            self.get_logger().warn(
                f'RSSI signal timeout! Không nhận dữ liệu trong {age:.1f}s '
                f'(ngưỡng: {self._timeout}s). '
                f'Kiểm tra NodeMCU đã bật và Beacon đang phát BLE.')

    # ────────────────────────────────────────────────────────────────────
    # Cleanup
    # ────────────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()           # type: ignore[union-attr]
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RssiSerialNode()
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
