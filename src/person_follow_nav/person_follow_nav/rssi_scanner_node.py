"""
rssi_scanner_node.py
====================
Doc 3 board quet RSSI (ESP32 cam thang USB, firmware robot_rssi/src/scanner) va phat MAU THO len ROS.
Node nay CHI doc cong — khong loc, khong uoc luong (viec do cua rssi_bearing_node), khong bao gio ghi /cmd_vel.

Tach doc cong khoi uoc luong de: ghi rosbag /rssi/raw + /odom roi phat lai chinh thuat toan ma khong can xe.

Topics published:
  /rssi/raw     std_msgs/String (JSON)  {"stamp": t, "samples": [[id, rssi, t_mau], ...]}  — chi phat khi co mau moi
  /rssi/status  std_msgs/String (JSON)  1 Hz: tung board {hz, age_sec, dbm, port}, beacon_age_sec, ports{cong: trang thai}

Parameters:
  ports   danh sach cong serial (mang chuoi PHANG, hoac mot chuoi cach nhau dau phay). Dung /dev/serial/by-path/...
          — by-id cua cac board va LiDAR TRUNG nhau (cung chip CH340). Board tu xung A/B/C nen thu tu khong quan trong.
"""

from __future__ import annotations

import json
import queue
import threading
import time
from typing import Any, Dict, List

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .rssi_io import BoardReader


class RssiScannerNode(Node):

    def __init__(self) -> None:
        super().__init__("rssi_scanner_node")
        self._declare_params()
        self._read_params()

        self.q: "queue.Queue" = queue.Queue()
        self.stop = threading.Event()
        self.port_state: Dict[str, str] = {p: "chua mo" for p in self.ports}
        self.port_board: Dict[str, str] = {}
        self.recent: Dict[str, List[tuple]] = {}        # board -> [(t, rssi)] trong 2 s gan day
        self.last_t: Dict[str, float] = {}
        self.warned_dup = False
        self.lock = threading.Lock()

        self.pub_raw = self.create_publisher(String, self.raw_topic, 50)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)

        if not self.ports:
            self.get_logger().error(
                "Chua khai bao cong: -p ports:=\"/dev/serial/by-path/AAA,/dev/serial/by-path/BBB,...\"")
        self.readers = [BoardReader(p, self.baudrate, self._on_sample, self._on_event, self.stop)
                        for p in self.ports]
        for r in self.readers:
            r.start()

        self.create_timer(1.0 / max(1.0, self.publish_hz), self._publish_raw)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(f"rssi_scanner: doc {len(self.ports)} cong @ {self.baudrate} baud -> {self.raw_topic}")

    # ─────────────────────────────────────────────────────────────────────
    def _declare_params(self) -> None:
        d: Dict[str, Any] = {
            "ports": [""],
            "baudrate": 115200,
            "raw_topic": "/rssi/raw",
            "status_topic": "/rssi/status",
            "publish_hz": 20.0,
            "silent_warn_sec": 5.0,
        }
        # dynamic_typing: cho phep ports la mang chuoi HOAC mot chuoi "a,b,c" (launch arg), so nguyen hoac so thuc
        dyn = ParameterDescriptor(dynamic_typing=True)
        for k, v in d.items():
            self.declare_parameter(k, v, dyn)

    def _read_params(self) -> None:
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        raw = g("ports")
        if isinstance(raw, str):
            raw = raw.split(",")
        self.ports = [str(p).strip() for p in (raw or []) if str(p).strip()]
        self.baudrate = int(g("baudrate"))
        self.raw_topic = str(g("raw_topic"))
        self.status_topic = str(g("status_topic"))
        self.publish_hz = float(g("publish_hz"))
        self.silent_warn = float(g("silent_warn_sec"))

    # ── goi tu luong doc cong ────────────────────────────────────────────
    def _on_sample(self, t: float, board: str, rssi: int, seq: int, port: str) -> None:
        self.q.put((t, board, rssi, port))

    def _on_event(self, port: str, text: str) -> None:
        with self.lock:
            self.port_state[port] = text
        if not rclpy.ok():
            return
        log = self.get_logger().info if text.startswith(("da mo", "board")) else self.get_logger().warn
        log(f"{port}: {text}")

    # ─────────────────────────────────────────────────────────────────────
    def _publish_raw(self) -> None:
        samples = []
        while True:
            try:
                t, board, rssi, port = self.q.get_nowait()
            except queue.Empty:
                break
            samples.append([board, rssi, round(t, 4)])
            self.last_t[board] = t
            self.recent.setdefault(board, []).append((t, rssi))
            prev = self.port_board.get(port)
            if prev is None:
                if board in self.port_board.values() and not self.warned_dup:
                    self.warned_dup = True
                    self.get_logger().error(f"Hai cong cung xung la board {board} — nap nham env (scan_a/b/c)?")
                self.port_board[port] = board
        if samples:
            self.pub_raw.publish(String(data=json.dumps({"stamp": round(time.time(), 4), "samples": samples})))

    def _publish_status(self) -> None:
        now = time.time()
        boards = {}
        for board, lst in self.recent.items():
            lst[:] = [(t, r) for t, r in lst if t >= now - 2.0]
            boards[board] = {
                "hz": round(len(lst) / 2.0, 1),
                "age_sec": round(now - self.last_t.get(board, 0.0), 2),
                "dbm": round(sum(r for _, r in lst) / len(lst), 1) if lst else None,
                "port": next((p for p, b in self.port_board.items() if b == board), ""),
            }
        ages = [b["age_sec"] for b in boards.values()]
        with self.lock:
            ports = dict(self.port_state)
        out = {"stamp": round(now, 3), "boards": boards,
               "beacon_age_sec": round(min(ages), 2) if ages else None, "ports": ports}
        self.pub_status.publish(String(data=json.dumps(out)))
        if boards and min(ages) > self.silent_warn:
            self.get_logger().warn(
                f"Khong thay beacon {min(ages):.0f} s — beacon tat / het pin / qua xa?", throttle_duration_sec=10.0)

    def destroy_node(self) -> None:
        self.stop.set()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RssiScannerNode()
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
