#!/usr/bin/env python3
"""watch_follow.py — theo doi xe khi chay that, 1 dong moi 0.5 s (08/10).

CHI DOC topic: khong publish, khong goi service -> chay luc nao cung an toan, khong anh huong xe.
Chay khi run_full.sh / follow_nav_real.launch.py dang chay o terminal khac:

    python3 watch_follow.py            # in man hinh + ghi run_logs/watch_<ngay_gio>.txt
    python3 watch_follow.py --geom     # them cot hinh hoc camera (buoc B0: cam_dist_m so voi distance_m)
    python3 watch_follow.py --no-log   # khong ghi file

Moi dong:
  giay  STATE (* = dang bat bam, > = vua doi trang thai)  lenh /cmd_vel that (v m/s, w rad/s)
  | nguon tracker  khoang cach  goc (do, trai duong)  [--geom: kc tu camera (tu dau/chan), chieu cao nguoi, lech goc ngua]
  | camera: trang thai, so khung/s, tre (ms)
  | rssi: dBm/mau-moi-giay tung board A B C; "quet N" khi xe dang xoay do; "huong +N" khi do xong (so voi mui xe)
  | canh bao (topic cu / thieu / tan so thap)  | note cua planner
"""
import argparse
import json
import sys
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

WS = Path(__file__).resolve().parents[3]

# topic -> (kieu, qos, nguong "cu" (s), ten ngan, tan so toi thieu)
TOPICS = {
    "/scan": (LaserScan, "sensor", 0.6, "scan", 8.0),
    "/odom": (Odometry, "sensor", 0.5, "odom", 15.0),
    "/cmd_vel": (Twist, 10, 0.3, "cmd_vel", 10.0),          # watchdog driver 1 s; planner 15 Hz (CLAUDE.md muc 12)
    "/person_reid/target": (String, 10, 1.0, "camera", 0.0),
    "/follow/target": (String, 10, 0.5, "tracker", 0.0),
    "/follow/planner_status": (String, 10, 0.5, "planner", 0.0),
    "/rssi/status": (String, 10, 2.5, "rssi", 0.0),           # 1 Hz
    "/rssi/bearing": (String, 10, 1.0, "rssi_bearing", 0.0),  # 5 Hz
}
OPTIONAL = ("/rssi/status", "/rssi/bearing")   # khong chay RSSI (--no-rssi) thi khong canh bao


def fmt(x, spec, dash="-"):
    if x is None:
        return dash
    try:
        return format(float(x), spec)
    except (TypeError, ValueError):
        return dash


class Watch(Node):
    def __init__(self, geom, log_f, period):
        super().__init__("watch_follow")
        self.geom = geom
        self.log_f = log_f
        self.color = sys.stdout.isatty()
        self.t0 = time.time()
        self.last = {}
        self.hist = {t: deque() for t in TOPICS}
        self.data = {}
        self.cmd = None
        self.prev_state = None
        self.n = 0
        for topic, (typ, qos, _, _, _) in TOPICS.items():
            q = qos_profile_sensor_data if qos == "sensor" else qos
            self.create_subscription(typ, topic, lambda m, t=topic: self._cb(t, m), q)
        self.create_timer(period, self._tick)

    def _cb(self, topic, msg):
        now = time.time()
        self.last[topic] = now
        self.hist[topic].append(now)
        if isinstance(msg, String):
            try:
                self.data[topic] = json.loads(msg.data)
            except ValueError:
                pass
        elif isinstance(msg, Twist):
            self.cmd = (msg.linear.x, msg.angular.z)

    def _out(self, line, plain=None):
        print(line, flush=True)
        if self.log_f is not None:
            self.log_f.write((plain if plain is not None else line) + "\n")
            self.log_f.flush()

    def _tick(self):
        now = time.time()
        for h in self.hist.values():
            while h and h[0] < now - 2.0:
                h.popleft()
        hz = {t: len(h) / 2.0 for t, h in self.hist.items()}
        ps = self.data.get("/follow/planner_status", {})
        tg = self.data.get("/follow/target", {})
        cam = self.data.get("/person_reid/target", {})
        rs = self.data.get("/rssi/status")
        rb = self.data.get("/rssi/bearing")

        if self.n % 25 == 0:
            hdr = ("    giay STATE      lenh v/w      | nguon tracker  kc    goc  "
                   + ("| kc-cam (tu)  cao  lech-ngua " if self.geom else "")
                   + "| camera trang thai  Hz  tre | rssi dBm/mau-s | canh bao | note")
            self._out(hdr)
        self.n += 1

        state = str(ps.get("state", "?"))
        mark = ">" if (self.prev_state is not None and state != self.prev_state) else " "
        self.prev_state = state
        en = "*" if ps.get("enabled") else " "
        v, w = self.cmd if self.cmd is not None else (ps.get("cmd_v"), ps.get("cmd_w"))
        parts = [f"{mark}{now - self.t0:7.1f} {state:<8}{en} v{fmt(v, '+.2f')} w{fmt(w, '+.2f')}"]

        src = str(tg.get("source", "-"))
        parts.append(f"| {src:<13} {fmt(tg.get('distance_m'), '.2f')}m {fmt(tg.get('bearing_deg'), '+4.0f')}d")
        if self.geom:
            parts.append(f"| {fmt(tg.get('cam_dist_m'), '.2f')}m ({str(tg.get('cam_dist_from') or '-'):<7}) "
                         f"{fmt(tg.get('person_height_m'), '.2f')}m {fmt(tg.get('elev_bias_deg'), '+.1f')}d")

        cst = str(cam.get("status", "-"))[:17]
        parts.append(f"| {cst:<17} {hz['/person_reid/target']:4.1f} {fmt(cam.get('latency_ms'), '3.0f')}ms")

        if rs is None:
            parts.append("| rssi -")
        else:
            boards = rs.get("boards", {}) or {}
            s = " ".join(f"{k}{fmt(boards[k].get('dbm'), '.0f')}/{fmt(boards[k].get('hz'), '.0f')}"
                         for k in sorted(boards)) or "chua thay board"
            if rb:
                if rb.get("valid"):
                    s += f" huong {fmt(rb.get('bearing_base_deg'), '+.0f')}d"
                elif (rb.get("swept_deg") or 0) > 20:
                    s += f" quet {fmt(rb.get('swept_deg'), '.0f')}"
                if not rb.get("beacon_ok"):
                    s += " MAT-BEACON"
            parts.append("| " + s)

        warn = []
        for topic, (_, _, stale, short, min_hz) in TOPICS.items():
            if topic not in self.last:
                if topic not in OPTIONAL:
                    warn.append(f"{short}:chua-co")
                continue
            age = now - self.last[topic]
            if age > stale:
                warn.append(f"{short}:cu-{age:.1f}s")
            elif min_hz > 0 and now - self.t0 > 3.0 and hz[topic] < min_hz:
                warn.append(f"{short}:{hz[topic]:.0f}Hz")
        wtxt = " ".join(warn) if warn else "-"
        note = str(ps.get("note", ""))[:80]

        plain = " ".join(parts) + f" | {wtxt} | {note}"
        if self.color and warn:
            line = " ".join(parts) + f" | \033[91m{wtxt}\033[0m | {note}"
        elif self.color and mark == ">":
            line = "\033[1m" + plain + "\033[0m"
        else:
            line = plain
        self._out(line, plain)


def main():
    ap = argparse.ArgumentParser(description="Theo doi xe khi chay that (chi doc topic)")
    ap.add_argument("--geom", action="store_true", help="them cot hinh hoc camera (kc tu camera, chieu cao, lech ngua)")
    ap.add_argument("--no-log", action="store_true", help="khong ghi file run_logs/watch_*.txt")
    ap.add_argument("--period", type=float, default=0.5, help="chu ky in (s), mac dinh 0.5")
    args = ap.parse_args()

    log_f = None
    if not args.no_log:
        d = WS / "run_logs"
        try:
            d.mkdir(exist_ok=True)
        except OSError:
            d = Path.cwd()
        path = d / time.strftime("watch_%m%d_%H%M%S.txt")
        log_f = open(path, "w", encoding="utf-8")
        print(f"ghi log: {path}", flush=True)

    rclpy.init()
    node = Watch(args.geom, log_f, args.period)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if log_f is not None:
            log_f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
