#!/usr/bin/env python3
"""
test_web.py — kiem tra robot_web KHONG can xe (can ROS: source install/setup.bash truoc)
=========================================================================================
Chay trong mien ROS rieng (ROS_DOMAIN_ID 77, chi localhost) -> khong lenh nao toi duoc xe that.
Dung mot "robot gia" (topic + service gia) va mot run_full.sh GIA trong thu muc tam, chay node web that
(ros2 run robot_web web_server) roi bam tung nut qua HTTP:
  - dang nhap: chua dang nhap -> 401; sai mat khau / thieu header X-Robot-Web -> 403
  - Kiem tra / Khoi dong / Tat he thong: doc dung OK / CANH BAO / HONG; co HONG thi KHONG khoi dong;
    Tat he thong goi /follow/stop TRUOC roi moi Ctrl-C (cach >= 0.45 s); nhan ra va tat duoc he thong bat ngoai web
  - luat nut: chua hoc nguoi / dang hoc -> khong bat bam; dang bam -> khong bat lai, khong hoc, khong xoa;
    0 hoac 2 node ghi /cmd_vel, LiDAR / odom / camera ngung -> khong bat bam; planner khac dang chay -> khong
    khoi dong; DUNG luon chay, KHONG can dang nhap, mat /follow/stop thi dung /follow/disable
  - anh camera: JPEG rong 480; video MJPEG >= 12 hinh/s; toi da 3 nguoi xem; dong trang -> thoi nhan anh; mat anh -> 204
  - ca 3 node (robot_web, _probe, _video) khong publish topic nao; CPU khi mo trang va khi xem video
  - ghi bag (run_full.sh --bag gia): 1 nut bat / dung, dem topic, dung luong, danh sach bag; tu ghi khi Khoi dong
    va tu dung SAU khi he thong tat; loi ghi; nhan ra va dung duoc "ros2 bag record" ngoai web
  - dong su kien: he thong, nut bam, xe bat / ngung bam, hoc xong, dot "tim nguoi" (kem thoi gian, gop lap lai),
    cam bien ngung, 2 node ghi /cmd_vel, pin may (sysfs gia) + pin khung xe (/bw_dr03/power); ghi vao file nhat ky
Ket qua: dong cuoi "=> DAT" hoac "=> LOI".

  cd ~/ros2_reid_from_github && source install/setup.bash
  python3 src/robot_web/scripts/test_web.py
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "77"                         # mien rieng, chi trong may nay
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image, LaserScan  # noqa: E402
from std_msgs.msg import Float32, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

try:
    import cv2
except ImportError:
    cv2 = None

PORT = 18765
PASSWORD = "246810"
RESULTS: list = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    RESULTS.append((name, bool(ok), detail))
    print(f"  {'DAT' if ok else 'LOI'}  {name}" + (f"   ({detail})" if detail else ""), flush=True)
    return bool(ok)


def wait_for(fn, timeout: float, step: float = 0.1):
    t_end = time.time() + timeout
    while time.time() < t_end:
        try:
            v = fn()
        except Exception:
            v = None
        if v:
            return v
        time.sleep(step)
    return None


# ── robot gia ────────────────────────────────────────────────────────────────
class FakeRobot(Node):
    SRV = ["/follow/enable", "/follow/disable", "/follow/stop", "/person_reid/start_enroll",
           "/person_reid/finish_enroll", "/person_reid/reset"]

    def __init__(self) -> None:
        super().__init__("fake_robot")
        self.lock = threading.Lock()
        self.on = {"planner": False, "identity": False, "scan": False, "odom": False, "image": False, "power": False}
        self.plan_state = None             # ep trang thai planner khi dang bam (vd. "SEARCH"); None = FOLLOW
        self.power_v = 12.34
        self.want_cmd = 0
        self.cmd_pubs: list = []
        self.want_srv = {s: True for s in self.SRV}
        self.srv = {}
        self.enabled = False
        self.id_status, self.ready = "WAITING_FOR_ENROLLMENT", False
        self.calls: list = []
        self.p_planner = self.create_publisher(String, "/follow/planner_status", 10)
        self.p_id = self.create_publisher(String, "/person_reid/target", 10)
        self.p_img = self.create_publisher(Image, "/person_reid/debug_image", 2)
        self.p_scan = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.p_odom = self.create_publisher(Odometry, "/odom", 20)
        self.p_power = self.create_publisher(Float32, "/bw_dr03/power", 10)
        for s in self.SRV:
            self.srv[s] = self.create_service(Trigger, s, self._make_cb(s))
        img = np.zeros((480, 640, 3), np.uint8)
        img[:, :, 1] = np.linspace(0, 255, 640, dtype=np.uint8)[None, :]
        self.img_bytes = img.tobytes()
        self.k = 0
        # tan so nhu xe that: planner / camera / lenh 15 Hz, LiDAR 10 Hz, odom 50 Hz, anh 15 Hz
        self.create_timer(0.05, self._tick)
        self.create_timer(1 / 15, self._pub15)
        self.create_timer(0.1, self._pub_scan)
        self.create_timer(0.02, self._pub_odom)

    def _make_cb(self, name: str):
        def cb(req, res):
            with self.lock:
                self.calls.append((name, time.time()))
                if name == "/follow/enable":
                    self.enabled = True
                elif name in ("/follow/disable", "/follow/stop"):
                    self.enabled = False
                elif name == "/person_reid/start_enroll":
                    self.id_status, self.ready = "ENROLLING", False
                elif name == "/person_reid/finish_enroll":
                    self.id_status, self.ready = "ENROLLMENT_DONE", True
                elif name == "/person_reid/reset":
                    self.id_status, self.ready = "WAITING_FOR_ENROLLMENT", False
            res.success, res.message = True, "gia: " + name
            return res
        return cb

    def called(self, name: str):
        with self.lock:
            return [t for n, t in self.calls if n == name]

    def _tick(self) -> None:
        self.k += 1
        with self.lock:
            want_cmd, want_srv = self.want_cmd, dict(self.want_srv)
        # them / bot publisher /cmd_vel va service ngay trong luong executor (an toan)
        while len(self.cmd_pubs) < want_cmd:
            self.cmd_pubs.append(self.create_publisher(Twist, "/cmd_vel", 10))
        while len(self.cmd_pubs) > want_cmd:
            self.destroy_publisher(self.cmd_pubs.pop())
        for s, w in want_srv.items():
            if not w and self.srv.get(s) is not None:
                self.destroy_service(self.srv[s])
                self.srv[s] = None

    def _pub15(self) -> None:
        with self.lock:
            on = dict(self.on)
            enabled, st, ready, ps = self.enabled, self.id_status, self.ready, self.plan_state
        if on["planner"]:
            self.p_planner.publish(String(data=json.dumps({
                "enabled": enabled, "state": (ps or "FOLLOW") if enabled else "IDLE", "note": "gia",
                "target_distance_m": 1.2, "target_source": "camera+lidar"})))
        if on["identity"]:
            self.p_id.publish(String(data=json.dumps({
                "status": st, "identity_ready": ready, "enrolled_samples": 80 if ready else 10,
                "enrollment_progress": 1.0 if ready else 0.12, "target_found": ready, "num_tracks": 1})))
        for p in self.cmd_pubs:
            p.publish(Twist())
        if on["image"]:
            m = Image()
            m.height, m.width, m.encoding, m.step = 480, 640, "bgr8", 1920
            m.data = self.img_bytes
            self.p_img.publish(m)

    def _pub_scan(self) -> None:
        if self.on["scan"]:
            m = LaserScan()
            m.angle_min, m.angle_max, m.angle_increment = 0.0, 6.2657, 0.01745
            m.range_min, m.range_max = 0.1, 10.0
            m.ranges = [2.0] * 360
            self.p_scan.publish(m)

    def _pub_odom(self) -> None:
        if self.on["odom"]:
            self.p_odom.publish(Odometry())
        if self.on["power"] and self.k % 2 == 0:
            self.p_power.publish(Float32(data=float(self.power_v)))

    def set(self, **kw) -> None:
        with self.lock:
            for k, v in kw.items():
                if k == "cmd":
                    self.want_cmd = int(v)
                else:
                    self.on[k] = bool(v)

    def drop_service(self, name: str) -> None:
        with self.lock:
            self.want_srv[name] = False


# ── HTTP ─────────────────────────────────────────────────────────────────────
class Http:
    def __init__(self, base: str) -> None:
        self.base, self.cookie = base, ""

    def _do(self, rq: urllib.request.Request, timeout: float):
        if self.cookie:
            rq.add_header("Cookie", self.cookie)
        try:
            with urllib.request.urlopen(rq, timeout=timeout) as r:
                return r.status, r.read(), r.headers
        except urllib.error.HTTPError as e:
            return e.code, e.read(), e.headers

    def get(self, path: str, timeout: float = 4.0):
        return self._do(urllib.request.Request(self.base + path), timeout)

    def post(self, path: str, body=None, header: bool = True, timeout: float = 10.0):
        h = {"Content-Type": "application/json"}
        if header:
            h["X-Robot-Web"] = "1"
        rq = urllib.request.Request(self.base + path, data=json.dumps(body or {}).encode(), headers=h, method="POST")
        return self._do(rq, timeout)

    def status(self):
        code, body, _ = self.get("/api/status")
        return json.loads(body) if code == 200 else None

    def act(self, name: str, body=None):
        code, b, _ = self.post("/api/action/" + name, body)
        try:
            j = json.loads(b)
        except ValueError:
            j = {"ok": False, "msg": f"HTTP {code}"}
        return code, j


def open_mjpeg(h: "Http"):
    """Mo luong video, tra (ket noi, ma HTTP, response) — chua doc noi dung."""
    import http.client
    from urllib.parse import urlparse
    u = urlparse(h.base)
    conn = http.client.HTTPConnection(u.hostname, u.port, timeout=5)
    conn.request("GET", "/api/camera.mjpg", headers={"Cookie": h.cookie})
    r = conn.getresponse()
    return conn, r.status, r


def read_mjpeg(h: "Http", sec: float):
    """Doc luong video sec giay: (ma HTTP, so hinh, tap be ngang, hinh/s)."""
    conn, code, r = open_mjpeg(h)
    if code != 200:
        r.close()
        conn.close()
        return code, 0, set(), 0.0
    buf, n, widths, t0, t_first = b"", 0, set(), time.time(), None
    while time.time() - t0 < sec:
        chunk = r.read1(65536)
        if not chunk:
            break
        buf += chunk
        while True:
            a = buf.find(b"\xff\xd8")
            b = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
            if a < 0 or b < 0:
                break
            im = cv2.imdecode(np.frombuffer(buf[a:b + 2], np.uint8), cv2.IMREAD_COLOR) if cv2 is not None else None
            if im is not None:
                n += 1
                widths.add(im.shape[1])
                t_first = t_first or time.time()
            buf = buf[b + 2:]
    r.close()                      # dong ca response: conn.close() mot minh thi socket van mo (fp giu tham chieu)
    conn.close()
    dur = time.time() - (t_first or t0)
    return 200, n, widths, ((n - 1) / dur if n > 1 and dur > 0 else 0.0)


FAKE_RUN_FULL = r'''#!/usr/bin/env bash
# run_full.sh GIA cho test_web.py — in ket qua kiem tra kieu ban that, roi chay mot "launch" gia toi khi Ctrl-C
D="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
EV="$D/events.txt"
G=$'\033[92m'; Y=$'\033[93m'; R=$'\033[91m'; X=$'\033[0m'
if [ "$1" = "--bag" ]; then
  OUT="$D/bags/follow_$(date +%m%d_%H%M%S)"
  mkdir -p "$D/bags"
  echo "Ghi bag -> $OUT (Ctrl-C de dung; khong ghi anh camera)"
  # dong duoi chi de web doc danh sach topic (lenh ":" khong lam gi)
  : ros2 bag record -o "$OUT" --topics /scan /odom /cmd_vel \
    /follow/planner_status
  exec python3 "$D/fake_bag.py" "$OUT" "$EV"
fi
echo "1. File cong thiet bi + workspace"
echo "  ${G}OK${X}    /tmp/gia/rssi_env.sh"
echo "4. Cong thiet bi"
echo "  ${Y}CANH BAO${X}  khong thay beacon (gia)"
if [ -f "$D/hong.flag" ]; then
  echo "  ${R}HONG${X}  PL = /dev/gia KHONG ton tai (LiDAR)"
  echo "${R} CO MUC HONG — sua truoc khi chay.${X}"
  exit 1
fi
echo "${G} KIEM TRA DAT.${X}"
[ "$1" = "--check" ] && exit 0
echo "ARGS $*" >> "$EV"
echo "ros2 launch person_follow_nav follow_nav_real.launch.py lidar_port:=/dev/gia"
exec python3 -c '
import signal, sys, time
ev = sys.argv[1]
def bye(*a):
    open(ev, "a").write("SIGINT %.3f\n" % time.time())
    sys.exit(0)
signal.signal(signal.SIGINT, bye)
open(ev, "a").write("LAUNCH %.3f\n" % time.time())
while True:
    print("[gia] dang chay", flush=True)
    time.sleep(0.5)
' "$EV"
'''

FAKE_BAG = r'''
import os, signal, sys, time
out, ev = sys.argv[1], sys.argv[2]
if os.path.exists(os.path.join(os.path.dirname(ev), "bagfail.flag")):
    print("[ERROR] [rosbag2_recorder]: Output folder already exists (gia)", flush=True)
    sys.exit(1)
os.makedirs(out)
t0 = time.time()
def bye(*a):
    with open(os.path.join(out, "metadata.yaml"), "w") as f:
        f.write("rosbag2_bagfile_information:\n  version: 9\n  storage_identifier: mcap\n"
                "  duration:\n    nanoseconds: %d\n  starting_time:\n    nanoseconds_since_epoch: %d\n"
                "  message_count: 321\n  topics_with_message_count:\n    - topic_metadata:\n        name: /scan\n"
                "      message_count: 300\n" % (int((time.time() - t0) * 1e9), int(t0 * 1e9)))
    open(ev, "a").write("BAG_SIGINT %.3f\n" % time.time())
    sys.exit(0)
signal.signal(signal.SIGINT, bye)
print("stdin is not a terminal device. Keyboard handling disabled.[INFO] [1] [rosbag2_recorder]: Listening for topics...", flush=True)
for t in ("/scan", "/odom", "/cmd_vel"):
    print("[INFO] [1] [rosbag2_recorder]: Subscribed to topic '%s'" % t, flush=True)
while True:
    with open(os.path.join(out, os.path.basename(out) + "_0.mcap"), "ab") as f:
        f.write(b"x" * 20000)
    time.sleep(0.2)
'''

EXT_BAG = ("import signal, sys, time\nev = sys.argv[1]\n"
           "def bye(*a):\n    open(ev, 'a').write('EXTBAG_SIGINT %.3f\\n' % time.time())\n    sys.exit(0)\n"
           "signal.signal(signal.SIGINT, bye)\nwhile True:\n    time.sleep(0.2)\n")

EXT_CODE = ("import signal, sys, time\nev = sys.argv[1]\n"
            "def bye(*a):\n    open(ev, 'a').write('EXT_SIGINT %.3f\\n' % time.time())\n    sys.exit(0)\n"
            "signal.signal(signal.SIGINT, bye)\nwhile True:\n    time.sleep(0.2)\n")


def events(ws: Path):
    p = ws / "events.txt"
    return p.read_text().splitlines() if p.exists() else []


def get_events(h: "Http") -> list:
    code, body, _ = h.get("/api/events")
    return json.loads(body)["items"] if code == 200 else []


def find_ev(items: list, code: str, **kw):
    """Su kien dau tien cung loai co cac truong a[k] == v."""
    for e in items:
        if e["code"] == code and all(e["a"].get(k) == v for k, v in kw.items()):
            return e
    return None


def closed(e):
    return e if e is not None and "end" in e else None


def in_order(want: list, got: list) -> bool:
    """want la day con (giu thu tu) cua got."""
    it = iter(got)
    return all(any(g == w for g in it) for w in want)


def write_bat(ps: Path, pct: int, status: str) -> None:
    """sysfs pin gia (BAT0 + ADP0) cho tham so power_supply_dir."""
    b, a = ps / "BAT0", ps / "ADP0"
    b.mkdir(parents=True, exist_ok=True)
    a.mkdir(parents=True, exist_ok=True)
    for k, v in (("type", "Battery"), ("present", "1"), ("capacity", str(pct)), ("status", status),
                 ("energy_now", str(pct * 600000)), ("energy_full", "60000000"), ("power_now", "20000000")):
        (b / k).write_text(v + "\n")
    (a / "type").write_text("Mains\n")
    (a / "online").write_text("0\n" if status == "Discharging" else "1\n")


def group_cpu(pgid: int) -> float:
    """Tong thoi gian CPU (s) cua moi tien trinh trong nhom."""
    tot, hz = 0.0, os.sysconf("SC_CLK_TCK")
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            s = open(f"/proc/{d}/stat").read()
        except OSError:
            continue
        f = s[s.rfind(")") + 2:].split()
        if int(f[2]) == pgid:            # truong 5 (pgrp) sau "pid (comm)"
            tot += (int(f[11]) + int(f[12])) / hz
    return tot


def main() -> int:
    tmp = Path(tempfile.mkdtemp(prefix="test_web_"))
    ws = tmp / "ws"
    scr = ws / "src" / "person_follow_nav" / "scripts"
    scr.mkdir(parents=True)
    (scr / "run_full.sh").write_text(FAKE_RUN_FULL)
    (scr / "run_full.sh").chmod(0o755)
    (ws / "fake_bag.py").write_text(FAKE_BAG)
    pwf = tmp / "robot_web.conf"
    pwf.write_text(json.dumps({"password": PASSWORD}))
    ps = tmp / "power_supply"
    write_bat(ps, 15, "Discharging")              # pin may < 20 % ngay tu dau -> su kien canh bao

    rclpy.init()
    fake = FakeRobot()
    ex = SingleThreadedExecutor()
    ex.add_node(fake)
    stop_spin = threading.Event()

    def spin() -> None:              # vong co co dung: ex.spin() trong luong daemon lam tien trinh test chet luc thoat
        while not stop_spin.is_set():
            ex.spin_once(timeout_sec=0.1)
    th_spin = threading.Thread(target=spin, daemon=True)
    th_spin.start()

    log = open(tmp / "web.log", "w")
    web = subprocess.Popen(["ros2", "run", "robot_web", "web_server", "--ros-args",
                            "-p", f"port:={PORT}", "-p", f"ws_dir:={ws}", "-p", f"password_file:={pwf}",
                            "-p", "stop_wait_sec:=0.5", "-p", f"bag_dir:={ws / 'bags'}",
                            "-p", f"power_supply_dir:={ps}", "-p", "chassis_power_unit:=V",
                            "-p", "chassis_power_warn:=11.8", "-p", "chassis_power_bad:=11.3"],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    h = Http(f"http://127.0.0.1:{PORT}")
    ext = ext_bag = None
    try:
        print("\n1. Khoi dong node web + dang nhap")
        up = wait_for(lambda: h.get("/")[0] == 200, 20.0, 0.3)
        check("trang web len (GET /)", up is not None)
        if not up:
            print(open(tmp / "web.log").read()[-3000:])
            return 1
        check("chua dang nhap -> 401", h.get("/api/status")[0] == 401)
        check("sai mat khau -> 403", h.post("/api/login", {"password": "000000"})[0] == 403)
        check("thieu header X-Robot-Web -> 403", h.post("/api/login", {"password": PASSWORD}, header=False)[0] == 403)
        code, _, hdr = h.post("/api/login", {"password": PASSWORD})
        tok = (hdr.get("Set-Cookie") or "").split(";")[0]
        check("dung mat khau -> 200 + cookie", code == 200 and tok.startswith("rw_token="))
        check("chua dang nhap thi khong bam duoc nut -> 401", h.act("follow_enable")[0] == 401)
        h.cookie = tok
        check("da dang nhap -> doc duoc trang thai", h.status() is not None)
        c, j = h.act("bag_start")
        check("Ghi bag khi he thong chua chay -> bi chan", not j["ok"] and "chua chay" in j["msg"], j["msg"])

        print("\n2. Kiem tra / Khoi dong / Tat he thong (run_full.sh gia) + tu ghi bag + su kien khi dang chay")
        fake.set(scan=True, odom=True, cmd=1)        # co san truoc khi RUNNING (nhu xe that: planner ghi /cmd_vel)
        c, j = h.act("check")
        check("Kiem tra: bat dau", j["ok"], j["msg"])
        s = wait_for(lambda: (lambda x: x if x["system"]["state"] != "CHECKING" else None)(h.status()), 10.0)
        lv = [it["level"] for it in (s or {}).get("check", [])]
        check("Kiem tra: doc dung OK + CANH BAO, khong HONG", "OK" in lv and "CANH BAO" in lv and "HONG" not in lv, str(lv))
        check("Kiem tra: bao 'kiem tra dat'", s is not None and "kiem tra dat" in s["system"]["detail"],
              s and s["system"]["detail"])

        (ws / "hong.flag").write_text("1")
        c, j = h.act("start", {"rssi": True})
        s = wait_for(lambda: (lambda x: x if x["system"]["state"] == "FAILED" else None)(h.status()), 10.0)
        lv = [it["level"] for it in (s or {}).get("check", [])]
        check("co muc HONG -> KHONG khoi dong (FAILED)", s is not None and "HONG" in lv, s and s["system"]["detail"])
        check("co muc HONG -> launch gia khong chay", not any(e.startswith("LAUNCH") for e in events(ws)))
        (ws / "hong.flag").unlink()

        c, j = h.act("start", {"rssi": False, "bag": True})
        check("Khoi dong (kem tu ghi bag): bat dau", j["ok"] and "tu ghi bag" in j["msg"], j["msg"])
        s = wait_for(lambda: (lambda x: x if x["system"]["state"] == "STARTING" else None)(h.status()), 10.0)
        check("Khoi dong: qua kiem tra -> STARTING", s is not None)
        check("Khoi dong: launch chay, co --no-rssi", wait_for(lambda: any(e.startswith("LAUNCH") for e in events(ws)), 5.0)
              and any("--no-rssi" in e for e in events(ws)), str(events(ws)))
        s = wait_for(lambda: (lambda x: x if x["bag"]["state"] == "RECORDING" and x["bag"]["name"] else None)(h.status()), 6.0)
        check("tu ghi bag ngay khi qua kiem tra", s is not None and s["bag"]["auto"], s and str(s["bag"]))
        fake.set(planner=True)
        check("planner gui trang thai -> RUNNING",
              wait_for(lambda: h.status()["system"]["state"] == "RUNNING", 6.0) is not None)
        c, j = h.act("start")
        check("dang chay -> khong khoi dong them", not j["ok"], j["msg"])
        time.sleep(1.5)                                # web thay /scan tu luc RUNNING
        fake.set(scan=False)
        e = wait_for(lambda: find_ev(get_events(h), "stale", what="scan"), 5.0)
        check("LiDAR ngung gui khi dang chay -> su kien 'ngung gui' (dang mo)", e is not None and "end" not in e, str(e))
        time.sleep(1.0)
        fake.set(scan=True)
        e = wait_for(lambda: closed(find_ev(get_events(h), "stale", what="scan")), 5.0)
        d = (e["end"] - e["t"]) if e else None
        check("LiDAR gui lai -> su kien dong (next=ok), co thoi gian", e is not None and e["a"].get("next") == "ok"
              and 1.0 < d < 5.0, f"{d:.1f} s" if d is not None else "khong thay")
        fake.set(cmd=2)
        e = wait_for(lambda: find_ev(get_events(h), "cmdpubs", n=2), 6.0)
        check("2 node ghi /cmd_vel khi dang chay -> su kien", e is not None, str(e))
        fake.set(cmd=1)
        check("ve 1 node ghi /cmd_vel -> su kien dong",
              wait_for(lambda: closed(find_ev(get_events(h), "cmdpubs", n=2)), 6.0) is not None)
        n_stop0 = len(fake.called("/follow/stop"))
        c, j = h.act("shutdown")
        check("Tat he thong: bat dau", j["ok"], j["msg"])
        s = wait_for(lambda: (lambda x: x if x["system"]["state"] == "STOPPED" else None)(h.status()), 15.0)
        check("Tat he thong -> STOPPED", s is not None, s and s["system"]["detail"])
        stops = fake.called("/follow/stop")[n_stop0:]
        sig = [float(e.split()[1]) for e in events(ws) if e.startswith("SIGINT")]
        gap = (sig[0] - stops[0]) if (stops and sig) else None
        check("Tat he thong: /follow/stop TRUOC Ctrl-C, cach >= 0.45 s", gap is not None and gap >= 0.45,
              f"cach {gap:.2f} s" if gap is not None else f"stop={stops} sigint={sig}")
        s = wait_for(lambda: (lambda x: x if x["bag"]["state"] == "OFF" else None)(h.status()), 8.0)
        bsig = [float(e.split()[1]) for e in events(ws) if e.startswith("BAG_SIGINT")]
        bgap = (bsig[0] - sig[0]) if (bsig and sig) else None
        check("tat he thong -> tu dung ghi bag SAU khi he thong tat (>= 1.5 s)", s is not None and bgap is not None
              and bgap >= 1.5, f"cach {bgap:.2f} s" if bgap is not None else str(events(ws)))
        it = get_events(h)
        e = find_ev(it, "bag", ev="done")
        check("bag tu ghi: su kien 'da luu' doc metadata.yaml, ly do 'he thong da tat'",
              e is not None and e["lvl"] == "ok" and e["a"].get("why") == "he thong da tat" and (e["a"].get("dur") or 0) > 3
              and e["a"].get("auto") is None and e["a"].get("topics") == 3, str(e))
        e = find_ev(it, "bag", ev="start")
        check("su kien 'bat dau ghi bag' co ten + tu dong", e is not None and e["a"].get("auto") is True
              and str(e["a"].get("name", "")).startswith("follow_"), str(e))
        seq = [x["a"]["state"] for x in it if x["code"] == "sys"]
        check("su kien he thong dung thu tu (kiem tra, loi, khoi dong, chay, tat)", in_order(
            ["CHECKING", "STOPPED", "CHECKING", "FAILED", "CHECKING", "STARTING", "RUNNING", "STOPPING", "STOPPED"], seq),
            " ".join(seq))
        fake.set(planner=False, scan=False, odom=False, cmd=0)

        ext = subprocess.Popen([sys.executable, "-c", EXT_CODE, str(ws / "events.txt"), "launch",
                                "follow_nav_real.launch.py"], start_new_session=True)
        s = wait_for(lambda: (lambda x: x if x["system"]["external"] else None)(h.status()), 8.0)
        check("nhan ra he thong bat ngoai web", s is not None)
        c, j = h.act("shutdown")
        s = wait_for(lambda: (lambda x: x if x["system"]["state"] == "STOPPED" else None)(h.status()), 15.0)
        check("tat duoc he thong bat ngoai web",
              s is not None and any(e.startswith("EXT_SIGINT") for e in events(ws)) and ext.poll() is not None)

        print("\n3. Luat cac nut (robot gia dang chay) + su kien hoc / bam / tim nguoi")
        fake.set(planner=True, identity=True, scan=True, odom=True, image=True, cmd=1, power=True)

        def ready_view():
            x = h.status()
            sen = x["sensors"]
            ok = (x["planner"]["age"] is not None and x["planner"]["age"] < 1 and x["identity"]["age"] is not None
                  and sen["scan_age"] is not None and sen["scan_age"] < 0.5 and sen["cmd_pubs"] == 1)
            return x if ok else None
        check("web thay du topic robot gia", wait_for(ready_view, 8.0) is not None)
        c, j = h.act("start")
        check("planner khac dang chay -> khong khoi dong", not j["ok"] and "planner" in j["msg"], j["msg"])
        n_en = len(fake.called("/follow/enable"))
        c, j = h.act("follow_enable")
        check("chua hoc nguoi -> khong bat bam", not j["ok"] and "chua hoc" in j["msg"]
              and len(fake.called("/follow/enable")) == n_en, j["msg"])
        c, j = h.act("enroll_finish")
        check("chua hoc -> nut Xong bi chan", not j["ok"], j["msg"])
        c, j = h.act("enroll_start")
        check("Hoc nguoi -> goi /person_reid/start_enroll", j["ok"] and fake.called("/person_reid/start_enroll"), j["msg"])
        wait_for(lambda: h.status()["identity"]["status"] == "ENROLLING", 3.0)
        c, j = h.act("follow_enable")
        check("dang hoc -> khong bat bam", not j["ok"] and "dang hoc" in j["msg"], j["msg"])
        c, j = h.act("enroll_finish")
        check("Xong -> goi /person_reid/finish_enroll", j["ok"], j["msg"])
        wait_for(lambda: h.status()["identity"]["ready"], 3.0)
        e = wait_for(lambda: find_ev(get_events(h), "enroll", ev="done"), 3.0)
        check("hoc xong -> su kien 'hoc xong nguoi' (so mau)", e is not None and e["a"].get("samples") == 80, str(e))
        c, j = h.act("follow_enable")
        check("da hoc + du cam bien -> bat bam", j["ok"] and len(fake.called("/follow/enable")) == n_en + 1, j["msg"])
        wait_for(lambda: h.status()["planner"]["enabled"], 3.0)
        check("bat bam -> su kien 'xe bat dau bam'", wait_for(lambda: find_ev(get_events(h), "follow", on=True), 3.0) is not None)
        fake.plan_state = "SEARCH"
        time.sleep(2.5)
        fake.plan_state = None
        e = wait_for(lambda: closed(find_ev(get_events(h), "plan", state="SEARCH")), 4.0)
        d = (e["end"] - e["t"]) if e else None
        check("planner SEARCH 2.5 s -> MOT dot 'tim nguoi' ~2.5 s, ket thuc -> FOLLOW",
              e is not None and e["a"].get("next") == "FOLLOW" and 1.5 < d < 3.8, f"{d:.1f} s" if d is not None else "khong thay")
        time.sleep(1.0)
        fake.plan_state = "SEARCH"
        time.sleep(1.5)
        fake.plan_state = None
        time.sleep(2.0)
        srch = [x for x in get_events(h) if x["code"] == "plan" and x["a"].get("state") == "SEARCH"]
        check("SEARCH lap lai sau < 4 s -> gop vao dot cu (1 dong, n=2)",
              len(srch) == 1 and srch[0].get("n") == 2 and "end" in srch[0], str(srch))
        for name, word in (("follow_enable", "dang bam"), ("enroll_start", "dang bam"), ("reset_identity", "dang bam")):
            c, j = h.act(name)
            check(f"dang bam -> {name} bi chan", not j["ok"] and word in j["msg"], j["msg"])
        c, j = h.act("follow_disable")
        check("Tam dung -> goi /follow/disable", j["ok"], j["msg"])
        wait_for(lambda: not h.status()["planner"]["enabled"], 3.0)
        check("tam dung -> su kien 'xe ngung bam'", wait_for(lambda: find_ev(get_events(h), "follow", on=False), 3.0) is not None)

        for kw, want, word in ((dict(cmd=2), 2, "2 node"), (dict(cmd=0), 0, "khong thay node")):
            fake.set(**kw)
            wait_for(lambda: h.status()["sensors"]["cmd_pubs"] == want, 6.0)
            c, j = h.act("follow_enable")
            check(f"{want} node ghi /cmd_vel -> khong bat bam", not j["ok"] and word in j["msg"], j["msg"])
        fake.set(cmd=1)
        wait_for(lambda: h.status()["sensors"]["cmd_pubs"] == 1, 6.0)
        for key, word in (("scan", "LiDAR"), ("odom", "odom"), ("identity", "camera")):
            fake.set(**{key: False})
            time.sleep(2.3 if key == "identity" else 1.0)
            c, j = h.act("follow_enable")
            check(f"{key} ngung -> khong bat bam", not j["ok"] and word in j["msg"], j["msg"])
            fake.set(**{key: True})
            time.sleep(0.6)
        wait_for(lambda: h.status()["identity"]["age"] < 1.0, 3.0)
        c, j = h.act("follow_enable")
        check("cam bien tro lai -> bat bam duoc", j["ok"], j["msg"])

        print("\n4. Ghi bag bang 1 nut (planner dang chay)")
        c, j = h.act("bag_start")
        check("Ghi: bat dau", j["ok"], j["msg"])
        s = wait_for(lambda: (lambda x: x if x["bag"]["name"] and len(x["bag"]["topics"] or []) >= 3 else None)(h.status()), 5.0)
        b = (s or {}).get("bag", {})
        check("Ghi: doc ten bag + danh sach topic tu run_full.sh + 3/4 topic da dang ky",
              s is not None and b["name"].startswith("follow_") and not b["auto"]
              and b["expected"] == ["/scan", "/odom", "/cmd_vel", "/follow/planner_status"]
              and sorted(b["topics"]) == ["/cmd_vel", "/odom", "/scan"], str(b))
        time.sleep(2.5)
        b1 = h.status()["bag"]
        time.sleep(2.2)
        b2 = h.status()["bag"]
        check("Ghi: dung luong + thoi gian tang dan", (b2["size"] or 0) > (b1["size"] or 0) > 0 and b2["dur"] > b1["dur"],
              f"{b1['size']} -> {b2['size']} B, {b1['dur']} -> {b2['dur']} s")
        c, j = h.act("bag_start")
        check("dang ghi -> khong ghi them", not j["ok"] and "dang ghi" in j["msg"], j["msg"])
        name = b2["name"]
        c, j = h.act("bag_stop")
        check("Dung ghi: bat dau", j["ok"], j["msg"])
        s = wait_for(lambda: (lambda x: x if x["bag"]["state"] == "OFF" else None)(h.status()), 8.0)
        r0 = next((r for r in (s or {}).get("bag", {}).get("recent", []) if r["name"] == name), None)
        check("Dung ghi: Ctrl-C toi tien trinh ghi; danh sach bag co metadata (thoi gian, so tin)",
              s is not None and r0 is not None and r0["ok"] and r0["msgs"] == 321 and r0["dur"] > 3.0, str(r0))
        e = find_ev(get_events(h), "bag", ev="done", name=name)
        check("su kien 'da luu bag' + ly do 'bam Dung ghi'", e is not None and e["lvl"] == "ok"
              and e["a"].get("why") == "bam Dung ghi", str(e))
        (ws / "bagfail.flag").write_text("1")
        c, j = h.act("bag_start")
        e = wait_for(lambda: find_ev(get_events(h), "bag", ev="fail"), 5.0)
        s = wait_for(lambda: (lambda x: x if x["bag"]["state"] == "OFF" else None)(h.status()), 3.0)
        check("ghi bag loi -> su kien loi (ma 1 + dong loi cuoi), tro ve OFF", e is not None and e["a"].get("code") == 1
              and "Output folder" in e["a"].get("last", "") and s is not None, str(e))
        (ws / "bagfail.flag").unlink()
        ext_bag = subprocess.Popen([sys.executable, "-c", EXT_BAG, str(ws / "events.txt"), "bag", "record", "-o",
                                    str(ws / "bags" / "ngoai_web")], start_new_session=True)
        s = wait_for(lambda: (lambda x: x if x["bag"]["external"] else None)(h.status()), 6.0)
        check("nhan ra 'ros2 bag record' chay ngoai web", s is not None and s["bag"]["name"] == "ngoai_web"
              and s["bag"]["state"] == "RECORDING", s and str(s["bag"]))
        c, j = h.act("bag_stop")
        s = wait_for(lambda: (lambda x: x if x["bag"]["state"] == "OFF" else None)(h.status()), 8.0)
        check("dung duoc ghi ngoai web (Ctrl-C)", s is not None and any(e.startswith("EXTBAG_SIGINT") for e in events(ws))
              and wait_for(lambda: ext_bag.poll() is not None, 3.0) is not None)

        print("\n5. Pin may tinh (sysfs gia) + pin khung xe (/bw_dr03/power)")
        bt = h.status()["power"]["bat"]
        check("pin may: 15 %, dang dung pin, con ~0.45 h", bt is not None and bt["pct"] == 15 and bt["status"] == "Discharging"
              and bt["ac"] is False and bt["h_left"] is not None and abs(bt["h_left"] - 0.45) < 0.05, str(bt))
        e = find_ev(get_events(h), "bat", src="may")
        check("pin may <= 20 % -> su kien canh bao", e is not None and e["lvl"] == "warn", str(e))
        write_bat(ps, 8, "Discharging")
        e = wait_for(lambda: next((x for x in get_events(h) if x["code"] == "bat" and x["a"].get("src") == "may"
                                   and x["lvl"] == "bad"), None), 7.0)
        check("pin may <= 10 % -> su kien nguy hiem", e is not None, str(e))
        time.sleep(5.5)
        n_bat = len([x for x in get_events(h) if x["code"] == "bat" and x["a"].get("src") == "may"])
        check("pin van <= 10 % -> khong bao lap", n_bat == 2, f"{n_bat} su kien")
        ch = h.status()["power"]["chassis"]
        check("pin khung xe: trung vi ~12.34 V, binh thuong", ch is not None and abs(ch["v"] - 12.34) < 0.01
              and ch["unit"] == "V" and ch["lvl"] == "", str(ch))
        fake.power_v = 11.0
        e = wait_for(lambda: find_ev(get_events(h), "bat", src="xe"), 18.0)
        check("pin khung xe <= nguong bad -> su kien", e is not None and e["lvl"] == "bad", str(e))
        fake.power_v = 12.34
        write_bat(ps, 80, "Charging")

        print("\n6. Anh camera + video truc tiep (MJPEG)")
        # anh chi duoc dang ky khi co nguoi xem -> lan xin dau tra 204, toi da ~1 s sau moi co anh
        first = h.get("/api/camera.jpg")[0]
        check("anh: lan xin dau khi chua ai xem -> 204 (chua dang ky anh)", first == 204, f"HTTP {first}")
        got = wait_for(lambda: (lambda r: r if r[0] == 200 else None)(h.get("/api/camera.jpg")), 4.0, 0.2)
        code, body, hdr = got if got else h.get("/api/camera.jpg")
        w = None
        if code == 200 and cv2 is not None:
            im = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR)
            w = None if im is None else im.shape[1]
        check("anh: JPEG be ngang 480", code == 200 and body[:2] == b"\xff\xd8" and w == 480, f"HTTP {code}, rong {w}")
        code, n, widths, fps = read_mjpeg(h, 4.0)
        check("video: >= 12 hinh/s (camera gia 15 Hz), JPEG rong 480", code == 200 and fps >= 12.0 and widths == {480},
              f"HTTP {code}, {n} hinh, {fps:.1f} hinh/s, rong {sorted(widths)}")
        wait_for(lambda: h.status()["video"]["viewers"] == 0, 3.0)   # luong truoc vua dong (~0.1-0.2 s)
        conns = [open_mjpeg(h) for _ in range(3)]
        fourth = open_mjpeg(h)
        check("video: toi da 3 nguoi xem, nguoi thu 4 -> 503", [c[1] for c in conns] == [200] * 3 and fourth[1] == 503,
              f"{[c[1] for c in conns]} + {fourth[1]}")
        for c in conns + [fourth]:
            c[2].close()
            c[0].close()
        check("video: dong trang -> node biet nguoi xem roi di",
              wait_for(lambda: h.status()["video"]["viewers"] == 0, 5.0) is not None, str(h.status()["video"]))
        time.sleep(6.5)                                       # khong ai xem > 5 s -> huy dang ky anh
        v = h.status()["video"]
        check("video: het nguoi xem > 5 s -> thoi nhan anh camera", v.get("subscribed") is False, str(v))

        print("\n7. Node web chi doc, khong ghi")
        for nn in ("robot_web", "robot_web_probe", "robot_web_video"):
            pubs = wait_for(lambda: fake.get_publisher_names_and_types_by_node(nn, "/"), 3.0) or []
            topics = sorted(t for t, _ in pubs)
            check(f"node {nn} khong publish topic nao (ngoai /rosout, /parameter_events)",
                  bool(topics) and set(topics) <= {"/rosout", "/parameter_events"}, str(topics))

        print("\n8. CPU (robot gia phat dung tan so xe that: /odom 50 Hz, anh 640x480 15 Hz...)")
        stop_evt = threading.Event()

        def poller():
            while not stop_evt.is_set():
                h.status()
                time.sleep(0.4)
        th = threading.Thread(target=poller, daemon=True)
        th.start()
        time.sleep(1.0)
        c0, t0 = group_cpu(web.pid), time.time()
        time.sleep(8.0)
        cpu = 100.0 * (group_cpu(web.pid) - c0) / (time.time() - t0)
        check("mo trang, KHONG xem video: < 4% mot nhan", cpu < 4.0, f"{cpu:.1f} %")
        res: list = []
        vt = threading.Thread(target=lambda: res.append(read_mjpeg(h, 11.0)), daemon=True)
        vt.start()
        time.sleep(2.0)                                       # bo 2 s dau (dang ky anh)
        c0, t0 = group_cpu(web.pid), time.time()
        time.sleep(8.0)
        cpu = 100.0 * (group_cpu(web.pid) - c0) / (time.time() - t0)
        vt.join(5.0)
        stop_evt.set()
        th.join(2.0)
        vfps = res[0][3] if res else 0.0
        check("xem video truc tiep: >= 12 hinh/s va < 12% mot nhan", vfps >= 12.0 and cpu < 12.0,
              f"{vfps:.1f} hinh/s, {cpu:.1f} %")

        print("\n9. Nut DUNG + dong su kien + file nhat ky")
        fake.set(image=False)
        h2 = Http(h.base)                                   # khong co cookie
        n0 = len(fake.called("/follow/stop"))
        c, j = h2.act("estop")
        check("DUNG khong can dang nhap", c == 200 and j["ok"] and len(fake.called("/follow/stop")) == n0 + 1, j["msg"])
        c, _, _ = h2.post("/api/action/estop", header=False)
        check("DUNG thieu header -> 403 (chan trang web la goi ho)", c == 403)
        fake.drop_service("/follow/stop")
        time.sleep(1.5)
        c, j = h.act("estop")
        check("mat /follow/stop -> DUNG bang /follow/disable", j["ok"] and "/follow/disable" in j["msg"], j["msg"])
        fake.drop_service("/follow/disable")
        time.sleep(1.5)
        c, j = h.act("estop")
        check("mat ca hai -> bao dung cong tac nguon", not j["ok"] and "cong tac" in j["msg"], j["msg"])
        code = h.get("/api/camera.jpg")[0]
        check("mat anh camera -> 204", code == 204, f"HTTP {code}")

        code, body, _ = h.get("/api/events")
        j = json.loads(body) if code == 200 else {}
        items = j.get("items", [])
        check("/api/events: boot + rev + danh sach", code == 200 and bool(j.get("boot")) and j.get("rev", 0) >= len(items) > 20,
              f"{len(items)} su kien, rev {j.get('rev')}")
        check("trang thai bao ev.rev de trang biet khi nao tai lai", h.status()["ev"]["rev"] >= j.get("rev", 0))
        acts = [x for x in items if x["code"] == "act"]
        names = [x["a"]["name"] for x in acts]
        check("su kien nut bam: ten + ket qua + IP; DUNG muc 'bad'",
              all(k in names for k in ("check", "start", "shutdown", "bag_start", "bag_stop", "estop"))
              and all(x["a"].get("who") == "127.0.0.1" for x in acts)
              and all(x["lvl"] == "bad" for x in acts if x["a"]["name"] == "estop"), str(sorted(set(names))))
        logs = list((ws / "run_logs").glob("web_*.log"))
        lines = logs[0].read_text().splitlines() if logs else []
        ev_lines = [ln for ln in lines if " su_kien " in ln]
        check("nhat ky: nut bam + su kien ghi vao run_logs/web_*.log", len(lines) >= 40 and len(ev_lines) >= 15
              and any("tim nguoi" in ln for ln in ev_lines) and any("da luu follow_" in ln for ln in ev_lines),
              f"{len(lines)} dong, {len(ev_lines)} su kien")
    finally:
        for p in (ext, ext_bag):
            if p is not None and p.poll() is None:
                p.kill()
        try:
            os.killpg(web.pid, signal.SIGINT)
            web.wait(10.0)
            check("node web tat gon khi Ctrl-C (ma thoat 0)", web.returncode == 0, f"ma {web.returncode}")
        except (ProcessLookupError, subprocess.TimeoutExpired):
            check("node web tat gon khi Ctrl-C", False)
            os.killpg(web.pid, signal.SIGKILL)
        stop_spin.set()
        th_spin.join(3.0)
        ex.shutdown()
        fake.destroy_node()
        rclpy.try_shutdown()

    n_bad = sum(1 for _, ok, _ in RESULTS if not ok)
    print(f"\n{len(RESULTS) - n_bad}/{len(RESULTS)} muc dat — nhat ky node web: {tmp / 'web.log'}")
    print("=> DAT" if n_bad == 0 else "=> LOI")
    return 0 if n_bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
