"""
web_node.py — trang dieu khien robot bam nguoi qua trinh duyet
==============================================================
Chay tren MAY CHAY ROBOT (bay gio la laptop tren xe, sau nay la mini PC). May khac (dien thoai, laptop
khac) cung mang chi can mo trinh duyet:   http://<ip may robot>:8080

Node nay CHI DOC topic va GOI service co san. KHONG BAO GIO ghi /cmd_vel — giu bat bien "chi MOT nguon ra
lenh cho xe" (follow_planner_node). Nut DUNG tren web la dung MEM qua Wi-Fi, co the tre / mat ket noi
-> an toan cuoi cung la cong tac cat nguon dong co tren xe.

Nut -> viec lam. May chu TU KIEM dieu kien (khong chi dua vao trang), tra ve ly do khi tu choi:
  check           run_full.sh --check                         khi he thong chua chay
  start           run_full.sh [--no-rssi]                     khi chua chay va khong co planner nao dang chay.
                  KHONG BAO GIO them --force: kiem tra co muc HONG thi run_full.sh tu thoat, web bao loi.
  shutdown        /follow/stop -> cho stop_wait_sec -> SIGINT ca NHOM tien trinh launch (nhu Ctrl-C)
                  Phai dung xe TRUOC: khung xe giu lenh cuoi ~3 s khi driver tat (CLAUDE.md 7.3).
  enroll_start    /person_reid/start_enroll                   camera dang gui du lieu, xe khong dang bam
  enroll_finish   /person_reid/finish_enroll                  dang hoc
  reset_identity  /person_reid/reset                          xe khong dang bam
  follow_enable   /follow/enable    planner dang chay va CHUA bat, da hoc xong, LiDAR + odom con du lieu,
                                    dung 1 node ghi /cmd_vel
  follow_disable  /follow/disable                             luon
  bag_start       run_full.sh --bag (ghi ~/bags/follow_<ngay_gio>, DUNG danh sach topic cua run_full.sh, KHONG
                  ghi anh camera)        he thong dang chay (hoac co planner), chua ghi, o dia con >= bag_min_free_gb
  bag_stop        SIGINT nhom tien trinh ghi (nhu Ctrl-C: ros2 bag dong file + ghi metadata.yaml)   khi dang ghi
  estop           /follow/stop (khong goi duoc thi thu /follow/disable)   luon, KHONG can dang nhap
  ("start" kem "bag": true -> tu ghi bag ngay khi qua kiem tra)

Ghi bag do web bat tu dung 2 s sau khi he thong tat, hoac khi o dia con < bag_stop_free_gb. Tien trinh ghi o
nhom tien trinh rieng: tat web khong dung ghi; bat lai web thi web nhan ra ("ros2 bag record") va van dung duoc.

Dong su kien (trang web + file nhat ky): he thong bat / tat / loi, xe bat / ngung bam, planner ne / tim nguoi /
bi chan / dung xe (kem thoi gian), cam bien ngung gui, nhieu node ghi /cmd_vel, mat beacon, hoc xong nguoi,
ghi bag, pin yeu, nut da bam. Pin: may tinh (power_supply_dir/BAT*) + so doc /bw_dr03/power cua driver khung
xe ([CAN XAC NHAN] don vi: driver goi la power_v; nguong canh bao chassis_power_* mac dinh tat).

Mat ket noi dien thoai -> xe van bam binh thuong (nguoi dung chon 08/10); trang tu bao "MAT KET NOI".

He thong do web bat chay trong NHOM TIEN TRINH rieng (start_new_session): Ctrl-C / tat node web KHONG tat
he thong robot (khong tat ngang khi xe dang chay). Bat lai web thi web tu nhan ra he thong dang chay
(tim tien trinh "ros2 launch ... follow_nav_real.launch.py") va van tat duoc.

Mat khau: file password_file (mac dinh ~/.config/robot_web.conf, JSON {"password": "..."}), NGOAI repo vi
repo cong khai. Chua co file thi tu tao ma 6 so va in ra luc khoi dong.
Nhat ky moi lan bam nut: <ws>/run_logs/web_<thangngay>.log

Chay:  ros2 run robot_web web_server              (tham so: --ros-args -p port:=8080 ...)
"""

from __future__ import annotations

import json
import os
import re
import secrets
import select
import socket
import signal
import subprocess
import threading
import time
from collections import deque
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from geometry_msgs.msg import Twist  # CHI de dem tan so /cmd_vel (subscribe) — node nay KHONG publish
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

try:
    import cv2
    # Anh nho (480 px): OpenCV chia viec cho nhieu luong roi cac luong cho viec quay vong -> ton CPU gap nhieu lan
    # (do 08/10: xem video 14 hinh/s 23.8 % mot nhan). Mot luong la du. Chi anh huong tien trinh node web.
    cv2.setNumThreads(1)
except ImportError:  # khong co OpenCV thi chi mat anh camera, cac nut van chay
    cv2 = None

STATIC_DIR = Path(__file__).resolve().parent / "static"
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
CHECK_LINE = re.compile(r"^\s*(OK|CANH BAO|HONG)\s+(.*\S)\s*$")
SECTION_LINE = re.compile(r"^\s*(\d+[a-z]?)\.\s+(.*\S)\s*$")
LAUNCH_FILE = "follow_nav_real.launch.py"
BAG_LINE = re.compile(r"Ghi bag -> (\S+)")                       # dong run_full.sh --bag in ra truoc khi ghi
SUB_LINE = re.compile(r"Subscribed to topic '([^']+)'")          # ros2 bag record: da dang ky mot topic
# metadata.yaml (ros2 bag ghi khi dong file): khoa cap 1 thut 2 dau cach
META_DUR = re.compile(r"^  duration:\s*\n\s+nanoseconds:\s*(\d+)", re.M)
META_START = re.compile(r"^  starting_time:\s*\n\s+nanoseconds_since_epoch:\s*(\d+)", re.M)
META_MSGS = re.compile(r"^  message_count:\s*(\d+)", re.M)

SERVICES = {
    "enroll_start": "/person_reid/start_enroll",
    "enroll_finish": "/person_reid/finish_enroll",
    "reset_identity": "/person_reid/reset",
    "follow_enable": "/follow/enable",
    "follow_disable": "/follow/disable",
    "estop": "/follow/stop",
}
ACTIONS = ("check", "start", "shutdown", "enroll_start", "enroll_finish", "reset_identity",
           "follow_enable", "follow_disable", "estop", "bag_start", "bag_stop")

# "Con du lieu" (s). /scan: bang scan_timeout_sec cua planner; /odom 20-50 Hz
SCAN_STALE = 0.6
ODOM_STALE = 0.5
PLANNER_STALE = 2.0
IDENTITY_STALE = 2.0
IMAGE_STALE = 2.0
RSSI_STALE = 3.0
CHASSIS_STALE = 2.0

# Trang thai planner coi la binh thuong khi dang bam ("OFF" = khong bam). Cac trang thai khac thanh mot "dot"
# tren dong su kien (bat dau -> ket thuc, kem thoi gian).
PLAN_NORMAL = ("FOLLOW", "ARRIVED", "OFF")
PLAN_LVL = {"AVOID": "info", "OCCLUDED": "info", "SEARCH": "warn", "IDLE": "warn", "BLOCKED": "bad", "ESTOP": "bad"}
PLAN_TXT = {"AVOID": "ne vat can", "OCCLUDED": "nguoi khuat camera", "SEARCH": "tim nguoi",
            "BLOCKED": "bi chan duong", "ESTOP": "planner dung xe", "IDLE": "dung cho (khong thay nguoi)"}
STALE_TXT = {"scan": "LiDAR /scan", "odom": "khung xe /odom", "cam": "camera /person_reid/target",
             "planner": "planner /follow/planner_status", "rssi": "RSSI /rssi/bearing"}
LVL_LOG = {"ok": "OK ", "info": "-  ", "warn": "CB ", "bad": "LOI"}
BAT_WARN, BAT_BAD = 20, 10            # % pin may tinh khi dang dung pin


def _now() -> float:
    return time.time()


def _name_thread(name: str) -> None:
    """Dat ten luong cho he dieu hanh (top -H, do CPU theo luong). Loi thi bo qua."""
    try:
        with open(f"/proc/self/task/{threading.get_native_id()}/comm", "w") as f:
            f.write(name[:15])
    except OSError:
        pass


def _age(t: float, now: float) -> Optional[float]:
    return None if t <= 0.0 else round(now - t, 2)


def _fresh(t: float, now: float, lim: float) -> bool:
    return t > 0.0 and now - t < lim


class Rate:
    """Tan so + luc nhan tin cuoi cua mot topic, do tu TIN MOI NHAT lay moi probe_period_sec.

    Node khong xu ly tung tin (rclpy ton ~0.75 ms moi tin: /odom 50 Hz + 5 topic khac = 8 % mot nhan CPU).
    Moi lan lay chi co tin moi nhat (QoS giu 1 tin); tan so tinh tu publication_sequence_number cua no.
    Nhieu node cung ghi mot topic (vd. 2 nguon /cmd_vel) thi so thu tu lan lon -> bao 0 (khong ro).
    Goi duoi app.lock.
    """

    def __init__(self, window: float = 2.0) -> None:
        self.window = window
        self.s: deque = deque()       # (luc nhan, so thu tu)
        self.last = 0.0

    def tick(self, info: Any, now: float) -> None:
        t, seq = now, None
        if isinstance(info, dict):
            rt = info.get("received_timestamp")
            if rt:
                t = min(now, rt * 1e-9)
            seq = info.get("publication_sequence_number")
        self.last = t
        if seq is None:
            self.s.clear()
            return
        if self.s and (seq <= self.s[-1][1] or seq - self.s[-1][1] > 5000):
            self.s.clear()            # nguon khac / nguon vua khoi dong lai
        self.s.append((t, seq))
        while len(self.s) > 2 and self.s[0][0] < t - self.window:
            self.s.popleft()

    def hz(self, now: float) -> float:
        if len(self.s) < 2 or now - self.last > 1.0:
            return 0.0
        (t0, q0), (t1, q1) = self.s[0], self.s[-1]
        return round((q1 - q0) / (t1 - t0), 1) if t1 > t0 else 0.0


def _pg_alive(pgid: int) -> bool:
    """Con tien trinh SONG trong nhom? Khong tinh zombie (da thoat, cho cha thu don) — killpg(pgid, 0) van
    bao "con" voi zombie nen tat xong van ngoi cho het 25 + 8 + 3 s."""
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/stat", "rb") as f:
                st = f.read()
        except OSError:
            continue
        fields = st[st.rfind(b")") + 2:].split()
        if len(fields) > 2 and int(fields[2]) == pgid and fields[0] not in (b"Z", b"X"):
            return True
    return False


def _proc_cmdlines() -> List[Tuple[int, List[str]]]:
    """(pid, argv) cua moi tien trinh — doc /proc MOT lan cho ca tim launch lan tim tien trinh ghi bag."""
    out = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                raw = f.read()
        except OSError:
            continue
        if raw:
            out.append((int(d), [a.decode(errors="replace") for a in raw.split(b"\0") if a]))
    return out


def _proc_start(pid: int) -> Optional[float]:
    """Luc tien trinh bat dau (giay epoch): truong 22 cua /proc/<pid>/stat + btime."""
    try:
        with open(f"/proc/{pid}/stat", "rb") as f:
            st = f.read()
        ticks = int(st[st.rfind(b")") + 2:].split()[19])
        with open("/proc/stat", "rb") as f:
            for ln in f:
                if ln.startswith(b"btime "):
                    return int(ln.split()[1]) + ticks / os.sysconf("SC_CLK_TCK")
    except (OSError, ValueError, IndexError):
        pass
    return None


def _dir_size(p: Optional[Path]) -> int:
    """Tong dung luong cac file ngay trong thu muc (bag: metadata.yaml + *.mcap)."""
    tot = 0
    if p is None:
        return 0
    try:
        with os.scandir(p) as it:
            for e in it:
                try:
                    if e.is_file(follow_symlinks=False):
                        tot += e.stat(follow_symlinks=False).st_size
                except OSError:
                    pass
    except OSError:
        pass
    return tot


def bag_info(p: Path) -> Dict[str, Any]:
    """Thong tin mot bag. ok=False: chua co metadata.yaml (dang ghi, hoac bi tat ngang -> can ros2 bag reindex)."""
    info: Dict[str, Any] = {"name": p.name, "t": None, "size": _dir_size(p), "dur": None, "msgs": None, "ok": False}
    try:
        info["t"] = round(p.stat().st_mtime, 1)
        txt = (p / "metadata.yaml").read_text(encoding="utf-8", errors="replace")
    except OSError:
        return info
    m, t0, n = META_DUR.search(txt), META_START.search(txt), META_MSGS.search(txt)
    if m:
        info["dur"], info["ok"] = round(int(m.group(1)) / 1e9, 1), True
    if t0:
        info["t"] = round(int(t0.group(1)) / 1e9, 1)
    if n:
        info["msgs"] = int(n.group(1))
    return info


def bag_topics(script: Optional[Path]) -> List[str]:
    """Danh sach topic cua lenh "ros2 bag record" trong run_full.sh (doc tu chinh file -> khong lech khi sua)."""
    if script is None:
        return []
    try:
        lines = script.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return []
    out: List[str] = []
    on = False
    for ln in lines:
        s = ln.strip()
        if not on:
            if s.startswith("#") or "ros2 bag record" not in s:
                continue
            on = True
        out += [t for t in s.split() if t.startswith("/") and len(t) > 1]
        if not s.endswith("\\"):
            break
    return out


def _bag_out(pid: int, args: List[str]) -> Optional[Path]:
    """Thu muc ghi cua mot tien trinh "ros2 bag record" (tham so -o / --output; duong dan tuong doi theo cwd)."""
    out = None
    for i, a in enumerate(args):
        if a in ("-o", "--output") and i + 1 < len(args):
            out = args[i + 1]
        elif a.startswith("--output="):
            out = a.split("=", 1)[1]
    if not out:
        return None
    p = Path(out)
    if not p.is_absolute():
        try:
            p = Path(os.readlink(f"/proc/{pid}/cwd")) / p
        except OSError:
            return None
    return p


def _rd(p: Path) -> Optional[str]:
    try:
        return p.read_text().strip()
    except OSError:
        return None


def _rdn(p: Path) -> Optional[float]:
    s = _rd(p)
    try:
        return float(s) if s else None
    except ValueError:
        return None


def read_battery(root: Path) -> Optional[Dict[str, Any]]:
    """Pin may tinh tu sysfs: {pct, status, ac, w, h_left}. Khong co pin (mini PC cam dien) -> None.
    status: Charging / Discharging / Full / Not charging (cam dien nhung khong sac, vd. che do bao ve pin)."""
    try:
        ents = sorted(root.iterdir())
    except OSError:
        return None
    bats, ac = [], None
    for d in ents:
        typ = _rd(d / "type")
        if typ == "Mains":
            on = _rd(d / "online")
            if on is not None:
                ac = bool(ac) or on == "1"
        elif typ == "Battery" and _rd(d / "present") != "0":
            pct = _rdn(d / "capacity")
            if pct is None:
                continue
            e_now, e_full, p_now = _rdn(d / "energy_now"), _rdn(d / "energy_full"), _rdn(d / "power_now")
            if e_now is None:                      # pin bao theo dien tich (uAh, uA) -> doi ra uWh, uW
                v = _rdn(d / "voltage_now")
                c_now, c_full, i_now = _rdn(d / "charge_now"), _rdn(d / "charge_full"), _rdn(d / "current_now")
                if v:
                    e_now = c_now * v / 1e6 if c_now is not None else None
                    e_full = c_full * v / 1e6 if c_full is not None else None
                    p_now = abs(i_now) * v / 1e6 if i_now is not None else None
            bats.append({"pct": pct, "status": _rd(d / "status") or "", "e_now": e_now, "e_full": e_full,
                         "p_now": p_now})
    if not bats:
        return None
    e_now = sum(b["e_now"] or 0.0 for b in bats)
    e_full = sum(b["e_full"] or 0.0 for b in bats)
    pct = 100.0 * e_now / e_full if len(bats) > 1 and e_full > 0 else bats[0]["pct"]
    stats = [b["status"] for b in bats]
    status = "Discharging" if "Discharging" in stats else ("Charging" if "Charging" in stats else stats[0])
    p_now = sum(b["p_now"] or 0.0 for b in bats)
    h_left = e_now / p_now if status == "Discharging" and p_now > 0 and e_now > 0 else None
    return {"pct": int(round(min(100.0, max(0.0, pct)))), "status": status, "ac": ac,
            "w": round(p_now / 1e6, 1) if p_now > 0 else None, "h_left": round(h_left, 2) if h_left else None}


def _fmt_dur(d: Any) -> str:
    if d is None:
        return "?"
    d = int(round(float(d)))
    return f"{d // 3600}:{d // 60 % 60:02d}:{d % 60:02d}" if d >= 3600 else f"{d // 60}:{d % 60:02d}"


def ev_text(ev: Dict[str, Any], phase: str = "") -> str:
    """Mot dong chu khong dau cho file nhat ky. phase "end": su kien "dot" vua ket thuc."""
    c, a = ev["code"], ev["a"]
    dur = f" sau {ev.get('end', _now()) - ev['t']:.1f} s" if phase == "end" else ""
    if c == "sys":
        return f"he thong {a.get('prev')} -> {a.get('state')}" + (f": {a['detail']}" if a.get("detail") else "")
    if c == "follow":
        return "xe bat dau bam" if a.get("on") else "xe ngung bam"
    if c == "plan":
        name = PLAN_TXT.get(str(a.get("state")), str(a.get("state")))
        if phase == "end":
            return f"het {name}{dur} -> {a.get('next')}" + (f" (gop {ev['n']} lan)" if ev.get("n", 1) > 1 else "")
        return name + (f": {a['note']}" if a.get("note") else "")
    if c == "enroll":
        k = a.get("ev")
        return {"done": f"hoc xong nguoi ({a.get('samples')} mau)", "cancel": "huy hoc nguoi (chua du mau)",
                "clear": "khong con nguoi da hoc"}.get(str(k), str(k))
    if c == "stale":
        name = STALE_TXT.get(str(a.get("what")), str(a.get("what")))
        if phase == "end":
            return f"{name} ngung gui{dur} — " + ("gui lai" if a.get("next") == "ok" else "he thong tat")
        return f"{name} ngung gui"
    if c == "cmdpubs":
        return f"so node ghi /cmd_vel ve 1{dur}" if phase == "end" else f"co {a.get('n')} node ghi /cmd_vel"
    if c == "beacon":
        if phase == "end":
            return f"mat beacon{dur} — " + ("co lai" if a.get("next") == "ok" else "khong con du lieu RSSI")
        return "mat tin hieu beacon"
    if c == "bag":
        k = a.get("ev")
        if k == "start":
            return f"bat dau ghi {a.get('name')}" + (" (tu dong)" if a.get("auto") else "") + \
                (" (ngoai web)" if a.get("external") else "")
        if k == "done":
            return (f"da luu {a.get('name')}: {_fmt_dur(a.get('dur'))}, {(a.get('size') or 0) / 1e6:.1f} MB"
                    + (" — THIEU metadata.yaml" if a.get("nometa") else "") + (f" ({a['why']})" if a.get("why") else ""))
        return f"ghi bag loi (ma {a.get('code')}): {a.get('last', '')}"
    if c == "bat":
        return f"pin {'may tinh' if a.get('src') == 'may' else 'khung xe'} con {a.get('val')}"
    return c


class Stable:
    """Gia tri on dinh: chi doi khi gia tri moi gap n lan LIEN TIEP (moi lan = mot nhip _tick 0.5 s)
    -> trang thai chop nhoang (vd. FOLLOW <-> AVOID trong 1 nhip) khong lam tran dong su kien."""

    def __init__(self, value: Any, n: int = 2) -> None:
        self.value, self.cand, self.k, self.n = value, value, 0, n

    def update(self, v: Any) -> bool:
        if v == self.value:
            self.cand, self.k = v, 0
            return False
        if v != self.cand:
            self.cand, self.k = v, 0
        self.k += 1
        if self.k < self.n:
            return False
        self.value, self.k = v, 0
        return True


def find_ws(param: str) -> Optional[Path]:
    """Thu muc workspace: tham so ws_dir, bien WS (~/rssi_env.sh), roi di nguoc tu vi tri file nay."""
    cands = [param, os.environ.get("WS", "")] + [str(p) for p in Path(__file__).resolve().parents]
    for c in cands:
        if not c:
            continue
        p = Path(c).expanduser()
        if (p / "src" / "person_follow_nav" / "scripts" / "run_full.sh").is_file():
            return p
    return None


def load_password(path: str) -> Tuple[str, bool]:
    """(mat khau, vua tao moi?). File hong -> ValueError (khong ghi de mat khau nguoi dung da dat)."""
    p = Path(path).expanduser()
    if p.exists():
        try:
            pw = str(json.loads(p.read_text(encoding="utf-8")).get("password", "")).strip()
        except Exception as exc:
            raise ValueError(f"khong doc duoc {p}: {exc}") from exc
        if not pw:
            raise ValueError(f"{p} khong co truong \"password\"")
        return pw, False
    pw = "%06d" % secrets.randbelow(10 ** 6)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps({"password": pw}) + "\n", encoding="utf-8")
    os.chmod(p, 0o600)
    return pw, True


def image_to_bgr(msg: Image, width: int) -> Optional[np.ndarray]:
    """sensor_msgs/Image -> anh BGR thu nho ve be ngang `width` (khong can cv_bridge)."""
    enc = msg.encoding.lower()
    ch = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1, "8uc1": 1, "8uc3": 3}.get(enc)
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if ch is None or h <= 0 or w <= 0 or step < w * ch:
        return None
    try:
        buf = np.frombuffer(msg.data, dtype=np.uint8)
    except TypeError:                   # phong khi data la list
        buf = np.asarray(msg.data, dtype=np.uint8)
    if buf.size < h * step:
        return None
    arr = buf[: h * step].reshape(h, step)[:, : w * ch].reshape(h, w, ch)
    if enc == "rgb8":
        arr = arr[:, :, ::-1]
    elif enc == "bgra8":
        arr = arr[:, :, :3]
    elif enc == "rgba8":
        arr = arr[:, :, 2::-1]
    elif ch == 1:
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    if 0 < width < w:
        # INTER_LINEAR: 0.5 ms/khung (INTER_AREA 1.9 ms) — thu 640 -> 480 nhin gan nhu nhau
        arr = cv2.resize(arr, (width, max(1, int(round(h * width / w)))), interpolation=cv2.INTER_LINEAR)
    return np.ascontiguousarray(arr)


# ─────────────────────────────────────────────────────────────────────────────
# Dong su kien
# ─────────────────────────────────────────────────────────────────────────────

class Events:
    """Su kien = dict {id, t, lvl, code, a}. Su kien "dot" (vd. xe dang tim nguoi) co them key/ep: begin() mo,
    end() ghi luc ket thuc (trang hien thoi gian keo dai). Cung key lap lai trong merge_sec sau khi dong ma chua
    co su kien nao khac xen giua -> MO LAI dot cu (n += 1) thay vi them dong moi. Moi lan them / sua: rev += 1,
    trang chi tai lai danh sach khi rev doi. Moi su kien (tru nut bam — audit() da ghi) cung ghi vao file nhat ky."""

    def __init__(self, app: "RobotWeb", maxlen: int = 200) -> None:
        self.app = app
        self.lock = threading.Lock()
        self.items: deque = deque(maxlen=maxlen)
        self.open: Dict[str, Dict[str, Any]] = {}
        self.rev = 0
        self.next_id = 1
        self.boot = secrets.token_hex(4)          # doi khi node web khoi dong lai -> trang xoa danh sach cu

    def _new(self, code: str, lvl: str, a: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        ev = {"id": self.next_id, "t": round(_now(), 2), "lvl": lvl, "code": code, "a": dict(a or {})}
        self.next_id += 1
        self.items.append(ev)
        self.rev += 1
        return ev

    def add(self, code: str, lvl: str, a: Optional[Dict[str, Any]] = None, log: bool = True) -> None:
        with self.lock:
            ev = self._new(code, lvl, a)
            line = ev_text(ev) if log else None
        if line is not None:
            self.app.audit_event(code, lvl, line)

    def begin(self, key: str, code: str, lvl: str, a: Optional[Dict[str, Any]] = None, merge_sec: float = 4.0) -> None:
        now = _now()
        with self.lock:
            if key in self.open:
                return
            last = self.items[-1] if self.items else None
            if last is not None and last.get("key") == key and "end" in last and now - last["end"] < merge_sec:
                del last["end"]
                last["n"] = last.get("n", 1) + 1
                self.open[key] = last
                self.rev += 1
                ev = last
            else:
                ev = self._new(code, lvl, a)
                ev["key"], ev["ep"] = key, True
                self.open[key] = ev
            line = ev_text(ev)
        self.app.audit_event(code, lvl, line)

    def end(self, key: str, a: Optional[Dict[str, Any]] = None) -> None:
        with self.lock:
            ev = self.open.pop(key, None)
            if ev is None:
                return
            ev["end"] = round(_now(), 2)
            if a:
                ev["a"].update(a)
            self.rev += 1
            line, code, lvl = ev_text(ev, "end"), ev["code"], ev["lvl"]
        self.app.audit_event(code, lvl, line)

    def meta(self) -> Dict[str, Any]:
        with self.lock:
            return {"boot": self.boot, "rev": self.rev, "n": len(self.items)}

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:                            # chep ca "a": end() sua tai cho trong luong khac
            return {"boot": self.boot, "rev": self.rev, "t": round(_now(), 2),
                    "items": [dict(e, a=dict(e["a"])) for e in self.items]}


# ─────────────────────────────────────────────────────────────────────────────
# Bat / tat he thong robot
# ─────────────────────────────────────────────────────────────────────────────

class Stack:
    """run_full.sh trong nhom tien trinh rieng; giu nhat ky + ket qua kiem tra. Moi truong deu duoi self.lock."""

    def __init__(self, app: "RobotWeb") -> None:
        self.app = app
        self.lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.mode = ""                # "check" | "run"
        self.state = "STOPPED"        # STOPPED CHECKING STARTING RUNNING STOPPING FAILED
        self.detail = ""
        self.since = _now()
        self.items: List[Dict[str, str]] = []
        self.log: deque = deque(maxlen=400)
        self.ext_pgid: Optional[int] = None
        self.stopping = False
        self.user_stop = False        # nguoi dung bam Tat: tien trinh ket thuc la DUNG Y, khong phai loi
        self.rssi = True
        self.exit_code: Optional[int] = None

    def _set(self, state: str, detail: str) -> None:
        prev = self.state
        self.state, self.detail = state, detail
        if state != prev:
            self.since = _now()
            self.app.events.add("sys", {"RUNNING": "ok", "FAILED": "bad"}.get(state, "info"),
                                {"state": state, "prev": prev, "detail": detail, "mode": self.mode})

    def alive(self) -> bool:
        return (self.proc is not None and self.proc.poll() is None) or self.ext_pgid is not None

    def snapshot(self, n_log: int = 14) -> Dict[str, Any]:
        with self.lock:
            return {"state": self.state, "detail": self.detail, "since": round(self.since, 1),
                    "alive": self.alive(), "external": self.proc is None and self.ext_pgid is not None,
                    "mode": self.mode, "rssi": self.rssi, "exit_code": self.exit_code,
                    "items": list(self.items), "log": list(self.log)[-n_log:]}

    def log_lines(self, n: int) -> List[str]:
        with self.lock:
            return list(self.log)[-n:]

    def launch(self, mode: str, rssi: bool) -> Tuple[bool, str]:
        script = self.app.run_script
        if script is None:
            return False, "khong thay run_full.sh (dat tham so ws_dir)"
        args = ["bash", str(script)]
        if mode == "check":
            args.append("--check")
        elif not rssi:
            args.append("--no-rssi")
        with self.lock:
            if self.alive() or self.stopping:
                return False, "he thong dang chay — tat truoc"
            env = dict(os.environ)
            env["PYTHONUNBUFFERED"] = "1"
            try:
                proc = subprocess.Popen(args, cwd=str(self.app.ws), env=env, stdin=subprocess.DEVNULL,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                        errors="replace", bufsize=1, start_new_session=True)
            except OSError as exc:
                return False, f"khong chay duoc run_full.sh: {exc}"
            self.proc, self.mode, self.rssi, self.exit_code = proc, mode, rssi, None
            self.user_stop = False
            self.items = []
            self.log.clear()
            self._set("CHECKING", "dang kiem tra thiet bi" + ("" if mode == "check" else " truoc khi chay"))
        threading.Thread(target=self._reader, args=(proc, mode), daemon=True).start()
        return True, "dang kiem tra" if mode == "check" else "dang kiem tra roi khoi dong"

    def _reader(self, proc: subprocess.Popen, mode: str) -> None:
        section, in_check = "", True
        assert proc.stdout is not None
        for raw in proc.stdout:
            line = ANSI.sub("", raw.rstrip("\n"))
            with self.lock:
                self.log.append(line)
                if not in_check:
                    continue
                m = CHECK_LINE.match(line)
                if m:
                    self.items.append({"level": m.group(1), "text": m.group(2), "section": section})
                    continue
                s = SECTION_LINE.match(line)
                if s and len(line) < 120:
                    section = s.group(2)
                if "KIEM TRA DAT" in line or "--force: van chay" in line:
                    in_check = False
                    if mode == "run":
                        self._set("STARTING", "dang khoi dong cac node")
        code = proc.wait()
        with self.lock:
            if self.proc is proc:
                self.proc = None
            self.exit_code = code
            n_bad = sum(1 for it in self.items if it["level"] == "HONG")
            n_warn = sum(1 for it in self.items if it["level"] == "CANH BAO")
            if self.user_stop:                  # tat chu dong: _stop_seq dat trang thai (thu tu hai luong bat ky)
                if not self.stopping:
                    self._set("STOPPED", "da tat")
                return
            if mode == "check":
                if n_bad:
                    self._set("STOPPED", f"kiem tra: {n_bad} muc HONG, {n_warn} canh bao — sua truoc khi chay")
                else:
                    self._set("STOPPED", f"kiem tra dat ({n_warn} canh bao) — bam Khoi dong")
            elif in_check:
                self._set("FAILED", f"kiem tra co {n_bad} muc HONG — khong khoi dong (xem ket qua)")
            else:
                self._set("FAILED", f"he thong tu tat (ma {code}) — xem nhat ky")

    def shutdown(self) -> Tuple[bool, str]:
        with self.lock:
            if self.stopping:
                return False, "dang tat"
            pgid = None
            if self.proc is not None and self.proc.poll() is None:
                try:
                    pgid = os.getpgid(self.proc.pid)
                except ProcessLookupError:
                    pgid = None
            elif self.ext_pgid is not None:
                pgid = self.ext_pgid
            if pgid is None:
                return False, "he thong chua chay"
            if pgid == os.getpgid(0):
                return False, "khong tat duoc: he thong chung nhom tien trinh voi web"
            self.stopping = True
            self.user_stop = True
            self._set("STOPPING", "dung xe roi tat cac node")
        threading.Thread(target=self._stop_seq, args=(pgid,), daemon=True).start()
        return True, "dang tat he thong (dung xe truoc)"

    def _stop_seq(self, pgid: int) -> None:
        ok, msg = self.app.call("estop", timeout=1.5)          # 1. dung xe TRUOC khi tat driver
        self.app.audit("-", "shutdown", ok, "/follow/stop: " + msg)
        time.sleep(self.app.stop_wait)                         # 2. cho lenh dung toi khung xe
        for sig, wait in ((signal.SIGINT, 25.0), (signal.SIGTERM, 8.0), (signal.SIGKILL, 3.0)):
            try:                                               # 3. nhu Ctrl-C ca launch
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            t_end = _now() + wait
            while _now() < t_end and _pg_alive(pgid):
                time.sleep(0.2)
            if not _pg_alive(pgid):
                break
            self.app.audit("-", "shutdown", False, f"nhom {pgid} chua tat sau {sig.name}, gui tiep")
        with self.lock:
            self.stopping = False
            self.ext_pgid = None
            self._set("STOPPED", "da tat")

    def scan_external(self, cmds: Optional[List[Tuple[int, List[str]]]] = None) -> None:
        """Tim launch he thong khong do web bat (bat tu terminal, hoac web vua khoi dong lai)."""
        with self.lock:
            if self.proc is not None or self.stopping:
                return
        found = None
        me = os.getpgid(0)
        for pid, args in (_proc_cmdlines() if cmds is None else cmds):
            if "launch" in args and any(a.endswith(LAUNCH_FILE) for a in args):
                try:
                    pg = os.getpgid(pid)
                except OSError:
                    continue
                if pg != me:
                    found = pg
                    break
        with self.lock:
            if self.proc is not None or self.stopping:
                return
            had = self.ext_pgid is not None
            self.ext_pgid = found
            if found is not None and not had:
                self.mode = "run"
                self._set("STARTING", "he thong dang chay (bat ngoai web)")
            elif found is None and had:
                self._set("STOPPED", "he thong da tat (ngoai web)")

    def tick(self, planner_fresh: bool) -> None:
        with self.lock:
            if self.state == "STARTING" and planner_fresh:
                self._set("RUNNING", "")
            elif self.state == "RUNNING" and not planner_fresh:
                self.detail = "planner khong gui trang thai — kiem tra nhat ky"
            elif self.state == "RUNNING" and planner_fresh and self.detail:
                self.detail = ""


# ─────────────────────────────────────────────────────────────────────────────
# Ghi bag
# ─────────────────────────────────────────────────────────────────────────────

class Bag:
    """Ghi bag bang "run_full.sh --bag" (cung lenh ghi tay, cung danh sach topic), nhom tien trinh rieng.
    Moi truong duoi self.lock. gen tang moi lan bat dau / nhan tien trinh ngoai -> luong cu khong ghi de."""

    def __init__(self, app: "RobotWeb", root: Path) -> None:
        self.app = app
        self.root = root                           # phai trung $HOME/bags cua run_full.sh (chi de liet ke)
        self.lock = threading.Lock()
        self.proc: Optional[subprocess.Popen] = None
        self.ext_pgid: Optional[int] = None
        self.gen = 0
        self.state = "OFF"                         # OFF RECORDING STOPPING
        self.path: Optional[Path] = None
        self.t_start = 0.0
        self.auto = False
        self.why = ""                              # ly do dung (nguoi bam / he thong tat / o dia day)
        self.topics: List[str] = []                # topic ros2 bag da dang ky
        self.log: deque = deque(maxlen=40)
        self.size = 0
        self.free: Optional[int] = None
        self.recent: List[Dict[str, Any]] = []
        self.t_recent = 0.0
        self.expected = bag_topics(app.run_script)

    def owned_recording(self) -> bool:
        with self.lock:
            return self.state == "RECORDING" and self.proc is not None

    def start(self, auto: bool = False) -> Tuple[bool, str]:
        script = self.app.run_script
        if script is None:
            return False, "khong thay run_full.sh"
        with self.lock:
            if self.state != "OFF":
                return False, "dang ghi roi" if self.state == "RECORDING" else "dang dung ghi"
            if self.free is not None and self.free < self.app.bag_min_free * 1e9:
                return False, f"o dia con {self.free / 1e9:.1f} GB (< {self.app.bag_min_free:g} GB)"
            env = dict(os.environ)
            env["PYTHONUNBUFFERED"] = "1"
            try:
                proc = subprocess.Popen(["bash", str(script), "--bag"], cwd=str(self.app.ws), env=env,
                                        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, errors="replace", bufsize=1, start_new_session=True)
            except OSError as exc:
                return False, f"khong chay duoc run_full.sh --bag: {exc}"
            self.gen += 1
            self.proc, self.ext_pgid, self.state = proc, None, "RECORDING"
            self.path, self.t_start, self.auto, self.why, self.size = None, _now(), auto, "", 0
            self.topics = []
            self.log.clear()
            gen = self.gen
        threading.Thread(target=self._reader, args=(proc, gen), daemon=True).start()
        return True, "bat dau ghi bag" + (" (tu dong)" if auto else "")

    def _reader(self, proc: subprocess.Popen, gen: int) -> None:
        assert proc.stdout is not None
        for raw in proc.stdout:
            line = ANSI.sub("", raw.rstrip("\n"))
            started = None
            with self.lock:
                self.log.append(line)
                if self.gen != gen:
                    continue
                m = BAG_LINE.search(line)
                if m and self.path is None:
                    self.path = Path(m.group(1))
                    started = {"ev": "start", "name": self.path.name, "auto": self.auto}
                for t in SUB_LINE.findall(line):
                    if t not in self.topics:
                        self.topics.append(t)
            if started:
                self.app.events.add("bag", "info", started)
        code = proc.wait()
        with self.lock:
            if self.gen != gen:
                return
            self.proc, self.state, self.t_recent = None, "OFF", 0.0
            path, why, n_top = self.path, self.why, len(self.topics)
            last = next((ln for ln in reversed(self.log) if ln.strip()), "")
        self._finish(path, code, why, last, n_top)

    def _finish(self, path: Optional[Path], code: Optional[int], why: str, last: str, n_top: Optional[int]) -> None:
        """Su kien ket thuc ghi (doc metadata.yaml ros2 bag vua ghi) + lam moi danh sach bag."""
        info = bag_info(path) if path is not None else None
        if info is not None and info["ok"]:
            self.app.events.add("bag", "ok", {"ev": "done", "name": info["name"], "dur": info["dur"],
                                              "size": info["size"], "msgs": info["msgs"], "topics": n_top, "why": why})
        elif why or code == 0:
            self.app.events.add("bag", "warn", {"ev": "done", "name": path.name if path else "?", "nometa": True,
                                                "size": info["size"] if info else 0, "why": why})
        else:
            self.app.events.add("bag", "bad", {"ev": "fail", "code": code, "last": last[-160:]})
        self.refresh(force=True)

    def stop(self, why: str) -> Tuple[bool, str]:
        with self.lock:
            if self.state != "RECORDING":
                return False, "khong dang ghi"
            pgid = None
            if self.proc is not None and self.proc.poll() is None:
                try:
                    pgid = os.getpgid(self.proc.pid)
                except ProcessLookupError:
                    pgid = None
            elif self.ext_pgid is not None:
                pgid = self.ext_pgid
            if pgid is None:
                return False, "khong thay tien trinh ghi"
            if pgid == os.getpgid(0):
                return False, "khong dung duoc: tien trinh ghi chung nhom voi web"
            self.state, self.why = "STOPPING", why
            gen = self.gen
        threading.Thread(target=self._stop_seq, args=(pgid, gen), daemon=True).start()
        return True, "dang dung ghi bag"

    def _stop_seq(self, pgid: int, gen: int) -> None:
        for sig, wait in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
            try:                                   # SIGINT = Ctrl-C: ros2 bag ghi het bo dem + metadata.yaml
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            except OSError as exc:                 # vd. khong co quyen: van dang ghi
                self.app.audit("-", "bag_stop", False, f"khong gui duoc {sig.name} toi nhom {pgid}: {exc}")
                with self.lock:
                    if self.gen == gen and self.state == "STOPPING":
                        self.state = "RECORDING"
                return
            t_end = _now() + wait
            while _now() < t_end and _pg_alive(pgid):
                time.sleep(0.1)
            if not _pg_alive(pgid):
                break
            self.app.audit("-", "bag_stop", False, f"nhom {pgid} chua dung sau {sig.name}, gui tiep")
        with self.lock:
            if self.gen != gen or self.proc is not None:
                return                             # tien trinh do web bat: _reader tong ket
            ext = self.ext_pgid == pgid
            self.ext_pgid, self.state, self.t_recent = None, "OFF", 0.0
            path, why = self.path, self.why
        if ext:
            self._finish(path, 0, why, "", None)

    def scan_external(self, cmds: List[Tuple[int, List[str]]]) -> None:
        """Tim "ros2 bag record" khong do web bat (ghi tu terminal, hoac web vua khoi dong lai giua luc ghi)."""
        with self.lock:
            if self.proc is not None or self.state == "STOPPING":
                return
            cur = self.ext_pgid
        me, uid = os.getpgid(0), os.getuid()
        found = None
        for pid, args in cmds:
            if not any(args[i] == "bag" and args[i + 1] == "record" for i in range(len(args) - 1)):
                continue
            try:
                pg = os.getpgid(pid)
                mine = os.stat(f"/proc/{pid}").st_uid == uid      # tien trinh user khac: khong dung duoc
            except OSError:
                continue
            if pg == me or not mine:
                continue
            found = (pg, pid, args)
            if pg == cur:
                break
        ended = None
        with self.lock:
            if self.proc is not None or self.state == "STOPPING":
                return
            if found is not None and found[0] != self.ext_pgid:
                pg, pid, args = found
                self.gen += 1
                self.ext_pgid, self.state = pg, "RECORDING"
                self.path = _bag_out(pid, args)
                self.t_start = _proc_start(pid) or _now()
                self.auto, self.why, self.size, self.topics = False, "", 0, []
                self.log.clear()
                started = {"ev": "start", "name": self.path.name if self.path else "?", "external": True}
            else:
                started = None
                if found is None and self.ext_pgid is not None:     # tien trinh ngoai tu ket thuc
                    self.ext_pgid, self.state, self.t_recent = None, "OFF", 0.0
                    ended = (self.path, self.why)
        if started:
            self.app.events.add("bag", "info", started)
        if ended:
            self._finish(ended[0], 0, ended[1], "", None)

    def refresh(self, force: bool = False) -> None:
        """Dung luong bag dang ghi, cho trong o dia, danh sach bag gan day (moi 2 s; danh sach moi 10 s)."""
        now = _now()
        with self.lock:
            path = self.path if self.state != "OFF" else None
            rec = self.state == "RECORDING"
            need_recent = force or now - self.t_recent > 10.0
        size = _dir_size(path)
        try:
            stv = os.statvfs(str(self.root if self.root.is_dir() else Path.home()))
            free: Optional[int] = stv.f_bavail * stv.f_frsize
        except OSError:
            free = None
        recent = None
        if need_recent:
            try:
                dirs = [p for p in self.root.iterdir() if p.is_dir()]
            except OSError:
                dirs = []

            def mtime(p: Path) -> float:
                try:
                    return p.stat().st_mtime
                except OSError:
                    return 0.0
            recent = [bag_info(p) for p in sorted(dirs, key=mtime, reverse=True)[:6]]
        with self.lock:
            if path is not None and path == self.path:
                self.size = size
            self.free = free
            if recent is not None:
                self.recent, self.t_recent = recent, now
        if rec and free is not None and free < self.app.bag_stop_free * 1e9:
            ok, msg = self.stop("o dia sap day")
            if ok:
                self.app.audit("-", "bag_stop", ok, f"tu dung: o dia con {free / 1e9:.2f} GB — {msg}")

    def snapshot(self) -> Dict[str, Any]:
        now = _now()
        with self.lock:
            on = self.state != "OFF"
            return {"state": self.state, "external": self.ext_pgid is not None, "auto": self.auto,
                    "name": self.path.name if (on and self.path) else None,
                    "path": str(self.path) if (on and self.path) else None,
                    "dur": round(now - self.t_start, 1) if on else None, "size": self.size if on else None,
                    "topics": None if (not on or self.ext_pgid is not None) else list(self.topics),
                    "expected": list(self.expected), "free": self.free, "dir": str(self.root),
                    "recent": [dict(r) for r in self.recent]}


# ─────────────────────────────────────────────────────────────────────────────
# Node ROS + may chu web
# ─────────────────────────────────────────────────────────────────────────────

class RobotWeb(Node):
    def __init__(self) -> None:
        # Khong mo service tham so (6 service it dung nhung executor phai xet moi nhip)
        super().__init__("robot_web", start_parameter_services=False)
        defaults = {
            "port": 8080,
            "host": "0.0.0.0",
            "ws_dir": "",
            "password_file": "~/.config/robot_web.conf",
            "image_width": 480,
            "image_max_fps": 15.0,
            "jpeg_quality": 65,
            "max_streams": 3,
            "stop_wait_sec": 0.5,
            "probe_period_sec": 0.25,
            "bag_dir": "~/bags",                 # PHAI trung $HOME/bags ma run_full.sh --bag ghi vao (chi de liet ke)
            "bag_min_free_gb": 1.0,              # o dia con it hon -> khong cho bat dau ghi
            "bag_stop_free_gb": 0.3,             # dang ghi ma o dia con it hon -> tu dung ghi
            "power_supply_dir": "/sys/class/power_supply",
            "chassis_power_unit": "",            # [CAN XAC NHAN] don vi /bw_dr03/power (driver goi la power_v)
            "chassis_power_warn": 0.0,           # nguong canh bao pin khung xe (0 = tat, chua biet loai pin)
            "chassis_power_bad": 0.0,
        }
        dyn = ParameterDescriptor(dynamic_typing=True)
        for k, v in defaults.items():
            self.declare_parameter(k, v, dyn)

        def g(k: str) -> Any:
            return self.get_parameter(k).value

        self.port = int(g("port"))
        self.host = str(g("host"))
        self.img_width = int(g("image_width"))
        self.img_fps = max(0.5, float(g("image_max_fps")))
        self.jpeg_q = int(g("jpeg_quality"))
        self.max_streams = max(1, int(g("max_streams")))
        self.stop_wait = max(0.0, float(g("stop_wait_sec")))
        self.ws = find_ws(str(g("ws_dir")))
        self.run_script = None if self.ws is None else self.ws / "src/person_follow_nav/scripts/run_full.sh"
        self.password, created = load_password(str(g("password_file")))
        self.pw_file = str(Path(str(g("password_file"))).expanduser())
        self.pw_created = created
        self.tokens: set = set()
        self.audit_lock = threading.Lock()
        self.log_dir = (self.ws / "run_logs") if self.ws is not None else Path.home()
        self.events = Events(self)
        self.bag_min_free = max(0.0, float(g("bag_min_free_gb")))
        self.bag_stop_free = max(0.0, float(g("bag_stop_free_gb")))
        self.ps_dir = Path(str(g("power_supply_dir")))
        self.ch_unit = str(g("chassis_power_unit"))
        self.ch_warn = float(g("chassis_power_warn"))
        self.ch_bad = float(g("chassis_power_bad"))

        self.lock = threading.Lock()
        self.rates = {k: Rate() for k in ("scan", "odom", "cmd", "planner", "identity", "image")}
        self.planner: Dict[str, Any] = {}
        self.identity: Dict[str, Any] = {}
        self.rssi_status: Dict[str, Any] = {}
        self.rssi_bearing: Dict[str, Any] = {}
        self.t_rssi = self.t_bearing = 0.0
        self.image_raw: Optional[bytes] = None      # anh camera chua giai ma (CDR)
        self.jpeg_lock = threading.Lock()
        self.jpeg: Optional[bytes] = None
        self.jpeg_src: Optional[bytes] = None
        self.t_jpeg = 0.0
        self.cmd_pubs = 0
        self.last_action: Dict[str, Any] = {}
        self.bat: Optional[Dict[str, Any]] = None
        self.chassis: deque = deque()                # (luc nhan, so doc /bw_dr03/power) trong 10 s
        self.bat_logged = 101                        # nguong pin may da bao (101 = chua bao)
        self.ch_logged = ""                          # muc pin khung xe da bao: "" / warn / bad
        self.k_tick = 0
        # Theo doi thay doi cho dong su kien (_watch): gia tri lan truoc + bo loc on dinh 2 nhip
        self.w: Dict[str, Any] = {"enabled": None, "plan": "OFF", "enrolling": None, "ready": None,
                                  "alive": False, "seen": set()}
        self.st_plan = Stable("OFF")
        self.st_stale = {k: Stable(False) for k in ("scan", "odom", "cam", "planner", "rssi")}
        self.st_pubs = Stable(False)
        self.st_beacon = Stable(False)
        self.bag_auto_pending = False                # "start" kem bag: ghi ngay khi qua kiem tra
        self.bag_stop_at = 0.0                       # he thong vua tat: dung ghi bag luc nay

        # Moi subscription nam o node PHU robot_web_probe, KHONG duoc executor chinh quay lien tuc: cu
        # probe_period_sec thi lay TIN MOI NHAT cua tung topic (QoS giu 1 tin) -> moi topic <= 4 lan xu ly/s
        # du ben kia phat 50 Hz. Topic thoi gian thuc (/scan, /odom, /cmd_vel, anh) nhan dang tho (raw=True).
        # Anh camera chi dang ky khi co nguoi dang xem (xem _tick).
        latest = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.latest_qos = latest
        self.probe = rclpy.create_node("robot_web_probe", start_parameter_services=False, enable_rosout=False)
        self.probe_ex = SingleThreadedExecutor()
        self.probe_ex.add_node(self.probe)
        self.n_cb = 0
        self.probe.create_subscription(String, "/follow/planner_status", self._planner_cb, latest)
        self.probe.create_subscription(String, "/person_reid/target", self._identity_cb, latest)
        self.probe.create_subscription(String, "/rssi/status", self._rssi_cb, latest)
        self.probe.create_subscription(String, "/rssi/bearing", self._bearing_cb, latest)
        self.probe.create_subscription(LaserScan, "/scan", self._scan_cb, latest, raw=True)
        self.probe.create_subscription(Odometry, "/odom", self._odom_cb, latest, raw=True)
        self.probe.create_subscription(Twist, "/cmd_vel", self._cmd_cb, latest, raw=True)
        self.probe.create_subscription(Float32, "/bw_dr03/power", self._power_cb, latest)
        self.image_sub = None
        self.cam_want_until = 0.0
        # Anh camera: node RIENG robot_web_video, luong rieng lay KHUNG MOI NHAT moi 1/image_max_fps (QoS giu 1
        # khung) -> chi xu ly dung so khung can gui (camera phat 15 khung x 0.9 MB/s; nhan het moi khung ton ~8 %
        # mot nhan du chi gui 8 hinh/s), va video khong chen vao executor chinh (nut DUNG / service).
        self.video = rclpy.create_node("robot_web_video", start_parameter_services=False, enable_rosout=False)
        self.video_ex = SingleThreadedExecutor()
        self.video_ex.add_node(self.video)
        # Video (MJPEG): anh nhan ngay khi camera phat (node CHINH, khong qua probe) -> moi khung moi duoc ma hoa
        # MOT lan, moi nguoi xem dung chung; moi luong tu gioi han image_max_fps.
        self.frame_cond = threading.Condition()
        self.frame_id = 0
        self.n_streams = 0
        self.sent: deque = deque(maxlen=400)
        self.closing = False
        self.srv = {k: self.create_client(Trigger, v) for k, v in SERVICES.items()}

        self.stack = Stack(self)
        self.bag = Bag(self, Path(str(g("bag_dir"))).expanduser())
        self.create_timer(float(g("probe_period_sec")), self._probe_tick)
        self.create_timer(0.5, self._tick)
        self.create_timer(2.0, self._slow_tick)

    # ── nhan topic (luong executor) ──────────────────────────────────────
    @staticmethod
    def _json(msg: String) -> Optional[Dict[str, Any]]:
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return None
        return d if isinstance(d, dict) else None

    def _planner_cb(self, msg: String, info: Any) -> None:
        self.n_cb += 1
        d = self._json(msg)
        if d is not None:
            with self.lock:
                self.planner = d
                self.rates["planner"].tick(info, _now())

    def _identity_cb(self, msg: String, info: Any) -> None:
        self.n_cb += 1
        d = self._json(msg)
        if d is not None:
            with self.lock:
                self.identity = d
                self.rates["identity"].tick(info, _now())

    def _image_cb(self, raw: bytes, info: Any) -> None:
        with self.lock:
            self.image_raw = raw
            self.rates["image"].tick(info, _now())
        with self.frame_cond:
            self.frame_id += 1
            self.frame_cond.notify_all()

    def _rssi_cb(self, msg: String, info: Any) -> None:
        self.n_cb += 1
        d = self._json(msg)
        if d is not None:
            with self.lock:
                self.rssi_status, self.t_rssi = d, _now()

    def _bearing_cb(self, msg: String, info: Any) -> None:
        self.n_cb += 1
        d = self._json(msg)
        if d is not None:
            with self.lock:
                self.rssi_bearing, self.t_bearing = d, _now()

    def _scan_cb(self, raw: bytes, info: Any) -> None:
        self.n_cb += 1
        with self.lock:
            self.rates["scan"].tick(info, _now())

    def _odom_cb(self, raw: bytes, info: Any) -> None:
        self.n_cb += 1
        with self.lock:
            self.rates["odom"].tick(info, _now())

    def _cmd_cb(self, raw: bytes, info: Any) -> None:
        self.n_cb += 1
        with self.lock:
            self.rates["cmd"].tick(info, _now())

    def _power_cb(self, msg: Float32, info: Any) -> None:
        self.n_cb += 1
        v, now = float(msg.data), _now()
        if v != v or abs(v) > 1e6:                   # NaN / so rac
            return
        with self.lock:
            self.chassis.append((now, v))
            while self.chassis and self.chassis[0][0] < now - 10.0:
                self.chassis.popleft()

    def _probe_tick(self) -> None:
        """Lay tin moi nhat cua tung topic (moi spin_once xu ly mot tin); dung khi khong con tin nao."""
        for _ in range(16):
            n0 = self.n_cb
            self.probe_ex.spin_once(timeout_sec=0.0)
            if self.n_cb == n0:
                break

    def _tick(self) -> None:
        n = self.count_publishers("/cmd_vel")
        now = _now()
        with self.lock:
            self.cmd_pubs = n
            planner_fresh = _fresh(self.rates["planner"].last, now, PLANNER_STALE)
        self.stack.tick(planner_fresh)
        if self.k_tick % 10 == 0:                    # pin: moi 5 s
            self._read_power(now)
        self.k_tick += 1
        try:
            self._watch(self.status(), now)
        except Exception as exc:  # dong su kien khong duoc lam chet nhip theo doi
            self.get_logger().warn(f"su kien: {exc!r}")

    def _slow_tick(self) -> None:
        cmds = _proc_cmdlines()
        self.stack.scan_external(cmds)
        self.bag.scan_external(cmds)
        self.bag.refresh()

    def _chassis_now(self, now: float) -> Optional[Dict[str, Any]]:
        """So doc /bw_dr03/power: trung vi 10 s (ap tut luc dong co keo). Goi duoi self.lock."""
        if not self.chassis:
            return None
        vals = sorted(v for _, v in self.chassis)
        v = vals[len(vals) // 2]
        lvl = ("bad" if self.ch_bad > 0 and v <= self.ch_bad else
               "warn" if self.ch_warn > 0 and v <= self.ch_warn else "")
        return {"v": round(v, 2), "age": round(now - self.chassis[-1][0], 2), "unit": self.ch_unit, "lvl": lvl,
                "warn": self.ch_warn, "bad": self.ch_bad}

    def _read_power(self, now: float) -> None:
        b = read_battery(self.ps_dir)
        with self.lock:
            self.bat = b
            ch = self._chassis_now(now)
        if b is not None:
            if b["status"] != "Discharging" or b["pct"] > BAT_WARN + 5:
                self.bat_logged = 101
            elif b["pct"] <= BAT_BAD < self.bat_logged:
                self.bat_logged = BAT_BAD
                self.events.add("bat", "bad", {"src": "may", "val": f"{b['pct']}%", "pct": b["pct"]})
            elif b["pct"] <= BAT_WARN < self.bat_logged:
                self.bat_logged = BAT_WARN
                self.events.add("bat", "warn", {"src": "may", "val": f"{b['pct']}%", "pct": b["pct"]})
        if ch is None or ch["age"] > CHASSIS_STALE:
            return
        rank = {"": 0, "warn": 1, "bad": 2}
        if not ch["lvl"] and ch["v"] > max(self.ch_warn, self.ch_bad) + 0.3:     # tre 0.3 tranh bao lap
            self.ch_logged = ""
        elif ch["lvl"] and rank[ch["lvl"]] > rank[self.ch_logged]:
            self.ch_logged = ch["lvl"]
            self.events.add("bat", ch["lvl"], {"src": "xe", "val": f"{ch['v']:g} {self.ch_unit}".strip(),
                                               "v": ch["v"]})

    def _watch(self, s: Dict[str, Any], now: float) -> None:
        """So voi nhip truoc -> dong su kien. Moi 0.5 s (luong executor). Trang thai chop nhoang bi loc (Stable)."""
        ev, w = self.events, self.w
        sysd, p, idt, sen, rs = s["system"], s["planner"], s["identity"], s["sensors"], s["rssi"]
        running = sysd["state"] == "RUNNING"
        pl_fresh = p["age"] is not None and p["age"] < PLANNER_STALE
        id_fresh = idt["age"] is not None and idt["age"] < IDENTITY_STALE
        enabled = pl_fresh and bool(p["enabled"])

        # 1. xe bat / ngung bam
        if pl_fresh:
            if w["enabled"] is not None and enabled != w["enabled"]:
                ev.add("follow", "ok" if enabled else "info", {"on": enabled})
            w["enabled"] = enabled
        # 2. "dot" trang thai bat thuong cua planner khi dang bam (ne, tim nguoi, bi chan, dung xe...)
        if self.st_plan.update(str(p.get("state") or "?") if enabled else "OFF"):
            old, new = w["plan"], self.st_plan.value
            w["plan"] = new
            if old not in PLAN_NORMAL:
                ev.end("plan:" + old, {"next": new})
            if new not in PLAN_NORMAL:
                ev.begin("plan:" + new, "plan", PLAN_LVL.get(new, "warn"),
                         {"state": new, "note": str(p.get("note") or "")[:120]})
        # 3. hoc nguoi: xong / huy / xoa
        if id_fresh:
            enrolling = str(idt.get("status") or "").startswith("ENROLLING")
            ready = bool(idt["ready"])
            if w["enrolling"] and not enrolling:
                ev.add("enroll", "ok" if ready else "info",
                       {"ev": "done" if ready else "cancel", "samples": idt.get("samples")})
            elif w["ready"] is False and ready and not enrolling:     # hoc xong ma chua kip thay luc dang hoc
                ev.add("enroll", "ok", {"ev": "done", "samples": idt.get("samples")})
            elif w["ready"] and not ready and not enrolling:
                ev.add("enroll", "info", {"ev": "clear"})
            w["enrolling"], w["ready"] = enrolling, ready
        # 4. cam bien / node ngung gui — chi khi he thong dang chay va da tung thay du lieu tu luc chay
        fresh_now = {"scan": sen["scan_age"] is not None and sen["scan_age"] < SCAN_STALE,
                     "odom": sen["odom_age"] is not None and sen["odom_age"] < ODOM_STALE,
                     "cam": id_fresh, "planner": pl_fresh,
                     "rssi": rs["bearing_age"] is not None and rs["bearing_age"] < RSSI_STALE}
        if not running:
            w["seen"] = set()
        for k, ok in fresh_now.items():
            if running and ok:
                w["seen"].add(k)
            if self.st_stale[k].update(running and k in w["seen"] and not ok):
                if self.st_stale[k].value:
                    ev.begin("stale:" + k, "stale", "bad", {"what": k})
                else:
                    ev.end("stale:" + k, {"next": "ok" if running else "off"})
        # 5. so node ghi /cmd_vel khac 1 (bat bien he thong)
        pubs = int(sen["cmd_pubs"])
        if self.st_pubs.update(running and pl_fresh and pubs != 1):
            if self.st_pubs.value:
                ev.begin("cmdpubs", "cmdpubs", "bad", {"n": pubs})
            else:
                ev.end("cmdpubs")
        # 6. mat tin hieu beacon (RSSI dang chay)
        if self.st_beacon.update(running and rs["beacon_ok"] is False):
            if self.st_beacon.value:
                ev.begin("beacon", "beacon", "warn")
            else:
                ev.end("beacon", {"next": "ok" if rs["beacon_ok"] else "none"})
        # 7. ghi bag theo he thong: tu ghi khi qua kiem tra (neu chon), tu dung 2 s sau khi he thong tat
        st = sysd["state"]
        if self.bag_auto_pending:
            if st in ("STARTING", "RUNNING"):
                self.bag_auto_pending = False
                ok, msg = self.bag.start(auto=True)
                self.audit("-", "bag_start", ok, "tu dong khi khoi dong: " + msg)
            elif st in ("STOPPED", "FAILED"):
                self.bag_auto_pending = False
        alive = bool(sysd["alive"]) or st == "STOPPING"
        if w["alive"] and not alive:
            self.bag_stop_at = now + 2.0
        elif alive:
            self.bag_stop_at = 0.0
        w["alive"] = alive
        if self.bag_stop_at and now >= self.bag_stop_at:
            self.bag_stop_at = 0.0
            if self.bag.owned_recording():
                ok, msg = self.bag.stop("he thong da tat")
                self.audit("-", "bag_stop", ok, "tu dung khi he thong tat: " + msg)

    def _video_loop(self) -> None:
        """Luong anh camera: chi dang ky khi co nguoi xem. Khung moi toi thi xu ly ngay; camera nhanh hon
        image_max_fps thi ngu cho du chu ky roi lay khung MOI NHAT (QoS giu 1, khung giua bi bo o tang DDS)."""
        _name_thread("rw_video")
        period = 1.0 / self.img_fps
        t_last = 0.0
        while not self.closing:
            t0 = _now()
            with self.lock:
                want = t0 < self.cam_want_until or self.n_streams > 0
            # tao / huy subscription ngay trong luong duy nhat quay node video (an toan voi rclpy)
            if want and self.image_sub is None:
                self.image_sub = self.video.create_subscription(Image, "/person_reid/debug_image", self._image_cb,
                                                                self.latest_qos, raw=True)
            elif not want and self.image_sub is not None:
                self.video.destroy_subscription(self.image_sub)
                self.image_sub = None
                with self.lock:
                    self.image_raw = None
            if self.image_sub is None:
                time.sleep(0.25)
                continue
            wait = period - (_now() - t_last)
            if wait > 0:
                time.sleep(wait)
            n0 = self.frame_id
            try:
                self.video_ex.spin_once(timeout_sec=0.25)    # co san khung moi nhat thi lay ngay, chua thi cho
            except Exception as exc:  # khong de luong video chet ngang
                self.get_logger().warn(f"video: {exc}")
            if self.frame_id != n0:
                t_last = _now()

    # ── trang thai cho trang web ─────────────────────────────────────────
    def status(self) -> Dict[str, Any]:
        now = _now()
        with self.lock:
            r = self.rates
            p, idt = dict(self.planner), dict(self.identity)
            rs, rb = dict(self.rssi_status), dict(self.rssi_bearing)
            out: Dict[str, Any] = {
                "t": round(now, 2),
                "planner": {"age": _age(r["planner"].last, now), "enabled": bool(p.get("enabled", False)),
                            "state": p.get("state"), "note": p.get("note"),
                            "dist": p.get("target_distance_m"), "source": p.get("target_source"),
                            "cmd_v": p.get("cmd_v"), "cmd_w": p.get("cmd_w")},
                "identity": {"age": _age(r["identity"].last, now), "hz": r["identity"].hz(now),
                             "status": idt.get("status"), "ready": bool(idt.get("identity_ready", False)),
                             "found": bool(idt.get("target_found", False)),
                             "samples": idt.get("enrolled_samples"), "progress": idt.get("enrollment_progress"),
                             "num_tracks": idt.get("num_tracks"), "reason": idt.get("reason")},
                "sensors": {"scan_hz": r["scan"].hz(now), "scan_age": _age(r["scan"].last, now),
                            "odom_hz": r["odom"].hz(now), "odom_age": _age(r["odom"].last, now),
                            "cmd_hz": r["cmd"].hz(now), "cmd_age": _age(r["cmd"].last, now),
                            "cmd_pubs": self.cmd_pubs, "img_hz": r["image"].hz(now),
                            "img_age": _age(r["image"].last, now)},
                "video": {"viewers": self.n_streams,
                          "fps": round(sum(1 for t in self.sent if t > now - 2.0) / 2.0 / max(1, self.n_streams), 1),
                          "max_fps": self.img_fps, "width": self.img_width,
                          "subscribed": self.image_sub is not None},
                "rssi": {"age": _age(self.t_rssi, now), "boards": rs.get("boards", {}),
                         "beacon_age": rs.get("beacon_age_sec"), "bearing_age": _age(self.t_bearing, now),
                         "beacon_ok": rb.get("beacon_ok") if _fresh(self.t_bearing, now, RSSI_STALE) else None,
                         "bearing_valid": bool(rb.get("valid", False)) if _fresh(self.t_bearing, now, RSSI_STALE) else False},
                "last_action": dict(self.last_action),
                "power": {"bat": dict(self.bat) if self.bat else None, "chassis": self._chassis_now(now)},
            }
        out["bag"] = self.bag.snapshot()
        out["ev"] = self.events.meta()
        sysd = self.stack.snapshot()
        out["system"] = {k: sysd[k] for k in ("state", "detail", "since", "alive", "external", "mode", "rssi", "exit_code")}
        out["system"]["run_script"] = self.run_script is not None
        out["check"] = sysd["items"]
        out["log"] = sysd["log"]
        out["allowed"] = self._allowed(out, now)
        return out

    def _allowed(self, s: Dict[str, Any], now: float) -> Dict[str, List[Any]]:
        sysd, p, idt, sen = s["system"], s["planner"], s["identity"], s["sensors"]
        alive, st = sysd["alive"], sysd["state"]
        pl_fresh = p["age"] is not None and p["age"] < PLANNER_STALE
        id_fresh = idt["age"] is not None and idt["age"] < IDENTITY_STALE
        enabled = pl_fresh and p["enabled"]
        idst = str(idt.get("status") or "")
        enrolling = id_fresh and idst.startswith("ENROLLING")
        scan_ok = sen["scan_age"] is not None and sen["scan_age"] < SCAN_STALE
        odom_ok = sen["odom_age"] is not None and sen["odom_age"] < ODOM_STALE
        pubs = int(sen["cmd_pubs"])

        def rule(*conds: Tuple[bool, str]) -> List[Any]:
            for ok, why in conds:
                if not ok:
                    return [False, why]
            return [True, ""]

        busy = (not alive and st not in ("CHECKING", "STOPPING"), "he thong dang chay — tat truoc"
                if alive else "dang kiem tra / dang tat")
        a = {
            "check": rule((self.run_script is not None, "khong thay run_full.sh"), busy),
            "start": rule((self.run_script is not None, "khong thay run_full.sh"), busy,
                          (not pl_fresh, "dang co planner chay (he thong khac?) — tat truoc")),
            "shutdown": rule((alive, "he thong chua chay"), (st != "STOPPING", "dang tat")),
            "enroll_start": rule((id_fresh, "camera chua gui du lieu"), (not enabled, "dang bam — tam dung truoc"),
                                 (not enrolling, "dang hoc roi")),
            "enroll_finish": rule((id_fresh, "camera chua gui du lieu"), (enrolling, "chua bat dau hoc")),
            "reset_identity": rule((id_fresh, "camera chua gui du lieu"), (not enabled, "dang bam — tam dung truoc")),
            "follow_enable": rule((pl_fresh, "planner chua chay"), (not enabled, "dang bam roi"),
                                  (id_fresh, "camera chua gui du lieu"), (not enrolling, "dang hoc nguoi — bam Xong"),
                                  (bool(idt["ready"]), "chua hoc nguoi"),
                                  (scan_ok, "LiDAR khong gui du lieu"), (odom_ok, "khung xe (/odom) khong gui du lieu"),
                                  (pubs >= 1, "khong thay node nao ghi /cmd_vel"),
                                  (pubs <= 1, f"co {pubs} node cung ghi /cmd_vel — nguy hiem, tat bot")),
            "follow_disable": [True, ""],
            "estop": [True, ""],
        }
        bag = s["bag"]
        a["bag_start"] = rule((self.run_script is not None, "khong thay run_full.sh"),
                              (bag["state"] == "OFF", "dang ghi roi" if bag["state"] == "RECORDING" else "dang dung ghi"),
                              (st in ("STARTING", "RUNNING") or pl_fresh, "he thong chua chay — khoi dong truoc"),
                              (bag["free"] is None or bag["free"] >= self.bag_min_free * 1e9,
                               f"o dia con duoi {self.bag_min_free:g} GB"))
        a["bag_stop"] = rule((bag["state"] == "RECORDING", "khong dang ghi"))
        return a

    # ── goi service (luong web; client rclpy co khoa rieng) ──────────────
    def call(self, key: str, timeout: float = 2.0) -> Tuple[bool, str]:
        cli, name = self.srv[key], SERVICES[key]
        if not cli.service_is_ready():
            return False, f"{name} chua san sang"
        fut = cli.call_async(Trigger.Request())
        t_end = _now() + timeout
        while not fut.done():
            if _now() > t_end:
                cli.remove_pending_request(fut)
                return False, f"{name} khong tra loi trong {timeout:.1f} s"
            time.sleep(0.005)
        try:
            res = fut.result()
        except Exception as exc:  # loi tu phia service
            return False, f"{name}: {exc}"
        if res is None:
            return False, f"{name}: khong co ket qua"
        return bool(res.success), str(res.message)

    def action(self, name: str, body: Dict[str, Any], who: str) -> Tuple[bool, str]:
        if name not in ACTIONS:
            return False, "nut khong ton tai"
        if name == "estop":                                    # khong bao gio chan
            ok, msg = self.call("estop", timeout=1.0)
            if not ok:
                ok2, msg2 = self.call("follow_disable", timeout=1.0)
                ok, msg = ok2, (f"{msg}; /follow/disable: {msg2}" if ok2 else
                                f"KHONG DUNG DUOC QUA WEB ({msg}; {msg2}) — dung cong tac nguon dong co")
        else:
            allowed = self._allowed(self.status(), _now()).get(name, [False, "?"])
            if not allowed[0]:
                ok, msg = False, "chua bam duoc: " + str(allowed[1])
            elif name == "check":
                ok, msg = self.stack.launch("check", True)
            elif name == "start":
                ok, msg = self.stack.launch("run", bool(body.get("rssi", True)))
                self.bag_auto_pending = ok and bool(body.get("bag", False))
                if self.bag_auto_pending:
                    msg += " (tu ghi bag khi qua kiem tra)"
            elif name == "shutdown":
                ok, msg = self.stack.shutdown()
            elif name == "bag_start":
                ok, msg = self.bag.start()
            elif name == "bag_stop":
                ok, msg = self.bag.stop("bam Dung ghi")
            else:
                ok, msg = self.call(name, timeout=3.0)
        self.audit(who, name, ok, msg)
        self.events.add("act", "bad" if name == "estop" else ("info" if ok else "warn"),
                        {"name": name, "ok": ok, "msg": msg, "who": who}, log=False)
        with self.lock:
            self.last_action = {"name": name, "ok": ok, "msg": msg, "t": round(_now(), 2)}
        return ok, msg

    def _write_log(self, line: str) -> None:
        try:
            with self.audit_lock:
                self.log_dir.mkdir(parents=True, exist_ok=True)
                with open(self.log_dir / time.strftime("web_%m%d.log"), "a", encoding="utf-8") as f:
                    f.write(line)
        except OSError:
            pass

    def audit_event(self, code: str, lvl: str, text: str) -> None:
        """Dong su kien vao CUNG file nhat ky nut bam (cot 2 = "su_kien")."""
        self._write_log(f"{time.strftime('%H:%M:%S')} {'su_kien':15s} {code:15s} {LVL_LOG.get(lvl, '-  ')} {text}\n")

    def audit(self, who: str, name: str, ok: bool, msg: str) -> None:
        self._write_log(f"{time.strftime('%H:%M:%S')} {who:15s} {name:15s} {'OK ' if ok else 'LOI'} {msg}\n")
        # rclpy cam mot dong goi log o hai muc khac nhau -> tach hai nhanh
        if ok:
            self.get_logger().info(f"[{who}] {name}: {msg}")
        else:
            self.get_logger().warn(f"[{who}] {name}: {msg}")

    # ── anh camera ───────────────────────────────────────────────────────
    def jpeg_snapshot(self) -> Optional[bytes]:
        if cv2 is None:
            return None
        now = _now()
        with self.lock:
            self.cam_want_until = now + 5.0          # co nguoi xem -> giu dang ky anh them 5 s
            raw, t = self.image_raw, self.rates["image"].last
        if raw is None or now - t > IMAGE_STALE:
            return None
        with self.jpeg_lock:
            if self.jpeg is not None and self.jpeg_src is raw:     # khung nay da ma hoa -> dung chung
                return self.jpeg
            try:
                img = image_to_bgr(deserialize_message(raw, Image), self.img_width)
            except Exception:
                return None
            if img is None:
                return None
            ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_q])
            if not ok:
                return None
            self.jpeg, self.jpeg_src, self.t_jpeg = buf.tobytes(), raw, now
            return self.jpeg

    def wait_frame(self, last_id: int, timeout: float) -> Tuple[Optional[bytes], int]:
        """Cho khung anh MOI (khac last_id), tra (JPEG, so khung). Het gio / mat anh -> (None, last_id)."""
        with self.frame_cond:
            self.frame_cond.wait_for(lambda: self.frame_id != last_id or self.closing, timeout=timeout)
            fid = self.frame_id
        if fid == last_id or self.closing:
            return None, last_id
        return self.jpeg_snapshot(), fid

    def stream_enter(self) -> bool:
        with self.lock:
            if self.n_streams >= self.max_streams:
                return False
            self.n_streams += 1
            self.cam_want_until = _now() + 5.0
            return True

    def stream_exit(self) -> None:
        with self.lock:
            self.n_streams = max(0, self.n_streams - 1)

    def note_sent(self) -> None:
        with self.lock:
            self.sent.append(_now())

    # ── may chu web ──────────────────────────────────────────────────────
    def serve(self) -> None:
        app = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "robot_web/0.1"

            def log_message(self, fmt: str, *args: Any) -> None:  # tat log moi request
                pass

            def _send(self, code: int, body: bytes, ctype: str, extra: Optional[Dict[str, str]] = None) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.end_headers()
                if self.command != "HEAD":
                    self.wfile.write(body)

            def _json(self, code: int, obj: Any, extra: Optional[Dict[str, str]] = None) -> None:
                self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                           "application/json; charset=utf-8", extra)

            def _authed(self) -> bool:
                c = SimpleCookie()
                try:
                    c.load(self.headers.get("Cookie", ""))
                except Exception:
                    return False
                tok = c.get("rw_token")
                return tok is not None and tok.value in app.tokens

            def _body(self) -> Dict[str, Any]:
                try:
                    n = min(int(self.headers.get("Content-Length", "0") or 0), 4096)
                    d = json.loads(self.rfile.read(n).decode("utf-8") or "{}") if n > 0 else {}
                except (ValueError, UnicodeDecodeError):
                    return {}
                return d if isinstance(d, dict) else {}

            def do_GET(self) -> None:
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    try:
                        body = (STATIC_DIR / "index.html").read_bytes()
                    except OSError:
                        return self._send(500, b"thieu static/index.html", "text/plain")
                    return self._send(200, body, "text/html; charset=utf-8")
                if not path.startswith("/api/"):
                    return self._send(404, b"not found", "text/plain")
                if not self._authed():
                    return self._json(401, {"ok": False, "msg": "can dang nhap"})
                if path == "/api/status":
                    return self._json(200, app.status())
                if path == "/api/events":
                    return self._json(200, app.events.snapshot())
                if path == "/api/camera.mjpg":
                    return self._stream()
                if path == "/api/camera.jpg":
                    jpg = app.jpeg_snapshot()
                    if jpg is None:
                        return self._send(204, b"", "image/jpeg")
                    return self._send(200, jpg, "image/jpeg")
                if path == "/api/log":
                    q = parse_qs(urlparse(self.path).query)
                    n = max(1, min(400, int((q.get("n") or ["120"])[0] or 120)))
                    return self._json(200, {"lines": app.stack.log_lines(n)})
                return self._send(404, b"not found", "text/plain")

            def do_POST(self) -> None:
                path = urlparse(self.path).path
                if self.headers.get("X-Robot-Web") != "1":       # chan trang web khac goi ho (CSRF)
                    return self._json(403, {"ok": False, "msg": "thieu header X-Robot-Web"})
                who = self.client_address[0]
                if path == "/api/login":
                    pw = str(self._body().get("password", ""))
                    if secrets.compare_digest(pw.encode("utf-8"), app.password.encode("utf-8")):
                        tok = secrets.token_urlsafe(24)
                        app.tokens.add(tok)
                        app.audit(who, "login", True, "dang nhap")
                        return self._json(200, {"ok": True, "msg": "da dang nhap"}, {
                            "Set-Cookie": f"rw_token={tok}; Path=/; HttpOnly; SameSite=Strict; Max-Age=2592000"})
                    time.sleep(1.0)                              # cham doan mat khau
                    app.audit(who, "login", False, "sai mat khau")
                    return self._json(403, {"ok": False, "msg": "sai mat khau"})
                if path == "/api/action/estop":
                    # DUNG khong can dang nhap: dung xe luon an toan, va khong bao gio bi khoa ngoai nut dung
                    # (vd. web vua khoi dong lai -> phien cu het han)
                    ok, msg = app.action("estop", {}, who)
                    return self._json(200, {"ok": ok, "msg": msg})
                if not self._authed():
                    return self._json(401, {"ok": False, "msg": "can dang nhap"})
                if path.startswith("/api/action/"):
                    name = path[len("/api/action/"):]
                    ok, msg = app.action(name, self._body(), who)
                    return self._json(200, {"ok": ok, "msg": msg})
                return self._json(404, {"ok": False, "msg": "khong co"})

            def _client_gone(self) -> bool:
                """Trinh duyet da dong ket noi? (doc thu 1 byte, khong lay ra)."""
                try:
                    r, _, _ = select.select([self.connection], [], [], 0)
                    return bool(r) and self.connection.recv(1, socket.MSG_PEEK) == b""
                except OSError:
                    return True

            def _stream(self) -> None:
                """Video MJPEG (multipart/x-mixed-replace): day khung ngay khi camera co khung moi, <= image_max_fps."""
                if not app.stream_enter():
                    return self._send(503, b"qua nhieu nguoi dang xem video", "text/plain")
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Connection", "close")
                    self.end_headers()
                    _name_thread("rw_stream")
                    last_id = -1
                    while not app.closing:                 # luong video da gioi han image_max_fps
                        jpg, fid = app.wait_frame(last_id, 2.0)
                        if jpg is None:
                            if self._client_gone():          # khong co anh moi: van phai biet khi nguoi xem roi di
                                break
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                         % len(jpg) + jpg + b"\r\n")
                        self.wfile.flush()
                        last_id = fid
                        app.note_sent()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    app.stream_exit()

            do_HEAD = do_GET

        threading.Thread(target=self._video_loop, daemon=True, name="robot_web_video").start()
        httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        httpd.daemon_threads = True
        self.httpd = httpd
        ips = []
        try:
            ips = [a for a in subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=3)
                   .stdout.split() if ":" not in a]
        except (OSError, subprocess.SubprocessError):
            pass
        urls = ", ".join(f"http://{ip}:{self.port}" for ip in ips) or f"http://<ip may nay>:{self.port}"
        self.get_logger().info(f"Trang dieu khien: {urls}")
        if self.pw_created:
            self.get_logger().warn(f"Mat khau trang web (moi tao): {self.password} — luu trong {self.pw_file}")
        else:
            self.get_logger().info(f"Mat khau trang web: xem {self.pw_file}")
        if self.ws is None:
            self.get_logger().warn("Khong thay workspace (run_full.sh) — nut Kiem tra / Khoi dong bi tat. Dat -p ws_dir:=...")
        httpd.serve_forever(poll_interval=0.5)


def _sigterm(signum: int, frame: Any) -> None:
    raise KeyboardInterrupt


def main(args: Optional[List[str]] = None) -> None:
    # Tu xu ly Ctrl-C / SIGTERM (systemd): bo xu ly tin hieu cua rclpy dung ngang luong executor
    # -> "terminate called without an active exception". O day tat theo thu tu: web -> executor -> node.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _sigterm)
    # Dat lai SIGINT: chay nen bang "&" thi SIGINT bi BO QUA va duoc truyen xuong ca run_full.sh -> nut Tat
    # he thong gui SIGINT khong tac dung, phai cho 25 s moi sang SIGTERM. Co handler rieng thi tien trinh con
    # (exec) tro ve mac dinh.
    signal.signal(signal.SIGINT, _sigterm)
    node = RobotWeb()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    def spin() -> None:
        _name_thread("rw_exec")
        ex.spin()

    th = threading.Thread(target=spin, daemon=True)
    th.start()
    try:
        node.serve()
    except KeyboardInterrupt:
        pass
    finally:
        # KHONG tat he thong robot o day: no chay o nhom tien trinh rieng, tat ngang khi xe dang chay la nguy hiem
        node.closing = True
        with node.frame_cond:
            node.frame_cond.notify_all()
        httpd = getattr(node, "httpd", None)
        if httpd is not None:
            httpd.server_close()
        ex.shutdown(timeout_sec=2.0)
        th.join(timeout=2.0)
        node.probe_ex.shutdown(timeout_sec=1.0)
        node.probe.destroy_node()
        time.sleep(0.3)                                  # luong video thay closing va dung spin
        node.video_ex.shutdown(timeout_sec=1.0)
        node.video.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
