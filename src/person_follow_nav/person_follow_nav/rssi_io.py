"""
rssi_io.py — doc cong serial cua cac board quet RSSI (firmware robot_rssi/src/scanner).

Khong phu thuoc ROS. Dung cho rssi_scanner_node.

Moi board in mot dong moi lan thay beacon:  R,<id>,<seq>,<ms>,<rssi>   (id = A/B/C)
va moi giay mot dong:                        H,<id>,<ms>,<n>,<drop>
Board TU XUNG danh tinh trong tung dong -> thu tu cong khong quan trong.

AN TOAN: KHONG mo cong dang bi tien trinh khac giu. LiDAR SC-Mini va cac board deu la chip CH340, ten by-id
trung nhau; mo nham cong LiDAR thi hai tien trinh chia nhau du lieu -> sc_mini mat goi, planner thieu /scan.
"""

from __future__ import annotations

import glob
import os
import threading
import time
from typing import Callable, List, Optional, Tuple

try:
    import serial
except ImportError:
    serial = None  # type: ignore


def port_users(dev: str) -> List[Tuple[int, str]]:
    """Cac tien trinh KHAC dang mo thiet bi `dev` (quet /proc/*/fd)."""
    real = os.path.realpath(dev)
    me = os.getpid()
    users = []
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        try:
            pid = int(fd_dir.split("/")[2])
        except ValueError:
            continue
        if pid == me:
            continue
        try:
            for fd in os.listdir(fd_dir):
                try:
                    if os.readlink(os.path.join(fd_dir, fd)) == real:
                        with open(f"/proc/{pid}/cmdline", "rb") as f:
                            cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
                        users.append((pid, cmd[:80]))
                        break
                except OSError:
                    continue
        except OSError:
            continue    # tien trinh cua user khac / da thoat
    return users


def parse_line(line: bytes) -> Optional[tuple]:
    """Dong tho -> ("R", id, seq, rssi) | ("H", id, n, drop) | ("I", id) | None."""
    parts = line.decode("ascii", errors="replace").strip().split(",")
    try:
        if parts[0] == "R" and len(parts) == 5:
            return ("R", parts[1], int(parts[2]), int(parts[4]))
        if parts[0] == "H" and len(parts) == 5:
            return ("H", parts[1], int(parts[3]), int(parts[4]))
        if parts[0] == "I" and len(parts) >= 2:
            return ("I", parts[1])
    except ValueError:
        return None
    return None


class BoardReader(threading.Thread):
    """Mot luong doc mot cong. Tu ket noi lai khi rut/cam; khong bao gio mo cong dang bi giu.

    on_sample(t, board_id, rssi, seq, port)  — goi trong luong doc, phai nhanh va an toan luong
    on_event(port, text)                     — doi trang thai (mo duoc / loi / dang bi giu...), chi goi khi DOI
    """

    def __init__(self, port: str, baud: int, on_sample: Callable, on_event: Callable,
                 stop: threading.Event, retry_sec: float = 2.0) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = int(baud)
        self.on_sample = on_sample
        self.on_event = on_event
        self.stop = stop
        self.retry_sec = float(retry_sec)
        self._state = ""

    def _event(self, text: str) -> None:
        if text != self._state:           # chi bao khi trang thai doi, khong lap lai moi 2 s
            self._state = text
            try:
                self.on_event(self.port, text)
            except Exception:
                pass                      # vd. ROS dang tat nen khong ghi log duoc — luong doc khong duoc chet vi the

    def run(self) -> None:
        if serial is None:
            self._event("chua cai pyserial (sudo apt install python3-serial)")
            return
        while not self.stop.is_set():
            if not os.path.exists(self.port):
                self._event("khong co cong (chua cam / sai duong dan)")
                self.stop.wait(self.retry_sec)
                continue
            users = port_users(self.port)
            if users:
                self._event(f"dang bi tien trinh khac giu (PID {users[0][0]}: {users[0][1][:40]}) — KHONG mo")
                self.stop.wait(self.retry_sec)
                continue
            ser = serial.Serial()
            ser.port = self.port
            ser.baudrate = self.baud
            ser.timeout = 0.5
            # Nha DTR/RTS truoc khi mo: mach tu reset cua ESP32 noi vao hai chan nay
            ser.dtr = False
            ser.rts = False
            try:
                ser.open()
            except Exception as exc:
                self._event(f"khong mo duoc: {exc}")
                self.stop.wait(self.retry_sec)
                continue
            self._event("da mo")
            buf = b""
            try:
                while not self.stop.is_set():
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    t = time.time()
                    buf += chunk
                    # Doc HET cac dong co san; khong xoa bo dem (xoa bo dem la vut dung du lieu moi nhat)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        m = parse_line(line)
                        if m is None:
                            continue
                        if m[0] == "R":
                            self.on_sample(t, m[1], m[3], m[2], self.port)
                        elif m[0] == "I":
                            self._event(f"board {m[1]} khoi dong")
                    if len(buf) > 4096:       # rac khong co xuong dong (sai baud / sai thiet bi)
                        buf = b""
            except Exception as exc:
                self._event(f"loi doc: {exc}")
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
            self.stop.wait(self.retry_sec)
