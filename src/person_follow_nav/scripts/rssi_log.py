#!/usr/bin/env python3
"""
rssi_log.py — ghi mau RSSI THO tu 3 board quet (firmware robot_rssi/src/scanner), KHONG can ROS.

Giai doan R0 cua ke hoach RSSI (30/09): do so mau/giay, do nhieu tung board, do tuong phan
do than xe che va offset giua cac board — TRUOC khi viet bo uoc luong huong. Moi nguong ve
sau lay tu so do o day.

CACH DUNG
---------
  # Tim cong cua 3 board (by-path theo cong USB vat ly — by-id cua NodeMCU va LiDAR
  # trung nhau vi cung chip CH340):
  ls -l /dev/serial/by-path/

  # Ghi 60 s, beacon dung yen cach xe 1.5 m, thang truoc mui:
  python3 rssi_log.py --ports /dev/serial/by-path/AAA /dev/serial/by-path/BBB \\
      /dev/serial/by-path/CCC --sec 60 --out truoc_1m5.csv

  # So sanh nhieu lan ghi (vd. 4 huong truoc/trai/sau/phai):
  python3 rssi_log.py --summary truoc_1m5.csv trai_1m5.csv sau_1m5.csv phai_1m5.csv

Board tu xung danh tinh A/B/C trong tung dong nen thu tu cong khong quan trong.

AN TOAN
-------
Script TU CHOI mo cong dang bi tien trinh khac giu. LiDAR SC-Mini va NodeMCU cung chip
CH340: truyen nham cong LiDAR ma van mo thi hai tien trinh chia nhau du lieu -> sc_mini
mat goi, planner thieu /scan.
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import queue
import sys
import threading
import time
from typing import Dict, List, Tuple

import numpy as np

try:
    import serial
except ImportError:
    serial = None  # type: ignore

G = "\033[92m"
R = "\033[91m"
Y = "\033[93m"
B = "\033[1m"
X = "\033[0m"

CSV_HEADER = ["t_pc", "id", "seq", "ms", "rssi", "port"]


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


def reader(port: str, baud: int, out_q: "queue.Queue", stop: threading.Event) -> None:
    """Doc mot cong, day (t_pc, id, seq, ms, rssi, port) hoac ban tin I/H vao hang doi."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.5
    # Nha DTR/RTS truoc khi mo: mach tu reset cua ESP32 noi vao hai chan nay
    ser.dtr = False
    ser.rts = False
    try:
        ser.open()
    except Exception as exc:
        out_q.put(("ERR", port, f"khong mo duoc: {exc}"))
        return
    buf = b""
    while not stop.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except Exception as exc:
            out_q.put(("ERR", port, f"loi doc: {exc}"))
            break
        if not chunk:
            continue
        t = time.time()
        buf += chunk
        # Doc HET cac dong co san, khong xoa bo dem (xoa bo dem la vut dung du lieu moi nhat)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            parts = line.decode("ascii", errors="replace").strip().split(",")
            if parts[0] == "R" and len(parts) == 5:
                try:
                    out_q.put(("R", t, parts[1], int(parts[2]), int(parts[3]), int(parts[4]), port))
                except ValueError:
                    pass
            elif parts[0] in ("I", "H"):
                out_q.put((parts[0], port, parts[1:]))
    ser.close()


def summarize(rows: List[tuple], title: str = "") -> Dict[str, dict]:
    """rows: (t_pc, id, seq, ms, rssi, port). In bang thong ke tung board, tra ve dict."""
    by_id: Dict[str, dict] = {}
    for sid in sorted({r[1] for r in rows}):
        rr = [r for r in rows if r[1] == sid]
        t = np.array([r[0] for r in rr])
        seq = np.array([r[2] for r in rr])
        rssi = np.array([r[4] for r in rr], dtype=float)
        dur = max(1e-3, t[-1] - t[0]) if len(t) > 1 else 1e-3
        dseq = np.diff(seq)
        lost = int(np.sum(dseq[dseq > 1] - 1))          # seq giam = board reset, bo qua
        resets = int(np.sum(dseq < 0))
        gaps = np.diff(t)
        by_id[sid] = {
            "n": len(rr), "hz": len(rr) / dur, "lost": lost, "resets": resets,
            "gap_max": float(gaps.max()) if len(gaps) else 0.0,
            "med": float(np.median(rssi)), "mean": float(rssi.mean()), "std": float(rssi.std()),
            "p10": float(np.percentile(rssi, 10)), "p90": float(np.percentile(rssi, 90)),
        }
    if title:
        print(f"\n{B}{title}{X}")
    print(f"  {'id':<3}{'mau':>6}{'mau/s':>8}{'mat':>6}{'ho max':>8}"
          f"{'trung vi':>10}{'TB':>8}{'do lech':>9}{'p10':>7}{'p90':>7}")
    for sid, s in by_id.items():
        hz_col = G if s["hz"] >= 10 else R
        print(f"  {sid:<3}{s['n']:>6}{hz_col}{s['hz']:>8.1f}{X}{s['lost']:>6}{s['gap_max']:>7.2f}s"
              f"{s['med']:>10.1f}{s['mean']:>8.1f}{s['std']:>9.2f}{s['p10']:>7.0f}{s['p90']:>7.0f}"
              + (f"  {Y}(reset {s['resets']} lan){X}" if s["resets"] else ""))
    ids = list(by_id)
    if len(ids) >= 2:
        diffs = []
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = ids[i], ids[j]
                diffs.append(f"{a}-{b} = {by_id[a]['med'] - by_id[b]['med']:+.1f} dB")
        print("  Chenh trung vi: " + ",  ".join(diffs))
    return by_id


def load_csv(path: str) -> List[tuple]:
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append((float(r["t_pc"]), r["id"], int(r["seq"]), int(r["ms"]), int(r["rssi"]), r["port"]))
    return rows


def record(args) -> int:
    if serial is None:
        print(f"{R}Chua cai pyserial: pip install pyserial{X}")
        return 1
    for p in args.ports:
        if not os.path.exists(p):
            print(f"{R}Khong co cong {p}{X}  -> ls -l /dev/serial/by-path/")
            return 1
        users = port_users(p)
        if users:
            print(f"{R}Cong {p} ({os.path.realpath(p)}) dang bi tien trinh khac giu:{X}")
            for pid, cmd in users:
                print(f"    PID {pid}: {cmd}")
            print("  -> Co the day la LiDAR (cung chip CH340). Kiem tra lai cong, KHONG mo chung.")
            return 1

    q: "queue.Queue" = queue.Queue()
    stop = threading.Event()
    threads = [threading.Thread(target=reader, args=(p, args.baud, q, stop), daemon=True)
               for p in args.ports]
    for th in threads:
        th.start()

    # Khong ghi de lan do cu (30/09: chay lai cung ten da xoa mat ban S1 60 s)
    if os.path.exists(args.out):
        stem, ext = os.path.splitext(args.out)
        k = 2
        while os.path.exists(f"{stem}_{k}{ext}"):
            k += 1
        print(f"{Y}{args.out} da co — ghi vao {stem}_{k}{ext}{X}")
        args.out = f"{stem}_{k}{ext}"
    out = open(args.out, "w", newline="")
    w = csv.writer(out)
    w.writerow(CSV_HEADER)
    rows: List[tuple] = []
    port_id: Dict[str, str] = {}
    alive: Dict[str, str] = {}          # cong -> id: board da gui bat ky dong hop le nao (I/H/R)
    last_h: Dict[str, list] = {}
    t0 = time.time()
    last_print = t0
    print(f"Dang ghi vao {args.out} — Ctrl-C de dung som"
          + (f", tu dung sau {args.sec:.0f} s" if args.sec > 0 else ""))
    try:
        while args.sec <= 0 or (time.time() - t0) < args.sec:
            try:
                m = q.get(timeout=0.2)
            except queue.Empty:
                m = None
            if m is not None:
                if m[0] == "R":
                    _, t, sid, seq, ms, rssi, port = m
                    rows.append((t, sid, seq, ms, rssi, port))
                    w.writerow([f"{t:.4f}", sid, seq, ms, rssi, port])
                    if port_id.get(port, sid) != sid:
                        print(f"{Y}Cong {port} doi danh tinh {port_id[port]} -> {sid}?{X}")
                    port_id[port] = sid
                    alive[port] = sid
                elif m[0] == "I":
                    print(f"  {m[1]}: board {','.join(m[2])} khoi dong")
                    alive[m[1]] = m[2][0]
                elif m[0] == "H":
                    last_h[m[2][0]] = m[2]
                    alive[m[1]] = m[2][0]
                elif m[0] == "ERR":
                    print(f"{R}  {m[1]}: {m[2]}{X}")
            now = time.time()
            if now - last_print >= 2.0:
                last_print = now
                out.flush()
                recent = [r for r in rows if r[0] > now - 2.0]
                parts = []
                for sid in sorted(set(alive.values())):
                    rr = [r[4] for r in recent if r[1] == sid]
                    drop = last_h.get(sid, [sid, 0, 0, 0])[3]
                    parts.append(f"{sid}: {len(rr) / 2.0:4.1f} mau/s "
                                 + (f"TV {np.median(rr):5.1f} dBm" if rr else "  -- khong thay beacon")
                                 + (f" {Y}(bo {drop}){X}" if str(drop) != "0" else ""))
                silent = [p for p in args.ports if p not in alive]
                if silent and now - t0 > 3.0:
                    parts.append(f"{R}chua co dong nao tu: {' '.join(silent)}{X}")
                print(f"  t={now - t0:5.1f}s  " + " | ".join(parts))
    except KeyboardInterrupt:
        pass
    stop.set()
    out.close()

    ids = set(port_id.values())
    if len(ids) < len(port_id):
        print(f"{R}Hai cong cung xung mot danh tinh — nap nham env (scan_a/b/c)?{X}")
    if not rows:
        if alive:
            print(f"{R}Board {sorted(set(alive.values()))} dang chay nhung KHONG THAY BEACON.{X} "
                  "Beacon da cap nguon chua (sac du phong co tu ngat)? Co o gan khong?")
        else:
            print(f"{R}Khong nhan duoc dong nao tu cac cong.{X} Dung firmware src/scanner? Baud {args.baud}? Dung cong?")
        return 1
    summarize(rows, f"KET QUA {args.out}  ({len(rows)} mau)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Ghi / tom tat mau RSSI tho tu 3 board quet")
    ap.add_argument("--ports", nargs="+", help="cong serial cua cac board quet (by-path)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sec", type=float, default=60.0, help="thoi gian ghi, <= 0 = toi khi Ctrl-C")
    ap.add_argument("--out", default=time.strftime("rssi_%Y%m%d_%H%M%S.csv"))
    ap.add_argument("--summary", nargs="+", metavar="CSV", help="chi tom tat cac file da ghi")
    args = ap.parse_args()

    if args.summary:
        for path in args.summary:
            summarize(load_csv(path), path)
        return 0
    if not args.ports:
        ap.error("can --ports (hoac --summary)")
    return record(args)


if __name__ == "__main__":
    sys.exit(main())
