#!/usr/bin/env python3
"""
setup_udev.py — Tạo lại udev rules cho máy HIỆN TẠI.

TẠI SAO CẦN FILE NÀY
--------------------
File `99-robot-usb.rules` cũ trong repo dùng đường dẫn cổng USB VẬT LÝ:

    SUBSYSTEM=="tty", KERNELS=="1-2.3", ... SYMLINK+="robot_lidar"
                      ^^^^^^^^^^^^^^^^

Chuỗi "1-2.3" là số hiệu cổng USB trên bo mạch của MÁY CŨ. Máy Lenovo Legion
R9000P của bạn có topology USB khác hoàn toàn, nên không rule nào khớp
=> /dev/robot_lidar không được tạo ra => sc_mini báo "Can't access port".

Lý do phải dùng KERNELS thay vì serial: Lidar SC-Mini và mạch RSSI NodeMCU
đều là chip CH340 (1a86:7523) và KHÔNG có serial riêng — đúng như ghi chú
"vì các thiết bị CH340 không có serial duy nhất" trong file rules cũ của bạn.
Chỉ có BW-DR03 (FTDI FT231X) là có serial thật: D30BF0NT.

`setup_usb_rules.py` cũ không xử lý được ca này: nó ưu tiên ATTRS{serial},
mà hai CH340 lại trùng serial, nên sinh ra rule xung đột.

CÁCH DÙNG
---------
  1. Cắm cả 3 thiết bị vào (Lidar, BW-DR03, NodeMCU RSSI).
  2. python3 setup_udev.py
  3. Làm theo hướng dẫn: rút và cắm lại từng thiết bị để script nhận diện.
  4. sudo cp /tmp/99-robot-usb.rules /etc/udev/rules.d/
     sudo udevadm control --reload-rules && sudo udevadm trigger

LƯU Ý QUAN TRỌNG
----------------
Với thiết bị phải nhận diện bằng cổng vật lý, sau này BẮT BUỘC cắm đúng cổng
USB đó. Đổi cổng là symlink biến mất. Nên dán nhãn lên cổng USB trên máy.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from typing import Dict, List, Optional

G, Y, R, B, BD, X = "\033[92m", "\033[93m", "\033[91m", "\033[94m", "\033[1m", "\033[0m"


def sh(cmd: str, timeout: float = 5.0) -> str:
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except Exception:
        return ""


def list_tty() -> List[str]:
    out = []
    for d in sorted(os.listdir("/dev")):
        if d.startswith("ttyUSB") or d.startswith("ttyACM"):
            out.append("/dev/" + d)
    return sorted(out)


def props(port: str) -> Dict[str, str]:
    """Thuộc tính udev của một cổng."""
    d: Dict[str, str] = {}
    for line in sh(f"udevadm info -q property -n {port}").split("\n"):
        if "=" in line:
            k, _, v = line.partition("=")
            d[k.strip()] = v.strip()
    # KERNELS = đường dẫn cổng USB vật lý, lấy từ dạng cây
    raw = sh(f"udevadm info -a -n {port}")
    for line in raw.split("\n"):
        line = line.strip()
        if line.startswith("KERNELS==") and "usb" not in line and ":" not in line:
            val = line.split("==", 1)[1].strip().strip('"')
            # dạng hợp lệ: 1-2.3 , 3-1 , 1-2.2.4 ...
            if val and val[0].isdigit() and "-" in val:
                d.setdefault("KERNELS", val)
    return d


def describe(port: str, p: Dict[str, str]) -> str:
    vid = p.get("ID_VENDOR_ID", "?")
    pid = p.get("ID_MODEL_ID", "?")
    ser = p.get("ID_SERIAL_SHORT", "")
    ven = p.get("ID_VENDOR", "?").replace("_", " ")
    mdl = p.get("ID_MODEL", "?").replace("_", " ")
    ker = p.get("KERNELS", "?")
    s = f"{port}  │  {ven} {mdl}  │  {vid}:{pid}"
    s += f"  │  serial: {ser if ser else R + 'KHÔNG CÓ' + X}"
    s += f"  │  cổng vật lý: {ker}"
    return s


def snapshot() -> Dict[str, Dict[str, str]]:
    return {p: props(p) for p in list_tty()}


def identify_by_replug(label: str, hint: str) -> Optional[Dict[str, str]]:
    """Nhận diện thiết bị bằng cách rút ra rồi cắm lại — cách chắc chắn nhất."""
    print(f"\n{BD}{B}── Nhận diện: {label} ──{X}")
    print(f"   {hint}")
    input(f"   {Y}RÚT{X} {label} ra khỏi máy, rồi nhấn Enter...")
    before = set(list_tty())
    time.sleep(0.5)
    input(f"   {G}CẮM LẠI{X} {label} (vào đúng cổng bạn muốn dùng lâu dài), rồi nhấn Enter...")
    time.sleep(2.0)
    after = set(list_tty())
    new = sorted(after - before)
    if not new:
        print(f"   {R}Không thấy cổng mới xuất hiện. Bỏ qua thiết bị này.{X}")
        return None
    if len(new) > 1:
        print(f"   {Y}Thấy nhiều cổng mới: {new} — lấy cổng đầu tiên.{X}")
    port = new[0]
    p = props(port)
    p["_port"] = port
    print(f"   {G}✓{X} {describe(port, p)}")
    return p


def make_rule(symlink: str, p: Dict[str, str]) -> Optional[str]:
    """Sinh 1 dòng rule. Ưu tiên serial (ổn định nhất), không có thì dùng cổng vật lý."""
    vid = p.get("ID_VENDOR_ID")
    pid = p.get("ID_MODEL_ID")
    ser = p.get("ID_SERIAL_SHORT", "")
    ker = p.get("KERNELS")

    if ser and len(ser) >= 6 and ser not in ("0", "0000"):
        return (f'SUBSYSTEM=="tty", ATTRS{{serial}}=="{ser}", '
                f'SYMLINK+="{symlink}", MODE="0666"')

    if ker and vid and pid:
        return (f'SUBSYSTEM=="tty", KERNELS=="{ker}", '
                f'ATTRS{{idVendor}}=="{vid}", ATTRS{{idProduct}}=="{pid}", '
                f'SYMLINK+="{symlink}", MODE="0666"')

    return None


def main() -> None:
    print(f"\n{BD}{G}{'='*64}{X}")
    print(f"{BD}{G}  TẠO LẠI UDEV RULES CHO MÁY NÀY{X}")
    print(f"{BD}{G}{'='*64}{X}")

    ports = list_tty()
    if not ports:
        print(f"\n{R}Không tìm thấy cổng USB serial nào.{X}")
        print("Kiểm tra: cắm thiết bị chưa? dây USB có truyền dữ liệu không (không phải dây sạc)?")
        print("Xem kernel có nhận không:  sudo dmesg | tail -30")
        sys.exit(1)

    print(f"\n{G}Đang thấy {len(ports)} cổng:{X}")
    snap = snapshot()
    for p in ports:
        print("   " + describe(p, snap[p]))

    dup = {}
    for p, d in snap.items():
        key = (d.get("ID_VENDOR_ID"), d.get("ID_MODEL_ID"), d.get("ID_SERIAL_SHORT", ""))
        dup.setdefault(key, []).append(p)
    for key, plist in dup.items():
        if len(plist) > 1:
            print(f"\n{Y}Lưu ý: {len(plist)} thiết bị trùng hoàn toàn VID:PID:serial "
                  f"({key[0]}:{key[1]}) — {', '.join(plist)}{X}")
            print(f"{Y}Nhóm này bắt buộc phân biệt bằng cổng USB vật lý.{X}")

    devices = [
        ("robot_lidar", "Lidar SC-Mini", "Thường là CH340 (1a86:7523), không có serial"),
        ("robot_base", "Khung xe BW-DR03", "FTDI FT231X (0403:6015), có serial riêng"),
        ("robot_rssi", "Mạch RSSI NodeMCU", "CH340 (1a86:7523), không có serial"),
    ]

    rules: List[str] = []
    picked: Dict[str, str] = {}
    for symlink, label, hint in devices:
        info = identify_by_replug(label, hint)
        if info is None:
            continue
        rule = make_rule(symlink, info)
        if rule is None:
            print(f"   {R}Không đủ thông tin để tạo rule cho {label}.{X}")
            continue
        rules.append(f"# {label}\n{rule}")
        picked[symlink] = info["_port"]

    if not rules:
        print(f"\n{R}Không tạo được rule nào.{X}")
        sys.exit(1)

    content = (
        "# Robot USB Persistent Port Rules\n"
        "# Tạo bởi setup_udev.py cho máy hiện tại.\n"
        "# CẢNH BÁO: các rule dùng KERNELS== phụ thuộc CỔNG USB VẬT LÝ.\n"
        "#           Cắm sang cổng khác là symlink biến mất. Hãy dán nhãn cổng.\n\n"
        + "\n\n".join(rules) + "\n"
    )

    out = "/tmp/99-robot-usb.rules"
    with open(out, "w") as f:
        f.write(content)

    print(f"\n{BD}── Nội dung rules đã ghi ra {out} ──{X}")
    print(B + content + X)
    print(f"{BD}Áp dụng bằng lệnh:{X}")
    print(f"""
  sudo cp {out} /etc/udev/rules.d/99-robot-usb.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  sleep 2 && ls -l /dev/robot_*
""")
    print(f"{BD}Nếu vẫn không thấy symlink, rút cắm lại thiết bị một lần nữa.{X}\n")


if __name__ == "__main__":
    main()
