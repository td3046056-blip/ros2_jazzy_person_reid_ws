#!/usr/bin/env python3
"""
setup_usb_rules.py — Tự động tạo udev rules để cố định cổng USB cho hệ thống robot.

Cách dùng:
  1. Cắm TẤT CẢ thiết bị USB vào (Lidar, Robot BW-DR03, RSSI NodeMCU).
  2. Chạy script này: python3 setup_usb_rules.py
  3. Làm theo hướng dẫn trên màn hình.

Sau khi chạy xong:
  /dev/robot_lidar  → Lidar SC-Mini
  /dev/robot_base   → Khung xe BW-DR03
  /dev/robot_rssi   → Mạch RSSI NodeMCU
"""

import subprocess
import os
import sys


# Màu sắc terminal
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
BLUE   = "\033[94m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


def run(cmd: str) -> str:
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.stdout.strip()
    except Exception:
        return ""


def get_usb_info(port: str) -> dict:
    """Lấy thông tin nhận dạng của thiết bị USB ở cổng port."""
    info = {}
    raw = run(f"udevadm info -a -n {port}")
    for line in raw.split('\n'):
        line = line.strip()
        for key in ['serial', 'idVendor', 'idProduct', 'manufacturer', 'product']:
            if f'ATTRS{{{key}}}==' in line:
                val = line.split('==')[1].strip().strip('"')
                if key not in info:   # Chỉ lấy giá trị đầu tiên (thiết bị, không phải hub)
                    info[key] = val
    return info


def list_ports() -> list[str]:
    ports = []
    for prefix in ['/dev/ttyUSB', '/dev/ttyACM']:
        for i in range(10):
            p = f"{prefix}{i}"
            if os.path.exists(p):
                ports.append(p)
    return sorted(ports)


def pick_device(ports: list[str], role: str, role_hint: str) -> tuple[str, dict]:
    """Cho user chọn cổng USB cho một thiết bị."""
    print(f"\n{BOLD}{BLUE}── Chọn cổng cho: {role} ({role_hint}) ──{RESET}")
    
    device_infos = []
    for i, p in enumerate(ports):
        info = get_usb_info(p)
        mfr = info.get('manufacturer', '?')
        prod = info.get('product', '?')
        serial = info.get('serial', '?')
        print(f"  [{i}] {p}  │  {mfr} - {prod}  │  serial: {serial}")
        device_infos.append((p, info))
    
    print(f"  [s] Bỏ qua (thiết bị này chưa cắm)")
    
    while True:
        choice = input(f"  Nhập số (0-{len(ports)-1}) hoặc 's' để bỏ qua: ").strip()
        if choice.lower() == 's':
            return None, {}
        try:
            idx = int(choice)
            if 0 <= idx < len(ports):
                return device_infos[idx]
        except ValueError:
            pass
        print(f"  {RED}Lựa chọn không hợp lệ, thử lại.{RESET}")


def build_udev_rule(symlink: str, info: dict) -> str:
    """Tạo 1 dòng udev rule từ thông tin thiết bị."""
    if 'serial' in info and info['serial'] not in ['?', '']:
        # Ưu tiên dùng serial number — chính xác nhất
        match = f'ATTRS{{serial}}=="{info["serial"]}"'
    elif 'idVendor' in info and 'idProduct' in info:
        # Fallback: dùng Vendor+Product ID
        match = f'ATTRS{{idVendor}}=="{info["idVendor"]}", ATTRS{{idProduct}}=="{info["idProduct"]}"'
    else:
        return None
    
    return (
        f'SUBSYSTEM=="tty", {match}, '
        f'SYMLINK+="{symlink}", MODE="0666"'
    )


def main():
    print(f"\n{BOLD}{GREEN}╔══════════════════════════════════════════════════════╗{RESET}")
    print(f"{BOLD}{GREEN}║     Robot USB Port Setup — Tạo Symlink Cố Định       ║{RESET}")
    print(f"{BOLD}{GREEN}╚══════════════════════════════════════════════════════╝{RESET}\n")
    
    if os.geteuid() != 0:
        print(f"{YELLOW}⚠️  Cần quyền root để ghi udev rules. Nhập mật khẩu khi được hỏi.{RESET}\n")
    
    ports = list_ports()
    if not ports:
        print(f"{RED}❌ Không tìm thấy cổng USB Serial nào. Hãy cắm thiết bị rồi chạy lại!{RESET}")
        sys.exit(1)
    
    print(f"{GREEN}✅ Tìm thấy {len(ports)} cổng USB:{RESET}")
    for p in ports:
        info = get_usb_info(p)
        mfr = info.get('manufacturer', '?')
        prod = info.get('product', '?')
        print(f"   {p}  →  {mfr} - {prod}")
    
    # Định nghĩa 3 thiết bị cần cố định
    devices = [
        ('robot_lidar', 'Lidar SC-Mini', 'Thường là Espressif hoặc Silabs CP210x'),
        ('robot_base',  'Robot BW-DR03', 'Thường là CH340 hoặc FTDI'),
        ('robot_rssi',  'Mạch RSSI NodeMCU', 'Thường là Espressif/CP2102'),
    ]
    
    rules = []
    selections = {}
    
    for symlink, role, hint in devices:
        port, info = pick_device(ports, role, hint)
        if port is None:
            print(f"  {YELLOW}⏭️  Bỏ qua {role}.{RESET}")
            continue
        
        rule = build_udev_rule(symlink, info)
        if rule is None:
            print(f"  {RED}❌ Không lấy được thông tin thiết bị để tạo rule!{RESET}")
            continue
        
        rules.append(rule)
        selections[symlink] = port
        serial = info.get('serial', info.get('idVendor', '?'))
        print(f"  {GREEN}✅ /dev/{symlink} → {port}  (serial: {serial}){RESET}")
    
    if not rules:
        print(f"\n{RED}❌ Không có rule nào được tạo. Thoát.{RESET}")
        sys.exit(1)
    
    # Ghi file udev rule
    rule_file = '/etc/udev/rules.d/99-robot-usb.rules'
    rule_content = "# Robot USB Persistent Port Rules\n"
    rule_content += "# Tạo tự động bởi setup_usb_rules.py\n\n"
    rule_content += '\n'.join(rules) + '\n'
    
    print(f"\n{BOLD}── Nội dung file udev rule sẽ được ghi: ──{RESET}")
    print(f"{BLUE}{rule_content}{RESET}")
    
    confirm = input(f"Ghi vào {rule_file}? (y/N): ").strip().lower()
    if confirm != 'y':
        print(f"{YELLOW}Hủy bỏ.{RESET}")
        sys.exit(0)
    
    try:
        write_cmd = f"echo '{rule_content}' | sudo tee {rule_file}"
        os.system(f"sudo bash -c \"echo '{rule_content.strip()}' > {rule_file}\"")
        os.system("sudo udevadm control --reload-rules")
        os.system("sudo udevadm trigger")
        
        print(f"\n{GREEN}{BOLD}✅ Hoàn tất! Udev rules đã được áp dụng.{RESET}")
        print(f"\nSau khi cắm lại thiết bị, các cổng sẽ cố định thành:")
        for symlink, port in selections.items():
            print(f"   /dev/{symlink}  (trước là {port})")
        
        print(f"\n{BOLD}Lệnh chạy robot (không cần điền cổng nữa):{RESET}")
        print(f"""
{GREEN}# Terminal 1 — Lidar:
ros2 launch sc_mini sc_mini.launch.py port:=/dev/robot_lidar

# Terminal 2 — Stack chính:
ros2 launch robot_rssi_ros2 rssi_vfh_follow.launch.py{RESET}
""")
    except Exception as e:
        print(f"{RED}❌ Lỗi khi ghi file: {e}{RESET}")
        print(f"Thử ghi thủ công:\n  sudo nano {rule_file}")
        print(f"Nội dung:\n{rule_content}")


if __name__ == '__main__':
    main()
