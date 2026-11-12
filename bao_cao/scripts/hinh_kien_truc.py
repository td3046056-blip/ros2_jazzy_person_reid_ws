#!/usr/bin/env python3
"""So do kien truc phan mem ROS 2 (graphviz): node, topic chinh, phan cung. Theo muc 5 CLAUDE.md."""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kieu_hinh import HINH  # noqa: E402

DOT = r'''
digraph G {
  graph [rankdir=TB, fontname="DejaVu Sans", fontsize=11, nodesep=0.3, ranksep=0.5, splines=spline, pad=0.15];
  node  [fontname="DejaVu Sans", fontsize=10, style="rounded,filled", shape=box, penwidth=1.0];
  edge  [fontname="DejaVu Sans", fontsize=8.5, color="#555555", arrowsize=0.7];

  subgraph cluster_hw {
    label="Phần cứng"; style="rounded,dashed"; color="#999999"; fontcolor="#555555";
    cam   [label="Camera KINGSEN\n(mắt cá 114°, cao 0,70 m)", fillcolor="#e8f1fb", color="#4a7ab5"];
    lidar [label="LiDAR SC-Mini\n(360°, 10 Hz, cao 0,18 m)", fillcolor="#e8f1fb", color="#4a7ab5"];
    base  [label="Khung xe BW-DR03\n(vi sai, encoder)", fillcolor="#e8f1fb", color="#4a7ab5"];
    esp   [label="3 board ESP32\n+ beacon BLE", fillcolor="#e8f1fb", color="#4a7ab5"];
  }

  ident [label="person_follow_identity\nYOLO26n + DeepSORT + OSNet x0.5\n(nhận đúng chủ – ReID)", fillcolor="#fdf0e2", color="#d08a3a"];
  scmini[label="sc_mini\n(driver LiDAR)", fillcolor="#f2f2f2", color="#888888"];
  drv   [label="decoded_serial_node\n(driver khung xe)", fillcolor="#f2f2f2", color="#888888"];
  rscan [label="rssi_scanner_node", fillcolor="#f2f2f2", color="#888888"];
  rbear [label="rssi_bearing_node\n(xoay dò hướng beacon)", fillcolor="#efeafa", color="#7a62b5"];
  track [label="target_tracker_node\nvị trí người trong khung odom\n(ghép camera + LiDAR, dự đoán khi bị che)", fillcolor="#e6f4ea", color="#3b8f55"];
  plan  [label="follow_planner_node\nchọn khe + DWA + bản đồ lưới\nchui khe hẹp + thoát kẹt + SEARCH\n(nguồn DUY NHẤT ghi /cmd_vel)", fillcolor="#fde8e2", color="#c4502f", penwidth=1.6];

  { rank=same; ident; scmini; drv; rscan; }
  { rank=same; track; rbear; }
  cam -> ident [label="ảnh"];
  lidar -> scmini [label="USB"];
  base -> drv [dir=both, label="USB"];
  esp -> rscan [label="USB"];

  ident -> track [label="/person_reid/target\n(góc, bbox)"];
  scmini -> track [label="/scan"];
  drv -> track [label="/odom"];
  scmini -> plan [label="/scan"];
  drv -> plan [label="/odom"];
  track -> plan [label="/follow/target\n(x, y, vận tốc người)"];
  rscan -> rbear [label="/rssi/raw"];
  drv -> rbear [label="/odom", style=dashed];
  rbear -> plan [label="/rssi/bearing\n(chỉ khi tìm lại người)", style=dashed];
  plan -> drv [label="/cmd_vel  (v, ω)", color="#c4502f", fontcolor="#c4502f", penwidth=1.6, constraint=false];
}
'''

if __name__ == "__main__":
    os.makedirs(HINH, exist_ok=True)
    out = os.path.join(HINH, "he_thong_kien_truc_ros2.png")
    subprocess.run(["dot", "-Tpng", "-Gdpi=300", "-o", out], input=DOT.encode(), check=True)
    subprocess.run(["dot", "-Tsvg", "-o", out.replace(".png", ".svg")], input=DOT.encode(), check=True)
    print("da luu", out)
