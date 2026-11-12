#!/usr/bin/env bash
# Phat lai bag chay that trong RViz2 de CHUP MAN HINH cho bao cao (khong can xe).
#
#   bash bao_cao/rviz/xem_bag.sh ~/bags/follow_1107_104423 [giay_bat_dau] [toc_do]
#   vd: bash bao_cao/rviz/xem_bag.sh ~/bags/follow_1107_104423 330 0.5
#
# An toan: chay trong MIEN ROS RIENG (ROS_DOMAIN_ID=77) nen khong lan voi xe that dang chay;
# KHONG phat /cmd_vel tu bag va khong chay planner -> khong co nguon nao ghi /cmd_vel.
#
# TF LiDAR: cac launch file cua xe phat base_link -> laser voi goc 0, trong khi code dung
# lidar_yaw_offset_deg = -90 (muc 3 CLAUDE.md). Planner/tracker khong dung TF nen xe chay dung, nhung RViz
# se ve vong quet lech 90 do. Script nay KHONG phat /tf_static cua bag ma tu phat TF dung (yaw -90 do).
set -euo pipefail
BAG="${1:?can duong dan bag, vd ~/bags/follow_1107_104423}"
START="${2:-0}"
RATE="${3:-1.0}"
DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77

PIDS=()
don_dep() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done; }
trap don_dep EXIT INT TERM

ros2 run tf2_ros static_transform_publisher --x 0.10 --y 0.0 --z 0.18 --yaw -1.5707963 \
  --frame-id base_link --child-frame-id laser --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
PIDS+=($!)
python3 "$DIR/rviz_markers.py" --ros-args -p use_sim_time:=true &
PIDS+=($!)
rviz2 -d "$DIR/bao_cao.rviz" --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
PIDS+=($!)
sleep 3
echo "Phat bag tu giay $START, toc do x$RATE. Space: tam dung (trong terminal nay). Ctrl-C: thoat."
ros2 bag play "$BAG" --clock --rate "$RATE" --start-offset "$START" \
  --topics /scan /odom /tf /follow/target /follow/planner_status
