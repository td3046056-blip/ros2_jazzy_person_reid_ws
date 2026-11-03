#!/usr/bin/env python3
"""Doc bag mcap (CAN ROS) -> pickle cac topic de PHAT LAI planner / tracker that ma khong can ROS (03/11).

    source /opt/ros/jazzy/setup.bash
    python3 dump_bag.py ~/bags/follow_1015_104402 /tmp/b104402.pkl
    python3 replay_planner.py /tmp/b104402.pkl ~/ros2_ws/ros2_jazzy_person_reid_ws/src/person_follow_nav out.json
    TR_SET="camera_height_m=0.70" python3 replay_tracker.py /tmp/b104402.pkl <pkg_dir> out_tracker.json

Luu: /scan (ranges + goc), /odom (vi tri + quaternion), /cmd_vel, /follow/target, /follow/planner_status,
/person_reid/target, /rssi/bearing (JSON giu nguyen chuoi).
"""
import pickle
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
want = {"/scan", "/odom", "/follow/target", "/follow/planner_status", "/cmd_vel", "/rssi/bearing", "/person_reid/target"}
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
cls = {n: get_message(types[n]) for n in want if n in types}
D = {n: [] for n in cls}
while r.has_next():
    topic, data, t = r.read_next()
    if topic not in cls:
        continue
    m = deserialize_message(data, cls[topic])
    ts = t * 1e-9
    if topic == "/scan":
        D[topic].append((ts, dict(ranges=list(m.ranges), angle_min=m.angle_min, angle_increment=m.angle_increment,
                                  range_min=m.range_min, range_max=m.range_max)))
    elif topic == "/odom":
        p, q = m.pose.pose.position, m.pose.pose.orientation
        D[topic].append((ts, dict(x=p.x, y=p.y, qx=q.x, qy=q.y, qz=q.z, qw=q.w)))
    elif topic == "/cmd_vel":
        D[topic].append((ts, dict(v=m.linear.x, w=m.angular.z)))
    else:
        D[topic].append((ts, m.data))
pickle.dump(D, open(out, "wb"))
print({k: len(v) for k, v in D.items()})
