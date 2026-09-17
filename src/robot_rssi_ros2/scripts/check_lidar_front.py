#!/usr/bin/env python3
"""
check_lidar_front.py
====================
Kiểm tra xem góc nào của Lidar SC-Mini là hướng THẲNG TRƯỚC thực tế của robot.

Cách dùng:
  1. Đặt 1 vật cản (tay hoặc thùng carton) ngay trước MŨI ROBOT (cách 30-50cm).
  2. Bật Lidar ở terminal khác:
       ros2 launch sc_mini sc_mini.launch.py port:=/dev/robot_lidar
  3. Chạy script này:
       python3 check_lidar_front.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class CheckLidarFront(Node):
    def __init__(self):
        super().__init__('check_lidar_front')
        self.create_subscription(LaserScan, '/scan', self._scan_cb, rclpy.qos.qos_profile_sensor_data)
        self.get_logger().info('Đang lắng nghe /scan... Hãy đặt vật cản NGAY TRƯỚC MŨI ROBOT.')

    def _scan_cb(self, msg: LaserScan):
        min_dist = 999.0
        min_angle_deg = 0.0
        
        for i, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max:
                if r < min_dist:
                    min_dist = r
                    angle_rad = msg.angle_min + i * msg.angle_increment
                    min_angle_deg = math.degrees(angle_rad) % 360.0

        if min_dist < 2.0:
            print(f'➜ Vật cản gần nhất: cách {min_dist:.2f}m ở góc LIDAR: {min_angle_deg:.1f}°')
            print(f'   Gợi ý: lidar_front_center_deg nên đặt là {min_angle_deg:.1f}°\n')

def main():
    rclpy.init()
    node = CheckLidarFront()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
