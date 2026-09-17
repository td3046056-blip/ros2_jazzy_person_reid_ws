#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')

        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.callback, 10)
        self.pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        self.get_logger().info('TwistStamper ready: /cmd_vel -> /diff_drive_controller/cmd_vel')

    def callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header = Header()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg
        self.pub.publish(stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
