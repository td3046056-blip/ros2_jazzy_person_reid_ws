import json
import struct
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32, Float32MultiArray, String
from geometry_msgs.msg import Twist

import serial
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


HEADER = b'\xcd\xeb\xd7'


class BWDR03DecodedSerialNode(Node):
    def __init__(self):
        super().__init__('bw_dr03_decoded_serial_node')

        # =========================
        # Parameters
        # =========================
        self.declare_parameter('port', '/dev/bw_dr03')
        self.declare_parameter('baudrate', 115200)

        # max_linear: vận tốc tiến tối đa tương ứng max_percent
        # max_angular: vận tốc quay tối đa tương ứng max_percent
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('max_angular', 1.0)

        # max_percent: phần trăm tốc độ gửi xuống BW-DR03, 0-100
        # Khi test nên để 20-30 trước, không để cao ngay
        self.declare_parameter('max_percent', 30)

        # deadband: vùng chết cho cmd_vel
        self.declare_parameter('deadband', 0.02)

        # stop_brake: giá trị brake khi gửi lệnh stop
        self.declare_parameter('stop_brake', 30)

        # cmd_timeout: nếu quá thời gian này không có /cmd_vel thì gửi STOP
        self.declare_parameter('cmd_timeout', 1.0)
        
        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('wheel_separation', 0.40)
        self.declare_parameter('encoder_ppr_default', 750)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.encoder_ppr_default = int(self.get_parameter('encoder_ppr_default').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        
        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.max_percent = int(self.get_parameter('max_percent').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.stop_brake = int(self.get_parameter('stop_brake').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # =========================
        # Serial
        # =========================
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01
        )

        self.buffer = bytearray()

        # =========================
        # Publishers
        # =========================
        self.pub_decoded = self.create_publisher(String, '/bw_dr03/decoded', 10)
        self.pub_status = self.create_publisher(Int32, '/bw_dr03/status', 10)
        self.pub_power = self.create_publisher(Float32, '/bw_dr03/power', 10)
        self.pub_theta = self.create_publisher(Float32, '/bw_dr03/theta', 10)
        self.pub_encoder_r = self.create_publisher(Int32, '/bw_dr03/encoder_delta_r', 10)
        self.pub_encoder_l = self.create_publisher(Int32, '/bw_dr03/encoder_delta_l', 10)
        self.pub_imu = self.create_publisher(Float32MultiArray, '/bw_dr03/imu_raw', 10)
        self.pub_sonar = self.create_publisher(Float32MultiArray, '/bw_dr03/sonar', 10)
        
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # =========================
        # Subscriber /cmd_vel
        # =========================
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.last_cmd_time = time.time()
        self.is_stopped = True
        
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_time = self.get_clock().now()

        # Timer đọc serial từ BW-DR03
        self.read_timer = self.create_timer(0.005, self.read_serial)

        # Watchdog tự dừng nếu mất /cmd_vel
        self.watchdog_timer = self.create_timer(0.1, self.cmd_watchdog)

        self.get_logger().info(f'BW-DR03 decoder connected on {port} at {baudrate}')
        self.get_logger().info(
            f'Params: max_linear={self.max_linear}, '
            f'max_angular={self.max_angular}, '
            f'max_percent={self.max_percent}, '
            f'cmd_timeout={self.cmd_timeout}'
        )

    # ============================================================
    # SERIAL READ + DECODE
    # ============================================================

    def read_serial(self):
        try:
            n = self.ser.in_waiting

            if n <= 0:
                return

            data = self.ser.read(n)

            if not data:
                return

            self.buffer.extend(data)
            self.parse_buffer()

        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read warning: {e}')
            return

        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')
            return

    def parse_buffer(self):
        while True:
            header_index = self.buffer.find(HEADER)

            if header_index < 0:
                if len(self.buffer) > 3:
                    self.buffer = self.buffer[-3:]
                return

            if header_index > 0:
                del self.buffer[:header_index]

            if len(self.buffer) < 4:
                return

            packet_len = self.buffer[3]
            total_len = 4 + packet_len

            if len(self.buffer) < total_len:
                return

            content = bytes(self.buffer[4:total_len])
            del self.buffer[:total_len]

            self.decode_packet(content)

    def decode_packet(self, content):
        # BW-DR03 thường gửi content dài 125 byte:
        # 25 trường, mỗi trường 4 byte + 1 byte ngăn cách
        if len(content) < 125:
            return

        formats = [
            'i',  # 0 status
            'f',  # 1 power
            'f',  # 2 theta
            'I',  # 3 encoder_ppr
            'i',  # 4 encoder_delta_r
            'i',  # 5 encoder_delta_l
            'i',  # 6 encoder_delta_car
            'I',  # 7 upward
            'f',  # 8 max_speed
            'i',  # 9 hbz1
            'i',  # 10 hbz2
            'i',  # 11 hbz3
            'i',  # 12 hbz4
            'f',  # 13 distance1
            'f',  # 14 distance2
            'f',  # 15 imu acc x
            'f',  # 16 imu acc y
            'f',  # 17 imu acc z
            'f',  # 18 imu gyro x
            'f',  # 19 imu gyro y
            'f',  # 20 imu gyro z
            'f',  # 21 imu mag x
            'f',  # 22 imu mag y
            'f',  # 23 imu mag z
            'I',  # 24 timestamp
        ]

        values = []

        try:
            for i, fmt in enumerate(formats):
                offset = i * 5
                value = struct.unpack_from('<' + fmt, content, offset)[0]
                values.append(value)

        except struct.error:
            return

        status = int(values[0])
        power = float(values[1])
        theta = float(values[2])
        encoder_ppr = int(values[3])
        encoder_delta_r = int(values[4])
        encoder_delta_l = int(values[5])
        
        self.update_odom(
            encoder_delta_r=encoder_delta_r,
            encoder_delta_l=encoder_delta_l,
            encoder_ppr=encoder_ppr
        )
        encoder_delta_car = int(values[6])
        upward = int(values[7])
        max_speed = float(values[8])

        hbz = [
            int(values[9]),
            int(values[10]),
            int(values[11]),
            int(values[12])
        ]

        distance1 = float(values[13])
        distance2 = float(values[14])

        imu = [float(v) for v in values[15:24]]
        timestamp = int(values[24])

        decoded = {
            'status': status,
            'power_v': power,
            'theta_deg': theta,
            'encoder_ppr': encoder_ppr,
            'encoder_delta_r': encoder_delta_r,
            'encoder_delta_l': encoder_delta_l,
            'encoder_delta_car': encoder_delta_car,
            'upward': upward,
            'max_speed': max_speed,
            'hbz': hbz,
            'distance1': distance1,
            'distance2': distance2,
            'imu': {
                'acc_x': imu[0],
                'acc_y': imu[1],
                'acc_z': imu[2],
                'gyro_x': imu[3],
                'gyro_y': imu[4],
                'gyro_z': imu[5],
                'mag_x': imu[6],
                'mag_y': imu[7],
                'mag_z': imu[8],
            },
            'timestamp': timestamp,
        }

        self.pub_decoded.publish(String(data=json.dumps(decoded)))
        self.pub_status.publish(Int32(data=status))
        self.pub_power.publish(Float32(data=power))
        self.pub_theta.publish(Float32(data=theta))
        self.pub_encoder_r.publish(Int32(data=encoder_delta_r))
        self.pub_encoder_l.publish(Int32(data=encoder_delta_l))
        self.pub_imu.publish(Float32MultiArray(data=imu))
        self.pub_sonar.publish(Float32MultiArray(data=[distance1, distance2]))

    # ============================================================
    # MOTOR COMMAND - 13 BYTE PROTOCOL
    # ============================================================
    #
    # Thực tế test của bạn:
    # cd eb d7 09 74 46 46 53 53 1e 1e 00 00
    # làm 2 bánh tiến.
    #
    # Mapping:
    # right motor = Mb1 + H1
    # left motor  = Ma1 + H0
    #
    # Packet:
    # [CD EB D7] [09] [74] [right_mode] [left_mode] [S] [S]
    # [right_value] [left_value] [0] [0]
    #
    # mode:
    # F = Forward
    # B = Backward
    # S = Stop / Brake
    # ============================================================
    
    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return qz, qw


    def update_odom(self, encoder_delta_r, encoder_delta_l, encoder_ppr):
        if encoder_ppr <= 0:
            encoder_ppr = self.encoder_ppr_default

        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9

        if dt <= 0.0:
            dt = 1e-3

        self.last_odom_time = now

        # Quãng đường mỗi xung encoder
        meter_per_count = (2.0 * math.pi * self.wheel_radius) / float(encoder_ppr)

        # Theo bạn đã xác nhận:
        # tiến = encoder_delta_l/r dương
        dist_r = encoder_delta_r * meter_per_count
        dist_l = encoder_delta_l * meter_per_count

        dist_center = (dist_r + dist_l) * 0.5
        delta_yaw = (dist_r - dist_l) / self.wheel_separation

        # Tích phân theo midpoint để ổn định hơn
        yaw_mid = self.odom_yaw + delta_yaw * 0.5

        self.odom_x += dist_center * math.cos(yaw_mid)
        self.odom_y += dist_center * math.sin(yaw_mid)
        self.odom_yaw += delta_yaw

        # Giữ yaw trong [-pi, pi]
        self.odom_yaw = math.atan2(
            math.sin(self.odom_yaw),
            math.cos(self.odom_yaw)
        )

        linear_v = dist_center / dt
        angular_v = delta_yaw / dt

        qz, qw = self.yaw_to_quaternion(self.odom_yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = linear_v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_v

        self.pub_odom.publish(odom_msg)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id

            tf_msg.transform.translation.x = self.odom_x
            tf_msg.transform.translation.y = self.odom_y
            tf_msg.transform.translation.z = 0.0

            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf_msg)
    

    def send_two_motor_command(self, right_mode, left_mode, right_value, left_value):
        right_value = max(0, min(100, int(right_value)))
        left_value = max(0, min(100, int(left_value)))

        packet = bytes([
            0xCD, 0xEB, 0xD7, 0x09, 0x74,
            ord(right_mode),
            ord(left_mode),
            ord('S'),
            ord('S'),
            right_value,
            left_value,
            0,
            0
        ])

        self.get_logger().info(f'SEND MOTOR CMD: {packet.hex(" ")}')

        try:
            self.ser.write(packet)
            self.ser.flush()

        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def send_stop(self):
        self.send_two_motor_command(
            'S',
            'S',
            self.stop_brake,
            self.stop_brake
        )

    def wheel_to_mode_speed(self, value):
        if abs(value) < self.deadband:
            return 'S', self.stop_brake

        speed = int(abs(value) * self.max_percent)
        speed = max(5, min(self.max_percent, speed))

        if value > 0:
            return 'F', speed
        else:
            return 'B', speed

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        self.get_logger().info(
            f'RECEIVED /cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}'
        )

        x = msg.linear.x
        z = msg.angular.z

        # Nếu lệnh gần 0 thì dừng
        if abs(x) < self.deadband and abs(z) < self.deadband:
            if not self.is_stopped:
                self.send_stop()
                self.is_stopped = True
            return

        self.is_stopped = False

        # Chuẩn ROS:
        # linear.x > 0  : robot tiến
        # linear.x < 0  : robot lùi
        # angular.z > 0 : robot quay trái
        # angular.z < 0 : robot quay phải

        x_norm = max(-1.0, min(1.0, x / self.max_linear))
        z_norm = max(-1.0, min(1.0, z / self.max_angular))

        # Điều khiển vi sai:
        # right = x + z
        # left  = x - z
        #
        # Khi z > 0:
        # right dương, left âm -> robot quay trái
        right_cmd = x_norm + z_norm
        left_cmd = x_norm - z_norm

        # Normalize để không vượt [-1, 1]
        max_abs = max(abs(right_cmd), abs(left_cmd), 1.0)
        right_cmd = right_cmd / max_abs
        left_cmd = left_cmd / max_abs

        right_mode, right_speed = self.wheel_to_mode_speed(right_cmd)
        left_mode, left_speed = self.wheel_to_mode_speed(left_cmd)

        self.send_two_motor_command(
            right_mode,
            left_mode,
            right_speed,
            left_speed
        )

    def cmd_watchdog(self):
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            if not self.is_stopped:
                self.get_logger().warn('cmd_vel timeout -> STOP')
                self.send_stop()
                self.is_stopped = True

    def destroy_node(self):
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.send_stop()
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = BWDR03DecodedSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
