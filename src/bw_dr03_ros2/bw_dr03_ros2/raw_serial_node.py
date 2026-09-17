import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class BWDR03RawSerialNode(Node):
    def __init__(self):
        super().__init__('bw_dr03_raw_serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.publisher_ = self.create_publisher(String, '/bw_dr03/raw_packet', 10)

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        self.get_logger().info(f'Connected to BW-DR03 on {port} at {baudrate}')

        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        try:
            data = self.ser.read(256)
            if data:
                msg = String()
                msg.data = data.hex(' ')
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BWDR03RawSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
