import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class RobotController(Node):
    def __init__(self):
        super().__init__('piduino02')

        # Automatically detect serial port
        self.port = self.detect_serial_port()
        self.ser = None

        try:
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            self.get_logger().info(f"Serial connection established on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial device: {e}")
            return

        # Subscribe to ultrasonic sensor data
        self.create_subscription(Range, '/sensor_data', self.sensor_callback, 10)

        # Publisher for movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.obstacle_threshold = 0.22  # 22 cm

        self.get_logger().info("RobotController node is running...")

    def detect_serial_port(self):
        """ Check available serial ports and return the correct one. """
        import glob
        possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')

        if possible_ports:
            return possible_ports[0]  # Use the first detected port
        else:
            self.get_logger().error("No serial ports found!")
            return None

    def sensor_callback(self, msg: Range):
        if not self.ser:
            return

        self.get_logger().info(f"Received sensor data: {msg.range} m")
        if msg.range < self.obstacle_threshold:
            self.get_logger().info("Obstacle detected! Stopping and turning.")
            self.ser.write(b'S')  # Stop
            self.ser.write(b'R')  # Turn right
        else:
            self.get_logger().info("No obstacle detected. Moving forward.")
            self.ser.write(b'F')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 