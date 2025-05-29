import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class RobotController(Node):
    def __init__(self):
        super().__init__('piduino02')

        # Detect and open serial port
        self.port = self.detect_serial_port()
        self.ser = None

        try:
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            self.get_logger().info(f"Serial connection established on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial device: {e}")
            return

        # Start a timer to continuously read serial data
        self.create_timer(0.1, self.read_serial)

        # Publishers for left and right sensor data
        self.left_sensor_pub = self.create_publisher(Range, '/sensor_data_left', 10)
        self.right_sensor_pub = self.create_publisher(Range, '/sensor_data_right', 10)

        # Movement thresholds (converted from cm to meters)
        self.min_distance = 22.0 / 100.0  # 22 cm → 0.22 meters (Stop & back up)
        self.adjust_distance = 30.0 / 100.0  # 30 cm → 0.30 meters (Minor turn)

        self.get_logger().info("RobotController node is running...")

        # Start moving forward immediately
        self.move_forward()

    def detect_serial_port(self):
        """ Detect available serial ports and return the first valid one. """
        import glob
        possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        return possible_ports[0] if possible_ports else None

    def read_serial(self):
        """ Reads serial data, extracts distances, and publishes to ROS 2 """
        if self.ser.in_waiting > 0:
            try:
                # Read the full line from the serial port
                data = self.ser.readline().decode('utf-8').strip()

                # Print raw serial data for debugging
                self.get_logger().info(f"Raw Serial Data: {data}")

                # Check if it starts with "DIST"
                if data.startswith("DIST"):
                    parts = data.split()  # Split by spaces
                    if len(parts) == 3:  # Expecting ["DIST", "xx.xx", "yy.yy"]
                        distance_left_cm = float(parts[1])  # Left sensor in cm
                        distance_right_cm = float(parts[2])  # Right sensor in cm

                        # Convert cm → meters
                        distance_left = distance_left_cm / 100.0
                        distance_right = distance_right_cm / 100.0

                        # Publish left and right distances separately
                        left_msg = Range()
                        left_msg.range = distance_left
                        self.left_sensor_pub.publish(left_msg)

                        right_msg = Range()
                        right_msg.range = distance_right
                        self.right_sensor_pub.publish(right_msg)

                        # Print debug output
                        self.get_logger().info(f"Left Sensor: {distance_left:.2f} m, Right Sensor: {distance_right:.2f} m")

                        # Process each sensor separately
                        self.sensor_callback(distance_left, distance_right)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")

    def move_forward(self):
        """ Move forward """
        self.get_logger().info("Moving forward")
        self.ser.write(b'F')

    def stop_and_reverse(self):
        """ Stop, reverse, and prepare to turn """
        self.get_logger().info("TOO CLOSE! Stopping and backing up.")
        self.ser.write(b'S')  # Stop
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.3))
        self.ser.write(b'B')  # Back up
        self.get_logger().info("Backing up for 300ms")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.3))

    def turn_left(self, minor=False):
        """ Turn left slightly or fully based on condition """
        if minor:
            self.get_logger().info("Minor left turn (Obstacle nearby)")
            self.ser.write(b'L')
        else:
            self.get_logger().info("Full left turn (Obstacle too close)")
            self.ser.write(b'L')

    def turn_right(self, minor=False):
        """ Turn right slightly or fully based on condition """
        if minor:
            self.get_logger().info("Minor right turn (Obstacle nearby)")
            self.ser.write(b'R')
        else:
            self.get_logger().info("Full right turn (Obstacle too close)")
            self.ser.write(b'R')

    def sensor_callback(self, distance_left, distance_right):
        """ Reacts to sensor data and commands movement """
        
        self.get_logger().info(f"Processed Left Sensor: {distance_left:.2f} m, Right Sensor: {distance_right:.2f} m")

        if distance_left < self.min_distance or distance_right < self.min_distance:
            self.get_logger().info("TOO CLOSE! Executing stop and reverse.")
            self.stop_and_reverse()

            # Turn away from the closer obstacle
            if distance_left < distance_right:
                self.turn_right()
            else:
                self.turn_left()
        
        elif distance_left < self.adjust_distance:
            self.get_logger().info("Obstacle on the left, adjusting right.")
            self.turn_right(minor=True)
        
        elif distance_right < self.adjust_distance:
            self.get_logger().info("Obstacle on the right, adjusting left.")
            self.turn_left(minor=True)

        else:
            self.get_logger().info("Path is clear. Moving forward.")
            self.move_forward()

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 