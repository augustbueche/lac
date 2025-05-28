import rclpy
from rclpy.node import Node
import serial
import sys
import termios
import tty

class MotorTest(Node):
    def __init__(self):
        super().__init__('motortest')

        # Detect and open serial port
        self.port = self.detect_serial_port()
        self.ser = None

        try:
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            self.get_logger().info(f"Serial connection established on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial device: {e}")
            return

        self.get_logger().info("MotorTest node is running...")

    def detect_serial_port(self):
        """ Detect available serial ports and return the first valid one. """
        import glob
        possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        return possible_ports[0] if possible_ports else None

    def get_key(self):
        """ Get a single key press from the user """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """ Main loop to get keyboard input and control motors """
        self.get_logger().info("Use W/A/S/D to move, Q to stop, and E to exit.")
        while rclpy.ok():
            key = self.get_key()
            if key == 'w':
                self.get_logger().info("Moving forward")
                self.ser.write(b'F')
            elif key == 's':
                self.get_logger().info("Moving backward")
                self.ser.write(b'B')
            elif key == 'a':
                self.get_logger().info("Turning left")
                self.ser.write(b'L')
            elif key == 'd':
                self.get_logger().info("Turning right")
                self.ser.write(b'R')
            elif key == 'q':
                self.get_logger().info("Stopping")
                self.ser.write(b'S')
            elif key == 'e':
                self.get_logger().info("Exiting")
                break
            self.get_logger().info("Use W/A/S/D to move, Q to stop, and E to exit.")

def main(args=None):
    rclpy.init(args=args)
    node = MotorTest()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
     