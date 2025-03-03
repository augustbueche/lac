import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

THRESHOLD = XX  # Set your threshold value
SERIAL_PORT = '/dev/ttyUSB0'  # Update this based on your Arduino connection
BAUD_RATE = 9600

class FanController(Node):
    def __init__(self):
        super().__init__('fan_controller')
        self.subscription = self.create_subscription(
            String,
            'airquality',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
       
        # Initialize Serial Communication with Arduino
        try:
            self.arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Allow time for Arduino to initialize
            self.get_logger().info("Connected to Arduino successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None  # Handle connection failure

    def listener_callback(self, msg):
        if self.arduino:
            try:
                sensor_value = float(msg.data)  # Convert message to float
                self.get_logger().info(f"Received sensor value: {sensor_value}")

                if sensor_value >= THRESHOLD:
                    self.arduino.write(b'ON\n')  # Send 'ON' command to Arduino
                    self.get_logger().info("Fan turned ON (Sent to Arduino)")
                else:
                    self.arduino.write(b'OFF\n')  # Send 'OFF' command to Arduino
                    self.get_logger().info("Fan turned OFF (Sent to Arduino)")
            except ValueError:
                self.get_logger().error(f"Invalid sensor data received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = FanController()
    try:
        rclpy.spin(node)
    finally:
        if node.arduino:
            node.arduino.close()  # Close serial connection on shutdown
        rclpy.shutdown()

if __name__ == '__main__':
    main()