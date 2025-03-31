import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust the port and baud rate as needed
        self.ultrasonic_publisher = self.create_publisher(String, 'ultrasonic_data', 10)
        self.encoder_publisher = self.create_publisher(String, 'encoder_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Track previous encoder values for delta calculation
        self.last_left_encoder = 0
        self.last_right_encoder = 0

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith('A:'):
                # Publish ultrasonic data
                self.ultrasonic_publisher.publish(String(data=line))
                self.get_logger().info(f'Publishing ultrasonic data: {line}')
            elif line.startswith('Left Encoder:'):
                # Parse encoder values
                try:
                    parts = line.split(',')
                    left_value = int(parts[0].split(':')[1].strip())
                    right_value = int(parts[1].split(':')[1].strip())

                    # Calculate deltas
                    delta_left = left_value - self.last_left_encoder
                    delta_right = right_value - self.last_right_encoder

                    # Update previous values
                    self.last_left_encoder = left_value
                    self.last_right_encoder = right_value

                    # Publish signed deltas
                    encoder_data = f"Left Delta: {delta_left}, Right Delta: {delta_right}"
                    self.encoder_publisher.publish(String(data=encoder_data))
                    self.get_logger().info(f'Publishing encoder data: {encoder_data}')
                except (IndexError, ValueError) as e:
                    self.get_logger().error(f"Error parsing encoder data: {line} - {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()