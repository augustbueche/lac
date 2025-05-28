import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.ultrasonic_publisher = self.create_publisher(String, 'ultrasonic_data', 10)
        self.encoder_publisher = self.create_publisher(String, 'encoder_data', 10)
        self.mpu_publisher = self.create_publisher(String, 'mpu_data', 10)
        self.compass_publisher = self.create_publisher(String, 'compass_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Track previous encoder values for delta calculation
        self.last_left_encoder = 0
        self.last_right_encoder = 0

        # Distance per tick (meters)
        self.distance_per_tick = 0.01128

        # Track total distance
        self.total_left_distance = 0.0
        self.total_right_distance = 0.0

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f"Raw serial data: {line}")
            try:
                parts = line.split(',')

                # Publish ultrasonic data
                if line.startswith('A:'):
                    self.ultrasonic_publisher.publish(String(data=line))
                    self.get_logger().info(f'Publishing ultrasonic data: {line}')

                # Extract and publish encoder data
                left_str = next((p for p in parts if 'Left Encoder' in p), None)
                right_str = next((p for p in parts if 'Right Encoder' in p), None)
                if left_str and right_str:
                    left = int(left_str.split(':')[1].strip())
                    right = int(right_str.split(':')[1].strip())

                    delta_left = left - self.last_left_encoder
                    delta_right = right - self.last_right_encoder
                    self.last_left_encoder = left
                    self.last_right_encoder = right

                    left_dist = delta_left * self.distance_per_tick
                    right_dist = delta_right * self.distance_per_tick
                    self.total_left_distance += left_dist
                    self.total_right_distance += right_dist

                    encoder_data = (
                        f"Left Steps: {left}, Right Steps: {right}, "
                        f"Left Distance: {self.total_left_distance:.3f} m, "
                        f"Right Distance: {self.total_right_distance:.3f} m"
                    )
                    self.encoder_publisher.publish(String(data=encoder_data))
                    self.get_logger().info(f'Publishing encoder data: {encoder_data}')

                # Extract and publish MPU6050 data
                mpu_values = {}
                for part in parts:
                    if any(k in part for k in ['AccX','AccY','AccZ','GyroX','GyroY','GyroZ']):
                        label, val = part.split(':')
                        key = label.replace('MPU ', '').strip()
                        mpu_values[key] = float(val.strip())

                if len(mpu_values) == 6:
                    mpu_data_str = (
                        f"AccX: {mpu_values['AccX']}, AccY: {mpu_values['AccY']}, AccZ: {mpu_values['AccZ']}, "
                        f"GyroX: {mpu_values['GyroX']}, GyroY: {mpu_values['GyroY']}, GyroZ: {mpu_values['GyroZ']}"
                    )
                    self.mpu_publisher.publish(String(data=mpu_data_str))
                    self.get_logger().info(f'Publishing MPU6050 data: {mpu_data_str}')

                # Extract and publish compass data
                compass_str = next((p for p in parts if 'Compass' in p), None)
                if compass_str:
                    heading = float(compass_str.split(':')[1].strip())
                    compass_data = f"Heading: {heading} degrees"
                    self.compass_publisher.publish(String(data=compass_data))
                    self.get_logger().info(f'Publishing compass data: {compass_data}')

            except Exception as e:
                self.get_logger().error(f"Error parsing data: {line} - {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
