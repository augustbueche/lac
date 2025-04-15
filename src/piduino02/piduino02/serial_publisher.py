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
        self.mpu_publisher = self.create_publisher(String, 'mpu_data', 10)
        self.compass_publisher = self.create_publisher(String, 'compass_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Track previous encoder values for delta calculation
        self.last_left_encoder = 0
        self.last_right_encoder = 0

        # Distance per tick (calculated from calibration test)
        self.distance_per_tick = 0.01128  # meters per tick

        # Track total distance
        self.total_left_distance = 0.0
        self.total_right_distance = 0.0

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            try:
                # Parse the combined sensor data
                if line.startswith('A:'):
                    # Publish ultrasonic data
                    self.ultrasonic_publisher.publish(String(data=line))
                    self.get_logger().info(f'Publishing ultrasonic data: {line}')

                    # Extract encoder values
                    parts = line.split(',')
                    left_encoder_str = [part for part in parts if 'Left Encoder' in part][0]
                    right_encoder_str = [part for part in parts if 'Right Encoder' in part][0]

                    left_value = int(left_encoder_str.split(':')[1].strip())
                    right_value = int(right_encoder_str.split(':')[1].strip())

                    # Calculate deltas
                    delta_left = left_value - self.last_left_encoder
                    delta_right = right_value - self.last_right_encoder

                    # Update previous values
                    self.last_left_encoder = left_value
                    self.last_right_encoder = right_value

                    # Calculate distances
                    left_distance = delta_left * self.distance_per_tick
                    right_distance = delta_right * self.distance_per_tick

                    # Update total distances
                    self.total_left_distance += left_distance
                    self.total_right_distance += right_distance

                    # Publish encoder data with distances
                    encoder_data = (
                        f"Left Steps: {left_value}, Right Steps: {right_value}, "
                        f"Left Distance: {self.total_left_distance:.3f} m, "
                        f"Right Distance: {self.total_right_distance:.3f} m"
                    )
                    self.encoder_publisher.publish(String(data=encoder_data))
                    self.get_logger().info(f'Publishing encoder data: {encoder_data}')

                    # Extract MPU6050 data
                    mpu_data = {
                        "AccX": None,
                        "AccY": None,
                        "AccZ": None,
                        "GyroX": None,
                        "GyroY": None,
                        "GyroZ": None
                    }
                    for key in mpu_data.keys():
                        for part in parts:
                            if key in part:
                                mpu_data[key] = float(part.split(':')[1].strip())

                    mpu_data_str = (
                        f"AccX: {mpu_data['AccX']}, AccY: {mpu_data['AccY']}, AccZ: {mpu_data['AccZ']}, "
                        f"GyroX: {mpu_data['GyroX']}, GyroY: {mpu_data['GyroY']}, GyroZ: {mpu_data['GyroZ']}"
                    )
                    self.mpu_publisher.publish(String(data=mpu_data_str))
                    self.get_logger().info(f'Publishing MPU6050 data: {mpu_data_str}')

                    # Extract compass data
                    compass_str = [part for part in parts if 'Compass' in part][0]
                    compass_heading = float(compass_str.split(':')[1].strip())
                    compass_data = f"Heading: {compass_heading} degrees"
                    self.compass_publisher.publish(String(data=compass_data))
                    self.get_logger().info(f'Publishing compass data: {compass_data}')

            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Error parsing data: {line} - {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()