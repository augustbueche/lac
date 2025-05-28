import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            String,
            'ultrasonic_data',
            self.ultrasonic_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize serial communication with Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust the port and baud rate as needed

    def ultrasonic_callback(self, msg):
        data = msg.data
        self.get_logger().info(f'Received ultrasonic data: {data}')
        self.process_ultrasonic_data(data)

    def process_ultrasonic_data(self, data):
        # Parse the ultrasonic data
        try:
            parts = data.split(',')
            distanceA = float(parts[0].split(':')[1])
            distanceB = float(parts[1].split(':')[1])
        except (IndexError, ValueError) as e:
            self.get_logger().error(f'Error parsing ultrasonic data: {e}')
            return

        # Implement obstacle avoidance logic
        if (distanceA > 0 and distanceA < 22) or (distanceB > 0 and distanceB < 22):
            self.stop_motors()
            self.reverse_motors()
            if distanceA < distanceB:
                self.turn_left()
            else:
                self.turn_right()
        elif (distanceA >= 22 and distanceA < 30) or (distanceB >= 22 and distanceB < 30):
            if distanceA < distanceB:
                self.minor_turn_left()
            else:
                self.minor_turn_right()
            self.forward_motors()
        else:
            self.forward_motors()

    def stop_motors(self):
        self.get_logger().info('Stopping motors')
        self.ser.write(b'S')

    def reverse_motors(self):
        self.get_logger().info('Reversing motors')
        self.ser.write(b'B')

    def turn_left(self):
        self.get_logger().info('Turning left')
        self.ser.write(b'L')

    def turn_right(self):
        self.get_logger().info('Turning right')
        self.ser.write(b'R')

    def minor_turn_left(self):
        self.get_logger().info('Minor turn left')
        self.ser.write(b'ML')

    def minor_turn_right(self):
        self.get_logger().info('Minor turn right')
        self.ser.write(b'MR')

    def forward_motors(self):
        self.get_logger().info('Moving forward')
        self.ser.write(b'F')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
     