import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8

MAX_PWM = int(0.95 * 255)  # 95% of 255 → 242 Motor driver requires this, DO NOT EXCEED!
DEADZONE = 0.07		   # ignore minor joystick inputs

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # Subscribe to joystick input
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        # Create publishers for left and right motor pins
        self.left_motor_pin1_pub = self.create_publisher(UInt8, '/left_motor/pin1', 10)
        self.left_motor_pin2_pub = self.create_publisher(UInt8, '/left_motor/pin2', 10)
        self.right_motor_pin1_pub = self.create_publisher(UInt8, '/right_motor/pin1', 10)
        self.right_motor_pin2_pub = self.create_publisher(UInt8, '/right_motor/pin2', 10)

    def joy_callback(self, msg: Joy):
        # Index 1: Left stick Y, Index 3: Right stick Y
        left_input = msg.axes[1]
        right_input = msg.axes[4]

        # Process each motor direction
        self.process_motor_input(left_input, 
                                 self.left_motor_pin1_pub, 
                                 self.left_motor_pin2_pub, 
                                 'left')

        self.process_motor_input(right_input, 
                                 self.right_motor_pin1_pub, 
                                 self.right_motor_pin2_pub, 
                                 'right')

    def process_motor_input(self, value, pin1_pub, pin2_pub, label):
        # deadzone 
        if abs(value) < DEADZONE:
            value = 0.0

        pwm = int(abs(value) * MAX_PWM)
        pin1_msg = UInt8()
        pin2_msg = UInt8()

        if value > 0:
            # Forward: pin1 gets PWM, pin2 is LOW
            pin1_msg.data = pwm
            pin2_msg.data = 0
        elif value < 0:
            # Reverse: pin2 gets PWM, pin1 is LOW
            pin1_msg.data = 0
            pin2_msg.data = pwm
        else:
            # Stop
            pin1_msg.data = 0
            pin2_msg.data = 0

        # Publish to motor pins
        pin1_pub.publish(pin1_msg)
        pin2_pub.publish(pin2_msg)

        self.get_logger().info(
            f"{label.capitalize()} motor → pin1: {pin1_msg.data}, pin2: {pin2_msg.data}"
        )

def main(args=None):
    rclpy.init(args=args)
    motor_driver = MotorDriver()
    rclpy.spin(motor_driver)
    motor_driver.destroy_node()
    rclpy.shutdown()
