import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import board
import busio
import adafruit_ssd1306

class OledDisplay(Node):
    def __init__(self):
        super().__init__('oled_display')
        self.subscription = self.create_subscription(
            String,
            'ultrasonic_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize I2C interface
        i2c = busio.I2C(board.SCL, board.SDA)
        self.display = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

        # Clear the display
        self.display.fill(0)
        self.display.show()

        # Timer to update display
        self.timer = self.create_timer(0.5, self.update_display)
        self.latest_data = ""

    def listener_callback(self, msg):
        self.latest_data = msg.data

    def update_display(self):
        self.display.fill(0)
        self.display.text(self.latest_data, 0, 0, 1)
        self.display.show()

def main(args=None):
    rclpy.init(args=args)
    node = OledDisplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
