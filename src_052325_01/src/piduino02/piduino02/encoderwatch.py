import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class EncoderWatch(Node):
    def __init__(self):
        super().__init__('encoderwatch')
        self.subscription = self.create_subscription(
            String,
            'encoder_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received encoder data: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderWatch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
     