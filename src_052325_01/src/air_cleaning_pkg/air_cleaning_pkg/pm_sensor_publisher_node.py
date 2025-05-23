import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class AirQualityPublisher(Node):
    def __init__(self):
        super().__init__('air_quality_publisher')
        self.publisher_ = self.create_publisher(String, 'airquality', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 1000000, timeout=0.01)  
        self.timer = self.create_timer(0.1, self.publish_air_quality_data)  # Publish air quality data every 0.1 seconds

    def publish_air_quality_data(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received from sensor: {line}")
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error reading from serial port: {e}")

def main(args=None):
   rclpy.init(args=args)
   node = AirQualityPublisher() 
   rclpy.spin(node)
   rclpy.shutdown()

if __name__ == '__main__':
    main()
