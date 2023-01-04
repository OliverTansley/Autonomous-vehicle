import rclpy
from rclpy.node import Node
import LineDetection.LineDetection as LD

from sensor_msgs.msg import LaserScan

class LaserScanner(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(LaserScan,"/scan",self.callback,10)
        self.run()

    @staticmethod
    def callback(msg):
    
        LD.LineDetector.make_seed_segments(msg.ranges)
    

    def run(self):
        rclpy.create_node("scan_printer")
        rclpy.spin(self)


if __name__ == "__main__":
    rclpy.init()
    ls = LaserScanner()