import rclpy
from rclpy.node import Node
from lidar_serial import Lidar
from sensor_msgs.msg import LaserScan
import sys
class LidarPublisher(Node):

    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.lidar = Lidar(laser_scan=True)

    def scan(self):
        while True:
            msg = self.lidar.full_scan()
            self.publisher.publish(msg)
    

if __name__ == '__main__':
    rclpy.init(args=None)
    pub = LidarPublisher()
    try:
        pub.scan()
    except KeyboardInterrupt:
        pub.destroy_node()
        rclpy.shutdown()
    
    