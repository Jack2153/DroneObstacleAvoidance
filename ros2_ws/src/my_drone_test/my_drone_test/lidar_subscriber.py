#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriberNode(Node):
    goLocation = True
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.lidar_subscriber_ = self.create_subscription(
            LaserScan, "/lidar", self.lidar_callback, 10)
        

    def lidar_callback(self, msg: LaserScan):
        self.get_logger().info(str(msg.ranges))
       
def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()