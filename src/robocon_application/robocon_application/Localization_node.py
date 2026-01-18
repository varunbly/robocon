#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'robot_pose', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.publish_pose)
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.get_logger().info('Localization Node Started')
        
    def odom_callback(self, msg):
        # Update position from odometry
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
    def imu_callback(self, msg):
        # Update orientation from IMU
        pass
        
    def publish_pose(self):
        # Publish current robot pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = np.sin(self.theta/2)
        msg.pose.pose.orientation.w = np.cos(self.theta/2)
        self.pose_pub.publish(msg)

def main():
    rclpy.init()
    node = LocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()