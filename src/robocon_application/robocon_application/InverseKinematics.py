#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import math

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        
        # Publishers for joint commands
        self.joint1_pub = self.create_publisher(Float64, 'arm_joint1_cmd', 10)
        self.joint2_pub = self.create_publisher(Float64, 'arm_joint2_cmd', 10)
        
        # Subscribers
        self.target_sub = self.create_subscription(Point, 'arm_target', self.target_callback, 10)
        
        # Arm parameters
        self.link1_length = 0.3  # Length of first arm link
        self.link2_length = 0.25  # Length of second arm link
        
        self.get_logger().info('Inverse Kinematics Node Started')
        
    def target_callback(self, msg):
        # Calculate inverse kinematics for target position
        x = msg.x
        y = msg.y
        z = msg.z
        
        # 2D IK calculation (ignoring z for simplicity)
        joint_angles = self.calculate_ik(x, y)
        
        if joint_angles:
            self.publish_joint_commands(joint_angles[0], joint_angles[1])
        else:
            self.get_logger().warn('Target position unreachable')
            
    def calculate_ik(self, x, y):
        # 2-DOF inverse kinematics
        l1 = self.link1_length
        l2 = self.link2_length
        
        # Distance to target
        distance = math.sqrt(x*x + y*y)
        
        # Check if target is reachable
        if distance > (l1 + l2) or distance < abs(l1 - l2):
            return None
            
        # Calculate joint angles using law of cosines
        cos_theta2 = (distance*distance - l1*l1 - l2*l2) / (2 * l1 * l2)
        
        # Clamp to valid range
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        
        # Two solutions for elbow up/down
        theta2 = math.acos(cos_theta2)  # Elbow up
        
        # Calculate theta1
        alpha = math.atan2(y, x)
        beta = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
        theta1 = alpha - beta
        
        return [theta1, theta2]
        
    def publish_joint_commands(self, joint1_angle, joint2_angle):
        # Publish joint commands
        joint1_msg = Float64()
        joint1_msg.data = joint1_angle
        self.joint1_pub.publish(joint1_msg)
        
        joint2_msg = Float64()
        joint2_msg.data = joint2_angle
        self.joint2_pub.publish(joint2_msg)
        
        self.get_logger().info(f'Joint commands: J1={joint1_angle:.2f}, J2={joint2_angle:.2f}')
        
    def pick_spear_position(self):
        # Predefined position for picking spears
        target = Point()
        target.x = 0.4
        target.y = 0.0
        target.z = 0.1
        self.target_callback(target)
        
    def home_position(self):
        # Return arm to home position
        self.publish_joint_commands(0.0, 0.0)

def main():
    rclpy.init()
    node = InverseKinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()