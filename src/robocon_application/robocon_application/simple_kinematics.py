#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class SimpleKinematics(Node):
    def __init__(self):
        super().__init__('simple_kinematics_node')

        # --- PARAMETERS ---
        # Adjust these to match your robot's physical dimensions in the SDF!
        self.wheel_separation = 0.40  # Distance between left and right wheels (meters)
        self.wheel_radius = 0.05      # Radius of the wheel (meters)

        # --- SUBSCRIBER ---
        # Listens to the command from Nav2
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Nav2 publishes here
            self.cmd_vel_callback,
            10)

        # --- PUBLISHERS ---
        # Left Side Publishers
        self.pub_fl = self.create_publisher(Float64, '/R2/front_left_wheel/cmd_vel', 10)
        self.pub_ml = self.create_publisher(Float64, '/R2/mid_left_wheel/cmd_vel', 10)
        self.pub_bl = self.create_publisher(Float64, '/R2/back_left_wheel/cmd_vel', 10)

        # Right Side Publishers
        self.pub_fr = self.create_publisher(Float64, '/R2/front_right_wheel/cmd_vel', 10)
        self.pub_mr = self.create_publisher(Float64, '/R2/mid_right_wheel/cmd_vel', 10)
        self.pub_br = self.create_publisher(Float64, '/R2/back_right_wheel/cmd_vel', 10)

        self.get_logger().info('Kinematics Node Started. Listening on /cmd_vel...')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # --- DIFFERENTIAL DRIVE MATH ---
        # Calculate velocity for left and right sides
        # v_left = (v - w * separation / 2)
        # v_right = (v + w * separation / 2)
        vel_left_linear = linear_x - (angular_z * self.wheel_separation / 2.0)
        vel_right_linear = linear_x + (angular_z * self.wheel_separation / 2.0)

        # Convert Linear Velocity (m/s) to Angular Velocity (rad/s) for the Joint Controller
        # ang_vel = linear_vel / radius
        vel_left_rads = vel_left_linear / self.wheel_radius
        vel_right_rads = vel_right_linear / self.wheel_radius

        # Create message
        msg_left = Float64()
        msg_left.data = float(vel_left_rads)

        msg_right = Float64()
        msg_right.data = float(vel_right_rads)

        # --- PUBLISH TO ALL 6 WHEELS ---
        # Left side
        self.pub_fl.publish(msg_left)
        self.pub_ml.publish(msg_left)
        self.pub_bl.publish(msg_left)

        # Right side
        self.pub_fr.publish(msg_right)
        self.pub_mr.publish(msg_right)
        self.pub_br.publish(msg_right)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()