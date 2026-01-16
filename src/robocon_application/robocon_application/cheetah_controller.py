#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import math

class CheetahController(Node):
    def __init__(self):
        super().__init__('cheetah_controller')
        
        self.joint_pub = self.create_publisher(Float64MultiArray, '/model/mit_cheetah/joint/fr_joint/cmd_pos', 10)
        self.joint_pub2 = self.create_publisher(Float64MultiArray, '/model/mit_cheetah/joint/fl_joint/cmd_pos', 10)
        self.joint_pub3 = self.create_publisher(Float64MultiArray, '/model/mit_cheetah/joint/rr_joint/cmd_pos', 10)
        self.joint_pub4 = self.create_publisher(Float64MultiArray, '/model/mit_cheetah/joint/rl_joint/cmd_pos', 10)
        
        self.cmd_sub = self.create_subscription(Twist, 'cheetah_cmd_vel', self.cmd_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.t = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        
    def cmd_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        
    def control_loop(self):
        self.t += 0.1
        
        # Simple walking gait
        if abs(self.linear_x) > 0.1:
            freq = 2.0 * abs(self.linear_x)
            amplitude = 0.3
            
            # Trotting gait - diagonal legs move together
            fr_pos = amplitude * math.sin(freq * self.t)
            fl_pos = amplitude * math.sin(freq * self.t + math.pi)
            rr_pos = amplitude * math.sin(freq * self.t + math.pi)
            rl_pos = amplitude * math.sin(freq * self.t)
        else:
            fr_pos = fl_pos = rr_pos = rl_pos = 0.0
            
        # Add turning
        turn_offset = self.angular_z * 0.2
        fr_pos += turn_offset
        rr_pos += turn_offset
        fl_pos -= turn_offset
        rl_pos -= turn_offset
        
        # Publish joint commands
        msg = Float64MultiArray()
        msg.data = [fr_pos]
        self.joint_pub.publish(msg)
        
        msg.data = [fl_pos]
        self.joint_pub2.publish(msg)
        
        msg.data = [rr_pos]
        self.joint_pub3.publish(msg)
        
        msg.data = [rl_pos]
        self.joint_pub4.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = CheetahController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()