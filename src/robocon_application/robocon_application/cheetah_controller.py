#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

class CheetahController(Node):
    def __init__(self):
        super().__init__('cheetah_controller')
        
        # Hip publishers
        self.fr_hip_pub = self.create_publisher(Float64, '/model/mit_cheetah/joint/fr_hip/cmd_pos', 10)
        self.fl_hip_pub = self.create_publisher(Float64, '/model/mit_cheetah/joint/fl_hip/cmd_pos', 10)
        self.rr_hip_pub = self.create_publisher(Float64, '/model/mit_cheetah/joint/rr_hip/cmd_pos', 10)
        self.rl_hip_pub = self.create_publisher(Float64, '/model/mit_cheetah/joint/rl_hip/cmd_pos', 10)
        
        # Knee publishers
        self.fr_knee_pub = self.create_publisher(Float64, '/model/mit_cheetah/joint/fr_knee/cmd_pos', 10)
        self.fl_knee_pub = self.create_publisher(Float64, '/model/mit_cheetah/joint/fl_knee/cmd_pos', 10)
        self.rr_knee_pub = self.create_publisher(Float64, '/model/mit_cheetah/joint/rr_knee/cmd_pos', 10)
        self.rl_knee_pub = self.create_publisher(Float64, '/model/mit_cheetah/joint/rl_knee/cmd_pos', 10)
        
        self.cmd_sub = self.create_subscription(Twist, 'cheetah_cmd_vel', self.cmd_callback, 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.t = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        
    def cmd_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        
    def control_loop(self):
        self.t += 0.05
        
        if abs(self.linear_x) > 0.01:
            freq = 3.0 * abs(self.linear_x)
            
            # Trotting gait
            phase_fr = freq * self.t
            phase_fl = freq * self.t + math.pi
            phase_rr = freq * self.t + math.pi
            phase_rl = freq * self.t
            
            # Hip angles - swing forward/back
            fr_hip = 0.5 * math.sin(phase_fr)
            fl_hip = 0.5 * math.sin(phase_fl)
            rr_hip = 0.5 * math.sin(phase_rr)
            rl_hip = 0.5 * math.sin(phase_rl)
            
            # Knee angles - bend more when leg is in air
            fr_knee = -1.2 - 0.6 * (1 + math.cos(phase_fr))
            fl_knee = -1.2 - 0.6 * (1 + math.cos(phase_fl))
            rr_knee = -1.2 - 0.6 * (1 + math.cos(phase_rr))
            rl_knee = -1.2 - 0.6 * (1 + math.cos(phase_rl))
        else:
            fr_hip = fl_hip = rr_hip = rl_hip = 0.0
            fr_knee = fl_knee = rr_knee = rl_knee = -1.2
        
        # Turning
        turn = self.angular_z * 0.3
        fr_hip += turn
        rr_hip += turn
        fl_hip -= turn
        rl_hip -= turn
        
        # Publish
        msg = Float64()
        
        msg.data = fr_hip
        self.fr_hip_pub.publish(msg)
        msg.data = fr_knee
        self.fr_knee_pub.publish(msg)
        
        msg.data = fl_hip
        self.fl_hip_pub.publish(msg)
        msg.data = fl_knee
        self.fl_knee_pub.publish(msg)
        
        msg.data = rr_hip
        self.rr_hip_pub.publish(msg)
        msg.data = rr_knee
        self.rr_knee_pub.publish(msg)
        
        msg.data = rl_hip
        self.rl_hip_pub.publish(msg)
        msg.data = rl_knee
        self.rl_knee_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = CheetahController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()