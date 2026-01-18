#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.publisher = self.create_publisher(Twist, 'car_cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Car Controller Started!')
        self.get_logger().info('Controls: W=Forward, S=Backward, A=Left, D=Right, Q=Quit')
        
    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)
        
    def timer_callback(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()
            
            if key == 'w':
                self.send_velocity(-2.0, 0.0)
            elif key == 's':
                self.send_velocity(2.0, 0.0)
            elif key == 'a':
                self.send_velocity(0.0, -1.0)
            elif key == 'd':
                self.send_velocity(0.0, 1.0)
            elif key == 'q':
                self.send_velocity(0.0, 0.0)
                rclpy.shutdown()
        else:
            self.send_velocity(0.0, 0.0)

def main():
    rclpy.init()
    
    # Set terminal to raw mode
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)
    
    controller = CarController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
