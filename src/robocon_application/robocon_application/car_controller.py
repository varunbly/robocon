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
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Car Controller Started!')
        self.get_logger().info('Controls: W=Forward, S=Backward, A=Left, D=Right, Q=Quit')
        self.last_key = None
        
    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)
        
    def timer_callback(self):
        key = None
        # Read all available input to prevent buffer buildup
        while select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()
            
        if key:
            self.last_key = key
            if key == 'w':
                self.send_velocity(2.0, 0.0)  # Forward
            elif key == 's':
                self.send_velocity(-2.0, 0.0)  # Backward
            elif key == 'a':
                self.send_velocity(0.0, 1.0)  # Left
            elif key == 'd':
                self.send_velocity(0.0, -1.0)  # Right
            elif key == ' ':  # Spacebar to stop
                self.send_velocity(0.0, 0.0)
                self.last_key = None
            elif key == 'q':
                self.send_velocity(0.0, 0.0)
                rclpy.shutdown()
        else:
            # Always send stop command when no key is pressed
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
