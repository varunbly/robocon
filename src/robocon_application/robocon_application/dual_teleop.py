#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

msg = """
Control Two Robots!
---------------------------
Current Robot: {robot}

Controls:
   w: forward
   s: backward
   a: turn left
   d: turn right
   x: stop
   
   1: Switch to R1
   2: Switch to R2
   
   q: quit
"""

class DualTeleop(Node):
    def __init__(self):
        super().__init__('dual_teleop')
        self.pub_r1 = self.create_publisher(Twist, '/R1/cmd_vel', 10)
        self.pub_r2 = self.create_publisher(Twist, '/R2/cmd_vel', 10)
        self.active_robot = 'R1' # Default to R1
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_menu(self):
        print(msg.format(robot=self.active_robot))

def main():
    rclpy.init()
    node = DualTeleop()
    
    node.print_menu()
    
    try:
        while True:
            key = node.get_key()
            twist = Twist()
            
            # Switch Robot
            if key == '1':
                node.active_robot = 'R1'
                print(f"Switched to R1")
                node.print_menu()
                continue
            elif key == '2':
                node.active_robot = 'R2'
                print(f"Switched to R2")
                node.print_menu()
                continue
            
            # Movement
            if key == 'w':
                twist.linear.x = 0.5
            elif key == 's':
                twist.linear.x = -0.5
            elif key == 'a':
                twist.angular.z = 1.0
            elif key == 'd':
                twist.angular.z = -1.0
            elif key == 'x':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == 'q':
                break

            # Publish to active robot
            if node.active_robot == 'R1':
                node.pub_r1.publish(twist)
            else:
                node.pub_r2.publish(twist)

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
