#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

msg = """
Control R1 Robot
---------------------------
Controls:
   w: forward
   s: backward
   a: turn left (FAST)
   d: turn right (FAST)
   x: stop
   q: quit
"""

class R1Teleop(Node):
    def __init__(self):
        super().__init__('r1_teleop')
        self.pub = self.create_publisher(Twist, '/R1/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main():
    rclpy.init()
    node = R1Teleop()
    
    print(msg)
    
    try:
        while True:
            key = node.get_key()
            twist = Twist()
            
            if key == 'w':
                twist.linear.x = 0.5
            elif key == 's':
                twist.linear.x = -0.5
            elif key == 'a':
                twist.angular.z = 2.5  # Increased > 2x (was 1.0)
            elif key == 'd':
                twist.angular.z = -2.5 # Increased > 2x (was 1.0)
            elif key == 'x':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == 'q':
                break

            node.pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
