#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control the Cheetah Robot
---------------------------
   w: forward
   s: backward
   a: turn left
   d: turn right
   x: stop
   q: quit
"""

class CheetahTeleop(Node):
    def __init__(self):
        super().__init__('cheetah_teleop')
        self.pub = self.create_publisher(Twist, 'cheetah_cmd_vel', 10)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main():
    rclpy.init()
    node = CheetahTeleop()
    node.settings = termios.tcgetattr(sys.stdin)
    
    print(msg)
    
    try:
        while True:
            key = node.get_key()
            twist = Twist()
            
            if key == 'w':
                twist.linear.x = 1.0
            elif key == 's':
                twist.linear.x = -1.0
            elif key == 'a':
                twist.angular.z = 1.0
            elif key == 'd':
                twist.angular.z = -1.0
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
