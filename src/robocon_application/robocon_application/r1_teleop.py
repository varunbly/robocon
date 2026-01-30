#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
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
        self.pub_fl = self.create_publisher(
            Float64, '/model/R1/joint/front_left_wheel_joint/cmd_vel', 10)
        self.pub_bl = self.create_publisher(
            Float64, '/model/R1/joint/back_left_wheel_joint/cmd_vel', 10)
        self.pub_fr = self.create_publisher(
            Float64, '/model/R1/joint/front_right_wheel_joint/cmd_vel', 10)
        self.pub_br = self.create_publisher(
            Float64, '/model/R1/joint/back_right_wheel_joint/cmd_vel', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)   # BLOCKS until key is pressed
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    
    def publish_wheels(self, fl, bl, fr, br):
        self.pub_fl.publish(Float64(data=fl))
        self.pub_bl.publish(Float64(data=bl))
        self.pub_fr.publish(Float64(data=fr))
        self.pub_br.publish(Float64(data=br))

def main():
    rclpy.init()
    node = R1Teleop()
    
    print(msg)
    
    try:
        while True:
            key = node.get_key()
            twist = Twist()
            fl=bl=fr=br=0.0
            if key == 'w':
                fl=bl=fr=br=5.0
            elif key == 's':
                fl=bl=fr=br=-5.0
            elif key == 'a':
                fl=bl=-5.0
                fr=br=5.0
            elif key == 'd':
                fl=bl=5.0
                fr=br=-5.0
            elif key == 'q':    
                break

            node.publish_wheels(fl,bl,fr,br)

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
