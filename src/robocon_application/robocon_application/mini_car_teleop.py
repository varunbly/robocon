#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select

msg = """
Control Mini Car (Velocity Control)
---------------------------
Controls:
   w: move forward
   s: move backward
   a: turn left
   d: turn right
   
   q: quit
"""

class MiniCarTeleop(Node):
    def __init__(self):
        super().__init__('mini_car_teleop')
        
        # Velocity Publishers
        self.pub_left = self.create_publisher(Float64, '/mini_car/left_wheel/cmd_vel', 10)
        self.pub_right = self.create_publisher(Float64, '/mini_car/right_wheel/cmd_vel', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main():
    rclpy.init()
    node = MiniCarTeleop()
    
    print(msg)
    
    try:
        while True:
            key = node.get_key()
            
            v_l = 0.0
            v_r = 0.0
            speed = 10.0
            
            if key == 'w':
                v_l = speed
                v_r = speed
                
            elif key == 's':
                v_l = -speed
                v_r = -speed
                
            elif key == 'a':
                v_l = -speed
                v_r = speed
                
            elif key == 'd':
                v_l = speed
                v_r = -speed
                
            elif key == 'q':
                break
            
            msg_l = Float64()
            msg_l.data = v_l
            msg_r = Float64()
            msg_r.data = v_r
            
            node.pub_left.publish(msg_l)
            node.pub_right.publish(msg_r)

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
