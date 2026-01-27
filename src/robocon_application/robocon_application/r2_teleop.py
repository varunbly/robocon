#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

from std_msgs.msg import Float64

msg = """
Control R2 Robot (Independent Wheel Control)
---------------------------
Controls (Hold to move):
   w: move all forward
   s: move all backward
   a: pivot left
   d: pivot right
   
   Specific Wheels (Example):
   t: mid wheels forward
   g: mid wheels backward
   
   q: quit
"""

class R2Teleop(Node):
    def __init__(self):
        super().__init__('r2_teleop')
        
        # Original DiffDrive publisher (optional, keeping for safety or mixed use)
        self.pub_cmd_vel = self.create_publisher(Twist, '/R2/cmd_vel', 10)
        
        # Individual Wheel Publishers - Using "A_" prefix as requested
        # Front
        self.pub_A_front_left = self.create_publisher(Float64, '/R2/front_left_wheel/cmd_vel', 10)
        self.pub_A_front_right = self.create_publisher(Float64, '/R2/front_right_wheel/cmd_vel', 10)
        # Mid
        self.pub_A_mid_left = self.create_publisher(Float64, '/R2/mid_left_wheel/cmd_vel', 10)
        self.pub_A_mid_right = self.create_publisher(Float64, '/R2/mid_right_wheel/cmd_vel', 10)
        # Back
        self.pub_A_back_left = self.create_publisher(Float64, '/R2/back_left_wheel/cmd_vel', 10)
        self.pub_A_back_right = self.create_publisher(Float64, '/R2/back_right_wheel/cmd_vel', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # Non-blocking check with 0.1s timeout
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main():
    rclpy.init()
    node = R2Teleop()
    
    print(msg)
    
    try:
        while True:
            key = node.get_key()
            
            # Initialize velocities for this loop iteration
            v_fl = 0.0 # Front Left
            v_fr = 0.0 # Front Right
            v_ml = 0.0 # Mid Left
            v_mr = 0.0 # Mid Right
            v_bl = 0.0 # Back Left
            v_br = 0.0 # Back Right
            
            # Key mappings
            if key == 'w':
                # All Forward
                speed = 50.0 # Radians/sec
                v_fl = v_fr = v_ml = v_mr = v_bl = v_br = speed
                
            elif key == 's':
                # All Backward
                speed = -50.0
                v_fl = v_fr = v_ml = v_mr = v_bl = v_br = speed
                
            elif key == 'a':
                # Pivot Left (Left side back, Right side fwd)
                speed = 50.0
                v_fr = v_mr = v_br = speed
                v_fl = v_ml = v_bl = -speed
                
            elif key == 'd':
                # Pivot Right (Left side fwd, Right side back)
                speed = 50.0
                v_fl = v_ml = v_bl = speed
                v_fr = v_mr = v_br = -speed
            
            # elif key == 't':
            #      # Only Mid Forward
            #      v_ml = 5.0
            #      v_mr = 5.0
                 
            # elif key == 'g':
            #      # Only Mid Backward
            #      v_ml = -5.0
            #      v_mr = -5.0

            elif key == 'q':
                break
            
            # Publish to all wheels
            # Create Float64 messages
            msg_fl = Float64(); msg_fl.data = v_fl
            msg_fr = Float64(); msg_fr.data = v_fr
            msg_ml = Float64(); msg_ml.data = v_ml
            msg_mr = Float64(); msg_mr.data = v_mr
            msg_bl = Float64(); msg_bl.data = v_bl
            msg_br = Float64(); msg_br.data = v_br
            
            node.pub_A_front_left.publish(msg_fl)
            node.pub_A_front_right.publish(msg_fr)
            node.pub_A_mid_left.publish(msg_ml)
            node.pub_A_mid_right.publish(msg_mr)
            node.pub_A_back_left.publish(msg_bl)
            node.pub_A_back_right.publish(msg_br)
            
            # Also publish zero to main cmd_vel to ensure DiffDrive doesn't fight (optional)
            twist = Twist()
            node.pub_cmd_vel.publish(twist)

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
