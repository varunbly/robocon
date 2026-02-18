import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys, select, termios, tty

msg = """
--------------------------------
   RRR Robot Arm Teleop
--------------------------------
   Base:      A / D
   Shoulder:  W / S
   Elbow:     I / K
   Wrist P:   J / L
   Wrist R:   U / O
   Gripper:   T (Close) / G (Open)

   CTRL-C to quit
--------------------------------
"""

# Joint Limits (Approximate from SDF)
LIMITS = {
    'joint0': (-3.14, 3.14),
    'joint1': (-1.57, 1.57),
    'joint2': (-2.5, 2.5),
    'joint3': (-2.5, 2.5),
    'joint4': (-3.14, 3.14),
    'gripper': (-0.18, 0.1)
}

STEP_SIZE = 0.1
GRIPPER_STEP = 0.01

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('rrr_teleop')
        
        # Publishers
        self.pub_j0 = self.create_publisher(Float64, '/rrr/joint0/cmd_pos', 10)
        self.pub_j1 = self.create_publisher(Float64, '/rrr/joint1/cmd_pos', 10)
        self.pub_j2 = self.create_publisher(Float64, '/rrr/joint2/cmd_pos', 10)
        self.pub_j3 = self.create_publisher(Float64, '/rrr/joint3/cmd_pos', 10)
        self.pub_j4 = self.create_publisher(Float64, '/rrr/joint4/cmd_pos', 10)
        self.pub_grip = self.create_publisher(Float64, '/rrr/gripper/cmd_pos', 10)

        # Initial Positions
        self.positions = {
            'joint0': 0.0, 'joint1': 0.0, 'joint2': 0.0,
            'joint3': 0.0, 'joint4': 0.0, 'gripper': 0.0
        }
        
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def pub(self):
        self.pub_j0.publish(Float64(data=self.positions['joint0']))
        self.pub_j1.publish(Float64(data=self.positions['joint1']))
        self.pub_j2.publish(Float64(data=self.positions['joint2']))
        self.pub_j3.publish(Float64(data=self.positions['joint3']))
        self.pub_j4.publish(Float64(data=self.positions['joint4']))
        self.pub_grip.publish(Float64(data=self.positions['gripper']))

    def clamp(self, name, value):
        low, high = LIMITS[name]
        return max(low, min(value, high))

def main():
    rclpy.init()
    node = ArmTeleop()
    print(msg)

    try:
        while True:
            key = node.getKey()
            if key == '\x03': # CTRL+C
                break
            
            # Key Logic
            if key == 'a': node.positions['joint0'] = node.clamp('joint0', node.positions['joint0'] + STEP_SIZE)
            elif key == 'd': node.positions['joint0'] = node.clamp('joint0', node.positions['joint0'] - STEP_SIZE)
            
            elif key == 'w': node.positions['joint1'] = node.clamp('joint1', node.positions['joint1'] + STEP_SIZE)
            elif key == 's': node.positions['joint1'] = node.clamp('joint1', node.positions['joint1'] - STEP_SIZE)
            
            elif key == 'i': node.positions['joint2'] = node.clamp('joint2', node.positions['joint2'] + STEP_SIZE)
            elif key == 'k': node.positions['joint2'] = node.clamp('joint2', node.positions['joint2'] - STEP_SIZE)

            elif key == 'j': node.positions['joint3'] = node.clamp('joint3', node.positions['joint3'] + STEP_SIZE)
            elif key == 'l': node.positions['joint3'] = node.clamp('joint3', node.positions['joint3'] - STEP_SIZE)

            elif key == 'u': node.positions['joint4'] = node.clamp('joint4', node.positions['joint4'] + STEP_SIZE)
            elif key == 'o': node.positions['joint4'] = node.clamp('joint4', node.positions['joint4'] - STEP_SIZE)

            elif key == 't': node.positions['gripper'] = node.clamp('gripper', node.positions['gripper'] - GRIPPER_STEP) # Close
            elif key == 'g': node.positions['gripper'] = node.clamp('gripper', node.positions['gripper'] + GRIPPER_STEP) # Open

            node.pub()

    except Exception as e:
        print(e)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()