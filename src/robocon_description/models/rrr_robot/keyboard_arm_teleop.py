#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Joint names must match the order in arm_controllers.yaml
JOINTS = [
    "joint1",
    "joint_new",
    "joint2",
    "joint3",
    "joint4",
    "finger_left_joint"
]

STEP = 0.05

class KeyboardArmTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_arm_teleop')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.pos = [0.0] * len(JOINTS)
        self.get_logger().info("Keyboard Teleop Started. Use keys to move.")

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def send(self):
        traj = JointTrajectory()
        traj.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = self.pos
        point.time_from_start.sec = 1  # Move to position in 1 second

        traj.points.append(point)
        self.pub.publish(traj)

    def run(self):
        print("""
----------------------------------
Control Your Arm:
----------------------------------
W / S : joint1 (Base)
A / D : joint_new (Shoulder)
I / K : joint2 (Elbow)
J / L : joint3 (Forearm)
U / O : joint4 (Wrist)   <-- ADDED
Q / E : gripper          <-- GRIPPER
X     : exit
----------------------------------
""")
        while rclpy.ok():
            key = self.get_key()

            if key == 'w': self.pos[0] += STEP
            elif key == 's': self.pos[0] -= STEP
            elif key == 'a': self.pos[1] += STEP
            elif key == 'd': self.pos[1] -= STEP
            elif key == 'i': self.pos[2] += STEP
            elif key == 'k': self.pos[2] -= STEP
            elif key == 'j': self.pos[3] += STEP
            elif key == 'l': self.pos[3] -= STEP
            # Added Joint 4 Controls
            elif key == 'u': self.pos[4] += STEP
            elif key == 'o': self.pos[4] -= STEP
            # Gripper Controls
            elif key == 'q': self.pos[5] += STEP
            elif key == 'e': self.pos[5] -= STEP
            elif key == 'x': break

            self.send()

def main():
    rclpy.init()
    node = KeyboardArmTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()