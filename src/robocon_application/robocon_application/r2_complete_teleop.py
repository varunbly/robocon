#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class R2CompleteTeleop(Node):
    def __init__(self):
        super().__init__('r2_complete_teleop')
        
        # ... (Parameters) ...

        # ... (Parameters) ...

        # --- Parameters ---
        self.base_speed_scale = 15.0
        self.arm_speed_scale = 0.05 # Position increment per loop

        # --- Limits (from rrr_teleop.py) ---
        self.LIMITS = {
            'joint0': (-3.14, 3.14),
            'joint1': (-1.57, 1.57),
            'joint2': (-2.5, 2.5),
            'joint3': (-2.5, 2.5),
            'joint4': (-3.14, 3.14),
            'gripper': (-0.18, 0.1)
        }

        # --- Publishers ---
        # Base (R2 Wheels)
        self.pub_fl = self.create_publisher(Float64, '/R2/front_left_wheel/cmd_vel', 10)
        self.pub_fr = self.create_publisher(Float64, '/R2/front_right_wheel/cmd_vel', 10)
        self.pub_ml = self.create_publisher(Float64, '/R2/mid_left_wheel/cmd_vel', 10)
        self.pub_mr = self.create_publisher(Float64, '/R2/mid_right_wheel/cmd_vel', 10)
        self.pub_bl = self.create_publisher(Float64, '/R2/back_left_wheel/cmd_vel', 10)
        self.pub_br = self.create_publisher(Float64, '/R2/back_right_wheel/cmd_vel', 10)

        # Arm Joints
        self.pub_j0 = self.create_publisher(Float64, '/rrr/joint0/cmd_pos', 10)
        self.pub_j1 = self.create_publisher(Float64, '/rrr/joint1/cmd_pos', 10)
        self.pub_j2 = self.create_publisher(Float64, '/rrr/joint2/cmd_pos', 10)
        self.pub_j3 = self.create_publisher(Float64, '/rrr/joint3/cmd_pos', 10)
        self.pub_j4 = self.create_publisher(Float64, '/rrr/joint4/cmd_pos', 10)
        self.pub_gripper = self.create_publisher(Float64, '/rrr/gripper/cmd_pos', 10)

        # --- Subscriber ---
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile_sensor_data)

        # --- Interior State for Arm Positions ---
        self.j0_pos = 0.0
        self.j1_pos = 0.0
        self.j2_pos = 0.0
        self.j3_pos = 0.0
        self.j4_pos = 0.0
        self.gripper_pos = 0.0

        self.get_logger().info("R2 Complete Teleop Node Started")
        self.get_logger().info("Base: Left Stick")
        self.get_logger().info("Arm J0 (Base): Right Stick X")
        self.get_logger().info("Arm J1 (Shoulder): Right Stick Y")
        self.get_logger().info("Arm J2 (Elbow): D-Pad UP/DOWN")
        self.get_logger().info("Arm J3 (Wrist P): D-Pad LEFT/RIGHT")
        self.get_logger().info("Arm J4 (Wrist R): LB/RB")
        self.get_logger().info("Gripper: A (Open) / B (Close)")

    def clamp(self, value, limit_name):
        low, high = self.LIMITS[limit_name]
        return max(low, min(value, high))

    def joy_callback(self, msg):
        # Debug: Print first few axes and buttons
        self.get_logger().info(f"Axes: {msg.axes[:4]} Buttons: {msg.buttons[:4]}")
        
        # --- Base Control (Left Stick) ---
        # Axis 1: Left Stick Up/Down (Forward/Backward)
        # Axis 0: Left Stick Left/Right (Turn)
        
        fwd = msg.axes[1] * self.base_speed_scale
        turn = msg.axes[0] * self.base_speed_scale

        if abs(fwd) > 0.1 or abs(turn) > 0.1:
            self.get_logger().info(f"Base Cmd: Fwd={fwd:.2f}, Turn={turn:.2f}")

        # Skid steer / Diff drive mixing
        left_vel = fwd - turn
        right_vel = fwd + turn
        
        # Invert left side as per r2_teleop behavior (observed negative publishing)
        msg_fl = Float64(); msg_fl.data = -left_vel
        msg_fr = Float64(); msg_fr.data = right_vel
        msg_ml = Float64(); msg_ml.data = -left_vel
        msg_mr = Float64(); msg_mr.data = right_vel
        msg_bl = Float64(); msg_bl.data = -left_vel
        msg_br = Float64(); msg_br.data = right_vel

        self.pub_fl.publish(msg_fl)
        self.pub_fr.publish(msg_fr)
        self.pub_ml.publish(msg_ml)
        self.pub_mr.publish(msg_mr)
        self.pub_bl.publish(msg_bl)
        self.pub_br.publish(msg_br)


        # --- Arm Control ---
        
        # J0: Right Stick Left/Right (Axis 3)
        if abs(msg.axes[3]) > 0.1:
            self.j0_pos += msg.axes[3] * self.arm_speed_scale
            self.get_logger().info(f"J0 Move: {self.j0_pos:.2f}")
        self.j0_pos = self.clamp(self.j0_pos, 'joint0')

        # J1: Right Stick Up/Down (Axis 4)
        if abs(msg.axes[4]) > 0.1:
            self.j1_pos += msg.axes[4] * self.arm_speed_scale
            self.get_logger().info(f"J1 Move: {self.j1_pos:.2f}")
        self.j1_pos = self.clamp(self.j1_pos, 'joint1')

        # J2: D-Pad Up/Down (Axis 7)
        if abs(msg.axes[7]) > 0.1:
            self.j2_pos += msg.axes[7] * self.arm_speed_scale
            self.get_logger().info(f"J2 Move: {self.j2_pos:.2f}")
        self.j2_pos = self.clamp(self.j2_pos, 'joint2')
            
        # J3: D-Pad Left/Right (Axis 6)
        if abs(msg.axes[6]) > 0.1:
            self.j3_pos += msg.axes[6] * self.arm_speed_scale
            self.get_logger().info(f"J3 Move: {self.j3_pos:.2f}")
        self.j3_pos = self.clamp(self.j3_pos, 'joint3')

        # J4: LB (Button 4) / RB (Button 5)
        if msg.buttons[4]: # LB - Rotate Left
            self.j4_pos -= self.arm_speed_scale
            self.get_logger().info("LB Pressed (J4-)")
        if msg.buttons[5]: # RB - Rotate Right
            self.j4_pos += self.arm_speed_scale
            self.get_logger().info("RB Pressed (J4+)")
        self.j4_pos = self.clamp(self.j4_pos, 'joint4')

        # Gripper: A (Button 0) / B (Button 1)
        # rrr_teleop: 'g' (Open) -> +Step, 't' (Close) -> -Step
        # Mapping: A -> Open (+), B -> Close (-)
        if msg.buttons[0]: # A - Open
            self.gripper_pos += 0.01 # Using smaller step for gripper
            self.get_logger().info("A Pressed (Open)")
        if msg.buttons[1]: # B - Close
            self.gripper_pos -= 0.01
            self.get_logger().info("B Pressed (Close)")
        self.gripper_pos = self.clamp(self.gripper_pos, 'gripper')

        # Publish Arm Positions
        p_j0 = Float64(); p_j0.data = self.j0_pos
        p_j1 = Float64(); p_j1.data = self.j1_pos
        p_j2 = Float64(); p_j2.data = self.j2_pos
        p_j3 = Float64(); p_j3.data = self.j3_pos
        p_j4 = Float64(); p_j4.data = self.j4_pos
        p_grip = Float64(); p_grip.data = self.gripper_pos

        self.pub_j0.publish(p_j0)
        self.pub_j1.publish(p_j1)
        self.pub_j2.publish(p_j2)
        self.pub_j3.publish(p_j3)
        self.pub_j4.publish(p_j4)
        self.pub_gripper.publish(p_grip)

def main(args=None):
    rclpy.init(args=args)
    node = R2CompleteTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
