#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import sys

class JointPublisherNode(Node):
    def __init__(self):
        super().__init__('joint_publisher_node')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Joint Publisher Node has been started.')
        self.get_logger().info('Enter joint index (0-6) and angle (radians) separated by space (e.g., "0 1.57").')
        self.get_logger().info('Or enter "help" for instructions.')
        
        # Joint names matching the URDF
        self.joint_names = [
            'joint0', 
            'joint1', 
            'joint2', 
            'joint3', 
            'joint4', 
            'finger_left_joint', 
            'finger_right_joint'
        ]
        
        # Initial joint positions
        self.joint_positions = [0.0] * len(self.joint_names)
        
        # Thread for input
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input()
                if not user_input:
                    continue
                
                parts = user_input.strip().split()
                if len(parts) == 0:
                    continue
                    
                command = parts[0].lower()
                
                if command == 'help':
                    print("\nCommands:")
                    print("  <index> <angle>: Set joint at <index> to <angle> (radians)")
                    print("  all <angle>: Set all joints to <angle>")
                    print("  list: List current joint values")
                    print("  exit: Quit the node")
                    print("Joint Indices:")
                    for i, name in enumerate(self.joint_names):
                        print(f"  {i}: {name}")
                    print("")
                    continue
                
                if command == 'exit':
                    print("Exiting...")
                    rclpy.shutdown()
                    return

                if command == 'list':
                    print("\nCurrent Joint Values:")
                    for i, (name, val) in enumerate(zip(self.joint_names, self.joint_positions)):
                        print(f"  {i}: {name} = {val:.3f}")
                    print("")
                    continue

                if command == 'all':
                    try:
                        val = float(parts[1])
                        for i in range(len(self.joint_positions)):
                            self.joint_positions[i] = val
                        print(f"All joints set to {val}")
                    except (IndexError, ValueError):
                        print("Usage: all <angle>")
                    continue

                # Parse index value
                try:
                    idx = int(parts[0])
                    val = float(parts[1])
                    
                    if 0 <= idx < len(self.joint_names):
                        self.joint_positions[idx] = val
                        print(f"Set {self.joint_names[idx]} to {val}")
                    else:
                        print(f"Index out of range (0-{len(self.joint_names)-1})")
                except (IndexError, ValueError):
                    print("Invalid input. Format: <index> <angle>")
                    
            except EOFError:
                break
            except Exception as e:
                print(f"Error: {e}")

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = []
        msg.effort = []
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
