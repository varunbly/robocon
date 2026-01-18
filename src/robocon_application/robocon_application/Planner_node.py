#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
import math

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'car_cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'planner_status', 10)
        
        # Subscribers
        self.goal_sub = self.create_subscription(Point, 'goal_position', self.goal_callback, 10)
        self.current_pose_sub = self.create_subscription(PoseStamped, 'current_pose', self.pose_callback, 10)
        self.obstacle_sub = self.create_subscription(Point, 'detected_obstacle', self.obstacle_callback, 10)
        
        # Timer for path execution
        self.timer = self.create_timer(0.1, self.execute_path)
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.path_active = False
        self.obstacles = []
        
        self.get_logger().info('Planner Node Started')
        
    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.path_active = True
        self.plan_path()
        self.get_logger().info(f'New goal: ({msg.x}, {msg.y})')
        
    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        
    def obstacle_callback(self, msg):
        # Add obstacle to list
        self.obstacles.append((msg.x, msg.y))
        
    def plan_path(self):
        # Simple straight-line path planning
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        # Create waypoints from current to goal
        num_points = 10
        for i in range(num_points + 1):
            t = i / num_points
            x = self.current_x + t * (self.goal_x - self.current_x)
            y = self.current_y + t * (self.goal_y - self.current_y)
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)
            
        self.path_pub.publish(path)
        
    def execute_path(self):
        if not self.path_active:
            return
            
        # Calculate distance to goal
        distance = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        
        if distance < 0.1:  # Reached goal
            self.stop_robot()
            self.path_active = False
            self.publish_status("goal_reached")
        else:
            self.move_towards_goal()
            
    def move_towards_goal(self):
        # Simple proportional controller
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_yaw
        
        # Normalize angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        # Control commands
        linear_vel = min(1.0, math.sqrt(dx*dx + dy*dy))
        angular_vel = 2.0 * angle_diff
        
        self.publish_velocity(linear_vel, angular_vel)
        
    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
        
    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)
        
    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main():
    rclpy.init()
    node = PlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()