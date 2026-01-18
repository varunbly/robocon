#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.pose_callback, 10)
        
        # Timer for map publishing
        self.timer = self.create_timer(1.0, self.publish_map)
        
        # Map parameters
        self.map_width = 200
        self.map_height = 200
        self.map_resolution = 0.05  # meters per pixel
        self.map_origin_x = -5.0
        self.map_origin_y = -5.0
        
        # Initialize map
        self.occupancy_map = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.get_logger().info('Mapping Node Started')
        
    def pose_callback(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        
    def lidar_callback(self, msg):
        # Process laser scan data
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Update map with laser data
        for i, (range_val, angle) in enumerate(zip(ranges, angles)):
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
                
            # Calculate obstacle position in world coordinates
            world_angle = self.robot_yaw + angle
            obstacle_x = self.robot_x + range_val * np.cos(world_angle)
            obstacle_y = self.robot_y + range_val * np.sin(world_angle)
            
            # Convert to map coordinates
            map_x = int((obstacle_x - self.map_origin_x) / self.map_resolution)
            map_y = int((obstacle_y - self.map_origin_y) / self.map_resolution)
            
            # Update map if within bounds
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                self.occupancy_map[map_y, map_x] = 100  # Occupied
                
            # Mark free space along the ray
            self.mark_free_space(self.robot_x, self.robot_y, obstacle_x, obstacle_y)
            
    def mark_free_space(self, start_x, start_y, end_x, end_y):
        # Bresenham's line algorithm to mark free space
        x0 = int((start_x - self.map_origin_x) / self.map_resolution)
        y0 = int((start_y - self.map_origin_y) / self.map_resolution)
        x1 = int((end_x - self.map_origin_x) / self.map_resolution)
        y1 = int((end_y - self.map_origin_y) / self.map_resolution)
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.occupancy_map[y, x] == -1:  # Unknown space
                    self.occupancy_map[y, x] = 0  # Free space
                    
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
    def publish_map(self):
        # Publish occupancy grid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.orientation.w = 1.0
        
        # Flatten map data
        msg.data = self.occupancy_map.flatten().tolist()
        
        self.map_pub.publish(msg)

def main():
    rclpy.init()
    node = MappingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()