#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Publishers
        self.spear_pub = self.create_publisher(Point, 'detected_spear', 10)
        self.obstacle_pub = self.create_publisher(Point, 'detected_obstacle', 10)
        self.detection_status_pub = self.create_publisher(String, 'detection_status', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, 'camera/image', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, 'lidar/points', self.lidar_callback, 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        self.get_logger().info('Object Detection Node Started')
        
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect spears (brown cylindrical objects)
            spear_position = self.detect_spears(cv_image)
            if spear_position:
                self.publish_spear_detection(spear_position)
                
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
            
    def detect_spears(self, image):
        # Simple color-based detection for brown spears
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Brown color range
        lower_brown = (10, 50, 20)
        upper_brown = (20, 255, 200)
        
        mask = cv2.inRange(hsv, lower_brown, upper_brown)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour (closest spear)
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
        
        return None
        
    def publish_spear_detection(self, position):
        # Convert pixel coordinates to world coordinates (simplified)
        msg = Point()
        msg.x = float(position[0] * 0.01)  # Convert to meters
        msg.y = float(position[1] * 0.01)
        msg.z = 0.0
        self.spear_pub.publish(msg)
        
        status = String()
        status.data = "spear_detected"
        self.detection_status_pub.publish(status)
        
    def lidar_callback(self, msg):
        # Process LiDAR data for obstacle detection
        pass

def main():
    rclpy.init()
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()