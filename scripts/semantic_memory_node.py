#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diff_drive_robot.srv import GetObjectPose
from geometry_msgs.msg import PoseStamped
import json

class SemanticMemoryNode(Node):
    def __init__(self):
        super().__init__('semantic_memory_node')
        
        self.memory = {}
        
        # Subscribe to detections
        self.create_subscription(String, '/detected_objects', self.detection_callback, 10)
        
        # Create service
        self.create_service(GetObjectPose, '/get_object_pose', self.get_pose_callback)
        
        self.get_logger().info("SemanticMemoryNode initialized")

    def detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
            name = data['name']
            x = data['x']
            y = data['y']
            
            # Simple averaging/updating or just overwriting. We'll just overwrite with the latest detection.
            self.memory[name] = (x, y)
            self.get_logger().info(f"Remembered {name} at ({x:.2f}, {y:.2f})")
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse detection: {e}")

    def get_pose_callback(self, request, response):
        name = request.object_name.lower()
        if name in self.memory:
            x, y = self.memory[name]
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 # identity quaternion
            
            response.success = True
            response.pose = pose
            self.get_logger().info(f"Provided pose for {name}")
        else:
            response.success = False
            self.get_logger().debug(f"Object {name} not found in memory")
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMemoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
