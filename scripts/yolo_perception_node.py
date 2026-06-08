#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import json
import math

class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.create_subscription(Image, '/camera/image_raw', self.color_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        
        # Publisher
        self.obj_pub = self.create_publisher(String, '/detected_objects', 10)
        
        # Timer for inference
        self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("YoloPerceptionNode initialized")

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def info_callback(self, msg):
        self.camera_info = msg

    def timer_callback(self):
        if self.color_image is None or self.depth_image is None:
            return

        # Run YOLO inference
        results = self.model(self.color_image, verbose=False)
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls_id]
                
                if conf < 0.10:
                    continue
                
                self.get_logger().info(f"YOLO raw detection: {class_name} ({conf:.2f})")
                    
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Center pixel
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                
                # Ensure pixel is within bounds
                if cy >= self.depth_image.shape[0] or cx >= self.depth_image.shape[1]:
                    continue
                    
                depth = self.depth_image[cy, cx]
                
                # Ignore invalid depth
                if np.isnan(depth) or np.isinf(depth) or depth <= 0.0:
                    continue
                
                # Deproject to 3D point in camera frame
                # If camera_info is not available, approximate using 1.047 fov
                if self.camera_info is not None:
                    fx = self.camera_info.k[0]
                    fy = self.camera_info.k[4]
                    px = self.camera_info.k[2]
                    py = self.camera_info.k[5]
                else:
                    fov = 1.047
                    width = 640
                    height = 480
                    fx = width / (2.0 * math.tan(fov / 2.0))
                    fy = fx
                    px = width / 2.0
                    py = height / 2.0
                
                X = (cx - px) * depth / fx
                Y = (cy - py) * depth / fy
                Z = float(depth)
                
                # Create PointStamped
                point_camera = PointStamped()
                point_camera.header.frame_id = 'camera_link_optical'
                point_camera.header.stamp = self.get_clock().now().to_msg()
                point_camera.point.x = X
                point_camera.point.y = Y
                point_camera.point.z = Z
                
                try:
                    # Look up transform from camera_link_optical to map
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        'camera_link_optical',
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    point_map = do_transform_point(point_camera, transform)
                    
                    # Publish the detected object
                    msg = String()
                    data = {
                        'name': class_name,
                        'x': point_map.point.x,
                        'y': point_map.point.y
                    }
                    msg.data = json.dumps(data)
                    self.obj_pub.publish(msg)
                    
                    self.get_logger().info(f"Detected {class_name} at map coords: ({data['x']:.2f}, {data['y']:.2f})")
                    
                except Exception as e:
                    self.get_logger().warning(f"Could not transform point: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
