#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from diff_drive_robot.srv import GetObjectPose
from nav2_msgs.action import NavigateToPose
from rclpy.parameter import Parameter
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import threading
import math
import random
import time

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
        
        self.memory_client = self.create_client(GetObjectPose, '/get_object_pose')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TF2 Setup for finding robot's position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriber for topic-based commands (avoids terminal spam issues)
        self.create_subscription(String, '/ui_command', self.command_callback, 10)
        
        self.pending_object = None
        self.state = 'IDLE' # IDLE, WANDERING, SPINNING, GOING_TO_TARGET
        
        # 2Hz active polling timer
        self.poll_timer = self.create_timer(0.5, self.poll_memory)
        self.spin_timer = None
        self.spin_ticks = 0
        
        self.get_logger().info("UI Node started. Type an object name (e.g., 'person' or 'chair') and press Enter.")
        
        self.input_thread = threading.Thread(target=self.read_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def command_callback(self, msg):
        cmd = msg.data.strip()
        if cmd:
            self.pending_object = cmd.lower()
            self.get_logger().info(f"Topic command received. Seeking '{self.pending_object}'...")
            self.state = 'WANDERING'
            self.wander_to_random_point()

    def read_input(self):
        while rclpy.ok():
            try:
                cmd = input("Command> ")
                if cmd.strip():
                    self.pending_object = cmd.strip().lower()
                    self.get_logger().info(f"Terminal command received. Seeking '{self.pending_object}'...")
                    self.state = 'WANDERING'
                    self.wander_to_random_point()
            except EOFError:
                break

    def poll_memory(self):
        """Actively checks if the pending object has appeared in memory."""
        if self.state == 'IDLE' or self.pending_object is None:
            return
            
        if not self.memory_client.wait_for_service(timeout_sec=1.0):
            return
            
        req = GetObjectPose.Request()
        req.object_name = self.pending_object
        
        future = self.memory_client.call_async(req)
        future.add_done_callback(self.poll_response_callback)

    def poll_response_callback(self, future):
        try:
            response = future.result()
            if response.success and self.state in ['WANDERING', 'SPINNING']:
                self.get_logger().info(f"*** TARGET '{self.pending_object}' SPOTTED! ***")
                self.get_logger().info("Canceling search and navigating directly to target...")
                
                self.state = 'GOING_TO_TARGET'
                self.send_nav_goal(response.pose)
                self.pending_object = None
        except Exception as e:
            pass

    def wander_to_random_point(self):
        if self.state != 'WANDERING':
            return
            
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 server unreachable.")
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Pick random point inside a valid range
        goal_msg.pose.pose.position.x = random.uniform(-3.5, 3.5)
        goal_msg.pose.pose.position.y = random.uniform(-3.5, 3.5)
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Wandering to ({goal_msg.pose.pose.position.x:.2f}, {goal_msg.pose.pose.position.y:.2f})")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.wander_goal_accepted)

    def wander_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Wander goal REJECTED by Nav2. Retrying in 2 seconds...")
            time.sleep(2.0)
            if self.state == 'WANDERING':
                self.wander_to_random_point()
            return
            
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.wander_goal_done)
        
    def wander_goal_done(self, future):
        if self.state == 'WANDERING':
            self.get_logger().info("Reached wander point (or aborted). Spinning to scan room...")
            self.state = 'SPINNING'
            self.spin_search()

    def spin_search(self):
        """Perform a fast manual spin using cmd_vel."""
        self.get_logger().info("Executing high-speed search spin...")
        self.spin_ticks = 0
        if self.spin_timer is not None:
            self.spin_timer.cancel()
        self.spin_timer = self.create_timer(0.1, self.spin_timer_callback)
        
    def spin_timer_callback(self):
        if self.state != 'SPINNING':
            if self.spin_timer:
                self.spin_timer.cancel()
                self.spin_timer = None
            return
            
        self.spin_ticks += 1
        
        # Spin for exactly 3 seconds at 2.1 rad/s (approx 360 degrees)
        if self.spin_ticks <= 30:
            twist = Twist()
            twist.angular.z = 2.1
            self.cmd_vel_pub.publish(twist)
        else:
            if self.spin_timer:
                self.spin_timer.cancel()
                self.spin_timer = None
            self.get_logger().info("Spin complete. Target not in sight. Exploring new area...")
            self.state = 'WANDERING'
            self.wander_to_random_point()

    def send_nav_goal(self, target_pose):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return
            
        # Get robot position from TF
        robot_x = 0.0
        robot_y = 0.0
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except Exception as e:
            self.get_logger().warning(f"TF lookup failed, defaulting to origin vector: {e}")
            
        obj_x = target_pose.pose.position.x
        obj_y = target_pose.pose.position.y
        
        # Vector FROM object TO robot
        dx = robot_x - obj_x
        dy = robot_y - obj_y
        dist = math.sqrt(dx*dx + dy*dy)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        
        offset = 0.5 # 0.5 meters away
        
        if dist > offset:
            # Apply offset along the vector towards the robot
            goal_msg.pose.pose.position.x = obj_x + (dx/dist) * offset
            goal_msg.pose.pose.position.y = obj_y + (dy/dist) * offset
            
            # Point robot at the object
            yaw = math.atan2(-dy, -dx)
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.target_goal_accepted)
        
    def target_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Target goal REJECTED. Robot might be stuck.")
            self.state = 'IDLE'
            return
        self.get_logger().info("Target goal ACCEPTED. Navigating to object...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.target_goal_done)

    def target_goal_done(self, future):
        self.get_logger().info("Mission Complete! Target object reached successfully.")
        self.state = 'IDLE'

def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
