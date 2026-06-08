#include <string>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "diff_drive_robot/srv/get_object_pose.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace BT;

class GetObjectPoseAction : public SyncActionNode
{
public:
  GetObjectPoseAction(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("get_object_pose_action_node");
    client_ = node_->create_client<diff_drive_robot::srv::GetObjectPose>("/get_object_pose");
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  static PortsList providedPorts()
  {
    return {
      InputPort<std::string>("object_name"),
      OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose")
    };
  }

  NodeStatus tick() override
  {
    std::string object_name;
    if (!getInput("object_name", object_name)) {
      throw RuntimeError("missing required input [object_name]");
    }

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(node_->get_logger(), "Service /get_object_pose not available");
      return NodeStatus::FAILURE;
    }

    auto request = std::make_shared<diff_drive_robot::srv::GetObjectPose::Request>();
    request->object_name = object_name;

    auto future = client_->async_send_request(request);
    
    // Spin until future is complete (since this is SyncActionNode, it might block, but it's fast)
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(node_->get_logger(), "Service call failed");
      return NodeStatus::FAILURE;
    }

    auto result = future.get();
    if (!result->success) {
      return NodeStatus::FAILURE;
    }

    // Get the object's pose
    double obj_x = result->pose.pose.position.x;
    double obj_y = result->pose.pose.position.y;

    // Get the robot's current pose from TF
    double robot_x = 0.0;
    double robot_y = 0.0;
    try {
      auto transformStamped = tf_buffer_->lookupTransform(
        "map", "base_link",
        tf2::TimePointZero, tf2::durationFromSec(1.0));
      robot_x = transformStamped.transform.translation.x;
      robot_y = transformStamped.transform.translation.y;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform map to base_link: %s", ex.what());
      // Default to origin if TF fails
    }

    // Calculate the vector from object to robot
    double dx = robot_x - obj_x;
    double dy = robot_y - obj_y;
    double dist = std::sqrt(dx*dx + dy*dy);

    double offset = 0.5; // 0.5 meters
    geometry_msgs::msg::PoseStamped goal_pose = result->pose;
    
    if (dist > offset) {
      // Normalize and apply offset
      goal_pose.pose.position.x = obj_x + (dx / dist) * offset;
      goal_pose.pose.position.y = obj_y + (dy / dist) * offset;
      
      // Calculate orientation to face the object
      double yaw = std::atan2(-dy, -dx);
      goal_pose.pose.orientation.z = std::sin(yaw / 2.0);
      goal_pose.pose.orientation.w = std::cos(yaw / 2.0);
    }

    setOutput("goal_pose", goal_pose);
    return NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<diff_drive_robot::srv::GetObjectPose>::SharedPtr client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GetObjectPoseAction>("GetObjectPoseAction");
}
