#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;

class CheckCommandCondition : public ConditionNode
{
public:
  CheckCommandCondition(const std::string& name, const NodeConfiguration& config)
    : ConditionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("check_command_condition_node");
    sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/user_command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_command_ = msg->data;
      });
  }

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("target_object") };
  }

  NodeStatus tick() override
  {
    rclcpp::spin_some(node_);

    std::string target_object;
    if (!getInput("target_object", target_object)) {
      throw RuntimeError("missing required input [target_object]");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_command_ == target_object) {
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string latest_command_;
  std::mutex mutex_;
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CheckCommandCondition>("CheckCommandCondition");
}
