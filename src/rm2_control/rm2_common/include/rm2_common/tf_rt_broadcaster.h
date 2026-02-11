//
// Created by ch on 2025/10/5.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_publisher.h>

namespace rm2_common
{
class TfRtBroadcaster
{
public:
  TfRtBroadcaster() = default;
  virtual void init(rclcpp::Node::SharedPtr root_node);
  virtual void init(rclcpp_lifecycle::LifecycleNode::SharedPtr root_node);
  virtual void sendTransform(const geometry_msgs::msg::TransformStamped& transform);
  virtual void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped>& transforms);

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_pub_{};
};

class StaticTfRtBroadcaster : public TfRtBroadcaster
{
public:
  void init(rclcpp::Node::SharedPtr root_node) override;
  void init(rclcpp_lifecycle::LifecycleNode::SharedPtr root_node) override;
  void sendTransform(const geometry_msgs::msg::TransformStamped& transform) override;
  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped>& transforms) override;

private:
  tf2_msgs::msg::TFMessage net_message_{};
};

}  // namespace rm2_common
