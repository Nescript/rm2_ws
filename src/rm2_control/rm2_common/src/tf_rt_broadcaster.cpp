//
// Created by ch on 2025/10/5.
//

#include "rm2_common/tf_rt_broadcaster.h"

#include <vector>
#include <tf2_msgs/msg/tf_message.hpp>

namespace rm2_common
{
void TfRtBroadcaster::init(rclcpp::Node::SharedPtr root_node)
{
  auto pub = root_node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>(pub));
}

void TfRtBroadcaster::init(rclcpp_lifecycle::LifecycleNode::SharedPtr root_node)
{
  auto pub = root_node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>(pub));
}

void TfRtBroadcaster::sendTransform(const geometry_msgs::msg::TransformStamped& transform)
{
  std::vector<geometry_msgs::msg::TransformStamped> v1;
  v1.push_back(transform);
  sendTransform(v1);
}

void TfRtBroadcaster::sendTransform(const std::vector<geometry_msgs::msg::TransformStamped>& transforms)
{
  tf2_msgs::msg::TFMessage message;
  for (const auto& transform : transforms)
  {
    message.transforms.push_back(transform);
  }
  if (realtime_pub_->trylock())
  {
    realtime_pub_->msg_ = message;
    realtime_pub_->unlockAndPublish();
  }
}

void StaticTfRtBroadcaster::init(rclcpp::Node::SharedPtr root_node)
{
  auto pub = root_node->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", 100);
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>(pub));
}

void StaticTfRtBroadcaster::init(rclcpp_lifecycle::LifecycleNode::SharedPtr root_node)
{
  auto pub = root_node->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", 100);
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>(pub));
}

void StaticTfRtBroadcaster::sendTransform(const geometry_msgs::msg::TransformStamped& transform)
{
  std::vector<geometry_msgs::msg::TransformStamped> v1;
  v1.push_back(transform);
  sendTransform(v1);
}

void StaticTfRtBroadcaster::sendTransform(const std::vector<geometry_msgs::msg::TransformStamped>& transforms)
{
  for (const auto& transform : transforms)
  {
    bool match_found = false;
    for (auto& it_msg : net_message_.transforms)
    {
      if (transform.child_frame_id == it_msg.child_frame_id)
      {
        it_msg = transform;
        match_found = true;
        break;
      }
    }
    if (!match_found)
      net_message_.transforms.push_back(transform);
  }
  if (realtime_pub_->trylock())
  {
    realtime_pub_->msg_ = net_message_;
    realtime_pub_->unlockAndPublish();
  }
}

}  // namespace rm2_common
