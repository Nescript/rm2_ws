//
// Created by ocean_cing on 2026/3/4.
//

#pragma once

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rm2_common
{
class RobotStateNode : public rclcpp::Node
{
public:
  RobotStateNode() : Node("robot_state_node")
  {
    buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  }

  std::shared_ptr<tf2_ros::Buffer> getBuffer()
  {
    return buffer_;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

class RobotStateManager
{
public:
  static RobotStateManager& instance()
  {
    static RobotStateManager inst;
    return inst;
  }

  std::shared_ptr<tf2_ros::Buffer> getBuffer()
  {
    return robot_state_node_->getBuffer();
  }

private:
  RobotStateManager()
  {
    robot_state_node_ = std::make_shared<RobotStateNode>();

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    executor_->add_node(robot_state_node_);

    spin_thread_ =
      std::thread([this]()
      {
        executor_->spin();
      });
  }

  ~RobotStateManager()
  {
    executor_->cancel();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
  }

  std::shared_ptr<RobotStateNode> robot_state_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

} // namespace rm2_common