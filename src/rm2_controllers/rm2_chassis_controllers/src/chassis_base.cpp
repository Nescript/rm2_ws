//
// Created by ocean_cing on 2026/2/9.
//

#include "rm2_chassis_controllers/chassis_base.h"

namespace rm2_chassis_controllers
{
controller_interface::CallbackReturn ChassisBase::on_init()
{
  try
  {
    publish_rate_ = get_node()->declare_parameter<double>("publish_rate", 100.0);
    publish_map_tf_ = get_node()->declare_parameter<bool>("publish_map_tf", false);
    publish_odom_tf_ = get_node()->declare_parameter<bool>("publish_odom_tf", false);
    slam_topic_ = get_node()->declare_parameter<std::string>("slam_topic", "");
    localization_topic_ = get_node()->declare_parameter<std::string>("localization_topic", "");

    velocity_coeff_ = get_node()->declare_parameter<double>("power.vel_coeff", 0.0);
    effort_coeff_ = get_node()->declare_parameter<double>("power.effort_coeff", 0.0);
    power_offset_ = get_node()->declare_parameter<double>("power.power_offset", 0.0);

    wheel_radius_ = get_node()->declare_parameter<double>("wheel_radius", 0.0);
    twist_angular_ = get_node()->declare_parameter<double>("twist_angular", M_PI / 6);
    max_odom_vel_ = get_node()->declare_parameter<double>("max_odom_vel", 10);
    timeout_ = get_node()->declare_parameter<double>("timeout", 0.1);

  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Chassis parameters initialization failed: %s", ex.what());
    return CallbackReturn::ERROR;
  }

  // robot_state_handle_ =

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisBase::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Should make sure the initialization of pid_follow
  pid_follow_.pid_ptr = std::make_shared<control_toolbox::PidROS>(get_node(), "pid_follow");
  pid_follow_.pid_ptr->initialize_from_ros_parameters();

  // How to assign Qos?
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1),
    [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg)
    {
      this->cmdVelCallback(msg);
    });

  cmd_chassis_sub_ = get_node()->create_subscription<rm2_msgs::msg::ChassisCmd>("/cmd_chassis", rclcpp::QoS(1),
    [this](const rm2_msgs::msg::ChassisCmd::ConstSharedPtr msg)
    {
      this->cmdChassisCallback(msg);
    });

  slam_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(slam_topic_, rclcpp::QoS(10),
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
      this->slamCallback(msg);
    });

  localization_sub_ = get_node()->create_subscription<geometry_msgs::msg::TransformStamped>(localization_topic_, rclcpp::QoS(10),
    [this](const geometry_msgs::msg::TransformStamped::ConstSharedPtr msg)
    {
      this->localizationCallback(msg);
    });

  ramp_x_ = std::make_unique<RampFilter<double>>(0, 0.001);
  ramp_y_ = std::make_unique<RampFilter<double>>(0, 0.001);
  ramp_w_ = std::make_unique<RampFilter<double>>(0, 0.001);

  if (publish_map_tf_)
  {
    global_map2robot_odom_.header.stamp = get_node()->get_clock()->now();
    global_map2robot_odom_.header.frame_id = global_map_frame_id_;
    global_map2robot_odom_.child_frame_id = robot_odom_frame_id_;
    global_map2robot_odom_.transform.rotation.w = 1.0;
    brcst4global_map2robot_odom_.init(get_node());
    brcst4global_map2robot_odom_.sendTransform(global_map2robot_odom_);

    global_map2camera_init_.header.stamp = get_node()->get_clock()->now();
    global_map2camera_init_.header.frame_id = global_map_frame_id_;
    global_map2camera_init_.child_frame_id = "camera_init";
    global_map2camera_init_.transform.rotation.w = 1.0;
    brcst4global_map2camera_init_.init(get_node());
    brcst4global_map2camera_init_.sendTransform(global_map2camera_init_);
  }

  if (publish_odom_tf_)
  {
    robot_odom2robot_base_.header.stamp = get_node()->get_clock()->now();
    robot_odom2robot_base_.header.frame_id = robot_odom_frame_id_;
    robot_odom2robot_base_.child_frame_id = robot_base_frame_id_;
    global_map2robot_odom_.transform.rotation.w = 1;
    brcst4robot_odom2robot_base_.init(get_node());
    brcst4robot_odom2robot_base_.sendTransform(robot_odom2robot_base_);

  }

  return CallbackReturn::SUCCESS;
}



}