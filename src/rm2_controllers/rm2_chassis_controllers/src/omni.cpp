//
// Created by ocean_cing on 2026/2/12.
//

#include "rm2_chassis_controllers/omni.h"
#include <pluginlib/class_list_macros.hpp>

namespace rm2_chassis_controllers
{
hardware_interface::CallbackReturn OmniController::on_init()
{
  if (ChassisBase::on_init() != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  try
  {
    K = get_node()->declare_parameter<double>("K", 1.0);
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Chassis parameters initialization failed: %s", ex.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  if (ChassisBase::on_configure(previous_state) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  chassis2joints_.resize(wheel_joints_.joint_names.size(), 3);
  for (size_t i = 0; i < wheel_joints_.joint_names.size(); ++i)
  {
    std::vector<double> pose;
    double roller_angle;
    double radius;
    try
    {
      pose = get_node()->declare_parameter<std::vector<double>>(wheel_joints_.joint_names[i] + ".pose", std::vector<double>{});
      if (pose.size() != 3)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Build matrix 'chassis2joints_' failed: pose's size is %lu", pose.size());
        return CallbackReturn::ERROR;
      }

      roller_angle = get_node()->declare_parameter<double>(wheel_joints_.joint_names[i] + ".roller_angle", 0.);
      radius = get_node()->declare_parameter<double>(wheel_joints_.joint_names[i] + ".radius", 0.07625);
    }
    catch (std::exception& ex)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Build matrix 'chassis2joints_' failed: %s", ex.what());
      return CallbackReturn::ERROR;
    }

    // Ref: Modern Robotics, Chapter 13.2: Omnidirectional Wheeled Mobile Robots
    Eigen::MatrixXd direction(1, 2), in_wheel(2, 2), in_chassis(2, 3);
    double beta = pose[2];
    direction << 1, tan(roller_angle);
    in_wheel << cos(beta), sin(beta), -sin(beta), cos(beta);
    in_chassis << -pose[1], 1., 0., pose[0], 0., 1.;
    Eigen::MatrixXd chassis2joint = 1. / radius * direction * in_wheel * in_chassis;
    chassis2joints_.block<1, 3>(i, 0) = chassis2joint;
  }

  return CallbackReturn::SUCCESS;
}

void OmniController::moveJoint(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  Eigen::Vector3d vel_chassis;
  if (state_ == RAW)
  {
    geometry_msgs::msg::Twist vel_base = odometry();
    double Kp = K * abs(vel_cmd_.z);
    vel_cmd_.x = vel_cmd_.x + Kp * (vel_cmd_.x - vel_base.linear.x);
    vel_cmd_.y = vel_cmd_.y + Kp * (vel_cmd_.y - vel_base.linear.y);
  }
  vel_chassis << vel_cmd_.z, vel_cmd_.x, vel_cmd_.y;
  Eigen::VectorXd vel_joints = chassis2joints_ * vel_chassis;

  for (size_t i = 0; i < wheel_joints_.joint_names.size(); ++i)
  {
    (void)command_interfaces_[wheel_joints_.cmd_index[i]].set_value(wheel_joints_.pids[i]->compute_command(
      vel_joints[i] - state_interfaces_[wheel_joints_.vel_index[i]].get_optional<double>().value(), period));
  }
}

geometry_msgs::msg::Twist OmniController::odometry()
{
  Eigen::VectorXd vel_joints(wheel_joints_.joint_names.size());
  for (size_t i = 0; i < wheel_joints_.joint_names.size(); i++)
  {
    vel_joints[i] = state_interfaces_[wheel_joints_.vel_index[i]].get_optional<double>().value();
  }
  Eigen::Vector3d vel_chassis = (chassis2joints_.transpose() * chassis2joints_).inverse() * chassis2joints_.transpose() * vel_joints;
  geometry_msgs::msg::Twist twist;
  twist.angular.z = vel_chassis(0);
  twist.linear.x = vel_chassis(1);
  twist.linear.y = vel_chassis(2);
  return twist;
}
}

PLUGINLIB_EXPORT_CLASS(rm2_chassis_controllers::OmniController, controller_interface::ControllerInterface)
