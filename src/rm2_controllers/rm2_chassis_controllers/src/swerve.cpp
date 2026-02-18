//
// Created by ocean_cing on 2026/2/12.
//

#include "rm2_chassis_controllers/swerve.h"
#include <angles/angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace rm2_chassis_controllers
{
hardware_interface::CallbackReturn SwerveController::on_init()
{
  if (ChassisBase::on_init() != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  try
  {
    modules_.wheel_joints.joint_names = get_node()->declare_parameter<std::vector<std::string>>("joint_names.wheels", std::vector<std::string>{});
    modules_.pivot_joints.joint_names = get_node()->declare_parameter<std::vector<std::string>>("joint_names.pivots", std::vector<std::string>{});
    power_limit_joints_ = &modules_.wheel_joints;
    if (modules_.wheel_joints.joint_names.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "No wheel joint given (namespace: %s)", get_node()->get_name());
      return CallbackReturn::ERROR;
    }
    if (modules_.pivot_joints.joint_names.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "No pivot joint given (namespace: %s)", get_node()->get_name());
      return CallbackReturn::ERROR;
    }
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Chassis parameters initialization failed: %s", ex.what());
    return CallbackReturn::ERROR;
  }
  if (modules_.wheel_joints.joint_names.size() == modules_.pivot_joints.joint_names.size())
  {
    modules_.size = modules_.wheel_joints.joint_names.size();
    for (size_t i = 0; i < modules_.size; ++i)
    {
      bool is_match = modules_.wheel_joints.joint_names[i].compare(0, 10, modules_.pivot_joints.joint_names[i], 0, 10);
      if (!is_match)
      {
        return CallbackReturn::ERROR;
      }
    }
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Position of wheel and pivot not match (namespace: %s)", get_node()->get_name());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  if (ChassisBase::on_configure(previous_state) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  buildJointsPids(modules_.wheel_joints);
  buildJointsPids(modules_.pivot_joints);

  //
  for (size_t i = 0; i < modules_.size; ++i)
  {
    auto pos = get_node()->declare_parameter<std::vector<double>>(modules_.pivot_joints.joint_names[i] + ".position", std::vector<double>{});
    if (pos.size() != 2)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Initialize 'modules_' failed: position's size is %lu", pos.size());
      return CallbackReturn::ERROR;
    }
    Vec2<double> position = {pos[0], pos[1]};
    auto radius = get_node()->declare_parameter<double>(modules_.wheel_joints.joint_names[i] + ".radius", 0.049);
    auto pivot_offset = get_node()->declare_parameter<double>(modules_.pivot_joints.joint_names[i] + ".offset", 0.);
    auto pivot_buffer_threshold = get_node()->declare_parameter<double>(modules_.pivot_joints.joint_names[i] + ".buffer_threshold", 10.0);
    auto pivot_effort_threshold = get_node()->declare_parameter<double>(modules_.pivot_joints.joint_names[i] + ".effort_threshold", 0.9);
    auto pivot_position_error_threshold = get_node()->declare_parameter<double>(modules_.pivot_joints.joint_names[i] + ".position_error_threshold", 0.1);
    auto max_reduce_cnt = get_node()->declare_parameter<double>(modules_.pivot_joints.joint_names[i] + ".max_reduce_cnt", 100.0);

    modules_.position.push_back(position);
    modules_.wheel_radius.push_back(radius);
    modules_.pivot_offset.push_back(pivot_offset);
    modules_.pivot_buffer_threshold.push_back(pivot_buffer_threshold);
    modules_.pivot_effort_threshold.push_back(pivot_effort_threshold);
    modules_.pivot_position_error_threshold.push_back(pivot_position_error_threshold);
    modules_.pivot_max_reduce_cnt.push_back(max_reduce_cnt);
  }

  power_manager_sub_ = get_node()->create_subscription<rm2_msgs::msg::PowerHeatData>("/rm2_referee/power_manager", rclcpp::QoS(10),
    [this](const rm2_msgs::msg::PowerHeatData::SharedPtr msg)
    {
      this->powerManagerCallback(msg);
    });

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  if (ChassisBase::on_activate(previous_state) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  auto command_interface_index_map = buildInterfaceIndexMap(command_interfaces_);
  auto state_interface_index_map = buildInterfaceIndexMap(state_interfaces_);
  modules_.wheel_joints.reset();
  modules_.pivot_joints.reset();
  modules_.wheel_joints.reserve(modules_.wheel_joints.joint_names.size());
  modules_.pivot_joints.reserve(modules_.pivot_joints.joint_names.size());
  buildJointsIndex(modules_.wheel_joints, command_interface_index_map, state_interface_index_map);
  buildJointsIndex(modules_.pivot_joints, command_interface_index_map, state_interface_index_map);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  if (ChassisBase::on_deactivate(previous_state) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  modules_.wheel_joints.reset();
  modules_.pivot_joints.reset();

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const std::string& joint_name : modules_.wheel_joints.joint_names)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  for (const std::string& joint_name : modules_.pivot_joints.joint_names)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const std::string& joint_name : modules_.wheel_joints.joint_names)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  for (const std::string& joint_name : modules_.pivot_joints.joint_names)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

void SwerveController::moveJoint(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  Vec2<double> vel_center(vel_cmd_.x, vel_cmd_.y);
  for (size_t i = 0; i < modules_.size; ++i)
  {
    Vec2<double> vel = vel_center + vel_cmd_.z * Vec2<double>(-modules_.position[i].y(), modules_.position[i].x());
    double vel_angle = std::atan2(vel.y(), vel.x()) + modules_.pivot_offset[i];
    // Direction flipping and Stray module mitigation
    auto pivot_current_pos = state_interfaces_[modules_.pivot_joints.pos_index[i]].get_optional<double>().value();
    double a = angles::shortest_angular_distance(pivot_current_pos, vel_angle);
    double b = angles::shortest_angular_distance(pivot_current_pos, vel_angle + M_PI);
    double target_pos = std::abs(a) < std::abs(b) ? vel_angle : vel_angle + M_PI;
    double pos_error = angles::shortest_angular_distance(pivot_current_pos, target_pos);
    if (chassis_power_buffer_ > modules_.pivot_buffer_threshold[i] &&
      !isPivotBlock(state_interfaces_[modules_.pivot_joints.eff_index[i]].get_optional<double>().value(), pos_error, modules_, i))
    {
      pivot_block_cnt_ = 0;
      (void)command_interfaces_[modules_.pivot_joints.cmd_index[i]].set_value(modules_.pivot_joints.pids[i]->compute_command(
        target_pos - pivot_current_pos, period));
    }
    else
    {
      reduceTargetPosition(target_pos, pos_error, modules_, i);
      (void)command_interfaces_[modules_.pivot_joints.cmd_index[i]].set_value(modules_.pivot_joints.pids[i]->compute_command(
        target_pos - pivot_current_pos, period));
    }

    (void)command_interfaces_[modules_.wheel_joints.cmd_index[i]].set_value(modules_.wheel_joints.pids[i]->compute_command(
      vel.norm() / modules_.wheel_radius[i] * std::cos(a) -
      state_interfaces_[modules_.wheel_joints.vel_index[i]].get_optional<double>().value(), period));
  }
}

bool SwerveController::isPivotBlock(const double& cur_effort, const double& position_error, const ModuleGroup& module_group, const size_t& index)
{
  bool is_pivot_block = abs(cur_effort) > module_group.pivot_effort_threshold[index] &&
    abs(position_error) > module_group.pivot_position_error_threshold[index];
  return is_pivot_block;
}

void SwerveController::reduceTargetPosition(double& target_pos, const double& position_error, const ModuleGroup& module_group, const size_t& index)
{
  pivot_block_cnt_++;
  if (pivot_block_cnt_ > module_group.pivot_max_reduce_cnt[index])
  {
    pivot_block_cnt_ = module_group.pivot_max_reduce_cnt[index];
  }
  double reduce_step_size = position_error / module_group.pivot_max_reduce_cnt[index];

  if (abs(position_error) > abs(pivot_block_cnt_ * reduce_step_size))
  {
    target_pos -= pivot_block_cnt_ * reduce_step_size;
  }
  else
  {
    target_pos = state_interfaces_[module_group.pivot_joints.pos_index[index]].get_optional<double>().value();
  }
}

geometry_msgs::msg::Twist SwerveController::odometry()
{
  geometry_msgs::msg::Twist vel_data{};
  geometry_msgs::msg::Twist vel_modules{};
  for (size_t i = 0; i < modules_.size; ++i)
  {
    geometry_msgs::msg::Twist vel;
    auto wheel_current_vel = state_interfaces_[modules_.wheel_joints.vel_index[i]].get_optional<double>().value();
    auto pivot_current_pos = state_interfaces_[modules_.pivot_joints.pos_index[i]].get_optional<double>().value();
    vel.linear.x = wheel_current_vel * modules_.wheel_radius[i] * std::cos(pivot_current_pos);
    vel.linear.y = wheel_current_vel * modules_.wheel_radius[i] * std::sin(pivot_current_pos);
    vel.angular.z = wheel_current_vel * modules_.wheel_radius[i] *
      std::cos(pivot_current_pos - std::atan2(modules_.position[i].x(), -modules_.position[i].y()));
    vel_modules.linear.x += vel.linear.x;
    vel_modules.linear.y += vel.linear.y;
    vel_modules.angular.z += vel.angular.z;
  }
  vel_data.linear.x = vel_modules.linear.x / modules_.size;
  vel_data.linear.y = vel_modules.linear.y / modules_.size;
  vel_data.angular.z = vel_modules.angular.z / modules_.size /
    std::sqrt(std::pow(modules_.position.begin()->x(), 2) + std::pow(modules_.position.begin()->y(), 2));
  return vel_data;
}

void SwerveController::powerManagerCallback(const rm2_msgs::msg::PowerHeatData::ConstSharedPtr msg)
{
  chassis_power_buffer_ = msg->chassis_power_buffer;
}
} // namespace rm2_chassis_controller

PLUGINLIB_EXPORT_CLASS(rm2_chassis_controllers::SwerveController, controller_interface::ControllerInterface)