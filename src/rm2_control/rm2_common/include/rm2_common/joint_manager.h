//
// Created by Nesc on 2026/3/11.
//

#pragma once

#include <optional>
#include <unordered_map>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <string>
#include <vector>
#include <functional>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>

namespace rm2_common
{
class JointHandle
{
public:
  explicit JointHandle(const std::string& name) : name_(name) {}
  ~JointHandle() = default;

  void bind_interfaces(
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> pos,
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> vel,
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> effort,
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd)
  {
    pos_state = pos;
    vel_state = vel;
    effort_state = effort;
    cmd_interface = cmd;
  }

  void read()
  {
    if (pos_state) cached_pos = pos_state->get().get_value();
    if (vel_state) cached_vel = vel_state->get().get_value();
    if (effort_state) cached_effort = effort_state->get().get_value();
  }
  
  inline double getVelocity() const { return cached_vel; }
  inline double getPosition() const { return cached_pos; }
  inline double getEffort() const { return cached_effort; }
  inline double getCommand() const
  {
    if (cmd_interface)
    {
      return cmd_interface->get().get_value();
    }
    return 0.0;
  }
  inline std::string getName() const { return name_; }
  
  void setCommand(double command)
  {
    if (cmd_interface) 
    {
      cmd_interface->get().set_value(command);
    }
  }

private:
  std::string name_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> pos_state;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> vel_state;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> effort_state;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_interface;
  double cached_pos = 0.0;
  double cached_vel = 0.0;
  double cached_effort = 0.0;
};

class JointManager
{
public:
  JointManager() = default;
  explicit JointManager(std::vector<std::string> names)
  {
    for (const auto& name : names)
    {
      joints_.emplace_back(name);
    }
  }
  ~JointManager() = default;

  void init(std::vector<std::string> names)
  {
    joints_.clear();
    for (const auto& name : names)
    {
      joints_.emplace_back(name);
    }
  }
  
  void add_joint(std::string name) { joints_.emplace_back(name); }
  
  size_t size() const { return joints_.size(); }
  
  std::vector<std::string> get_names() const 
  {
    std::vector<std::string> names;
    names.reserve(joints_.size());
    for (const auto& joint : joints_) { names.push_back(joint.getName()); }
    return names;
  }
  
  void read_all() 
  {
    for (auto & joint : joints_) 
    {
      joint.read();
    }
  }
  
  void write_all(const std::vector<double>& commands) 
  {
    for (size_t i = 0; i < joints_.size(); ++i) {
      joints_[i].setCommand(commands[i]);
    }
  }

  void bind_all(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
                std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces)
  {
    std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_map;
    for (auto& it : state_interfaces) {
      state_map.emplace(it.get_name(), std::ref(it));
    }

    std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_map;
    for (auto& it : command_interfaces) {
      cmd_map.emplace(it.get_name(), std::ref(it));
    }

    for (auto & joint : joints_) 
    {
      const std::string name = joint.getName();
      
      std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> pos_opt = std::nullopt;
      auto pos_it = state_map.find(name + "/position");
      if (pos_it != state_map.end()) {
        pos_opt = std::make_optional(pos_it->second);
      }

      std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> vel_opt = std::nullopt;
      auto vel_it = state_map.find(name + "/velocity");
      if (vel_it != state_map.end()) {
        vel_opt = std::make_optional(vel_it->second);
      }

      std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> effort_opt = std::nullopt;
      auto effort_it = state_map.find(name + "/effort");
      if (effort_it != state_map.end()) {
        effort_opt = std::make_optional(effort_it->second);
      }

      std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_opt = std::nullopt;
      auto cmd_it = cmd_map.find(name + "/effort");
      if (cmd_it != cmd_map.end()) {
        cmd_opt = std::make_optional(cmd_it->second);
      }

      // todo yaml
      joint.bind_interfaces(pos_opt, vel_opt, effort_opt, cmd_opt);
    }
  }

  std::vector<std::string> get_command_interface_names() const
  {
    std::vector<std::string> names;
    for (const auto & joint : joints_) 
    {
      for (const auto & type : allowed_command_types_) 
      {
        names.push_back(joint.getName() + "/" + type);
      }
    }
    return names;
  }
  
  std::vector<std::string> get_state_interface_names() const 
  {
    std::vector<std::string> names;
    for (const auto & joint : joints_) 
    {
      for (const auto & type : allowed_state_types_) 
      {
        names.push_back(joint.getName() + "/" + type);
      }
    }
    return names;
  }
  
  auto begin() { return joints_.begin(); }
  auto end() { return joints_.end(); }
  auto begin() const { return joints_.begin(); }
  auto end() const { return joints_.end(); }

  JointHandle& operator[](size_t index) { return joints_.at(index); }
  const JointHandle& operator[](size_t index) const { return joints_.at(index); }

private:
  std::vector<JointHandle> joints_;
  std::vector<std::string> allowed_command_types_ = {hardware_interface::HW_IF_EFFORT};
  std::vector<std::string> allowed_state_types_ = {
     hardware_interface::HW_IF_POSITION,
     hardware_interface::HW_IF_VELOCITY,
     hardware_interface::HW_IF_EFFORT
   };
};
}