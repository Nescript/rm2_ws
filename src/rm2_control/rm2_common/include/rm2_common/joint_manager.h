//
// Created by Nesc on 2026/3/11.
//

#pragma once

#include <optional>
#include <functional>
#include <controller_interface/controller_interface.hpp>
#include <unordered_map>

namespace joint_manager 
{
class JointHandle
{
public:
  explicit JointHandle(const std::string& name) : name_(name) {}
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
    if (pos_state) {
      cached_pos = pos_state->get().get_value();
      cached_vel = vel_state->get().get_value();
      cached_effort = effort_state->get().get_value();
    }
  }
  inline double getVelocity() const {return cached_vel;}
  inline double getPosition() const {return cached_pos;}
  inline double getEffort() const {return cached_effort;}
  inline std::string getName() const {return name_;}
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
  explicit JointManager(std::vector<std::string> names)
  {
    for (const auto& name : names)
    {
      joints_.emplace_back(name);
    }
  };
  void read_all() {
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

  // todo: avoid copy
  // todo：make it more elegrant
  void bind_all(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
                std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces)
  {
    // 1. 建立索引表 (提升查找速度到 O(1))
    std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_map;
    for (auto& itf : state_interfaces) {
      state_map.emplace(itf.get_name(), std::ref(itf));
    }

    std::unordered_map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_map;
    for (auto& itf : command_interfaces) {
      cmd_map.emplace(itf.get_name(), std::ref(itf));
    }

    // 2. 遍历关节进行绑定
    for (auto & joint : joints_) 
    {
      const std::string name = joint.getName();
      
      // 获取状态接口 (使用 std::optional 处理不存在的情况)
      auto get_state = [&](const std::string& type) -> std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> {
        auto it = state_map.find(name + "/" + type);
        return (it != state_map.end()) ? std::make_optional(it->second) : std::nullopt;
      };

      auto get_cmd = [&](const std::string& type) -> std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> {
        auto it = cmd_map.find(name + "/" + type);
        return (it != cmd_map.end()) ? std::make_optional(it->second) : std::nullopt;
      };

      // 3. 执行真正的绑定
      joint.bind_interfaces(
        get_state(hardware_interface::HW_IF_POSITION),
        get_state(hardware_interface::HW_IF_VELOCITY),
        get_state(hardware_interface::HW_IF_EFFORT),
        get_cmd(hardware_interface::HW_IF_EFFORT) // 根据 config 可能有不同类型
      );
    }
  }
  std::vector<std::string> get_command_interface_names()
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
  JointHandle& operator[](size_t index) 
  {
    return joints_.at(index); 
  }
private:
  std::vector<JointHandle> joints_;
  std::vector<std::string> allowed_command_types_ = {"effort"};
  std::vector<std::string> allowed_state_types_ = {"position", "velocity", "effort"};
  // Maybe we should use yaml to set allowed command types

};
}