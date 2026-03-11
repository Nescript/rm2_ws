//
// Created by Nesc on 2026/3/11.
//

#pragma once

#include <optional>
#include <functional>
#include <controller_interface/controller_interface.hpp>

namespace joint_manager 
{
class JointHandle
{
public:
  explicit JointHandle(const std::string& name) : name_(name) {}
  double getVelocity() const;
  double getPosition() const;
  double getEffort() const;
  double setCommand(double command);
private:
  std::string name_;
  std::optional<std::reference_wrapper<controller_interface::LoanedStateInterface>> pos_state;
  std::optional<std::reference_wrapper<controller_interface::LoanedStateInterface>> vel_state;
  std::optional<std::reference_wrapper<controller_interface::LoanedCommandInterface>> cmd_interface;
  double cached_pos = 0.0;
  double cached_vel = 0.0;
};
class JointManager
{

};
}