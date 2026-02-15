//
// Created by ocean_cing on 2026/2/12.
//

#pragma once

#include "rm2_chassis_controllers/chassis_base.h"

namespace rm2_chassis_controllers
{
class OmniController : public ChassisBase
{
public:
  OmniController() = default;
  hardware_interface::CallbackReturn on_init() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

private:
  void moveJoint(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) override;
  geometry_msgs::msg::Twist odometry() override;

  double K = 0.;  // Feedforward gain
  Eigen::MatrixXd chassis2joints_;
};

} // namespace rm2_chassis_controllers