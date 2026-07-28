/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @file omni_base_input_odometry.cpp
/// @brief Omnidirectional cart odometry class

#include <hsrb_base_controllers/omni_base_input_odometry.hpp>

namespace hsrb_base_controllers {

/// Odometry input from external sources
InputOdometry::InputOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  odometry_subscriber_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, std::bind(&InputOdometry::OdometryCallback, this, std::placeholders::_1));
  InitOdometry();
}

void InputOdometry::InitOdometry() {
  nav_msgs::msg::Odometry initial_odom;
  initial_odom.header.stamp = rclcpp::Time(0);
  initial_odom.pose.pose.orientation.w = 1.0;
  odometry_buffer_.initRT(initial_odom);
}

/// Odometry callback
void InputOdometry::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odometry_buffer_.writeFromNonRT(*msg);
}

}  // namespace hsrb_base_controllers
