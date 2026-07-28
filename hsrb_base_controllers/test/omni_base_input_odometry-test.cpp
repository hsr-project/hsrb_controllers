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
/// @file omni_base_input_odometry-test.cpp
/// @brief Test for the external input odometry class

#include <gtest/gtest.h>

#include <hsrb_base_controllers/omni_base_input_odometry.hpp>

#include "utils.hpp"

namespace hsrb_base_controllers {

/// Initialize the odometry
TEST(OmniBaseInputOdometryTest, InitOdometry) {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  auto odom = InputOdometry(node);
  node->activate();

  odom.InitOdometry();

  auto output = odom.GetOdometry();
  EXPECT_EQ(output.header.stamp.sec, 0);
  EXPECT_EQ(output.header.stamp.nanosec, 0);
  EXPECT_EQ(output.pose.pose.position.x, 0.0);
  EXPECT_EQ(output.pose.pose.position.y, 0.0);
  EXPECT_EQ(output.pose.pose.position.z, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.x, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.y, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.z, 0.0);
  EXPECT_EQ(output.pose.pose.orientation.w, 1.0);
}

/// Retrieve the current odometry
TEST(OmniBaseInputOdometryTest, GetOdometry) {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  auto odom = InputOdometry(node);
  auto publisher = node->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SystemDefaultsQoS());
  node->activate();

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = node->now();
  msg.pose.pose.position.x = 1.0;
  msg.pose.pose.position.y = 2.0;
  msg.pose.pose.position.z = 3.0;
  msg.pose.pose.orientation.x = 4.0;
  msg.pose.pose.orientation.y = 5.0;
  msg.pose.pose.orientation.z = 6.0;
  msg.pose.pose.orientation.w = 7.0;

  publisher->publish(msg);
  auto timeout = TimeoutDetection(node->get_clock());
  while (rclcpp::Time(odom.GetOdometry().header.stamp).nanoseconds() == 0) {
    timeout.Run();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  auto output = odom.GetOdometry();
  EXPECT_EQ(output.pose.pose.position.x, 1.0);
  EXPECT_EQ(output.pose.pose.position.y, 2.0);
  EXPECT_EQ(output.pose.pose.position.z, 3.0);
  EXPECT_EQ(output.pose.pose.orientation.x, 4.0);
  EXPECT_EQ(output.pose.pose.orientation.y, 5.0);
  EXPECT_EQ(output.pose.pose.orientation.z, 6.0);
  EXPECT_EQ(output.pose.pose.orientation.w, 7.0);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
