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
/// @brief Test of controller state

#include <gtest/gtest.h>

#include <hsrb_gripper_controller/hrh_gripper_distance.hpp>

#include "utils.hpp"

namespace {

}  // namespace

namespace hsrb_gripper_controller {

// Correctly published by StatePublisher
TEST(DistancePublisherTest, Publish) {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  node->declare_parameter("distance_publish_rate", 2.0);
  node->declare_parameter("robot_description", ReadRobotDescriptionFromFile());
  auto pub = std::make_shared<DistancePublisher>(node, "~/fingertip_distance");
  node->activate();

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto counter = std::make_shared<SubscriptionCounter<std_msgs::msg::Float32>>(
      client_node, "test_node/fingertip_distance");
  pub->SetLastStatePublishedTime(node->now());

  rclcpp::WallRate loop_rate(10.0);
  for (uint32_t i = 0; i < 12; ++i) {
    pub->Publish(1.0, node->now());
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(node->get_node_base_interface());
    loop_rate.sleep();
  }
  EXPECT_EQ(counter->count(), 2);

  auto msg = counter->last_msg();

  EXPECT_NEAR(msg.data, 0.122194, kEpsilon);
}

// Correctly published by StatePublisher
TEST(DistancePublisherTest, PublishRate) {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  node->declare_parameter("distance_publish_rate", 5.0);
  node->declare_parameter("robot_description", ReadRobotDescriptionFromFile());
  auto pub = std::make_shared<DistancePublisher>(node, "~/fingertip_distance");
  node->activate();

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto counter = std::make_shared<SubscriptionCounter<std_msgs::msg::Float32>>(
      client_node, "test_node/fingertip_distance");
  pub->SetLastStatePublishedTime(node->now());

  rclcpp::WallRate loop_rate(10.0);
  for (uint32_t i = 0; i < 12; ++i) {
    pub->Publish(1.0, node->now());
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(node->get_node_base_interface());
    loop_rate.sleep();
  }
  EXPECT_EQ(counter->count(), 5);
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
