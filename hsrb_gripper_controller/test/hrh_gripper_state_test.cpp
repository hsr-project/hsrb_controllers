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

#include <hsrb_gripper_controller/hrh_gripper_state.hpp>

#include "utils.hpp"

namespace {

trajectory_msgs::msg::JointTrajectoryPoint MakeReferencePoint() {
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { 0.5 };
  point.velocities = { 1.0 };
  point.accelerations = { 1.5 };
  point.effort = { 2.0 };

  return point;
}

trajectory_msgs::msg::JointTrajectoryPoint MakeFeedbackPoint() {
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { 1.0 };
  point.velocities = { 2.0 };
  point.accelerations = { 3.0 };
  point.effort = { 4.0 };

  return point;
}

}  // namespace

namespace hsrb_gripper_controller {

// Correctly published by StatePublisher
TEST(StatePublisherTest, Publish) {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  node->declare_parameter("state_publish_rate", 2.0);
  auto pub = std::make_shared<StatePublisher>(node, "~/controller_state", "joint_1");
  node->activate();

  trajectory_msgs::msg::JointTrajectoryPoint reference = MakeReferencePoint();
  trajectory_msgs::msg::JointTrajectoryPoint feedback = MakeFeedbackPoint();

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto counter = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
      client_node, "test_node/controller_state");
  pub->SetLastStatePublishedTime(node->now());

  rclcpp::WallRate loop_rate(10.0);
  for (uint32_t i = 0; i < 12; ++i) {
    pub->Publish(reference, feedback, node->now());
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(node->get_node_base_interface());
    loop_rate.sleep();
  }
  EXPECT_EQ(counter->count(), 2);

  trajectory_msgs::msg::JointTrajectoryPoint check_reference;
  check_reference.positions = { 0.5 };
  check_reference.velocities = { 1.0 };
  check_reference.accelerations = { 1.5 };
  check_reference.effort = { 2.0 };

  trajectory_msgs::msg::JointTrajectoryPoint check_feedback;
  check_feedback.positions = { 1.0 };
  check_feedback.velocities = { 2.0 };
  check_feedback.accelerations = { 3.0 };
  check_feedback.effort = { 4.0 };

  trajectory_msgs::msg::JointTrajectoryPoint check_error;
  check_error.positions = { -0.5 };
  check_error.velocities = { -1.0 };
  check_error.accelerations = { -1.5 };
  check_error.effort = { -2.0 };

  CheckStateMsg(counter->last_msg(), check_reference, check_feedback, check_error, { "joint_1" });
}

// Publication rate can be changed with state_publish_rate
TEST(StatePublisherTest, PublishRate) {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  node->declare_parameter("state_publish_rate", 5.0);
  auto pub = std::make_shared<StatePublisher>(node, "~/controller_state", "joint_1");
  node->activate();

  trajectory_msgs::msg::JointTrajectoryPoint reference = MakeReferencePoint();
  trajectory_msgs::msg::JointTrajectoryPoint feedback = MakeFeedbackPoint();

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto counter = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
      client_node, "test_node/controller_state");
  pub->SetLastStatePublishedTime(node->now());

  rclcpp::WallRate loop_rate(10.0);
  for (uint32_t i = 0; i < 12; ++i) {
    pub->Publish(reference, feedback, node->now());
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(node->get_node_base_interface());
    loop_rate.sleep();
  }
  EXPECT_EQ(counter->count(), 5);
}

// Does not calculate error if reference and feedback sizes differ
TEST(StatePublisherTest, ErrorSizeMismatch) {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  node->declare_parameter("state_publish_rate", 2.0);
  auto pub = std::make_shared<StatePublisher>(node, "~/controller_state", "joint_1");
  node->activate();

  trajectory_msgs::msg::JointTrajectoryPoint reference = MakeReferencePoint();
  trajectory_msgs::msg::JointTrajectoryPoint feedback;

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto counter = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
      client_node, "test_node/controller_state");
  pub->SetLastStatePublishedTime(node->now());

  rclcpp::WallRate loop_rate(10.0);
  for (uint32_t i = 0; i < 12; ++i) {
    pub->Publish(reference, feedback, node->now());
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(node->get_node_base_interface());
    loop_rate.sleep();
  }
  EXPECT_EQ(counter->count(), 2);

  auto msg = counter->last_msg();
  ASSERT_EQ(msg.reference.positions.size(), 1);
  ASSERT_EQ(msg.reference.velocities.size(), 1);
  ASSERT_EQ(msg.reference.accelerations.size(), 1);
  ASSERT_EQ(msg.reference.effort.size(), 1);
  EXPECT_TRUE(msg.feedback.positions.empty());
  EXPECT_TRUE(msg.feedback.velocities.empty());
  EXPECT_TRUE(msg.feedback.accelerations.empty());
  EXPECT_TRUE(msg.feedback.effort.empty());
  EXPECT_TRUE(msg.error.positions.empty());
  EXPECT_TRUE(msg.error.velocities.empty());
  EXPECT_TRUE(msg.error.accelerations.empty());
  EXPECT_TRUE(msg.error.effort.empty());
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
