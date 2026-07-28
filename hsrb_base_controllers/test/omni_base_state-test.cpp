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
/// @file omni_base_state-test.cpp
/// @brief Test class for the state of an omnidirectional cart

#include <gtest/gtest.h>

#include <hsrb_base_controllers/omni_base_state.hpp>

#include "utils.hpp"

namespace {
constexpr double kEpsilon = 1.0e-9;
}  // namespace

namespace hsrb_base_controllers {

// Calculate error from actual and desired
TEST(ControllerStateTest, UpdateErrorNormal) {
  ControllerState state;
  state.actual.positions = {0.1, 0.2, 0.3};
  state.actual.velocities = {1.1, 1.2};
  state.actual.accelerations = {2.1};
  state.desired.positions = {-0.1, -0.2, -0.3};
  state.desired.velocities = {-1.1, -1.2};
  state.desired.accelerations = {-2.1};
  state.UpdateError();

  ASSERT_EQ(state.error.positions.size(), 3);
  EXPECT_NEAR(state.error.positions[0], -0.2, kEpsilon);
  EXPECT_NEAR(state.error.positions[1], -0.4, kEpsilon);
  EXPECT_NEAR(state.error.positions[2], -0.6, kEpsilon);

  ASSERT_EQ(state.error.velocities.size(), 2);
  EXPECT_NEAR(state.error.velocities[0], -2.2, kEpsilon);
  EXPECT_NEAR(state.error.velocities[1], -2.4, kEpsilon);

  ASSERT_EQ(state.error.accelerations.size(), 1);
  EXPECT_NEAR(state.error.accelerations[0], -4.2, kEpsilon);
}

// If the sizes of actual and desired do not match, error is empty
TEST(ControllerStateTest, UpdateErrorSizeMismatch) {
  ControllerState state;
  state.actual.positions = {0.1, 0.2, 0.3};
  state.actual.velocities = {1.1, 1.2};
  state.actual.accelerations = {2.1};
  state.desired.positions = {-0.1, -0.2};
  state.desired.velocities = {-1.1};
  state.desired.accelerations = {-2.1, -2.2, -2.3};
  state.UpdateError();

  EXPECT_TRUE(state.error.positions.empty());
  EXPECT_TRUE(state.error.velocities.empty());
  EXPECT_TRUE(state.error.accelerations.empty());
}

// Initialization
TEST(ControllerJointRollStateTest, Initialize) {
  ControllerJointRollState state;
  state.Reset(0.1, 0.2);

  ASSERT_EQ(state.actual.positions.size(), 1);
  EXPECT_EQ(state.actual.positions[0], 0.1);

  ASSERT_EQ(state.actual.velocities.size(), 1);
  EXPECT_EQ(state.actual.velocities[0], 0.2);

  ASSERT_EQ(state.desired.positions.size(), 1);
  EXPECT_EQ(state.desired.positions[0], 0.1);

  ASSERT_EQ(state.desired.velocities.size(), 1);
  EXPECT_EQ(state.desired.velocities[0], 0.2);

  ASSERT_EQ(state.desired.accelerations.size(), 1);
  EXPECT_EQ(state.desired.accelerations[0], 0.0);

  ASSERT_EQ(state.error.positions.size(), 1);
  EXPECT_EQ(state.error.positions[0], 0.0);

  ASSERT_EQ(state.error.velocities.size(), 1);
  EXPECT_EQ(state.error.velocities[0], 0.0);
}

// Error calculation
TEST(ControllerJointRollStateTest, UpdateError) {
  ControllerJointRollState state;
  state.Reset(0.1, 0.2);
  state.desired.positions[0] = 0.0;
  state.desired.velocities[0] = -0.1;
  state.desired.accelerations[0] = -0.2;
  state.UpdateError();

  ASSERT_EQ(state.actual.positions.size(), 1);
  EXPECT_EQ(state.actual.positions[0], 0.1);

  ASSERT_EQ(state.actual.velocities.size(), 1);
  EXPECT_EQ(state.actual.velocities[0], 0.2);

  ASSERT_EQ(state.desired.positions.size(), 1);
  EXPECT_EQ(state.desired.positions[0], 0.0);

  ASSERT_EQ(state.desired.velocities.size(), 1);
  EXPECT_EQ(state.desired.velocities[0], -0.1);

  ASSERT_EQ(state.desired.accelerations.size(), 1);
  EXPECT_EQ(state.desired.accelerations[0], -0.2);

  ASSERT_EQ(state.error.positions.size(), 1);
  EXPECT_NEAR(state.error.positions[0], -0.1, kEpsilon);

  ASSERT_EQ(state.error.velocities.size(), 1);
  EXPECT_NEAR(state.error.velocities[0], -0.3, kEpsilon);
}

// Initialization with desired
TEST(ControllerBaseStateTest, InitWithDesired) {
  ControllerBaseState state;
  state.Reset(Eigen::Vector3d(0.1, 0.2, M_PI / 2.0), Eigen::Vector3d(1.1, 1.2, 1.3),
              {-0.1, -0.2, -3.0}, {-1.1, -1.2, -1.3}, {-2.1, -2.2, -2.3});

  ASSERT_EQ(state.actual.positions.size(), 3);
  EXPECT_EQ(state.actual.positions[0], 0.1);
  EXPECT_EQ(state.actual.positions[1], 0.2);
  EXPECT_EQ(state.actual.positions[2], M_PI / 2.0);

  ASSERT_EQ(state.actual.velocities.size(), 3);
  EXPECT_NEAR(state.actual.velocities[0], -1.2, kEpsilon);
  EXPECT_NEAR(state.actual.velocities[1], 1.1, kEpsilon);
  EXPECT_NEAR(state.actual.velocities[2], 1.3, kEpsilon);

  EXPECT_TRUE(state.actual.accelerations.empty());

  ASSERT_EQ(state.desired.positions.size(), 3);
  EXPECT_EQ(state.desired.positions[0], -0.1);
  EXPECT_EQ(state.desired.positions[1], -0.2);
  EXPECT_EQ(state.desired.positions[2], -3.0);

  ASSERT_EQ(state.desired.velocities.size(), 3);
  EXPECT_EQ(state.desired.velocities[0], -1.1);
  EXPECT_EQ(state.desired.velocities[1], -1.2);
  EXPECT_EQ(state.desired.velocities[2], -1.3);

  ASSERT_EQ(state.desired.accelerations.size(), 3);
  EXPECT_EQ(state.desired.accelerations[0], -2.1);
  EXPECT_EQ(state.desired.accelerations[1], -2.2);
  EXPECT_EQ(state.desired.accelerations[2], -2.3);

  ASSERT_EQ(state.error.positions.size(), 3);
  EXPECT_NEAR(state.error.positions[0], -0.2, kEpsilon);
  EXPECT_NEAR(state.error.positions[1], -0.4, kEpsilon);
  EXPECT_NEAR(state.error.positions[2], -3.0 - M_PI / 2.0 + 2.0 * M_PI, kEpsilon);

  ASSERT_EQ(state.error.velocities.size(), 3);
  EXPECT_NEAR(state.error.velocities[0], 0.1, kEpsilon);
  EXPECT_NEAR(state.error.velocities[1], -2.3, kEpsilon);
  EXPECT_NEAR(state.error.velocities[2], -2.6, kEpsilon);

  EXPECT_TRUE(state.error.accelerations.empty());
}

// Initialization without desired
TEST(ControllerBaseStateTest, InitWithoutDesired) {
  ControllerBaseState state;
  state.Reset(Eigen::Vector3d(0.1, 0.2, M_PI / 2.0), Eigen::Vector3d(1.1, 1.2, 1.3));

  ASSERT_EQ(state.actual.positions.size(), 3);
  EXPECT_EQ(state.actual.positions[0], 0.1);
  EXPECT_EQ(state.actual.positions[1], 0.2);
  EXPECT_EQ(state.actual.positions[2], M_PI / 2.0);

  ASSERT_EQ(state.actual.velocities.size(), 3);
  EXPECT_NEAR(state.actual.velocities[0], -1.2, kEpsilon);
  EXPECT_NEAR(state.actual.velocities[1], 1.1, kEpsilon);
  EXPECT_NEAR(state.actual.velocities[2], 1.3, kEpsilon);

  ASSERT_EQ(state.desired.positions.size(), 3);
  EXPECT_EQ(state.desired.positions[0], 0.1);
  EXPECT_EQ(state.desired.positions[1], 0.2);
  EXPECT_EQ(state.desired.positions[2], M_PI / 2.0);

  ASSERT_EQ(state.desired.velocities.size(), 3);
  EXPECT_NEAR(state.desired.velocities[0], -1.2, kEpsilon);
  EXPECT_NEAR(state.desired.velocities[1], 1.1, kEpsilon);
  EXPECT_NEAR(state.desired.velocities[2], 1.3, kEpsilon);

  ASSERT_EQ(state.error.positions.size(), 3);
  EXPECT_NEAR(state.error.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(state.error.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(state.error.positions[2], 0.0, kEpsilon);

  ASSERT_EQ(state.error.velocities.size(), 3);
  EXPECT_NEAR(state.error.velocities[0], 0.0, kEpsilon);
  EXPECT_NEAR(state.error.velocities[1], 0.0, kEpsilon);
  EXPECT_NEAR(state.error.velocities[2], 0.0, kEpsilon);

  EXPECT_TRUE(state.actual.accelerations.empty());
  EXPECT_TRUE(state.desired.accelerations.empty());
  EXPECT_TRUE(state.error.accelerations.empty());
}

// Check handling of PI overflow in UpdateError
TEST(ControllerBaseStateTest, UpdateErrorOverPi) {
  ControllerBaseState state;
  state.Reset(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(), {0.0, 0.0, 2.0}, {}, {});

  ASSERT_EQ(state.error.positions.size(), 3);
  EXPECT_NEAR(state.error.positions[2], 1.0, kEpsilon);

  state.desired.positions[2] = 4.0;
  state.UpdateError();
  EXPECT_NEAR(state.error.positions[2], 3.0, kEpsilon);

  state.desired.positions[2] = 6.0;
  state.UpdateError();
  EXPECT_NEAR(state.error.positions[2], 5.0 - 2.0 * M_PI, kEpsilon);

  state.desired.positions[2] = -1.0;
  state.UpdateError();
  EXPECT_NEAR(state.error.positions[2], -2.0, kEpsilon);

  state.desired.positions[2] = -4.0;
  state.UpdateError();
  EXPECT_NEAR(state.error.positions[2], -5.0 + 2.0 * M_PI, kEpsilon);
}

// Initialization
TEST(ControllerJointStateTest, Initialize) {
  ControllerJointState state;
  state.Reset(Eigen::Vector3d(0.1, 0.2, 0.3),
              Eigen::Vector3d(1.1, 1.2, 1.3),
              -0.3,
              Eigen::Vector3d(-1.1, -1.2, -1.3));

  ASSERT_EQ(state.actual.positions.size(), 3);
  EXPECT_EQ(state.actual.positions[0], 0.1);
  EXPECT_EQ(state.actual.positions[1], 0.2);
  EXPECT_EQ(state.actual.positions[2], 0.3);

  ASSERT_EQ(state.actual.velocities.size(), 3);
  EXPECT_EQ(state.actual.velocities[0], 1.1);
  EXPECT_EQ(state.actual.velocities[1], 1.2);
  EXPECT_EQ(state.actual.velocities[2], 1.3);

  EXPECT_TRUE(state.actual.accelerations.empty());

  ASSERT_EQ(state.desired.positions.size(), 3);
  EXPECT_EQ(state.desired.positions[0], 0.0);
  EXPECT_EQ(state.desired.positions[1], 0.0);
  EXPECT_EQ(state.desired.positions[2], -0.3);

  ASSERT_EQ(state.desired.velocities.size(), 3);
  EXPECT_EQ(state.desired.velocities[0], -1.1);
  EXPECT_EQ(state.desired.velocities[1], -1.2);
  EXPECT_EQ(state.desired.velocities[2], -1.3);

  EXPECT_TRUE(state.desired.accelerations.empty());

  ASSERT_EQ(state.error.positions.size(), 3);
  EXPECT_EQ(state.error.positions[0], 0.0);
  EXPECT_EQ(state.error.positions[1], 0.0);
  EXPECT_NEAR(state.error.positions[2], -0.6, kEpsilon);

  ASSERT_EQ(state.error.velocities.size(), 3);
  EXPECT_NEAR(state.error.velocities[0], -2.2, kEpsilon);
  EXPECT_NEAR(state.error.velocities[1], -2.4, kEpsilon);
  EXPECT_NEAR(state.error.velocities[2], -2.6, kEpsilon);

  EXPECT_TRUE(state.error.accelerations.empty());
}

// Conversion to JointTrajectoryControllerState
TEST(ConvertTest, JointTrajectoryControllerState) {
  ControllerState state;
  state.actual.positions = {0.1, 0.2, 0.3};
  state.actual.velocities = {1.1, 1.2};
  state.actual.accelerations = {2.1};
  state.desired.positions = {-0.1, -0.2, -0.3};
  state.desired.velocities = {-1.1, -1.2};
  state.desired.accelerations = {-2.1};
  state.output.velocities = {3.0};
  state.UpdateError();

  control_msgs::msg::JointTrajectoryControllerState msg;
  Convert(state, rclcpp::Time(1, 2), {"joint_1", "joint_2", "joint_3"}, msg);

  EXPECT_EQ(msg.header.stamp.sec, 1);
  EXPECT_EQ(msg.header.stamp.nanosec, 2);

  ASSERT_EQ(msg.joint_names.size(), 3);
  EXPECT_EQ(msg.joint_names[0], "joint_1");
  EXPECT_EQ(msg.joint_names[1], "joint_2");
  EXPECT_EQ(msg.joint_names[2], "joint_3");

  ASSERT_EQ(msg.feedback.positions.size(), 3);
  EXPECT_NEAR(msg.feedback.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(msg.feedback.positions[1], 0.2, kEpsilon);
  EXPECT_NEAR(msg.feedback.positions[2], 0.3, kEpsilon);

  ASSERT_EQ(msg.feedback.velocities.size(), 2);
  EXPECT_NEAR(msg.feedback.velocities[0], 1.1, kEpsilon);
  EXPECT_NEAR(msg.feedback.velocities[1], 1.2, kEpsilon);

  ASSERT_EQ(msg.feedback.accelerations.size(), 1);
  EXPECT_NEAR(msg.feedback.accelerations[0], 2.1, kEpsilon);

  ASSERT_EQ(msg.reference.positions.size(), 3);
  EXPECT_NEAR(msg.reference.positions[0], -0.1, kEpsilon);
  EXPECT_NEAR(msg.reference.positions[1], -0.2, kEpsilon);
  EXPECT_NEAR(msg.reference.positions[2], -0.3, kEpsilon);

  ASSERT_EQ(msg.reference.velocities.size(), 2);
  EXPECT_NEAR(msg.reference.velocities[0], -1.1, kEpsilon);
  EXPECT_NEAR(msg.reference.velocities[1], -1.2, kEpsilon);

  ASSERT_EQ(msg.reference.accelerations.size(), 1);
  EXPECT_NEAR(msg.reference.accelerations[0], -2.1, kEpsilon);

  ASSERT_EQ(msg.error.positions.size(), 3);
  EXPECT_NEAR(msg.error.positions[0], -0.2, kEpsilon);
  EXPECT_NEAR(msg.error.positions[1], -0.4, kEpsilon);
  EXPECT_NEAR(msg.error.positions[2], -0.6, kEpsilon);

  ASSERT_EQ(msg.error.velocities.size(), 2);
  EXPECT_NEAR(msg.error.velocities[0], -2.2, kEpsilon);
  EXPECT_NEAR(msg.error.velocities[1], -2.4, kEpsilon);

  ASSERT_EQ(msg.error.accelerations.size(), 1);
  EXPECT_NEAR(msg.error.accelerations[0], -4.2, kEpsilon);

  ASSERT_EQ(msg.output.velocities.size(), 1);
  EXPECT_NEAR(msg.output.velocities[0], 3.0, kEpsilon);
}

// Conversion to JointTrajectoryControllerState (as controller state)
TEST(ConvertAsControllerStateTest, JointTrajectoryControllerState) {
  ControllerState state;
  state.actual.positions = {0.1, 0.2, 0.3};
  state.actual.velocities = {1.1, 1.2};
  state.actual.accelerations = {2.1};
  state.desired.positions = {-0.1, -0.2, -0.3};
  state.desired.velocities = {-1.1, -1.2};
  state.desired.accelerations = {-2.1};
  state.output.velocities = {3.0};
  state.UpdateError();

  control_msgs::msg::JointTrajectoryControllerState msg;
  ConvertAsControllerState(state, rclcpp::Time(1, 2), {"joint_1", "joint_2", "joint_3"}, msg);

  EXPECT_EQ(msg.header.stamp.sec, 1);
  EXPECT_EQ(msg.header.stamp.nanosec, 2);

  ASSERT_EQ(msg.joint_names.size(), 3);
  EXPECT_EQ(msg.joint_names[0], "joint_1");
  EXPECT_EQ(msg.joint_names[1], "joint_2");
  EXPECT_EQ(msg.joint_names[2], "joint_3");

  ASSERT_EQ(msg.feedback.positions.size(), 3);
  EXPECT_NEAR(msg.feedback.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(msg.feedback.positions[1], 0.2, kEpsilon);
  EXPECT_NEAR(msg.feedback.positions[2], 0.3, kEpsilon);

  ASSERT_EQ(msg.feedback.velocities.size(), 2);
  EXPECT_NEAR(msg.feedback.velocities[0], 1.1, kEpsilon);
  EXPECT_NEAR(msg.feedback.velocities[1], 1.2, kEpsilon);

  ASSERT_EQ(msg.feedback.accelerations.size(), 1);
  EXPECT_NEAR(msg.feedback.accelerations[0], 2.1, kEpsilon);

  ASSERT_EQ(msg.reference.positions.size(), 3);
  EXPECT_NEAR(msg.reference.positions[0], -0.1, kEpsilon);
  EXPECT_NEAR(msg.reference.positions[1], -0.2, kEpsilon);
  EXPECT_NEAR(msg.reference.positions[2], -0.3, kEpsilon);

  ASSERT_EQ(msg.reference.velocities.size(), 2);
  EXPECT_NEAR(msg.reference.velocities[0], -1.1, kEpsilon);
  EXPECT_NEAR(msg.reference.velocities[1], -1.2, kEpsilon);

  ASSERT_EQ(msg.reference.accelerations.size(), 1);
  EXPECT_NEAR(msg.reference.accelerations[0], -2.1, kEpsilon);

  ASSERT_EQ(msg.error.positions.size(), 3);
  EXPECT_NEAR(msg.error.positions[0], -0.2, kEpsilon);
  EXPECT_NEAR(msg.error.positions[1], -0.4, kEpsilon);
  EXPECT_NEAR(msg.error.positions[2], -0.6, kEpsilon);

  ASSERT_EQ(msg.error.velocities.size(), 2);
  EXPECT_NEAR(msg.error.velocities[0], -2.2, kEpsilon);
  EXPECT_NEAR(msg.error.velocities[1], -2.4, kEpsilon);

  ASSERT_EQ(msg.error.accelerations.size(), 1);
  EXPECT_NEAR(msg.error.accelerations[0], -4.2, kEpsilon);

  ASSERT_EQ(msg.output.velocities.size(), 1);
  EXPECT_NEAR(msg.output.velocities[0], 3.0, kEpsilon);
}

// Conversion to JointTrajectoryPoint
TEST(ConvertTest, JointTrajectoryPoint) {
  State state;
  state.positions = {0.1, 0.2, 0.3};
  state.velocities = {1.1, 1.2};
  state.accelerations = {2.1};

  trajectory_msgs::msg::JointTrajectoryPoint msg;
  Convert(state, msg);

  ASSERT_EQ(msg.positions.size(), 3);
  EXPECT_NEAR(msg.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(msg.positions[1], 0.2, kEpsilon);
  EXPECT_NEAR(msg.positions[2], 0.3, kEpsilon);

  ASSERT_EQ(msg.velocities.size(), 2);
  EXPECT_NEAR(msg.velocities[0], 1.1, kEpsilon);
  EXPECT_NEAR(msg.velocities[1], 1.2, kEpsilon);

  ASSERT_EQ(msg.accelerations.size(), 1);
  EXPECT_NEAR(msg.accelerations[0], 2.1, kEpsilon);
}

// Correctly published by StatePublisher
TEST(StatePublisherTest, Publish) {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  node->declare_parameter("state_publish_rate", 2.0);
  auto pub = std::make_shared<StatePublisher>(
      node, "~/state", std::vector<std::string>({"joint_1", "joint_2", "joint_3"}));
  auto pub_as_controller_state = std::make_shared<ControllerStatePublisher>(
      node, "~/controller_state", std::vector<std::string>({"joint_1", "joint_2", "joint_3"}));
  node->activate();

  ControllerState state;
  state.actual.positions = {0.1, 0.2, 0.3};
  state.actual.velocities = {1.1, 1.2};
  state.actual.accelerations = {2.1};
  state.desired.positions = {-0.1, -0.2, -0.3};
  state.desired.velocities = {-1.1, -1.2};
  state.desired.accelerations = {-2.1};
  state.output.velocities = {3.0};
  state.UpdateError();

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto counter = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
      client_node, "test_node/state");
  auto counter_for_controller_state = std::make_shared<SubscriptionCounter<
      control_msgs::msg::JointTrajectoryControllerState>>(client_node, "test_node/controller_state");
  pub->set_last_state_published_time(node->now());
  pub_as_controller_state->set_last_state_published_time(node->now());

  rclcpp::WallRate loop_rate(10.0);
  for (uint32_t i = 0; i < 12; ++i) {
    pub->Publish(state, node->now());
    pub_as_controller_state->Publish(state, node->now());
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(node->get_node_base_interface());
    loop_rate.sleep();
  }

  // check pub results
  EXPECT_EQ(counter->count(), 2);

  auto msg = counter->last_msg();

  ASSERT_EQ(msg.joint_names.size(), 3);
  EXPECT_EQ(msg.joint_names[0], "joint_1");
  EXPECT_EQ(msg.joint_names[1], "joint_2");
  EXPECT_EQ(msg.joint_names[2], "joint_3");

  ASSERT_EQ(msg.feedback.positions.size(), 3);
  EXPECT_NEAR(msg.feedback.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(msg.feedback.positions[1], 0.2, kEpsilon);
  EXPECT_NEAR(msg.feedback.positions[2], 0.3, kEpsilon);

  ASSERT_EQ(msg.feedback.velocities.size(), 2);
  EXPECT_NEAR(msg.feedback.velocities[0], 1.1, kEpsilon);
  EXPECT_NEAR(msg.feedback.velocities[1], 1.2, kEpsilon);

  ASSERT_EQ(msg.feedback.accelerations.size(), 1);
  EXPECT_NEAR(msg.feedback.accelerations[0], 2.1, kEpsilon);

  ASSERT_EQ(msg.reference.positions.size(), 3);
  EXPECT_NEAR(msg.reference.positions[0], -0.1, kEpsilon);
  EXPECT_NEAR(msg.reference.positions[1], -0.2, kEpsilon);
  EXPECT_NEAR(msg.reference.positions[2], -0.3, kEpsilon);

  ASSERT_EQ(msg.reference.velocities.size(), 2);
  EXPECT_NEAR(msg.reference.velocities[0], -1.1, kEpsilon);
  EXPECT_NEAR(msg.reference.velocities[1], -1.2, kEpsilon);

  ASSERT_EQ(msg.reference.accelerations.size(), 1);
  EXPECT_NEAR(msg.reference.accelerations[0], -2.1, kEpsilon);

  ASSERT_EQ(msg.error.positions.size(), 3);
  EXPECT_NEAR(msg.error.positions[0], -0.2, kEpsilon);
  EXPECT_NEAR(msg.error.positions[1], -0.4, kEpsilon);
  EXPECT_NEAR(msg.error.positions[2], -0.6, kEpsilon);

  ASSERT_EQ(msg.error.velocities.size(), 2);
  EXPECT_NEAR(msg.error.velocities[0], -2.2, kEpsilon);
  EXPECT_NEAR(msg.error.velocities[1], -2.4, kEpsilon);

  ASSERT_EQ(msg.error.accelerations.size(), 1);
  EXPECT_NEAR(msg.error.accelerations[0], -4.2, kEpsilon);

  ASSERT_EQ(msg.output.velocities.size(), 1);
  EXPECT_NEAR(msg.output.velocities[0], 3.0, kEpsilon);

  // check pub_as_controller_state results
  EXPECT_EQ(counter_for_controller_state->count(), 2);

  auto msg_as_controller_state = counter_for_controller_state->last_msg();
  ASSERT_EQ(msg_as_controller_state.joint_names.size(), 3);
  EXPECT_EQ(msg_as_controller_state.joint_names[0], "joint_1");
  EXPECT_EQ(msg_as_controller_state.joint_names[1], "joint_2");
  EXPECT_EQ(msg_as_controller_state.joint_names[2], "joint_3");

  ASSERT_EQ(msg_as_controller_state.feedback.positions.size(), 3);
  EXPECT_NEAR(msg_as_controller_state.feedback.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(msg_as_controller_state.feedback.positions[1], 0.2, kEpsilon);
  EXPECT_NEAR(msg_as_controller_state.feedback.positions[2], 0.3, kEpsilon);

  ASSERT_EQ(msg_as_controller_state.feedback.velocities.size(), 2);
  EXPECT_NEAR(msg_as_controller_state.feedback.velocities[0], 1.1, kEpsilon);
  EXPECT_NEAR(msg_as_controller_state.feedback.velocities[1], 1.2, kEpsilon);

  ASSERT_EQ(msg_as_controller_state.feedback.accelerations.size(), 1);
  EXPECT_NEAR(msg_as_controller_state.feedback.accelerations[0], 2.1, kEpsilon);

  ASSERT_EQ(msg_as_controller_state.reference.positions.size(), 3);
  EXPECT_NEAR(msg_as_controller_state.reference.positions[0], -0.1, kEpsilon);
  EXPECT_NEAR(msg_as_controller_state.reference.positions[1], -0.2, kEpsilon);
  EXPECT_NEAR(msg_as_controller_state.reference.positions[2], -0.3, kEpsilon);

  ASSERT_EQ(msg_as_controller_state.reference.velocities.size(), 2);
  EXPECT_NEAR(msg_as_controller_state.reference.velocities[0], -1.1, kEpsilon);
  EXPECT_NEAR(msg_as_controller_state.reference.velocities[1], -1.2, kEpsilon);

  ASSERT_EQ(msg_as_controller_state.reference.accelerations.size(), 1);
  EXPECT_NEAR(msg_as_controller_state.reference.accelerations[0], -2.1, kEpsilon);

  ASSERT_EQ(msg_as_controller_state.error.positions.size(), 3);
  EXPECT_NEAR(msg_as_controller_state.error.positions[0], -0.2, kEpsilon);
  EXPECT_NEAR(msg_as_controller_state.error.positions[1], -0.4, kEpsilon);
  EXPECT_NEAR(msg_as_controller_state.error.positions[2], -0.6, kEpsilon);

  ASSERT_EQ(msg_as_controller_state.error.velocities.size(), 2);
  EXPECT_NEAR(msg_as_controller_state.error.velocities[0], -2.2, kEpsilon);
  EXPECT_NEAR(msg_as_controller_state.error.velocities[1], -2.4, kEpsilon);

  ASSERT_EQ(msg_as_controller_state.error.accelerations.size(), 1);
  EXPECT_NEAR(msg_as_controller_state.error.accelerations[0], -4.2, kEpsilon);

  ASSERT_EQ(msg_as_controller_state.output.velocities.size(), 1);
  EXPECT_NEAR(msg_as_controller_state.output.velocities[0], 3.0, kEpsilon);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
