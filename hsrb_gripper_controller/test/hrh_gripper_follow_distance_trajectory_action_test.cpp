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
/// @brief Test of Hrh opening width time-following control action

#include <gtest/gtest.h>

#include <hsrb_gripper_controller/hrh_gripper_follow_distance_trajectory_action.hpp>
#include <tmc_control_msgs/action/gripper_apply_effort.hpp>
#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

#include "utils.hpp"

namespace hsrb_gripper_controller {

class FollowDistanceTrajectoryActionTest : public GripperActionTestBase<control_msgs::action::FollowJointTrajectory> {
 public:
  FollowDistanceTrajectoryActionTest() : GripperActionTestBase("follow_distance_trajectory") {
    distance_calculator_ = std::make_shared<HrhGripperDistanceCalculator>();
  }
  virtual ~FollowDistanceTrajectoryActionTest() = default;

  void SetUp() override;
 protected:
  HrhGripperDistanceCalculator::Ptr distance_calculator_;
};

void FollowDistanceTrajectoryActionTest::SetUp() {
  GripperActionTestBase::SetUp();
  hardware_->position->set_current(0.0);
  hardware_->velocity->set_current(0.0);
}

TEST_F(FollowDistanceTrajectoryActionTest, ActionSucceeded) {
  StartupController();
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Since the trajectory is determined during the first update, move the position near the target position after calling Update
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  std::vector<double> command_distances;
  command_distances.push_back(distance_calculator_->GetDistanceFromPosition(hardware_->position->command()));

  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.1));

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_distances.push_back(distance_calculator_->GetDistanceFromPosition(hardware_->position->command()));
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  double previous_command = 0.0;
  for (double command : command_distances) {
    EXPECT_LE(command, (0.1 + kEpsilon));
    EXPECT_GE(command, previous_command);
    previous_command = command;
  }
  EXPECT_LT(command_distances.front(), command_distances.back());
}

TEST_F(FollowDistanceTrajectoryActionTest, GoalToleranceViolated) {
  StartupController();
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Since the trajectory is determined during the first update, move the position near the target position after calling Update
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.107));

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED)));
}

TEST_F(FollowDistanceTrajectoryActionTest, PreemptFromOutside) {
  StartupController();

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 30; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  }

  double last_command = hardware_->position->command();

  // External interruption
  controller_->PreemptActiveGoal();

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));

  // The state does not change due to the interruption
  for (int i = 0; i < 10; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
  }
}

TEST_F(FollowDistanceTrajectoryActionTest, CancelGoal) {
  StartupController();

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 30; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  }

  double last_command = hardware_->position->command();

  // Throw a cancel
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel = action_client_->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_cancel);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));

  // The state does not change due to the interruption
  for (int i = 0; i < 10; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
  }
}

TEST_F(FollowDistanceTrajectoryActionTest, DistanceGoalTolerance) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("distance_goal_tolerance", 0.005);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Since the trajectory is determined during the first update, move the position after calling Update
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.105));

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(FollowDistanceTrajectoryActionTest, PositionGoalTimeTolerance) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("position_goal_time_tolerance", 0.15);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Since the trajectory is determined during the first update, move the position after calling Update
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.105));

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(FollowDistanceTrajectoryActionTest, DistanceMaxThreashold) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("hand_motor_joint_max", 0.5);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Since the trajectory is determined during the first update, move the position near the target position after calling Update
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  std::vector<double> command_positions;
  command_positions.push_back(hardware_->position->command());

  hardware_->position->set_current(0.5);

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_positions.push_back(hardware_->position->command());
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  for (double command : command_positions) {
    EXPECT_LE(command, (0.5 + kEpsilon));
  }
}

TEST_F(FollowDistanceTrajectoryActionTest, DistanceMinThreashold) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("hand_motor_joint_min", 0.5);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  // Assume it is at the minimum position
  hardware_->position->set_current(0.5);

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Since the trajectory is determined during the first update, move the position near the target position after calling Update
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  std::vector<double> command_positions;
  command_positions.push_back(hardware_->position->command());

  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.1));

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_positions.push_back(hardware_->position->command());
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  for (double command : command_positions) {
    EXPECT_GE(command, (0.5 - kEpsilon));
  }
}

TEST_F(FollowDistanceTrajectoryActionTest, InvalidJointName) {
  StartupController();

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();
  goal.trajectory.joint_names = { "invalid" };

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

TEST_F(FollowDistanceTrajectoryActionTest, InvalidJointNumbers) {
  StartupController();

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();
  goal.trajectory.joint_names = { kHandJointName, kHandJointName };

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

TEST_F(FollowDistanceTrajectoryActionTest, ZeroDurationTrajectory) {
  StartupController();

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();
  goal.trajectory.points[0].time_from_start = rclcpp::Duration(0, 0);

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

TEST_F(FollowDistanceTrajectoryActionTest, PastTrajectory) {
  StartupController();

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();
  goal.trajectory.header.stamp = node_->now() - rclcpp::Duration(2, 0);

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

TEST_F(FollowDistanceTrajectoryActionTest, EmptyTrajectory) {
  StartupController();

  ActionType::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();
  goal.trajectory.points.clear();

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

TEST_F(FollowDistanceTrajectoryActionTest, AcceptDistanceTrajectoryTopic) {
  StartupController();
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  auto publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("~/distance_trajectory",
                                                                                  rclcpp::SystemDefaultsQoS());

  publisher->on_activate();
  publisher->publish(MakeDistanceTrajectory());

  std::vector<double> command_distances;
  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_distances.push_back(distance_calculator_->GetDistanceFromPosition(hardware_->position->command()));
  }

  double previous_command = 0.0;
  for (double command : command_distances) {
    EXPECT_LE(command, (0.1 + kEpsilon));
    EXPECT_GE(command, previous_command);
    previous_command = command;
  }
  EXPECT_LT(command_distances.front(), command_distances.back());
}

TEST_F(FollowDistanceTrajectoryActionTest, AcceptDistanceTrajectoryTopicThreasholdMax) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("hand_motor_joint_max", 0.5);
  StartupController(node_options);

  auto publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("~/distance_trajectory",
                                                                                  rclcpp::SystemDefaultsQoS());

  publisher->on_activate();
  publisher->publish(MakeDistanceTrajectory());

  std::vector<double> command_positions;
  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_positions.push_back(hardware_->position->command());
  }

  for (double command : command_positions) {
    EXPECT_LE(command, (0.5 + kEpsilon));
  }
}

TEST_F(FollowDistanceTrajectoryActionTest, AcceptDistanceTrajectoryTopicThreasholdMin) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("hand_motor_joint_min", 0.5);
  StartupController(node_options);

  // Assume it is at the minimum position
  hardware_->position->set_current(0.5);

  auto publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("~/distance_trajectory",
                                                                                  rclcpp::SystemDefaultsQoS());

  publisher->on_activate();
  publisher->publish(MakeDistanceTrajectory());

  std::vector<double> command_positions;
  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_positions.push_back(hardware_->position->command());
  }

  for (double command : command_positions) {
    EXPECT_GE(command, (0.5 - kEpsilon));
  }
}

TEST_F(FollowDistanceTrajectoryActionTest, RejectTrajectoryTopic) {
  StartupController();

  auto publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("~/distance_trajectory",
                                                                                  rclcpp::SystemDefaultsQoS());
  publisher->on_activate();

  auto trajectory = MakeDistanceTrajectory();
  trajectory.joint_names = { "invalid" };
  publisher->publish(trajectory);

  double last_command = hardware_->position->command();

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 10; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
  }
}

TEST_F(FollowDistanceTrajectoryActionTest, TargetMode) {
  StartupController();
  auto action_server = std::make_shared<HrhGripperFollowDistanceTrajectoryAction>(controller_.get());
  EXPECT_EQ(action_server->target_mode(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
}

TEST_F(FollowDistanceTrajectoryActionTest, WithoutOpenLoopControl) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<bool>("open_loop_control", false);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.05));

  auto publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("~/distance_trajectory",
                                                                                  rclcpp::SystemDefaultsQoS());
  publisher->on_activate();

  const auto trajectory = MakeDistanceTrajectory();
  publisher->publish(trajectory);

  std::vector<double> command_distances;
  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 25; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_distances.push_back(distance_calculator_->GetDistanceFromPosition(hardware_->position->command()));
  }

  publisher->publish(trajectory);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_distances.push_back(distance_calculator_->GetDistanceFromPosition(hardware_->position->command()));
  }

  // The behavior is monotonically increasing from the current value, and then increases again from the current value midway
  uint32_t command_jumping = 0;
  double previous_command = 0.05;
  for (double command : command_distances) {
    EXPECT_LE(command, (0.1 + kEpsilon));
    if (command < previous_command) {
      ++command_jumping;
    }
    previous_command = command;
  }
  EXPECT_EQ(command_jumping, 1);

  // At 0.1m/sec for 10 frames at 100Hz, the expected value is 0.01; if the loop is unstable, allow some buffer
  EXPECT_NEAR((command_distances[15] - command_distances[5]), 0.01, 0.01);
  // The speed should be the same in the latter half
  EXPECT_NEAR((command_distances[50] - command_distances[40]), 0.01, 0.01);
}

TEST_F(FollowDistanceTrajectoryActionTest, WithOpenLoopControl) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<bool>("open_loop_control", true);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  auto publisher = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("~/distance_trajectory",
                                                                                  rclcpp::SystemDefaultsQoS());
  publisher->on_activate();

  const auto trajectory = MakeDistanceTrajectory();
  publisher->publish(trajectory);

  std::vector<double> command_distances;
  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 25; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_distances.push_back(distance_calculator_->GetDistanceFromPosition(hardware_->position->command()));
  }

  publisher->publish(trajectory);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_distances.push_back(distance_calculator_->GetDistanceFromPosition(hardware_->position->command()));
  }

  // It continues to increase monotonically from the initial value (0.0), but the speed changes at the 25th point
  double previous_command = 0.0;
  for (double command : command_distances) {
    EXPECT_LE(command, (0.1 + kEpsilon));
    EXPECT_GE(command, previous_command);
    previous_command = command;
  }

  // At 0.2m/sec for 10 frames at 100Hz, the expected value is 0.02; if the loop is unstable, allow some buffer
  EXPECT_NEAR(command_distances[15] - command_distances[5], 0.02, 0.01);
  // The speed should be halved in the latter half
  EXPECT_NEAR(command_distances[50] - command_distances[40], 0.01, 0.01);

  // To shorten the test time, create the client first so that wait_for completes quickly
  auto apply_force_client = rclcpp_action::create_client<tmc_control_msgs::action::GripperApplyEffort>(
      node_, std::string(kControllerNodeName) + "/apply_force");

  // In the same control mode, it connects from the last command value
  const auto another_trajectory = MakeDistanceTrajectory(0.05);
  publisher->publish(another_trajectory);
  command_distances.clear();
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_distances.push_back(distance_calculator_->GetDistanceFromPosition(hardware_->position->command()));
  }

  // Monotonically decreasing from the last command value of 0.1
  previous_command = 0.1;
  for (double command : command_distances) {
    EXPECT_GE(command, (0.05 - kEpsilon));
    EXPECT_LE(command, previous_command);
    previous_command = command;
  }

  // When switching to another control mode, the retained value is reset and connects from the current value
  apply_force_client->wait_for_action_server();
  auto apply_force_goal = tmc_control_msgs::action::GripperApplyEffort::Goal();
  apply_force_goal.effort = 1.0;
  auto apply_effort_future_goal_handle = apply_force_client->async_send_goal(apply_force_goal);
  rclcpp::spin_until_future_complete(node_, apply_effort_future_goal_handle);

  // Publish the trajectory again
  publisher->publish(trajectory);
  command_distances.clear();
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    command_distances.push_back(distance_calculator_->GetDistanceFromPosition(hardware_->position->command()));
  }

  // Monotonically increasing from the current value (0.0)
  previous_command = 0.0;
  for (double command : command_distances) {
    EXPECT_LE(command, (0.1 + kEpsilon));
    EXPECT_GE(command, previous_command);
    previous_command = command;
  }
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
