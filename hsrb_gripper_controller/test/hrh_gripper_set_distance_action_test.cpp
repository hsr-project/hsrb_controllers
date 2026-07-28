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
/// @brief Test of Hrh grip control action

#include <gtest/gtest.h>

#include <hsrb_gripper_controller/hrh_gripper_set_distance_action.hpp>
#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

#include "utils.hpp"


namespace hsrb_gripper_controller {

class SetDistanceActionTest : public GripperActionTestBase<tmc_control_msgs::action::GripperSetDistance> {
 public:
  SetDistanceActionTest() : GripperActionTestBase("set_distance") {
    distance_calculator_ = std::make_shared<HrhGripperDistanceCalculator>();
  }
  virtual ~SetDistanceActionTest() = default;
 protected:
  HrhGripperDistanceCalculator::Ptr distance_calculator_;
};

TEST_F(SetDistanceActionTest, ActionSucceeded) {
  StartupController();
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(1050));

  double current_position = distance_calculator_->GetPositionFromDistance(0.052);
  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(current_position);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_NEAR(hardware_->position->command(),
              (current_position + 2.0 * -0.002 + 0.0 * -0.002 + 2.5 * -0.002),
              kEpsilon);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(SetDistanceActionTest, ActionAborted) {
  StartupController();
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(1350));

  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.057));

  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED)));
}

TEST_F(SetDistanceActionTest, PreemptFromOutside) {
  StartupController();
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.052));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // External interrupt
  controller_->PreemptActiveGoal();

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(SetDistanceActionTest, CancelGoal) {
  StartupController();
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.052));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Throw a cancel
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel = action_client_->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_cancel);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(SetDistanceActionTest, DistanceGoalTolerance) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("distance_goal_tolerance", 0.005);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(1050));

  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.055));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(SetDistanceActionTest, StallVelocityThreshold) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("stall_velocity_threshold", 0.03);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(1050));

  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.052));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_FALSE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  // Since WaitForStatus above waits for 1 second, wait a little
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  hardware_->velocity->set_current(0.03);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(SetDistanceActionTest, StallTimeout) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("distance_control_stall_timeout", 2.2);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(1050));

  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.052));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_FALSE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  // Since WaitForStatus above waits for 1 second, just wait for the remaining time
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(SetDistanceActionTest, DistanceControlPgain) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("distance_control_pgain", 1.0);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  double current_position = distance_calculator_->GetPositionFromDistance(0.052);
  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(current_position);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from gain and difference
  EXPECT_NEAR(hardware_->position->command(),
              (current_position + 1.0 * -0.002 + 0.0 * -0.002 + 2.5 * -0.002),
              kEpsilon);
}

TEST_F(SetDistanceActionTest, DistanceControlIgain) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("distance_control_igain", 2.0);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  double current_position = distance_calculator_->GetPositionFromDistance(0.052);
  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(current_position);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from gain and difference
  EXPECT_NEAR(hardware_->position->command(),
              (current_position + 2.0 * -0.002 + 2.0 * -0.002 + 2.5 * -0.002),
              kEpsilon);
}

TEST_F(SetDistanceActionTest, DistanceControlDgain) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("distance_control_dgain", 1.0);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  double current_position = distance_calculator_->GetPositionFromDistance(0.052);
  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(current_position);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from gain and difference
  EXPECT_NEAR(hardware_->position->command(),
              (current_position + 2.0 * -0.002 + 0.0 * -0.002 + 1.0 * -0.002),
              kEpsilon);
}

TEST_F(SetDistanceActionTest, DistanceMaxThreashold) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("hand_motor_joint_max", 0.3);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.052));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_NEAR(hardware_->position->command(), 0.3, kEpsilon);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  hardware_->position->set_current(0.3);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from default gain and difference
  const auto last_command = hardware_->position->command();
  EXPECT_LT(distance_calculator_->GetDistanceFromPosition(last_command), 0.05);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(SetDistanceActionTest, DistanceMinThreashold) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("hand_motor_joint_min", 0.5);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.052));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_NEAR(hardware_->position->command(), 0.5, kEpsilon);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  hardware_->position->set_current(0.5);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from default gain and difference
  const auto last_command = hardware_->position->command();
  EXPECT_GT(distance_calculator_->GetDistanceFromPosition(last_command), 0.05);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(SetDistanceActionTest, AcceptDistanceTopic) {
  StartupController();
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  auto publisher =
      node_->create_publisher<std_msgs::msg::Float32>("~/command_distance", rclcpp::SystemDefaultsQoS());

  std_msgs::msg::Float32 distance_topic;
  distance_topic.data = 0.05;
  publisher->on_activate();
  publisher->publish(distance_topic);

  rclcpp::WallRate rate(10.0);
  for (int i = 0; i < 10; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
  }

  double current_position = distance_calculator_->GetPositionFromDistance(0.052);
  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(current_position);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from default gain and difference
  const auto last_command = hardware_->position->command();
  EXPECT_NEAR(last_command,
              (current_position + 2.0 * -0.002 + 0.0 * -0.002 + 2.5 * -0.002),
              kEpsilon);
}

TEST_F(SetDistanceActionTest, AcceptDistanceTopicThreasholdMax) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("hand_motor_joint_max", 0.3);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  auto publisher =
      node_->create_publisher<std_msgs::msg::Float32>("~/command_distance", rclcpp::SystemDefaultsQoS());

  std_msgs::msg::Float32 distance_topic;
  distance_topic.data = 0.05;
  publisher->on_activate();
  publisher->publish(distance_topic);

  rclcpp::WallRate rate(10.0);
  for (int i = 0; i < 10; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
  }

  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.052));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  EXPECT_NEAR(hardware_->position->command(), 0.3, kEpsilon);
}

TEST_F(SetDistanceActionTest, AcceptDistanceTopicThreasholdMin) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("hand_motor_joint_min", 0.5);
  StartupController(node_options);
  ASSERT_TRUE(distance_calculator_->InitializeHandSizeData(node_));

  auto publisher =
      node_->create_publisher<std_msgs::msg::Float32>("~/command_distance", rclcpp::SystemDefaultsQoS());

  std_msgs::msg::Float32 distance_topic;
  distance_topic.data = 0.05;
  publisher->on_activate();
  publisher->publish(distance_topic);

  rclcpp::WallRate rate(10.0);
  for (int i = 0; i < 10; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
  }

  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(distance_calculator_->GetPositionFromDistance(0.052));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  EXPECT_NEAR(hardware_->position->command(), 0.5, kEpsilon);
}

TEST_F(SetDistanceActionTest, TargetMode) {
  StartupController();
  auto action_server = std::make_shared<HrhGripperSetDistanceAction>(controller_.get());
  EXPECT_EQ(action_server->target_mode(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
