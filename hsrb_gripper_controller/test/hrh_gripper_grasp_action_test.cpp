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
/// @brief Test for Hrh grip control action

#include <gtest/gtest.h>

#include <hsrb_gripper_controller/hrh_gripper_grasp_action.hpp>
#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

#include "utils.hpp"

namespace hsrb_gripper_controller {

class GraspActionTest : public GripperActionTestBase<tmc_control_msgs::action::GripperApplyEffort> {
 public:
  GraspActionTest() : GripperActionTestBase("grasp") {}
  virtual ~GraspActionTest() = default;
};

TEST_F(GraspActionTest, ActionSucceeded) {
  StartupController();

  ActionType::Goal goal;
  goal.effort = 3.0;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Grip start
  hardware_->effort->set_current(0.0);
  hardware_->grasping_flag->set_current(false);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_TRUE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  // Gripping in progress
  hardware_->grasping_flag->set_current(true);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_FALSE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  // Grip complete
  hardware_->effort->set_current(2.1);
  hardware_->grasping_flag->set_current(false);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_FALSE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(GraspActionTest, ActionAborted) {
  StartupController();

  ActionType::Goal goal;
  goal.effort = 3.0;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Grip start
  hardware_->effort->set_current(1.9);
  hardware_->grasping_flag->set_current(false);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Gripping in progress
  hardware_->grasping_flag->set_current(true);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Grip complete
  hardware_->grasping_flag->set_current(false);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED)));
}

TEST_F(GraspActionTest, PreemptFromOutside) {
  StartupController();

  ActionType::Goal goal;
  goal.effort = 3.0;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Grip start
  hardware_->effort->set_current(1.9);
  hardware_->grasping_flag->set_current(false);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_TRUE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  // External interruption
  controller_->PreemptActiveGoal();

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));

  // State does not change due to interruption
  hardware_->grasping_flag->set_current(true);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_TRUE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);
}

TEST_F(GraspActionTest, CancelGoal) {
  StartupController();

  ActionType::Goal goal;
  goal.effort = 3.0;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Grip start
  hardware_->effort->set_current(1.9);
  hardware_->grasping_flag->set_current(false);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Throw cancel
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel = action_client_->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_cancel);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GraspActionTest, GoalTorelance) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("torque_goal_tolerance", 1.2);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.effort = 3.0;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // Grip start
  hardware_->effort->set_current(1.9);
  hardware_->grasping_flag->set_current(false);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Gripping in progress
  hardware_->grasping_flag->set_current(true);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Grip complete
  hardware_->grasping_flag->set_current(false);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(GraspActionTest, TargetMode) {
  StartupController();
  auto action_server = std::make_shared<HrhGripperGraspAction>(controller_.get());
  EXPECT_EQ(action_server->target_mode(), tmc_exxx_servo_motor_protocol::kDriveModeHandGrasp);
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
