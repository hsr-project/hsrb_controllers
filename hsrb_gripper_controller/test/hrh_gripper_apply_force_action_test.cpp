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

#include <hsrb_gripper_controller/hrh_gripper_apply_force_action.hpp>
#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

#include "utils.hpp"


namespace hsrb_gripper_controller {

class ApplyForceActionTest : public GripperActionTestBase<tmc_control_msgs::action::GripperApplyEffort> {
 public:
  ApplyForceActionTest() : GripperActionTestBase("apply_force") {}
  virtual ~ApplyForceActionTest() = default;
};

TEST_F(ApplyForceActionTest, ActionSucceeded) {
  StartupController();

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from default gain and difference
  const auto last_command = hardware_->position->command();
  EXPECT_NEAR(last_command, 0.1 * -0.05 + 0.15 * -0.05 + 0.4 * 1.0, kEpsilon);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  // Since do_control_stop is false, the position continues to update even after the action is completed
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  EXPECT_GT(std::abs(hardware_->position->command() - last_command), kEpsilon);
}

TEST_F(ApplyForceActionTest, ControlStopAfterCompletion) {
  StartupController();

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = true;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from default gain and difference
  const auto last_command = hardware_->position->command();
  EXPECT_NEAR(last_command, 0.1 * -0.05 + 0.15 * -0.05 + 0.4 * 1.0, kEpsilon);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  // Since do_control_stop is true, the position does not update after the action is completed
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
}

TEST_F(ApplyForceActionTest, ActionAborted) {
  StartupController();

  ActionType::Goal goal;
  goal.effort = 1.15;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED)));
}

TEST_F(ApplyForceActionTest, PreemptFromOutside) {
  StartupController();

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  const auto last_command = hardware_->position->command();

  // External interruption
  controller_->PreemptActiveGoal();

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));

  // Since there was an interruption, the update stops even if do_control_stop is false
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
}

TEST_F(ApplyForceActionTest, CancelGoal) {
  StartupController();

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  const auto last_command = hardware_->position->command();

  // Throw a cancel
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel = action_client_->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_cancel);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));

  // Since there was an interruption, the update stops even if do_control_stop is false
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  EXPECT_DOUBLE_EQ(hardware_->position->command(), last_command);
}

TEST_F(ApplyForceActionTest, ForceGoalTolerance) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("force_goal_tolerance", 0.2);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.effort = 1.15;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}

TEST_F(ApplyForceActionTest, StallVelocityThreshold) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("stall_velocity_threshold", 0.03);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(1050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(500.0);
  hardware_->spring_r_position->set_current(500.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_FALSE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED)));

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  hardware_->velocity->set_current(0.03);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED)));
}

TEST_F(ApplyForceActionTest, StallTimeout) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("stall_timeout", 2.2);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(1050));

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(500.0);
  hardware_->spring_r_position->set_current(500.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_FALSE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED)));

  // Since the above WaitForStatus waits for 1 second, just wait for the remaining time
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED)));
}

TEST_F(ApplyForceActionTest, ForceControlPgain) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("force_control_pgain", 1.0);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from gain and difference
  EXPECT_NEAR(hardware_->position->command(), 1.0 * -0.05 + 0.15 * -0.05 + 0.4 * 1.0, kEpsilon);
}

TEST_F(ApplyForceActionTest, ForceControlIgain) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("force_control_igain", 1.0);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from gain and difference
  EXPECT_NEAR(hardware_->position->command(), 0.1 * -0.05 + 1.0 * -0.05 + 0.4 * 1.0, kEpsilon);
}

TEST_F(ApplyForceActionTest, ForceControlDgain) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("force_control_dgain", 1.0);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Derived from gain and difference
  EXPECT_NEAR(hardware_->position->command(), 0.1 * -0.05 + 0.15 * -0.05 + 1.0 * 1.0, kEpsilon);
}

TEST_F(ApplyForceActionTest, ForceIerrMax) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<double>("force_control_pgain", 0.0);
  node_options.append_parameter_override<double>("force_control_igain", 0.2);
  node_options.append_parameter_override<double>("force_control_dgain", 0.0);
  StartupController(node_options);

  ActionType::Goal goal;
  goal.effort = 20.0;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(0.0);
  hardware_->spring_r_position->set_current(0.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_NEAR(hardware_->position->command(), 0.2 * -0.15, kEpsilon);

  // End with a cancel
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel = action_client_->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_cancel);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
    controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));

  // Adjust force_ierr_max and confirm it is reflected
  auto results = controller_->get_node()->set_parameters({ rclcpp::Parameter("force_ierr_max", 0.05) });
  for (auto& result : results) {
    EXPECT_TRUE(result.successful);
  }
  EXPECT_EQ(controller_->on_configure(controller_->get_node()->get_current_state()),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.1));

  EXPECT_NEAR(hardware_->position->command(), 0.2 * -0.05, kEpsilon);
}

TEST_F(ApplyForceActionTest, ForceCalibDataPath) {
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<std::string>("force_calib_data_path", "test.yaml");
  StartupController(node_options);

  ActionType::Goal goal;
  goal.effort = 1.05;
  goal.do_control_stop = false;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  hardware_->velocity->set_current(0.04);
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  // Check only that there is a command value and change when there is no calibration result
  // Test whether the calibration result is correctly reflected is in hrh_gripper_controller_apply_force_calculator-test.cpp
  EXPECT_GT(std::abs(hardware_->position->command() - 0.1 * -0.05 + 0.15 * -0.05 + 0.4 * 1.0), kEpsilon);
}

TEST_F(ApplyForceActionTest, TargetMode) {
  StartupController();
  auto action_server = std::make_shared<HrhGripperApplyForceAction>(controller_.get());
  EXPECT_EQ(action_server->target_mode(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
