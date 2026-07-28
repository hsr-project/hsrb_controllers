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
/// @file hrh_gripper_controller_output_position_control_test.cpp
/// @brief Test of HRH gripper controller
#include <gtest/gtest.h>

#include <hsrb_gripper_controller/hrh_gripper_follow_trajectory_action.hpp>
#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

#include "utils.hpp"


namespace hsrb_gripper_controller {
class HrhGripperControllerTest : public GripperActionTestBase<control_msgs::action::FollowJointTrajectory> {
 public:
  HrhGripperControllerTest() : GripperActionTestBase("follow_joint_trajectory") {}
  virtual ~HrhGripperControllerTest() = default;

  void SetUp() override;
};

void HrhGripperControllerTest::SetUp() {
  GripperActionTestBase::SetUp();
  hardware_->position->set_current(-0.4);
  hardware_->velocity->set_current(0.0);
  hardware_->spring_l_position->set_current(0.05);
  hardware_->spring_r_position->set_current(0.05);
}

// Position control at the output joint position considering spring joints
TEST_F(HrhGripperControllerTest, FollowTrajectoryWithSpringJoint) {
  StartupController();

  ActionType::Goal goal;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { -0.4 };
  point.time_from_start = rclcpp::Duration(0, 500000000);

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = { kHandJointName };
  trajectory.points = { point };

  goal.trajectory = trajectory;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // The trajectory is determined during the first update, so move the position near the target position after calling Update
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  std::vector<double> command_positions;
  command_positions.push_back(hardware_->position->command());

  hardware_->spring_l_position->set_current(0.05);
  hardware_->spring_r_position->set_current(0.05);
  hardware_->position->set_current(-0.4);

  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    EXPECT_NEAR(hardware_->position->command(), -0.45, 0.01);
  }
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
      controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}
// Position control with position correction due to overcurrent
TEST_F(HrhGripperControllerTest, FollowTrajectoryWithOverCurrent) {
  StartupController();

  ActionType::Goal goal;
  trajectory_msgs::msg::JointTrajectoryPoint point;

  point.positions = { -0.4 };
  point.time_from_start = rclcpp::Duration(0, 500000000);

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = { kHandJointName };
  trajectory.points = { point };

  goal.trajectory = trajectory;

  auto future_goal_handle = action_client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  // The trajectory is determined during the first update, so move the position near the target position after calling Update
  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));

  std::vector<double> command_positions;
  command_positions.push_back(hardware_->position->command());

  hardware_->spring_l_position->set_current(0.05);
  hardware_->spring_r_position->set_current(0.05);
  hardware_->position->set_current(-0.4);
  hardware_->current->set_current(-1.6);

  controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
  hardware_->current->set_current(0.0);
  rclcpp::WallRate rate(100.0);
  double prev_position = hardware_->position->command();
  for (int i = 0; i < 60; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_->get_node_base_interface());
    controller_->update(node_->now(), rclcpp::Duration::from_seconds(0.1));
    hardware_->current->set_current(0.0);
    if ((hardware_->position->command() - prev_position) == 0.0) {
      break;
    }
    EXPECT_NEAR(fabs(hardware_->position->command() - prev_position), 0.001, 0.002);
    prev_position = hardware_->position->command();
  }
  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE((WaitForStatus<ActionType, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>(
      controller_, { node_->get_node_base_interface() }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
}
}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
