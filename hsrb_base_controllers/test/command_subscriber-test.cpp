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
/// @file command_subscriber-test.cpp
/// @brief Test class for input command control of omnidirectional vehicle control

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <hsrb_base_controllers/command_subscriber.hpp>
#include <hsrb_base_controllers/omni_base_control_method.hpp>

#include "utils.hpp"

namespace {

template<typename Action>
bool WaitForStatus(rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr,
                   typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle,
                   int8_t expected) {
  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(1.0);
  while (goal_handle->get_status() != expected) {
    rclcpp::spin_some(node_ptr->get_node_base_interface());
    if (std::chrono::system_clock::now() > end_time) {
      std::cout << "The last status is " << static_cast<int32_t>(goal_handle->get_status()) << std::endl;
      return false;
    }
  }
  return true;
}

}  // namespace


namespace hsrb_base_controllers {

class CommandInterfaceMock : public IControllerCommandInterface {
 public:
  MOCK_METHOD0(IsAcceptable, bool());
  MOCK_METHOD1(UpdateVelocity, void(const geometry_msgs::msg::Twist::SharedPtr&));
  MOCK_METHOD1(ValidateTrajectory, bool(const trajectory_msgs::msg::JointTrajectory&));
  MOCK_METHOD1(UpdateTrajectory, void(const trajectory_msgs::msg::JointTrajectory::SharedPtr&));
  MOCK_METHOD1(ResetTrajectory, void(const std::shared_ptr<OmniBaseTrajectoryControl>&));
  MOCK_METHOD0(PreemptActiveGoal, void());
};

class OmniBaseTrajectoryControlMock : public OmniBaseTrajectoryControl {
 public:
  explicit OmniBaseTrajectoryControlMock(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
      : OmniBaseTrajectoryControl(node, {}) {}

  MOCK_METHOD2(AcceptTrajectory, void(const trajectory_msgs::msg::JointTrajectory::SharedPtr&, const Eigen::Vector3d&));
};


class CommandVelocitySubscriberTest  : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<CommandInterfaceMock> interface_mock_;
  CommandVelocitySubscriber::Ptr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

void CommandVelocitySubscriberTest::SetUp() {
  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node_->configure();
  interface_mock_ = std::make_shared<CommandInterfaceMock>();
  subscriber_ = std::make_shared<CommandVelocitySubscriber>(node_, interface_mock_.get());
  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", rclcpp::SystemDefaultsQoS());
  node_->activate();
}

// Pass the received velocity command to the Controller
TEST_F(CommandVelocitySubscriberTest, SubscribeOnRunning) {
  bool is_called = false;
  EXPECT_CALL(*interface_mock_, IsAcceptable())
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Assign(&is_called, true), ::testing::Return(true)));

  auto received_command = std::make_shared<geometry_msgs::msg::Twist>();
  EXPECT_CALL(*interface_mock_, UpdateVelocity(::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::SaveArgPointee<0>(received_command));

  geometry_msgs::msg::Twist command_msg;
  command_msg.linear.x = 1.0;

  publisher_->publish(command_msg);
  auto timeout = TimeoutDetection(node_->get_clock());
  while (!is_called) {
    timeout.Run();
    rclcpp::spin_some(node_->get_node_base_interface());
  }

  EXPECT_EQ(received_command->linear.x, 1.0);
}

// Do not pass the received velocity command to the Controller as command reception is not allowed
TEST_F(CommandVelocitySubscriberTest, SubscribeOnStopped) {
  bool is_called = false;
  EXPECT_CALL(*interface_mock_, IsAcceptable())
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Assign(&is_called, true), ::testing::Return(false)));
  EXPECT_CALL(*interface_mock_, UpdateVelocity(::testing::_)).Times(0);

  geometry_msgs::msg::Twist command_msg;
  command_msg.linear.x = 1.0;

  publisher_->publish(command_msg);
  auto timeout = TimeoutDetection(node_->get_clock());
  while (!is_called) {
    timeout.Run();
    rclcpp::spin_some(node_->get_node_base_interface());
  }
}


class CommandTrajectorySubscriberTest  : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<CommandInterfaceMock> interface_mock_;
  CommandTrajectorySubscriber::Ptr subscriber_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  OmniBaseTrajectoryControl::Ptr target_control_;
};

void CommandTrajectorySubscriberTest::SetUp() {
  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  interface_mock_ = std::make_shared<CommandInterfaceMock>();
  subscriber_ = std::make_shared<CommandTrajectorySubscriber>(node_, "~/joint_trajectory", interface_mock_.get());
  publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS());

  target_control_ = std::make_shared<OmniBaseTrajectoryControlMock>(node_);
  subscriber_->set_target_control(target_control_);
}

// Pass the received joint trajectory to the Controller
TEST_F(CommandTrajectorySubscriberTest, SubscribeOnRunning) {
  bool is_called = false;
  EXPECT_CALL(*interface_mock_, IsAcceptable())
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Assign(&is_called, true), ::testing::Return(true)));

  auto validated_command = trajectory_msgs::msg::JointTrajectory();
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::SaveArg<0>(&validated_command), ::testing::Return(true)));

  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(1);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(target_control_)).Times(1);

  auto received_command = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::SaveArgPointee<0>(received_command));

  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.frame_id = "test";

  publisher_->publish(command_msg);
  auto timeout = TimeoutDetection(node_->get_clock());
  while (!is_called) {
    timeout.Run();
    rclcpp::spin_some(node_->get_node_base_interface());
  }

  EXPECT_EQ(validated_command.header.frame_id, "test");
  EXPECT_EQ(received_command->header.frame_id, "test");
}

// Do not pass the received joint trajectory to the Controller as command reception is not allowed
TEST_F(CommandTrajectorySubscriberTest, SubscribeOnStopped) {
  bool is_called = false;
  EXPECT_CALL(*interface_mock_, IsAcceptable())
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Assign(&is_called, true), ::testing::Return(false)));
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_)).Times(0);
  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(0);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(::testing::_)).Times(0);
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_)).Times(0);

  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.frame_id = "test";

  publisher_->publish(command_msg);
  auto timeout = TimeoutDetection(node_->get_clock());
  while (!is_called) {
    timeout.Run();
    rclcpp::spin_some(node_->get_node_base_interface());
  }
}

// Do not pass the received joint trajectory to the Controller as it is invalid
TEST_F(CommandTrajectorySubscriberTest, SubscribeInvalidTrajectory) {
  bool is_called = false;
  EXPECT_CALL(*interface_mock_, IsAcceptable())
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Assign(&is_called, true), ::testing::Return(true)));
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_)).Times(1).WillRepeatedly(::testing::Return(false));
  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(0);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(::testing::_)).Times(0);
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_)).Times(0);

  trajectory_msgs::msg::JointTrajectory command_msg;
  command_msg.header.frame_id = "test";

  publisher_->publish(command_msg);
  auto timeout = TimeoutDetection(node_->get_clock());
  while (!is_called) {
    timeout.Run();
    rclcpp::spin_some(node_->get_node_base_interface());
  }
}


class TrajectoryActionServerTest : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<::testing::NiceMock<CommandInterfaceMock>> interface_mock_;
  TrajectoryActionServer::Ptr server_;

  using ActionType = control_msgs::action::FollowJointTrajectory;
  rclcpp_action::Client<ActionType>::SharedPtr client_;

  OmniBaseTrajectoryControl::Ptr target_control_;
};

void TrajectoryActionServerTest::SetUp() {
  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  interface_mock_ = std::make_shared<::testing::NiceMock<CommandInterfaceMock>>();

  node_->declare_parameter("constraints.odom_x.trajectory", 1.0);
  node_->declare_parameter("constraints.odom_x.goal", 1.0);
  node_->declare_parameter("constraints.goal_time", 1.0);

  std::vector<std::string> coordinate = {"odom_x", "odom_y", "odom_t"};
  server_ = std::make_shared<TrajectoryActionServer>(
      node_, coordinate, "~/follow_joint_trajectory", interface_mock_.get());

  client_ = rclcpp_action::create_client<ActionType>(node_, "~/follow_joint_trajectory");
  EXPECT_TRUE(client_->wait_for_action_server());

  target_control_ = std::make_shared<OmniBaseTrajectoryControlMock>(node_);
  server_->set_target_control(target_control_);
}

// Pass the received joint trajectory to the Controller
TEST_F(TrajectoryActionServerTest, ReceiveGoal) {
  EXPECT_CALL(*interface_mock_, IsAcceptable()).Times(1).WillRepeatedly(::testing::Return(true));

  auto validated_command = trajectory_msgs::msg::JointTrajectory();
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::SaveArg<0>(&validated_command), ::testing::Return(true)));

  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(1);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(target_control_)).Times(1);

  auto received_command = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::SaveArgPointee<0>(received_command));

  ActionType::Goal goal;
  goal.trajectory.header.frame_id = "test";

  auto future_goal_handle = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>(node_, goal_handle, action_msgs::msg::GoalStatus::STATUS_ACCEPTED));

  EXPECT_EQ(validated_command.header.frame_id, "test");
  EXPECT_EQ(received_command->header.frame_id, "test");
}

// Do not accept the goal as command reception is not allowed
TEST_F(TrajectoryActionServerTest, ReceiveGoalOnStopped) {
  EXPECT_CALL(*interface_mock_, IsAcceptable()).Times(1).WillRepeatedly(::testing::Return(false));

  ActionType::Goal goal;
  auto future_goal_handle = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

// Do not accept the goal as the joint trajectory is invalid
TEST_F(TrajectoryActionServerTest, ReceiveInvalidGoal) {
  EXPECT_CALL(*interface_mock_, IsAcceptable()).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_)).Times(1).WillRepeatedly(::testing::Return(false));
  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(0);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(::testing::_)).Times(0);
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_)).Times(0);

  ActionType::Goal goal;
  auto future_goal_handle = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  EXPECT_EQ(future_goal_handle.get().get(), nullptr);
}

// Cancel the action
TEST_F(TrajectoryActionServerTest, CancelGoal) {
  EXPECT_CALL(*interface_mock_, IsAcceptable()).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_)).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(1);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(target_control_)).Times(1);
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_)).Times(1);

  ActionType::Goal goal;
  auto future_goal_handle = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  EXPECT_CALL(*interface_mock_, ResetTrajectory(::testing::IsNull())).Times(1);

  auto future_cancel = client_->async_cancel_goal(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_cancel);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);

  EXPECT_TRUE(WaitForStatus<ActionType>(node_, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

// UpdateResult during trajectory tracking, excessive error
TEST_F(TrajectoryActionServerTest, UpdateResultOutsidePathTorelance) {
  EXPECT_CALL(*interface_mock_, IsAcceptable()).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_)).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(1);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(target_control_)).Times(1);
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_)).Times(1);

  ActionType::Goal goal;
  auto future_goal_handle = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  server_->UpdateActionResult(control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(node_, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));
}

// UpdateResult upon completion of trajectory tracking, normal case
TEST_F(TrajectoryActionServerTest, UpdateResultInsideGoalTorelance) {
  EXPECT_CALL(*interface_mock_, IsAcceptable()).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_)).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(1);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(target_control_)).Times(1);
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_)).Times(1);

  ActionType::Goal goal;
  auto future_goal_handle = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  server_->UpdateActionResult(control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(node_, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

// UpdateResult upon completion of trajectory tracking, unable to reach the goal within the time
TEST_F(TrajectoryActionServerTest, UpdateResultOutsideGoalTimeTorelance) {
  EXPECT_CALL(*interface_mock_, IsAcceptable()).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_)).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(1);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(target_control_)).Times(1);
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_)).Times(1);

  ActionType::Goal goal;
  auto future_goal_handle = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  server_->UpdateActionResult(control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(node_, goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));
}

// Issue feedback
TEST_F(TrajectoryActionServerTest, PublishFeedback) {
  EXPECT_CALL(*interface_mock_, IsAcceptable()).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_)).Times(1).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(1);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(target_control_)).Times(1);
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_)).Times(1);

  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  rclcpp_action::Client<ActionType>::SendGoalOptions send_goal_options;
  ActionType::Feedback feedback;
  send_goal_options.feedback_callback = [&feedback](GoalHandle::SharedPtr,
                                                    const std::shared_ptr<const ActionType::Feedback> _feedback) {
    feedback = *_feedback;
  };

  ActionType::Goal goal;
  auto future_goal_handle = client_->async_send_goal(goal, send_goal_options);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  auto stamp = node_->get_clock()->now();
  auto state = ControllerBaseState();
  state.Reset(Eigen::Vector3d(2.0, 1.0, 0.0), Eigen::Vector3d(3.0, 4.0, 5.0));
  server_->SetFeedback(state, stamp);

  auto timeout = TimeoutDetection(node_->get_clock());
  while (feedback.actual.positions.size() != 3) {
    timeout.Run();
    rclcpp::spin_some(node_->get_node_base_interface());
  }
  EXPECT_EQ(rclcpp::Time(feedback.header.stamp).nanoseconds(), stamp.nanoseconds());

  ASSERT_EQ(feedback.actual.positions.size(), 3);
  EXPECT_DOUBLE_EQ(feedback.actual.positions[0], 2.0);
  EXPECT_DOUBLE_EQ(feedback.actual.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(feedback.actual.positions[2], 0.0);

  ASSERT_EQ(feedback.actual.velocities.size(), 3);
  EXPECT_DOUBLE_EQ(feedback.actual.velocities[0], 3.0);
  EXPECT_DOUBLE_EQ(feedback.actual.velocities[1], 4.0);
  EXPECT_DOUBLE_EQ(feedback.actual.velocities[2], 5.0);

  ASSERT_EQ(feedback.desired.positions.size(), 3);
  EXPECT_DOUBLE_EQ(feedback.desired.positions[0], 2.0);
  EXPECT_DOUBLE_EQ(feedback.desired.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(feedback.desired.positions[2], 0.0);

  ASSERT_EQ(feedback.desired.velocities.size(), 3);
  EXPECT_DOUBLE_EQ(feedback.desired.velocities[0], 3.0);
  EXPECT_DOUBLE_EQ(feedback.desired.velocities[1], 4.0);
  EXPECT_DOUBLE_EQ(feedback.desired.velocities[2], 5.0);

  ASSERT_EQ(feedback.error.positions.size(), 3);
  EXPECT_DOUBLE_EQ(feedback.error.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(feedback.error.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(feedback.error.positions[2], 0.0);

  ASSERT_EQ(feedback.error.velocities.size(), 3);
  EXPECT_DOUBLE_EQ(feedback.error.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(feedback.error.velocities[1], 0.0);
  EXPECT_DOUBLE_EQ(feedback.error.velocities[2], 0.0);

  EXPECT_TRUE(feedback.actual.accelerations.empty());
  EXPECT_TRUE(feedback.desired.accelerations.empty());
  EXPECT_TRUE(feedback.error.accelerations.empty());

  feedback = ActionType::Feedback();

  stamp = node_->get_clock()->now();
  state = ControllerBaseState();
  state.Reset(Eigen::Vector3d(2.0, 1.0, 0.0), Eigen::Vector3d(3.0, 4.0, 5.0),
              {2.1, 1.2, 0.3}, {3.4, 4.5, 5.6}, {6.0, 7.0, 8.0});
  server_->SetFeedback(state, stamp);

  timeout = TimeoutDetection(node_->get_clock());
  while (feedback.desired.positions.size() != 3) {
    timeout.Run();
    rclcpp::spin_some(node_->get_node_base_interface());
  }
  ASSERT_EQ(feedback.desired.positions.size(), 3);
  EXPECT_DOUBLE_EQ(feedback.desired.positions[0], 2.1);
  EXPECT_DOUBLE_EQ(feedback.desired.positions[1], 1.2);
  EXPECT_DOUBLE_EQ(feedback.desired.positions[2], 0.3);

  ASSERT_EQ(feedback.desired.velocities.size(), 3);
  EXPECT_DOUBLE_EQ(feedback.desired.velocities[0], 3.4);
  EXPECT_DOUBLE_EQ(feedback.desired.velocities[1], 4.5);
  EXPECT_DOUBLE_EQ(feedback.desired.velocities[2], 5.6);

  ASSERT_EQ(feedback.desired.accelerations.size(), 3);
  EXPECT_DOUBLE_EQ(feedback.desired.accelerations[0], 6.0);
  EXPECT_DOUBLE_EQ(feedback.desired.accelerations[1], 7.0);
  EXPECT_DOUBLE_EQ(feedback.desired.accelerations[2], 8.0);

  ASSERT_EQ(feedback.error.positions.size(), 3);
  EXPECT_NEAR(feedback.error.positions[0], 0.1, 1e-6);
  EXPECT_NEAR(feedback.error.positions[1], 0.2, 1e-6);
  EXPECT_NEAR(feedback.error.positions[2], 0.3, 1e-6);

  ASSERT_EQ(feedback.error.velocities.size(), 3);
  EXPECT_NEAR(feedback.error.velocities[0], 0.4, 1e-6);
  EXPECT_NEAR(feedback.error.velocities[1], 0.5, 1e-6);
  EXPECT_NEAR(feedback.error.velocities[2], 0.6, 1e-6);

  EXPECT_TRUE(feedback.error.accelerations.empty());
}

// A new goal arrives after trajectory tracking is completed
TEST_F(TrajectoryActionServerTest, AcceptNewGoal) {
  EXPECT_CALL(*interface_mock_, IsAcceptable()).Times(2).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_)).Times(2).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(2);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(target_control_)).Times(2);
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_)).Times(2);

  ActionType::Goal goal;
  auto future_goal_handle = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  server_->UpdateActionResult(control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(node_, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  future_goal_handle = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle);

  goal_handle = future_goal_handle.get();
  EXPECT_TRUE(WaitForStatus<ActionType>(node_, goal_handle, action_msgs::msg::GoalStatus::STATUS_ACCEPTED));
}

// A new goal arrives before trajectory tracking is completed
TEST_F(TrajectoryActionServerTest, AcceptNewGoalBeforeComplete) {
  EXPECT_CALL(*interface_mock_, IsAcceptable()).Times(2).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, ValidateTrajectory(::testing::_)).Times(2).WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*interface_mock_, PreemptActiveGoal()).Times(2);
  EXPECT_CALL(*interface_mock_, ResetTrajectory(target_control_)).Times(2);
  EXPECT_CALL(*interface_mock_, UpdateTrajectory(::testing::_)).Times(2);

  ActionType::Goal goal;
  auto future_goal_handle_first = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle_first);

  auto goal_handle_first = future_goal_handle_first.get();
  EXPECT_TRUE(goal_handle_first.get());

  auto future_goal_handle_second = client_->async_send_goal(goal);
  rclcpp::spin_until_future_complete(node_, future_goal_handle_second);

  auto goal_handle_second = future_goal_handle_second.get();
  EXPECT_TRUE(goal_handle_second.get());

  EXPECT_TRUE(WaitForStatus<ActionType>(node_, goal_handle_first, action_msgs::msg::GoalStatus::STATUS_CANCELED));
  EXPECT_TRUE(WaitForStatus<ActionType>(node_, goal_handle_second, action_msgs::msg::GoalStatus::STATUS_ACCEPTED));
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
