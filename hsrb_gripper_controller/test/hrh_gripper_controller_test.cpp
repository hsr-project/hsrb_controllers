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
/// @file hrh_gripper_controller-test.cpp
/// @brief Test of HRH gripper controller, checking if action interruption works

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/float32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <hsrb_gripper_controller/hrh_gripper_set_distance_action.hpp>
#include <tmc_control_msgs/action/gripper_apply_effort.hpp>
#include <tmc_control_msgs/action/gripper_set_distance.hpp>
#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

#include "utils.hpp"

namespace hsrb_gripper_controller {


class GripperControllerTest : public ::testing::Test {
 public:
  void SetUp() override {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override<std::string>("robot_description", ReadRobotDescriptionFromFile());
    controller_ = std::make_shared<TestableHrhGripperController>(node_options);
    controller_node_ = controller_->get_node();

    // To eliminate the effect of update being called multiple times with WaitFor, set igain to zero
    controller_node_->declare_parameter<double>("force_control_igain", 0.0);

    hardware_ = std::make_shared<HardwareStub>(kHandJointName);
    controller_->assign_interfaces(std::move(hardware_->command_interfaces), std::move(hardware_->state_interfaces));
    EXPECT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    client_node_ = rclcpp::Node::make_shared(kClientNodeName);
    trajectory_publisher_ = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        std::string(kControllerNodeName) + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
    distance_publisher_ = client_node_->create_publisher<std_msgs::msg::Float32>(
        std::string(kControllerNodeName) + "/command_distance", rclcpp::SystemDefaultsQoS());
    distance_trajectory_publisher_ = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        std::string(kControllerNodeName) + "/distance_trajectory", rclcpp::SystemDefaultsQoS());

    apply_force_action_client_ = rclcpp_action::create_client<ApplayEffortAction>(
        client_node_, std::string(kControllerNodeName) + "/apply_force");
    grasp_action_client_ = rclcpp_action::create_client<ApplayEffortAction>(
        client_node_, std::string(kControllerNodeName) + "/grasp");
    follow_trajectory_action_client_ = rclcpp_action::create_client<FollowTrajectoryAction>(
        client_node_, std::string(kControllerNodeName) + "/follow_joint_trajectory");
    set_distance_action_client_ = rclcpp_action::create_client<SetDistanceAction>(
        client_node_, std::string(kControllerNodeName) + "/set_distance");
    follow_distance_trajectory_action_client_ = rclcpp_action::create_client<FollowTrajectoryAction>(
        client_node_, std::string(kControllerNodeName) + "/follow_distance_trajectory");
    state_subscriber_ = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
        client_node_, std::string(kControllerNodeName) + "/controller_state");
    distance_subscriber_ = std::make_shared<SubscriptionCounter<std_msgs::msg::Float32>>(
        client_node_, std::string(kControllerNodeName) + "/fingertip_distance");

    EXPECT_TRUE(apply_force_action_client_->wait_for_action_server());
    EXPECT_TRUE(grasp_action_client_->wait_for_action_server());
    EXPECT_TRUE(follow_trajectory_action_client_->wait_for_action_server());
    EXPECT_TRUE(set_distance_action_client_->wait_for_action_server());
    EXPECT_TRUE(follow_distance_trajectory_action_client_->wait_for_action_server());
    executor_.add_node(controller_->get_node()->get_node_base_interface());
  }

  void TearDown() override {
    controller_->release_interfaces();
  }

 protected:
  TestableHrhGripperController::Ptr controller_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> controller_node_;
  HardwareStub::Ptr hardware_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr distance_trajectory_publisher_;

  using ApplayEffortAction = tmc_control_msgs::action::GripperApplyEffort;
  rclcpp_action::Client<ApplayEffortAction>::SharedPtr apply_force_action_client_;
  rclcpp_action::Client<ApplayEffortAction>::SharedPtr grasp_action_client_;

  using FollowTrajectoryAction = control_msgs::action::FollowJointTrajectory;
  rclcpp_action::Client<FollowTrajectoryAction>::SharedPtr follow_trajectory_action_client_;
  rclcpp_action::Client<FollowTrajectoryAction>::SharedPtr follow_distance_trajectory_action_client_;

  using SetDistanceAction = tmc_control_msgs::action::GripperSetDistance;
  rclcpp_action::Client<SetDistanceAction>::SharedPtr set_distance_action_client_;

  SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>::Ptr state_subscriber_;
  SubscriptionCounter<std_msgs::msg::Float32>::Ptr distance_subscriber_;

  void SpinOnce(rclcpp::WallRate& rate) {
    rate.sleep();
    executor_.spin_some();
    rclcpp::spin_some(client_node_);
    EXPECT_EQ(controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.1)),
              controller_interface::return_type::OK);
  }

  template <typename FutureT>
  void WaitFor(rclcpp::WallRate& rate, const typename std::shared_future<FutureT>& future) {
    while (rclcpp::spin_until_future_complete(client_node_, future, std::chrono::microseconds(1)) ==
           rclcpp::FutureReturnCode::TIMEOUT) {
      SpinOnce(rate);
    }
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<ApplayEffortAction>> StartApplyForceAction() {
    ApplayEffortAction::Goal goal;
    goal.effort = 1.0;
    auto future_goal_handle = apply_force_action_client_->async_send_goal(goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    // Wait for apply_force to start
    while (rclcpp::ok()) {
      SpinOnce(rate);
      // Expected command value is 0.1 * -1.0
      if (std::abs(hardware_->position->command() + 0.1) < kEpsilon) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    return goal_handle;
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowTrajectoryAction>> StartFollowTrajectoryAction() {
    // To prevent aborted due to excessive deviation during trajectory tracking, set a value close to the trajectory goal position
    hardware_->position->set_current(0.96);

    FollowTrajectoryAction::Goal follow_goal;
    follow_goal.trajectory = MakeTrajectory();
    auto future_goal_handle = follow_trajectory_action_client_->async_send_goal(follow_goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    // Wait for tracking to start
    while (rclcpp::ok()) {
      SpinOnce(rate);
      if (hardware_->position->command() > 0.96) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    return goal_handle;
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<ApplayEffortAction>> StartGraspAction() {
    ApplayEffortAction::Goal grasp_goal;
    grasp_goal.effort = 3.0;
    auto future_goal_handle = grasp_action_client_->async_send_goal(grasp_goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    // Wait for gripping to start
    while (rclcpp::ok()) {
      SpinOnce(rate);
      if (hardware_->grasping_flag->bool_command()) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandGrasp);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    return goal_handle;
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<SetDistanceAction>> StartSetDistanceAction() {
    // Set the speed to prevent the action from ending
    hardware_->velocity->set_current(0.1);

    SetDistanceAction::Goal goal;
    goal.distance = 0.05;
    auto future_goal_handle = set_distance_action_client_->async_send_goal(goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    // Wait for set_distance to start
    while (rclcpp::ok()) {
      SpinOnce(rate);
      // Since the goal is around 0.33, the command value will be larger than the initial value
      if (hardware_->position->command() > 0.0) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    return goal_handle;
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowTrajectoryAction>> StartFollowDistanceTrajectoryAction() {
    // To prevent aborted due to excessive deviation during trajectory tracking, set a value close to the trajectory goal position (around 0.75)
    hardware_->position->set_current(0.73);

    FollowTrajectoryAction::Goal follow_goal;
    follow_goal.trajectory = MakeDistanceTrajectory();
    auto future_goal_handle = follow_distance_trajectory_action_client_->async_send_goal(follow_goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    // Wait for tracking to start
    while (rclcpp::ok()) {
      SpinOnce(rate);
      if (hardware_->position->command() > 0.0) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    return goal_handle;
  }

  void SendTrajectoryTopic() {
    // To prevent aborted due to excessive deviation during trajectory tracking, set a value close to the trajectory goal position
    hardware_->position->set_current(1.46);

    auto trajectory = MakeTrajectory();
    trajectory.points[0].positions[0] = 1.5;
    trajectory_publisher_->publish(trajectory);

    // Wait for tracking to start
    rclcpp::WallRate rate(100.0);
    while (rclcpp::ok()) {
      SpinOnce(rate);
      if (hardware_->position->command() > 1.46) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
  }

  void SendDistanceTopic() {
    // Set the speed to prevent the process from ending
    hardware_->velocity->set_current(0.1);

    std_msgs::msg::Float32 distance_topic;
    distance_topic.data = 0.05;
    distance_publisher_->publish(distance_topic);

    // Wait for set_distance to start
    rclcpp::WallRate rate(100.0);
    while (rclcpp::ok()) {
      SpinOnce(rate);
      // Since the goal is around 0.33, the command value will be larger than the initial value
      if (hardware_->position->command() > 0.0) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
  }

  void SendDistanceTrajectoryTopic() {
    auto trajectory = MakeDistanceTrajectory();
    distance_trajectory_publisher_->publish(trajectory);

    // Wait for tracking to start
    rclcpp::WallRate rate(100.0);
    while (rclcpp::ok()) {
      SpinOnce(rate);
      if (hardware_->position->command() > 0.0) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
  }

  void PreemptWithApplyForceAction() {
    // Set the speed to allow the action to end
    hardware_->velocity->set_current(0.0);

    ApplayEffortAction::Goal goal;
    goal.effort = 1.0;
    goal.do_control_stop = false;

    auto future_goal_handle = apply_force_action_client_->async_send_goal(goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    std::this_thread::sleep_for(std::chrono::milliseconds(2050));

    // Set so that the target force and the low-pass filtered force are the same value
    hardware_->position->set_current(0.0);
    hardware_->spring_l_position->set_current(5.0);
    hardware_->spring_r_position->set_current(5.0);
    SpinOnce(rate);
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);

    // Derived from gain and difference
    EXPECT_NEAR(hardware_->position->command(), 0.4 * 1.0, kEpsilon);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
      controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
  }

  void PreemptWithFollowTrajectoryAction() {
    // To prevent aborted due to excessive deviation during trajectory tracking, set a value close to the trajectory goal position
    hardware_->position->set_current(0.96);

    FollowTrajectoryAction::Goal follow_goal;
    follow_goal.trajectory = MakeTrajectory();

    auto future_goal_handle = follow_trajectory_action_client_->async_send_goal(follow_goal);
    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    // Test trajectory moves to 1.0 in 0.5 seconds
    for (int i = 0; i < 60; ++i) {
      SpinOnce(rate);
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
    EXPECT_DOUBLE_EQ(hardware_->position->command(), 1.0);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
      controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
  }

  void PreemptWithGraspAction() {
    // To ensure the action succeeds, align the current force with the target force
    hardware_->effort->set_current(3.0);

    ApplayEffortAction::Goal grasp_goal;
    grasp_goal.effort = 3.0;
    auto future_grasp_goal_handle = grasp_action_client_->async_send_goal(grasp_goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_grasp_goal_handle);

    // Wait for gripping to start
    while (rclcpp::ok()) {
      SpinOnce(rate);
      if (hardware_->grasping_flag->bool_command()) {
        break;
      }
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandGrasp);

    // During gripping
    hardware_->grasping_flag->set_current(true);
    SpinOnce(rate);

    // Gripping complete
    hardware_->grasping_flag->set_current(false);
    SpinOnce(rate);

    auto grasp_goal_handle = future_grasp_goal_handle.get();
    EXPECT_TRUE(grasp_goal_handle.get());
    EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
      controller_, { client_node_ }, grasp_goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
  }

  void PreemptWithSetDistanceAction() {
    auto distance_calculator = std::make_shared<HrhGripperDistanceCalculator>();
    ASSERT_TRUE(distance_calculator->InitializeHandSizeData(controller_node_));

    // Temporarily assume the target position to reset the integral of the difference to 0
    hardware_->position->set_current(distance_calculator->GetPositionFromDistance(0.05));

    SetDistanceAction::Goal goal;
    goal.distance = 0.05;
    auto future_goal_handle = set_distance_action_client_->async_send_goal(goal);

    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    std::this_thread::sleep_for(std::chrono::milliseconds(1050));

    double current_position = distance_calculator->GetPositionFromDistance(0.052);
    hardware_->velocity->set_current(0.04);
    hardware_->position->set_current(current_position);
    SpinOnce(rate);
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);

    // Derived from gain and difference
    EXPECT_NEAR(hardware_->position->command(),
                (current_position + 2.0 * -0.002 + 0.0 * -0.002 + 2.5 * -0.002),
                kEpsilon);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    EXPECT_TRUE((WaitForStatus<SetDistanceAction, rclcpp::Node::SharedPtr>(
      controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
  }

  void PreemptWithFollowDistanceTrajectoryAction() {
    auto distance_calculator = std::make_shared<HrhGripperDistanceCalculator>();
    ASSERT_TRUE(distance_calculator->InitializeHandSizeData(controller_node_));

    // To prevent aborted due to excessive deviation during trajectory tracking, set a value close to the trajectory goal position
    hardware_->position->set_current(distance_calculator->GetPositionFromDistance(0.098));

    FollowTrajectoryAction::Goal follow_goal;
    follow_goal.trajectory = MakeDistanceTrajectory();

    auto future_goal_handle = follow_distance_trajectory_action_client_->async_send_goal(follow_goal);
    rclcpp::WallRate rate(100.0);
    WaitFor(rate, future_goal_handle);

    hardware_->position->set_current(distance_calculator->GetPositionFromDistance(0.1));

    // Test trajectory moves to 0.1 in 0.5 seconds
    for (int i = 0; i < 60; ++i) {
      SpinOnce(rate);
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
    EXPECT_NEAR(hardware_->position->command(), distance_calculator->GetPositionFromDistance(0.1), kEpsilon);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());
    EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
      controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));
  }

  void PreemptWithTrajectoryTopic() {
    // To prevent aborted due to excessive deviation during trajectory tracking, set a value close to the trajectory goal position
    hardware_->position->set_current(1.46);

    auto trajectory = MakeTrajectory();
    trajectory.points[0].positions[0] = 1.5;
    trajectory_publisher_->publish(trajectory);

    // Test trajectory moves to 1.5 in 0.5 seconds
    rclcpp::WallRate rate(100.0);
    for (int i = 0; i < 60; ++i) {
      SpinOnce(rate);
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
    EXPECT_DOUBLE_EQ(hardware_->position->command(), 1.5);
  }

  void PreemptWithDistanceTopic() {
    auto distance_calculator = std::make_shared<HrhGripperDistanceCalculator>();
    ASSERT_TRUE(distance_calculator->InitializeHandSizeData(controller_node_));

    double current_position = distance_calculator->GetPositionFromDistance(0.052);
    hardware_->position->set_current(current_position);

    std_msgs::msg::Float32 distance_topic;
    distance_topic.data = 0.05;
    distance_publisher_->publish(distance_topic);

    // Ensure the command is updated only once in the update function
    rclcpp::WallRate rate(100.0);
    SpinOnce(rate);

    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
    EXPECT_NEAR(hardware_->position->command(),
                (current_position + 2.0 * -0.002 + 0.0 * -0.002 + 2.5 * -0.002),
                kEpsilon);
  }

  void PreemptWithDistanceTrajectoryTopic() {
    auto distance_calculator = std::make_shared<HrhGripperDistanceCalculator>();
    ASSERT_TRUE(distance_calculator->InitializeHandSizeData(controller_node_));

    auto trajectory = MakeDistanceTrajectory();
    distance_trajectory_publisher_->publish(trajectory);

    // Test trajectory moves to 0.1 in 0.5 seconds
    rclcpp::WallRate rate(100.0);
    for (int i = 0; i < 60; ++i) {
      SpinOnce(rate);
    }
    EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
    EXPECT_NEAR(hardware_->position->command(), distance_calculator->GetPositionFromDistance(0.1), kEpsilon);
  }
};


TEST_F(GripperControllerTest, ApplyForceAction) {
  auto distance_calculator = std::make_shared<HrhGripperDistanceCalculator>();
  ASSERT_TRUE(distance_calculator->InitializeHandSizeData(controller_node_));

  // Check the values of controller_state and fingertip_distance before receiving the action
  rclcpp::WallRate rate(100.0);
  while ((state_subscriber_->count() == 0) ||
         (distance_subscriber_->count() == 0)) {
    SpinOnce(rate);
  }

  trajectory_msgs::msg::JointTrajectoryPoint start_reference;
  start_reference.positions = { 0.0 };
  start_reference.velocities = { 0.0 };
  start_reference.effort = { 0.0 };

  trajectory_msgs::msg::JointTrajectoryPoint start_error;
  start_error.positions = { 0.0 };
  start_error.velocities = { 0.0 };
  start_error.effort = { 0.0 };

  // Reference is set to the current value
  // Reference and feedback are set to the same value
  CheckStateMsg(state_subscriber_->last_msg(), start_reference, start_reference, start_error, { kHandJointName });

  EXPECT_NEAR(distance_subscriber_->last_msg().data, distance_calculator->GetDistanceFromPosition(0.0), kEpsilon);

  ApplayEffortAction::Goal goal;
  goal.effort = 1.0;
  goal.do_control_stop = false;

  auto future_goal_handle = apply_force_action_client_->async_send_goal(goal);

  WaitFor(rate, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  // Set so that the target force and the low-pass filtered force are the same value
  hardware_->spring_l_position->set_current(5.0);
  hardware_->spring_r_position->set_current(5.0);
  SpinOnce(rate);

  // Retrieve the values of controller_state and fingertip_distance again
  const uint32_t start_state_cnt = state_subscriber_->count();
  const uint32_t start_distance_cnt = distance_subscriber_->count();
  while ((state_subscriber_->count() <= start_state_cnt) ||
         (distance_subscriber_->count() <= start_distance_cnt)) {
    rclcpp::spin_some(client_node_);
  }

  // Should be able to subscribe three times with SpinOnce and WaitFor
  EXPECT_GE(state_subscriber_->count(), 3);
  EXPECT_GE(distance_subscriber_->count(), 3);

  // Reference position is the position command value, effort is the command torque
  trajectory_msgs::msg::JointTrajectoryPoint reference;
  reference.positions = { 0.4 * 1.0 };
  reference.effort = { 1.0 };

  // Feedback effort is the internally calculated value
  trajectory_msgs::msg::JointTrajectoryPoint feedback;
  feedback.positions = { 0.0 };
  feedback.velocities = { 0.0 };
  feedback.effort = { 1.0 };

  // Error is the difference between reference and feedback
  trajectory_msgs::msg::JointTrajectoryPoint error;
  error.positions = { 0.4 * 1.0 };
  error.effort = { 0.0 };

  CheckStateMsg(state_subscriber_->last_msg(), reference, feedback, error, { kHandJointName });

  EXPECT_NEAR(distance_subscriber_->last_msg().data, distance_calculator->GetDistanceFromPosition(0.0), kEpsilon);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  // Derived from gain and difference
  EXPECT_NEAR(hardware_->position->command(), 0.4 * 1.0, kEpsilon);

  EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
}

TEST_F(GripperControllerTest, FollowTrajectoryAction) {
  auto distance_calculator = std::make_shared<HrhGripperDistanceCalculator>();
  ASSERT_TRUE(distance_calculator->InitializeHandSizeData(controller_node_));

  // To prevent aborted due to excessive deviation during trajectory tracking, set a value close to the trajectory goal position
  hardware_->position->set_current(0.96);

  // Check the values of controller_state and fingertip_distance before receiving the action
  rclcpp::WallRate rate(100.0);
  while ((state_subscriber_->count() == 0) ||
         (distance_subscriber_->count() == 0)) {
    SpinOnce(rate);
  }

  trajectory_msgs::msg::JointTrajectoryPoint start_reference;
  start_reference.positions = { 0.96 };
  start_reference.velocities = { 0.0 };
  start_reference.effort = { 0.0 };

  trajectory_msgs::msg::JointTrajectoryPoint start_error;
  start_error.positions = { 0.0 };
  start_error.velocities = { 0.0 };
  start_error.effort = { 0.0 };

  // Reference is set to the current value
  // Reference and feedback are set to the same value
  CheckStateMsg(state_subscriber_->last_msg(), start_reference, start_reference, start_error, { kHandJointName });

  EXPECT_NEAR(distance_subscriber_->last_msg().data, distance_calculator->GetDistanceFromPosition(0.96), kEpsilon);

  FollowTrajectoryAction::Goal goal;
  goal.trajectory = MakeTrajectory();
  auto future_goal_handle = follow_trajectory_action_client_->async_send_goal(goal);

  WaitFor(rate, future_goal_handle);

  const uint32_t start_state_cnt = state_subscriber_->count();
  const uint32_t start_distance_cnt = distance_subscriber_->count();

  // Test trajectory moves to 1.0 in 0.5 seconds
  std::vector<double> command_positions;
  for (int i = 0; i < 60; ++i) {
    SpinOnce(rate);
    command_positions.push_back(hardware_->position->command());
  }

  // Retrieve the values of controller_state and fingertip_distance again
  while ((state_subscriber_->count() <= start_state_cnt) ||
         (distance_subscriber_->count() <= start_distance_cnt)) {
    rclcpp::spin_some(client_node_);
  }

  EXPECT_GE(state_subscriber_->count(), 30);
  EXPECT_GE(distance_subscriber_->count(), 30);

  // Reference position is the target position
  trajectory_msgs::msg::JointTrajectoryPoint reference;
  reference.positions = { 1.0 };
  reference.velocities = { 0.0 };
  reference.accelerations = { 0.0 };
  reference.effort = { 0.0 };

  // Feedback is the current value
  trajectory_msgs::msg::JointTrajectoryPoint feedback;
  feedback.positions = { 0.96 };
  feedback.velocities = { 0.0 };
  feedback.effort = { 0.0 };

  // Error is the difference between reference and feedback
  trajectory_msgs::msg::JointTrajectoryPoint error;
  error.positions = { 0.04 };
  error.velocities = { 0.0 };
  error.effort = { 0.0 };

  CheckStateMsg(state_subscriber_->last_msg(), reference, feedback, error, { kHandJointName });

  EXPECT_NEAR(distance_subscriber_->last_msg().data, distance_calculator->GetDistanceFromPosition(0.96), kEpsilon);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  double previous_command = 0.96;
  for (double command : command_positions) {
    EXPECT_LE(command, 1.0);
    EXPECT_GE(command, previous_command);
    previous_command = command;
  }
  EXPECT_LT(command_positions.front(), command_positions.back());

  EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
}

TEST_F(GripperControllerTest, GraspAction) {
  auto distance_calculator = std::make_shared<HrhGripperDistanceCalculator>();
  ASSERT_TRUE(distance_calculator->InitializeHandSizeData(controller_node_));

  // To ensure the action succeeds, set the current force close to the target force
  hardware_->effort->set_current(2.5);
  hardware_->grasping_flag->set_current(false);

  // Check the values of controller_state and fingertip_distance before receiving the action
  rclcpp::WallRate rate(100.0);
  while ((state_subscriber_->count() == 0) ||
         (distance_subscriber_->count() == 0)) {
    SpinOnce(rate);
  }

  trajectory_msgs::msg::JointTrajectoryPoint start_reference;
  start_reference.positions = { 0.0 };
  start_reference.velocities = { 0.0 };
  start_reference.effort = { 2.5 };

  trajectory_msgs::msg::JointTrajectoryPoint start_error;
  start_error.positions = { 0.0 };
  start_error.velocities = { 0.0 };
  start_error.effort = { 0.0 };

  // Reference is set to the current value
  // Reference and feedback are set to the same value
  CheckStateMsg(state_subscriber_->last_msg(), start_reference, start_reference, start_error, { kHandJointName });

  EXPECT_NEAR(distance_subscriber_->last_msg().data, distance_calculator->GetDistanceFromPosition(0.0), kEpsilon);

  ApplayEffortAction::Goal goal;
  goal.effort = 3.0;
  auto future_goal_handle = grasp_action_client_->async_send_goal(goal);

  WaitFor(rate, future_goal_handle);

  while (rclcpp::ok()) {
    SpinOnce(rate);
    // Break on gripping start
    if (hardware_->grasping_flag->bool_command()) {
      EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);
      break;
    }
  }

  // During gripping
  hardware_->grasping_flag->set_current(true);
  SpinOnce(rate);

  EXPECT_FALSE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  // Gripping complete
  hardware_->grasping_flag->set_current(false);

  // Retrieve the values of controller_state and fingertip_distance again
  const uint32_t start_state_cnt = state_subscriber_->count();
  const uint32_t start_distance_cnt = distance_subscriber_->count();
  while ((state_subscriber_->count() <= start_state_cnt) ||
         (distance_subscriber_->count() <= start_distance_cnt)) {
    SpinOnce(rate);
  }

  // Should be able to subscribe three times with SpinOnce and WaitFor
  EXPECT_GE(state_subscriber_->count(), 3);
  EXPECT_GE(distance_subscriber_->count(), 3);

  // Reference position is the current position, effort is the command torque
  trajectory_msgs::msg::JointTrajectoryPoint reference;
  reference.positions = { 0.0 };
  reference.effort = { 3.0 };

  // Feedback is the current value
  trajectory_msgs::msg::JointTrajectoryPoint feedback;
  feedback.positions = { 0.0 };
  feedback.velocities = { 0.0 };
  feedback.effort = { 2.5 };

  // Error is the difference between reference and feedback
  trajectory_msgs::msg::JointTrajectoryPoint error;
  error.positions = { 0.0 };
  error.effort = { 0.5 };

  CheckStateMsg(state_subscriber_->last_msg(), reference, feedback, error, { kHandJointName });

  EXPECT_NEAR(distance_subscriber_->last_msg().data, distance_calculator->GetDistanceFromPosition(0.0), kEpsilon);

  EXPECT_FALSE(hardware_->grasping_flag->bool_command());
  EXPECT_DOUBLE_EQ(hardware_->effort->command(), 3.0);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandGrasp);
}

TEST_F(GripperControllerTest, SetDistanceAction) {
  auto distance_calculator = std::make_shared<HrhGripperDistanceCalculator>();
  ASSERT_TRUE(distance_calculator->InitializeHandSizeData(controller_node_));

  // Temporarily assume the target position to reset the integral of the difference to 0
  double start_position = distance_calculator->GetPositionFromDistance(0.05);
  hardware_->position->set_current(start_position);

  // Check the values of controller_state and fingertip_distance before receiving the action
  rclcpp::WallRate rate(100.0);
  while ((state_subscriber_->count() == 0) ||
         (distance_subscriber_->count() == 0)) {
    SpinOnce(rate);
  }

  trajectory_msgs::msg::JointTrajectoryPoint start_reference;
  start_reference.positions = { start_position };
  start_reference.velocities = { 0.0 };
  start_reference.effort = { 0.0 };

  trajectory_msgs::msg::JointTrajectoryPoint start_error;
  start_error.positions = { 0.0 };
  start_error.velocities = { 0.0 };
  start_error.effort = { 0.0 };

  // Reference is set to the current value
  // Reference and feedback are set to the same value
  CheckStateMsg(state_subscriber_->last_msg(), start_reference, start_reference, start_error, { kHandJointName });

  EXPECT_NEAR(distance_subscriber_->last_msg().data, 0.05, kEpsilon);

  SetDistanceAction::Goal goal;
  goal.distance = 0.05;

  auto future_goal_handle = set_distance_action_client_->async_send_goal(goal);

  WaitFor(rate, future_goal_handle);

  std::this_thread::sleep_for(std::chrono::milliseconds(1350));

  double current_position = distance_calculator->GetPositionFromDistance(0.052);
  hardware_->velocity->set_current(0.04);
  hardware_->position->set_current(current_position);
  SpinOnce(rate);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<SetDistanceAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  // Derived from gain and difference
  double command_value = current_position + 2.0 * -0.002 + 0.0 * -0.002 + 2.5 * -0.002;
  EXPECT_NEAR(hardware_->position->command(), command_value, kEpsilon);

  EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);

  // Retrieve the values of controller_state and fingertip_distance again
  const uint32_t start_state_cnt = state_subscriber_->count();
  const uint32_t start_distance_cnt = distance_subscriber_->count();
  while ((state_subscriber_->count() <= start_state_cnt) ||
         (distance_subscriber_->count() <= start_distance_cnt)) {
    rclcpp::spin_some(client_node_);
  }

  // Should be able to subscribe three times with SpinOnce and WaitFor
  EXPECT_GE(state_subscriber_->count(), 3);
  EXPECT_GE(distance_subscriber_->count(), 3);

  // Reference position is the current position, effort is the command torque
  trajectory_msgs::msg::JointTrajectoryPoint reference;
  reference.positions = { command_value };

  // Feedback is the current value
  trajectory_msgs::msg::JointTrajectoryPoint feedback;
  feedback.positions = { current_position };
  feedback.velocities = { 0.04 };
  feedback.effort = { 0.0 };

  // Error is the difference between reference and feedback
  trajectory_msgs::msg::JointTrajectoryPoint error;
  error.positions = { command_value - current_position };

  CheckStateMsg(state_subscriber_->last_msg(), reference, feedback, error, { kHandJointName });

  EXPECT_NEAR(distance_subscriber_->last_msg().data, 0.052, kEpsilon);
}

TEST_F(GripperControllerTest, FollowDistanceTrajectoryAction) {
  auto distance_calculator = std::make_shared<HrhGripperDistanceCalculator>();
  ASSERT_TRUE(distance_calculator->InitializeHandSizeData(controller_node_));

  double start_position = distance_calculator->GetPositionFromDistance(0.05);
  hardware_->position->set_current(start_position);

  // Check the values of controller_state and fingertip_distance before receiving the action
  rclcpp::WallRate rate(100.0);
  while ((state_subscriber_->count() == 0) ||
         (distance_subscriber_->count() == 0)) {
    SpinOnce(rate);
  }

  trajectory_msgs::msg::JointTrajectoryPoint start_reference;
  start_reference.positions = { start_position };
  start_reference.velocities = { 0.0 };
  start_reference.effort = { 0.0 };

  trajectory_msgs::msg::JointTrajectoryPoint start_error;
  start_error.positions = { 0.0 };
  start_error.velocities = { 0.0 };
  start_error.effort = { 0.0 };

  // Reference is set to the current value
  // Reference and feedback are set to the same value
  CheckStateMsg(state_subscriber_->last_msg(), start_reference, start_reference, start_error, { kHandJointName });

  EXPECT_NEAR(distance_subscriber_->last_msg().data, 0.05, kEpsilon);

  FollowTrajectoryAction::Goal goal;
  goal.trajectory = MakeDistanceTrajectory();
  auto future_goal_handle = follow_distance_trajectory_action_client_->async_send_goal(goal);

  WaitFor(rate, future_goal_handle);

  hardware_->position->set_current(distance_calculator->GetPositionFromDistance(0.1));

  const uint32_t start_state_cnt = state_subscriber_->count();
  const uint32_t start_distance_cnt = distance_subscriber_->count();

  // Test trajectory moves to 0.1 in 0.5 seconds
  std::vector<double> command_distances;
  for (int i = 0; i < 60; ++i) {
    SpinOnce(rate);
    command_distances.push_back(distance_calculator->GetDistanceFromPosition(hardware_->position->command()));
  }

  // Retrieve the values of controller_state and fingertip_distance again
  while ((state_subscriber_->count() <= start_state_cnt) ||
         (distance_subscriber_->count() <= start_distance_cnt)) {
    rclcpp::spin_some(client_node_);
  }

  // Reference position is the current position, effort is the command torque
  trajectory_msgs::msg::JointTrajectoryPoint reference;
  reference.positions = { distance_calculator->GetPositionFromDistance(0.1) };

  // Feedback is the current value
  trajectory_msgs::msg::JointTrajectoryPoint feedback;
  feedback.positions = { distance_calculator->GetPositionFromDistance(0.1) };
  feedback.velocities = { 0.0 };
  feedback.effort = { 0.0 };

  // Error is the difference between reference and feedback
  trajectory_msgs::msg::JointTrajectoryPoint error;
  error.positions = { 0.0 };

  CheckStateMsg(state_subscriber_->last_msg(), reference, feedback, error, { kHandJointName });

  EXPECT_NEAR(distance_subscriber_->last_msg().data, 0.1, kEpsilon);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());
  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)));

  double previous_command = 0.05;
  for (double command : command_distances) {
    EXPECT_LE(command, (0.1 + kEpsilon));
    EXPECT_GE(command, previous_command);
    previous_command = command;
  }
  EXPECT_LT(command_distances.front(), command_distances.back());

  EXPECT_DOUBLE_EQ(hardware_->drive_mode->command(), tmc_exxx_servo_motor_protocol::kDriveModeHandPosition);
}

TEST_F(GripperControllerTest, ApplyForceAndFollowTrajectory) {
  auto goal_handle = StartApplyForceAction();
  PreemptWithFollowTrajectoryAction();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, ApplyForceAndGrasp) {
  auto goal_handle = StartApplyForceAction();
  PreemptWithGraspAction();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, ApplyForceAndSetDistance) {
  auto goal_handle = StartApplyForceAction();
  PreemptWithSetDistanceAction();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, ApplyForceAndFollowDistanceTrajectory) {
  auto goal_handle = StartApplyForceAction();
  PreemptWithFollowDistanceTrajectoryAction();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, ApplyForceAndSendTrajectoryTopic) {
  auto goal_handle = StartApplyForceAction();
  PreemptWithTrajectoryTopic();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, ApplyForceAndSendDistanceTopic) {
  auto goal_handle = StartApplyForceAction();
  PreemptWithDistanceTopic();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, ApplyForceAndSendDistanceTrajectoryTopic) {
  auto goal_handle = StartApplyForceAction();
  PreemptWithDistanceTrajectoryTopic();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowTrajectoryAndApplyForce) {
  auto goal_handle = StartFollowTrajectoryAction();
  PreemptWithApplyForceAction();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowTrajectoryAndGrasp) {
  auto goal_handle = StartFollowTrajectoryAction();
  PreemptWithGraspAction();

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowTrajectoryAndSetDistance) {
  auto goal_handle = StartFollowTrajectoryAction();
  PreemptWithSetDistanceAction();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowTrajectoryAndFollowDistanceTrajectory) {
  auto goal_handle = StartFollowTrajectoryAction();
  PreemptWithFollowDistanceTrajectoryAction();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowTrajectoryAndSendTrajectoryTopic) {
  auto goal_handle = StartFollowTrajectoryAction();
  PreemptWithTrajectoryTopic();

  std::this_thread::sleep_for(std::chrono::milliseconds(2050));

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowTrajectoryAndSendDistanceTopic) {
  auto goal_handle = StartFollowTrajectoryAction();
  PreemptWithDistanceTopic();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowTrajectoryAndSendDistanceTrajectoryTopic) {
  auto goal_handle = StartFollowTrajectoryAction();
  PreemptWithDistanceTrajectoryTopic();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, GraspAndApplyForce) {
  auto goal_handle = StartGraspAction();
  PreemptWithApplyForceAction();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, GraspAndFollowTrajectory) {
  auto goal_handle = StartGraspAction();
  PreemptWithFollowTrajectoryAction();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, GraspAndSetDistance) {
  auto goal_handle = StartGraspAction();
  PreemptWithSetDistanceAction();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, GraspAndFollowDistanceTrajectory) {
  auto goal_handle = StartGraspAction();
  PreemptWithFollowDistanceTrajectoryAction();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, GraspAndSendTrajectoryTopic) {
  auto goal_handle = StartGraspAction();
  PreemptWithTrajectoryTopic();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, GraspAndSendDistanceTopic) {
  auto goal_handle = StartGraspAction();
  PreemptWithDistanceTopic();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, GraspAndSendDistanceTrajectoryTopic) {
  auto goal_handle = StartGraspAction();
  PreemptWithDistanceTrajectoryTopic();

  EXPECT_TRUE((WaitForStatus<ApplayEffortAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, SetDistanceAndApplyForce) {
  auto goal_handle = StartSetDistanceAction();
  PreemptWithApplyForceAction();

  EXPECT_TRUE((WaitForStatus<SetDistanceAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, SetDistanceAndFollowTrajectory) {
  auto goal_handle = StartSetDistanceAction();
  PreemptWithFollowTrajectoryAction();

  EXPECT_TRUE((WaitForStatus<SetDistanceAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, SetDistanceAndGrasp) {
  auto goal_handle = StartSetDistanceAction();
  PreemptWithGraspAction();

  EXPECT_TRUE((WaitForStatus<SetDistanceAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, SetDistanceAndFollowDistanceTrajectory) {
  auto goal_handle = StartSetDistanceAction();
  PreemptWithFollowDistanceTrajectoryAction();

  EXPECT_TRUE((WaitForStatus<SetDistanceAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, SetDistanceAndSendTrajectoryTopic) {
  auto goal_handle = StartSetDistanceAction();
  PreemptWithTrajectoryTopic();

  EXPECT_TRUE((WaitForStatus<SetDistanceAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, SetDistanceAndSendDistanceTopic) {
  auto goal_handle = StartSetDistanceAction();
  PreemptWithDistanceTopic();

  EXPECT_TRUE((WaitForStatus<SetDistanceAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, SetDistanceAndSendDistanceTrajectoryTopic) {
  auto goal_handle = StartSetDistanceAction();
  PreemptWithDistanceTrajectoryTopic();

  EXPECT_TRUE((WaitForStatus<SetDistanceAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowDistanceTrajectoryAndApplyForce) {
  auto goal_handle = StartFollowDistanceTrajectoryAction();
  PreemptWithApplyForceAction();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowDistanceTrajectoryAndFollowTrajectory) {
  auto goal_handle = StartFollowDistanceTrajectoryAction();
  PreemptWithFollowTrajectoryAction();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowDistanceTrajectoryAndGrasp) {
  auto goal_handle = StartFollowDistanceTrajectoryAction();
  PreemptWithGraspAction();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowDistanceTrajectoryAndSetDistance) {
  auto goal_handle = StartFollowDistanceTrajectoryAction();
  PreemptWithSetDistanceAction();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowDistanceTrajectoryAndSendTrajectoryTopic) {
  auto goal_handle = StartFollowDistanceTrajectoryAction();
  PreemptWithTrajectoryTopic();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowDistanceTrajectoryAndSendDistanceTopic) {
  auto goal_handle = StartFollowDistanceTrajectoryAction();
  PreemptWithDistanceTopic();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, FollowDistanceTrajectoryAndSendDistanceTrajectoryTopic) {
  auto goal_handle = StartFollowDistanceTrajectoryAction();
  PreemptWithDistanceTrajectoryTopic();

  EXPECT_TRUE((WaitForStatus<FollowTrajectoryAction, rclcpp::Node::SharedPtr>(
    controller_, { client_node_ }, goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED)));
}

TEST_F(GripperControllerTest, SendTrajectoryTopicAndApplyForce) {
  SendTrajectoryTopic();
  PreemptWithApplyForceAction();
}

TEST_F(GripperControllerTest, SendTrajectoryTopicAndFollowTrajectory) {
  SendTrajectoryTopic();
  PreemptWithFollowTrajectoryAction();
}

TEST_F(GripperControllerTest, SendTrajectoryTopicAndGrasp) {
  SendTrajectoryTopic();
  PreemptWithGraspAction();
}

TEST_F(GripperControllerTest, SendTrajectoryTopicAndSetDistance) {
  SendTrajectoryTopic();
  PreemptWithSetDistanceAction();
}

TEST_F(GripperControllerTest, SendTrajectoryTopicAndFollowDistanceTrajectory) {
  SendTrajectoryTopic();
  PreemptWithFollowDistanceTrajectoryAction();
}

TEST_F(GripperControllerTest, SendTrajectoryTopicAndSendDistanceTopic) {
  SendTrajectoryTopic();
  PreemptWithDistanceTopic();
}

TEST_F(GripperControllerTest, SendTrajectoryTopicAndSendDistanceTrajectoryTopic) {
  SendTrajectoryTopic();
  PreemptWithDistanceTrajectoryTopic();
}

TEST_F(GripperControllerTest, SendDistanceTopicAndApplyForce) {
  SendDistanceTopic();
  PreemptWithApplyForceAction();
}

TEST_F(GripperControllerTest, SendDistanceTopicAndFollowTrajectory) {
  SendDistanceTopic();
  PreemptWithFollowTrajectoryAction();
}

TEST_F(GripperControllerTest, SendDistanceTopicAndGrasp) {
  SendDistanceTopic();
  PreemptWithGraspAction();
}

TEST_F(GripperControllerTest, SendDistanceTopicAndSetDistance) {
  SendDistanceTopic();
  PreemptWithSetDistanceAction();
}

TEST_F(GripperControllerTest, SendDistanceTopicAndFollowDistanceTrajectory) {
  SendDistanceTopic();
  PreemptWithFollowDistanceTrajectoryAction();
}

TEST_F(GripperControllerTest, SendDistanceTopicAndSendTrajectoryTopic) {
  SendDistanceTopic();
  PreemptWithTrajectoryTopic();
}

TEST_F(GripperControllerTest, SendDistanceTopicAndSendDistanceTrajectoryTopic) {
  SendDistanceTopic();
  PreemptWithDistanceTrajectoryTopic();
}

TEST_F(GripperControllerTest, SendDistanceTrajectoryTopicAndApplyForce) {
  SendDistanceTrajectoryTopic();
  PreemptWithApplyForceAction();
}

TEST_F(GripperControllerTest, SendDistanceTrajectoryTopicAndFollowTrajectory) {
  SendDistanceTrajectoryTopic();
  PreemptWithFollowTrajectoryAction();
}

TEST_F(GripperControllerTest, SendDistanceTrajectoryTopicAndGrasp) {
  SendDistanceTrajectoryTopic();
  PreemptWithGraspAction();
}

TEST_F(GripperControllerTest, SendDistanceTrajectoryTopicAndSetDistance) {
  SendDistanceTrajectoryTopic();
  PreemptWithSetDistanceAction();
}

TEST_F(GripperControllerTest, SendDistanceTrajectoryTopicAndFollowDistanceTrajectory) {
  SendDistanceTrajectoryTopic();
  PreemptWithFollowDistanceTrajectoryAction();
}

TEST_F(GripperControllerTest, SendDistanceTrajectoryTopicAndSendTrajectoryTopic) {
  SendDistanceTrajectoryTopic();
  PreemptWithTrajectoryTopic();
}

TEST_F(GripperControllerTest, SendDistanceTrajectoryTopicAndSendDistanceTopic) {
  SendDistanceTrajectoryTopic();
  PreemptWithDistanceTopic();
}
}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
