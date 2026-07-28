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
/// @file omni_base_controller-test.cpp
/// @brief Test for omnidirectional cart speed controller

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <hsrb_base_controllers/omni_base_controller.hpp>

#include "hardware_stub.hpp"
#include "utils.hpp"

namespace {

const char* const kControllerNodeName = "controller_manager";
const char* const kClientNodeName = "test_node";
constexpr double kEpsilon = 0.02;
constexpr double kUpdateFrequency = 100.0;
constexpr double kPositionErrorThreshold = 0.05;
constexpr double kVelocityErrorThreshold = 0.1;
constexpr double kWheelVelocityLimitThreshold = 8.5;
constexpr double kYawVelocityLimitThreshold = 1.8;

// Create test input trajectory
trajectory_msgs::msg::JointTrajectory GetTestOdomTrajectory() {
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"odom_x", "odom_y", "odom_t"};

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.1, 0.1, 0.3};
  point.velocities = {0.0, 0.0, 0.0};
  point.time_from_start = rclcpp::Duration(1, 0);
  trajectory.points.push_back(point);

  point.positions = {0.0, 0.0, 0.0};
  point.time_from_start = rclcpp::Duration(2, 0);
  trajectory.points.push_back(point);

  return trajectory;
}

trajectory_msgs::msg::JointTrajectory GetTestRollTrajectory() {
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = { "base_roll_joint" };

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { 1.0 };
  point.time_from_start = rclcpp::Duration(1, 0);
  trajectory.points.push_back(point);

  return trajectory;
}

rclcpp::NodeOptions GetTestNodeOptions(bool open_loop_control = false,
                                       bool use_base_roll_velocity = false) {
  auto options = rclcpp::NodeOptions()
      .allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  options.append_parameter_override<std::vector<std::string>>("base_coordinates", {"odom_x", "odom_y", "odom_t"});
  options.append_parameter_override<std::string>("robot_description", hsrb_base_controllers::GetRobotDescription());
  options.append_parameter_override<float>("odometry_publish_rate", 200.0);
  options.append_parameter_override<std::string>("joints.steer", "base_roll_joint");
  options.append_parameter_override<std::string>("joints.r_wheel", "base_r_drive_wheel_joint");
  options.append_parameter_override<std::string>("joints.l_wheel", "base_l_drive_wheel_joint");
  options.append_parameter_override<float>("odom_x.p_gain", 0.01);
  options.append_parameter_override<float>("odom_y.p_gain", 0.01);
  options.append_parameter_override<float>("odom_t.p_gain", 0.01);
  options.append_parameter_override<float>("constraints.odom_x.trajectory", 0.5);
  options.append_parameter_override<float>("constraints.goal_time", 0.5);
  options.append_parameter_override<bool>("open_loop_control", open_loop_control);
  options.append_parameter_override<bool>("use_base_roll_velocity", use_base_roll_velocity);
  return options;
}

}  // namespace

namespace hsrb_base_controllers {

TEST(OmniBaseControllerInitializationTest, SubscribeVelocity) {
  std::atomic_bool running = true;

  std::thread publish_thread([&running]() {
    auto client_node = rclcpp::Node::make_shared(kClientNodeName);
    auto publisher = client_node->create_publisher<geometry_msgs::msg::Twist>(
        std::string(kControllerNodeName) + "/cmd_vel", rclcpp::SystemDefaultsQoS());
    const auto msg = geometry_msgs::msg::Twist();
    while (running) {
      publisher->publish(msg);
      rclcpp::spin_some(client_node);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
  });

  auto controller = std::make_shared<OmniBaseController>();
  EXPECT_EQ(controller->init(kControllerNodeName, "", 0, "", GetTestNodeOptions()),
            controller_interface::return_type::OK);

  std::thread spin_thread([&controller, &running]() {
    while (running) {
      rclcpp::spin_some(controller->get_node()->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
  });

  ASSERT_NO_THROW(controller->configure());

  running = false;
  publish_thread.join();
  spin_thread.join();
}

using ActionType = control_msgs::action::FollowJointTrajectory;

struct PositionHandle {
  using CommandHandleType = CommandPositionHandle;
  static constexpr bool use_base_roll_velocity = false;
};
struct VelocityHandle {
  using CommandHandleType = CommandVelocityHandle;
  static constexpr bool use_base_roll_velocity = true;
};

template<typename CommandHandleType>
class OmniBaseControllerTest : public ::testing::Test {
 public:
  void SetupController(bool open_loop_control = false);

  void TearDown() override {
    controller_->release_interfaces();
  }

 protected:
  std::shared_ptr<OmniBaseController> controller_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr controller_node_;
  rclcpp::Node::SharedPtr client_node_;
  typename HardwareStub<typename CommandHandleType::CommandHandleType>::Ptr hardware_;
  TopicRelay<nav_msgs::msg::Odometry>::Ptr odom_relay_;
  SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>::Ptr state_counter_;
  SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>::Ptr joint_state_counter_;
  SubscriptionCounter<nav_msgs::msg::Odometry>::Ptr wheel_odom_counter_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr odom_trajectory_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr roll_trajectory_publisher_;

  rclcpp_action::Client<ActionType>::SharedPtr odom_action_client_;
  rclcpp_action::Client<ActionType>::SharedPtr roll_action_client_;

  void SpinOnce(rclcpp::WallRate& rate, bool do_update = true);

  template <typename Action>
  bool WaitForStatus(typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle,
                     int8_t expected, bool do_update = true);

  template <typename Handle>
  void WaitForReady(typename std::shared_future<Handle>& future, rclcpp::WallRate& rate);

  void StartCmdVelTopic();
  void StartOdomTrajectoryTopic();
  void StartRollTrajectoryTopic();
  std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionType>> StartOdomTrajectoryAction();
  std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionType>> StartRollTrajectoryAction();

  void PreemptCmdVelTopic();
  void PreemptOdomTrajectoryAction();
  void PreemptOdomTrajectoryTopic();
  void PreemptRollTrajectoryAction();
  void PreemptRollTrajectoryTopic();
};

template<typename CommandHandleType>
void OmniBaseControllerTest<CommandHandleType>::SetupController(bool open_loop_control) {
  controller_ = std::make_shared<OmniBaseController>();

  const auto options = GetTestNodeOptions(open_loop_control, CommandHandleType::use_base_roll_velocity);
  EXPECT_EQ(controller_->init(kControllerNodeName, "", 0, "", options), controller_interface::return_type::OK);
  EXPECT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  hardware_ = std::make_shared<HardwareStub<typename CommandHandleType::CommandHandleType>>(kUpdateFrequency);
  controller_->assign_interfaces(std::move(hardware_->command_interfaces), std::move(hardware_->state_interfaces));
  EXPECT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  client_node_ = rclcpp::Node::make_shared(kClientNodeName);
  odom_relay_ = std::make_shared<TopicRelay<nav_msgs::msg::Odometry>>(
      client_node_, std::string(kControllerNodeName) + "/wheel_odom", "odom");
  state_counter_ = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
      client_node_, std::string(kControllerNodeName) + "/state");
  wheel_odom_counter_ = std::make_shared<SubscriptionCounter<nav_msgs::msg::Odometry>>(
      client_node_, std::string(kControllerNodeName) + "/wheel_odom");
  joint_state_counter_ = std::make_shared<SubscriptionCounter<control_msgs::msg::JointTrajectoryControllerState>>(
      client_node_, std::string(kControllerNodeName) + "/internal_state");

  cmd_vel_publisher_ = client_node_->create_publisher<geometry_msgs::msg::Twist>(
      std::string(kControllerNodeName) + "/cmd_vel", rclcpp::SystemDefaultsQoS());
  odom_trajectory_publisher_ = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      std::string(kControllerNodeName) + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  roll_trajectory_publisher_ = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      std::string(kControllerNodeName) + "/roll_joint_trajectory", rclcpp::SystemDefaultsQoS());

  odom_action_client_ = rclcpp_action::create_client<ActionType>(
      client_node_, std::string(kControllerNodeName) + "/follow_joint_trajectory");
  roll_action_client_ = rclcpp_action::create_client<ActionType>(
      client_node_, std::string(kControllerNodeName) + "/follow_roll_joint_trajectory");

  EXPECT_TRUE(odom_action_client_->wait_for_action_server());
  EXPECT_TRUE(roll_action_client_->wait_for_action_server());

  controller_node_ = controller_->get_node();
}

template<typename CommandHandleType>
void OmniBaseControllerTest<CommandHandleType>::SpinOnce(rclcpp::WallRate& rate, bool do_update) {
  rate.sleep();
  rclcpp::spin_some(client_node_);
  rclcpp::spin_some(controller_node_->get_node_base_interface());
  // Fixed value due to test failures caused by period value
  EXPECT_EQ(
      controller_->update(controller_node_->get_clock()->now(), std::chrono::duration<double>(1.0 / kUpdateFrequency)),
      controller_interface::return_type::OK);
  rclcpp::spin_some(client_node_);
  rclcpp::spin_some(controller_node_->get_node_base_interface());
  if (do_update) hardware_->Update();
}

template<typename CommandHandleType>
template <typename Action>
bool OmniBaseControllerTest<CommandHandleType>::WaitForStatus(
    typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle,
    int8_t expected, bool do_update) {
  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(5.0);
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  while (goal_handle->get_status() != expected) {
    SpinOnce(loop_rate, do_update);
    if (std::chrono::system_clock::now() > end_time) {
      std::cout << "The last status is " << static_cast<int32_t>(goal_handle->get_status()) << std::endl;
      return false;
    }
  }
  return true;
}

template<typename CommandHandleType>
template <typename Handle>
void OmniBaseControllerTest<CommandHandleType>::WaitForReady(
    typename std::shared_future<Handle>& future, rclcpp::WallRate& rate) {
  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(5.0);
  while (rclcpp::ok()) {
    if (std::chrono::system_clock::now() > end_time) {
      FAIL();
    }
    const auto result = future.wait_for(std::chrono::milliseconds(1));
    if (result == std::future_status::ready) {
      return;
    } else {
      SpinOnce(rate);
    }
  }
}

template<typename CommandHandleType>
void OmniBaseControllerTest<CommandHandleType>::StartCmdVelTopic() {
  geometry_msgs::msg::Twist command_velocity;
  command_velocity.linear.x = -0.1;
  command_velocity.linear.y = -0.05;

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  while (rclcpp::ok()) {
    cmd_vel_publisher_->publish(command_velocity);
    SpinOnce(loop_rate);

    if ((std::abs(hardware_->l_wheel_handle->command()) > 0.01)
     || (std::abs(hardware_->r_wheel_handle->command()) > 0.01)
     || (std::abs(hardware_->steer_handle->command()) > 0.01)) {
      break;
    }
  }
}

template<typename CommandHandleType>
void OmniBaseControllerTest<CommandHandleType>::StartOdomTrajectoryTopic() {
  auto trajectory = GetTestOdomTrajectory();

  // No need for multiple points, so remove one
  trajectory.points.pop_back();
  odom_trajectory_publisher_->publish(trajectory);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  while (rclcpp::ok()) {
    SpinOnce(loop_rate);

    if ((std::abs(hardware_->l_wheel_handle->command()) > 0.01)
     || (std::abs(hardware_->r_wheel_handle->command()) > 0.01)
     || (std::abs(hardware_->steer_handle->command()) > 0.01)) {
      break;
    }
  }
}

template<typename CommandHandleType>
void OmniBaseControllerTest<CommandHandleType>::StartRollTrajectoryTopic() {
  roll_trajectory_publisher_->publish(GetTestRollTrajectory());

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  while (rclcpp::ok()) {
    SpinOnce(loop_rate);

    if (std::abs(hardware_->steer_handle->command()) > 0.0) {
      break;
    }
  }
}

template<typename CommandHandleType>
std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionType>>
    OmniBaseControllerTest<CommandHandleType>::StartOdomTrajectoryAction() {
  ActionType::Goal odom_goal;
  odom_goal.trajectory = GetTestOdomTrajectory();

  // No need for multiple points, so remove one
  odom_goal.trajectory.points.pop_back();

  auto future_goal_handle = odom_action_client_->async_send_goal(odom_goal);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  WaitForReady(future_goal_handle, loop_rate);

  while (rclcpp::ok()) {
    SpinOnce(loop_rate);

    if ((std::abs(hardware_->l_wheel_handle->command()) > 0.01)
     || (std::abs(hardware_->r_wheel_handle->command()) > 0.01)
     || (std::abs(hardware_->steer_handle->command()) > 0.01)) {
      break;
    }
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle);
  return goal_handle;
}

template<typename CommandHandleType>
std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionType>>
    OmniBaseControllerTest<CommandHandleType>::StartRollTrajectoryAction() {
  ActionType::Goal goal;
  goal.trajectory = GetTestRollTrajectory();

  auto future_goal_handle = roll_action_client_->async_send_goal(goal);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  WaitForReady(future_goal_handle, loop_rate);

  while (rclcpp::ok()) {
    SpinOnce(loop_rate);

    if (std::abs(hardware_->steer_handle->command()) > 0.0) {
      break;
    }
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle);
  return goal_handle;
}

template<typename CommandHandleType>
void OmniBaseControllerTest<CommandHandleType>::PreemptCmdVelTopic() {
  geometry_msgs::msg::Twist command_velocity;
  command_velocity.linear.x = -0.1;
  command_velocity.linear.y = -0.1;

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 50; ++i) {
    cmd_vel_publisher_->publish(command_velocity);
    SpinOnce(loop_rate);
  }

  // Cannot interrupt with cmd_vel topic, so it won't reach the specified speed
  auto base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.velocities.size(), 3);
  EXPECT_GT(std::abs(-0.1 - base_state.feedback.velocities[0]), kEpsilon);
  EXPECT_GT(std::abs(-0.1 - base_state.feedback.velocities[1]), kEpsilon);

  // Since the trajectory is set to 1 second, keep it running for 1 second
  for (int i = 0; i < 50; ++i) {
    SpinOnce(loop_rate);
  }
}

template<typename CommandHandleType>
void OmniBaseControllerTest<CommandHandleType>::PreemptOdomTrajectoryAction() {
  ActionType::Goal roll_goal;
  roll_goal.trajectory = GetTestOdomTrajectory();
  roll_goal.trajectory.points.clear();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.05, 0.05, 0.1};
  point.velocities = {0.0, 0.0, 0.0};
  point.time_from_start = rclcpp::Duration(1, 0);
  roll_goal.trajectory.points.push_back(point);

  auto future_goal_handle = odom_action_client_->async_send_goal(roll_goal);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  WaitForReady(future_goal_handle, loop_rate);

  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);
  }

  auto base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.05, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.05, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.1, kEpsilon);

  auto odom_goal_handle = future_goal_handle.get();
  EXPECT_TRUE(odom_goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>(
      odom_goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

template<typename CommandHandleType>
void OmniBaseControllerTest<CommandHandleType>::PreemptOdomTrajectoryTopic() {
  auto topic_msg = GetTestOdomTrajectory();
  topic_msg.points.clear();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.05, 0.05, 0.1};
  point.velocities = {0.0, 0.0, 0.0};
  point.time_from_start = rclcpp::Duration(1, 0);
  topic_msg.points.push_back(point);

  odom_trajectory_publisher_->publish(topic_msg);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 100; ++i) {
    SpinOnce(loop_rate);
  }

  auto base_state = state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.05, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.05, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.1, kEpsilon);
}

template<typename CommandHandleType>
void OmniBaseControllerTest<CommandHandleType>::PreemptRollTrajectoryAction() {
  ActionType::Goal roll_goal;
  roll_goal.trajectory = GetTestRollTrajectory();
  roll_goal.trajectory.points.clear();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { 1.5 };
  point.time_from_start = rclcpp::Duration(1, 0);
  roll_goal.trajectory.points.push_back(point);

  auto future_goal_handle = roll_action_client_->async_send_goal(roll_goal);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  WaitForReady(future_goal_handle, loop_rate);

  // Slightly increase loop count as state reception failed with exact loop count
  for (int i = 0; i < 110; ++i) {
    SpinOnce(loop_rate);
  }

  // Verify only the rotation axis
  auto joint_state = joint_state_counter_->last_msg();
  ASSERT_EQ(joint_state.feedback.positions.size(), 3);
  EXPECT_NEAR(joint_state.feedback.positions[2], 1.5, 0.05);

  auto roll_goal_handle = future_goal_handle.get();
  EXPECT_TRUE(roll_goal_handle.get());
  EXPECT_TRUE(WaitForStatus<ActionType>(
      roll_goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

template<typename CommandHandleType>
void OmniBaseControllerTest<CommandHandleType>::PreemptRollTrajectoryTopic() {
  auto topic_msg = GetTestRollTrajectory();
  topic_msg.points.clear();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = { 1.5 };
  point.time_from_start = rclcpp::Duration(1, 0);
  topic_msg.points.push_back(point);

  roll_trajectory_publisher_->publish(topic_msg);

  // Slightly increase loop count as state reception failed with exact loop count
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 110; ++i) {
    SpinOnce(loop_rate);
  }

  // Verify only the rotation axis
  auto joint_state = joint_state_counter_->last_msg();
  ASSERT_EQ(joint_state.feedback.positions.size(), 3);
  EXPECT_NEAR(joint_state.feedback.positions[2], 1.5, 0.05);
}

typedef ::testing::Types<PositionHandle, VelocityHandle> TestTypes;
TYPED_TEST_SUITE(OmniBaseControllerTest, TestTypes);

/// Move the cart by giving constant velocity commands to x and y
TYPED_TEST(OmniBaseControllerTest, CommandVelocity) {
  this->SetupController();

  geometry_msgs::msg::Twist command_velocity;
  command_velocity.linear.x = -0.1;
  command_velocity.linear.y = -0.05;

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 199; ++i) {
    this->cmd_vel_publisher_->publish(command_velocity);
    this->SpinOnce(loop_rate);
  }

  // Default is 50Hz, resulting in 99, but loosen conditions as publisher's trymock sometimes fails
  EXPECT_GE(this->state_counter_->count(), 90);
  auto base_state = this->state_counter_->last_msg();

  ASSERT_EQ(base_state.joint_names.size(), 3);
  EXPECT_EQ(base_state.joint_names[0], "odom_x");
  EXPECT_EQ(base_state.joint_names[1], "odom_y");
  EXPECT_EQ(base_state.joint_names[2], "odom_t");

  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], -0.2, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], -0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.0, kEpsilon);

  ASSERT_EQ(base_state.feedback.velocities.size(), 3);
  EXPECT_NEAR(base_state.feedback.velocities[0], -0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.velocities[1], -0.05, kEpsilon);
  EXPECT_NEAR(base_state.feedback.velocities[2], 0.0, kEpsilon);

  auto wheel_odom = this->wheel_odom_counter_->last_msg();
  EXPECT_EQ(wheel_odom.header.frame_id, "odom");
  EXPECT_EQ(wheel_odom.child_frame_id, "base_footprint_wheel");
  EXPECT_NEAR(wheel_odom.pose.pose.position.x, -0.2, kEpsilon);
  EXPECT_NEAR(wheel_odom.pose.pose.position.y, -0.1, kEpsilon);
  EXPECT_NEAR(wheel_odom.pose.pose.orientation.z, 0.0, kEpsilon);
  EXPECT_NEAR(wheel_odom.pose.pose.orientation.w, 1.0, kEpsilon);
  EXPECT_NEAR(wheel_odom.twist.twist.linear.x, -0.1, kEpsilon);
  EXPECT_NEAR(wheel_odom.twist.twist.linear.y, -0.05, kEpsilon);
  EXPECT_NEAR(wheel_odom.twist.twist.angular.z, 0.0, kEpsilon);
}

/// Verify correct tracking of test trajectory input via topic
TYPED_TEST(OmniBaseControllerTest, SendOdomTrajectoryTopic) {
  this->SetupController();

  this->odom_trajectory_publisher_->publish(GetTestOdomTrajectory());

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 100; ++i) {
    this->SpinOnce(loop_rate);
  }
  auto base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.3, kEpsilon);

  for (int i = 0; i < 100; ++i) {
    this->SpinOnce(loop_rate);

    // Originally existed in the test, so check it just in case
    auto state = this->state_counter_->last_msg();
    for (int i = 0; i < 3; ++i) {
      ASSERT_LT(fabs(state.error.positions[i]),  kPositionErrorThreshold);
      ASSERT_LT(fabs(state.error.velocities[i]), kVelocityErrorThreshold);
    }
  }
  base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.0, kEpsilon);

  ASSERT_EQ(base_state.feedback.velocities.size(), 3);
  EXPECT_NEAR(base_state.feedback.velocities[0], 0.0, kVelocityErrorThreshold);
  EXPECT_NEAR(base_state.feedback.velocities[1], 0.0, kVelocityErrorThreshold);
  EXPECT_NEAR(base_state.feedback.velocities[2], 0.0, kVelocityErrorThreshold);
}

/// Verify correct tracking of test trajectory input via topic
TYPED_TEST(OmniBaseControllerTest, SendRollTrajectoryTopic) {
  this->SetupController();
  this->roll_trajectory_publisher_->publish(GetTestRollTrajectory());

  // Slightly increase loop count as state reception failed with exact loop count
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 110; ++i) {
    this->SpinOnce(loop_rate);
  }
  auto joint_state = this->joint_state_counter_->last_msg();
  ASSERT_EQ(joint_state.feedback.positions.size(), 3);
  EXPECT_NEAR(joint_state.feedback.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(joint_state.feedback.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(joint_state.feedback.positions[2], 1.0, 0.05);

  ASSERT_EQ(joint_state.feedback.velocities.size(), 3);
  EXPECT_NEAR(joint_state.feedback.velocities[0], 0.0, kVelocityErrorThreshold);
  EXPECT_NEAR(joint_state.feedback.velocities[1], 0.0, kVelocityErrorThreshold);
  EXPECT_NEAR(joint_state.feedback.velocities[2], 0.0, kVelocityErrorThreshold);
}

/// Verify correct tracking of test trajectory input via action
TYPED_TEST(OmniBaseControllerTest, SendOdomTrajectoryAction) {
  this->SetupController();

  ActionType::Goal goal;
  goal.trajectory = GetTestOdomTrajectory();

  ActionType::Result result;
  auto result_callback = [&result](const rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& _result) {
    result = *_result.result;
  };
  ActionType::Feedback feedback;
  auto feedback_callback = [&feedback](rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr,
                                       const std::shared_ptr<const ActionType::Feedback> _feedback) {
    feedback = *_feedback;
  };

  auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_options.result_callback = result_callback;
  send_goal_options.feedback_callback = feedback_callback;

  auto future_goal_handle = this->odom_action_client_->async_send_goal(goal, send_goal_options);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 100; ++i) {
    this->SpinOnce(loop_rate);
  }
  auto base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.3, kEpsilon);

  ASSERT_EQ(feedback.actual.positions.size(), 3);
  EXPECT_NEAR(feedback.actual.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(feedback.actual.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(feedback.actual.positions[2], 0.3, kEpsilon);

  EXPECT_EQ(feedback.actual.velocities.size(), 3);
  EXPECT_TRUE(feedback.actual.accelerations.empty());

  ASSERT_EQ(feedback.desired.positions.size(), 3);
  EXPECT_NEAR(feedback.desired.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(feedback.desired.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(feedback.desired.positions[2], 0.3, kEpsilon);

  EXPECT_EQ(feedback.desired.velocities.size(), 3);
  EXPECT_EQ(feedback.desired.accelerations.size(), 3);

  ASSERT_EQ(feedback.error.positions.size(), 3);
  EXPECT_NEAR(feedback.error.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(feedback.error.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(feedback.error.positions[2], 0.0, kEpsilon);

  ASSERT_EQ(feedback.error.velocities.size(), 3);
  EXPECT_NEAR(feedback.error.velocities[0], 0.0, kEpsilon);
  EXPECT_NEAR(feedback.error.velocities[1], 0.0, kEpsilon);
  EXPECT_NEAR(feedback.error.velocities[2], 0.0, kEpsilon);

  EXPECT_TRUE(feedback.error.accelerations.empty());

  for (int i = 0; i < 100; ++i) {
    this->SpinOnce(loop_rate);
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  this->SpinOnce(loop_rate);
  EXPECT_EQ(result.error_code, control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);

  base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.0, kEpsilon);
}

/// Verify correct tracking of test trajectory input via action
TYPED_TEST(OmniBaseControllerTest, SendRollTrajectoryAction) {
  this->SetupController();

  ActionType::Goal goal;
  goal.trajectory = GetTestRollTrajectory();

  ActionType::Result result;
  auto result_callback = [&result](const rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& _result) {
    result = *_result.result;
  };
  ActionType::Feedback feedback;
  auto feedback_callback = [&feedback](rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr,
                                       const std::shared_ptr<const ActionType::Feedback> _feedback) {
    feedback = *_feedback;
  };

  auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_options.result_callback = result_callback;
  send_goal_options.feedback_callback = feedback_callback;

  auto future_goal_handle = this->roll_action_client_->async_send_goal(goal, send_goal_options);

  // Slightly increase loop count as state reception failed with exact loop count
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 110; ++i) {
    this->SpinOnce(loop_rate);
  }

  auto base_state = this->joint_state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 1.0, 0.05);

  ASSERT_EQ(base_state.feedback.velocities.size(), 3);
  EXPECT_NEAR(base_state.feedback.velocities[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.velocities[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.velocities[2], 0.0, kEpsilon);

  ASSERT_EQ(feedback.actual.positions.size(), 1);
  EXPECT_NEAR(feedback.actual.positions[0], 1.0, 0.1);

  EXPECT_EQ(feedback.actual.velocities.size(), 1);
  EXPECT_TRUE(feedback.actual.accelerations.empty());

  ASSERT_EQ(feedback.desired.positions.size(), 1);
  EXPECT_NEAR(feedback.desired.positions[0], 1.0, 0.05);

  EXPECT_EQ(feedback.desired.velocities.size(), 1);
  EXPECT_EQ(feedback.desired.accelerations.size(), 1);

  ASSERT_EQ(feedback.error.positions.size(), 1);
  EXPECT_NEAR(feedback.error.positions[0], 0.0, 0.05);

  EXPECT_EQ(feedback.error.velocities.size(), 1);
  EXPECT_TRUE(feedback.error.accelerations.empty());

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  this->SpinOnce(loop_rate);
  EXPECT_EQ(result.error_code, control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);
}

/// Verify proper trajectory generation and tracking even with command values exceeding PI in rotation direction
TYPED_TEST(OmniBaseControllerTest, OverPISteerTrajectory) {
  this->SetupController();

  ActionType::Goal goal;
  goal.trajectory.joint_names = {"odom_x", "odom_y", "odom_t"};

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0, 0.0, 3.0};
  point.time_from_start = rclcpp::Duration(3, 0);
  goal.trajectory.points.push_back(point);

  point.positions = {0.0, 0.0, -3.0};
  point.time_from_start = rclcpp::Duration(4, 0);
  goal.trajectory.points.push_back(point);

  point.positions = {0.0, 0.0, -2.5};
  point.time_from_start = rclcpp::Duration(5, 0);
  goal.trajectory.points.push_back(point);

  auto future_goal_handle = this->odom_action_client_->async_send_goal(goal);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 300; ++i) {
    this->SpinOnce(loop_rate);
  }
  auto base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_GT(base_state.feedback.positions[2], 2.9);

  for (int i = 0; i < 100; ++i) {
    this->SpinOnce(loop_rate);
  }
  base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_LT(base_state.feedback.positions[2], -2.9);

  for (int i = 0; i < 120; ++i) {
    this->SpinOnce(loop_rate);
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));

  base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[2], -2.5, kEpsilon);
}

/// Verify proper stopping when unable to track test trajectory input via topic
TYPED_TEST(OmniBaseControllerTest, StopFollowingInTopic) {
  this->SetupController();

  auto trajectory = GetTestOdomTrajectory();
  trajectory.points[0].positions[0] = 5.0;
  this->odom_trajectory_publisher_->publish(trajectory);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  double max_error = 0.0;
  for (int i = 0; i < 100; ++i) {
    this->SpinOnce(loop_rate);

    auto base_state = this->state_counter_->last_msg();
    if (base_state.error.positions.size() == 3) {
      max_error = std::max(max_error, std::abs(base_state.error.positions[0]));
    }
  }
  EXPECT_GT(max_error, 0.5);

  // Arbitrary threshold to check minimal progress before stopping
  auto base_state = this->state_counter_->last_msg();
  EXPECT_LT(base_state.feedback.positions[0], 0.2);

  ASSERT_EQ(base_state.feedback.velocities.size(), 3);
  EXPECT_NEAR(base_state.feedback.velocities[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.velocities[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.velocities[2], 0.0, kEpsilon);
}

/// Verify proper stopping when unable to track test trajectory input via action
TYPED_TEST(OmniBaseControllerTest, StopFollowingInAction) {
  this->SetupController();

  ActionType::Goal goal;
  goal.trajectory = GetTestOdomTrajectory();
  goal.trajectory.points[0].positions[0] = 5.0;

  ActionType::Result result;
  auto result_callback = [&result](const rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& _result) {
    result = *_result.result;
  };
  auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_options.result_callback = result_callback;

  auto future_goal_handle = this->odom_action_client_->async_send_goal(goal, send_goal_options);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 100; ++i) {
    this->SpinOnce(loop_rate);
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(this-> template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED));

  this->SpinOnce(loop_rate);
  EXPECT_EQ(result.error_code, control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED);
}

/// Verify correct failure result when precision is insufficient for the goal of test trajectory input via action
TYPED_TEST(OmniBaseControllerTest, OverGoalTolerance) {
  this->SetupController();

  ActionType::Goal goal;
  goal.trajectory = GetTestOdomTrajectory();

  ActionType::Result result;
  auto result_callback = [&result](const rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& _result) {
  result = *_result.result;
  };
  auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_options.result_callback = result_callback;

  auto future_goal_handle = this->odom_action_client_->async_send_goal(goal, send_goal_options);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 200; ++i) {
    this->SpinOnce(loop_rate);
  }

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_ABORTED, false));

  this->SpinOnce(loop_rate);
  EXPECT_EQ(result.error_code, control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED);
}

/// Test for action cancellation
TYPED_TEST(OmniBaseControllerTest, OdomActionCancel) {
  this->SetupController();

  // Start cart trajectory action
  auto goal_handle = this->StartOdomTrajectoryAction();

  // Cancel action
  auto future_cancel = this->odom_action_client_->async_cancel_goal(goal_handle);
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  this->WaitForReady(future_cancel, loop_rate);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);

  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

/// Test for action cancellation
TYPED_TEST(OmniBaseControllerTest, RollActionCancel) {
  this->SetupController();

  // Start rotation axis trajectory action
  auto goal_handle = this->StartRollTrajectoryAction();

  // Cancel action
  auto future_cancel = this->roll_action_client_->async_cancel_goal(goal_handle);
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  this->WaitForReady(future_cancel, loop_rate);

  auto cancel_response = future_cancel.get();
  EXPECT_EQ(cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);

  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

/// Send Goal twice consecutively (the first one gets canceled by ClearActiveGoal)
TYPED_TEST(OmniBaseControllerTest, SendOdomGoalTwice) {
  this->SetupController();

  // Start cart trajectory action
  auto goal_handle_first = this->StartOdomTrajectoryAction();

  // Resend cart trajectory action
  ActionType::Goal goal;
  goal.trajectory = GetTestOdomTrajectory();
  auto future_goal_handle_second = this->odom_action_client_->async_send_goal(goal);
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  this->WaitForReady(future_goal_handle_second, loop_rate);

  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle_first, action_msgs::msg::GoalStatus::STATUS_CANCELED));

  auto goal_handle_second = future_goal_handle_second.get();
  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle_second, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

/// Send Goal twice consecutively (the first one gets canceled by ClearActiveGoal)
TYPED_TEST(OmniBaseControllerTest, SendRollGoalTwice) {
  this->SetupController();

  // Start rotation axis trajectory action
  auto goal_handle_first = this->StartRollTrajectoryAction();

  // Resend rotation axis trajectory action
  ActionType::Goal goal;
  goal.trajectory = GetTestRollTrajectory();
  auto future_goal_handle_second = this->roll_action_client_->async_send_goal(goal);
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  this->WaitForReady(future_goal_handle_second, loop_rate);

  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle_first, action_msgs::msg::GoalStatus::STATUS_CANCELED));

  auto goal_handle_second = future_goal_handle_second.get();
  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle_second, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

/// Verify failure of configure when cart coordinate axis parameter is missing
TYPED_TEST(OmniBaseControllerTest, NoOdomCoordParameter) {
  this->controller_ = std::make_shared<OmniBaseController>();

  rclcpp::NodeOptions options = rclcpp::NodeOptions()
      .allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  options.append_parameter_override<std::string>("robot_description", GetRobotDescription());
  options.append_parameter_override<float>("odometry_publish_rate", 200.0);
  options.append_parameter_override<std::string>("joints.steer", "base_roll_joint");
  options.append_parameter_override<std::string>("joints.r_wheel", "base_r_drive_wheel_joint");
  options.append_parameter_override<std::string>("joints.l_wheel", "base_l_drive_wheel_joint");
  options.append_parameter_override<float>("odom_x.p_gain", 0.01);
  options.append_parameter_override<float>("odom_y.p_gain", 0.01);
  options.append_parameter_override<float>("odom_t.p_gain", 0.01);
  options.append_parameter_override<float>("constraints.odom_x.trajectory", 0.5);
  options.append_parameter_override<float>("constraints.goal_time", 0.5);
  options.append_parameter_override<bool>("use_base_roll_velocity", TypeParam::use_base_roll_velocity);

  EXPECT_EQ(this->controller_->init(kControllerNodeName, "", 0, "", options), controller_interface::return_type::OK);
  EXPECT_EQ(this->controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

/// Send Goal without setting trajectory
TYPED_TEST(OmniBaseControllerTest, EmptyOdomTrajectoryGoal) {
  this->SetupController();

  ActionType::Goal goal;

  auto future_goal_handle = this->odom_action_client_->async_send_goal(goal);
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 10; ++i) {
    this->SpinOnce(loop_rate);
  }
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(goal_handle, nullptr);
}

/// Send Goal without setting trajectory
TYPED_TEST(OmniBaseControllerTest, EmptyRollTrajectoryGoal) {
  this->SetupController();

  ActionType::Goal goal;

  auto future_goal_handle = this->roll_action_client_->async_send_goal(goal);
  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 10; ++i) {
    this->SpinOnce(loop_rate);
  }
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(goal_handle, nullptr);
}

/// Verify speed limit activation when speed command exceeds threshold
TYPED_TEST(OmniBaseControllerTest, VelocityLimit) {
  this->SetupController();

  for (int32_t i = 0; i < 4; i++) {
    geometry_msgs::msg::Twist command_velocity;
    switch (i) {
      case 0:
        // Positive speed exceeding limit in x direction
        command_velocity.linear.x = 10.0;
        command_velocity.linear.y = 0.0;
        command_velocity.angular.z = 0.0;
        break;
      case 1:
        // Negative speed exceeding limit in y direction
       command_velocity.linear.x = 0.0;
       command_velocity.linear.y = -10.0;
       command_velocity.angular.z = 0.0;
       break;
     case 2:
        // Speed exceeding limit in yaw direction
        command_velocity.linear.x = 0.0;
        command_velocity.linear.y = 0.0;
        command_velocity.angular.z = 40.0;
        break;
      case 3:
        // Speed exceeding limit in x and yaw directions (verify double limit activation)
        command_velocity.linear.x = 10.0;
        command_velocity.linear.y = 0.0;
        command_velocity.angular.z = 40.0;
        break;
      default:
        break;
    }

    rclcpp::WallRate loop_rate(kUpdateFrequency);
    for (int j = 0; j < 100; ++j) {
      this->cmd_vel_publisher_->publish(command_velocity);
      this->SpinOnce(loop_rate);

      auto state = this->joint_state_counter_->last_msg();
      if (state.output.velocities.size() == 3) {
        // Compare with a buffer of 1/10,000 to account for decimal errors
        ASSERT_LE(std::abs(state.output.velocities[0]), kWheelVelocityLimitThreshold * 1.0001);
        ASSERT_LE(std::abs(state.output.velocities[1]), kWheelVelocityLimitThreshold * 1.0001);
        ASSERT_LE(std::abs(state.output.velocities[2]), kYawVelocityLimitThreshold * 1.0001);
      }
    }

    if (i == 0) {
      // Check difference between desired and output with clear command values
      auto joint_state = this->joint_state_counter_->last_msg();
      ASSERT_EQ(joint_state.reference.velocities.size(), 3);
      // 10.0 / 0.04 = 250.0
      ASSERT_NEAR(joint_state.reference.velocities[0], 250.0, kEpsilon);
      ASSERT_NEAR(joint_state.reference.velocities[1], 250.0, kEpsilon);
      ASSERT_NEAR(joint_state.reference.velocities[2], 0.0, kEpsilon);

      ASSERT_EQ(joint_state.output.velocities.size(), 3);
      ASSERT_NEAR(joint_state.output.velocities[0], kWheelVelocityLimitThreshold, kEpsilon);
      ASSERT_NEAR(joint_state.output.velocities[1], kWheelVelocityLimitThreshold, kEpsilon);
      ASSERT_NEAR(joint_state.output.velocities[2], 0.0, kEpsilon);

      auto base_state = this->state_counter_->last_msg();
      ASSERT_EQ(base_state.reference.velocities.size(), 3);
      ASSERT_NEAR(base_state.reference.velocities[0], 10.0, kEpsilon);
      ASSERT_NEAR(base_state.reference.velocities[1], 0.0, kEpsilon);
      ASSERT_NEAR(base_state.reference.velocities[2], 0.0, kEpsilon);

      ASSERT_EQ(base_state.output.velocities.size(), 3);
      ASSERT_NEAR(base_state.output.velocities[0], kWheelVelocityLimitThreshold * 0.04, kEpsilon);
      ASSERT_NEAR(base_state.output.velocities[1], 0.0, kEpsilon);
      ASSERT_NEAR(base_state.output.velocities[2], 0.0, kEpsilon);
    }
  }
}

/// Test for switching cart control methods
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethod) {
  this->SetupController();

  // Start with speed
  geometry_msgs::msg::Twist command_velocity;
  command_velocity.linear.x = 0.1;
  command_velocity.linear.y = 0.1;

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  for (int i = 0; i < 200; ++i) {
    this->cmd_vel_publisher_->publish(command_velocity);
    this->SpinOnce(loop_rate);
  }

  auto base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.2, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.2, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.0, kEpsilon);

  // Switch to trajectory
  this->odom_trajectory_publisher_->publish(GetTestOdomTrajectory());

  for (int i = 0; i < 100; ++i) {
    this->SpinOnce(loop_rate);
  }

  base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.3, kEpsilon);

  for (int i = 0; i < 100; ++i) {
    this->SpinOnce(loop_rate);
  }

  base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.0, kEpsilon);

  // Back to speed
  for (int i = 0; i < 200; ++i) {
    this->cmd_vel_publisher_->publish(command_velocity);
    this->SpinOnce(loop_rate);
  }

  base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.2, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.2, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.0, kEpsilon);
}

/// Test for switching cart control methods (from speed command topic to cart trajectory topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomVelocityTopicToOdomTrajectoryTopic) {
  this->SetupController();

  // Start speed command topic
  this->StartCmdVelTopic();

  // Interrupt with cart trajectory topic
  this->PreemptOdomTrajectoryTopic();
}

/// Test for switching cart control methods (from speed command topic to cart trajectory action)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomVelocityTopicToOdomTrajectoryAction) {
  this->SetupController();

  // Start speed command topic
  this->StartCmdVelTopic();

  // Interrupt with cart trajectory action
  this->PreemptOdomTrajectoryAction();
}

/// Test for switching cart control methods (from speed command topic to rotation axis trajectory topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomVelocityTopicToRollTrajectoryTopic) {
  this->SetupController();

  // Start speed command topic
  this->StartCmdVelTopic();

  // Interrupt with rotation axis trajectory topic
  this->PreemptRollTrajectoryTopic();
}

/// Test for switching cart control methods (from speed command topic to rotation axis trajectory action)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomVelocityTopicToRollTrajectoryAction) {
  this->SetupController();

  // Start speed command topic
  this->StartCmdVelTopic();

  // Interrupt with rotation axis trajectory action
  this->PreemptRollTrajectoryAction();
}

/// Test for switching cart control methods (from cart trajectory topic to speed command topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomTrajectoryTopicToOdomVelocityTopic) {
  this->SetupController();

  // Start cart trajectory topic
  this->StartOdomTrajectoryTopic();

  // Send speed command topic
  this->PreemptCmdVelTopic();

  // Ignore speed command and move along trajectory
  auto base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.3, kEpsilon);
}

/// Test for switching cart control methods (from cart trajectory topic to cart trajectory action)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomTrajectoryTopicToOdomTrajectoryAction) {
  this->SetupController();

  // Start cart trajectory topic
  this->StartOdomTrajectoryTopic();

  // Interrupt with cart trajectory action
  this->PreemptOdomTrajectoryAction();
}

/// Test for switching cart control methods (from cart trajectory topic to rotation axis trajectory action)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomTrajectoryTopicToRollTrajectoryAction) {
  this->SetupController();

  // Start cart trajectory topic
  this->StartOdomTrajectoryTopic();

  // Interrupt with rotation axis trajectory action
  this->PreemptRollTrajectoryAction();
}

/// Test for switching cart control methods (from cart trajectory topic to rotation axis trajectory topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomTrajectoryTopicToRollTrajectoryTopic) {
  this->SetupController();

  // Start cart trajectory topic
  this->StartOdomTrajectoryTopic();

  // Interrupt with rotation axis trajectory topic
  this->PreemptRollTrajectoryTopic();
}

/// Test for switching cart control methods (from cart trajectory action to cart speed command topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomTrajectoryActionToOdomVelocityTopic) {
  this->SetupController();

  // Start cart trajectory action
  auto goal_handle = this->StartOdomTrajectoryAction();

  // Send speed command topic
  this->PreemptCmdVelTopic();

  // Ignore speed command and move along trajectory
  auto base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.1, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.3, kEpsilon);

  // Action succeeds
  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

/// Test for switching cart control methods (from cart trajectory action to cart trajectory topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomTrajectoryActionToOdomTrajectoryTopic) {
  this->SetupController();

  // Start cart trajectory action
  auto goal_handle = this->StartOdomTrajectoryAction();

  // Interrupt with cart trajectory topic
  this->PreemptOdomTrajectoryTopic();

  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

/// Test for switching cart control methods (from cart trajectory action to rotation axis trajectory action)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomTrajectoryActionToRollTrajectoryAction) {
  this->SetupController();

  // Start cart trajectory action
  auto odom_goal_handle = this->StartOdomTrajectoryAction();

  // Interrupt with rotation axis trajectory action
  this->PreemptRollTrajectoryAction();

  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      odom_goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

/// Test for switching cart control methods (from cart trajectory action to rotation axis trajectory topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodOdomTrajectoryActionToRollTrajectoryTopic) {
  this->SetupController();

  // Start cart trajectory action
  auto goal_handle = this->StartOdomTrajectoryAction();

  // Interrupt with rotation axis trajectory topic
  this->PreemptRollTrajectoryTopic();

  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

/// Test for switching cart control methods (from rotation axis trajectory topic to cart speed command topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodRollTrajectoryTopicToOdomVelocityTopic) {
  this->SetupController();

  // Start rotation axis trajectory topic
  this->StartRollTrajectoryTopic();

  // Send speed command topic
  this->PreemptCmdVelTopic();

  // Ignore speed command, cart position does not move, and follows rotation axis position trajectory
  auto base_state = this->joint_state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 1.0, kEpsilon);
}

/// Test for switching cart control methods (from rotation axis trajectory topic to cart trajectory topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodRollTrajectoryTopicToOdomTrajectoryTopic) {
  this->SetupController();

  // Start rotation axis trajectory topic
  this->StartRollTrajectoryTopic();

  // Interrupt with cart trajectory topic
  this->PreemptOdomTrajectoryTopic();
}

/// Test for switching cart control methods (from rotation axis trajectory topic to cart trajectory action)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodRollTrajectoryTopicToOdomTrajectoryAction) {
  this->SetupController();

  // Start rotation axis trajectory topic
  this->StartRollTrajectoryTopic();

  // Interrupt with cart trajectory action
  this->PreemptOdomTrajectoryAction();
}

/// Test for switching cart control methods (from rotation axis trajectory topic to rotation axis trajectory action)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodRollTrajectoryTopicToRollTrajectoryAction) {
  this->SetupController();

  // Start rotation axis trajectory topic
  this->StartRollTrajectoryTopic();

  // Interrupt with rotation axis trajectory action
  this->PreemptRollTrajectoryAction();
}

/// Test for switching cart control methods (from rotation axis trajectory action to cart speed command topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodRollTrajectoryActionToOdomVelocityTopic) {
  this->SetupController();

  // Start rotation axis trajectory action
  auto goal_handle = this->StartRollTrajectoryAction();

  // Send speed command topic
  this->PreemptCmdVelTopic();

  // Ignore speed command, cart position does not move, and follows rotation axis position trajectory
  auto base_state = this->joint_state_counter_->last_msg();
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.0, kEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 1.0, kEpsilon);

  // Action succeeds
  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED));
}

/// Test for switching cart control methods (from rotation axis trajectory action to cart trajectory topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodRollTrajectoryActionToOdomTrajectoryTopic) {
  this->SetupController();

  // Start rotation axis trajectory action
  auto goal_handle = this->StartRollTrajectoryAction();

  // Interrupt with cart trajectory topic
  this->PreemptOdomTrajectoryTopic();

  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

/// Test for switching cart control methods (from rotation axis trajectory action to cart trajectory action)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodRollTrajectoryActionToOdomTrajectoryAction) {
  this->SetupController();

  // Start rotation axis trajectory action
  auto goal_handle = this->StartRollTrajectoryAction();

  // Interrupt with cart trajectory action
  this->PreemptOdomTrajectoryAction();

  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

/// Test for switching cart control methods (from rotation axis trajectory action to rotation axis trajectory topic)
TYPED_TEST(OmniBaseControllerTest, ChangeControlMethodRollTrajectoryActionToRollTrajectoryTopic) {
  this->SetupController();

  // Start rotation axis trajectory action
  auto goal_handle = this->StartRollTrajectoryAction();

  // Interrupt with cart trajectory action
  this->PreemptRollTrajectoryTopic();

  EXPECT_TRUE(this->template WaitForStatus<ActionType>(
      goal_handle, action_msgs::msg::GoalStatus::STATUS_CANCELED));
}

/// Test for open loop control
TYPED_TEST(OmniBaseControllerTest, OpenLoopControl) {
  this->SetupController(true);
  this->odom_relay_.reset();

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"odom_x", "odom_y", "odom_t"};

  constexpr double kGoalPosition = 0.1;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {kGoalPosition, 0.0, 0.0};
  const auto trajectory_duration = rclcpp::Duration(1, 0);
  point.time_from_start = trajectory_duration;
  trajectory.points.push_back(point);

  this->odom_trajectory_publisher_->publish(trajectory);

  rclcpp::WallRate loop_rate(kUpdateFrequency);
  const auto executing_duration = rclcpp::Duration(0, 200000000);
  auto timeout = this->client_node_->now() + executing_duration;
  while (this->client_node_->now() < timeout) {
    this->SpinOnce(loop_rate);
  }
  ASSERT_GE(this->state_counter_->count(), 1);

  auto base_state = this->state_counter_->last_msg();
  constexpr double kHardEpsilon = 1e-6;
  // Since odom_relay_ is removed, actual position does not move
  ASSERT_EQ(base_state.feedback.positions.size(), 3);
  EXPECT_NEAR(base_state.feedback.positions[0], 0.0, kHardEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[1], 0.0, kHardEpsilon);
  EXPECT_NEAR(base_state.feedback.positions[2], 0.0, kHardEpsilon);

  ASSERT_EQ(base_state.reference.positions.size(), 3);
  EXPECT_NEAR(base_state.reference.positions[0], kGoalPosition * executing_duration.seconds(), kGoalPosition * 0.05);
  EXPECT_NEAR(base_state.reference.positions[1], 0.0, kHardEpsilon);
  EXPECT_NEAR(base_state.reference.positions[2], 0.0, kHardEpsilon);
  // Velocities are generated from wheel motion, so they move even if odom_relay_ is removed
  const double expected_velocity = kGoalPosition / trajectory_duration.seconds();
  ASSERT_EQ(base_state.feedback.velocities.size(), 3);
  EXPECT_NEAR(base_state.feedback.velocities[0], expected_velocity, kEpsilon);
  EXPECT_NEAR(base_state.feedback.velocities[1], 0.0, kHardEpsilon);
  EXPECT_NEAR(base_state.feedback.velocities[2], 0.0, kHardEpsilon);

  ASSERT_EQ(base_state.reference.velocities.size(), 3);
  EXPECT_NEAR(base_state.reference.velocities[0], expected_velocity, kEpsilon);
  EXPECT_NEAR(base_state.reference.velocities[1], 0.0, kHardEpsilon);
  EXPECT_NEAR(base_state.reference.velocities[2], 0.0, kHardEpsilon);

  // Throw a new trajectory
  const double last_desired_position_x = base_state.reference.positions[0];

  trajectory.points[0].positions = {0.0, 0.1, 0.0};
  this->odom_trajectory_publisher_->publish(trajectory);

  timeout = this->client_node_->now() + executing_duration;
  while (this->client_node_->now() < timeout) {
    this->SpinOnce(loop_rate);
  }

  base_state = this->state_counter_->last_msg();
  ASSERT_EQ(base_state.reference.positions.size(), 3);
  // It is important to be at last_desired_position_x here
  EXPECT_NEAR(base_state.reference.positions[0], last_desired_position_x, kEpsilon);
  EXPECT_NEAR(base_state.reference.positions[1], kGoalPosition * executing_duration.seconds(), kGoalPosition * 0.05);
  EXPECT_NEAR(base_state.reference.positions[2], 0.0, kHardEpsilon);

  ASSERT_EQ(base_state.reference.velocities.size(), 3);
  // x should have a speed to return to 0.0
  EXPECT_NEAR(base_state.reference.velocities[0], -last_desired_position_x / trajectory_duration.seconds(), kEpsilon);
  EXPECT_NEAR(base_state.reference.velocities[1], expected_velocity, kEpsilon);
  EXPECT_NEAR(base_state.reference.velocities[2], 0.0, kHardEpsilon);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
