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
/// @file command_subscriber.cpp
/// @brief Input command control class for omnidirectional vehicle control
#include <hsrb_base_controllers/command_subscriber.hpp>

#include <rclcpp_action/create_server.hpp>

#include "utils.hpp"

namespace {
// Default value for action state update frequency [Hz]
constexpr double kDefaultActionMonitorRate = 100.0;
}

namespace hsrb_base_controllers {

/// Input command class
CommandSubscriber::CommandSubscriber(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                     IControllerCommandInterface* controller)
    : node_(node), controller_(controller), target_control_(nullptr) {}


/// Initialization of input velocity command class
CommandVelocitySubscriber::CommandVelocitySubscriber(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                                     IControllerCommandInterface* controller)
    : CommandSubscriber(node, controller) {
  // QoS for diff_drive_controller is SystemDefaults, steering_controllers generally use SensorDataQoS
  // SensorDataQoS is adopted considering the importance of timely acquisition of the latest values
  velocity_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", rclcpp::SensorDataQoS(),
      std::bind(&CommandVelocitySubscriber::CommandVelocityCallback, this, std::placeholders::_1));
}

/// Input velocity command callback
void CommandVelocitySubscriber::CommandVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (controller_->IsAcceptable()) {
    controller_->UpdateVelocity(msg);
  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Controller is not running.");
  }
}


/// Initialization of input trajectory command class
CommandTrajectorySubscriber::CommandTrajectorySubscriber(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                                         const std::string& topic_name,
                                                         IControllerCommandInterface* controller)
    : CommandSubscriber(node, controller) {
  // Align with the QoS of joint_trajectory_controller
  trajectory_subscriber_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      topic_name, rclcpp::SensorDataQoS(),
      std::bind(&CommandTrajectorySubscriber::CommandTrajectoryCallback, this, std::placeholders::_1));
}

/// Input trajectory command callback
void CommandTrajectorySubscriber::CommandTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  if (!controller_->IsAcceptable()) {
    RCLCPP_ERROR(node_->get_logger(), "Controller is not running.");
    return;
  }
  if (controller_->ValidateTrajectory(*msg)) {
    controller_->PreemptActiveGoal();
    controller_->ResetTrajectory(target_control_);
    controller_->UpdateTrajectory(msg);
  }
}


/// Initialization of input trajectory action command class
TrajectoryActionServer::TrajectoryActionServer(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                               const std::vector<std::string>& cordinates,
                                               const std::string& server_name,
                                               IControllerCommandInterface* controller)
    : CommandSubscriber(node, controller), cordinates_(cordinates) {
  double action_monitor_rate = GetPositiveParameter(node, "action_monitor_rate", kDefaultActionMonitorRate);
  action_monitor_period_ = 1.0 / action_monitor_rate;

  goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());

  action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      node, server_name,
      std::bind(&TrajectoryActionServer::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryActionServer::CancelCallback, this, std::placeholders::_1),
      std::bind(&TrajectoryActionServer::FeedbackSetupCallback, this, std::placeholders::_1));
}

void TrajectoryActionServer::UpdateActionResult(int32_t error_code) {
  const auto active_goal = *goal_handle_buffer_.readFromRT();
  if (!active_goal) {
    return;
  }

  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
  result->error_code = error_code;

  if (result->error_code == control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL) {
    active_goal->setSucceeded(result);
  } else {
    active_goal->setAborted(result);
  }
  goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
}

void TrajectoryActionServer::SetFeedback(const ControllerState& state, const rclcpp::Time& stamp) {
  const auto active_goal = *goal_handle_buffer_.readFromRT();
  if (!active_goal) {
    return;
  }

  const auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
  feedback->header.stamp = stamp;
  feedback->joint_names = cordinates_;
  Convert(state.actual, feedback->actual);
  Convert(state.desired, feedback->desired);
  Convert(state.error, feedback->error);
  active_goal->setFeedback(feedback);
}

void TrajectoryActionServer::PreemptActiveGoal() {
  const auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (active_goal) {
    auto action_result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    action_result->set__error_code(control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL);
    action_result->set__error_string("Current goal cancelled.");
    active_goal->setCanceled(action_result);
    goal_handle_timer_.reset();
    goal_handle_buffer_.reset();
  }
}

rclcpp_action::GoalResponse TrajectoryActionServer::GoalCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal) {
  if (!controller_->IsAcceptable()) {
    RCLCPP_ERROR(node_->get_logger(), "Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!controller_->ValidateTrajectory(goal->trajectory)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryActionServer::CancelCallback(const ServerGoalHandlePtr goal_handle) {
  const auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle) {
    controller_->ResetTrajectory(nullptr);

    auto action_result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    action_result->set__error_string("Current goal cancelled.");
    active_goal->setCanceled(action_result);
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryActionServer::FeedbackSetupCallback(ServerGoalHandlePtr goal_handle) {
  controller_->PreemptActiveGoal();
  controller_->ResetTrajectory(target_control_);

  const auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);
  controller_->UpdateTrajectory(msg);

  auto realtime_goal_handle = std::make_shared<RealtimeGoalHandle>(goal_handle);
  realtime_goal_handle->execute();
  goal_handle_buffer_.writeFromNonRT(realtime_goal_handle);

  goal_handle_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(action_monitor_period_),
      std::bind(&RealtimeGoalHandle::runNonRealtime, realtime_goal_handle));
}
}  // namespace hsrb_base_controllers
