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
#include "hsrb_gripper_controller/hrh_gripper_follow_distance_trajectory_action.hpp"
#include <limits>
#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>
#include "hsrb_gripper_controller/hrh_gripper_controller.hpp"

namespace {

// Default goal tolerance [m]
const double kDefaultDistanceGoalTolerance = 0.003;
// Default goal acceptance time [s]
const double kDefaultPositionGoalTimeTolerance = 0.05;
// Default upper limit angle of the hand [rad]
const double kDefaultHandMotorJointMax = 1.2;
// Default lower limit angle of the hand [rad]
const double kDefaultHandMotorJointMin = -0.5;

}  // unnamed namespace

namespace hsrb_gripper_controller {

HrhGripperFollowDistanceTrajectoryAction::HrhGripperFollowDistanceTrajectoryAction(HrhGripperController* controller)
    : HrhGripperAction(controller,
                       "/follow_distance_trajectory",
                       tmc_exxx_servo_motor_protocol::kDriveModeHandPosition),
      default_goal_tolerance_(kDefaultDistanceGoalTolerance),
      default_goal_time_tolerance_(kDefaultPositionGoalTimeTolerance) {}

void HrhGripperFollowDistanceTrajectoryAction::Update(const rclcpp::Time& time) {
  // Check if there is a trajectory
  auto current_msg = trajectory_ptr_->get_trajectory_msg();
  auto new_msg = trajectory_msg_buffer_.readFromRT();
  if (current_msg != *new_msg) {
    trajectory_ptr_->update(*new_msg);
  }
  if (!trajectory_active_ptr_ || !(*trajectory_active_ptr_)->has_trajectory_msg() ||
      (*trajectory_active_ptr_)->get_trajectory_msg()->points.empty()) {
    return;
  }

  // Sample of target position
  if (!(*trajectory_active_ptr_)->is_sampled_already()) {
    if (open_loop_control_ && last_command_state_.has_value()) {
      (*trajectory_active_ptr_)->set_point_before_trajectory_msg(last_sampled_time_, last_command_state_.value());
    } else {
      trajectory_msgs::msg::JointTrajectoryPoint current_state;
      current_state.positions = {
          distance_calculator_->GetDistanceFromPosition(controller_->GetCurrentPosition()) };
      current_state.velocities = {
          distance_calculator_->GetDistanceFromPosition(controller_->GetCurrentVelocity()) };
      (*trajectory_active_ptr_)->set_point_before_trajectory_msg(time, current_state);
    }
  }

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator start_segment_it;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator end_segment_it;
  joint_trajectory_controller::interpolation_methods::InterpolationMethod interpolation_method =
      joint_trajectory_controller::interpolation_methods::InterpolationMethod::VARIABLE_DEGREE_SPLINE;
  (*trajectory_active_ptr_)->sample(time, interpolation_method, desired_state, start_segment_it, end_segment_it);
  last_sampled_time_ = time;
  last_command_state_ = desired_state;

  controller_->SetComandPosition(distance_calculator_->GetPositionFromDistance(desired_state.positions[0]));

  const auto active_goal_handle = *goal_handle_buffer_.readFromNonRT();
  if (!active_goal_handle) {
    return;
  }

  double current_distance = distance_calculator_->GetDistanceFromPosition(controller_->GetCurrentPosition());

  // Send feedback
  const auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
  feedback->header.stamp = time;
  feedback->joint_names = {controller_->joint_name()};
  feedback->desired.positions = {distance_calculator_->GetDistanceFromPosition(desired_state.positions[0])};
  feedback->actual.positions = {current_distance};
  active_goal_handle->setFeedback(feedback);

  // Success or failure judgment
  CheckForSuccess(time, current_distance);
}

void HrhGripperFollowDistanceTrajectoryAction::PreemptActiveGoal() {
  HrhGripperAction::PreemptActiveGoal();
  trajectory_msg_buffer_.writeFromNonRT(std::make_shared<trajectory_msgs::msg::JointTrajectory>());
  if (!controller_->IsActiveControlMode(shared_from_this())) {
    last_command_state_.reset();
  }
}

trajectory_msgs::msg::JointTrajectoryPoint HrhGripperFollowDistanceTrajectoryAction::GetReferenceState() {
  trajectory_msgs::msg::JointTrajectoryPoint reference;

  if (last_command_state_.has_value()) {
    reference.positions = { distance_calculator_->GetPositionFromDistance(last_command_state_.value().positions[0]) };
    reference.time_from_start = last_command_state_.value().time_from_start;
  } else {
    reference.positions = { controller_->GetCurrentPosition() };
    reference.velocities = { controller_->GetCurrentVelocity() };
    reference.effort = { controller_->GetCurrentTorque() };
  }

  return reference;
}

trajectory_msgs::msg::JointTrajectoryPoint HrhGripperFollowDistanceTrajectoryAction::GetFeedbackState() {
  trajectory_msgs::msg::JointTrajectoryPoint feedback;

  feedback.positions = { controller_->GetCurrentPosition() };
  feedback.velocities = { controller_->GetCurrentVelocity() };
  feedback.effort = { controller_->GetCurrentTorque() };

  return feedback;
}

/// Implementation of action initialization
bool HrhGripperFollowDistanceTrajectoryAction::InitImpl(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  // Parameter settings
  default_goal_tolerance_ =
      GetPositiveParameter(node, "distance_goal_tolerance", kDefaultDistanceGoalTolerance);
  default_goal_time_tolerance_ =
      GetNonNegativeParameter(node, "position_goal_time_tolerance", kDefaultPositionGoalTimeTolerance);
  open_loop_control_ = GetParameter(node, "open_loop_control", false);
  std::string gripper_namespace = GetParameter(node, "namespace", "~");

  goal_condition_buffer_.initRT(GoalCondition());

  trajectory_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  trajectory_active_ptr_ = &trajectory_ptr_;
  trajectory_msg_buffer_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  // Initialization of class for calculating opening width
  distance_calculator_ = std::make_shared<HrhGripperDistanceCalculator>();
  if (!distance_calculator_->InitializeHandSizeData(node)) {
    return false;
  }

  // Calculate upper and lower limits of opening width
  const double hand_motor_joint_max = GetParameter(node, "hand_motor_joint_max", kDefaultHandMotorJointMax);
  const double hand_motor_joint_min = GetParameter(node, "hand_motor_joint_min", kDefaultHandMotorJointMin);
  distance_max_ = distance_calculator_->GetDistanceFromPosition(hand_motor_joint_max);
  distance_min_ = distance_calculator_->GetDistanceFromPosition(hand_motor_joint_min);

  std::string interface_name = gripper_namespace + "/distance_trajectory";
  distance_trajectory_command_sub_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      interface_name,
      1,
      std::bind(&HrhGripperFollowDistanceTrajectoryAction::DistanceTrajectoryCommandCallback,
                this,
                std::placeholders::_1));

  return true;
}

/// Check if the goal is acceptable
bool HrhGripperFollowDistanceTrajectoryAction::ValidateGoal(
    const control_msgs::action::FollowJointTrajectory::Goal& goal) {
  return ValidateTrajectory(node_, goal.trajectory, controller_->joint_name());
}

/// Update the action goal
void HrhGripperFollowDistanceTrajectoryAction::UpdateActionImpl(
    const control_msgs::action::FollowJointTrajectory::Goal& goal) {
  // path_tolerance is not supported
  if (!goal.path_tolerance.empty()) {
    RCLCPP_WARN(node_->get_logger(), "path_tolerance is not supported. Ignoring the parameter...");
  }

  GoalCondition goal_condition;
  goal_condition.goal =
        std::max(std::min(goal.trajectory.points.back().positions[0], distance_max_), distance_min_);

  auto start_time = rclcpp::Time(goal.trajectory.header.stamp);
  if (start_time == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
    start_time = node_->now();
  }
  goal_condition.expected_arrival_time =
      start_time + rclcpp::Duration(goal.trajectory.points.back().time_from_start);

  rclcpp::Duration goal_time_tolerance(0, 0);
  if (goal.goal_tolerance.size() == 1 && goal.goal_tolerance[0].name == controller_->joint_name()) {
    goal_condition.goal_tolerance = goal.goal_tolerance[0].position;
    goal_time_tolerance = goal.goal_time_tolerance;
  } else {
    goal_condition.goal_tolerance = default_goal_tolerance_;
    goal_time_tolerance = rclcpp::Duration::from_seconds(default_goal_time_tolerance_);
  }
  if (goal_time_tolerance == rclcpp::Duration(0, 0)) {
    goal_condition.abort_time = rclcpp::Time(std::numeric_limits<int64_t>::max());
  } else {
    goal_condition.abort_time = goal_condition.expected_arrival_time + goal_time_tolerance;
  }
  goal_condition_buffer_.writeFromNonRT(goal_condition);

  LimitTrajectory(goal.trajectory);
}

/// Callback when an opening width trajectory command is received via topic
void HrhGripperFollowDistanceTrajectoryAction::DistanceTrajectoryCommandCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  if (ValidateTrajectory(node_, *msg, controller_->joint_name())) {
    controller_->PreemptActiveGoal();
    controller_->ChangeControlMode(shared_from_this());
    LimitTrajectory(*msg);
  }
}

/// Create a trajectory with restricted upper and lower limits
void HrhGripperFollowDistanceTrajectoryAction::LimitTrajectory(
    const trajectory_msgs::msg::JointTrajectory& distance_trajectory) {
  trajectory_msgs::msg::JointTrajectory::SharedPtr position_trajectory =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(distance_trajectory);
  for (auto index = 0u; index < distance_trajectory.points.size(); ++index) {
    position_trajectory->points[index].positions[0] =
        std::max(std::min(distance_trajectory.points[index].positions[0], distance_max_), distance_min_);
  }

  trajectory_msg_buffer_.writeFromNonRT(position_trajectory);
}

/// Success determination
void HrhGripperFollowDistanceTrajectoryAction::CheckForSuccess(
    const rclcpp::Time& time, const double current_distance) {
  // If not an action, do not determine success or failure
  const auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (!active_goal) {
    return;
  }

  // Do not judge until the arrival time
  auto goal_condition = *(goal_condition_buffer_.readFromRT());
  if (time < goal_condition.expected_arrival_time) {
    return;
  }

  if (std::abs(current_distance - goal_condition.goal) < goal_condition.goal_tolerance) {
    // Success if the current opening width is within the allowable error
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->set__error_code(control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);
    active_goal->setSucceeded(result);
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  } else if (time >= goal_condition.abort_time) {
    // Failure if it does not stay within the allowable error for a certain period of time
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->set__error_code(control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED);
    active_goal->setAborted(result);
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

}  // namespace hsrb_gripper_controller
