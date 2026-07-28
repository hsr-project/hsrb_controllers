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
#include "hsrb_gripper_controller/hrh_gripper_set_distance_action.hpp"

#include <limits>
#include <rclcpp/rclcpp.hpp>

#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

#include "hsrb_gripper_controller/hrh_gripper_controller.hpp"


namespace {

// Default goal tolerance [m]
const double kDefaultDistanceGoalTolerance = 0.005;
// Default stall detection velocity threshold [rad/s]
const double kDefaultStallVelocityThreshold = 0.05;
// Default arrival judgment time [s]
const double kDefaultDistanceControlStallTimeout = 1.3;
// Default opening width control P gain
const double kDefaultDistanceControlPgain = 2.0;
// Default opening width control I gain
const double kDefaultDistanceControlIgain = 0.0;
// Default opening width control D gain
const double kDefaultDistanceControlDgain = 2.5;
// Default hand upper limit angle [rad]
const double kDefaultHandMotorJointMax = 1.2;
// Default hand lower limit angle [rad]
const double kDefaultHandMotorJointMin = -0.5;

}  // unnamed namespace

namespace hsrb_gripper_controller {

/// Constructor
HrhGripperSetDistanceAction::HrhGripperSetDistanceAction(HrhGripperController* controller)
    : HrhGripperAction(controller, "/set_distance", tmc_exxx_servo_motor_protocol::kDriveModeHandPosition),
      goal_tolerance_(kDefaultDistanceGoalTolerance),
      stall_velocity_threshold_(kDefaultStallVelocityThreshold),
      distance_control_stall_timeout_(kDefaultDistanceControlStallTimeout),
      distance_control_pgain_(kDefaultDistanceControlPgain),
      distance_control_igain_(kDefaultDistanceControlIgain),
      distance_control_dgain_(kDefaultDistanceControlDgain),
      integrated_distance_error_(0.0),
      last_error_(0.0) {}

/// Periodic update process
void HrhGripperSetDistanceAction::Update(const rclcpp::Time& time) {
  if (!IsActive() && *(stop_flag_buffer_.readFromRT())) {
    return;
  }

  // Calculate current fingertip distance
  double current_distance =
      distance_calculator_->GetDistanceFromPosition(controller_->GetCurrentPosition(),
                                                    controller_->GetLeftSpringPosition(),
                                                    controller_->GetRightSpringPosition());

  // Send feedback
  const auto active_goal_handle = *goal_handle_buffer_.readFromNonRT();
  if (active_goal_handle) {
    const auto feedback = std::make_shared<tmc_control_msgs::action::GripperSetDistance::Feedback>();
    feedback->distance = current_distance;
    active_goal_handle->setFeedback(feedback);
  }

  // Calculate and set command value
  current_command_pos_ = GetCommandPos(current_distance);
  controller_->SetComandPosition(current_command_pos_);

  // Success judgment
  CheckForSuccess(time, current_distance);
}

/// Cancel ongoing action
void HrhGripperSetDistanceAction::PreemptActiveGoal() {
  HrhGripperAction::PreemptActiveGoal();
  stop_flag_buffer_.writeFromNonRT(true);
}

trajectory_msgs::msg::JointTrajectoryPoint HrhGripperSetDistanceAction::GetReferenceState() {
  trajectory_msgs::msg::JointTrajectoryPoint reference;

  reference.positions = { current_command_pos_ };

  return reference;
}

trajectory_msgs::msg::JointTrajectoryPoint HrhGripperSetDistanceAction::GetFeedbackState() {
  trajectory_msgs::msg::JointTrajectoryPoint feedback;

  feedback.positions = { controller_->GetCurrentPosition() };
  feedback.velocities = { controller_->GetCurrentVelocity() };
  feedback.effort = { controller_->GetCurrentTorque() };

  return feedback;
}

/// Implement action initialization
bool HrhGripperSetDistanceAction::InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  // Set parameters
  goal_tolerance_ =
      GetPositiveParameter(node, "distance_goal_tolerance", kDefaultDistanceGoalTolerance);
  stall_velocity_threshold_ =
      GetPositiveParameter(node, "stall_velocity_threshold", kDefaultStallVelocityThreshold);
  distance_control_stall_timeout_ =
      GetPositiveParameter(node, "distance_control_stall_timeout", kDefaultDistanceControlStallTimeout);
  distance_control_pgain_ =
      GetNonNegativeParameter(node, "distance_control_pgain", kDefaultDistanceControlPgain);
  distance_control_igain_ =
      GetNonNegativeParameter(node, "distance_control_igain", kDefaultDistanceControlIgain);
  distance_control_dgain_ =
      GetNonNegativeParameter(node, "distance_control_dgain", kDefaultDistanceControlDgain);
  hand_motor_joint_max_ = GetParameter(node, "hand_motor_joint_max", kDefaultHandMotorJointMax);
  hand_motor_joint_min_ = GetParameter(node, "hand_motor_joint_min", kDefaultHandMotorJointMin);
  std::string gripper_namespace = GetParameter(node, "namespace", "~");

  // Initialize member variables
  goal_buffer_.initRT(0.0);
  stop_flag_buffer_.initRT(true);

  // Initialize class for calculating opening width
  distance_calculator_ = std::make_shared<HrhGripperDistanceCalculator>();
  if (!distance_calculator_->InitializeHandSizeData(node)) {
    return false;
  }

  // Calculate upper and lower limits of opening width
  distance_max_ = distance_calculator_->GetDistanceFromPosition(hand_motor_joint_max_);
  distance_min_ = distance_calculator_->GetDistanceFromPosition(hand_motor_joint_min_);

  // Receive opening width
  std::string interface_name = gripper_namespace + "/command_distance";
  distance_command_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      interface_name,
      1,
      std::bind(&HrhGripperSetDistanceAction::DistanceCommandCallback, this, std::placeholders::_1));

  return true;
}

/// Update action goal
void HrhGripperSetDistanceAction::UpdateActionImpl(
    const tmc_control_msgs::action::GripperSetDistance::Goal& goal) {
  SetCommandValue(goal.distance);
}

/// Calculate target position from error between opening width command value and current value
double HrhGripperSetDistanceAction::GetCommandPos(const double current_distance) {
  // Calculate error
  const double ref_distance = *(goal_buffer_.readFromRT());
  const double error = ref_distance - current_distance;

  // Motion command
  integrated_distance_error_ += error;
  double command_position = controller_->GetCurrentPosition()
                            + distance_control_pgain_ * error
                            + distance_control_igain_ * integrated_distance_error_
                            + distance_control_dgain_ * (error - last_error_);

  last_error_ = error;

  return std::max(std::min(command_position, hand_motor_joint_max_), hand_motor_joint_min_);
}

/// Success judgment
void HrhGripperSetDistanceAction::CheckForSuccess(const rclcpp::Time& time, const double current_distance) {
  double current_velocity = controller_->GetCurrentVelocity();
  if (fabs(current_velocity) > stall_velocity_threshold_) {
    // Determine as moving and update the last moved time
    last_movement_time_ = time;
  } else if ((time - last_movement_time_).seconds() > distance_control_stall_timeout_) {
    // Determine as stall state
    auto result = std::make_shared<tmc_control_msgs::action::GripperSetDistance::Result>();
    result->distance = current_distance;
    result->stalled = true;

    // Stop motion
    stop_flag_buffer_.writeFromNonRT(true);

    const auto active_goal = *goal_handle_buffer_.readFromNonRT();
    if (!active_goal) {
      return;
    }

    double command = *(goal_buffer_.readFromRT());
    if (fabs(command - current_distance) < goal_tolerance_) {
      active_goal->gh_->succeed(result);
    } else {
      active_goal->gh_->abort(result);
    }

    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

/// Callback when opening width command is received via topic
void HrhGripperSetDistanceAction::DistanceCommandCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  controller_->PreemptActiveGoal();
  controller_->ChangeControlMode(shared_from_this());
  SetCommandValue(msg->data);
}

/// Set command
void HrhGripperSetDistanceAction::SetCommandValue(const double distance) {
  goal_buffer_.writeFromNonRT(std::max(std::min(distance, distance_max_), distance_min_));
  stop_flag_buffer_.writeFromNonRT(false);
  last_movement_time_ = node_->now();
  integrated_distance_error_ = 0.0;
  last_error_ = 0.0;
}

}  // namespace hsrb_gripper_controller
