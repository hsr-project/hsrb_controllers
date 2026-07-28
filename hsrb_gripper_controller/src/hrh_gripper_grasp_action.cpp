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
#include "hsrb_gripper_controller/hrh_gripper_grasp_action.hpp"

#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

#include "hsrb_gripper_controller/hrh_gripper_controller.hpp"

namespace {
// Default torque error threshold [Nm]
const double kDefaultTorqueGoalTolerance = 1.0;

}  // unnamed namespace

namespace hsrb_gripper_controller {

HrhGripperGraspAction::HrhGripperGraspAction(HrhGripperController* controller)
    : HrhGripperAction(controller, "/grasp", tmc_exxx_servo_motor_protocol::kDriveModeHandGrasp),
      is_sent_start_grasping_(false),
      goal_tolerance_(kDefaultTorqueGoalTolerance) {}

/// Periodic update process
void HrhGripperGraspAction::Update(const rclcpp::Time& time) {
  if (!IsActive()) {
    return;
  }
  bool grasping_flag = controller_->GetCurrentGraspingFlag();
  {
    std::lock_guard<std::mutex> guard(mutex_);
    bool start_grasping_flag;
    if (grasping_flag) {
      // Gripping start flag already ON
      start_grasping_flag = false;
      is_sent_start_grasping_ = true;
    } else {
      if (is_sent_start_grasping_) {
        // Gripping start flag sent, and gripping completed
        start_grasping_flag = false;
      } else {
        // Gripping start flag not yet sent
        start_grasping_flag = true;
      }
    }
    controller_->SetGraspCommand(start_grasping_flag, command_torque_);
  }
  CheckForSuccess();
}

trajectory_msgs::msg::JointTrajectoryPoint HrhGripperGraspAction::GetReferenceState() {
  trajectory_msgs::msg::JointTrajectoryPoint reference;

  reference.positions = { controller_->GetCurrentPosition() };
  reference.effort = { command_torque_ };

  return reference;
}

trajectory_msgs::msg::JointTrajectoryPoint HrhGripperGraspAction::GetFeedbackState() {
  trajectory_msgs::msg::JointTrajectoryPoint feedback;

  feedback.positions = { controller_->GetCurrentPosition() };
  feedback.velocities = { controller_->GetCurrentVelocity() };
  feedback.effort = { controller_->GetCurrentTorque() };

  return feedback;
}

/// Implementation of action initialization
bool HrhGripperGraspAction::InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  goal_tolerance_ = GetPositiveParameter(node, "torque_goal_tolerance", kDefaultTorqueGoalTolerance);
  return true;
}

/// Update the action target
void HrhGripperGraspAction::UpdateActionImpl(const tmc_control_msgs::action::GripperApplyEffort::Goal& goal) {
  std::lock_guard<std::mutex> guard(mutex_);
  command_torque_ = goal.effort;
  is_sent_start_grasping_ = false;
}

void HrhGripperGraspAction::CheckForSuccess() {
  bool grasping_flag = controller_->GetCurrentGraspingFlag();
  // Setting the gripping start flag on the control table initiates gripping,
  // The flag is reset upon stalling (command and status confirmation fields are the same)
  // If the gripping start flag has been sent and the current gripping flag is reset, gripping is complete.
  bool has_completed;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    has_completed = is_sent_start_grasping_ && !grasping_flag;
  }
  if (has_completed) {
    auto result = std::make_shared<tmc_control_msgs::action::GripperApplyEffort::Result>();
    result->stalled = true;
    result->effort = controller_->GetCurrentTorque();

    // Compare the command value and the current value when stalled and in equilibrium
    bool is_succeeded;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      is_succeeded = fabs(command_torque_ - result->effort) < goal_tolerance_;
    }

    const auto active_goal = *goal_handle_buffer_.readFromNonRT();
    if (is_succeeded) {
      active_goal->gh_->succeed(result);
    } else {
      active_goal->gh_->abort(result);
    }
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

}  // namespace hsrb_gripper_controller
