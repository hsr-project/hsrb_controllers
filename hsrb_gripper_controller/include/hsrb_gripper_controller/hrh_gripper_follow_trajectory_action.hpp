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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_TRAJECTORY_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_TRAJECTORY_ACTION_HPP_
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <joint_trajectory_controller/trajectory.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperFollowTrajectoryAction
/// @brief Hrh trajectory tracking action class
class HrhGripperFollowTrajectoryAction : public HrhGripperAction<control_msgs::action::FollowJointTrajectory> {
 public:
  /// Constructor
  /// @param [in] controller Parent controller
  explicit HrhGripperFollowTrajectoryAction(HrhGripperController* controller);
  virtual ~HrhGripperFollowTrajectoryAction() = default;

  void Update(const rclcpp::Time& time) override;

  void PreemptActiveGoal() override;

  trajectory_msgs::msg::JointTrajectoryPoint GetReferenceState() override;
  trajectory_msgs::msg::JointTrajectoryPoint GetFeedbackState() override;

 protected:
  /// Implementation of action initialization
  bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) override;
  /// Check if the goal is acceptable
  bool ValidateGoal(const control_msgs::action::FollowJointTrajectory::Goal& goal) override;
  /// Update the action goal
  void UpdateActionImpl(const control_msgs::action::FollowJointTrajectory::Goal& goal) override;
  /// Get the current joint position of the controlled target
  /// @return Joint position
  double GetPosition() const;

  /// Default allowable error for goal position [rad]
  double default_goal_tolerance_;
  /// Default allowable error for goal arrival time [s]
  double default_goal_time_tolerance_;

  // Whether to connect from the existing desired when a new trajectory arrives
  // Variable names and behavior are aligned with JointTrajectoryController
  bool open_loop_control_;
  /// Flag to control using the output axis corrected for spring compensation
  bool do_output_position_control_;
  /// Lower limit of current when closing [A], if 0.0 or more, no correction of command value due to overcurrent
  double current_min_;
  /// Incremental correction value to increase (open) the command value during overcurrent when closing
  double position_correction_incresing_step_;
  /// Incremental value to revert (decrease) the correction value when not in overcurrent; smaller than increasing is more stable
  double position_correction_decresing_step_;
  /// Correction value for the command value
  double position_correction_value_;

  // Last sampled state
  rclcpp::Time last_sampled_time_;
  std::optional<trajectory_msgs::msg::JointTrajectoryPoint> last_command_state_;

  /// Callback when a trajectory command is received via topic
  /// @param [in] msg Trajectory
  void TrajectoryCommandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  /// Trajectory command received
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_command_sub_;

  /// Hold the trajectory
  std::shared_ptr<joint_trajectory_controller::Trajectory>* trajectory_active_ptr_;
  std::shared_ptr<joint_trajectory_controller::Trajectory> trajectory_ptr_;
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory::SharedPtr> trajectory_msg_buffer_;

  /// @struct GoalCondition
  /// @brief Goal conditions
  struct GoalCondition {
    /// Command position [rad]
    double position;
    /// Target arrival time
    rclcpp::Time expected_arrival_time;
    /// Time to stop trajectory tracking
    rclcpp::Time abort_time;
    /// Allowable error for goal arrival time
    rclcpp::Time goal_time_tolerance;
    /// Allowable error for goal position
    double goal_tolerance;
  };
  /// Buffer for goal conditions
  realtime_tools::RealtimeBuffer<GoalCondition> goal_condition_buffer_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_TRAJECTORY_ACTION_HPP_
