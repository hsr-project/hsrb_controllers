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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_DISTANCE_TRAJECTORY_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_DISTANCE_TRAJECTORY_ACTION_HPP_
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <joint_trajectory_controller/trajectory.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"
#include "hsrb_gripper_controller/hrh_gripper_distance.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperFollowDistanceTrajectoryAction
/// @brief Hrh Opening Width Trajectory Following Action Class
class HrhGripperFollowDistanceTrajectoryAction
    : public HrhGripperAction<control_msgs::action::FollowJointTrajectory> {
 public:
  /// Constructor
  /// @param [in] controller Parent controller
  explicit HrhGripperFollowDistanceTrajectoryAction(HrhGripperController* controller);
  virtual ~HrhGripperFollowDistanceTrajectoryAction() = default;

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

  /// Callback when an opening width trajectory command is received via topic
  /// @param [in] msg Trajectory
  void DistanceTrajectoryCommandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  /// Create a trajectory with upper and lower limits
  /// @param [in] msg Opening width trajectory
  void LimitTrajectory(const trajectory_msgs::msg::JointTrajectory& distance_trajectory);

  /// Success determination
  /// @param [in] time             Current time
  /// @param [in] current_distance Current opening width
  void CheckForSuccess(const rclcpp::Time& time, const double current_distance);

  /// Default allowable error for goal position [m]
  double default_goal_tolerance_;
  /// Default allowable error for goal arrival time [s]
  double default_goal_time_tolerance_;
  // Maximum opening width [m]
  double distance_max_;
  // Minimum opening width [m]
  double distance_min_;

  // Whether to connect to the existing desired when a new trajectory arrives
  // Variable names and behavior are aligned with JointTrajectoryController
  bool open_loop_control_;
  // Last sampled state
  rclcpp::Time last_sampled_time_;
  std::optional<trajectory_msgs::msg::JointTrajectoryPoint> last_command_state_;
  /// Trajectory command reception
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr distance_trajectory_command_sub_;

  /// Fingertip distance calculator
  HrhGripperDistanceCalculator::Ptr distance_calculator_;

  /// Hold the trajectory
  std::shared_ptr<joint_trajectory_controller::Trajectory>* trajectory_active_ptr_;
  std::shared_ptr<joint_trajectory_controller::Trajectory> trajectory_ptr_;
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory::SharedPtr> trajectory_msg_buffer_;

  /// Goal condition buffer
  realtime_tools::RealtimeBuffer<GoalCondition> goal_condition_buffer_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_FOLLOW_DISTANCE_TRAJECTORY_ACTION_HPP_
