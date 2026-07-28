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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_SET_DISTANCE_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_SET_DISTANCE_ACTION_HPP_
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <std_msgs/msg/float32.hpp>
#include <tmc_control_msgs/action/gripper_set_distance.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"
#include "hsrb_gripper_controller/hrh_gripper_distance.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperSetDistanceAction
/// @brief Hrh fingertip distance setting action class
class HrhGripperSetDistanceAction : public HrhGripperAction<tmc_control_msgs::action::GripperSetDistance> {
 public:
  /// Constructor
  /// @param [in] controller Parent controller
  explicit HrhGripperSetDistanceAction(HrhGripperController* controller);
  virtual ~HrhGripperSetDistanceAction() = default;

  void Update(const rclcpp::Time& time) override;

  void PreemptActiveGoal() override;

  trajectory_msgs::msg::JointTrajectoryPoint GetReferenceState() override;
  trajectory_msgs::msg::JointTrajectoryPoint GetFeedbackState() override;

 protected:
  /// Implementation of action initialization
  bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) override;
  /// Update the action goal
  void UpdateActionImpl(const tmc_control_msgs::action::GripperSetDistance::Goal& goal) override;

  /// Calculate the target position from the error between the commanded and current opening width
  /// @param [in] current_distance Current opening width
  /// @return Target position
  double GetCommandPos(const double current_distance);

  /// Success determination
  /// @param [in] time             Current time
  /// @param [in] current_distance Current opening width
  void CheckForSuccess(const rclcpp::Time& time, const double current_distance);

  /// Callback when the opening width command is received via topic
  /// @param [in] msg Opening width
  void DistanceCommandCallback(const std_msgs::msg::Float32::SharedPtr msg);

  /// Set the command
  /// @param [in] distance  Opening width
  /// @param [in] stop_flag Control stop flag
  void SetCommandValue(const double distance);

  /// Allowable error for goal position [m]
  double goal_tolerance_;
  /// Velocity threshold for stall determination [rad/s]
  double stall_velocity_threshold_;
  /// Time for stall determination [s]
  double distance_control_stall_timeout_;
  /// Opening width control P gain
  double distance_control_pgain_;
  /// Opening width control I gain
  double distance_control_igain_;
  /// Opening width control D gain
  double distance_control_dgain_;
  /// Maximum angle [rad]
  double hand_motor_joint_max_;
  /// Minimum angle [rad]
  double hand_motor_joint_min_;

  // Maximum opening width [m]
  double distance_max_;
  // Minimum opening width [m]
  double distance_min_;
  // Integral value of error
  double integrated_distance_error_;
  // Previous error value
  double last_error_;

  // Last operating time
  rclcpp::Time last_movement_time_;

  /// Current target position
  double current_command_pos_;

  /// Opening width command reception
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_command_sub_;

  /// Fingertip distance calculator
  HrhGripperDistanceCalculator::Ptr distance_calculator_;

  /// Goal buffer
  realtime_tools::RealtimeBuffer<double> goal_buffer_;

  /// Operation stop flag buffer
  realtime_tools::RealtimeBuffer<bool> stop_flag_buffer_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_SET_DISTANCE_ACTION_HPP_
