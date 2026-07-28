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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_APPLY_FORCE_ACTION_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_APPLY_FORCE_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>
#include <tmc_control_msgs/action/gripper_apply_effort.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace hsrb_gripper_controller {

/// @class HrhGripperApplyForceCalculator
/// @brief Hrh gripper fingertip force calculation class
class HrhGripperApplyForceCalculator {
 public:
  using Ptr = std::shared_ptr<HrhGripperApplyForceCalculator>;
  /// Constructor
  HrhGripperApplyForceCalculator();
  /// Constructor
  /// @param [in] path Calibration file path
  HrhGripperApplyForceCalculator(const std::string& calibration_file_path, const rclcpp::Logger& logger);

  virtual ~HrhGripperApplyForceCalculator() = default;

  /// Calculate by integrating left and right fingertip forces
  // @return Current fingertip force [N]
  double GetCurrentForce(double hand_motor_pos, double left_spring_proximal_joint_pos,
                         double right_spring_proximal_joint_pos) const;

 private:
  /// Load calibration data for force control
  void LoadForceCalibrationData(const std::string& path, const rclcpp::Logger& logger);

  /// Calculate the current fingertip force by comparing it with internal force
  /// @return Current fingertip force [N]
  double CalculateForce(double hand_motor_pos, double spring_proximal_joint_pos,
                        const std::vector<std::vector<double> >& calib_data) const;

  /// Calculate the gripper's internal force from calibration data
  /// @return Internal force [N]
  double CalculateInternalForce(double hand_motor_pos, const std::vector<double>& calib_p0,
                                const std::vector<double>& calib_p1) const;

  /// Fingertip force calibration data [N]
  std::vector<std::vector<double>> hand_left_force_calib_data_;
  std::vector<std::vector<double>> hand_right_force_calib_data_;

  /// Spring constant of finger base joint [Nm/rad]
  double hand_spring_coeff_;

  /// Finger length [m]
  double arm_length_;
};


/// @class HrhGripperApplyForceAction
/// @brief Hrh gripper force control action class
class HrhGripperApplyForceAction : public HrhGripperAction<tmc_control_msgs::action::GripperApplyEffort> {
 public:
  /// Constructor
  /// @param [in] controller Parent controller
  explicit HrhGripperApplyForceAction(HrhGripperController* controller);
  virtual ~HrhGripperApplyForceAction() = default;

  void Update(const rclcpp::Time& time) override;

  void PreemptActiveGoal() override;

  trajectory_msgs::msg::JointTrajectoryPoint GetReferenceState() override;
  trajectory_msgs::msg::JointTrajectoryPoint GetFeedbackState() override;

 private:
  /// Implementation of action initialization
  bool InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) override;
  /// Update action target
  void UpdateActionImpl(const tmc_control_msgs::action::GripperApplyEffort::Goal& goal) override;

  /// Tolerance error for goal force [N]
  double goal_tolerance_;
  /// Stall detection speed threshold [rad/s]
  double stall_velocity_threshold_;
  /// Stall detection time [s]
  double stall_timeout_;

  /// PID gain for force control
  double force_control_pgain_;
  double force_control_igain_;
  double force_control_dgain_;

  /// Error integral accumulation limit for I control
  double force_ierr_max_;
  /// Error integral accumulation buffer for I control
  double force_ierr_buff_;

  /// Low-pass filter coefficient for fingertip force
  double force_lpf_coeff_;
  /// Low-pass filter buffer for fingertip force [N]
  double force_lpf_buff_;

  /// Fingertip force calculator
  HrhGripperApplyForceCalculator::Ptr force_calculator_;

  /// Command value buffer
  realtime_tools::RealtimeBuffer<double> command_buffer_;
  /// Action continuation flag buffer
  realtime_tools::RealtimeBuffer<bool> stop_flag_buffer_;

  /// Calculate target position from the error between command value and current fingertip force
  /// @return Target position
  double GetCommandPos();
  /// Current fingertip force after passing through low-pass filter [N]
  double current_force_lpf_;
  /// Current target position
  double current_command_pos_;

  /// Action success/failure determination
  /// @param [in] time Current time
  void CheckForSuccess(const rclcpp::Time& time);
  /// Last operation time
  rclcpp::Time last_movement_time_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_APPLY_FORCE_ACTION_HPP_
