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
/// @file controller_command_interface.hpp
/// @brief Interface class connecting controller and command values from ROS
#ifndef HSRB_BASE_CONTROLLERS_CONTROLLER_COMMAND_INTERFACE_HPP_
#define HSRB_BASE_CONTROLLERS_CONTROLLER_COMMAND_INTERFACE_HPP_

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <joint_trajectory_controller/tolerances.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace hsrb_base_controllers {

class OmniBaseTrajectoryControl;

class IControllerCommandInterface {
 public:
  using Ptr = std::shared_ptr<IControllerCommandInterface>;

  virtual ~IControllerCommandInterface() = default;

  // Returns whether commands can be accepted
  virtual bool IsAcceptable() = 0;

  // Sets input velocity command
  virtual void UpdateVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) = 0;

  // Validates input trajectory command
  virtual bool ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) = 0;
  // Sets input trajectory command
  virtual void UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory) = 0;
  // // TODO(Takeshita) アクションのgoalに含まれるtolerancesを扱うための関数を追加する
  // // Sets input trajectory command and tolerances
  // virtual void UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
  //                               const joint_trajectory_controller::SegmentTolerances& tolerances) = 0;
  // Resets input trajectory
  virtual void ResetTrajectory(const std::shared_ptr<OmniBaseTrajectoryControl>& no_reset_control) = 0;
  // Cancels the currently executing goal
  virtual void PreemptActiveGoal() = 0;
};

}  // namespace hsrb_base_controllers

#endif /*HSRB_BASE_CONTROLLERS_CONTROLLER_COMMAND_INTERFACE_HPP_*/
