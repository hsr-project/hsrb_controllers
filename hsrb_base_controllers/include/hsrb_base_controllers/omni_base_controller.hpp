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
/// @file omni_base_controller.hpp
/// @brief Omnidirectional Cart Controller Class
#ifndef HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_
#define HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_

#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <hsrb_base_controllers/command_subscriber.hpp>
#include <hsrb_base_controllers/controller_command_interface.hpp>
#include <hsrb_base_controllers/omni_base_control_method.hpp>
#include <hsrb_base_controllers/omni_base_joint_controller.hpp>
#include <hsrb_base_controllers/omni_base_odometry.hpp>

#include "tolerances.hpp"

namespace hsrb_base_controllers {

/// Omnidirectional Cart Velocity Controller Class
class OmniBaseController
    : public controller_interface::ControllerInterface,
      public IControllerCommandInterface {
 public:
  OmniBaseController() = default;
  ~OmniBaseController() = default;

  // Controller Initialization
  controller_interface::CallbackReturn on_init() override;

  // ros2_control Interface Configuration
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Calculate and Update Cart Joint Angular Velocity
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // Function Called During Configure
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  // Function Called During Activate
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  // Function Called During Deactivate
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // Returns Whether Commands Can Be Accepted
  bool IsAcceptable() override;

  // Set Input Velocity Command
  void UpdateVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) override;

  // Validate Input Trajectory Command
  bool ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) override;
  // Set Input Trajectory Command
  void UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory) override;
  // Reset Input Trajectory
  void ResetTrajectory(const std::shared_ptr<OmniBaseTrajectoryControl>& no_reset_control) override;
  // Abort the Current Goal
  void PreemptActiveGoal() override;

 protected:
  // Initialization Other Than ControllerInterface::init, Split for Testing
  bool InitImpl();

  // Tolerance for Trajectory Tracking
  SegmentTolerances default_tolerances_;
  SegmentTolerances active_tolerances_;
  // Check Tolerances During Trajectory Tracking
  // Return a Positive Number to Continue Tracking, or an Error Code (0 ~ -5) from control_msgs/action/FollowJointTrajectory to Stop Tracking
  int32_t CheckTorelances(const ControllerBaseState& state, bool before_last_point, double time_from_trajectory_end);

  // Input Velocity Command Subscriber
  CommandVelocitySubscriber::Ptr velocity_subscriber_;
  // Cart Trajectory Command Subscriber
  CommandTrajectorySubscriber::Ptr odom_trajectory_subscriber_;
  // Turning Axis Trajectory Command Subscriber
  CommandTrajectorySubscriber::Ptr roll_trajectory_subscriber_;
  // Cart Trajectory Action Command Server
  TrajectoryActionServer::Ptr odom_trajectory_action_;
  // Turning Axis Trajectory Action Command Server
  TrajectoryActionServer::Ptr roll_trajectory_action_;

  // Joint Controller
  OmniBaseJointControllerBase::Ptr joint_controller_;

  // Cart Wheel Odometry Calculation Class
  BaseOdometry::Ptr base_odometry_;
  WheelOdometry::Ptr wheel_odometry_;

  // Class for Calculating Cart Command Velocity
  OmniBaseVelocityControl::Ptr velocity_control_;
  OmniBaseOdomTrajectoryControl::Ptr odom_trajectory_control_;
  OmniBaseRollTrajectoryControl::Ptr roll_trajectory_control_;

  // Publish Cart State
  StatePublisherBase::Ptr joint_state_publisher_;
  StatePublisherBase::Ptr base_state_publisher_;
  StatePublisherBase::Ptr base_controller_state_publisher_;

  // Name of Turning Axis
  std::string base_roll_joint_name_;

  // Controller State, Stored as a Member Variable to Avoid Delay in Update
  ControllerBaseState base_state_;
  ControllerJointState joint_state_;
  ControllerJointRollState roll_state_;
};

}  // namespace hsrb_base_controllers

#endif/*HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROLLER_HPP_*/
