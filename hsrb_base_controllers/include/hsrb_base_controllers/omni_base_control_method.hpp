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
/// @file omni_base_control_method.hpp
/// @brief Omnidirectional cart control mode class
#ifndef HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROL_METHOD_HPP_
#define HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROL_METHOD_HPP_

#include <memory>
#include <string>
#include <vector>

#include <boost/noncopyable.hpp>

#include <Eigen/Core>

#include <geometry_msgs/msg/twist.hpp>
#include <joint_trajectory_controller/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <hsrb_base_controllers/omni_base_state.hpp>
#include <hsrb_base_controllers/tolerances.hpp>


namespace hsrb_base_controllers {

/// Interface class for cart control methods
class IBaseControlMethod : private boost::noncopyable {
 public:
  using Ptr = std::shared_ptr<IBaseControlMethod>;

  virtual ~IBaseControlMethod() {}
  // Initialize
  virtual void Activate() = 0;
};


/// Cart speed tracking
class OmniBaseVelocityControl : public IBaseControlMethod {
 public:
  using Ptr = std::shared_ptr<OmniBaseVelocityControl>;

  explicit OmniBaseVelocityControl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
  virtual ~OmniBaseVelocityControl() = default;
  // Initialize
  void Activate() override;

  // Get command speed
  Eigen::Vector3d GetOutputVelocity();
  // Update command speed
  void UpdateCommandVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg);

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Exclusive control
  std::mutex command_mutex_;
  // Command speed
  Eigen::Vector3d command_velocity_;
  // Time when the last speed command value was received
  rclcpp::Time last_velocity_subscribed_time_;
  // Time threshold for determining speed command value interruption
  double command_timeout_;
};


/// Cart trajectory tracking
class OmniBaseTrajectoryControl : public IBaseControlMethod {
 public:
  using Ptr = std::shared_ptr<OmniBaseTrajectoryControl>;

  explicit OmniBaseTrajectoryControl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                     const std::vector<std::string>& cordinates);
  virtual ~OmniBaseTrajectoryControl() = default;
  // Initialize
  void Activate() override;

  // Update the trajectory being tracked, return true if a trajectory exists
  bool UpdateActiveTrajectory();
  // Get target state for trajectory tracking
  bool SampleDesiredState(const rclcpp::Time& time,
                          const std::vector<double>& current_positions,
                          const std::vector<double>& current_velocities,
                          trajectory_msgs::msg::JointTrajectoryPoint& desired_state,
                          bool& before_last_point,
                          double& time_from_point);
  // Validate input trajectory command
  bool ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) const;
  // Reset the currently tracked trajectory
  void ResetCurrentTrajectory();
  // End trajectory tracking if conditions are met
  void TerminateControl(const rclcpp::Time& time, const double current_velocity);
  // Check tolerances during trajectory tracking
  // Return a positive number to continue tracking, or an error code (0 ~ -5) from control_msgs/action/FollowJointTrajectory to stop tracking
  int32_t CheckTorelances(const ControllerState& state,
                          const bool before_last_point,
                          const double time_from_trajectory_end);
  // Update tracking trajectory
  virtual void AcceptTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
                                const Eigen::Vector3d& base_positions) = 0;

 protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Cart coordinate axis name
  std::vector<std::string> coordinate_names_;
  // Speed threshold for determining trajectory tracking completion
  double stop_velocity_threshold_;
  // Whether to connect to the existing desired trajectory when a new trajectory arrives
  // Variable names and behavior are aligned with JointTrajectoryController
  bool open_loop_control_;
  // Last sampled state
  rclcpp::Time last_sampled_time_;
  trajectory_msgs::msg::JointTrajectoryPoint last_command_state_;
  // Whether last_command_state_ contains a value
  // It is correct to input the current value during activation like JointTrajectoryController
  // Implement with flag management to avoid extensive changes
  bool has_last_command_state_;

  std::shared_ptr<joint_trajectory_controller::Trajectory>* trajectory_active_ptr_ = nullptr;
  std::shared_ptr<joint_trajectory_controller::Trajectory> trajectory_ptr_ = nullptr;
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory::SharedPtr>  trajectory_msg_buffer_;

  // Tolerance width for trajectory tracking
  SegmentTolerances default_tolerances_;
  SegmentTolerances active_tolerances_;
};

class OmniBaseOdomTrajectoryControl : public OmniBaseTrajectoryControl {
 public:
  using Ptr = std::shared_ptr<OmniBaseOdomTrajectoryControl>;

  explicit OmniBaseOdomTrajectoryControl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                        const std::vector<std::string>& cordinates);
  virtual ~OmniBaseOdomTrajectoryControl() = default;

  // Update tracking trajectory
  void AcceptTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
                        const Eigen::Vector3d& base_positions) override;
  // Get command speed
  Eigen::Vector3d GetOutputVelocity(const ControllerState& base_state);
  // End trajectory tracking if conditions are met
  void TerminateControl(const rclcpp::Time& time, const ControllerState& base_state);

 private:
  // Feedback gain for control
  Eigen::Vector3d feedback_gain_;
};

class OmniBaseRollTrajectoryControl : public OmniBaseTrajectoryControl {
 public:
  using Ptr = std::shared_ptr<OmniBaseRollTrajectoryControl>;

  explicit OmniBaseRollTrajectoryControl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                         const std::vector<std::string>& cordinates);
  virtual ~OmniBaseRollTrajectoryControl() = default;

  // Update tracking trajectory
  void AcceptTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
                        const Eigen::Vector3d& /* base_positions */) override;
  // End trajectory tracking if conditions are met
  void TerminateControl(const rclcpp::Time& time, const ControllerState& joint_state);
};

}  // namespace hsrb_base_controllers

#endif /*HSRB_BASE_CONTROLLERS_OMNI_BASE_CONTROL_METHOD_HPP_*/
