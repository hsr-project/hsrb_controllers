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
/// @file omni_base_state.hpp
/// @brief Omnidirectional cart state class
#ifndef HSRB_BASE_CONTROLLERS_OMNI_BASE_STATE_HPP_
#define HSRB_BASE_CONTROLLERS_OMNI_BASE_STATE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <tmc_realtime_tools/realtime_publisher.hpp>

#include <hsrb_base_controllers/twin_caster_drive.hpp>


namespace hsrb_base_controllers {

struct State {
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

struct ControllerState {
  State actual;
  State desired;
  State output;
  State error;

  virtual ~ControllerState() = default;
  virtual void UpdateError();
};

struct ControllerJointRollState : public ControllerState {
  ControllerJointRollState();

  void Reset(const double actual_position, const double actual_velocity);

  void UpdateError() override;
};

struct ControllerBaseState : public ControllerState {
  ControllerBaseState();

  void Reset(const Eigen::Vector3d& actual_positions,
             const Eigen::Vector3d& actual_velocities,
             const std::vector<double>& desired_positions,
             const std::vector<double>& desired_velocities,
             const std::vector<double>& desired_accelerations);

  void Reset(const Eigen::Vector3d& actual_positions,
             const Eigen::Vector3d& actual_velocities);

  void UpdateError() override;
};

struct ControllerJointState : public ControllerState {
  ControllerJointState();

  void Reset(const Eigen::Vector3d& actual_positions,
             const Eigen::Vector3d& actual_velocities);

  void Reset(const Eigen::Vector3d& actual_positions,
             const Eigen::Vector3d& actual_velocities,
             double desired_yaw_position,
             const Eigen::Vector3d& desired_velocities);

  void UpdateError() override;
};

void Convert(const ControllerState& in, const rclcpp::Time& stamp, const std::vector<std::string>& joint_names,
             control_msgs::msg::JointTrajectoryControllerState& out);

void ConvertAsControllerState(
     const ControllerState& in,
     const rclcpp::Time& stamp,
     const std::vector<std::string>& joint_names,
     control_msgs::msg::JointTrajectoryControllerState& out);

void Convert(const State& in, trajectory_msgs::msg::JointTrajectoryPoint& out);

class StatePublisherBase {
 public:
  using Ptr = std::shared_ptr<StatePublisherBase>;

  StatePublisherBase(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                     const std::string& topic_name,
                     const std::vector<std::string>& joint_names);

  virtual ~StatePublisherBase() = default;

  void Publish(const ControllerState& state, const rclcpp::Time& stamp);

  void set_last_state_published_time(const rclcpp::Time& time) {
    last_state_published_time_ = time;
  }

 protected:
  std::vector<std::string> joint_names_;

 private:
  virtual void Convert(const ControllerState& state,
                       const rclcpp::Time& stamp,
                       control_msgs::msg::JointTrajectoryControllerState& out) const = 0;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Duration state_publish_period_;

  using RealtimePublisher = tmc_realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>;
  std::unique_ptr<RealtimePublisher> publisher_;
  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr publisher_impl_;

  // The time when the state was last issued
  rclcpp::Time last_state_published_time_;
};

class StatePublisher : public StatePublisherBase {
 public:
  StatePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                 const std::string& topic_name,
                 const std::vector<std::string>& joint_names)
      : StatePublisherBase(node, topic_name, joint_names) {}

 private:
  void Convert(const ControllerState& state,
               const rclcpp::Time& stamp,
               control_msgs::msg::JointTrajectoryControllerState& out) const override;
};

class ControllerStatePublisher : public StatePublisherBase {
 public:
  ControllerStatePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                           const std::string& topic_name,
                           const std::vector<std::string>& joint_names)
      : StatePublisherBase(node, topic_name, joint_names) {}

 private:
  void Convert(const ControllerState& state,
               const rclcpp::Time& stamp,
               control_msgs::msg::JointTrajectoryControllerState& out) const override;
};

}  // namespace hsrb_base_controllers

#endif /*HSRB_BASE_CONTROLLERS_OMNI_BASE_STATE_HPP_*/
