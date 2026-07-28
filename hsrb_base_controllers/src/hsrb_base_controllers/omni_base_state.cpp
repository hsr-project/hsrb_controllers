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
/// @file omni_base_state.cpp
/// @brief Class for the state of an omnidirectional cart
#include <hsrb_base_controllers/omni_base_state.hpp>

#include <string>
#include <vector>

#include <angles/angles.h>

#include "utils.hpp"

namespace {
// Cart state publish frequency [Hz]
const double kDefaultStatePublishRate = 50.0;

void CopyVector(const Eigen::Vector3d& input, std::vector<double>& dst_vector) {
  // dst_vector has already been resized
  for (auto i = 0; i < 3; ++i) {
    dst_vector[i] = input[i];
  }
}

void SetError(const std::vector<double>& desired,
              const std::vector<double>& actual,
              const uint32_t size,
              std::vector<double>& error_out) {
  // Already resized
  for (uint32_t i = 0; i < size; ++i) {
    error_out[i] = desired[i] - actual[i];
  }
}

}  // namespace

namespace hsrb_base_controllers {

void ControllerState::UpdateError() {
  if (actual.positions.size() == desired.positions.size()) {
    error.positions.resize(actual.positions.size());
    for (uint32_t i = 0; i < actual.positions.size(); ++i) {
      error.positions[i] = desired.positions[i] - actual.positions[i];
    }
  } else {
    error.positions.clear();
  }

  if (actual.velocities.size() == desired.velocities.size()) {
    error.velocities.resize(actual.velocities.size());
    for (uint32_t i = 0; i < actual.velocities.size(); ++i) {
      error.velocities[i] = desired.velocities[i] - actual.velocities[i];
    }
  } else {
    error.velocities.clear();
  }

  if (actual.accelerations.size() == desired.accelerations.size()) {
    error.accelerations.resize(actual.accelerations.size());
    for (uint32_t i = 0; i < actual.accelerations.size(); ++i) {
      error.accelerations[i] = desired.accelerations[i] - actual.accelerations[i];
    }
  } else {
    error.accelerations.clear();
  }
}


ControllerJointRollState::ControllerJointRollState() {
  actual.positions.resize(1, 0.0);
  actual.velocities.resize(1, 0.0);
  desired.positions.resize(1, 0.0);
  desired.velocities.resize(1, 0.0);
  desired.accelerations.resize(1, 0.0);
  error.positions.resize(1, 0.0);
  error.velocities.resize(1, 0.0);
}

void ControllerJointRollState::Reset(const double actual_position,
                                     const double actual_velocity) {
  actual.positions[0] = actual_position;
  actual.velocities[0] = actual_velocity;

  desired.positions[0] = actual.positions[0];
  desired.velocities[0] = actual.velocities[0];
  desired.accelerations[0] = 0.0;

  error.positions[0] = 0.0;
  error.velocities[0] = 0.0;
}

void ControllerJointRollState::UpdateError() {
  error.positions[0] = angles::shortest_angular_distance(
      actual.positions[0], desired.positions[0]);
  error.velocities[0] = desired.velocities[0] - actual.velocities[0];
}


ControllerBaseState::ControllerBaseState() {
  actual.positions.resize(kNumBaseCoordinateIDs, 0.0);
  actual.velocities.resize(kNumBaseCoordinateIDs, 0.0);
  desired.positions.resize(kNumBaseCoordinateIDs, 0.0);
  desired.velocities.resize(kNumBaseCoordinateIDs, 0.0);
  desired.accelerations.resize(kNumBaseCoordinateIDs, 0.0);
  error.positions.resize(kNumBaseCoordinateIDs, 0.0);
  error.velocities.resize(kNumBaseCoordinateIDs, 0.0);
}

void ControllerBaseState::Reset(const Eigen::Vector3d& actual_positions,
                                const Eigen::Vector3d& actual_velocities,
                                const std::vector<double>& desired_positions,
                                const std::vector<double>& desired_velocities,
                                const std::vector<double>& desired_accelerations) {
  CopyVector(actual_positions, actual.positions);

  // TODO(Takeshita) ここで変換しているのが微妙だなぁ
  // Since base_velocity_ is based on base_footprint, convert it to the odom reference frame
  Eigen::Matrix3d rot_mat;
  rot_mat << cos(actual_positions[kIndexBaseTheta]), -sin(actual_positions[kIndexBaseTheta]), 0.0,
             sin(actual_positions[kIndexBaseTheta]), cos(actual_positions[kIndexBaseTheta]), 0.0,
             0.0, 0.0, 1.0;
  Eigen::Vector3d transformed_velocity(rot_mat * actual_velocities);
  CopyVector(transformed_velocity, actual.velocities);

  desired.positions = desired_positions;
  desired.velocities = desired_velocities;
  desired.accelerations = desired_accelerations;

  SetError(desired.positions, actual.positions, 2, error.positions);
  error.positions[2] = angles::shortest_angular_distance(actual.positions[2], desired.positions[2]);

  SetError(desired.velocities, actual.velocities, 3, error.velocities);
}

void ControllerBaseState::Reset(const Eigen::Vector3d& actual_positions,
                                const Eigen::Vector3d& actual_velocities) {
  // Copied and pasted, but want to reduce unnecessary processing
  CopyVector(actual_positions, actual.positions);

  Eigen::Matrix3d rot_mat;
  rot_mat << cos(actual_positions[kIndexBaseTheta]), -sin(actual_positions[kIndexBaseTheta]), 0.0,
             sin(actual_positions[kIndexBaseTheta]), cos(actual_positions[kIndexBaseTheta]), 0.0,
             0.0, 0.0, 1.0;
  Eigen::Vector3d transformed_velocity(rot_mat * actual_velocities);
  CopyVector(transformed_velocity, actual.velocities);

  desired = actual;

  std::fill(error.positions.begin(), error.positions.end(), 0.0);
  std::fill(error.velocities.begin(), error.velocities.end(), 0.0);
}

void ControllerBaseState::UpdateError() {
  ControllerState::UpdateError();

  if (error.positions.size() == kNumBaseCoordinateIDs) {
    error.positions[kIndexBaseTheta] = angles::shortest_angular_distance(
        actual.positions[kIndexBaseTheta], desired.positions[kIndexBaseTheta]);
  }
}


ControllerJointState::ControllerJointState() {
  actual.positions.resize(3, 0.0);
  actual.velocities.resize(3, 0.0);
  desired.positions.resize(3, 0.0);
  desired.velocities.resize(3, 0.0);
  error.positions.resize(3, 0.0);
  error.velocities.resize(3, 0.0);
}

void ControllerJointState::Reset(const Eigen::Vector3d& actual_positions,
                                 const Eigen::Vector3d& actual_velocities) {
  CopyVector(actual_positions, actual.positions);
  CopyVector(actual_velocities, actual.velocities);
  desired = actual;
  std::fill(error.positions.begin(), error.positions.end(), 0.0);
  std::fill(error.velocities.begin(), error.velocities.end(), 0.0);
}

void ControllerJointState::Reset(const Eigen::Vector3d& actual_positions,
                                 const Eigen::Vector3d& actual_velocities,
                                 double desired_yaw_position,
                                 const Eigen::Vector3d& desired_velocities) {
  CopyVector(actual_positions, actual.positions);
  CopyVector(actual_velocities, actual.velocities);

  desired.positions[2] = desired_yaw_position;
  CopyVector(desired_velocities, desired.velocities);

  error.positions[2] = desired_yaw_position - actual_positions[2];
  SetError(desired.velocities, actual.velocities, 3, error.velocities);
}

void ControllerJointState::UpdateError() {
  error.positions[2] = angles::shortest_angular_distance(actual.positions[2], desired.positions[2]);
  error.velocities[0] = desired.velocities[0] - actual.velocities[0];
  error.velocities[1] = desired.velocities[1] - actual.velocities[1];
  error.velocities[2] = desired.velocities[2] - actual.velocities[2];
}

void Convert(const ControllerState& in, const rclcpp::Time& stamp, const std::vector<std::string>& joint_names,
             control_msgs::msg::JointTrajectoryControllerState& out) {
  out.header.stamp = stamp;
  out.joint_names = joint_names;
  Convert(in.actual, out.feedback);
  Convert(in.desired, out.reference);
  Convert(in.error, out.error);
  Convert(in.output, out.output);
}

void ConvertAsControllerState(
    const ControllerState& in,
    const rclcpp::Time& stamp,
    const std::vector<std::string>& joint_names,
    control_msgs::msg::JointTrajectoryControllerState& out) {
  out.header.stamp = stamp;
  out.joint_names = joint_names;
  Convert(in.actual, out.feedback);
  Convert(in.desired, out.reference);
  Convert(in.error, out.error);
  Convert(in.output, out.output);
}

void Convert(const State& in, trajectory_msgs::msg::JointTrajectoryPoint& out) {
  out.positions = in.positions;
  out.velocities = in.velocities;
  out.accelerations = in.accelerations;
}


StatePublisherBase::StatePublisherBase(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                       const std::string& topic_name,
                                       const std::vector<std::string>& joint_names)
    : node_(node),
    joint_names_(joint_names),
    state_publish_period_(0, 0),
    last_state_published_time_(node->now()) {
  const double state_publish_rate = GetPositiveParameter(node, "state_publish_rate", kDefaultStatePublishRate);
  state_publish_period_ = rclcpp::Duration::from_seconds(1.0 / state_publish_rate);

  publisher_impl_ = node->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
      topic_name, rclcpp::SystemDefaultsQoS());
  publisher_ = std::make_unique<RealtimePublisher>(publisher_impl_);
}

void StatePublisherBase::Publish(const ControllerState& state, const rclcpp::Time& stamp) {
  if (stamp - last_state_published_time_ >= state_publish_period_) {
    last_state_published_time_ += state_publish_period_;
    const auto msg = publisher_->trylock();
    if (msg) {
      Convert(state, stamp, *msg);
      publisher_->unlockAndPublish();
    }
  }
}

void StatePublisher::Convert(
    const ControllerState& state,
    const rclcpp::Time& stamp,
    control_msgs::msg::JointTrajectoryControllerState& out) const {
  hsrb_base_controllers::Convert(state, stamp, joint_names_, out);
}

void ControllerStatePublisher::Convert(
    const ControllerState& state,
    const rclcpp::Time& stamp,
    control_msgs::msg::JointTrajectoryControllerState& out) const {
  hsrb_base_controllers::ConvertAsControllerState(state, stamp, joint_names_, out);
}

}  // namespace hsrb_base_controllers
