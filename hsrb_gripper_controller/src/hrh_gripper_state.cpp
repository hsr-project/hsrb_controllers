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
#include "hsrb_gripper_controller/hrh_gripper_state.hpp"

#include <rclcpp/rclcpp.hpp>

#include "hsrb_gripper_controller/hrh_gripper_action.hpp"

namespace {

// Trolley state publish frequency [Hz]
const double kDefaultStatePublishRate = 50.0;

void CalculateError(const std::vector<double>& reference,
                    const std::vector<double>& feedback,
                    std::vector<double>& error) {
  if (reference.size() == feedback.size()) {
    error.resize(feedback.size());
    for (uint32_t i = 0; i < feedback.size(); ++i) {
      error[i] = reference[i] - feedback[i];
    }
  } else {
    error.clear();
  }
}

void UpdateError(const trajectory_msgs::msg::JointTrajectoryPoint& reference,
                 const trajectory_msgs::msg::JointTrajectoryPoint& feedback,
                 trajectory_msgs::msg::JointTrajectoryPoint& error) {
  CalculateError(reference.positions, feedback.positions, error.positions);
  CalculateError(reference.velocities, feedback.velocities, error.velocities);
  CalculateError(reference.accelerations, feedback.accelerations, error.accelerations);
  CalculateError(reference.effort, feedback.effort, error.effort);
}

}  // namespace

namespace hsrb_gripper_controller {

StatePublisher::StatePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                               const std::string& topic_name,
                               const std::string& joint_name)
    : node_(node),
      joint_name_(joint_name),
      state_publish_period_(0, 0),
      last_state_published_time_(node->now()) {
  const double state_publish_rate = GetPositiveParameter(node, "state_publish_rate", kDefaultStatePublishRate);

  state_publish_period_ = rclcpp::Duration::from_seconds(1.0 / state_publish_rate);

  publisher_impl_ = node->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
      topic_name, rclcpp::SystemDefaultsQoS());
  publisher_ = std::make_unique<RealtimePublisher>(publisher_impl_);
}

void StatePublisher::Publish(const trajectory_msgs::msg::JointTrajectoryPoint& reference,
                             const trajectory_msgs::msg::JointTrajectoryPoint& feedback,
                             const rclcpp::Time& stamp) {
  if ((stamp - last_state_published_time_) >= state_publish_period_) {
    if (publisher_ && publisher_->trylock()) {
      publisher_->msg_.header.stamp = stamp;
      publisher_->msg_.joint_names = { joint_name_ };
      publisher_->msg_.reference = reference;
      publisher_->msg_.feedback = feedback;
      UpdateError(reference, feedback, publisher_->msg_.error);
      publisher_->unlockAndPublish();

      last_state_published_time_ += state_publish_period_;
    }
  }
}

}  // namespace hsrb_gripper_controller
