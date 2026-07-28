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
#ifndef HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_DISTANCE_HPP_
#define HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_DISTANCE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <std_msgs/msg/float32.hpp>

namespace hsrb_gripper_controller {

/// @class HrhGripperDistanceCalculator
/// @brief Hrh gripper fingertip distance calculation class
class HrhGripperDistanceCalculator {
 public:
  using Ptr = std::shared_ptr<HrhGripperDistanceCalculator>;
  /// Constructor
  HrhGripperDistanceCalculator();

  /// Destructor
  virtual ~HrhGripperDistanceCalculator() = default;

  /// Initialize physical parameters of the hand
  bool InitializeHandSizeData(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

  /// Calculate fingertip distance
  /// @return Fingertip distance [m]
  double GetDistanceFromPosition(double hand_motor_pos) const;

  /// Calculate fingertip distance
  /// @return Fingertip distance [m]
  double GetDistanceFromPosition(
      double hand_motor_pos, double left_spring_proximal_joint_pos,
      double right_spring_proximal_joint_pos) const;

  /// Calculate joint angles from fingertip distance
  /// @return Joint angles [rad]
  double GetPositionFromDistance(double distance) const;

 private:
  /// Finger length [m]
  double proximal_to_distal_z_;
  double distance_palm_to_tip_;
};

class DistancePublisher {
 public:
  using Ptr = std::shared_ptr<DistancePublisher>;

  DistancePublisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                    const std::string& topic_name);

  void Publish(const double current_position, const rclcpp::Time& stamp);

  void SetLastStatePublishedTime(const rclcpp::Time& time) {
    last_distance_published_time_ = time;
  }

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Duration distance_publish_period_;

  using RealtimePublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float32>;
  std::unique_ptr<RealtimePublisher> publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_impl_;

  // Time when the last fingertip distance was issued
  rclcpp::Time last_distance_published_time_;

  /// Fingertip distance calculator
  HrhGripperDistanceCalculator::Ptr distance_calculator_;
};

}  // namespace hsrb_gripper_controller

#endif  // HSRB_GRIPPER_CONTROLLER_HRH_GRIPPER_DISTANCE_HPP_
