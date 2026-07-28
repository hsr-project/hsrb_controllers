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
/// @file omni_base_odometry.cpp
/// @brief Omnidirectional Cart Odometry Class
#include <string>

#include <tmc_utils/qos.hpp>

#include <hsrb_base_controllers/omni_base_odometry.hpp>
#include "utils.hpp"

namespace {
// Default publishing frequency of cart odometry [Hz]
constexpr double kDefaultOdometryPublishRate = 30.0;
// Default publishing frequency of cart odometry TF [Hz]
constexpr double kDefaultTransformPublishRate = 30.0;
}

namespace hsrb_base_controllers {

/// Initialization of the odometry calculation class
Odometry::Odometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    : odometry_(Eigen::Vector3d::Zero()),
      velocity_(Eigen::Vector3d::Zero()) {}


/// Initialization of the omnidirectional cart wheel odometry calculation class
BaseOdometry::BaseOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) : Odometry(node) {
  input_odom_ = std::make_shared<InputOdometry>(node);
}

/// Update the odometry
void BaseOdometry::UpdateOdometry(double period,
                                  const Eigen::Vector3d& positions,
                                  const Eigen::Vector3d& velocities) {
  auto current_odom = input_odom_->GetOdometry();
  odometry_<< current_odom.pose.pose.position.x,
              current_odom.pose.pose.position.y,
              2.0 * std::atan2(current_odom.pose.pose.orientation.z,
                               current_odom.pose.pose.orientation.w);
  // Since odometry like laser odometry may not calculate velocity
  // Input the velocity calculated within the controller
  velocity_ = velocities;
}

/// Initialize odometry data
void BaseOdometry::InitOdometry() {
  input_odom_->InitOdometry();
}

/// Initialization of the omnidirectional cart wheel odometry calculation class
WheelOdometry::WheelOdometry(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const OmniBaseSize& omnibase_size)
    : Odometry(node),
      last_odometry_published_time_(node->now()),
      last_transform_published_time_(node->now()),
      odometry_publish_period_(0, 0),
      transform_publish_period_(0, 0) {
  // Get the frame name related to cart odometry
  wheel_odom_frame_ = GetParameter(node, "wheel_odom_map_frame", "odom");
  wheel_base_frame_ = GetParameter(node, "wheel_odom_base_frame", "base_footprint_wheel");
  tf_prefix_ = GetParameter(node, "tf_prefix", "");
  // Set parameters for the omnidirectional cart model
  twin_drive_ = std::make_shared<TwinCasterDrive>(omnibase_size);

  // Set the odometry publisher
  auto odom_msg_init_func = [this](nav_msgs::msg::Odometry& msg) {
    // TODO(Takeshita) 厳密にはtf::resolveに相当する関数が必要
    msg.header.frame_id = tf_prefix_ + wheel_odom_frame_;
    msg.child_frame_id = tf_prefix_ + wheel_base_frame_;
  };
  odometry_publisher_impl_ = node->create_publisher<nav_msgs::msg::Odometry>(
      "~/wheel_odom", rclcpp::SystemDefaultsQoS());
  odometry_publisher_ = std::make_unique<OdometryPublisher>(odometry_publisher_impl_, odom_msg_init_func);

  transform_publisher_impl_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", tmc_utils::ReliableVolatileQoS());
  auto tf_msg_init_func = [this](tf2_msgs::msg::TFMessage& msg) {
    msg.transforms.resize(1);
    msg.transforms[0].header.frame_id = tf_prefix_ + wheel_odom_frame_;
    msg.transforms[0].child_frame_id = tf_prefix_ + wheel_base_frame_;
  };
  transform_publisher_ = std::make_unique<TFPublisher>(transform_publisher_impl_, tf_msg_init_func);

  // Get the publishing interval of cart odometry
  const double odometry_publish_rate = GetPositiveParameter(node, "odometry_publish_rate", kDefaultOdometryPublishRate);
  odometry_publish_period_ = rclcpp::Duration::from_seconds(1.0 / odometry_publish_rate);
  // Get the publishing interval of cart odometry TF
  const double transform_publish_rate = GetPositiveParameter(node, "transform_publish_rate",
                                                             kDefaultTransformPublishRate);
  transform_publish_period_ = rclcpp::Duration::from_seconds(1.0 / transform_publish_rate);
}

/// Update wheel odometry
void WheelOdometry::UpdateOdometry(double period,
                                   const Eigen::Vector3d& positions,
                                   const Eigen::Vector3d& velocities) {
  // Update odometry and cart velocity from wheel position and velocity
  odometry_ = twin_drive_->UpdateOdometry(period, positions, velocities);
  twin_drive_->Update(positions[kJointIDSteer]);
  velocity_ = twin_drive_->ConvertForward(velocities);
}


/// Publish odometry
void WheelOdometry::PublishOdometry(const rclcpp::Time& time) {
  const Eigen::Vector3d wheel_odometry = odometry_;
  const Eigen::Vector3d wheel_odom_velocity = velocity_;
  const Eigen::Quaterniond wheel_quat_trans(
      Eigen::AngleAxisd(wheel_odometry(kIndexBaseTheta), Eigen::Vector3d::UnitZ()));

  // Publish odometry topic
  if (time - last_odometry_published_time_ >= odometry_publish_period_) {
    last_odometry_published_time_ += odometry_publish_period_;
    // Publish wheel odometry topic
    const auto msg = odometry_publisher_->trylock();
    if (msg) {
      msg->header.stamp = time;
      // TODO(Takeshita) 厳密にはtf::resolveに相当する関数が必要
      msg->pose.pose.position.x = wheel_odometry(kIndexBaseX);
      msg->pose.pose.position.y = wheel_odometry(kIndexBaseY);
      msg->pose.pose.position.z = 0.0;
      msg->pose.pose.orientation.x = wheel_quat_trans.x();
      msg->pose.pose.orientation.y = wheel_quat_trans.y();
      msg->pose.pose.orientation.z = wheel_quat_trans.z();
      msg->pose.pose.orientation.w = wheel_quat_trans.w();
      msg->twist.twist.linear.x = wheel_odom_velocity(kIndexBaseX);
      msg->twist.twist.linear.y = wheel_odom_velocity(kIndexBaseY);
      msg->twist.twist.angular.z = wheel_odom_velocity(kIndexBaseTheta);
      odometry_publisher_->unlockAndPublish();
    }
  }

  // Publish odometry TF
  if (time - last_transform_published_time_ >= transform_publish_period_) {
    last_transform_published_time_ += transform_publish_period_;
    geometry_msgs::msg::Transform transform;
    transform.translation.x = wheel_odometry(kIndexBaseX);
    transform.translation.y = wheel_odometry(kIndexBaseY);
    transform.translation.z = 0.0;
    transform.rotation.x = wheel_quat_trans.x();
    transform.rotation.y = wheel_quat_trans.y();
    transform.rotation.z = wheel_quat_trans.z();
    transform.rotation.w = wheel_quat_trans.w();

    // Publish wheel odometry TF
    const auto msg = transform_publisher_->trylock();
    if (msg) {
      msg->transforms[0].header.stamp = time;
      msg->transforms[0].transform = transform;
      transform_publisher_->unlockAndPublish();
    }
  }
}

}  // namespace hsrb_base_controllers
