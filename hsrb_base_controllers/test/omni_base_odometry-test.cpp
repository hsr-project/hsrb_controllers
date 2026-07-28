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
/// @file omni_base_odometry-test.cpp
/// @brief Test class for omnidirectional cart odometry

#include <gtest/gtest.h>

#include <hsrb_base_controllers/omni_base_odometry.hpp>

#include "utils.hpp"

namespace {
constexpr double kEpsilon = 1.0e-5;
}  // namespace

namespace hsrb_base_controllers {

class BaseOdometryTest : public ::testing::Test {
 public:
  void SetUp() override;

 protected:
  BaseOdometry::Ptr odom_;
};

void BaseOdometryTest::SetUp() {
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  auto publisher = node->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SystemDefaultsQoS());
  odom_ = std::make_shared<BaseOdometry>(node);
  odom_->InitOdometry();
  node->activate();

  nav_msgs::msg::Odometry msg;
  msg.pose.pose.position.x = 10.0;
  msg.pose.pose.position.y = 20.0;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.0;
  msg.pose.pose.orientation.w = 1.0;

  publisher->publish(msg);
  auto timeout = TimeoutDetection(node->get_clock());
  while (true) {
    timeout.Run();
    rclcpp::spin_some(node->get_node_base_interface());
    odom_->UpdateOdometry(0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 2.0, 3.0));
    if (odom_->odometry()[0] > 0.0) {
      break;
    }
  }
}

/// Update odometry using external odometry data
TEST_F(BaseOdometryTest, UpdateBaseOdometry) {
  // Cart odometry
  Eigen::Vector3d odom = odom_->odometry();
  EXPECT_DOUBLE_EQ(odom(kIndexBaseX), 10.0);
  EXPECT_DOUBLE_EQ(odom(kIndexBaseY), 20.0);
  EXPECT_DOUBLE_EQ(odom(kIndexBaseTheta), 0.0);

  // Cart velocity
  Eigen::Vector3d vel = odom_->velocity();
  EXPECT_DOUBLE_EQ(vel(kIndexBaseX), 1.0);
  EXPECT_DOUBLE_EQ(vel(kIndexBaseY), 2.0);
  EXPECT_DOUBLE_EQ(vel(kIndexBaseTheta), 3.0);
}

/// Initialize odometry data
TEST_F(BaseOdometryTest, InitOdometry) {
  odom_->InitOdometry();
  odom_->UpdateOdometry(0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 2.0, 3.0));

  // Cart odometry
  Eigen::Vector3d odom = odom_->odometry();
  EXPECT_DOUBLE_EQ(odom(kIndexBaseX), 0.0);
  EXPECT_DOUBLE_EQ(odom(kIndexBaseY), 0.0);
  EXPECT_DOUBLE_EQ(odom(kIndexBaseTheta), 0.0);

  // Cart velocity
  Eigen::Vector3d vel = odom_->velocity();
  EXPECT_DOUBLE_EQ(vel(kIndexBaseX), 1.0);
  EXPECT_DOUBLE_EQ(vel(kIndexBaseY), 2.0);
  EXPECT_DOUBLE_EQ(vel(kIndexBaseTheta), 3.0);
}


class WheelOdometryTest : public ::testing::Test {
 public:
  void SetUp() override;

 protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  WheelOdometry::Ptr odom_;
};

void WheelOdometryTest::SetUp() {
  OmniBaseSize omnibase_size;
  omnibase_size.tread = 0.266000;
  omnibase_size.caster_offset = 0.110000;
  omnibase_size.wheel_radius = 0.040000;

  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node_->configure();
  node_->declare_parameter("odometry_publish_rate", 2.0);
  node_->declare_parameter("transform_publish_rate", 4.0);
  node_->activate();

  odom_ = std::make_shared<WheelOdometry>(node_, omnibase_size);
  odom_->UpdateOdometry(0.1, Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(-1.0, -2.0, -3.0));
}

/// Update and retrieve odometry
TEST_F(WheelOdometryTest, UpdateWheelOdometry) {
  // Cart odometry
  Eigen::Vector3d odom = odom_->odometry();
  EXPECT_NEAR(odom(kIndexBaseX), 0.00573091, kEpsilon);
  EXPECT_NEAR(odom(kIndexBaseY), -0.00239658, kEpsilon);
  EXPECT_NEAR(odom(kIndexBaseTheta), 0.0150376, kEpsilon);

  // Cart velocity
  Eigen::Vector3d vel = odom_->velocity();
  EXPECT_NEAR(vel(kIndexBaseX), 0.0570652, kEpsilon);
  EXPECT_NEAR(vel(kIndexBaseY), -0.024843, kEpsilon);
  EXPECT_NEAR(vel(kIndexBaseTheta), 3.15038, kEpsilon);
}

/// Publish odometry
TEST_F(WheelOdometryTest, PublishWheelOdometry) {
  auto client_node = rclcpp::Node::make_shared("client_node");
  auto counter = std::make_shared<SubscriptionCounter<nav_msgs::msg::Odometry>>(client_node, "test_node/wheel_odom");
  odom_->set_last_odometry_published_time(node_->now());

  rclcpp::WallRate loop_rate(10.0);
  for (uint32_t i = 0; i < 12; ++i) {
    odom_->PublishOdometry(node_->now());
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(node_->get_node_base_interface());
    loop_rate.sleep();
  }
  EXPECT_EQ(counter->count(), 2);

  auto msg = counter->last_msg();
  EXPECT_EQ(msg.header.frame_id, "odom");
  EXPECT_EQ(msg.child_frame_id, "base_footprint_wheel");
  EXPECT_NEAR(msg.pose.pose.position.x, 0.00573091, kEpsilon);
  EXPECT_NEAR(msg.pose.pose.position.y, -0.00239658, kEpsilon);
  EXPECT_NEAR(msg.pose.pose.orientation.z, std::sin(0.0150376 / 2.0), kEpsilon);
  EXPECT_NEAR(msg.pose.pose.orientation.w, std::cos(0.0150376 / 2.0), kEpsilon);
  EXPECT_NEAR(msg.twist.twist.linear.x, 0.0570652, kEpsilon);
  EXPECT_NEAR(msg.twist.twist.linear.y, -0.024843, kEpsilon);
  EXPECT_NEAR(msg.twist.twist.angular.z, 3.15038, kEpsilon);
}

/// Publish odometry tf
TEST_F(WheelOdometryTest, PublishWheelTfOdometry) {
  auto client_node = rclcpp::Node::make_shared("client_node");
  auto counter = std::make_shared<SubscriptionCounter<tf2_msgs::msg::TFMessage>>(client_node, "/tf");
  odom_->set_last_transform_published_time(node_->now());

  rclcpp::WallRate loop_rate(10.0);
  for (uint32_t i = 0; i < 10; ++i) {
    odom_->PublishOdometry(node_->now());
    rclcpp::spin_some(client_node);
    rclcpp::spin_some(node_->get_node_base_interface());
    loop_rate.sleep();
  }
  EXPECT_EQ(counter->count(), 3);

  auto transforms = counter->last_msg();
  ASSERT_EQ(transforms.transforms.size(), 1);

  auto transform = transforms.transforms[0];
  EXPECT_EQ(transform.header.frame_id, "odom");
  EXPECT_EQ(transform.child_frame_id, "base_footprint_wheel");
  EXPECT_NEAR(transform.transform.translation.x, 0.00573091, kEpsilon);
  EXPECT_NEAR(transform.transform.translation.y, -0.00239658, kEpsilon);
  EXPECT_NEAR(transform.transform.rotation.z, std::sin(0.0150376 / 2.0), kEpsilon);
  EXPECT_NEAR(transform.transform.rotation.w, std::cos(0.0150376 / 2.0), kEpsilon);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

