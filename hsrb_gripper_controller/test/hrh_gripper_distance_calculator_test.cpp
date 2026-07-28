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
/// @file hrh_gripper_controller_apply_force_calculator-test.cpp
/// @brief Test class for calculating the gripping force of the HRH gripper

#include <gtest/gtest.h>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <hsrb_gripper_controller/hrh_gripper_distance.hpp>

#include "utils.hpp"

namespace {

void spin(const rclcpp::Node::SharedPtr& node) {
  while (node) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace

namespace hsrb_gripper_controller {

class HrhGripperDistanceCalculatorTest : public ::testing::Test {
 public:
  void SetUp() override;

 protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr urdf_node_;
};

void HrhGripperDistanceCalculatorTest::SetUp() {
  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node_->configure();

  auto parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
  parameter_descriptor.dynamic_typing = true;
  node_->declare_parameter("model_name", "robot_description", parameter_descriptor);
  node_->declare_parameter("proximal_joint", "hand_l_proximal_joint", parameter_descriptor);
  node_->declare_parameter("distal_joint", "hand_l_distal_joint", parameter_descriptor);
  node_->declare_parameter("mimic_distal_joint", "hand_l_mimic_distal_joint", parameter_descriptor);
  node_->declare_parameter("finger_tip_frame_joint", "hand_l_finger_tip_frame_joint", parameter_descriptor);
  node_->declare_parameter("robot_description", ReadRobotDescriptionFromFile(), parameter_descriptor);
  node_->declare_parameter("parameter_connection_timeout", 0);

  node_->activate();

  urdf_node_ = rclcpp::Node::make_shared("urdf_node");
  urdf_node_->declare_parameter("robot_description", ReadRobotDescriptionFromFile());
}

/// Check if initialization fails when the name of proximal_joint is not in the provided interface list
TEST_F(HrhGripperDistanceCalculatorTest, BadProximalJointName) {
  node_->set_parameter({rclcpp::Parameter("proximal_joint", "bad_joint_name")});
  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  EXPECT_FALSE(calculator->InitializeHandSizeData(node_));
}

/// Check if initialization fails when the name of distal_joint is not in the provided interface list
TEST_F(HrhGripperDistanceCalculatorTest, BadDistalJointName) {
  node_->set_parameter({rclcpp::Parameter("distal_joint", "bad_joint_name")});
  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  EXPECT_FALSE(calculator->InitializeHandSizeData(node_));
}

/// Check if initialization fails when the name of mimic_distal_joint is not in the provided interface list
TEST_F(HrhGripperDistanceCalculatorTest, BadMimicDistalJointName) {
  node_->set_parameter({rclcpp::Parameter("mimic_distal_joint", "bad_joint_name")});
  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  EXPECT_FALSE(calculator->InitializeHandSizeData(node_));
}

/// Check if initialization fails when the name of finger_tip_frame_joint is not in the provided interface list
TEST_F(HrhGripperDistanceCalculatorTest, BadFingerTipFrameJointName) {
  node_->set_parameter({rclcpp::Parameter("finger_tip_frame_joint", "bad_joint_name")});
  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  EXPECT_FALSE(calculator->InitializeHandSizeData(node_));
}

/// Check if initialization fails with an invalid robot_description
TEST_F(HrhGripperDistanceCalculatorTest, InvalidRobotDescription) {
  node_->set_parameter({rclcpp::Parameter("robot_description", "invalid")});
  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  EXPECT_FALSE(calculator->InitializeHandSizeData(node_));
}

/// Initialize by obtaining the robot model from another node
TEST_F(HrhGripperDistanceCalculatorTest, RobotDescriptionFromAnotherNode) {
  node_->undeclare_parameter("robot_description");
  node_->declare_parameter("model_node_name", "urdf_node");

  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  auto spin_thread = std::thread(std::bind(spin, std::ref(urdf_node_)));
  EXPECT_TRUE(calculator->InitializeHandSizeData(node_));
  urdf_node_.reset();
  spin_thread.join();
}

/// The specified node does not contain a robot model
TEST_F(HrhGripperDistanceCalculatorTest, NoRobotDescriptionOnAnotherNode) {
  node_->undeclare_parameter("robot_description");
  node_->declare_parameter("model_node_name", "no_description_node");
  auto node = rclcpp::Node::make_shared("no_description_node");

  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  auto spin_thread = std::thread(std::bind(spin, std::ref(node)));
  EXPECT_FALSE(calculator->InitializeHandSizeData(node_));
  node.reset();
  spin_thread.join();
}

/// Attempt to reference another node, but the specified node does not exist
TEST_F(HrhGripperDistanceCalculatorTest, NoRobotDescriptionNode) {
  node_->undeclare_parameter("robot_description");
  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  EXPECT_FALSE(calculator->InitializeHandSizeData(node_));
}

/// Obtain the opening width from motor_pos only
TEST_F(HrhGripperDistanceCalculatorTest, GetDistance1) {
  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  EXPECT_TRUE(calculator->InitializeHandSizeData(node_));

  EXPECT_NEAR(calculator->GetDistanceFromPosition(1.0), 0.122194, kEpsilon);
}

/// Obtain the opening width from motor_pos and the left and right spring_proximal_joint_pos
TEST_F(HrhGripperDistanceCalculatorTest, GetDistance2) {
  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  EXPECT_TRUE(calculator->InitializeHandSizeData(node_));

  EXPECT_NEAR(calculator->GetDistanceFromPosition(1.0, 0.1, 0.1), 0.129157, kEpsilon);
}

/// Obtain the position from the opening width
TEST_F(HrhGripperDistanceCalculatorTest, GetPosition) {
  auto calculator = std::make_shared<HrhGripperDistanceCalculator>();
  EXPECT_TRUE(calculator->InitializeHandSizeData(node_));

  EXPECT_NEAR(calculator->GetPositionFromDistance(0.1), 0.751787, kEpsilon);
}

}  // namespace hsrb_gripper_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
