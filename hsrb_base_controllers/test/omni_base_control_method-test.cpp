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
/// @file omni_base_control_method-test.cpp
/// @brief Test class for omnidirectional cart control mode

#include <gtest/gtest.h>

#include <hsrb_base_controllers/omni_base_control_method.hpp>


namespace {
constexpr double kEpsilon = 1.0e-5;
}  // namespace

namespace hsrb_base_controllers {

class OmniBaseVelocityControlTest : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  OmniBaseVelocityControl::Ptr control_;
  geometry_msgs::msg::Twist::SharedPtr input_;
};

void OmniBaseVelocityControlTest::SetUp() {
  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node_->configure();
  node_->declare_parameter("command_timeout", 0.1);
  control_ = std::make_shared<OmniBaseVelocityControl>(node_);
  node_->activate();

  input_ = std::make_shared<geometry_msgs::msg::Twist>();
  input_->linear.x = 0.1;
  input_->linear.y = 0.2;
  input_->angular.z = 0.3;
}

// Update and retrieve speed commands
TEST_F(OmniBaseVelocityControlTest, UpdateCommandVelocity) {
  control_->UpdateCommandVelocity(input_);
  auto output = control_->GetOutputVelocity();

  EXPECT_DOUBLE_EQ(output[0], 0.1);
  EXPECT_DOUBLE_EQ(output[1], 0.2);
  EXPECT_DOUBLE_EQ(output[2], 0.3);
}

// Speed becomes zero if there are no command speeds for a certain period
TEST_F(OmniBaseVelocityControlTest, OutputVelocityIsZero) {
  control_->UpdateCommandVelocity(input_);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.2)));
  auto output = control_->GetOutputVelocity();

  EXPECT_DOUBLE_EQ(output[0], 0.0);
  EXPECT_DOUBLE_EQ(output[1], 0.0);
  EXPECT_DOUBLE_EQ(output[2], 0.0);
}

// Clear command speeds
TEST_F(OmniBaseVelocityControlTest, Activate) {
  control_->UpdateCommandVelocity(input_);
  control_->Activate();
  auto output = control_->GetOutputVelocity();

  EXPECT_DOUBLE_EQ(output[0], 0.0);
  EXPECT_DOUBLE_EQ(output[1], 0.0);
  EXPECT_DOUBLE_EQ(output[2], 0.0);
}


template <typename ControllerType>
class OmniBaseTrajectoryControlTest : public ::testing::Test {
 protected:
  void SetUp() override;
  void SetupController(std::vector<std::string>& coordinates);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  typename ControllerType::Ptr control_;
  trajectory_msgs::msg::JointTrajectory::SharedPtr input_trajectory_;
};

template <typename ControllerType>
void OmniBaseTrajectoryControlTest<ControllerType>::SetUp() {
  node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node_->configure();
}

template <typename ControllerType>
void OmniBaseTrajectoryControlTest<ControllerType>::SetupController(std::vector<std::string>& coordinates) {
  control_ = std::make_shared<ControllerType>(node_, coordinates);

  node_->activate();
  control_->Activate();

  input_trajectory_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  input_trajectory_->header.stamp = node_->now();
  input_trajectory_->joint_names = coordinates;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(coordinates.size(), 0.0);
  point.velocities.resize(coordinates.size(), 0.0);
  point.accelerations.resize(coordinates.size(), 0.0);
  point.time_from_start = rclcpp::Duration(10, 0);
  input_trajectory_->points.push_back(point);
}


class OmniBaseOdomTrajectoryControlTest : public OmniBaseTrajectoryControlTest<OmniBaseOdomTrajectoryControl> {
 protected:
  void SetUp() override;
};

void OmniBaseOdomTrajectoryControlTest::SetUp() {
  OmniBaseTrajectoryControlTest::SetUp();

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  SetupController(base_coordinates);
}

// Retrieve command speeds in trajectory tracking mode
TEST_F(OmniBaseOdomTrajectoryControlTest, GetTrajOutputVelocity) {
  ControllerState state;
  state.actual.positions = {0.0, 0.0, 0.0};
  state.error.positions = {1.0, 2.0, 3.0};
  state.desired.velocities = {1.1, 1.2, 1.3};

  // Default value of P gain is 1.0
  auto output_vel = control_->GetOutputVelocity(state);
  EXPECT_NEAR(output_vel[0], 2.1, kEpsilon);
  EXPECT_NEAR(output_vel[1], 3.2, kEpsilon);
  EXPECT_NEAR(output_vel[2], 4.3, kEpsilon);

  state.actual.positions = {0.0, 0.0, M_PI};

  output_vel = control_->GetOutputVelocity(state);
  EXPECT_NEAR(output_vel[0], -2.1, kEpsilon);
  EXPECT_NEAR(output_vel[1], -3.2, kEpsilon);
  EXPECT_NEAR(output_vel[2], 4.3, kEpsilon);
}

// Retrieve command speeds reflecting gain in trajectory tracking mode
TEST_F(OmniBaseOdomTrajectoryControlTest, GetTrajOutputVelocityWithGain) {
  node_->set_parameter({rclcpp::Parameter("odom_x.p_gain", 2.0)});
  node_->set_parameter({rclcpp::Parameter("odom_y.p_gain", 3.0)});
  node_->set_parameter({rclcpp::Parameter("odom_t.p_gain", 4.0)});

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseOdomTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  ControllerState state;
  state.actual.positions = {0.0, 0.0, 0.0};
  state.error.positions = {1.0, 2.0, 3.0};
  state.desired.velocities = {1.1, 1.2, 1.3};

  auto output_vel = control_->GetOutputVelocity(state);
  EXPECT_NEAR(output_vel[0], 3.1, kEpsilon);
  EXPECT_NEAR(output_vel[1], 7.2, kEpsilon);
  EXPECT_NEAR(output_vel[2], 13.3, kEpsilon);
}

// Default value is used when an invalid gain is specified
TEST_F(OmniBaseOdomTrajectoryControlTest, NotPositivePGain) {
  node_->set_parameter({rclcpp::Parameter("odom_x.p_gain", 0.0)});
  node_->set_parameter({rclcpp::Parameter("odom_y.p_gain", 0.0)});
  node_->set_parameter({rclcpp::Parameter("odom_t.p_gain", 0.0)});

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseOdomTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  ControllerState state;
  state.actual.positions = {0.0, 0.0, 0.0};
  state.error.positions = {1.0, 2.0, 3.0};
  state.desired.velocities = {1.1, 1.2, 1.3};

  auto output_vel = control_->GetOutputVelocity(state);
  EXPECT_NEAR(output_vel[0], 2.1, kEpsilon);
  EXPECT_NEAR(output_vel[1], 3.2, kEpsilon);
  EXPECT_NEAR(output_vel[2], 4.3, kEpsilon);
}

// Check for the presence of a trajectory
TEST_F(OmniBaseOdomTrajectoryControlTest, UpdateActiveTrajectory) {
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  // If a valid trajectory is accepted, Update succeeds as many times as needed while the trajectory exists
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  control_->ResetCurrentTrajectory();
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  input_trajectory_->points.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_FALSE(control_->UpdateActiveTrajectory());
}

// Retrieve the expected state
TEST_F(OmniBaseOdomTrajectoryControlTest, SampleDesiredState) {
  input_trajectory_->points.back().positions = {1.0, 2.0, 3.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0, 0.0, 0.0};
  std::vector<double> current_velocities = {0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(10, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 2.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 3.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.0);
  EXPECT_FALSE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, 0.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(11, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 2.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 3.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.0);
  EXPECT_FALSE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, 1.0);
}

// Retrieve the expected state
TEST_F(OmniBaseOdomTrajectoryControlTest, SampleDesiredStateWithOpenLoop) {
  node_->set_parameter({rclcpp::Parameter("open_loop_control", true)});

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseOdomTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  input_trajectory_->points.back().positions = {1.0, 2.0, 3.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0, 0.0, 0.0};
  std::vector<double> current_velocities = {0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  input_trajectory_->header.stamp.sec += 5;
  // Adjust the input of the rotation axis to exceed π
  input_trajectory_->points.back().positions = {1.5, 2.0, 3.5 - 2 * M_PI};
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  // Since open_loop_control is true, the state at the time of the last sampling becomes the reference
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.2);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 2.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.2);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  // Temporarily complete tracking
  ControllerState zero_velocity_state;
  zero_velocity_state.actual.velocities = {0.0, 0.0, 0.0};
  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(15, 0);
  control_->TerminateControl(stamp, zero_velocity_state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  // After completing trajectory tracking, the current position may have changed due to sampling, speed control, etc., based on the current state
  input_trajectory_->header.stamp = stamp;
  input_trajectory_->points.back().positions = {1.0, 2.0, 3.0};
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
}

// Retrieve the expected state
TEST_F(OmniBaseOdomTrajectoryControlTest, SampleDesiredStateWithoutOpenLoop) {
  node_->set_parameter({rclcpp::Parameter("open_loop_control", false)});

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseOdomTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  input_trajectory_->points.back().positions = {1.0, 2.0, 3.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0, 0.0, 0.0};
  std::vector<double> current_velocities = {0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  input_trajectory_->header.stamp.sec += 5;
  input_trajectory_->points.back().positions = {1.5, 2.0, -3.0};
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  // Since open_loop_control is false, sampling is based on current_positions/velocities
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.15);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], -0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.75);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], -1.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.15);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.2);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], -0.3);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);
}

// Joint names are rearranged
TEST_F(OmniBaseOdomTrajectoryControlTest, PermutatedTrajectory) {
  input_trajectory_->joint_names = {input_trajectory_->joint_names[0],
                                    input_trajectory_->joint_names[2],
                                    input_trajectory_->joint_names[1]};
  input_trajectory_->points.back().positions = {1.0, 2.0, 3.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0, 0.0, 0.0};
  std::vector<double> current_velocities = {0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[1], 1.5);
  EXPECT_DOUBLE_EQ(desired_state.positions[2], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_DOUBLE_EQ(desired_state.velocities[1], 0.3);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.2);
}

// Rotation axis correction is performed
TEST_F(OmniBaseOdomTrajectoryControlTest, OverPISteerTrajectory) {
  input_trajectory_->points.back().positions = {1.0, 2.0, M_PI - 1.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  input_trajectory_->points.push_back(input_trajectory_->points.back());
  input_trajectory_->points.back().positions = {1.0, 2.0, -M_PI + 1.0};
  input_trajectory_->points.back().time_from_start = rclcpp::Duration(20, 0);
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0, 0.0, 0.0};
  std::vector<double> current_velocities = {0.0, 0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(15, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[2], M_PI);
  EXPECT_DOUBLE_EQ(desired_state.velocities[2], 0.2);
}

// Normal case for ValidateTrajectory
TEST_F(OmniBaseOdomTrajectoryControlTest, ValidateTrajectory) {
  EXPECT_TRUE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check NG: Mismatch in the number of joints
TEST_F(OmniBaseOdomTrajectoryControlTest, DoNotMatchJointsSize) {
  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t", "test_joint"};
  control_ = std::make_shared<OmniBaseOdomTrajectoryControl>(node_, base_coordinates);
  control_->Activate();
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check NG: Mismatch between the number of elements in position and the number of joints
TEST_F(OmniBaseOdomTrajectoryControlTest, DoNotMatchPositionSize) {
  input_trajectory_->points.back().positions.push_back(0.0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check NG: Mismatch between the number of elements in velocity and the number of joints
TEST_F(OmniBaseOdomTrajectoryControlTest, DoNotMatchVelocitySize) {
  input_trajectory_->points.back().velocities.push_back(0.0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check OK: Number of elements in velocity is empty
TEST_F(OmniBaseOdomTrajectoryControlTest, VelocityIsEmpty) {
  input_trajectory_->points.back().velocities.clear();
  EXPECT_TRUE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check NG: Mismatch between the number of elements in acceleration and the number of joints
TEST_F(OmniBaseOdomTrajectoryControlTest, DoNotMatchAccelerationSize) {
  input_trajectory_->points.back().accelerations.push_back(0.0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check OK: Number of elements in acceleration is empty
TEST_F(OmniBaseOdomTrajectoryControlTest, AccelerationIsEmpty) {
  input_trajectory_->points.back().accelerations.clear();
  EXPECT_TRUE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check NG: Invalid if time_from_start is reversed
TEST_F(OmniBaseOdomTrajectoryControlTest, TimeFromStartIdGoingReverse) {
  input_trajectory_->points.push_back(input_trajectory_->points.back());
  input_trajectory_->points.back().time_from_start = rclcpp::Duration(1, 0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// Contains invalid joint names
TEST_F(OmniBaseOdomTrajectoryControlTest, IncludeInvalidJointName) {
  input_trajectory_->joint_names[0] = "unknown";
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// Complete trajectory tracking after stopping state and time elapse
TEST_F(OmniBaseOdomTrajectoryControlTest, TerminateControl) {
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.actual.velocities = {0.1, 0.0, 0.0};

  auto stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(10, 0);
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.0, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(11, 0);
  state.actual.velocities = {0.1, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.0, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());
}

// Change the threshold for stopping state
TEST_F(OmniBaseOdomTrajectoryControlTest, ChangeStopVelocityThreshold) {
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.actual.velocities = {0.001 + 1e-6, 0.0, 0.0};

  auto stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(11, 0);
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.001 - 1e-6, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  node_->set_parameter({rclcpp::Parameter("stop_velocity_threshold", 0.0)});

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseOdomTrajectoryControl>(node_, base_coordinates);
  control_->Activate();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.001 + 1e-6, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.001 - 1e-6, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  node_->set_parameter({rclcpp::Parameter("stop_velocity_threshold", 0.1)});

  control_ = std::make_shared<OmniBaseOdomTrajectoryControl>(node_, base_coordinates);
  control_->Activate();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.1 + 1e-6, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.1 - 1e-6, 0.0, 0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());
}

TEST_F(OmniBaseOdomTrajectoryControlTest, CheckPathToleranceSuccess) {
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.error.positions = {0.0, 0.0, 0.0};
  state.error.velocities = {0.0, 0.0, 0.0};

  EXPECT_NE(control_->CheckTorelances(state, true, 0.0),
            control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED);
}

TEST_F(OmniBaseOdomTrajectoryControlTest, CheckPathToleranceViolated) {
  node_->declare_parameter<double>("constraints.odom_x.trajectory", 1.0);
  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseOdomTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.error.positions = {1.5, 0.0, 0.0};
  state.error.velocities = {0.0, 0.0, 0.0};

  EXPECT_EQ(control_->CheckTorelances(state, true, 0.0),
            control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED);
}

TEST_F(OmniBaseOdomTrajectoryControlTest, CheckGoalToleranceSuccess) {
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.error.positions = {0.0, 0.0, 0.0};
  state.error.velocities = {0.0, 0.0, 0.0};

  EXPECT_EQ(control_->CheckTorelances(state, false, 0.0),
            control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);
}

TEST_F(OmniBaseOdomTrajectoryControlTest, CheckGoalTimeTolerance) {
  node_->declare_parameter<double>("constraints.goal_time", 2.0);

  std::vector<std::string> base_coordinates = {"odom_x", "odom_y", "odom_t"};
  control_ = std::make_shared<OmniBaseOdomTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.error.positions = {0.0, 0.0, 0.0};
  state.error.velocities = {0.1, 0.0, 0.0};

  EXPECT_NE(control_->CheckTorelances(state, false, 0.0),
            control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED);

  EXPECT_EQ(control_->CheckTorelances(state, false, 3.0),
            control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED);
}


class OmniBaseRollTrajectoryControlTest : public OmniBaseTrajectoryControlTest<OmniBaseRollTrajectoryControl> {
 protected:
  void SetUp() override;
};

void OmniBaseRollTrajectoryControlTest::SetUp() {
  OmniBaseTrajectoryControlTest::SetUp();

  std::vector<std::string> base_coordinates = {"base_roll_joint"};
  SetupController(base_coordinates);
}

// Check for the presence of a trajectory
TEST_F(OmniBaseRollTrajectoryControlTest, UpdateActiveTrajectory) {
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  // If a valid trajectory is accepted, Update succeeds as many times as needed while the trajectory exists
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  control_->ResetCurrentTrajectory();
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  input_trajectory_->points.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_FALSE(control_->UpdateActiveTrajectory());
}

// Retrieve the expected state
TEST_F(OmniBaseRollTrajectoryControlTest, SampleDesiredState) {
  input_trajectory_->points.back().positions = {1.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0};
  std::vector<double> current_velocities = {0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(10, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.0);
  EXPECT_FALSE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, 0.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(11, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.0);
  EXPECT_FALSE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, 1.0);
}

// Retrieve the expected state
TEST_F(OmniBaseRollTrajectoryControlTest, SampleDesiredStateWithOpenLoop) {
  node_->set_parameter({rclcpp::Parameter("open_loop_control", true)});

  std::vector<std::string> base_coordinates = {"base_roll_joint"};
  control_ = std::make_shared<OmniBaseRollTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  input_trajectory_->points.back().positions = {1.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0};
  std::vector<double> current_velocities = {0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  input_trajectory_->header.stamp.sec += 5;
  input_trajectory_->points.back().positions = {1.5};
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  // Since open_loop_control is true, the state at the time of the last sampling becomes the reference
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 1.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  // Temporarily complete tracking
  ControllerState zero_velocity_state;
  zero_velocity_state.actual.velocities = {0.0};
  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(15, 0);
  control_->TerminateControl(stamp, zero_velocity_state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  // After completing trajectory tracking, the current position may have changed due to sampling, speed control, etc., based on the current state
  input_trajectory_->header.stamp = stamp;
  input_trajectory_->points.back().positions = {1.0};
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
}

// Retrieve the expected state
TEST_F(OmniBaseRollTrajectoryControlTest, SampleDesiredStateWithoutOpenLoop) {
  node_->set_parameter({rclcpp::Parameter("open_loop_control", false)});

  std::vector<std::string> base_coordinates = {"base_roll_joint"};
  control_ = std::make_shared<OmniBaseRollTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  input_trajectory_->points.back().positions = {1.0};
  input_trajectory_->points.back().velocities.clear();
  input_trajectory_->points.back().accelerations.clear();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  std::vector<double> current_positions = {0.0};
  std::vector<double> current_velocities = {0.0};

  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  bool before_last_point;
  double time_from_point;

  rclcpp::Time stamp = input_trajectory_->header.stamp;
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.1);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);

  input_trajectory_->header.stamp.sec += 5;
  input_trajectory_->points.back().positions = {1.5};
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  // Since open_loop_control is false, sampling is based on current_positions/velocities
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.15);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -10.0);

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(5, 0);
  EXPECT_TRUE(control_->SampleDesiredState(stamp, current_positions, current_velocities,
                                           desired_state, before_last_point, time_from_point));
  EXPECT_DOUBLE_EQ(desired_state.positions[0], 0.75);
  EXPECT_DOUBLE_EQ(desired_state.velocities[0], 0.15);
  EXPECT_TRUE(before_last_point);
  EXPECT_DOUBLE_EQ(time_from_point, -5.0);
}

// Normal case for ValidateTrajectory
TEST_F(OmniBaseRollTrajectoryControlTest, ValidateTrajectory) {
  EXPECT_TRUE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check NG: Mismatch in the number of joints
TEST_F(OmniBaseRollTrajectoryControlTest, DoNotMatchJointsSize) {
  std::vector<std::string> base_coordinates = {"base_roll_joint", "test_joint"};
  control_ = std::make_shared<OmniBaseRollTrajectoryControl>(node_, base_coordinates);
  control_->Activate();
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check NG: Mismatch between the number of elements in position and the number of joints
TEST_F(OmniBaseRollTrajectoryControlTest, DoNotMatchPositionSize) {
  input_trajectory_->points.back().positions.push_back(0.0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check NG: Mismatch between the number of elements in velocity and the number of joints
TEST_F(OmniBaseRollTrajectoryControlTest, DoNotMatchVelocitySize) {
  input_trajectory_->points.back().velocities.push_back(0.0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check OK: Number of elements in velocity is empty
TEST_F(OmniBaseRollTrajectoryControlTest, VelocityIsEmpty) {
  input_trajectory_->points.back().velocities.clear();
  EXPECT_TRUE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check NG: Mismatch between the number of elements in acceleration and the number of joints
TEST_F(OmniBaseRollTrajectoryControlTest, DoNotMatchAccelerationSize) {
  input_trajectory_->points.back().accelerations.push_back(0.0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check OK: Number of elements in acceleration is empty
TEST_F(OmniBaseRollTrajectoryControlTest, AccelerationIsEmpty) {
  input_trajectory_->points.back().accelerations.clear();
  EXPECT_TRUE(control_->ValidateTrajectory(*input_trajectory_));
}

// JointTrajectory validity check NG: Invalid if time_from_start is reversed
TEST_F(OmniBaseRollTrajectoryControlTest, TimeFromStartIdGoingReverse) {
  input_trajectory_->points.push_back(input_trajectory_->points.back());
  input_trajectory_->points.back().time_from_start = rclcpp::Duration(1, 0);
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// Contains invalid joint names
TEST_F(OmniBaseRollTrajectoryControlTest, IncludeInvalidJointName) {
  input_trajectory_->joint_names[0] = "unknown";
  EXPECT_FALSE(control_->ValidateTrajectory(*input_trajectory_));
}

// Complete trajectory tracking after stopping state and time elapse
TEST_F(OmniBaseRollTrajectoryControlTest, TerminateControl) {
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.actual.velocities = {0.1};

  auto stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(10, 0);
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(11, 0);
  state.actual.velocities = {0.1};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.0};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());
}

// Change the threshold for stopping state
TEST_F(OmniBaseRollTrajectoryControlTest, ChangeStopVelocityThreshold) {
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.actual.velocities = {0.001 + 1e-6};

  auto stamp = rclcpp::Time(input_trajectory_->header.stamp) + rclcpp::Duration(11, 0);
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.001 - 1e-6};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  node_->set_parameter({rclcpp::Parameter("roll_stop_velocity_threshold", 0.0)});

  std::vector<std::string> base_coordinates = {"base_roll_joint"};
  control_ = std::make_shared<OmniBaseRollTrajectoryControl>(node_, base_coordinates);
  control_->Activate();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.001 + 1e-6};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.001 - 1e-6};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());

  node_->set_parameter({rclcpp::Parameter("roll_stop_velocity_threshold", 0.1)});

  control_ = std::make_shared<OmniBaseRollTrajectoryControl>(node_, base_coordinates);
  control_->Activate();
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.1 + 1e-6};
  control_->TerminateControl(stamp, state);
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  state.actual.velocities = {0.1 - 1e-6};
  control_->TerminateControl(stamp, state);
  EXPECT_FALSE(control_->UpdateActiveTrajectory());
}

TEST_F(OmniBaseRollTrajectoryControlTest, CheckPathToleranceSuccess) {
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.error.positions = {0.0};
  state.error.velocities = {0.0};

  EXPECT_NE(control_->CheckTorelances(state, true, 0.0),
            control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED);
}

TEST_F(OmniBaseRollTrajectoryControlTest, CheckPathToleranceViolated) {
  node_->declare_parameter<double>("constraints.base_roll_joint.trajectory", 1.0);
  std::vector<std::string> base_coordinates = {"base_roll_joint"};
  control_ = std::make_shared<OmniBaseRollTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.error.positions = {1.5};
  state.error.velocities = {0.0};

  EXPECT_EQ(control_->CheckTorelances(state, true, 0.0),
            control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED);
}

TEST_F(OmniBaseRollTrajectoryControlTest, CheckGoalToleranceSuccess) {
  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.error.positions = {0.0};
  state.error.velocities = {0.0};

  EXPECT_EQ(control_->CheckTorelances(state, false, 0.0),
            control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);
}

TEST_F(OmniBaseRollTrajectoryControlTest, CheckGoalPositionTolerance) {
  node_->declare_parameter<double>("constraints.base_roll_joint.goal", 0.02);
  std::vector<std::string> base_coordinates = {"base_roll_joint"};
  control_ = std::make_shared<OmniBaseRollTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.error.positions = {0.03};

  EXPECT_NE(control_->CheckTorelances(state, false, 0.0),
            control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);

  state.error.positions = {0.01};

  EXPECT_EQ(control_->CheckTorelances(state, false, 0.1),
            control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);
}

TEST_F(OmniBaseRollTrajectoryControlTest, CheckGoalTimeTolerance) {
  node_->declare_parameter<double>("constraints.goal_time", 2.0);

  std::vector<std::string> base_coordinates = {"base_roll_joint"};
  control_ = std::make_shared<OmniBaseRollTrajectoryControl>(node_, base_coordinates);
  control_->Activate();

  control_->AcceptTrajectory(input_trajectory_, Eigen::Vector3d::Zero());
  EXPECT_TRUE(control_->UpdateActiveTrajectory());

  ControllerState state;
  state.error.positions = {0.0};
  state.error.velocities = {0.1};

  EXPECT_NE(control_->CheckTorelances(state, false, 0.0),
            control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED);

  EXPECT_EQ(control_->CheckTorelances(state, false, 3.0),
            control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
