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

#include <gtest/gtest.h>

#include <hsrb_base_controllers/omni_base_joint_controller.hpp>

#include "hardware_stub.hpp"
#include "utils.hpp"

namespace {
constexpr double kEpsilon = 1.0e-6;

constexpr double kUpdateFrequency = 1000.0;
constexpr double kUpdatePeriod = 1.0 / kUpdateFrequency;

constexpr double FirstMoveTimeout = 2.0;
constexpr double SecondMoveTimeout = 5.0;

std::string CreateRobotDescription(const double tread,
                                   const double caster_offset,
                                   const double wheel_radius) {
  const std::string right_xyz =
      std::to_string(-caster_offset) + " " + std::to_string(-tread / 2.0) + " " + std::to_string(wheel_radius);
  const std::string left_xyz =
      std::to_string(-caster_offset) + " " + std::to_string(tread / 2.0) + " " + std::to_string(wheel_radius);
  std::string robot_description = R"(
<robot name="test_robot">
  <link name="base_link"/>
  <joint name="base_roll_joint" type="continuous">
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
    <axis xyz="0.0 0.0 1.0"/>
    <parent link="base_link"/>
    <child link="base_roll_link"/>
  </joint>
  <link name="base_roll_link"/>
  <joint name="base_r_drive_wheel_joint" type="continuous">
    <origin rpy="0.0 0.0 0.0" xyz=")" + right_xyz + R"("/>
    <axis xyz="0.0 1.0 0.0"/>
    <parent link="base_roll_link"/>
    <child link="base_r_drive_wheel_link"/>
  </joint>
  <link name="base_r_drive_wheel_link"/>
  <joint name="base_l_drive_wheel_joint" type="continuous">
    <origin rpy="0.0 0.0 0.0" xyz=")" + left_xyz + R"("/>
    <axis xyz="0.0 1.0 0.0"/>
    <parent link="base_roll_link"/>
    <child link="base_l_drive_wheel_link"/>
  </joint>
  <link name="base_l_drive_wheel_link"/>
</robot>
)";
  return robot_description;
}

struct RobotDescription {
  std::string robot_description;

  double wheel_radius;

  double yaw_velocity_limit;
  double wheel_velocity_limit;

  double yaw_acceleration_limit;
  double wheel_acceleration_limit;

  double max_base_velocity;
  double safe_base_velocity;

  RobotDescription(const double thread,
                   const double caster_offset,
                   const double wheel_radius,
                   const double yaw_velocity_limit,
                   const double wheel_velocity_limit,
                   const double yaw_acceleration_limit,
                   const double wheel_acceleration_limit)
      : robot_description(CreateRobotDescription(thread, caster_offset, wheel_radius)),
        wheel_radius(wheel_radius),
        yaw_velocity_limit(yaw_velocity_limit),
        wheel_velocity_limit(wheel_velocity_limit),
        yaw_acceleration_limit(yaw_acceleration_limit),
        wheel_acceleration_limit(wheel_acceleration_limit),
        max_base_velocity(wheel_radius * wheel_velocity_limit) {
    const double rate = std::sqrt(std::pow(thread / caster_offset, 2.0) + 1);
    safe_base_velocity = max_base_velocity / rate;
  }
};

RobotDescription GetHsrbDescription() {
  return RobotDescription(0.133, 0.11, 0.04, 1.8, 8.5, 5.0, 41.7);
}

RobotDescription GetHsrcDescription() {
  return RobotDescription(0.133, 0.11, 0.0445, 2.5, 13.5, 5.0, 41.7);
}

RobotDescription GetHsrfDescription() {
  // TODO(Takeshita) 調整後の値を入れる
  return RobotDescription(0.132, 0.108, 0.04725, 5.0, 41.8, 34.3, 60.0);
}

}  // namespace

namespace hsrb_base_controllers {

struct VelocityStamped {
  double stamp;
  Eigen::Vector3d velocities;
};

class OmniBaseJointControllerAccLimitTest : public ::testing::Test {
 public:
  void SetUpImpl(const RobotDescription& desc);
  void TearDown() override;

 protected:
  void UpdateOnce(const double linear_x, const double linear_y, const double angular_z);
  void ClearHistory();

  void ValidateJointOutput(const RobotDescription& desc);
  void ValidateBaseOutputLast(const double linear_x, const double linear_y, const double angular_z);

  enum class Trend { kUp, kDown, kFlat };
  void ValidateBaseOutputMonotonic(const int32_t index, const Trend trend, const double value);
  void ValidateBaseOutputWeakMonotonic(const int32_t index, const Trend trend, const double value,
                                       const double distance);

  void ValidateBaseOutputZero(const int32_t index, const double epsilon = kEpsilon);
  void ValidateBaseOutputPattern(const int32_t index, const double distance,
                                 const Trend first_trend, const uint32_t trend_num);

  // Although it could be done with parameterized tests instead of helper functions, it was considered more understandable to write the subtle differences in tolerance directly.
  void MoveForwardFromIdleHelper(const RobotDescription& desc, const double velocity);
  void MoveLeftFromIdleHelper(const RobotDescription& desc, const double velocity, const double tolerance);
  void MoveBackwardFromIdleHelper(const RobotDescription& desc, const double velocity, const double tolerance,
                                  const uint32_t trend_num = 5);

  void StopOnMovingHelper(const RobotDescription& desc, const double velocity);
  void MoveLeftOnMovingHelper(const RobotDescription& desc, const double velocity,
                              const uint32_t trend_num = 2, const uint32_t y_trend_num = 3);
  void MoveBackwardOnMovingHelper(const RobotDescription& desc, const double velocity, const double tolerance,
                                  const uint32_t trend_num = 5);

  OmniBaseJointControllerBase::Ptr controller_;
  HardwareStub<CommandVelocityHandle> hardware_;

  std::vector<VelocityStamped> joint_desired_history_;
  std::vector<VelocityStamped> joint_output_history_;
  std::vector<VelocityStamped> base_desired_history_;
  std::vector<VelocityStamped> base_output_history_;
  double stamp_;

  std::vector<VelocityStamped> joint_desired_history_all_;
  std::vector<VelocityStamped> joint_output_history_all_;
  std::vector<VelocityStamped> base_desired_history_all_;
  std::vector<VelocityStamped> base_output_history_all_;
};

void OmniBaseJointControllerAccLimitTest::SetUpImpl(const RobotDescription& desc) {
  // LifecycleNode is not the test subject, so success or failure is not checked.
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->configure();
  controller_ = std::make_shared<OmniBaseJointControllerBaseRollVelocity>(node);

  node->declare_parameter("joints.steer", "base_roll_joint");
  node->declare_parameter("joints.r_wheel", "base_r_drive_wheel_joint");
  node->declare_parameter("joints.l_wheel", "base_l_drive_wheel_joint");
  node->declare_parameter("robot_description", desc.robot_description);
  node->declare_parameter("parameter_connection_timeout", 0);
  // For now, set it to HSR-C.
  // TODO(Takeshita) パラメータ化したテスト
  node->declare_parameter("yaw_velocity_limit", desc.yaw_velocity_limit);
  node->declare_parameter("wheel_velocity_limit", desc.wheel_velocity_limit);
  node->declare_parameter("yaw_acceleration_limit", desc.yaw_acceleration_limit);
  node->declare_parameter("wheel_acceleration_limit", desc.wheel_acceleration_limit);
  node->activate();

  EXPECT_TRUE(controller_->Init());
  EXPECT_TRUE(controller_->Activate(hardware_.command_interfaces, hardware_.state_interfaces));

  stamp_ = 0.0;
  joint_desired_history_.push_back({stamp_, Eigen::Vector3d::Zero()});
  joint_output_history_.push_back({stamp_, Eigen::Vector3d::Zero()});
  base_desired_history_.push_back({stamp_, Eigen::Vector3d::Zero()});
  base_output_history_.push_back({stamp_, Eigen::Vector3d::Zero()});

  // If it's straight, the cart won't turn when reversing.
  hardware_.steer_handle->set_current_pos(0.01);
}

void OmniBaseJointControllerAccLimitTest::TearDown() {
  // Output the results to a file for operation confirmation.
  const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
  const auto filename = std::string(info->test_suite_name()) + "_" + info->name() + ".csv";

  joint_desired_history_all_.insert(joint_desired_history_all_.end(),
                                    joint_desired_history_.begin(), joint_desired_history_.end());
  joint_output_history_all_.insert(joint_output_history_all_.end(),
                                   joint_output_history_.begin(), joint_output_history_.end());
  base_desired_history_all_.insert(base_desired_history_all_.end(),
                                   base_desired_history_.begin(), base_desired_history_.end());
  base_output_history_all_.insert(base_output_history_all_.end(),
                                  base_output_history_.begin(), base_output_history_.end());

  std::ofstream ofs(filename);
  ofs << "time,"
         "base.desired.x,base.desired.y,base.desired.yaw,"
         "base.output.x,base.output.y,base.output.yaw,"
         "joint.desired.vel.r_wheel,joint.desired.vel.l_wheel,joint.desired.vel.steer,"
         "joint.desired.acc.r_wheel,joint.desired.acc.l_wheel,joint.desired.acc.steer,"
         "joint.output.vel.r_wheel,joint.output.vel.l_wheel,joint.output.vel.steer,"
         "joint.output.acc.r_wheel,joint.output.acc.l_wheel,joint.output.acc.steer" << std::endl;

  for (auto i = 0; i < joint_output_history_all_.size(); ++i) {
    const auto& jd = joint_desired_history_all_[i];
    const auto& jo = joint_output_history_all_[i];
    const auto& bd = base_desired_history_all_[i];
    const auto& bo = base_output_history_all_[i];
    const auto jd_acc = (i == 0) ? Eigen::Vector3d::Zero() :
        Eigen::Vector3d((jd.velocities - joint_desired_history_all_[i - 1].velocities) /
                            (jd.stamp - joint_desired_history_all_[i - 1].stamp));
    const Eigen::Vector3d jo_acc = (i == 0) ? Eigen::Vector3d::Zero() :
        Eigen::Vector3d((jo.velocities - joint_output_history_all_[i - 1].velocities) /
                            (jo.stamp - joint_output_history_all_[i - 1].stamp));
    ofs << jd.stamp << ","
        << bd.velocities.x() << "," << bd.velocities.y() << "," << bd.velocities.z() << ","
        << bo.velocities.x() << "," << bo.velocities.y() << "," << bo.velocities.z() << ","
        << jd.velocities.x() << "," << jd.velocities.y() << "," << jd.velocities.z() << ","
        << jd_acc.x() << "," << jd_acc.y() << "," << jd_acc.z() << ","
        << jo.velocities.x() << "," << jo.velocities.y() << "," << jo.velocities.z() << ","
        << jo_acc.x() << "," << jo_acc.y() << "," << jo_acc.z() << std::endl;
  }
  ofs.close();
}

void OmniBaseJointControllerAccLimitTest::UpdateOnce(const double linear_x,
                                                     const double linear_y,
                                                     const double angular_z) {
  controller_->SetJointCommand(kUpdatePeriod, Eigen::Vector3d(linear_x, linear_y, angular_z));
  hardware_.Update();

  stamp_ += kUpdatePeriod;
  joint_desired_history_.push_back({stamp_, controller_->joint_desired_velocity()});
  joint_output_history_.push_back({stamp_, controller_->joint_output_velocity()});
  base_desired_history_.push_back({stamp_, Eigen::Vector3d(linear_x, linear_y, angular_z)});
  base_output_history_.push_back({stamp_, controller_->base_output_velocity()});
}

void OmniBaseJointControllerAccLimitTest::ValidateJointOutput(const RobotDescription& desc) {
  EXPECT_LE(fabs(joint_output_history_[0].velocities(kJointIDLeftWheel)), desc.wheel_velocity_limit);
  EXPECT_LE(fabs(joint_output_history_[0].velocities(kJointIDRightWheel)), desc.wheel_velocity_limit);
  EXPECT_LE(fabs(joint_output_history_[0].velocities(kJointIDSteer)), desc.yaw_velocity_limit);

  for (auto i = 1; i < joint_output_history_.size(); ++i) {
    EXPECT_LE(fabs(joint_output_history_[i].velocities(kJointIDLeftWheel)), desc.wheel_velocity_limit + kEpsilon);
    EXPECT_LE(fabs(joint_output_history_[i].velocities(kJointIDRightWheel)), desc.wheel_velocity_limit + kEpsilon);
    EXPECT_LE(fabs(joint_output_history_[i].velocities(kJointIDSteer)), desc.yaw_velocity_limit + kEpsilon);

    const auto dt = joint_output_history_[i].stamp - joint_output_history_[i - 1].stamp;
    const auto acc = (joint_output_history_[i].velocities - joint_output_history_[i - 1].velocities) / dt;
    EXPECT_LE(fabs(acc(kJointIDLeftWheel)), desc.wheel_acceleration_limit + kEpsilon);
    EXPECT_LE(fabs(acc(kJointIDRightWheel)), desc.wheel_acceleration_limit + kEpsilon);
    EXPECT_LE(fabs(acc(kJointIDSteer)), desc.yaw_acceleration_limit + kEpsilon);
  }
}

void OmniBaseJointControllerAccLimitTest::ClearHistory() {
  joint_desired_history_all_.insert(joint_desired_history_all_.end(),
                                    joint_desired_history_.begin(), joint_desired_history_.end() - 1);
  joint_desired_history_.erase(joint_desired_history_.begin(), joint_desired_history_.end() - 1);

  joint_output_history_all_.insert(joint_output_history_all_.end(),
                                    joint_output_history_.begin(), joint_output_history_.end() - 1);
  joint_output_history_.erase(joint_output_history_.begin(), joint_output_history_.end() - 1);

  base_desired_history_all_.insert(base_desired_history_all_.end(),
                                    base_desired_history_.begin(), base_desired_history_.end() - 1);
  base_desired_history_.erase(base_desired_history_.begin(), base_desired_history_.end() - 1);

  base_output_history_all_.insert(base_output_history_all_.end(),
                                   base_output_history_.begin(), base_output_history_.end() - 1);
  base_output_history_.erase(base_output_history_.begin(), base_output_history_.end() - 1);
}

void OmniBaseJointControllerAccLimitTest::ValidateBaseOutputLast(const double linear_x,
                                                                 const double linear_y,
                                                                 const double angular_z) {
  EXPECT_FALSE(base_output_history_.empty());
  EXPECT_NEAR(base_output_history_.back().velocities(kIndexBaseX), linear_x, kEpsilon);
  EXPECT_NEAR(base_output_history_.back().velocities(kIndexBaseY), linear_y, kEpsilon);
  EXPECT_NEAR(base_output_history_.back().velocities(kIndexBaseTheta), angular_z, kEpsilon);
}

void OmniBaseJointControllerAccLimitTest::ValidateBaseOutputMonotonic(
    const int32_t index, const Trend trend, const double value) {
  ValidateBaseOutputWeakMonotonic(index, trend, value, kUpdatePeriod);
}

void OmniBaseJointControllerAccLimitTest::ValidateBaseOutputWeakMonotonic(
    const int32_t index, const Trend trend, const double value, const double distance) {
  const auto distance_int = static_cast<uint32_t>(distance / kUpdatePeriod);
  switch (trend) {
    case Trend::kUp:
      for (auto i = distance_int; i < base_output_history_.size(); ++i) {
        EXPECT_GE(base_output_history_[i].velocities[index] + kEpsilon,
                  base_output_history_[i - distance_int].velocities[index]);
      }
      for (auto i = 0; i < base_output_history_.size(); ++i) {
        EXPECT_LE(base_output_history_[i].velocities[index], value + kEpsilon);
      }
      return;
    case Trend::kDown:
      for (auto i = distance_int; i < base_output_history_.size(); ++i) {
        EXPECT_LE(base_output_history_[i].velocities[index] - kEpsilon,
                  base_output_history_[i - distance_int].velocities[index]);
      }
      for (auto i = 0; i < base_output_history_.size(); ++i) {
        EXPECT_GE(base_output_history_[i].velocities[index], value - kEpsilon);
      }
      return;
    case Trend::kFlat:
      ValidateBaseOutputZero(index, kEpsilon);
      return;
  }
}

void OmniBaseJointControllerAccLimitTest::ValidateBaseOutputZero(const int32_t index, const double epsilon) {
  for (size_t i = 1; i < base_output_history_.size(); ++i) {
    EXPECT_NEAR(base_output_history_[i].velocities[index], 0.0, epsilon);
  }
}

void OmniBaseJointControllerAccLimitTest::ValidateBaseOutputPattern(
    const int32_t index, const double distance, const Trend first_trend, const uint32_t trend_num) {
  const auto distance_int = static_cast<uint32_t>(distance / kUpdatePeriod);
  auto prev_trend = Trend::kFlat;
  std::vector<Trend> trends;
  for (auto i = distance_int; i < base_output_history_.size(); ++i) {
    const auto diff = base_output_history_[i].velocities[index] -
        base_output_history_[i - distance_int].velocities[index];
    Trend trend;
    // Allow some noise.
    constexpr double kWeakEpsilon = 1.0e-3;
    if (diff > kWeakEpsilon) {
      trend = Trend::kUp;
    } else if (diff < -kWeakEpsilon) {
      trend = Trend::kDown;
    } else {
      trend = Trend::kFlat;
    }
    if (trend != prev_trend && trend != Trend::kFlat) {
      trends.push_back(trend);
      prev_trend = trend;
    }
  }
  EXPECT_EQ(trends.size(), trend_num);

  const Trend second_trend = (first_trend == Trend::kUp) ? Trend::kDown : Trend::kUp;
  for (auto i = 0; i < trends.size(); ++i) {
    if (i % 2 == 0) {
      EXPECT_EQ(trends[i], first_trend);
    } else {
      EXPECT_EQ(trends[i], second_trend);
    }
  }
}

void OmniBaseJointControllerAccLimitTest::MoveForwardFromIdleHelper(
    const RobotDescription& desc, const double velocity) {
  SetUpImpl(desc);

  for (; stamp_ < FirstMoveTimeout; ) {
    UpdateOnce(velocity, 0.0, 0.0);
  }
  ValidateJointOutput(desc);
  ValidateBaseOutputLast(velocity, 0.0, 0.0);

  ValidateBaseOutputMonotonic(kIndexBaseX, Trend::kUp, velocity);
  ValidateBaseOutputZero(kIndexBaseY);
  ValidateBaseOutputZero(kIndexBaseTheta);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbMoveForwardFromIdleWithSafeSpeed) {
  const auto desc = GetHsrbDescription();
  MoveForwardFromIdleHelper(desc, desc.safe_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcMoveForwardFromIdleWithSafeSpeed) {
  const auto desc = GetHsrcDescription();
  MoveForwardFromIdleHelper(desc, desc.safe_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfMoveForwardFromIdleWithSafeSpeed) {
  const auto desc = GetHsrfDescription();
  MoveForwardFromIdleHelper(desc, desc.safe_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbMoveForwardFromIdleWithMaxSpeed) {
  const auto desc = GetHsrbDescription();
  MoveForwardFromIdleHelper(desc, desc.max_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcMoveForwardFromIdleWithMaxSpeed) {
  const auto desc = GetHsrcDescription();
  MoveForwardFromIdleHelper(desc, desc.max_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfMoveForwardFromIdleWithMaxSpeed) {
  const auto desc = GetHsrfDescription();
  MoveForwardFromIdleHelper(desc, desc.max_base_velocity);
}

void OmniBaseJointControllerAccLimitTest::MoveLeftFromIdleHelper(
    const RobotDescription& desc, const double velocity, const double tolerance) {
  SetUpImpl(desc);

  for (; stamp_ < FirstMoveTimeout; ) {
    UpdateOnce(0.0, velocity, 0.0);
  }
  ValidateJointOutput(desc);
  ValidateBaseOutputLast(0.0, velocity, 0.0);

  // If the acceleration setting doesn't allow smooth turning, it will wobble and not accelerate monotonically.
  ValidateBaseOutputZero(kIndexBaseX, tolerance);
  ValidateBaseOutputWeakMonotonic(kIndexBaseY, Trend::kUp, velocity, 0.2);
  ValidateBaseOutputZero(kIndexBaseTheta);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbMoveLeftFromIdleWithSafeSpeed) {
  const auto desc = GetHsrbDescription();
  MoveLeftFromIdleHelper(desc, desc.safe_base_velocity, 0.030);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcMoveLeftFromIdleWithSafeSpeed) {
  const auto desc = GetHsrcDescription();
  MoveLeftFromIdleHelper(desc, desc.safe_base_velocity, 0.035);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfMoveLeftFromIdleWithSafeSpeed) {
  const auto desc = GetHsrfDescription();
  MoveLeftFromIdleHelper(desc, desc.safe_base_velocity, 0.009);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbMoveLeftFromIdleWithMaxSpeed) {
  const auto desc = GetHsrbDescription();
  MoveLeftFromIdleHelper(desc, desc.max_base_velocity, 0.029);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcMoveLeftFromIdleWithMaxSpeed) {
  const auto desc = GetHsrcDescription();
  MoveLeftFromIdleHelper(desc, desc.max_base_velocity, 0.037);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfMoveLeftFromIdleWithMaxSpeed) {
  const auto desc = GetHsrfDescription();
  MoveLeftFromIdleHelper(desc, desc.max_base_velocity, 0.008);
}

void OmniBaseJointControllerAccLimitTest::MoveBackwardFromIdleHelper(
    const RobotDescription& desc, const double velocity, const double tolerance, const uint32_t trend_num) {
  SetUpImpl(desc);

  for (; stamp_ < FirstMoveTimeout; ) {
    UpdateOnce(-velocity, 0.0, 0.0);
  }
  ValidateJointOutput(desc);
  ValidateBaseOutputLast(-velocity, 0.0, 0.0);

  // If the acceleration setting doesn't allow smooth turning, it will wobble and not accelerate monotonically.
  ValidateBaseOutputPattern(kIndexBaseX, 0.1, Trend::kDown, trend_num);
  ValidateBaseOutputZero(kIndexBaseY, tolerance);
  ValidateBaseOutputZero(kIndexBaseTheta);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbMoveBackwardFromIdleWithSafeSpeed) {
  const auto desc = GetHsrbDescription();
  MoveBackwardFromIdleHelper(desc, desc.safe_base_velocity, 0.048);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcMoveBackwardFromIdleWithSafeSpeed) {
  const auto desc = GetHsrcDescription();
  MoveBackwardFromIdleHelper(desc, desc.safe_base_velocity, 0.064);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfMoveBackwardFromIdleWithSafeSpeed) {
  const auto desc = GetHsrfDescription();
  MoveBackwardFromIdleHelper(desc, desc.safe_base_velocity, 0.160, 3);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbMoveBackwardFromIdleWithMaxSpeed) {
  const auto desc = GetHsrbDescription();
  MoveBackwardFromIdleHelper(desc, desc.max_base_velocity, 0.050);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcMoveBackwardFromIdleWithMaxSpeed) {
  const auto desc = GetHsrcDescription();
  MoveBackwardFromIdleHelper(desc, desc.max_base_velocity, 0.065);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfMoveBackwardFromIdleWithMaxSpeed) {
  const auto desc = GetHsrfDescription();
  MoveBackwardFromIdleHelper(desc, desc.max_base_velocity, 0.160, 3);
}

void OmniBaseJointControllerAccLimitTest::StopOnMovingHelper(const RobotDescription& desc, const double velocity) {
  SetUpImpl(desc);

  for (; stamp_ < FirstMoveTimeout; ) {
    UpdateOnce(velocity, 0.0, 0.0);
  }
  ValidateJointOutput(desc);
  ValidateBaseOutputLast(velocity, 0.0, 0.0);

  ClearHistory();

  for (; stamp_ < SecondMoveTimeout; ) {
    UpdateOnce(0.0, 0.0, 0.0);
  }
  ValidateJointOutput(desc);
  ValidateBaseOutputLast(0.0, 0.0, 0.0);

  ValidateBaseOutputMonotonic(kIndexBaseX, Trend::kDown, 0.0);
  ValidateBaseOutputZero(kIndexBaseY);
  ValidateBaseOutputZero(kIndexBaseTheta);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbStopOnMovingWithSafeSpeed) {
  const auto desc = GetHsrbDescription();
  StopOnMovingHelper(desc, desc.safe_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcStopOnMovingWithSafeSpeed) {
  const auto desc = GetHsrcDescription();
  StopOnMovingHelper(desc, desc.safe_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfStopOnMovingWithSafeSpeed) {
  const auto desc = GetHsrfDescription();
  StopOnMovingHelper(desc, desc.safe_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbStopOnMovingWithMaxSpeed) {
  const auto desc = GetHsrbDescription();
  StopOnMovingHelper(desc, desc.max_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcStopOnMovingWithMaxSpeed) {
  const auto desc = GetHsrcDescription();
  StopOnMovingHelper(desc, desc.max_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfStopOnMovingWithMaxSpeed) {
  const auto desc = GetHsrfDescription();
  StopOnMovingHelper(desc, desc.max_base_velocity);
}

void OmniBaseJointControllerAccLimitTest::MoveLeftOnMovingHelper(
    const RobotDescription& desc, const double velocity,
    const uint32_t x_trend_num, const uint32_t y_trend_num) {
  SetUpImpl(desc);

  for (; stamp_ < FirstMoveTimeout; ) {
    UpdateOnce(velocity, 0.0, 0.0);
  }
  ValidateJointOutput(desc);
  ValidateBaseOutputLast(velocity, 0.0, 0.0);

  ClearHistory();

  for (; stamp_ < SecondMoveTimeout; ) {
    UpdateOnce(0.0, velocity, 0.0);
  }
  ValidateJointOutput(desc);
  ValidateBaseOutputLast(0.0, velocity, 0.0);

  // If the acceleration setting doesn't allow smooth turning, it will not accelerate monotonically.
  ValidateBaseOutputPattern(kIndexBaseX, 0.1, Trend::kDown, x_trend_num);
  ValidateBaseOutputPattern(kIndexBaseY, 0.1, Trend::kUp, y_trend_num);
  ValidateBaseOutputZero(kIndexBaseTheta);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbMoveLeftOnMovingWithSafeSpeed) {
  const auto desc = GetHsrbDescription();
  MoveLeftOnMovingHelper(desc, desc.safe_base_velocity, 3, 5);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcMoveLeftOnMovingWithSafeSpeed) {
  const auto desc = GetHsrcDescription();
  MoveLeftOnMovingHelper(desc, desc.safe_base_velocity, 3);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfMoveLeftOnMovingWithSafeSpeed) {
  const auto desc = GetHsrfDescription();
  MoveLeftOnMovingHelper(desc, desc.safe_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbMoveLeftOnMovingWithMaxSpeed) {
  const auto desc = GetHsrbDescription();
  MoveLeftOnMovingHelper(desc, desc.max_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcMoveLeftOnMovingWithMaxSpeed) {
  const auto desc = GetHsrcDescription();
  MoveLeftOnMovingHelper(desc, desc.max_base_velocity);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfMoveLeftOnMovingWithMaxSpeed) {
  const auto desc = GetHsrfDescription();
  MoveLeftOnMovingHelper(desc, desc.max_base_velocity);
}

void OmniBaseJointControllerAccLimitTest::MoveBackwardOnMovingHelper(
    const RobotDescription& desc, const double velocity, const double tolerance, const uint32_t trend_num) {
  SetUpImpl(desc);

  for (; stamp_ < FirstMoveTimeout; ) {
    // If it's completely forward, the cart won't turn when reversing, so make it slightly diagonal.
    UpdateOnce(velocity * std::cos(0.01), velocity * std::sin(0.01), 0.0);
  }
  ValidateJointOutput(desc);
  ValidateBaseOutputLast(velocity * std::cos(0.01), velocity * std::sin(0.01), 0.0);

  ClearHistory();

  for (; stamp_ < SecondMoveTimeout; ) {
    UpdateOnce(-velocity, 0.0, 0.0);
  }
  ValidateJointOutput(desc);
  ValidateBaseOutputLast(-velocity, 0.0, 0.0);
  // If the acceleration setting doesn't allow smooth turning, it will wobble and not accelerate monotonically.
  ValidateBaseOutputPattern(kIndexBaseX, 0.1, Trend::kDown, trend_num);
  ValidateBaseOutputZero(kIndexBaseY, tolerance);
  ValidateBaseOutputZero(kIndexBaseTheta);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbMoveBackwardOnMovingWithSafeSpeed) {
  const auto desc = GetHsrbDescription();
  MoveBackwardOnMovingHelper(desc, desc.safe_base_velocity, 0.048);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcMoveBackwardOnMovingWithSafeSpeed) {
  const auto desc = GetHsrcDescription();
  MoveBackwardOnMovingHelper(desc, desc.safe_base_velocity, 0.064);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfMoveBackwardOnMovingWithSafeSpeed) {
  const auto desc = GetHsrfDescription();
  MoveBackwardOnMovingHelper(desc, desc.safe_base_velocity, 0.100, 3);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrbMoveBackwardOnMovingWithMaxSpeed) {
  const auto desc = GetHsrbDescription();
  MoveBackwardOnMovingHelper(desc, desc.max_base_velocity, 0.050);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrcMoveBackwardOnMovingWithMaxSpeed) {
  const auto desc = GetHsrcDescription();
  MoveBackwardOnMovingHelper(desc, desc.max_base_velocity, 0.066);
}

TEST_F(OmniBaseJointControllerAccLimitTest, HsrfMoveBackwardOnMovingWithMaxSpeed) {
  const auto desc = GetHsrfDescription();
  MoveBackwardOnMovingHelper(desc, desc.max_base_velocity, 0.083, 3);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
