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
/// @brief Test class for the kinematic model of an omnidirectional vehicle
#include <cmath>
#include <iostream>
#include <limits>

#include <gtest/gtest.h>

#include <boost/random.hpp>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>

#include <hsrb_base_controllers/twin_caster_drive.hpp>


const double kLinearErrorLimit = 0.0001;
const double kAngularErrorLimit = 0.0001;
const double kJointErrorLimit = 0.0001;
const double kOdomLinearErrorLimit = 0.01;
const double kOdomAngularErrorLimit = 0.01;

namespace hsrb_base_controllers {
const OmniBaseSize kOmniBaseCorrectSize = { 1.0, 1.0, 1.0 };

// Test of initial setup parameters
TEST(TwinCasterDriveTest, InvalidParameter) {
  // Initialization with invalid parameters
  // Can it correctly throw exceptions?
  OmniBaseSize minus_tread_size = { -1.0, 1.0, 1.0 };
  EXPECT_ANY_THROW(TwinCasterDrive minus_tread(minus_tread_size));
  OmniBaseSize zero_tread_size = { 0.0, 1.0, 1.0 };
  EXPECT_ANY_THROW(TwinCasterDrive zero_tread(zero_tread_size));
  OmniBaseSize minus_offset_size = { 1.0, -1.0, 1.0 };
  EXPECT_ANY_THROW(TwinCasterDrive minus_offset(minus_offset_size));
  OmniBaseSize minus_radius_size = { 1.0, 1.0, -1.0 };
  EXPECT_ANY_THROW(TwinCasterDrive minus_radius(minus_radius_size));
  OmniBaseSize zero_radius_size = { 1.0, 1.0, 0.0 };
  EXPECT_ANY_THROW(TwinCasterDrive zero_radius(zero_radius_size));
  EXPECT_NO_THROW(TwinCasterDrive correct(kOmniBaseCorrectSize));
}


// Test whether kinematics and inverse kinematics correspond correctly
TEST(TwinCasterDriveTest, CircularConversion) {
  TwinCasterDrive drive(kOmniBaseCorrectSize);

  // Check if forward and inverse kinematics return to the original value in a state with an arbitrary steering axis angle
  for (int i = -360; i < 360; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 180.0;
    drive.Update(angle);
    Eigen::Vector3d joint_velocities(1.0, 0.0, 0.0);
    Eigen::Vector3d base_velocity = drive.ConvertForward(joint_velocities);
    Eigen::Vector3d joint_velocities2 = drive.ConvertInverse(base_velocity);

    EXPECT_NEAR(joint_velocities2(kJointIDRightWheel),
                joint_velocities(kJointIDRightWheel),
                kJointErrorLimit);
    EXPECT_NEAR(joint_velocities2(kJointIDLeftWheel),
                joint_velocities(kJointIDLeftWheel),
                kJointErrorLimit);
    EXPECT_NEAR(joint_velocities2(kJointIDSteer),
                joint_velocities(kJointIDSteer),
                kJointErrorLimit);
  }
}


// Test whether the conversion at zero velocity is performed correctly
TEST(TwinCasterDriveTest, ZeroVelocityConversion) {
  TwinCasterDrive drive(kOmniBaseCorrectSize);

  // Joint angular velocity -> Platform velocity
  Eigen::Vector3d joint_velocities(0.0, 0.0, 0.0);
  Eigen::Vector3d base_velocity =
      drive.ConvertForward(joint_velocities);
  EXPECT_NEAR(base_velocity(kIndexBaseX), 0.0, kLinearErrorLimit);
  EXPECT_NEAR(base_velocity(kIndexBaseY), 0.0, kLinearErrorLimit);
  EXPECT_NEAR(base_velocity(kIndexBaseTheta), 0.0, kLinearErrorLimit);

  // Platform velocity -> Joint angular velocity
  base_velocity << 0.0, 0.0, 0.0;
  joint_velocities = drive.ConvertInverse(base_velocity);
  EXPECT_NEAR(joint_velocities(kJointIDRightWheel), 0.0, kLinearErrorLimit);
  EXPECT_NEAR(joint_velocities(kJointIDLeftWheel), 0.0, kLinearErrorLimit);
  EXPECT_NEAR(joint_velocities(kJointIDSteer), 0.0, kLinearErrorLimit);
}


// Test whether the conversion from joint angular velocity to platform velocity is performed correctly
TEST(TwinCasterDriveTest, ForwardConversion) {
  TwinCasterDrive drive(kOmniBaseCorrectSize);

  // Move two wheel axes at the same speed to drive straight
  // Can it correctly calculate the platform velocity at arbitrary speed and arbitrary steering axis angle?
  boost::mt19937 rng(static_cast<uint64_t>(time(0)));
  boost::uniform_real<> linear_dist(-100.0, 100.0);
  boost::variate_generator<
      boost::mt19937, boost::uniform_real<> > linear_rand(rng, linear_dist);
  for (int i = -360; i < 360; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 180.0;
    const double velocity = linear_rand();
    drive.Update(angle);
    Eigen::Vector3d joint_velocities(velocity, velocity, 0.0);
    Eigen::Vector3d base_velocity = drive.ConvertForward(joint_velocities);

    EXPECT_NEAR(
        base_velocity(kIndexBaseX), velocity * cos(angle), kLinearErrorLimit);
    EXPECT_NEAR(
        base_velocity(kIndexBaseY), velocity * sin(angle), kLinearErrorLimit);
    EXPECT_NEAR(base_velocity(kIndexBaseTheta), 0.0, kAngularErrorLimit);
  }

  // Output the velocity to move only the steering axis
  // Check if the platform does not move in the translational direction and only outputs rotational velocity
  boost::uniform_real<> angular_dist(-10.0, 10.0);
  boost::variate_generator<
      boost::mt19937, boost::uniform_real<> > angular_rand(rng, angular_dist);
  for (int i = -360; i < 360; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 180.0;
    const double velocity = angular_rand();
    drive.Update(angle);
    Eigen::Vector3d joint_velocities(0.0, 0.0, velocity);
    Eigen::Vector3d base_velocity = drive.ConvertForward(joint_velocities);

    EXPECT_NEAR(base_velocity(kIndexBaseX), 0.0, kLinearErrorLimit);
    EXPECT_NEAR(base_velocity(kIndexBaseY), 0.0, kLinearErrorLimit);
    EXPECT_NEAR(base_velocity(kIndexBaseTheta), -velocity, kAngularErrorLimit);
  }
}


// Test whether the conversion from platform velocity to joint angular velocity is performed correctly
TEST(TwinCasterDriveTest, InverseConversion) {
  TwinCasterDrive drive(kOmniBaseCorrectSize);

  // Drive the platform straight with the steering axis at an arbitrary angle
  // Check if the wheel axes are moving at the correct speed and at the same speed
  boost::mt19937 rng(static_cast<uint64_t>(time(0)));
  boost::uniform_real<> linear_dist(-100.0, 100.0);
  boost::variate_generator<
      boost::mt19937, boost::uniform_real<> > linear_rand(rng, linear_dist);
  for (int i = -360; i < 360; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 180.0;
    const double velocity = linear_rand();
    drive.Update(angle);
    Eigen::Vector3d base_velocity(
        velocity * cos(angle), velocity * sin(angle), 0.0);
    Eigen::Vector3d joint_velocities = drive.ConvertInverse(base_velocity);

    EXPECT_NEAR(
        joint_velocities(kJointIDRightWheel), velocity, kJointErrorLimit);
    EXPECT_NEAR(
        joint_velocities(kJointIDLeftWheel), velocity, kJointErrorLimit);
    EXPECT_NEAR(joint_velocities(kJointIDSteer), 0.0, kJointErrorLimit);
  }

  // Rotate the platform with the steering axis at an arbitrary angle
  // Check if the steering axis is moving at the correct speed
  boost::uniform_real<> angular_dist(-10.0, 10.0);
  boost::variate_generator<
      boost::mt19937, boost::uniform_real<> > angular_rand(rng, angular_dist);
  for (int i = -360; i < 360; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 180.0;
    const double velocity = angular_rand();
    drive.Update(angle);
    Eigen::Vector3d base_velocity(0.0, 0.0, velocity);
    Eigen::Vector3d joint_velocities = drive.ConvertInverse(base_velocity);

    EXPECT_NEAR(joint_velocities(kJointIDRightWheel), 0.0, kJointErrorLimit);
    EXPECT_NEAR(joint_velocities(kJointIDLeftWheel), 0.0, kJointErrorLimit);
    EXPECT_NEAR(joint_velocities(kJointIDSteer), -velocity, kJointErrorLimit);
  }
}


// Test whether the odometry of the platform is correctly calculated from the joint angular velocity
TEST(TwinCasterDriveTest, BaseOdometryUpdate) {
  boost::scoped_ptr<class TwinCasterDrive> drive(
      new TwinCasterDrive(kOmniBaseCorrectSize));

  boost::mt19937 rng(static_cast<uint64_t>(time(0)));
  boost::uniform_real<> joint_dist(-100.0, 100.0);
  boost::variate_generator<
      boost::mt19937, boost::uniform_real<> > rand_r(rng, joint_dist);

  // Can odometry be correctly calculated even when starting from arbitrary joint angles?
  for (int i = 0; i < 10; ++i) {
    drive.reset(new TwinCasterDrive(kOmniBaseCorrectSize));

    double period = 0.001;
    Eigen::Vector3d joint_positions(rand_r(), rand_r(), rand_r());
    Eigen::Vector3d joint_velocities = Eigen::Vector3d::Zero();
    Eigen::Vector3d base_odometry =
        drive->UpdateOdometry(period, joint_positions, joint_velocities);

    // Is the odometry set to zero initially?
    EXPECT_NEAR(base_odometry(kIndexBaseX), 0.0, kLinearErrorLimit);
    EXPECT_NEAR(base_odometry(kIndexBaseY), 0.0, kLinearErrorLimit);
    EXPECT_NEAR(base_odometry(kIndexBaseTheta), 0.0, kAngularErrorLimit);

    double velocity = 1.0;
    int32_t update_num = 1000;
    double expected_distance =
        velocity * period * static_cast<double>(update_num);

    // Specify joint angular velocities to perform translational movement -> in-place rotation
    // Is the odometry calculated at the correct position?
    Eigen::Vector3d base_velocity(velocity, 0.0 , 0.0);
    for (int j = 0; j < update_num; ++j) {
      drive->Update(joint_positions(kJointIDSteer));
      joint_velocities = drive->ConvertInverse(base_velocity);
      base_odometry =
          drive->UpdateOdometry(period, joint_positions, joint_velocities);
      joint_positions += joint_velocities * period;
    }

    EXPECT_NEAR(
        base_odometry(kIndexBaseX), expected_distance, kOdomLinearErrorLimit);
    EXPECT_NEAR(base_odometry(kIndexBaseY), 0.0, kOdomLinearErrorLimit);
    EXPECT_NEAR(base_odometry(kIndexBaseTheta), 0.0, kOdomAngularErrorLimit);


    base_velocity << 0.0, velocity, 0.0;
    for (int j = 0; j < update_num; ++j) {
      drive->Update(joint_positions(kJointIDSteer));
      joint_velocities = drive->ConvertInverse(base_velocity);
      base_odometry =
          drive->UpdateOdometry(period, joint_positions, joint_velocities);
      joint_positions += joint_velocities * period;
    }

    EXPECT_NEAR(
        base_odometry(kIndexBaseX), expected_distance, kOdomLinearErrorLimit);
    EXPECT_NEAR(
        base_odometry(kIndexBaseY), expected_distance, kOdomLinearErrorLimit);
    EXPECT_NEAR(base_odometry(kIndexBaseTheta), 0.0, kOdomAngularErrorLimit);

    base_velocity << 0.0, 0.0, velocity;
    for (int j = 0; j < update_num; ++j) {
      drive->Update(joint_positions(kJointIDSteer));
      joint_velocities = drive->ConvertInverse(base_velocity);
      base_odometry =
          drive->UpdateOdometry(period, joint_positions, joint_velocities);
      joint_positions += joint_velocities * period;
    }

    EXPECT_NEAR(
        base_odometry(kIndexBaseX), expected_distance, kOdomLinearErrorLimit);
    EXPECT_NEAR(
        base_odometry(kIndexBaseY), expected_distance, kOdomLinearErrorLimit);
    EXPECT_NEAR(
        base_odometry(kIndexBaseTheta), expected_distance, kOdomAngularErrorLimit);
  }
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
