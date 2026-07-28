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
/// @file twin_caster_drive.hpp
/// Reference: Masayoshi Wada, Design and Omnidirectional Movement Control of a Four-Wheel Drive Electric Wheelchair,
///       Journal of the Robotics Society of Japan Vol.27 No.3, pp.314-324, 2009
/// Implement the kinematic model of the omnidirectional movement mechanism described in the above paper.
#ifndef HSRB_BASE_CONTROLLERS_TWIN_CASTER_DRIVE_HPP_
#define HSRB_BASE_CONTROLLERS_TWIN_CASTER_DRIVE_HPP_

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include <boost/noncopyable.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <angles/angles.h>

namespace hsrb_base_controllers {

// Index of each joint axis of the omnidirectional cart
enum OmniBaseJointID {
  kJointIDRightWheel,
  kJointIDLeftWheel,
  kJointIDSteer,
  kNumOmniBaseJointIDs
};

// Index of robot state position and orientation
enum BaseCoordinateID {
  kIndexBaseX,
  kIndexBaseY,
  kIndexBaseTheta,
  kNumBaseCoordinateIDs
};

/// Dimensional information of the cart
struct OmniBaseSize {
  // Tread[m]
  double tread;
  // Caster offset[m]
  double caster_offset;
  // Wheel radius[m]
  double wheel_radius;
};

/// @brief Kinematic model of the omnidirectional movement mechanism
class TwinCasterDrive : private boost::noncopyable {
 public:
  using Ptr = std::shared_ptr<TwinCasterDrive>;

  explicit TwinCasterDrive(const OmniBaseSize& omnibase_size)
      : tread_(omnibase_size.tread),
        caster_offset_(omnibase_size.caster_offset),
        wheel_radius_(omnibase_size.wheel_radius),
        joint_pos_initialized_(false) {
    if (tread_ < std::numeric_limits<double>::epsilon()) {
      throw std::domain_error("Tread must be greater than zero");
    }
    if (caster_offset_ < 0) {
      throw std::domain_error(
          "Caster offset must be greater than or equal to zero");
    }
    if (wheel_radius_ < std::numeric_limits<double>::epsilon()) {
      throw std::domain_error("Wheel radius must be greater than zero");
    }
    Update(0.0);
    caster_odometry_ << 0.0, 0.0, 0.0;
  }

  /// Calculate the velocity of the platform realized from the given joint velocities
  /// @param joint_velocity [in] Joint angular velocity (in the order of right wheel, left wheel, and turning axis)
  Eigen::Vector3d ConvertForward(const Eigen::Vector3d& joint_velocity) const {
    return jacobian_ * joint_velocity;
  }

  /// Calculate the required joint velocities from the given platform velocity.
  /// @param base_velocity [in] Platform velocity (forward/backward, left/right, rotation in the platform coordinate system)
  Eigen::Vector3d ConvertInverse(const Eigen::Vector3d& base_velocity) const {
    return inverse_jacobian_ * base_velocity;
  }

  /// Update the offset angle between the platform and caster cart.
  /// @param caster_position [in] Offset angle (rad) between the platform and caster cart
  void Update(double caster_position) {
    const double r = wheel_radius_;
    const double s = caster_offset_;
    const double w = tread_;
    const double cos_v = std::cos(caster_position);
    const double sin_v = std::sin(caster_position);
    const double j11 = r * cos_v * 0.5 - r * s * sin_v / w;
    const double j12 = r * cos_v * 0.5 + r * s * sin_v / w;
    const double j21 = r * sin_v * 0.5 + r * s * cos_v / w;
    const double j22 = r * sin_v * 0.5 - r * s * cos_v / w;
    const double j31 = r / w;
    const double j32 = -r / w;
    // It seems that the sign of the second term on the right-hand side of equation (19) regarding angular velocity in the reference paper is reversed.
    jacobian_ << j11, j12, 0,
                 j21, j22, 0,
                 j31, j32, -1.0;
    inverse_jacobian_ = jacobian_.inverse();
  }

  /// Update the platform odometry.
  Eigen::Vector3d UpdateOdometry(double period,
                                 const Eigen::Vector3d& joint_position,
                                 const Eigen::Vector3d& joint_velocity) {
    const double steer_joint_pos = joint_position[kJointIDSteer];
    if (!joint_pos_initialized_) {
      caster_odometry_ << -caster_offset_ * std::cos(steer_joint_pos),
                          -caster_offset_ * std::sin(steer_joint_pos),
                          steer_joint_pos;
      joint_pos_initialized_ = true;
    }

    const double dt = period;
    const double wr = joint_velocity[kJointIDRightWheel];
    const double wl = joint_velocity[kJointIDLeftWheel];
    const double vr = wr * wheel_radius_;
    const double vl = wl * wheel_radius_;
    const double v = (vr + vl) * 0.5;
    const double w = (vr - vl) / tread_;

    // Update the cart odometry
    caster_odometry_[kIndexBaseTheta] += w * dt;
    const double caster_odom_theta = caster_odometry_[kIndexBaseTheta];
    const double delta_x = v * std::cos(caster_odom_theta) * dt;
    const double delta_y = v * std::sin(caster_odom_theta) * dt;
    caster_odometry_[kIndexBaseX] += delta_x;
    caster_odometry_[kIndexBaseY] += delta_y;

    // Update the platform odometry from the cart odometry and the steering axis angle
    const double odom_x = caster_odometry_[kIndexBaseX] +
                          caster_offset_ * std::cos(caster_odom_theta);
    const double odom_y = caster_odometry_[kIndexBaseY] +
                          caster_offset_ * std::sin(caster_odom_theta);
    const double odom_yaw = caster_odom_theta - steer_joint_pos;

    Eigen::Vector3d base_odometry;
    base_odometry << odom_x, odom_y, angles::normalize_angle(odom_yaw);

    return base_odometry;
  }

 private:
  const double tread_;            // [m]
  const double caster_offset_;    // [m]
  const double wheel_radius_;     // [m]
  Eigen::Matrix3d jacobian_;
  Eigen::Matrix3d inverse_jacobian_;

  Eigen::Vector3d caster_odometry_;

  bool joint_pos_initialized_;
};

}  // namespace hsrb_base_controllers

#endif/*HSRB_BASE_CONTROLLERS_TWIN_CASTER_DRIVE_HPP_*/
