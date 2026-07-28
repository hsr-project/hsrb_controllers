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
/// @file omni_base_joint_controller.cpp
/// @brief Omnidirectional Cart Joint Controller Class
#include <hsrb_base_controllers/omni_base_joint_controller.hpp>

#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <urdf/model.h>

#include <tmc_utils/robot_description.hpp>

#include "utils.hpp"

namespace {

// Rotational axis command speed limit [rad/s]
constexpr double kYawVelocityLimit = 1.8;
// Wheel command speed limit [rad/s]
constexpr double kWheelVelocityLimit = 8.5;
/// Encoder speed threshold
/// A threshold to filter out rare cases (e.g., once every few hours) where encoder values spike (around 4000 rad/sec).
/// Since abnormal values are around 4000, setting the default value to 1000 will filter out such anomalies.
/// TODO(kazuhito_tanaka): 本質対策がされたら本設定を削除
// Rotational axis encoder speed threshold [rad/s]
constexpr double kYawActualVelocityThreshold = 1000.0;
// Wheel encoder speed threshold [rad/s]
constexpr double kWheelActualVelocityThreshold = 1000.0;

// Retrieve joint names, return an error if not found
bool GetJointName(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& parameter_name, std::string& joint_name_out) {
  joint_name_out = hsrb_base_controllers::GetParameter(node, parameter_name, "");
  if (joint_name_out.empty()) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not find " << parameter_name);
    return false;
  } else {
    return true;
  }
}

// Initialize OmniBaseSize
bool InitializeOmniBaseSize(const rclcpp::Logger& logger,
                            const std::string& robot_description,
                            const std::vector<std::string>& joint_names,
                            hsrb_base_controllers::OmniBaseSize & base_size_out) {
  auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initString(robot_description)) {
    RCLCPP_ERROR_STREAM(logger, "Failed to parse URDF");
    return false;
  }

  const auto steer_joint = urdf->getJoint(joint_names[hsrb_base_controllers::kJointIDSteer]);
  const auto l_wheel_joint = urdf->getJoint(joint_names[hsrb_base_controllers::kJointIDLeftWheel]);
  const auto r_wheel_joint = urdf->getJoint(joint_names[hsrb_base_controllers::kJointIDRightWheel]);
  if (!steer_joint || !l_wheel_joint || !r_wheel_joint) {
    RCLCPP_ERROR(logger, "Could not get joint param from urdf");
    return false;
  }

  base_size_out.tread = fabs(l_wheel_joint->parent_to_joint_origin_transform.position.y -
                              r_wheel_joint->parent_to_joint_origin_transform.position.y);
  base_size_out.caster_offset = fabs(l_wheel_joint->parent_to_joint_origin_transform.position.x);
  base_size_out.wheel_radius = fabs(l_wheel_joint->parent_to_joint_origin_transform.position.z);
  return true;
}

// Calculate the achievable Min/Max speed considering acceleration
void CalculateVelocityMinMax(const double prev,
                             const double vel_limit,
                             const double acc_limit,
                             const double& period,
                             double& vel_min,
                             double& vel_max) {
  vel_max = std::min(prev + acc_limit * period, vel_limit);
  vel_min = std::max(prev - acc_limit * period, -vel_limit);
}

// Calculate the achievable Min/Max speed considering acceleration
void CalculateVelocityMinMax(const Eigen::Vector3d& prev,
                             const Eigen::Vector3d& vel_limit,
                             const Eigen::Vector3d& acc_limit,
                             const double& period,
                             Eigen::Vector3d& vel_min,
                             Eigen::Vector3d& vel_max) {
  for (size_t i = 0; i < 3; ++i) {
    CalculateVelocityMinMax(prev[i], vel_limit[i], acc_limit[i], period, vel_min[i], vel_max[i]);
  }
}

// Check the validity of joint speeds
// Tolerance is an allowable error to avoid rejecting maximum speed
bool IsJointCommandValid(const Eigen::Vector3d& joint_velocities,
                         const Eigen::Vector3d& joint_velocities_min,
                         const Eigen::Vector3d& joint_velocities_max,
                         const double tolerance = 1.0e-10) {
  for (size_t i = 0; i < 3; ++i) {
    if (joint_velocities[i] < joint_velocities_min[i] - tolerance ||
        joint_velocities[i] > joint_velocities_max[i] + tolerance) {
      return false;
    }
  }
  return true;
}

// Calculate the distance from the range of joint speeds
double CalculateJointCommandDistance(const Eigen::Vector3d& joint_velocities,
                                     const Eigen::Vector3d& joint_velocities_min,
                                     const Eigen::Vector3d& joint_velocities_max,
                                     const double tolerance = 1.0e-10) {
  double distance = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    if (joint_velocities[i] < joint_velocities_min[i] - tolerance) {
      distance += joint_velocities_min[i] - joint_velocities[i];
    } else if (joint_velocities[i] > joint_velocities_max[i] + tolerance) {
      distance += joint_velocities[i] - joint_velocities_max[i];
    }
  }
  return distance;
}

double GetStateInterfaceValue(const std::reference_wrapper<hardware_interface::LoanedStateInterface> interface) {
  std::optional<double> opt_value = interface.get().get_optional();
  if (opt_value.has_value()) {
    return opt_value.value();
  } else {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace

namespace hsrb_base_controllers {

// Constructor, initialize parameters
OmniBaseJointControllerBase::OmniBaseJointControllerBase(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    : joint_command_position_(Eigen::Vector3d::Zero()),
      joint_desired_velocity_(Eigen::Vector3d::Zero()),
      joint_output_velocity_(Eigen::Vector3d::Zero()),
      base_output_velocity_(Eigen::Vector3d::Zero()),
      interface_types_(3, hardware_interface::HW_IF_VELOCITY),
      node_(node) {
}

// Initialize parameters
bool OmniBaseJointControllerBase::Init() {
  joint_names_.resize(kNumOmniBaseJointIDs);
  if (!GetJointName(node_, "joints.steer", joint_names_[kJointIDSteer]) ||
      !GetJointName(node_, "joints.l_wheel", joint_names_[kJointIDLeftWheel]) ||
      !GetJointName(node_, "joints.r_wheel", joint_names_[kJointIDRightWheel])) {
    return false;
  }
  if (!InitializeOmniBaseSize(node_->get_logger(), tmc_utils::ResolveRobotDescription(node_),
                              joint_names_, omnibase_size_)) {
    return false;
  }
  twin_drive_ = std::make_shared<TwinCasterDrive>(omnibase_size_);

  command_joint_names_.resize(joint_names_.size());
  command_joint_names_[kJointIDSteer] = GetParameter(
      node_, "command_joints.steer", joint_names_[kJointIDSteer]);
  command_joint_names_[kJointIDLeftWheel] = GetParameter(
      node_, "command_joints.l_wheel", joint_names_[kJointIDLeftWheel]);
  command_joint_names_[kJointIDRightWheel] = GetParameter(
      node_, "command_joints.r_wheel", joint_names_[kJointIDRightWheel]);

  velocity_limit_[kJointIDSteer] = GetPositiveParameter(node_, "yaw_velocity_limit", kYawVelocityLimit);
  const double wheel_velocity_limit = GetPositiveParameter(node_, "wheel_velocity_limit", kWheelVelocityLimit);
  velocity_limit_[kJointIDLeftWheel] = wheel_velocity_limit;
  velocity_limit_[kJointIDRightWheel] = wheel_velocity_limit;

  acceleration_limit_[kJointIDSteer] = GetPositiveParameter(node_, "yaw_acceleration_limit", 1.0e10);
  const double wheel_acceleration_limit = GetPositiveParameter(node_, "wheel_acceleration_limit", 1.0e10);
  acceleration_limit_[kJointIDLeftWheel] = wheel_acceleration_limit;
  acceleration_limit_[kJointIDRightWheel] = wheel_acceleration_limit;

  /// A threshold to filter out rare cases (e.g., once every few hours) where encoder values spike (around 4000 rad/sec).
  /// TODO(kazuhito_tanaka): 本質対策がされたら本設定を削除
  actual_velocity_threshold_[kJointIDSteer] = GetPositiveParameter(
      node_, "yaw_actual_velocity_threshold", kYawActualVelocityThreshold);
  const double wheel_actual_velocity_threshold = GetPositiveParameter(
      node_, "wheel_actual_velocity_threshold", kWheelActualVelocityThreshold);
  actual_velocity_threshold_[kJointIDLeftWheel] = wheel_actual_velocity_threshold;
  actual_velocity_threshold_[kJointIDRightWheel] = wheel_actual_velocity_threshold;

  velocity_filters_.resize(kNumOmniBaseJointIDs);
  std::vector<double> coeff_a = GetParameter(node_, "wheel_command_velocity_filter.a", std::vector<double>());
  std::vector<double> coeff_b = GetParameter(node_, "wheel_command_velocity_filter.b", std::vector<double>());
  if (coeff_a.size() > 0 && coeff_b.size() > 0) {
    Filter<> filter(coeff_a, coeff_b);
    velocity_filters_[kJointIDRightWheel] = filter;
    velocity_filters_[kJointIDLeftWheel] = filter;
  }
  coeff_a = GetParameter(node_, "steer_command_velocity_filter.a", std::vector<double>());
  coeff_b = GetParameter(node_, "steer_command_velocity_filter.b", std::vector<double>());
  if (coeff_a.size() > 0 && coeff_b.size() > 0) {
    velocity_filters_[kJointIDSteer] = Filter<>(coeff_a, coeff_b);
  }

  return true;
}

std::vector<std::string> OmniBaseJointControllerBase::state_interface_names() const {
  std::vector<std::string> names;
  for (const auto& name : joint_names_) {
    names.push_back(name + '/' + hardware_interface::HW_IF_POSITION);
    names.push_back(name + '/' + hardware_interface::HW_IF_VELOCITY);
  }
  return names;
}

/// Interface settings
bool OmniBaseJointControllerBase::Activate(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
                                           std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
  command_interfaces_.clear();
  current_position_interfaces_.clear();
  current_velocity_interfaces_.clear();

  for (auto i = 0; i < command_joint_names_.size(); ++i) {
    // Based on controller_interface::get_ordered_interfaces
    const std::string name = command_joint_names_[i] + '/' + interface_types_[i];
    for (auto& interface : command_interfaces) {
      if ((interface.get_prefix_name() == command_joint_names_[i]) || interface.get_name() == name) {
        command_interfaces_.emplace_back(std::ref(interface));
        break;
      }
    }
  }
  for (const auto& name : joint_names_) {
    for (auto& interface : state_interfaces) {
      if (interface.get_prefix_name() == name) {
        if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
          current_position_interfaces_.emplace_back(std::ref(interface));
        } else if (interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
          current_velocity_interfaces_.emplace_back(std::ref(interface));
        }
      }
    }
  }
  if (command_interfaces_.size() != kNumOmniBaseJointIDs ||
      current_position_interfaces_.size() != kNumOmniBaseJointIDs ||
      current_velocity_interfaces_.size() != kNumOmniBaseJointIDs) {
    return false;
  }

  ResetDesiredSteerPosition();
  return true;
}

/// Get axis position
bool OmniBaseJointControllerBase::GetJointPositions(Eigen::Vector3d& positions_out) const {
  for (auto i = 0; i < kNumOmniBaseJointIDs; ++i) {
    positions_out(i) = GetStateInterfaceValue(current_position_interfaces_[i]);
  }
  return true;
}

/// Get axis speed
bool OmniBaseJointControllerBase::GetJointVelocities(Eigen::Vector3d& velocities_out) const {
  for (auto i = 0; i < kNumOmniBaseJointIDs; ++i) {
    velocities_out(i) = GetStateInterfaceValue(current_velocity_interfaces_[i]);
    if (fabs(velocities_out(i)) > actual_velocity_threshold_[i]) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Too big joint velocity! [right, left, steer]=[%lf, %lf, %lf]",
          velocities_out(kJointIDRightWheel), velocities_out(kJointIDLeftWheel), velocities_out(kJointIDSteer));
      return false;
    }
  }
  return true;
}

/// Calculate command values
void OmniBaseJointControllerBase::SetJointCommand(const double period, const Eigen::Vector3d output_velocity) {
  // In Gazebo, there are cases where the period is zero, and if it's zero, acceleration limit calculations always fail
  // Therefore, skip processing when it's zero
  if (period <= std::numeric_limits<double>::epsilon()) {
    return;
  }

  // Convert command speed in robot body coordinate system to joint command speed
  twin_drive_->Update(GetStateInterfaceValue(current_position_interfaces_[kJointIDSteer]));
  auto joint_output_velocity = twin_drive_->ConvertInverse(output_velocity);

  // Retain the value before applying constraints as desired
  joint_desired_velocity_ = joint_output_velocity;

  // joint_output_velocity_ and base_output_velocity_ are used in calculations as the previous command values
  // Do not update except at the last step

  // Apply limits to the rotational axis speed
  for (auto i = 0; i < kNumOmniBaseJointIDs; ++i) {
    if (fabs(joint_output_velocity(i)) > velocity_limit_(i)) {
      const double ratio = 1.0 / (fabs(joint_output_velocity(i)) / velocity_limit_(i));
      joint_output_velocity *= ratio;
    }
  }

  // Apply a filter to the speed command value
  for (auto i = 0; i < kNumOmniBaseJointIDs; ++i) {
    joint_output_velocity(i) = velocity_filters_[i].update(joint_output_velocity(i));
  }

  // Determine joint speeds that satisfy the acceleration limit
  const auto joint_output_velocity_opt = ApplyAccelerationLimits(joint_output_velocity, joint_output_velocity_, period);
  if (joint_output_velocity_opt) {
    joint_output_velocity = joint_output_velocity_opt.value();
  } else {
    // If not achievable, give up and use the value with only speed limits and filters applied
    RCLCPP_WARN(node_->get_logger(), "Failed to apply acceleration limits");
  }

  // Update the command position of the rotational axis
  joint_command_position_(kJointIDSteer) += joint_output_velocity(kJointIDSteer) * period;

  // Set the command value to the command interface
  joint_output_velocity_ = joint_output_velocity;
  SetCommandToCommandInterface(period);

  // Retain the output result
  base_output_velocity_ = twin_drive_->ConvertForward(joint_output_velocity_);
}

/// Calculate command values
void OmniBaseJointControllerBase::SetJointCommand(const double period, const State& desired_state) {
  // Apply limits
  for (auto i = 0; i < kNumOmniBaseJointIDs; ++i) {
    joint_output_velocity_[i] = std::clamp(desired_state.velocities[i], -velocity_limit_[i], velocity_limit_[i]);
  }
  joint_desired_velocity_ = joint_output_velocity_;

  // Apply limits to the rotational axis speed
  if (fabs(desired_state.velocities[kJointIDSteer]) > velocity_limit_[kJointIDSteer]) {
    joint_command_position_(kJointIDSteer) += joint_output_velocity_(kJointIDSteer) * period;
  } else {
    joint_command_position_(kJointIDSteer) = desired_state.positions[kJointIDSteer];
  }

  // Set the command value to the command interface
  SetCommandToCommandInterface(period);
}

// Determine joint speeds that satisfy the acceleration limit
std::optional<Eigen::Vector3d> OmniBaseJointControllerBase::ApplyAccelerationLimits(
    const Eigen::Vector3d& joint_velocity_current,
    const Eigen::Vector3d& joint_velocity_prev,
    const double period) {
  // Calculate achievable speeds for each axis based on acceleration
  Eigen::Vector3d joint_velocity_min;
  Eigen::Vector3d joint_velocity_max;
  CalculateVelocityMinMax(joint_velocity_prev, velocity_limit_, acceleration_limit_, period,
                          joint_velocity_min, joint_velocity_max);

  // Exit immediately if achievable
  if (IsJointCommandValid(joint_velocity_current, joint_velocity_min, joint_velocity_max)) {
    return joint_velocity_current;
  }

  // Treat the speed with applied limits as the desired speed for the cart
  const auto base_desired_velocity = twin_drive_->ConvertForward(joint_velocity_current);

  // Use ternary search to find the optimal solution from linear interpolation between current and target cart speeds
  // Prefer the right side as it's better to be as close to the target speed as possible
  auto interpolation_cost_func = [&](const double ratio) {
    const auto base_command_candidate = base_desired_velocity * ratio + base_output_velocity_ * (1.0 - ratio);
    const auto joint_command_candidate = twin_drive_->ConvertInverse(base_command_candidate);
    return CalculateJointCommandDistance(joint_command_candidate, joint_velocity_min, joint_velocity_max);
  };

  constexpr double kEpsilon = 1.0e-8;
  const double interpolation_ratio = TernarySearchMinRight(interpolation_cost_func, kEpsilon);
  const double interpolation_distance = interpolation_cost_func(interpolation_ratio);
  if (interpolation_distance == 0.0) {
    return twin_drive_->ConvertInverse(
        base_desired_velocity * interpolation_ratio + base_output_velocity_ * (1.0 - interpolation_ratio));
  }

  // If no linear interpolation, decelerate while maintaining the ratio of current cart speeds
  // Experimentally, it's better to decelerate as much as possible, so prefer the left side
  auto braking_base_cost_func = [&](const double ratio) {
    const auto base_command_candidate = base_output_velocity_ * ratio;
    const auto joint_command_candidate = twin_drive_->ConvertInverse(base_command_candidate);
    return CalculateJointCommandDistance(joint_command_candidate, joint_velocity_min, joint_velocity_max);
  };

  const double braking_base_ratio = TernarySearchMinLeft(braking_base_cost_func, kEpsilon);
  const double braking_distance = braking_base_cost_func(braking_base_ratio);
  if (braking_distance == 0.0) {
    return twin_drive_->ConvertInverse(base_output_velocity_ * braking_base_ratio);
  }

  // Consider maintaining the current speed, but it might be impossible as the cart's state changes
  if (IsJointCommandValid(twin_drive_->ConvertInverse(base_output_velocity_), joint_velocity_min, joint_velocity_max)) {
    return twin_drive_->ConvertInverse(base_output_velocity_);
  }

  // If there's still no solution, decelerate while maintaining the ratio of wheel speeds
  // At this point, the cart may move in an unexpected direction, so it might be better to just let it pass
  // Binary search is sufficient, but reuse the ternary search code
  auto braking_joint_cost_func = [&](const double ratio) {
    const auto joint_command_candidate = joint_velocity_prev * ratio;
    return CalculateJointCommandDistance(joint_command_candidate, joint_velocity_min, joint_velocity_max);
  };

  const double braking_joint_ratio = TernarySearchMinLeft(braking_joint_cost_func, kEpsilon);
  const double braking_joint_distance = braking_joint_cost_func(braking_joint_ratio);
  if (braking_joint_distance == 0.0) {
    return joint_velocity_prev * braking_joint_ratio;
  }

  // Nothing can be done if it reaches here
  return std::nullopt;
}

OmniBaseJointControllerBaseRollPosition::OmniBaseJointControllerBaseRollPosition(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    : OmniBaseJointControllerBase(node) {
  interface_types_[kJointIDSteer] = hardware_interface::HW_IF_POSITION;
}

std::vector<std::string> OmniBaseJointControllerBaseRollPosition::command_interface_names() const {
  std::vector<std::string> names;
  names.push_back(command_joint_names_[kJointIDSteer] + '/' + hardware_interface::HW_IF_POSITION);
  names.push_back(command_joint_names_[kJointIDLeftWheel] + '/' + hardware_interface::HW_IF_VELOCITY);
  names.push_back(command_joint_names_[kJointIDRightWheel] + '/' + hardware_interface::HW_IF_VELOCITY);
  return names;
}

void OmniBaseJointControllerBaseRollPosition::SetCommandToCommandInterface(double period) {
  // Update and set the command position of the rotational axis
  static_cast<void>(
      command_interfaces_[kJointIDSteer].get().set_value(joint_command_position_(kJointIDSteer)));

  // Set the command speed for the wheels
  static_cast<void>(
      command_interfaces_[kJointIDRightWheel].get().set_value(joint_output_velocity_(kJointIDRightWheel)));
  static_cast<void>(
      command_interfaces_[kJointIDLeftWheel].get().set_value(joint_output_velocity_(kJointIDLeftWheel)));
}


std::vector<std::string> OmniBaseJointControllerBaseRollVelocity::command_interface_names() const {
  std::vector<std::string> names;
  names.push_back(command_joint_names_[kJointIDSteer] + '/' + hardware_interface::HW_IF_VELOCITY);
  names.push_back(command_joint_names_[kJointIDLeftWheel] + '/' + hardware_interface::HW_IF_VELOCITY);
  names.push_back(command_joint_names_[kJointIDRightWheel] + '/' + hardware_interface::HW_IF_VELOCITY);
  return names;
}

void OmniBaseJointControllerBaseRollVelocity::SetCommandToCommandInterface(double period) {
  static_cast<void>(
      command_interfaces_[kJointIDSteer].get().set_value(joint_output_velocity_(kJointIDSteer)));
  static_cast<void>(
      command_interfaces_[kJointIDRightWheel].get().set_value(joint_output_velocity_(kJointIDRightWheel)));
  static_cast<void>(
      command_interfaces_[kJointIDLeftWheel].get().set_value(joint_output_velocity_(kJointIDLeftWheel)));
}

}  // namespace hsrb_base_controllers
