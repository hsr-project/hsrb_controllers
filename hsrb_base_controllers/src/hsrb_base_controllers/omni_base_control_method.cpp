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
/// @file omni_base_control_method.cpp
/// @brief Omnidirectional Cart Control Mode Class

#include <hsrb_base_controllers/omni_base_control_method.hpp>

#include "utils.hpp"

namespace {
// Default value for cart speed command interruption judgment time [s]
constexpr double kDefaultCommandTimeout = 0.5;
// Threshold for the magnitude of speed to be judged as stopped
constexpr double kStopVelocityThreshold = 0.001;
// Time margin to judge as stopped for path following [s]
constexpr double kStopTimeMergin = 0.2;
// Control P gain for trajectory following deviation
constexpr double kDefaultPGain = 1.0;

// Create an array to rearrange two name arrays with different orders
std::vector<uint32_t> MakePermutationVector(
    const std::vector<std::string>& names1,
    const std::vector<std::string>& names2) {
  // Exit if the input array sizes do not match
  if (names1.size() != names2.size()) {
    return std::vector<uint32_t>();
  }

  // Find matching names and create an array for rearrangement
  std::vector<uint32_t> permutation_vector(names1.size());
  for (std::vector<std::string>::const_iterator it1 = names1.begin(); it1 != names1.end(); ++it1) {
    std::vector<std::string>::const_iterator it2 = std::find(names2.begin(), names2.end(), *it1);
    if (names2.end() == it2) {
      return std::vector<uint32_t>();
    } else {
      const uint32_t t1 = std::distance(names1.begin(), it1);
      const uint32_t t2 = std::distance(names2.begin(), it2);
      permutation_vector[t1] = t2;
    }
  }
  return permutation_vector;
}
}  // namespace

namespace hsrb_base_controllers {

OmniBaseVelocityControl::OmniBaseVelocityControl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    : node_(node) {
  // Get the speed command interruption judgment time
  command_timeout_ = GetParameter(node, "command_timeout", kDefaultCommandTimeout);
  if (command_timeout_ <= 0.0) {
    RCLCPP_INFO(
      node->get_logger(),
      "command_timeout must be positive. Use default value [%lf]", kDefaultCommandTimeout);
    command_timeout_ = kDefaultCommandTimeout;
  }
  // Also call in the constructor and initialize just in case
  Activate();
}

// Initialize
void OmniBaseVelocityControl::Activate() {
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_velocity_ = Eigen::Vector3d::Zero();
  last_velocity_subscribed_time_ = node_->get_clock()->now();
}

// Get the command speed
Eigen::Vector3d OmniBaseVelocityControl::GetOutputVelocity() {
  std::lock_guard<std::mutex> lock(command_mutex_);

  Eigen::Vector3d output_velocity = command_velocity_;
  if (node_->get_clock()->now() - last_velocity_subscribed_time_ > rclcpp::Duration::from_seconds(command_timeout_)) {
    output_velocity = Eigen::Vector3d::Zero();
  }
  return output_velocity;
}

// Update the command speed
void OmniBaseVelocityControl::UpdateCommandVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) {
  std::lock_guard<std::mutex> lock(command_mutex_);

  command_velocity_ << msg->linear.x, msg->linear.y, msg->angular.z;
  last_velocity_subscribed_time_ = node_->get_clock()->now();
}

// Constructor
OmniBaseTrajectoryControl::OmniBaseTrajectoryControl(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<std::string>& cordinates)
    : node_(node), coordinate_names_(cordinates) {
  open_loop_control_ = GetParameter(node, "open_loop_control", false);

  default_tolerances_ = get_segment_tolerances(node, cordinates);
}

// Initialization
void OmniBaseTrajectoryControl::Activate() {
  trajectory_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  trajectory_active_ptr_ = &trajectory_ptr_;
  trajectory_msg_buffer_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());
  has_last_command_state_ = false;
}

// Update the trajectory being followed, return true if a trajectory exists
bool OmniBaseTrajectoryControl::UpdateActiveTrajectory() {
  const auto current_msg = trajectory_ptr_->get_trajectory_msg();
  const auto new_msg = trajectory_msg_buffer_.readFromRT();
  if (current_msg != *new_msg) {
    trajectory_ptr_->update(*new_msg);
  }
  if (trajectory_active_ptr_ && (*trajectory_active_ptr_)->has_trajectory_msg()) {
    if ((*trajectory_active_ptr_)->get_trajectory_msg()->points.empty()) {
      return false;
    } else {
      return true;
    }
  } else {
    return false;
  }
}

// Get the target state for trajectory following
bool OmniBaseTrajectoryControl::SampleDesiredState(
    const rclcpp::Time& time,
    const std::vector<double>& current_positions,
    const std::vector<double>& current_velocities,
    trajectory_msgs::msg::JointTrajectoryPoint& desired_state,
    bool& before_last_point,
    double& time_from_point) {
  if (!(*trajectory_active_ptr_)->is_sampled_already()) {
    if (open_loop_control_ && has_last_command_state_) {
      (*trajectory_active_ptr_)->set_point_before_trajectory_msg(last_sampled_time_, last_command_state_);
    } else {
      trajectory_msgs::msg::JointTrajectoryPoint current_state;
      current_state.positions = current_positions;
      current_state.velocities = current_velocities;
      (*trajectory_active_ptr_)->set_point_before_trajectory_msg(time, current_state);
    }
  }
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator start_segment_it;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator end_segment_it;
  const bool is_ok = (*trajectory_active_ptr_)->sample(time,
      joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION,
      desired_state, start_segment_it, end_segment_it);
  if (is_ok) {
    before_last_point = end_segment_it != (*trajectory_active_ptr_)->end();
    const rclcpp::Time start_stamp = (*trajectory_active_ptr_)->time_from_start();
    const rclcpp::Time end_stamp = start_stamp + start_segment_it->time_from_start;
    time_from_point = time.seconds() - end_stamp.seconds();

    last_sampled_time_ = time;
    last_command_state_ = desired_state;
    has_last_command_state_ = true;
  }
  return is_ok;
}

// Validate the input trajectory command
bool OmniBaseTrajectoryControl::ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) const {
  // Invalid if joint names do not match
  if (trajectory.joint_names.size() != coordinate_names_.size()) {
    RCLCPP_ERROR(node_->get_logger(), "Trajectory's joint_size mismatch.");
    return false;
  }
  for (const auto& joint_name : trajectory.joint_names) {
    if (std::find(coordinate_names_.begin(), coordinate_names_.end(), joint_name) == coordinate_names_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's joint_name mismatch.");
      return false;
    }
  }

  // Check if each JointTrajectoryPoint is valid
  double last_time = -std::numeric_limits<double>::max();
  for (const auto& point : trajectory.points) {
    // Invalid if the number of position elements does not match the number of joints
    if (point.positions.size() != coordinate_names_.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's position size is wrong.");
      return false;
    }
    // Invalid if the number of velocity elements does not match the number of joints
    // Allow if velocity is empty
    if (!point.velocities.empty() && point.velocities.size() != coordinate_names_.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's velocity size is wrong.");
      return false;
    }
    // Invalid if the number of acceleration elements does not match the number of joints
    // Allow if acceleration is empty
    if (!point.accelerations.empty() && point.accelerations.size() != coordinate_names_.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's acceleration size is wrong.");
      return false;
    }
    // Invalid if time_from_start is regressive
    double time_from_start = static_cast<rclcpp::Duration>(point.time_from_start).seconds();
    if (time_from_start - last_time <= 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "Trajectory's time_from_start is going reverse.");
      return false;
    }
    last_time = time_from_start;
  }
  return true;
}

// Reset the currently followed trajectory
void OmniBaseTrajectoryControl::ResetCurrentTrajectory() {
  trajectory_msgs::msg::JointTrajectory empty_msg;
  empty_msg.header.stamp = rclcpp::Time(0);
  trajectory_msg_buffer_.writeFromNonRT(std::make_shared<trajectory_msgs::msg::JointTrajectory>(empty_msg));
  has_last_command_state_ = false;
}

// End trajectory following if conditions are met
void OmniBaseTrajectoryControl::TerminateControl(const rclcpp::Time& time, const double current_velocity) {
  if (!trajectory_active_ptr_ || !(*trajectory_active_ptr_)->has_trajectory_msg()) {
    return;
  }
  if ((*trajectory_active_ptr_)->get_trajectory_msg()->points.empty()) {
    return;
  }

  const double time_from_start = (time - (*trajectory_active_ptr_)->time_from_start()).seconds();
  // End trajectory following if the scheduled following time has passed and the current state is stationary
  const rclcpp::Duration command_trajectory_period = (--((*trajectory_active_ptr_)->end()))->time_from_start;
  if ((time_from_start > command_trajectory_period.seconds() + kStopTimeMergin) &&
      (current_velocity < stop_velocity_threshold_)) {
    ResetCurrentTrajectory();
  }
}

int32_t OmniBaseTrajectoryControl::CheckTorelances(const ControllerState& state,
                                                   const bool before_last_point,
                                                   const double time_from_trajectory_end) {
  trajectory_msgs::msg::JointTrajectoryPoint error;
  Convert(state.error, error);

  if (before_last_point) {
    // Since trajectory is being followed, just check if the path is deviating
    for (uint32_t i = 0; i < active_tolerances_.state_tolerance.size(); ++i) {
      if (!check_state_tolerance_per_joint(error, i, active_tolerances_.state_tolerance[i])) {
        RCLCPP_ERROR(node_->get_logger(), "Path tolerance violated.");
        return control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      }
    }
  } else {
    // Check if the goal is reached, wait if within the time
    bool abort = false;
    for (uint32_t i = 0; i < active_tolerances_.goal_state_tolerance.size(); ++i) {
      if (!check_state_tolerance_per_joint(error, i, active_tolerances_.goal_state_tolerance[i])) {
        abort = true;
        break;
      }
    }
    if (!abort) {
      return control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    } else if (active_tolerances_.goal_time_tolerance != 0.0) {
      // Using != with 0.0 is risky, but since the default value is 0.0, proceed with this
      if (time_from_trajectory_end > active_tolerances_.goal_time_tolerance) {
        RCLCPP_ERROR(node_->get_logger(), "Goal tolerance violated.");
        return control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
      }
    }
  }
  // Defined error codes are 0 or less, so return a positive number to indicate none of them
  return 1;
}

// Constructor, initialize parameters
OmniBaseOdomTrajectoryControl::OmniBaseOdomTrajectoryControl(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<std::string>& cordinates)
    : OmniBaseTrajectoryControl(node, cordinates) {
  stop_velocity_threshold_ = GetPositiveParameter(node, "stop_velocity_threshold", kStopVelocityThreshold);
  feedback_gain_(kIndexBaseX) = GetPositiveParameter(node, "odom_x.p_gain", kDefaultPGain);
  feedback_gain_(kIndexBaseY) = GetPositiveParameter(node, "odom_y.p_gain", kDefaultPGain);
  feedback_gain_(kIndexBaseTheta) = GetPositiveParameter(node, "odom_t.p_gain", kDefaultPGain);
}

// Get the command speed
Eigen::Vector3d OmniBaseOdomTrajectoryControl::GetOutputVelocity(
    const ControllerState& base_state) {
  // Add a term proportional to the position difference to the command speed to make it the target speed
  const Eigen::Vector3d desired(base_state.desired.velocities.data());
  const Eigen::Vector3d error(base_state.error.positions.data());
  Eigen::Vector3d output_velocity = desired + feedback_gain_.cwiseProduct(error);

  // Convert the speed in the reference coordinate system to the upper body coordinate system
  const double current_yaw = base_state.actual.positions.at(kIndexBaseTheta);
  Eigen::Matrix3d robot_to_floor;
  robot_to_floor <<
      std::cos(current_yaw), std::sin(current_yaw), 0.0,
     -std::sin(current_yaw), std::cos(current_yaw), 0.0,
      0.0, 0.0, 1.0;
  output_velocity = robot_to_floor * output_velocity;

  return output_velocity;
}

// Update the following trajectory
void OmniBaseOdomTrajectoryControl::AcceptTrajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
    const Eigen::Vector3d& base_positions) {
  // Stop if there are no points in the input trajectory
  if (trajectory->points.empty()) {
    ResetCurrentTrajectory();
    return;
  }

  // Create a rearrangement array for the input message
  const std::vector<std::string> trajectory_joint_names = trajectory->joint_names;
  const std::vector<uint32_t> permutation_vector = MakePermutationVector(coordinate_names_, trajectory_joint_names);

  // Create a trajectory considering the order of axis name descriptions
  trajectory_msgs::msg::JointTrajectory permutated_trajectory;
  permutated_trajectory.header = trajectory->header;
  permutated_trajectory.joint_names = trajectory->joint_names;
  for (const auto& input_point : trajectory->points) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto index : permutation_vector) {
      if (input_point.positions.size() == permutation_vector.size()) {
        point.positions.push_back(input_point.positions.at(index));
      }
      if (input_point.velocities.size() == permutation_vector.size()) {
        point.velocities.push_back(input_point.velocities.at(index));
      }
      if (input_point.accelerations.size() == permutation_vector.size()) {
        point.accelerations.push_back(input_point.accelerations.at(index));
      }
    }
    point.time_from_start = input_point.time_from_start;
    permutated_trajectory.points.push_back(point);
  }
  // Correct the turning axis
  double prev_position;
  if (open_loop_control_ && has_last_command_state_) {
    prev_position = last_command_state_.positions[kIndexBaseTheta];
  } else {
    prev_position = base_positions[kIndexBaseTheta];
  }
  for (auto& point : permutated_trajectory.points) {
    double diff = angles::shortest_angular_distance(prev_position, point.positions[kIndexBaseTheta]);
    point.positions[kIndexBaseTheta] = prev_position + diff;
    prev_position = point.positions[kIndexBaseTheta];
  }

  active_tolerances_ = default_tolerances_;

  trajectory_msg_buffer_.writeFromNonRT(
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(permutated_trajectory));
}

// End trajectory following if conditions are met
void OmniBaseOdomTrajectoryControl::TerminateControl(const rclcpp::Time& time, const ControllerState& base_state) {
  const Eigen::Vector3d current_velocity(base_state.actual.velocities.data());
  OmniBaseTrajectoryControl::TerminateControl(time, current_velocity.norm());
}

// Constructor, initialize parameters
OmniBaseRollTrajectoryControl::OmniBaseRollTrajectoryControl(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<std::string>& cordinates)
    : OmniBaseTrajectoryControl(node, cordinates) {
  stop_velocity_threshold_ = GetPositiveParameter(node, "roll_stop_velocity_threshold", kStopVelocityThreshold);
}

// Update the following trajectory
void OmniBaseRollTrajectoryControl::AcceptTrajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory,
    const Eigen::Vector3d& /* base_positions */) {
  // Stop if there are no points in the input trajectory
  if (trajectory->points.empty()) {
    ResetCurrentTrajectory();
    return;
  }

  trajectory_msgs::msg::JointTrajectory add_trajectory;
  add_trajectory.header = trajectory->header;
  add_trajectory.joint_names = trajectory->joint_names;
  add_trajectory.points.push_back(trajectory->points.back());

  active_tolerances_ = default_tolerances_;

  trajectory_msg_buffer_.writeFromNonRT(std::make_shared<trajectory_msgs::msg::JointTrajectory>(add_trajectory));
}

// End trajectory following if conditions are met
void OmniBaseRollTrajectoryControl::TerminateControl(const rclcpp::Time& time, const ControllerState& joint_state) {
  OmniBaseTrajectoryControl::TerminateControl(time, joint_state.actual.velocities[0]);
}

}  // namespace hsrb_base_controllers
