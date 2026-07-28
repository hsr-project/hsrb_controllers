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
/// @file omni_base_controller.cpp
/// @brief Omnidirectional Cart Controller Class

#include <hsrb_base_controllers/omni_base_controller.hpp>

#include <pluginlib/class_list_macros.hpp>

#include "utils.hpp"

namespace {
void ConvertVector(const Eigen::VectorXd& input_vector,
                   std::vector<double>& dst_vector) {
  dst_vector.resize(input_vector.size());
  Eigen::Map<Eigen::VectorXd> map(&dst_vector[0], dst_vector.size());
  map = input_vector;
}
}  // namespace

namespace hsrb_base_controllers {

controller_interface::CallbackReturn OmniBaseController::on_init() {
  // Does nothing in particular; all processing is in on_configure and on_activate
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OmniBaseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names = joint_controller_->command_interface_names();
  return conf;
}

controller_interface::InterfaceConfiguration OmniBaseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names = joint_controller_->state_interface_names();
  return conf;
}

controller_interface::return_type OmniBaseController::update(
    const rclcpp::Time& current_time, const rclcpp::Duration& period) {
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  // Retrieve position and velocity of each axis of the cart
  Eigen::Vector3d joint_positions;
  Eigen::Vector3d joint_velocities;
  if (!joint_controller_->GetJointPositions(joint_positions) ||
      !joint_controller_->GetJointVelocities(joint_velocities)) {
    return controller_interface::return_type::ERROR;
  }

  const double period_sec = period.seconds();
  // Update odometry
  wheel_odometry_->UpdateOdometry(period_sec, joint_positions, joint_velocities);
  base_odometry_->UpdateOdometry(period_sec, wheel_odometry_->odometry(), wheel_odometry_->velocity());

  // Update the cart's following state
  base_state_.Reset(base_odometry_->odometry(), base_odometry_->velocity());

  // Current state of the cart
  joint_state_.Reset(joint_positions, joint_velocities);

  // Current state of the turning axis
  roll_state_.Reset(joint_positions[kJointIDSteer], joint_velocities[kJointIDSteer]);

  if (roll_trajectory_control_->UpdateActiveTrajectory()) {
    // Trajectory following of the turning axis
    trajectory_msgs::msg::JointTrajectoryPoint desired_state;
    bool before_last_point;
    double time_from_point;
    const auto is_valid = roll_trajectory_control_->SampleDesiredState(
        current_time, roll_state_.actual.positions, roll_state_.actual.velocities,
        desired_state, before_last_point, time_from_point);
    if (is_valid) {
      roll_state_.desired.positions = desired_state.positions;
      roll_state_.desired.velocities = desired_state.velocities;
      roll_state_.desired.accelerations = desired_state.accelerations;
      roll_state_.UpdateError();

      joint_state_.desired.positions[kJointIDSteer] = desired_state.positions[0];
      joint_state_.desired.velocities[kJointIDSteer] = desired_state.velocities[0];

      auto status = roll_trajectory_control_->CheckTorelances(roll_state_, before_last_point, time_from_point);
      if (status <= 0) {
        roll_trajectory_action_->UpdateActionResult(status);
        roll_trajectory_control_->ResetCurrentTrajectory();
      } else {
        roll_trajectory_action_->SetFeedback(roll_state_, current_time);
      }
    }

    joint_state_.desired.velocities[kJointIDRightWheel] = 0.0;
    joint_state_.desired.velocities[kJointIDLeftWheel] = 0.0;

    joint_controller_->SetJointCommand(period_sec, joint_state_.desired);
  } else {
    Eigen::Vector3d output_velocity = Eigen::Vector3d::Zero();
    if (odom_trajectory_control_->UpdateActiveTrajectory()) {
      // Trajectory following of odom
      trajectory_msgs::msg::JointTrajectoryPoint desired_state;
      bool before_last_point;
      double time_from_point;
      const auto is_valid = odom_trajectory_control_->SampleDesiredState(
          current_time, base_state_.actual.positions, base_state_.actual.velocities,
          desired_state, before_last_point, time_from_point);
      if (is_valid) {
        base_state_.Reset(
            base_odometry_->odometry(), base_odometry_->velocity(),
            desired_state.positions, desired_state.velocities, desired_state.accelerations);
        output_velocity = odom_trajectory_control_->GetOutputVelocity(base_state_);

        auto status = odom_trajectory_control_->CheckTorelances(base_state_, before_last_point, time_from_point);
        if (status <= 0) {
          odom_trajectory_action_->UpdateActionResult(status);
          odom_trajectory_control_->ResetCurrentTrajectory();
        } else {
          odom_trajectory_action_->SetFeedback(base_state_, current_time);
        }
      }
    } else {
      // Velocity following
      output_velocity = velocity_control_->GetOutputVelocity();
      ConvertVector(output_velocity, base_state_.desired.velocities);
    }

    joint_controller_->SetJointCommand(period_sec, output_velocity);
    ConvertVector(joint_controller_->base_output_velocity(), base_state_.output.velocities);
  }

  // Publish the current state of the cart (command value, current position, difference)
  ConvertVector(joint_controller_->joint_command_position(), joint_state_.desired.positions);
  ConvertVector(joint_controller_->joint_desired_velocity(), joint_state_.desired.velocities);
  ConvertVector(joint_controller_->joint_output_velocity(), joint_state_.output.velocities);
  joint_state_.UpdateError();
  joint_state_publisher_->Publish(joint_state_, current_time);
  base_state_publisher_->Publish(base_state_, current_time);
  base_controller_state_publisher_->Publish(base_state_, current_time);

  // Publish wheel odometry and TF
  wheel_odometry_->PublishOdometry(current_time);

  odom_trajectory_control_->TerminateControl(current_time, base_state_);
  roll_trajectory_control_->TerminateControl(current_time, roll_state_);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn OmniBaseController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  // Retrieve the coordinate axis name of the controlled cart
  const auto base_coordinate_names = GetParameter<std::vector<std::string>>(get_node(), "base_coordinates", {});
  if (base_coordinate_names.size() != kNumBaseCoordinateIDs) {
    RCLCPP_ERROR(get_node()->get_logger(), "The size of joints must be three.");
    return controller_interface::CallbackReturn::ERROR;
  }
  base_roll_joint_name_ = GetParameter<std::string>(get_node(), "joints.steer", "base_roll_joint");
  std::vector<std::string> joint_names = { base_roll_joint_name_ };

  // Joint Controller Class
  const auto command_base_roll_velocity = GetParameter<bool>(get_node(), "use_base_roll_velocity", false);
  if (command_base_roll_velocity) {
    joint_controller_ = std::make_shared<OmniBaseJointControllerBaseRollVelocity>(get_node());
  } else {
    joint_controller_ = std::make_shared<OmniBaseJointControllerBaseRollPosition>(get_node());
  }
  if (!joint_controller_->Init()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Initializing OmniBaseJointController is failed.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Set the publisher for wheel odometry
  wheel_odometry_ = std::make_shared<WheelOdometry>(get_node(), joint_controller_->omnibase_size());
  // Set the publisher for cart odometry
  base_odometry_ = std::make_shared<BaseOdometry>(get_node());

  // Generate the controller
  velocity_control_ = std::make_shared<OmniBaseVelocityControl>(get_node());
  odom_trajectory_control_ = std::make_shared<OmniBaseOdomTrajectoryControl>(get_node(), base_coordinate_names);
  roll_trajectory_control_ = std::make_shared<OmniBaseRollTrajectoryControl>(get_node(), joint_names);

  // Generate subscriber/action server
  velocity_subscriber_ = std::make_shared<CommandVelocitySubscriber>(get_node(), this);
  odom_trajectory_subscriber_ = std::make_shared<CommandTrajectorySubscriber>(
      get_node(), "~/joint_trajectory", this);
  roll_trajectory_subscriber_ = std::make_shared<CommandTrajectorySubscriber>(
      get_node(), "~/roll_joint_trajectory", this);
  odom_trajectory_action_ = std::make_shared<TrajectoryActionServer>(
      get_node(), base_coordinate_names, "~/follow_joint_trajectory", this);
  roll_trajectory_action_ = std::make_shared<TrajectoryActionServer>(
      get_node(), joint_names, "~/follow_roll_joint_trajectory", this);

  odom_trajectory_subscriber_->set_target_control(odom_trajectory_control_);
  roll_trajectory_subscriber_->set_target_control(roll_trajectory_control_);
  odom_trajectory_action_->set_target_control(odom_trajectory_control_);
  roll_trajectory_action_->set_target_control(roll_trajectory_control_);

  // Set the publisher for internal joint states
  base_state_publisher_ = std::make_shared<StatePublisher>(
      get_node(), "~/state", base_coordinate_names);
  base_controller_state_publisher_ = std::make_shared<ControllerStatePublisher>(
      get_node(), "~/controller_state", base_coordinate_names);
  joint_state_publisher_ = std::make_shared<StatePublisher>(
      get_node(), "~/internal_state", joint_controller_->joint_names());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniBaseController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  if (!joint_controller_->Activate(command_interfaces_, state_interfaces_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  // Activate here to match JointTrajectoryController
  velocity_control_->Activate();
  odom_trajectory_control_->Activate();
  roll_trajectory_control_->Activate();

  const auto current_time = get_node()->get_clock()->now();
  wheel_odometry_->set_last_odometry_published_time(current_time);
  wheel_odometry_->set_last_transform_published_time(current_time);
  joint_state_publisher_->set_last_state_published_time(current_time);
  base_state_publisher_->set_last_state_published_time(current_time);
  base_controller_state_publisher_->set_last_state_published_time(current_time);

  // Initialize odometry at startup
  base_odometry_->InitOdometry();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniBaseController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  // Reset the current action goal when shutting down the controller
  odom_trajectory_action_->PreemptActiveGoal();
  roll_trajectory_action_->PreemptActiveGoal();

  // Clear command velocity
  const auto zero_velocity = std::make_shared<geometry_msgs::msg::Twist>();
  velocity_control_->UpdateCommandVelocity(zero_velocity);
  return controller_interface::CallbackReturn::SUCCESS;
}

bool OmniBaseController::IsAcceptable() {
  return get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

void OmniBaseController::PreemptActiveGoal() {
  odom_trajectory_action_->PreemptActiveGoal();
  roll_trajectory_action_->PreemptActiveGoal();
}

void OmniBaseController::UpdateVelocity(const geometry_msgs::msg::Twist::SharedPtr& msg) {
  velocity_control_->UpdateCommandVelocity(msg);
}

bool OmniBaseController::ValidateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
  if (trajectory.joint_names.size() == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory has no joint names");
    return false;
  } else if (trajectory.joint_names[0] == base_roll_joint_name_) {
    return roll_trajectory_control_->ValidateTrajectory(trajectory);
  } else if (trajectory.joint_names.size() == kNumBaseCoordinateIDs) {
    return odom_trajectory_control_->ValidateTrajectory(trajectory);
  }
  RCLCPP_ERROR(get_node()->get_logger(), "Trajectory's joint_name mismatch.");
  return false;
}

void OmniBaseController::UpdateTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& trajectory) {
  if (trajectory->joint_names[0] == base_roll_joint_name_) {
    roll_trajectory_control_->AcceptTrajectory(trajectory, base_odometry_->odometry());
  } else if (trajectory->joint_names.size() == kNumBaseCoordinateIDs) {
    odom_trajectory_control_->AcceptTrajectory(trajectory, base_odometry_->odometry());
  }
}

void OmniBaseController::ResetTrajectory(const std::shared_ptr<OmniBaseTrajectoryControl>& no_reset_control) {
  if (no_reset_control != odom_trajectory_control_) {
    odom_trajectory_control_->ResetCurrentTrajectory();
  }
  if (no_reset_control != roll_trajectory_control_) {
    roll_trajectory_control_->ResetCurrentTrajectory();
  }
}

}  // namespace hsrb_base_controllers

PLUGINLIB_EXPORT_CLASS(hsrb_base_controllers::OmniBaseController,
                       controller_interface::ControllerInterface);
