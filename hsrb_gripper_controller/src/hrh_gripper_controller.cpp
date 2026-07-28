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
#include <string>
#include <vector>
#include <boost/range/adaptor/indexed.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hsrb_gripper_controller/hrh_gripper_action.hpp>
#include <hsrb_gripper_controller/hrh_gripper_apply_force_action.hpp>
#include <hsrb_gripper_controller/hrh_gripper_follow_distance_trajectory_action.hpp>
#include <hsrb_gripper_controller/hrh_gripper_follow_trajectory_action.hpp>
#include <hsrb_gripper_controller/hrh_gripper_grasp_action.hpp>
#include <hsrb_gripper_controller/hrh_gripper_set_distance_action.hpp>

#include "hsrb_gripper_controller/hrh_gripper_controller.hpp"

namespace {

template <typename TYPE>
bool GetIndex(const std::vector<TYPE>& interfaces, const std::string& name, uint32_t& index_out_) {
  for (const auto& interface : interfaces | boost::adaptors::indexed()) {
    if (interface.value().get_interface_name() == name) {
      index_out_ = interface.index();
      return true;
    }
  }
  return false;
}

template <typename TYPE>
bool GetPositionIndex(const std::vector<TYPE>& interfaces, const std::string& joint_name, uint32_t& index_out_) {
  for (const auto& interface : interfaces | boost::adaptors::indexed()) {
    if (interface.value().get_prefix_name() == joint_name &&
        interface.value().get_interface_name() == hardware_interface::HW_IF_POSITION) {
      index_out_ = interface.index();
      return true;
    }
  }
  return false;
}

}  // unnamed namespace

namespace hsrb_gripper_controller {

HrhGripperController::HrhGripperController() {}

bool HrhGripperController::InitImpl() {
  std::vector<std::string> joint_names =
      GetParameter(get_node(), "joints", std::vector<std::string>({ "hand_motor_joint" }));
  if (joint_names.size() != 1) {
    //  RCLCPP_FATAL_STREAM(node_->get_logger(), "The size of joints must be one.");
    return false;
  }
  joint_name_ = joint_names.at(0);

  left_spring_joint_ = GetParameter(get_node(), "left_spring_joint", "hand_l_spring_proximal_joint");
  right_spring_joint_ = GetParameter(get_node(), "right_spring_joint", "hand_r_spring_proximal_joint");
  gripper_namespace_ = GetParameter(get_node(),  "namespace", "~");
  return true;
}

controller_interface::InterfaceConfiguration HrhGripperController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  conf.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  conf.names.push_back(joint_name_ + "/command_drive_mode");
  conf.names.push_back(joint_name_ + "/command_grasping_flag");
  return conf;
}

controller_interface::InterfaceConfiguration HrhGripperController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  conf.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  conf.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  conf.names.push_back(joint_name_ + "/current_drive_mode");
  conf.names.push_back(joint_name_ + "/current_grasping_flag");
  conf.names.push_back(joint_name_ + "/current");
  conf.names.push_back(left_spring_joint_ + "/" + hardware_interface::HW_IF_POSITION);
  conf.names.push_back(right_spring_joint_ + "/" + hardware_interface::HW_IF_POSITION);
  return conf;
}

controller_interface::return_type HrhGripperController::update(const rclcpp::Time& time,
                                                               const rclcpp::Duration& period) {
  trajectory_msgs::msg::JointTrajectoryPoint reference;
  trajectory_msgs::msg::JointTrajectoryPoint feedback;

  if (active_action_) {
    active_action_->Update(get_node()->get_clock()->now());

    // Retrieve state data from the currently running action
    reference = active_action_->GetReferenceState();
    feedback = active_action_->GetFeedbackState();
  } else {
    // If there is no running action, set the state data to the current value
    feedback.positions = { GetCurrentPosition() };
    feedback.velocities = { GetCurrentVelocity() };
    feedback.effort = { GetCurrentTorque() };
    reference = feedback;
  }
  int32_t command_mode = *(command_control_mode_.readFromRT());
  static_cast<void>(command_interfaces_[command_drive_mode_index_].set_value(static_cast<double>(command_mode)));

  // Publish the state
  gripper_state_publisher_->Publish(reference, feedback, time);

  // Publish the distance between fingertips
  gripper_distance_publisher_->Publish(GetCurrentPosition(), time);

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HrhGripperController::on_init() {
  if (InitImpl()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } else {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HrhGripperController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  // Initialize each action
  actions_.push_back(std::make_shared<HrhGripperFollowTrajectoryAction>(this));
  actions_.push_back(std::make_shared<HrhGripperGraspAction>(this));
  actions_.push_back(std::make_shared<HrhGripperApplyForceAction>(this));
  actions_.push_back(std::make_shared<HrhGripperSetDistanceAction>(this));
  actions_.push_back(std::make_shared<HrhGripperFollowDistanceTrajectoryAction>(this));
  for (auto& action : actions_) {
    if (!action->Init(get_node())) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  // Set the state publisher
  std::string controller_state_name = gripper_namespace_ + "/controller_state";
  gripper_state_publisher_ = std::make_shared<StatePublisher>(get_node(), controller_state_name, joint_name_);

  // Set the fingertip distance publisher
  std::string fingertip_distance_name = gripper_namespace_ + "/fingertip_distance";
  gripper_distance_publisher_ = std::make_shared<DistancePublisher>(get_node(), fingertip_distance_name);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HrhGripperController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  // Since the order is assign_interfaces => activate, it is already usable
  for (const auto& state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == "current_drive_mode") {
      command_control_mode_.initRT(static_cast<int32_t>(state_interface.get_value()));
      break;
    }
  }

  if (!GetPositionIndex(state_interfaces_, joint_name_, current_position_index_) ||
      !GetIndex(state_interfaces_, hardware_interface::HW_IF_VELOCITY, current_velocity_index_) ||
      !GetIndex(state_interfaces_, hardware_interface::HW_IF_EFFORT, current_effort_index_) ||
      !GetIndex(state_interfaces_, "current_grasping_flag", current_grasping_flag_index_) ||
      !GetIndex(state_interfaces_, "current", current_index_) ||
      !GetPositionIndex(state_interfaces_, left_spring_joint_, current_left_spring_index_) ||
      !GetPositionIndex(state_interfaces_, right_spring_joint_, current_right_spring_index_) ||
      !GetIndex(command_interfaces_, hardware_interface::HW_IF_POSITION, command_position_index_) ||
      !GetIndex(command_interfaces_, hardware_interface::HW_IF_EFFORT, command_effort_index_) ||
      !GetIndex(command_interfaces_, "command_grasping_flag", command_grasping_flag_index_) ||
      !GetIndex(command_interfaces_, "command_drive_mode", command_drive_mode_index_)) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  for (auto& action : actions_) {
    if (!action->Activate()) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  gripper_state_publisher_->SetLastStatePublishedTime(get_node()->get_clock()->now());
  gripper_distance_publisher_->SetLastStatePublishedTime(get_node()->get_clock()->now());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HrhGripperController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool HrhGripperController::IsAcceptable() {
  return get_node()->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

void HrhGripperController::PreemptActiveGoal() {
  for (auto& action : actions_) {
    action->PreemptActiveGoal();
  }
}

double HrhGripperController::GetCurrentPosition() const {
  return state_interfaces_[current_position_index_].get_value();
}

double HrhGripperController::GetCurrentVelocity() const {
  return state_interfaces_[current_velocity_index_].get_value();
}

double HrhGripperController::GetCurrentTorque() const { return state_interfaces_[current_effort_index_].get_value(); }

bool HrhGripperController::GetCurrentGraspingFlag() const {
  return state_interfaces_[current_grasping_flag_index_].get_value() > 0.0;
}

double HrhGripperController::GetLeftSpringPosition() const {
  return state_interfaces_[current_left_spring_index_].get_value();
}

double HrhGripperController::GetRightSpringPosition() const {
  return state_interfaces_[current_right_spring_index_].get_value();
}

double HrhGripperController::GetCurrent() const { return state_interfaces_[current_index_].get_value(); }

void HrhGripperController::SetComandPosition(double position) {
  static_cast<void>(command_interfaces_[command_position_index_].set_value(position));
}

void HrhGripperController::SetGraspCommand(bool grasping_flag, double effort) {
  static_cast<void>(command_interfaces_[command_effort_index_].set_value(effort));
  if (grasping_flag) {
    static_cast<void>(command_interfaces_[command_grasping_flag_index_].set_value(1.0));
  } else {
    static_cast<void>(command_interfaces_[command_grasping_flag_index_].set_value(-1.0));
  }
}

void HrhGripperController::ChangeControlMode(IHrhGripperAction::Ptr action) {
  command_control_mode_.writeFromNonRT(action->target_mode());
  active_action_ = action;
}

}  // namespace hsrb_gripper_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hsrb_gripper_controller::HrhGripperController, controller_interface::ControllerInterface)
