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
// Copyright (C) 2022 Toyeta Motor Corporation. All rights reserved.

#include "servo_diagnostic_broadcaster.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace {

const char* const kConnectionErrorRate = "connection_error_rate";
const char* const kTemperature = "temperature";
const char* const kCurrent = "current";
const char* const kCurrentDriveMode = "current_drive_mode";
const char* const kMotorId = "motor_id";
const char* const kWarningStatus = "warning_status";
const char* const kErrorStatus = "error_status";
const char* const kSafetyAlarmStatus = "safety_alarm_status";

const std::vector<std::pair<std::string, std::string>> kInterfaceAndDiagnosticKeys = {
    {kConnectionErrorRate, "Connection Error Rate"},
    {hardware_interface::HW_IF_POSITION, "Present Position"},
    {hardware_interface::HW_IF_VELOCITY, "Present Velocity"},
    {hardware_interface::HW_IF_EFFORT, "Present Effort"},
    {kTemperature, "Present Temperature"},
    {kCurrent, "Present Current"},
    {kCurrentDriveMode, "Driver Type"}
};
constexpr double kConnectionErrorRateTolerance = 0.3;

bool IsInterfaceIncluded(const std::string& joint_name, const std::string& interface_name,
                         const std::vector<hardware_interface::LoanedStateInterface>& interfaces) {
  for (const auto& interface : interfaces) {
    if (interface.get_prefix_name() == joint_name && interface.get_interface_name() == interface_name) {
      return true;
    }
  }
  return false;
}

void UpdateKeyValue(const std::map<std::string, double>& key_value_map,
                    diagnostic_msgs::msg::DiagnosticStatus& status_out) {
  // Use kInterfaceAndDiagnosticKeys to align the key value order
  for (const auto& interface_diagnostic_key : kInterfaceAndDiagnosticKeys) {
    diagnostic_msgs::msg::KeyValue key_value_msg;
    key_value_msg.key = interface_diagnostic_key.second;
    key_value_msg.value = std::to_string(key_value_map.at(interface_diagnostic_key.first));
    status_out.values.push_back(key_value_msg);
  }
}

double ExtractConnectionErrorRate(const std::map<std::string, double>& key_value_map) {
  const auto it = key_value_map.find(kConnectionErrorRate);
  if (it != key_value_map.end()) {
    return it->second;
  } else {
    return 0.0;
  }
}

void UpdateLevelAndMessage(const std::map<std::string, double>& key_value_map,
                           diagnostic_msgs::msg::DiagnosticStatus& status_out) {
  const auto connection_error_rate = ExtractConnectionErrorRate(key_value_map);
  if (connection_error_rate > kConnectionErrorRateTolerance) {
    status_out.level = std::max(status_out.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    status_out.message.append("Connection error rate high; ");
  } else if (connection_error_rate > 0.0) {
    status_out.level = std::max(status_out.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    status_out.message.append("Connection error occurred; ");
  }
}

}  // namespace


namespace hsrb_diagnostic_broadcaster {

controller_interface::InterfaceConfiguration ServoDiagnosticBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ServoDiagnosticBroadcaster::state_interface_configuration() const {
  if (joints_.empty()) {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::ALL};
  } else {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& name : joints_) {
      config.names.push_back(name + "/" + kConnectionErrorRate);
      config.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
      config.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
      config.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
      config.names.push_back(name + "/" + kTemperature);
      config.names.push_back(name + "/" + kCurrent);
      config.names.push_back(name + "/" + kCurrentDriveMode);
      config.names.push_back(name + "/" + kMotorId);
      config.names.push_back(name + "/" + kWarningStatus);
      config.names.push_back(name + "/" + kErrorStatus);
      config.names.push_back(name + "/" + kSafetyAlarmStatus);
    }
    return config;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ServoDiagnosticBroadcaster::on_init() {
  auto_declare<std::vector<std::string>>("joints", {});
  joints_ = get_node()->get_parameter("joints").as_string_array();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ServoDiagnosticBroadcaster::on_configure(const rclcpp_lifecycle::State& previous_state) {
  try {
    diagnostics_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<RealtimePublisher>(diagnostics_publisher_);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), e.what());
    return CallbackReturn::ERROR;
  }

  const double publish_rate = auto_declare("publish_rate", 1.0);
  if (publish_rate > std::numeric_limits<double>::epsilon()) {
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);
  } else {
    publish_period_ = boost::none;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ServoDiagnosticBroadcaster::on_activate(const rclcpp_lifecycle::State& previous_state) {
  std::vector<std::string> joint_names;
  for (const auto& state_interface : state_interfaces_) {
    joint_names.push_back(state_interface.get_prefix_name());
  }
  std::sort(joint_names.begin(), joint_names.end());
  joint_names.erase(std::unique(joint_names.begin(), joint_names.end()), joint_names.end());

  for (const auto& joint_name : joint_names) {
    // motor id is required
    for (const auto& state_interface : state_interfaces_) {
      if ((state_interface.get_prefix_name() == joint_name) && (state_interface.get_interface_name() == kMotorId)) {
        joint_name_motor_id_map_[joint_name] = std::to_string(static_cast<int>(state_interface.get_value()));
      }
    }
    if (joint_name_motor_id_map_.find(joint_name) == joint_name_motor_id_map_.end()) {
      continue;
    }

    joint_key_value_map_[joint_name] = std::map<std::string, double>();
    for (const auto& interface_diagnostic_key : kInterfaceAndDiagnosticKeys) {
      joint_key_value_map_[joint_name][interface_diagnostic_key.first] = 0.0;
      if (!IsInterfaceIncluded(joint_name, interface_diagnostic_key.first, state_interfaces_)) {
        RCLCPP_WARN_STREAM(get_node()->get_logger(),
                           "State interface " + joint_name + "/" + interface_diagnostic_key.first + " is not found.");
      }
    }

    warning_status_[joint_name] = std::make_shared<AlarmStatus>(
        tmc_exxx_servo_motor_protocol::ExxxWarningCategory(), diagnostic_msgs::msg::DiagnosticStatus::WARN);
    error_status_[joint_name] = std::make_shared<AlarmStatus>(
        tmc_exxx_servo_motor_protocol::ExxxErrorCategory(), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    safety_alarm_status_[joint_name] = std::make_shared<AlarmStatus>(
        tmc_exxx_servo_motor_protocol::ExxxSafetyErrorCategory(), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  }

  next_publish_time_ = get_node()->now();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ServoDiagnosticBroadcaster::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  joint_key_value_map_.clear();
  joint_name_motor_id_map_.clear();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ServoDiagnosticBroadcaster::update(const rclcpp::Time& time,
                                                                     const rclcpp::Duration& period) {
  for (const auto& state_interface : state_interfaces_) {
    if (joint_key_value_map_.find(state_interface.get_prefix_name()) == joint_key_value_map_.end()) {
      continue;
    }
    auto& key_value_map = joint_key_value_map_[state_interface.get_prefix_name()];
    if (key_value_map.find(state_interface.get_interface_name()) != key_value_map.end()) {
      key_value_map[state_interface.get_interface_name()] = state_interface.get_value();
    } else if (state_interface.get_interface_name() == kWarningStatus) {
      warning_status_[state_interface.get_prefix_name()]->Update(static_cast<int>(state_interface.get_value()));
    } else if (state_interface.get_interface_name() == kErrorStatus) {
      error_status_[state_interface.get_prefix_name()]->Update(static_cast<int>(state_interface.get_value()));
    } else if (state_interface.get_interface_name() == kSafetyAlarmStatus) {
      safety_alarm_status_[state_interface.get_prefix_name()]->Update(static_cast<int>(state_interface.get_value()));
    }
  }

  if (publish_period_ && get_node()->now() < next_publish_time_) {
    return controller_interface::return_type::OK;
  }

  if (realtime_publisher_ && realtime_publisher_->trylock()) {
    realtime_publisher_->msg_.header.stamp = get_node()->now();
    realtime_publisher_->msg_.status.clear();
    for (const auto& joint_key_value : joint_key_value_map_) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

      status.name = joint_key_value.first;
      status.hardware_id = joint_name_motor_id_map_[joint_key_value.first];
      UpdateKeyValue(joint_key_value.second, status);
      UpdateLevelAndMessage(joint_key_value.second, status);
      warning_status_[joint_key_value.first]->Publish(status);
      error_status_[joint_key_value.first]->Publish(status);
      safety_alarm_status_[joint_key_value.first]->Publish(status);
      if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
        status.message = "OK";
      }
      realtime_publisher_->msg_.status.push_back(status);
    }
    realtime_publisher_->unlockAndPublish();
  }

  if (publish_period_) {
    next_publish_time_ += publish_period_.value();
  }

  return controller_interface::return_type::OK;
}

}  // namespace hsrb_diagnostic_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hsrb_diagnostic_broadcaster::ServoDiagnosticBroadcaster,
                       controller_interface::ControllerInterface)
