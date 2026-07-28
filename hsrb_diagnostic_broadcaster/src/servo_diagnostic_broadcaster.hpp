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
#ifndef HSRB_DIAGNOSTIC_BROADCATER_SERVO_DIAGNOSTIC_BROADCATER_HPP_
#define HSRB_DIAGNOSTIC_BROADCATER_SERVO_DIAGNOSTIC_BROADCATER_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <boost/optional/optional.hpp>
#include <boost/system/error_code.hpp>

#include <controller_interface/controller_interface.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <tmc_exxx_servo_motor_protocol/exxx_error_category.hpp>
#include <tmc_exxx_servo_motor_protocol/exxx_warning_category.hpp>

namespace hsrb_diagnostic_broadcaster {

class ServoDiagnosticBroadcaster : public controller_interface::ControllerInterface {
 public:
  ServoDiagnosticBroadcaster() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  using RealtimePublisher = realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>;
  std::unique_ptr<RealtimePublisher> realtime_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

  std::vector<std::string> joints_;

  std::map<std::string, std::map<std::string, double>> joint_key_value_map_;
  std::map<std::string, std::string> joint_name_motor_id_map_;

  rclcpp::Time next_publish_time_;
  boost::optional<rclcpp::Duration> publish_period_;

  class AlarmStatus {
   public:
    using Ptr = std::shared_ptr<AlarmStatus>;

    AlarmStatus(const boost::system::error_category& category, int level)
        : category_(category), level_(level) { Reset(); }
    ~AlarmStatus() = default;

    void Update(int new_value) { value_ |= new_value; }
    void Publish(diagnostic_msgs::msg::DiagnosticStatus& status_out) {
      if (value_ != 0) {
        status_out.level = std::max<int>(status_out.level, level_);
        status_out.message.append(boost::system::error_code(value_, category_).message());
      }
      Reset();
    }
    void Reset() { value_ = 0; }

   private:
    int level_;
    const boost::system::error_category& category_;
    int value_;
  };
  std::map<std::string, AlarmStatus::Ptr> warning_status_;
  std::map<std::string, AlarmStatus::Ptr> error_status_;
  std::map<std::string, AlarmStatus::Ptr> safety_alarm_status_;
};

}  // namespace hsrb_diagnostic_broadcaster
#endif  // HSRB_DIAGNOSTIC_BROADCATER_SERVO_DIAGNOSTIC_BROADCATER_HPP_
