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

#include <chrono>

#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>

#include "../src/servo_diagnostic_broadcaster.hpp"

namespace {
const char* const kControllerNodeName = "controller_manager";
const char* const kClientNodeName = "test_node";

class Handle {
 public:
  using Ptr = std::shared_ptr<Handle>;

  Handle(const std::string& joint_name, const std::string& interface_name)
      : value_(0.0), state_interface_(joint_name, interface_name, &value_) {}
  virtual ~Handle() = default;

  hardware_interface::LoanedStateInterface state_interface() {
    return hardware_interface::LoanedStateInterface(state_interface_);
  }
  void set_value(double x) { value_ = x; }

 private:
  double value_;
  hardware_interface::StateInterface state_interface_;
};

struct JointStub {
  using Ptr = std::shared_ptr<JointStub>;

  Handle::Ptr motor_id;
  Handle::Ptr connection_error_rate;
  Handle::Ptr warning_status;
  Handle::Ptr error_status;
  Handle::Ptr safety_alarm_status;
  Handle::Ptr position;
  Handle::Ptr velocity;
  Handle::Ptr effort;
  Handle::Ptr temperature;
  Handle::Ptr current;
  Handle::Ptr drive_mode;

  explicit JointStub(const std::string& joint_name) {
    motor_id = std::make_shared<Handle>(joint_name, "motor_id");
    connection_error_rate = std::make_shared<Handle>(joint_name, "connection_error_rate");
    warning_status = std::make_shared<Handle>(joint_name, "warning_status");
    error_status = std::make_shared<Handle>(joint_name, "error_status");
    safety_alarm_status = std::make_shared<Handle>(joint_name, "safety_alarm_status");
    position = std::make_shared<Handle>(joint_name, hardware_interface::HW_IF_POSITION);
    velocity = std::make_shared<Handle>(joint_name, hardware_interface::HW_IF_VELOCITY);
    effort = std::make_shared<Handle>(joint_name, hardware_interface::HW_IF_EFFORT);
    temperature = std::make_shared<Handle>(joint_name, "temperature");
    current = std::make_shared<Handle>(joint_name, "current");
    drive_mode = std::make_shared<Handle>(joint_name, "current_drive_mode");
  }

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces(bool skip_motor_id = false) {
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    if (!skip_motor_id) state_interfaces.emplace_back(motor_id->state_interface());
    state_interfaces.emplace_back(connection_error_rate->state_interface());
    state_interfaces.emplace_back(warning_status->state_interface());
    state_interfaces.emplace_back(error_status->state_interface());
    state_interfaces.emplace_back(safety_alarm_status->state_interface());
    state_interfaces.emplace_back(position->state_interface());
    state_interfaces.emplace_back(velocity->state_interface());
    state_interfaces.emplace_back(effort->state_interface());
    state_interfaces.emplace_back(temperature->state_interface());
    state_interfaces.emplace_back(current->state_interface());
    state_interfaces.emplace_back(drive_mode->state_interface());
    return state_interfaces;
  }
};

struct HardwareStub {
  using Ptr = std::shared_ptr<HardwareStub>;

  JointStub::Ptr invalid_joint;
  JointStub::Ptr valid_joint_1;
  JointStub::Ptr valid_joint_2;

  HardwareStub() {
    invalid_joint = std::make_shared<JointStub>("invalid_joint");
    invalid_joint->motor_id->set_value(3.0);

    valid_joint_1 = std::make_shared<JointStub>("valid_joint_1");
    valid_joint_1->connection_error_rate->set_value(0.0);
    valid_joint_1->warning_status->set_value(0.0);
    valid_joint_1->error_status->set_value(0.0);
    valid_joint_1->safety_alarm_status->set_value(0.0);
    valid_joint_1->motor_id->set_value(1.0);
    valid_joint_1->position->set_value(11.0);
    valid_joint_1->velocity->set_value(21.0);
    valid_joint_1->effort->set_value(31.0);
    valid_joint_1->temperature->set_value(41.0);
    valid_joint_1->current->set_value(51.0);
    valid_joint_1->drive_mode->set_value(61.0);

    valid_joint_2 = std::make_shared<JointStub>("valid_joint_2");
    valid_joint_2->connection_error_rate->set_value(0.0);
    valid_joint_2->warning_status->set_value(0.0);
    valid_joint_2->error_status->set_value(0.0);
    valid_joint_2->safety_alarm_status->set_value(0.0);
    valid_joint_2->motor_id->set_value(2.0);
    valid_joint_2->position->set_value(12.0);
    valid_joint_2->velocity->set_value(22.0);
    valid_joint_2->effort->set_value(32.0);
    valid_joint_2->temperature->set_value(42.0);
    valid_joint_2->current->set_value(52.0);
    valid_joint_2->drive_mode->set_value(62.0);
  }

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces() {
    std::vector<std::pair<JointStub::Ptr, bool>> joints = {
        {invalid_joint, true}, {valid_joint_1, false}, {valid_joint_2, false}};

    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    for (const auto& joint_and_skip_flag : joints) {
      auto joint_interfaces = joint_and_skip_flag.first->state_interfaces(joint_and_skip_flag.second);
      std::move(joint_interfaces.begin(), joint_interfaces.end(), std::back_inserter(state_interfaces));
    }
    // Remove last inteface for testing with interface not configured
    state_interfaces.pop_back();
    return state_interfaces;
  }
};

class DiagnosticsSubscriber {
 public:
  using Ptr = std::shared_ptr<DiagnosticsSubscriber>;

  explicit DiagnosticsSubscriber(const rclcpp::Node::SharedPtr& node) : count_(0) {
    subscriber_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 1, std::bind(&DiagnosticsSubscriber::Callback, this, std::placeholders::_1));
  }

  void Reset() { count_ = 0; }

  diagnostic_msgs::msg::DiagnosticArray last_msg() const { return last_msg_; }
  uint32_t count() const { return count_; }

 private:
  void Callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    last_msg_ = *msg;
    ++count_;
  }
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscriber_;
  diagnostic_msgs::msg::DiagnosticArray last_msg_;
  uint32_t count_;
};

diagnostic_msgs::msg::DiagnosticStatus ExtractStatus(const diagnostic_msgs::msg::DiagnosticArray& array,
                                                     const std::string& joint_name) {
  for (const auto& status : array.status) {
    if (status.name == joint_name) {
      return status;
    }
  }
  return diagnostic_msgs::msg::DiagnosticStatus();
}

bool HasJoint(const diagnostic_msgs::msg::DiagnosticArray& array,
              const std::string& joint_name) {
  // In this test, status must have key-value
  return !ExtractStatus(array, joint_name).values.empty();
}

uint8_t ExtractLevel(const diagnostic_msgs::msg::DiagnosticArray& array,
                     const std::string& joint_name) {
  return ExtractStatus(array, joint_name).level;
}

std::string ExtractMessage(const diagnostic_msgs::msg::DiagnosticArray& array,
                           const std::string& joint_name) {
  return ExtractStatus(array, joint_name).message;
}

bool IsMessageIncluded(const diagnostic_msgs::msg::DiagnosticArray& array,
                       const std::string& joint_name,
                       const std::string& target_message) {
  const auto received_message = ExtractMessage(array, joint_name);
  const auto pos = received_message.find(target_message);
  return pos != std::string::npos;
}

std::string ExtractHardwareId(const diagnostic_msgs::msg::DiagnosticArray& array,
                              const std::string& joint_name) {
  return ExtractStatus(array, joint_name).hardware_id;
}

std::string ExtractValue(const diagnostic_msgs::msg::DiagnosticArray& array,
                         const std::string& joint_name,
                         const std::string& key) {
  auto status = ExtractStatus(array, joint_name);
  for (const auto& key_value : status.values) {
    if (key_value.key == key) {
      return key_value.value;
    }
  }
  return "";
}

}  // namespace

namespace hsrb_diagnostic_broadcaster {

class ServoDiagnosticBroadcasterTestBase : public ::testing::Test {
 protected:
  std::shared_ptr<ServoDiagnosticBroadcaster> controller_;

  HardwareStub::Ptr hardware_;

  rclcpp::Node::SharedPtr client_node_;
  DiagnosticsSubscriber::Ptr subscriber_;

  void Spin();
  void WaitForDiagnostics();
};

void ServoDiagnosticBroadcasterTestBase::Spin() {
  controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.1));
  rclcpp::spin_some(client_node_);
  ASSERT_EQ(controller_->update(controller_->get_node()->now(),
                                rclcpp::Duration::from_seconds(0.1)), controller_interface::return_type::OK);
}

void ServoDiagnosticBroadcasterTestBase::WaitForDiagnostics() {
  rclcpp::WallRate rate(100.0);
  auto timeout = client_node_->now() + rclcpp::Duration(3, 0);
  subscriber_->Reset();
  while (rclcpp::ok() && subscriber_->count() == 0) {
    if (client_node_->now() > timeout) {
      FAIL();
    }
    rate.sleep();
    Spin();
  }
}

class ServoDiagnosticBroadcasterTest : public ServoDiagnosticBroadcasterTestBase {
 protected:
  void SetUp(const double publish_rate = 10.0);
};

void ServoDiagnosticBroadcasterTest::SetUp(const double publish_rate) {
  ServoDiagnosticBroadcasterTestBase::SetUp();
  controller_ = std::make_shared<ServoDiagnosticBroadcaster>();
  rclcpp::NodeOptions options = rclcpp::NodeOptions();
  EXPECT_EQ(controller_->init(kControllerNodeName, "", 0, "", options), controller_interface::return_type::OK);

  controller_->get_node()->declare_parameter("publish_rate", publish_rate);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  hardware_ = std::make_shared<HardwareStub>();
  controller_->assign_interfaces({}, std::move(hardware_->state_interfaces()));

  EXPECT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  client_node_ = rclcpp::Node::make_shared(kClientNodeName);
  subscriber_ = std::make_shared<DiagnosticsSubscriber>(client_node_);
}

TEST_F(ServoDiagnosticBroadcasterTest, DiagnosticsStatus) {
  SetUp();
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  EXPECT_FALSE(HasJoint(msg, "invalid_joint"));

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(ExtractMessage(msg, "valid_joint_1"), "OK");
  EXPECT_EQ(ExtractHardwareId(msg, "valid_joint_1"), "1");
  EXPECT_EQ(ExtractValue(msg, "valid_joint_1", "Connection Error Rate"), std::to_string(0.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_1", "Present Position"), std::to_string(11.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_1", "Present Velocity"), std::to_string(21.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_1", "Present Effort"), std::to_string(31.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_1", "Present Temperature"), std::to_string(41.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_1", "Present Current"), std::to_string(51.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_1", "Driver Type"), std::to_string(61.0));

  ASSERT_TRUE(HasJoint(msg, "valid_joint_2"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_2"), diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(ExtractMessage(msg, "valid_joint_2"), "OK");
  EXPECT_EQ(ExtractHardwareId(msg, "valid_joint_2"), "2");
  EXPECT_EQ(ExtractValue(msg, "valid_joint_2", "Connection Error Rate"), std::to_string(0.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_2", "Present Position"), std::to_string(12.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_2", "Present Velocity"), std::to_string(22.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_2", "Present Effort"), std::to_string(32.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_2", "Present Temperature"), std::to_string(42.0));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_2", "Present Current"), std::to_string(52.0));
  // The interface is not configured. The value is initial value 0.0
  EXPECT_EQ(ExtractValue(msg, "valid_joint_2", "Driver Type"), std::to_string(0.0));
}

TEST_F(ServoDiagnosticBroadcasterTest, ValidPublishRate) {
  SetUp(3.0);

  rclcpp::WallRate rate(100.0);
  auto timeout = client_node_->now() + rclcpp::Duration(1, 0);
  while (rclcpp::ok() && client_node_->now() < timeout) {
    rate.sleep();
    Spin();
  }

  EXPECT_GE(subscriber_->count(), 3);
  EXPECT_LE(subscriber_->count(), 4);
}

TEST_F(ServoDiagnosticBroadcasterTest, PublishRateZero) {
  SetUp(0.0);

  const uint32_t update_count = 10;
  rclcpp::WallRate rate(100.0);
  for (int i = 0; i < update_count; ++i) {
    rate.sleep();
    Spin();
  }

  // RealtimePublisher does not guarantee immediate publish
  EXPECT_GE(subscriber_->count(), update_count - 1);
  EXPECT_LE(subscriber_->count(), update_count);
}

TEST_F(ServoDiagnosticBroadcasterTest, ConnectionErrorRateWarning) {
  SetUp();
  hardware_->valid_joint_1->connection_error_rate->set_value(0.01);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Connection error occurred"));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_1", "Connection Error Rate"), std::to_string(0.01));
}

TEST_F(ServoDiagnosticBroadcasterTest, ConnectionErrorRateError) {
  SetUp();
  hardware_->valid_joint_1->connection_error_rate->set_value(0.31);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Connection error rate high"));
  EXPECT_EQ(ExtractValue(msg, "valid_joint_1", "Connection Error Rate"), std::to_string(0.31));
}

TEST_F(ServoDiagnosticBroadcasterTest, WarningStatus) {
  SetUp(10.0);
  hardware_->valid_joint_1->warning_status->set_value(2.0);
  WaitForDiagnostics();

  auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::WARN);
  // See hsrb_drivers/tmc_exxx_servo_motor_protocol/src/tmc_exxx_servo_motor_protocol/exxx_warning_category.cpp
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Velocity command interrupted."));

  hardware_->valid_joint_1->warning_status->set_value(4.0);
  ASSERT_EQ(controller_->update(controller_->get_node()->now(),
            rclcpp::Duration::from_seconds(0.1)), controller_interface::return_type::OK);
  hardware_->valid_joint_1->warning_status->set_value(8.0);
  ASSERT_EQ(controller_->update(controller_->get_node()->now(),
            rclcpp::Duration::from_seconds(0.1)), controller_interface::return_type::OK);
  hardware_->valid_joint_1->warning_status->set_value(0.0);
  WaitForDiagnostics();

  msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Velocity reference out of range."));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Current command interrupted."));

  WaitForDiagnostics();

  msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(ExtractMessage(msg, "valid_joint_1"), "OK");
}

TEST_F(ServoDiagnosticBroadcasterTest, ErrorStatus) {
  SetUp(10.0);
  hardware_->valid_joint_1->error_status->set_value(2.0);
  WaitForDiagnostics();

  auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  // See hsrb_drivers/tmc_exxx_servo_motor_protocol/src/tmc_exxx_servo_motor_protocol/exxx_error_category.cpp
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Supplied voltage out of range."));

  hardware_->valid_joint_1->error_status->set_value(4.0);
  ASSERT_EQ(controller_->update(controller_->get_node()->now(),
                                rclcpp::Duration::from_seconds(0.1)), controller_interface::return_type::OK);
  hardware_->valid_joint_1->error_status->set_value(8.0);
  ASSERT_EQ(controller_->update(controller_->get_node()->now(),
                                rclcpp::Duration::from_seconds(0.1)), controller_interface::return_type::OK);
  hardware_->valid_joint_1->error_status->set_value(0.0);
  WaitForDiagnostics();

  msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Abnormal position."));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Over current."));

  WaitForDiagnostics();

  msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(ExtractMessage(msg, "valid_joint_1"), "OK");
}

TEST_F(ServoDiagnosticBroadcasterTest, SafetyAlarmStatus) {
  SetUp(10.0);
  hardware_->valid_joint_1->safety_alarm_status->set_value(2.0);
  WaitForDiagnostics();

  auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  // See hsrb_drivers/tmc_exxx_servo_motor_protocol/src/tmc_exxx_servo_motor_protocol/exxx_error_category.cpp
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Short error."));

  hardware_->valid_joint_1->safety_alarm_status->set_value(4.0);
  ASSERT_EQ(controller_->update(controller_->get_node()->now(),
                                rclcpp::Duration::from_seconds(0.1)), controller_interface::return_type::OK);
  hardware_->valid_joint_1->safety_alarm_status->set_value(16.0);
  ASSERT_EQ(controller_->update(controller_->get_node()->now(),
                                rclcpp::Duration::from_seconds(0.1)), controller_interface::return_type::OK);
  hardware_->valid_joint_1->safety_alarm_status->set_value(0.0);
  WaitForDiagnostics();

  msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Open error."));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Slow monitoring error."));

  WaitForDiagnostics();

  msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(ExtractMessage(msg, "valid_joint_1"), "OK");
}

TEST_F(ServoDiagnosticBroadcasterTest, ConnectionWaringAndWarningStatus) {
  SetUp();
  hardware_->valid_joint_1->connection_error_rate->set_value(0.01);
  hardware_->valid_joint_1->warning_status->set_value(2.0);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Connection error occurred"));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Velocity command interrupted."));
}

TEST_F(ServoDiagnosticBroadcasterTest, ConnectionWaringAndErrorStatus) {
  SetUp();
  hardware_->valid_joint_1->connection_error_rate->set_value(0.01);
  hardware_->valid_joint_1->error_status->set_value(2.0);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Connection error occurred"));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Supplied voltage out of range."));
}

TEST_F(ServoDiagnosticBroadcasterTest, ConnectionWaringAndSafetyAlarmStatus) {
  SetUp();
  hardware_->valid_joint_1->connection_error_rate->set_value(0.01);
  hardware_->valid_joint_1->safety_alarm_status->set_value(2.0);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Connection error occurred"));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Short error."));
}

TEST_F(ServoDiagnosticBroadcasterTest, ConnectionErrorAndWarningStatus) {
  SetUp();
  hardware_->valid_joint_1->connection_error_rate->set_value(0.31);
  hardware_->valid_joint_1->warning_status->set_value(2.0);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Connection error rate high"));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Velocity command interrupted."));
}

TEST_F(ServoDiagnosticBroadcasterTest, ConnectionErrorAndErrorStatus) {
  SetUp();
  hardware_->valid_joint_1->connection_error_rate->set_value(0.31);
  hardware_->valid_joint_1->error_status->set_value(2.0);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Connection error rate high"));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Supplied voltage out of range."));
}

TEST_F(ServoDiagnosticBroadcasterTest, ConnectionErrorAndSafetyAlarmStatus) {
  SetUp();
  hardware_->valid_joint_1->connection_error_rate->set_value(0.31);
  hardware_->valid_joint_1->safety_alarm_status->set_value(2.0);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Connection error rate high"));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Short error."));
}

TEST_F(ServoDiagnosticBroadcasterTest, WarningStatusAndErrorStatus) {
  SetUp();
  hardware_->valid_joint_1->warning_status->set_value(2.0);
  hardware_->valid_joint_1->error_status->set_value(2.0);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Velocity command interrupted."));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Supplied voltage out of range."));
}

TEST_F(ServoDiagnosticBroadcasterTest, WarningStatusAndSafetyAlarmStatus) {
  SetUp();
  hardware_->valid_joint_1->warning_status->set_value(2.0);
  hardware_->valid_joint_1->safety_alarm_status->set_value(2.0);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Velocity command interrupted."));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Short error."));
}

TEST_F(ServoDiagnosticBroadcasterTest, ErrorStatusAndSafetyAlarmStatus) {
  SetUp();
  hardware_->valid_joint_1->error_status->set_value(2.0);
  hardware_->valid_joint_1->safety_alarm_status->set_value(2.0);
  WaitForDiagnostics();

  const auto msg = subscriber_->last_msg();

  ASSERT_TRUE(HasJoint(msg, "valid_joint_1"));
  EXPECT_EQ(ExtractLevel(msg, "valid_joint_1"), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Supplied voltage out of range."));
  EXPECT_TRUE(IsMessageIncluded(msg, "valid_joint_1", "Short error."));
}

class ServoDiagnosticBroadcasterJointTest : public ServoDiagnosticBroadcasterTestBase {
 protected:
  void SetUp(const double publish_rate = 10.0);
};

void ServoDiagnosticBroadcasterJointTest::SetUp(const double publish_rate) {
  ServoDiagnosticBroadcasterTestBase::SetUp();
  /* Setting of joints */
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides() = {
      rclcpp::Parameter("joints", std::vector<std::string>({"valid_joint_1"}))};
  controller_ = std::make_shared<ServoDiagnosticBroadcaster>();
  EXPECT_EQ(controller_->init(kControllerNodeName, "", 0, "", node_options), controller_interface::return_type::OK);

  controller_->get_node()->declare_parameter("publish_rate", publish_rate);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  hardware_ = std::make_shared<HardwareStub>();
  controller_->assign_interfaces({}, std::move(hardware_->state_interfaces()));

  EXPECT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  client_node_ = rclcpp::Node::make_shared(kClientNodeName);
  subscriber_ = std::make_shared<DiagnosticsSubscriber>(client_node_);
}

TEST_F(ServoDiagnosticBroadcasterJointTest, JointSet) {
  SetUp();
  auto config = controller_->state_interface_configuration();
  EXPECT_EQ(controller_interface::interface_configuration_type::INDIVIDUAL, config.type);
  EXPECT_EQ(config.names.size(), 11);
  EXPECT_NE(std::find(config.names.begin(), config.names.end(),
            "valid_joint_1/connection_error_rate"), config.names.end());
  EXPECT_NE(std::find(config.names.begin(), config.names.end(), "valid_joint_1/position"), config.names.end());
  EXPECT_NE(std::find(config.names.begin(), config.names.end(), "valid_joint_1/velocity"), config.names.end());
  EXPECT_NE(std::find(config.names.begin(), config.names.end(), "valid_joint_1/effort"), config.names.end());
  EXPECT_NE(std::find(config.names.begin(), config.names.end(), "valid_joint_1/temperature"), config.names.end());
  EXPECT_NE(std::find(config.names.begin(), config.names.end(), "valid_joint_1/current"), config.names.end());
  EXPECT_NE(std::find(config.names.begin(), config.names.end(),
            "valid_joint_1/current_drive_mode"), config.names.end());
  EXPECT_NE(std::find(config.names.begin(), config.names.end(), "valid_joint_1/motor_id"), config.names.end());
  EXPECT_NE(std::find(config.names.begin(), config.names.end(), "valid_joint_1/warning_status"), config.names.end());
  EXPECT_NE(std::find(config.names.begin(), config.names.end(), "valid_joint_1/error_status"), config.names.end());
  EXPECT_NE(std::find(config.names.begin(), config.names.end(),
            "valid_joint_1/safety_alarm_status"), config.names.end());
}

}  // namespace hsrb_diagnostic_broadcaster

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
