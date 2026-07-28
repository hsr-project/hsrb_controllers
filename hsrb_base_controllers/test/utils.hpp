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
/// @brief A convenient function for testing

#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

namespace hsrb_base_controllers {

template<typename TYPE>
class SubscriptionCounter {
 public:
  using Ptr = std::shared_ptr<SubscriptionCounter>;

  SubscriptionCounter(const rclcpp::Node::SharedPtr& node, const std::string& topic_name) : count_(0) {
    subscriber_ = node->template create_subscription<TYPE>(
        topic_name, 1, std::bind(&SubscriptionCounter<TYPE>::Callback, this, std::placeholders::_1));
  }

  uint32_t count() const { return count_; }
  TYPE last_msg() const { return last_msg_; }

 private:
  void Callback(const typename TYPE::SharedPtr msg) {
    ++count_;
    last_msg_ = *msg;
  }
  typename rclcpp::Subscription<TYPE>::SharedPtr subscriber_;
  uint32_t count_;
  TYPE last_msg_;
};

template<typename TYPE>
class TopicRelay {
 public:
  using Ptr = std::shared_ptr<TopicRelay>;

  TopicRelay(const rclcpp::Node::SharedPtr& node, const std::string& in_name, const std::string& out_name) {
    sub_ = node->template create_subscription<TYPE>(
        in_name, 1, std::bind(&TopicRelay<TYPE>::Callback, this, std::placeholders::_1));
    pub_ = node->template create_publisher<TYPE>(out_name, rclcpp::SystemDefaultsQoS());
  }

 private:
  void Callback(const typename TYPE::SharedPtr msg) { pub_->publish(*msg); }

  typename rclcpp::Subscription<TYPE>::SharedPtr sub_;
  typename rclcpp::Publisher<TYPE>::SharedPtr pub_;
};

class TimeoutDetection {
 public:
  explicit TimeoutDetection(const rclcpp::Clock::SharedPtr& clock, double timeout_sec = 1.0) {
    clock_ = clock;
    timeout_stamp_ = clock_->now() + rclcpp::Duration::from_seconds(timeout_sec);
  }

  void Run() {
    if (clock_->now() > timeout_stamp_) {
      FAIL();
    }
  }

 private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time timeout_stamp_;
};

std::string GetRobotDescription() {
  std::fstream xml_file("robot.xml", std::fstream::in);
  std::string robot_description;
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    robot_description += (line + "\n");
  }
  xml_file.close();
  return robot_description;
}

}  // namespace hsrb_base_controllers
