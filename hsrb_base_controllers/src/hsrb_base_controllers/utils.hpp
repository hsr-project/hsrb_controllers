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
/// @brief Utility function
#ifndef HSRB_BASE_CONTROLLERS_UTILS_HPP_
#define HSRB_BASE_CONTROLLERS_UTILS_HPP_

#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace hsrb_base_controllers {

// Retrieve parameter with default value
template <typename ParameterType>
auto GetParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                  const std::string& name,
                  const ParameterType& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParameterType>(name, default_value);
  } else {
    return node->get_parameter(name).get_value<ParameterType>();
  }
}

// Retrieve parameter using default value if non-positive
double GetPositiveParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& parameter_name,
                            double default_value);

// Search for the minimum ratio using ternary search
// Used only in omni_base_joint_controller.cpp, but placed in utils for easier testing
using CostFunction = std::function<double(double)>;
double TernarySearchMinRight(const CostFunction& cost_function, double epsilon);
double TernarySearchMinLeft(const CostFunction& cost_function, double epsilon);

}  // namespace hsrb_base_controllers

#endif  // HSRB_BASE_CONTROLLERS_UTILS_HPP_
