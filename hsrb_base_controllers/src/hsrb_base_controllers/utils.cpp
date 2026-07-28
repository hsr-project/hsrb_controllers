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

#include "utils.hpp"

#include <functional>

namespace hsrb_base_controllers {

// Retrieve parameter using default value if negative
double GetPositiveParameter(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& parameter_name, double default_value) {
  auto value = GetParameter(node, parameter_name, default_value);
  if (value > 0.0) {
    return value;
  } else {
    RCLCPP_WARN_STREAM(node->get_logger(),
                       parameter_name << " must be positive. Use default value " << default_value);
    return default_value;
  }
}

// Find the minimum ratio using ternary search
template <typename Compare = std::less<double>>
double TernarySearchMin(const CostFunction& cost_function, double epsilon, Compare comp = Compare{}) {
  double low = 0.0;
  double high = 1.0;
  while (high - low > epsilon) {
    const double mid1 = (2.0 * low + high) / 3.0;
    const double mid2 = (low + 2.0 * high) / 3.0;
    const double cost1 = cost_function(mid1);
    const double cost2 = cost_function(mid2);

    if (comp(cost1, cost2)) {
      high = mid2;
    } else {
      low = mid1;
    }
  }
  return (low + high) / 2.0;
}

double TernarySearchMinRight(const CostFunction& cost_function, double epsilon) {
  return TernarySearchMin(cost_function, epsilon, std::less<double>{});
}

double TernarySearchMinLeft(const CostFunction& cost_function, double epsilon) {
  return TernarySearchMin(cost_function, epsilon, std::less_equal<double>{});
}

}  // namespace hsrb_base_controllers
