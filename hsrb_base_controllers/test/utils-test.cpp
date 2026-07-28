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

#include <gtest/gtest.h>

#include "../src/hsrb_base_controllers/utils.hpp"

namespace {
const double kEpsilon = 1.0e-6;
}  // namespace

namespace hsrb_base_controllers {

struct TernarySearchTestCase {
  double value_zero_range_min;
  double value_zero_range_max;

  CostFunction GetCostFunction() const {
    return [this](double x) {
      if (x < value_zero_range_min) {
        return std::abs(x - value_zero_range_min);
      } else if (x > value_zero_range_max) {
        return std::abs(x - value_zero_range_max);
      } else {
        return 0.0;
      }
    };
  }
};

TEST(UtilsTest, TernarySearchMinRight) {
  // When the range where the minimum value is 0 is narrow
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = 0.3;
    test_case.value_zero_range_max = 0.3001;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinRight(cost_function, kEpsilon);
    EXPECT_NEAR(result, 0.3001, kEpsilon);
    EXPECT_DOUBLE_EQ(cost_function(result), 0.0);
  }
  // When the range where the minimum value is 0 is wide
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = 0.3;
    test_case.value_zero_range_max = 0.7;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinRight(cost_function, kEpsilon);
    EXPECT_NEAR(result, 0.7, kEpsilon);
    EXPECT_DOUBLE_EQ(cost_function(result), 0.0);
  }
  // When the range where the minimum value is 0 is at the lower end
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = 0.0;
    test_case.value_zero_range_max = 0.2;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinRight(cost_function, kEpsilon);
    EXPECT_NEAR(result, 0.2, kEpsilon);
    EXPECT_DOUBLE_EQ(cost_function(result), 0.0);
  }
  // When the range where the minimum value is 0 is at the upper end
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = 0.8;
    test_case.value_zero_range_max = 1.0;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinRight(cost_function, kEpsilon);
    EXPECT_NEAR(result, 1.0, kEpsilon);
    EXPECT_DOUBLE_EQ(cost_function(result), 0.0);
  }
  // When the range where the minimum value is 0 is outside the upper end
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = 1.1;
    test_case.value_zero_range_max = 1.2;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinRight(cost_function, kEpsilon);
    EXPECT_NEAR(result, 1.0, kEpsilon);
    EXPECT_NEAR(cost_function(result), 0.1, kEpsilon);
  }
  // When the range where the minimum value is 0 is outside the lower end
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = -0.2;
    test_case.value_zero_range_max = -0.1;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinRight(cost_function, kEpsilon);
    EXPECT_NEAR(result, 0.0, kEpsilon);
    EXPECT_NEAR(cost_function(result), 0.1, kEpsilon);
  }
}

TEST(UtilsTest, TernarySearchMinLeft) {
  // When the range where the minimum value is 0 is narrow
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = 0.3;
    test_case.value_zero_range_max = 0.3001;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinLeft(cost_function, kEpsilon);
    EXPECT_NEAR(result, 0.3, kEpsilon);
    EXPECT_DOUBLE_EQ(cost_function(result), 0.0);
  }
  // When the range where the minimum value is 0 is wide
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = 0.3;
    test_case.value_zero_range_max = 0.7;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinLeft(cost_function, kEpsilon);
    EXPECT_NEAR(result, 0.3, kEpsilon);
    EXPECT_DOUBLE_EQ(cost_function(result), 0.0);
  }
  // When the range where the minimum value is 0 is at the lower end
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = 0.0;
    test_case.value_zero_range_max = 0.2;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinLeft(cost_function, kEpsilon);
    EXPECT_NEAR(result, 0.0, kEpsilon);
    EXPECT_DOUBLE_EQ(cost_function(result), 0.0);
  }
  // When the range where the minimum value is 0 is at the upper end
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = 0.8;
    test_case.value_zero_range_max = 1.0;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinLeft(cost_function, kEpsilon);
    EXPECT_NEAR(result, 0.8, kEpsilon);
    EXPECT_DOUBLE_EQ(cost_function(result), 0.0);
  }
  // When the range where the minimum value is 0 is outside the upper end
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = 1.1;
    test_case.value_zero_range_max = 1.2;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinLeft(cost_function, kEpsilon);
    EXPECT_NEAR(result, 1.0, kEpsilon);
    EXPECT_NEAR(cost_function(result), 0.1, kEpsilon);
  }
  // When the range where the minimum value is 0 is outside the lower end
  {
    TernarySearchTestCase test_case;
    test_case.value_zero_range_min = -0.2;
    test_case.value_zero_range_max = -0.1;
    auto cost_function = test_case.GetCostFunction();
    auto result = TernarySearchMinLeft(cost_function, kEpsilon);
    EXPECT_NEAR(result, 0.0, kEpsilon);
    EXPECT_NEAR(cost_function(result), 0.1, kEpsilon);
  }
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
