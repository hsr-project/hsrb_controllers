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
/// @file filter-test.cpp
/// @brief Test of the filter class

#include <vector>
#include <gtest/gtest.h>
#include <hsrb_base_controllers/filter.hpp>

namespace {
const double kEpsilon = 1.0e-5;
}  // anonymous namespace

namespace hsrb_base_controllers {

TEST(FilterTest, Default) {
  // By default, initialized with a=[1.0], b=[1.0]
  Filter<> filter;
  // Nothing is filtered
  EXPECT_EQ(1.0, filter.update(1.0));
  EXPECT_EQ(2.0, filter.update(2.0));
  EXPECT_EQ(3.0, filter.update(3.0));
}

TEST(FilterTest, Normal) {
  // By default, initialized with {1.0, 0.1, 0.9}, b={0.2, 0.8}
  double aa[] = {1.0, 0.1, 0.9};
  double bb[] = {0.2, 0.8};
  std::vector<double> a(aa, aa+3);
  std::vector<double> b(bb, bb+2);
  Filter<> filter(a, b);
  // Internal state is initialized to 0
  // y = 1.0*0.2 + 0.0*0.8 - 0.0*0.1 - 0.0*0.9
  EXPECT_NEAR(0.2, filter.update(1.0), kEpsilon);
  // y = 2.0*0.2 + 1.0*0.8 - 0.2*0.1 - 0.0*0.9
  EXPECT_NEAR(1.18, filter.update(2.0), kEpsilon);
  // y = 3.0*0.2 + 2.0*0.8 - 1.18*0.1 - 0.2*0.9
  EXPECT_NEAR(1.902, filter.update(3.0), kEpsilon);
  // y = 4.0*0.2 + 3.0*0.8 - 1.902*0.1 - 1.18*0.9
  EXPECT_NEAR(1.9478, filter.update(4.0), kEpsilon);
}

TEST(FilterTest, Reset) {
  // By default, initialized with {1.0, 0.1, 0.9}, b={0.2, 0.8}
  double aa[] = {1.0, 0.1, 0.9};
  double bb[] = {0.2, 0.8};
  std::vector<double> a(aa, aa+3);
  std::vector<double> b(bb, bb+2);
  Filter<> filter(a, b);
  // Initialize the internal state to 1.0 (a state where input and output are balanced at 1.0)
  filter.reset(1.0);
  // y = 1.0*0.2 + 1.0*0.8 - 1.0*0.1 - 1.0*0.9
  EXPECT_NEAR(0.0, filter.update(1.0), kEpsilon);
  // y = 2.0*0.2 + 1.0*0.8 - 0.0*0.1 - 1.0*0.9
  EXPECT_NEAR(0.3, filter.update(2.0), kEpsilon);
  // y = 3.0*0.2 + 2.0*0.8 - 0.3*0.1 - 0.0*0.9
  EXPECT_NEAR(2.17, filter.update(3.0), kEpsilon);
  // y = 4.0*0.2 + 3.0*0.8 - 2.17*0.1 - 0.3*0.9
  EXPECT_NEAR(2.713, filter.update(4.0), kEpsilon);
}

}  // namespace hsrb_base_controllers

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
