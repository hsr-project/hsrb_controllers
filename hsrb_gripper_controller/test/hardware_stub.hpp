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
/// @brief Handle that simulates hardware for testing
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace hsrb_gripper_controller {

class Handle {
 public:
  using Ptr = std::shared_ptr<Handle>;

  Handle(const std::string& joint_name, const std::string& state_name, const std::string& command_name)
      : current_(0.0),
        command_(0.0) {
    state_handle_ = std::make_shared<hardware_interface::StateInterface>(joint_name, state_name, &current_);
    command_handle_ = std::make_shared<hardware_interface::CommandInterface>(joint_name, command_name, &command_);
  }

  Handle(const std::string& joint_name, const std::string& interface_name)
      : Handle(joint_name, interface_name, interface_name) {}

  virtual ~Handle() = default;

  hardware_interface::LoanedStateInterface GetStateInterface() {
    return hardware_interface::LoanedStateInterface(state_handle_);
  }
  hardware_interface::LoanedCommandInterface GetCommandInterface() {
    return hardware_interface::LoanedCommandInterface(command_handle_, nullptr);
  }

  double command() const { return command_; }
  void set_current(double x) { current_ = x; }

 private:
  double current_;
  double command_;

  hardware_interface::StateInterface::SharedPtr state_handle_;
  hardware_interface::CommandInterface::SharedPtr command_handle_;
};

class BoolHandle : public Handle {
 public:
  using Ptr = std::shared_ptr<BoolHandle>;

  BoolHandle(const std::string& joint_name, const std::string& state_name, const std::string& command_name)
      : Handle(joint_name, state_name, command_name) {}

  bool bool_command() const { return command() > 0.0; }
  void set_current(bool x) {
    if (x) {
      Handle::set_current(1.0);
    } else {
      Handle::set_current(-1.0);
    }
  }
};

struct HardwareStub {
  using Ptr = std::shared_ptr<HardwareStub>;

  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;

  Handle::Ptr position;
  Handle::Ptr velocity;
  Handle::Ptr effort;
  Handle::Ptr current;

  Handle::Ptr drive_mode;
  BoolHandle::Ptr grasping_flag;

  Handle::Ptr spring_l_position;
  Handle::Ptr spring_r_position;

  explicit HardwareStub(const std::string& joint_name) {
    position = std::make_shared<Handle>(joint_name, hardware_interface::HW_IF_POSITION);
    velocity = std::make_shared<Handle>(joint_name, hardware_interface::HW_IF_VELOCITY);
    effort = std::make_shared<Handle>(joint_name, hardware_interface::HW_IF_EFFORT);
    current = std::make_shared<Handle>(joint_name, "current");

    drive_mode = std::make_shared<Handle>(joint_name, "current_drive_mode", "command_drive_mode");
    grasping_flag = std::make_shared<BoolHandle>(joint_name, "current_grasping_flag", "command_grasping_flag");

    // TODO(Takeshita) パラメータにする
    spring_l_position = std::make_shared<Handle>("hand_l_spring_proximal_joint", hardware_interface::HW_IF_POSITION);
    spring_r_position = std::make_shared<Handle>("hand_r_spring_proximal_joint", hardware_interface::HW_IF_POSITION);

    command_interfaces.emplace_back(position->GetCommandInterface());
    command_interfaces.emplace_back(effort->GetCommandInterface());
    command_interfaces.emplace_back(drive_mode->GetCommandInterface());
    command_interfaces.emplace_back(grasping_flag->GetCommandInterface());

    state_interfaces.emplace_back(position->GetStateInterface());
    state_interfaces.emplace_back(velocity->GetStateInterface());
    state_interfaces.emplace_back(effort->GetStateInterface());
    state_interfaces.emplace_back(current->GetStateInterface());
    state_interfaces.emplace_back(drive_mode->GetStateInterface());
    state_interfaces.emplace_back(grasping_flag->GetStateInterface());
    state_interfaces.emplace_back(spring_l_position->GetStateInterface());
    state_interfaces.emplace_back(spring_r_position->GetStateInterface());
  }
};

}  // namespace hsrb_gripper_controller
