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

namespace hsrb_base_controllers {

class Handle {
 public:
  explicit Handle(const std::string& name)
     : command_(0.0), current_pos_(0.0), current_vel_(0.0) {
    state_position_handle_ = std::make_shared<hardware_interface::StateInterface>(
        name, hardware_interface::HW_IF_POSITION, &current_pos_);
    state_velocity_handle_ = std::make_shared<hardware_interface::StateInterface>(
        name, hardware_interface::HW_IF_VELOCITY, &current_vel_);
  }
  virtual ~Handle() = default;

  hardware_interface::LoanedStateInterface GetPositionStateInterface() {
    return hardware_interface::LoanedStateInterface(state_position_handle_);
  }
  hardware_interface::LoanedStateInterface GetVelocityStateInterface() {
    return hardware_interface::LoanedStateInterface(state_velocity_handle_);
  }

  virtual void Update() = 0;

  double command() const { return command_; }
  void set_current_pos(double x) { current_pos_ = x; }
  void set_current_vel(double x) { current_vel_ = x; }

 protected:
  double command_;
  double current_pos_;
  double current_vel_;

 private:
  hardware_interface::StateInterface::SharedPtr state_position_handle_;
  hardware_interface::StateInterface::SharedPtr state_velocity_handle_;
};

class CommandPositionHandle : public Handle {
 public:
  using Ptr = std::shared_ptr<CommandPositionHandle>;

  explicit CommandPositionHandle(const std::string& name, double update_frequency)
      : Handle(name), update_frequency_(update_frequency) {
    command_position_handle_ = std::make_shared<hardware_interface::CommandInterface>(
        name, hardware_interface::HW_IF_POSITION, &command_);
  }
  virtual ~CommandPositionHandle() = default;

  hardware_interface::LoanedCommandInterface GetCommandInterface() {
    return hardware_interface::LoanedCommandInterface(command_position_handle_, nullptr);
  }

  void Update() override {
    current_vel_ = (command_ - current_pos_) * update_frequency_;
    current_pos_ = command_;
  }

 private:
  hardware_interface::CommandInterface::SharedPtr command_position_handle_;
  double update_frequency_;
};

class CommandVelocityHandle : public Handle {
 public:
  using Ptr = std::shared_ptr<CommandVelocityHandle>;

  explicit CommandVelocityHandle(const std::string& name, double update_frequency)
      : Handle(name), update_frequency_(update_frequency) {
    command_velocity_handle_ = std::make_shared<hardware_interface::CommandInterface>(
        name, hardware_interface::HW_IF_VELOCITY, &command_);
  }

  virtual ~CommandVelocityHandle() = default;

  hardware_interface::LoanedCommandInterface GetCommandInterface() {
    return hardware_interface::LoanedCommandInterface(command_velocity_handle_, nullptr);
  }

  void Update() override {
    current_pos_ += command_ / update_frequency_;
    current_vel_ = command_;
  }

 private:
  hardware_interface::CommandInterface::SharedPtr command_velocity_handle_;
  double update_frequency_;
};

template<typename SteerCommandHandleType>
struct HardwareStub {
  using Ptr = std::shared_ptr<HardwareStub>;

  typename SteerCommandHandleType::Ptr steer_handle;
  CommandVelocityHandle::Ptr l_wheel_handle;
  CommandVelocityHandle::Ptr r_wheel_handle;

  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;

  explicit HardwareStub(double update_frequency) {
    steer_handle = std::make_shared<SteerCommandHandleType>("base_roll_joint", update_frequency);
    l_wheel_handle = std::make_shared<CommandVelocityHandle>("base_l_drive_wheel_joint", update_frequency);
    r_wheel_handle = std::make_shared<CommandVelocityHandle>("base_r_drive_wheel_joint", update_frequency);

    command_interfaces.emplace_back(steer_handle->GetCommandInterface());
    command_interfaces.emplace_back(l_wheel_handle->GetCommandInterface());
    command_interfaces.emplace_back(r_wheel_handle->GetCommandInterface());

    state_interfaces.emplace_back(steer_handle->GetPositionStateInterface());
    state_interfaces.emplace_back(steer_handle->GetVelocityStateInterface());
    state_interfaces.emplace_back(l_wheel_handle->GetPositionStateInterface());
    state_interfaces.emplace_back(l_wheel_handle->GetVelocityStateInterface());
    state_interfaces.emplace_back(r_wheel_handle->GetPositionStateInterface());
    state_interfaces.emplace_back(r_wheel_handle->GetVelocityStateInterface());
  }
  HardwareStub() : HardwareStub(100.0) {}

  void Update() {
    steer_handle->Update();
    l_wheel_handle->Update();
    r_wheel_handle->Update();
  }
};

}  // namespace hsrb_base_controllers
